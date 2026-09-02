`timescale 1ns / 1ps
// =============================================================================
// n64_rx.v — N64 JoyBus Receiver
// =============================================================================
// Reference: https://n64brew.dev/wiki/Joybus_Protocol
//
// PROTOCOL OPERATION (from spec):
//   All bits are transmitted MSB first. Communication is always initiated by
//   the console. The line idles HIGH. The device pulls LOW to start each bit.
//
//   Bit encoding at 22MHz (clk_core, ~45.45ns/tick) — from n64brew.dev spec:
//
//     Logic 1:      Logic 0:      Controller Stop: 
//      __             ______        ___            
//     |  |           |      |      |   |           
//   ──┘  └──────── ──┘      └──  ──┘   └──────────  
//      1us  3us       3us  1us     2us    2us        
//
//   Logic 1             : 1us low  ( 22 ticks) + 3us high ( 66 ticks) = 4us
//   Logic 0             : 3us low  ( 66 ticks) + 1us high ( 22 ticks) = 4us
//   Controller Stop Bit : 2us low  ( 44 ticks) + 2us high ( 44 ticks) = 4us
//
//   N64 Console Stop Bit: 1us low  ( 22 ticks) + 2us high ( 44 ticks) = 3us
//
//   GameCube Stop Bit   : 1us low  ( 22 ticks) + 3us high ( 66 ticks) = 4us
//     -- identical to a logic-1 bit, per jefflongo.dev's GC controller
//     reverse-engineering writeup. NOT the same shape as the N64 controller
//     stop bit above it (which is genuinely unique, 2us/2us). This module
//     (n64_rx) has nothing to do with generating the GC stop bit -- that's
//     joybus_tx.v, on the CPLD's transmit side toward the Wii -- but the
//     entry is kept in this table since it's the natural place to compare
//     all four stop/terminator shapes side by side.
//
//   Console Stop Bit is sent by the console (CPLD) after 0x01 command.
//   It is 1us low + 2us high (3us total). Handled by n64_tx, not here.
//
// TIMING at 22MHz (clk_core, ~45.45ns/tick):
//   1us  ≈  22 ticks   (logic-1 low, console stop low)
//   2us  ≈  44 ticks   (controller stop low, controller stop high)
//   3us  ≈  66 ticks   (logic-0 low, logic-1 high)
//   THRESHOLD   = 44 ticks — midpoint between 22 (1us) and 66 (3us)
//     Controller stop bit (44 ticks low) hits exactly at threshold.
//     Because packet_done blocks it, this boundary case never matters.
//   IDLE_THRESH = 220 ticks (~10us) — above max 4us bit duration,
//     safely below 16ms inter-poll gap. Resets framing between polls.
//
//   Update THRESHOLD after hardware characterization with logic analyzer.
//   Measure actual low-pulse widths on your specific controller and adjust
//   if they differ significantly from ideal (e.g. 980ns → still ~49 ticks,
//   threshold of 100 is fine; but recalculate if your controller is unusual).
//
// COMMAND CONTEXT (0x01 — Controller State):
//   The CPLD sends command 0x01 to the N64 controller on Line A.
//   The controller responds with exactly 32 data bits + 1 controller stop bit.
//   This module captures those 32 bits into `packet`.
//
// PACKET FORMAT (32 bits, MSB first — bit 31 arrives first on the wire):
//   [31]    A button          [30]    B button
//   [29]    Z button          [28]    Start
//   [27]    D-Up              [26]    D-Down
//   [25]    D-Left            [24]    D-Right
//   [23]    RST               [22]    Reserved (always 0)
//             RST=1 when L+R+Start held simultaneously. Controller internally
//             resets analog stick to (0,0) and clears Start bit. CPLD receives
//             this packet normally — no special handling needed. top.v does not
//             map bit [23] to any GC output, so RST is correctly ignored.
//   [21]    L button          [20]    R button
//   [19]    C-Up              [18]    C-Down
//   [17]    C-Left            [16]    C-Right
//   [15:8]  Analog X — signed 8-bit two's complement, range ~-80 to +80
//             (spec allows -128 to +127 but physical stops limit real range)
//   [7:0]   Analog Y — signed 8-bit two's complement, range ~-80 to +80
//
// SPECIAL CASE — RST bit:
//   If L + R + Start are held simultaneously, the controller sets RST=1,
//   clears Start to 0, and resets the analog stick to (0,0). The 32-bit
//   packet still arrives — this module captures it normally. RST is not
//   a separate output; downstream logic can detect it if needed.
//
// OPEN-DRAIN NOTE:
//   The JoyBus line is open-drain. This module is RX only — it never drives
//   data_in. The n64_tx module drives Line A when sending the poll command.
//   top.v must ensure n64_tx tristates its output while n64_rx is receiving.
// =============================================================================

module n64_rx (
    input  wire        clk,        // clk_core, ~22MHz — ~45.45ns per tick
    input  wire        data_in,    // N64 JoyBus line (1kΩ pull-up to 3.3V, idles HIGH)
    output reg         bit_out,    // Decoded bit value: 1 or 0
    output reg         bit_valid,  // Pulses HIGH for exactly one clock cycle when bit_out is ready
    output reg  [31:0] packet,     // Latched 32-bit controller state, stable until next pkt_ready
    output reg         pkt_ready   // Pulses HIGH for exactly one clock cycle after all 32 bits received
);

    // -------------------------------------------------------------------------
    // Parameters
    // Update THRESHOLD after characterizing your specific controller in PulseView.
    // -------------------------------------------------------------------------
    parameter THRESHOLD   = 8'd44;    // ticks — midpoint between 22(1us) and 66(3us) at 22MHz
    parameter IDLE_THRESH = 10'd220;  // ticks (~10us at 22MHz), fits 10 bits (max 1023)

    // -------------------------------------------------------------------------
    // Stage 1: Two-stage synchronizer
    // -------------------------------------------------------------------------
    // data_in is asynchronous to our clk_core (~22MHz) domain. Sampling it directly
    // risks a metastable latch that can take arbitrarily long to resolve.
    // Two flip-flop stages give the signal one full clock period to settle.
    // d1 is the stable output — all downstream logic uses d1, never data_in.
    // Both initialized to 1'b1: line idles HIGH via pull-up resistor.
    // -------------------------------------------------------------------------
    reg d0 = 1'b1;
    reg d1 = 1'b1;

    always @(posedge clk) begin
        d0 <= data_in;
        d1 <= d0;
    end

    // Edge detection on the stable d1/d0 pair (d0 is one cycle ahead of d1)
    wire falling = (d1 == 1'b1) && (d0 == 1'b0);  // HIGH→LOW: new bit starting
    wire rising  = (d1 == 1'b0) && (d0 == 1'b1);  // LOW→HIGH: bit over, classify now

    // -------------------------------------------------------------------------
    // Stage 2: Low-pulse-width counter
    // -------------------------------------------------------------------------
    // Counts clk_core (~22MHz) ticks while the line is held LOW.
    // Reset to 0 on each falling edge (start of new bit).
    // Increments each cycle while d1==0.
    // On rising edge: compare to THRESHOLD to classify 1 vs 0.
    //
    // 8 bits covers up to 255 ticks (~11.6us at 22MHz). The longest valid
    // pulse is 3us (66 ticks), well inside range. Unlike high_count below,
    // this counter has NO saturation guard -- it will wrap past 255 if the
    // line is held low longer than that. In practice this would require an
    // abnormally long low pulse that shouldn't occur in valid protocol
    // operation (max legitimate low phase is 3us / 66 ticks), so it isn't
    // guarded the way high_count is, but it's worth knowing this is a
    // silent wraparound, not a saturating clamp, if you're ever debugging
    // an actual malformed/glitchy line.
    // -------------------------------------------------------------------------
    reg [7:0] low_count = 8'd0;

    // -------------------------------------------------------------------------
    // Stage 3: High-time idle counter
    // -------------------------------------------------------------------------
    // Counts ticks the line stays continuously HIGH (between transmissions).
    // Reset on any falling edge (line activity resets the idle timer).
    // When this reaches IDLE_THRESH (~10us), the stop bit has long passed
    // and we are in the inter-poll gap — safe to re-arm packet_done and
    // re-synchronize bit_count for the next poll cycle.
    //
    // Why 10us threshold (220 ticks)?
    //   Max valid HIGH phase inside a transmission = 3us (66 ticks, logic-1 high)
    //   Stop bit high phase ≥ 1us (line returns high and stays there)
    //   Inter-poll gap at 60Hz ≈ 16ms >> 10us
    //   10us is unambiguously between polls, not inside any valid bit sequence.
    //
    // 10-bit counter (matches reg [9:0], max range 1023 ticks / ~46.5us at
    // 22MHz), but it never actually reaches that range: the explicit
    // `if (high_count < IDLE_THRESH)` guard below stops it at IDLE_THRESH
    // (220 ticks, ~10us) and holds it there, so it fires exactly once, not
    // every cycle after.
    // -------------------------------------------------------------------------
    reg [9:0] high_count = 10'd0;

    // -------------------------------------------------------------------------
    // Stage 4: Bit counter and packet framing
    // -------------------------------------------------------------------------
    reg [5:0] bit_count = 6'd0;

    // packet_done: armed after bit 31 is received.
    // While HIGH, rising edges are IGNORED — this is what blocks the controller
    // stop bit (3us low = same width as logic-0) from being counted as a 33rd
    // data bit and corrupting the packet register.
    // Cleared when the idle gap timer fires (high_count == IDLE_THRESH).
    reg packet_done = 1'b0;

    // -------------------------------------------------------------------------
    // Main clocked logic — single always block, one driver per register
    // -------------------------------------------------------------------------
    always @(posedge clk) begin

        // Default: strobes are LOW every cycle. Asserted for exactly one cycle below.
        bit_valid <= 1'b0;
        pkt_ready <= 1'b0;

        // --- Low-pulse counter ---
        if (falling) begin
            // New bit starting: reset pulse counter and idle timer
            low_count  <= 8'd0;
            high_count <= 10'd0;
        end else if (d1 == 1'b0) begin
            // Line still LOW: increment pulse counter
            low_count  <= low_count + 8'd1;
        end

        // --- Idle (high) timer ---
        // Count consecutive ticks the line is HIGH. Saturate at IDLE_THRESH.
        if (d1 == 1'b1 && !falling) begin
            if (high_count < IDLE_THRESH)
                high_count <= high_count + 10'd1;
        end

        // --- Inter-poll gap detected: re-arm receiver ---
        // Fires exactly once when high_count reaches IDLE_THRESH (saturation
        // prevents re-firing). Clears packet_done so the next poll is accepted,
        // and re-syncs bit_count in case of any drift from line noise.
        if (high_count == IDLE_THRESH) begin
            packet_done <= 1'b0;
            bit_count   <= 6'd0;
        end

        // --- Bit classification on rising edge ---
        // Gated by !packet_done:
        //   - During normal reception (bits 0–31): packet_done=0, bits are classified
        //   - After bit 31 (packet_done=1): rising edges ignored
        //   - This specifically blocks the controller stop bit (3us low, same as
        //     logic-0) from being classified as data and shifting into packet.
        if (rising && !packet_done) begin
            bit_out   <= (low_count < THRESHOLD) ? 1'b1 : 1'b0;
            bit_valid <= 1'b1;
            // NOTE: bit_valid is set via non-blocking assignment.
            // The packet shift block below reads bit_valid from the END of the
            // PREVIOUS clock cycle (NBA semantics). So the shift happens one
            // cycle after classification. This is correct and intentional —
            // bit_out is also stable from the previous cycle when it shifts.
        end

        // --- Packet shift register ---
        // bit_valid here reads the value committed at end of the previous cycle.
        if (bit_valid) begin
            // Shift new bit into LSB. Since bit 31 (A button) arrives first on
            // the wire and enters here first, after 32 shifts it sits at [31].
            packet    <= {packet[30:0], bit_out};
            bit_count <= bit_count + 6'd1;

            if (bit_count == 6'd31) begin
                // 32nd bit just shifted in — packet is complete
                pkt_ready   <= 1'b1;   // pulse for exactly one cycle
                bit_count   <= 6'd0;   // reset framing counter
                packet_done <= 1'b1;   // arm stop-bit blocker immediately
            end
        end

    end

endmodule