`timescale 1ns / 1ps
// =============================================================================
// gc_rx.v — GameCube JoyBus Receiver
// =============================================================================
// Reference: https://n64brew.dev/wiki/Joybus_Protocol
//
// Listens on GC JoyBus Line B for commands from the Wii (acting as console).
// Decodes the 8-bit command byte and fires the appropriate one-cycle strobe
// so gc_tx can respond within the 62µs window.
//
// COMMANDS HANDLED:
//   0x00 → info_req   : Wii identifying what controller is connected
//   0x40 → poll_short : Short poll, expects 4-byte response
//   0x43 → poll_long  : Long poll,  expects 8-byte response
//   other → silently ignored, module resets cleanly for next command
//
// PROTOCOL — Console-side bit encoding (Wii is console on Line B):
//   Logic 1 : 1µs low  (~50 ticks)  + 3µs high (~150 ticks)
//   Logic 0 : 3µs low  (~150 ticks) + 1µs high (~50 ticks)
//   Console Stop Bit: 1µs low then line idles HIGH
//     Same width as logic-1. Blocked by packet_done after 8 data bits.
//
// TIMING:
//   THRESHOLD   = 100 ticks (2µs midpoint)
//   IDLE_THRESH = 500 ticks (10µs) — above max 4µs bit, below 16ms poll gap
//
// NBA PIPELINE NOTE (same pattern as n64_rx):
//   bit_valid is set via non-blocking assignment in the rising-edge clause.
//   The shift register and bit_count read bit_valid from the END of the
//   previous clock cycle. One cycle pipeline delay — correct and intentional.
//   bit_out/bit_valid committed cycle N, cmd_reg shifts cycle N+1.
//
// RESPONSE TIMING:
//   poll_short/poll_long/info_req fire on the cycle after the 8th bit shifts in.
//   top.v wires these directly to gc_tx send_* inputs — no registered path.
//   Response latency from Wii stop bit ≈ 3-4 clock cycles (~60-80ns).
//   Wii window: ~62µs (3100 ticks). Margin: >3000 ticks. No timing risk.
// =============================================================================

module gc_rx (
    input  wire       clk,        // 50MHz — 20ns per tick
    input  wire       data_in,    // GC JoyBus Line B (1kΩ pull-up, idles HIGH)
    output reg        poll_short, // one-cycle strobe: Wii sent 0x40
    output reg        poll_long,  // one-cycle strobe: Wii sent 0x43
    output reg        info_req,   // one-cycle strobe: Wii sent 0x00
    output reg  [7:0] cmd_out,    // raw decoded command byte (stable after cmd_ready)
    output reg        cmd_ready   // one-cycle strobe: cmd_out valid, strobes fired
);

    // -------------------------------------------------------------------------
    // Parameters — match n64_rx for consistency (same JoyBus timing)
    // -------------------------------------------------------------------------
    parameter THRESHOLD   = 8'd100;   // ticks — 1 vs 0 midpoint
    parameter IDLE_THRESH = 16'd500;  // ticks (~10µs) — inter-command gap

    // -------------------------------------------------------------------------
    // Stage 1: Two-stage synchronizer
    // Init HIGH — line idles HIGH via 1kΩ pull-up.
    // -------------------------------------------------------------------------
    reg d0 = 1'b1;
    reg d1 = 1'b1;

    always @(posedge clk) begin
        d0 <= data_in;
        d1 <= d0;
    end

    wire falling = (d1 == 1'b1) && (d0 == 1'b0);
    wire rising  = (d1 == 1'b0) && (d0 == 1'b1);

    // -------------------------------------------------------------------------
    // Stage 2: Low-pulse-width counter
    // Reset on falling edge, count while low, compare on rising edge.
    // -------------------------------------------------------------------------
    reg [7:0] low_count = 8'd0;

    // -------------------------------------------------------------------------
    // Stage 3: High-time idle counter
    // Counts continuous HIGH ticks. Resets on falling edge.
    // Saturates at IDLE_THRESH — fires exactly once per idle gap.
    // -------------------------------------------------------------------------
    reg [15:0] high_count = 16'd0;

    // -------------------------------------------------------------------------
    // Stage 4: Bit-level pipeline registers (same NBA pattern as n64_rx)
    // bit_valid/bit_val are set on rising edge via NBA.
    // cmd_reg and bit_count READ them the following cycle.
    // -------------------------------------------------------------------------
    reg       bit_valid = 1'b0;   // one-cycle strobe: new bit decoded
    reg       bit_val   = 1'b0;   // decoded value of the most recent bit

    // -------------------------------------------------------------------------
    // Stage 5: Command shift register and framing
    // -------------------------------------------------------------------------
    reg [7:0] cmd_reg     = 8'd0;
    reg [2:0] bit_count   = 3'd0;  // counts 0-7 (8 bits per command)
    reg       packet_done = 1'b0;  // blocks console stop bit after 8 data bits

    // -------------------------------------------------------------------------
    // Main clocked logic — single always block, one driver per register
    // -------------------------------------------------------------------------
    always @(posedge clk) begin

        // Default: all strobes LOW
        bit_valid  <= 1'b0;
        poll_short <= 1'b0;
        poll_long  <= 1'b0;
        info_req   <= 1'b0;
        cmd_ready  <= 1'b0;

        // --- Low counter ---
        if (falling) begin
            low_count  <= 8'd0;
            high_count <= 16'd0;
        end else if (d1 == 1'b0) begin
            low_count  <= low_count + 8'd1;
        end

        // --- High (idle) counter ---
        if (d1 == 1'b1 && !falling) begin
            if (high_count < IDLE_THRESH)
                high_count <= high_count + 16'd1;
        end

        // --- Inter-command gap: re-arm for next command ---
        // Fires once when high_count saturates — resets framing state.
        if (high_count == IDLE_THRESH) begin
            packet_done <= 1'b0;
            bit_count   <= 3'd0;
        end

        // --- Classify bit on rising edge ---
        // Gated by !packet_done: console stop bit (1µs low = logic-1 width)
        // is silently blocked after all 8 data bits have been received.
        if (rising && !packet_done) begin
            bit_val   <= (low_count < THRESHOLD) ? 1'b1 : 1'b0;
            bit_valid <= 1'b1;
            // NBA: bit_val and bit_valid take effect at END of this cycle.
            // The cmd_reg shift below reads them from the previous cycle.
        end

        // --- Command shift register ---
        // bit_valid here is the value from end of the PREVIOUS cycle (NBA).
        // Shifts new bit into LSB — MSB arrives first, so after 8 shifts
        // cmd_reg[7] holds the first bit received (MSB of command).
        if (bit_valid) begin
            cmd_reg   <= {cmd_reg[6:0], bit_val};
            bit_count <= bit_count + 3'd1;

            if (bit_count == 3'd7) begin
                // 8th bit just shifted in — full command received
                // cmd_reg now holds the complete command byte
                packet_done <= 1'b1;   // block console stop bit
                bit_count   <= 3'd0;

                // Latch and decode the completed command
                cmd_out   <= {cmd_reg[6:0], bit_val};  // include current bit
                cmd_ready <= 1'b1;

                // Fire the appropriate strobe
                // Use {cmd_reg[6:0], bit_val} — same as final cmd_out value
                case ({cmd_reg[6:0], bit_val})
                    8'h00: info_req   <= 1'b1;  // Wii info request
                    8'h40: poll_short <= 1'b1;  // short poll (4-byte response)
                    8'h43: poll_long  <= 1'b1;  // long poll  (8-byte response)
                    // All other commands: cmd_ready fires but no action strobe.
                    // gc_tx will not respond. The Wii will retry or move on.
                    default: ;
                endcase
            end
        end

    end

endmodule
