`timescale 1ns / 1ps
// =============================================================================
// gc_rx.v — GameCube JoyBus Receiver
// =============================================================================
// References:
//   https://n64brew.dev/wiki/Joybus_Protocol
//   https://hitmen.c02.at/files/yagcd/yagcd/chap9.html (YAGCD §9)
//   https://github.com/joypad-ai/joypad-os/blob/main/docs/protocols/GAMECUBE_JOYBUS.md
//
// COMMANDS HANDLED:
//   0x00 → info_req    : Wii probing controller type
//   0xFF → info_req    : Reset — same response as 0x00 (3-byte probe reply)
//   0x41 → origin_req  : Wii requesting origin/calibration data
//   0x40 → poll_short  : Standard poll — expects 4-byte response
//   0x43 → poll_long   : Extended poll — expects 8-byte response
//   other → silently ignored, module re-arms for next command
//
//   This means:
//     0x00, 0xFF, 0x41 are single-byte commands: 8 data bits + 1 stop bit
//       (9 bits total -- indistinguishable from the old per-byte model for
//       N=1, which is exactly why this bug never showed up on identify).
//     0x40, 0x43 are 3-byte commands: 24 data bits (command + report mode
//       + rumble, concatenated with nothing between them) + 1 stop bit
//       (25 bits total).
//
//   gc_rx doesn't know whether a command is 1 or 3 bytes until byte 0 (the
//   first 8 bits) is fully received, so total_bits is decided right then:
//   8 if cmd_reg isn't 0x40/0x43, else 24. Only the bit immediately after
//   total_bits data bits have been received is treated as the stop bit.
//   Bytes 1-2 of a multi-byte command are counted through but not stored
//   (rumble byte is unused -- no rumble motor -- and report mode is
//   always 0x03, not worth a register)
//
//
// Non-blocking assignments (NBA) PIPELINE (same pattern as n64_rx):
//   bit_valid/bit_val set on rising edge via NBA, read next cycle by shift reg.
//   One cycle pipeline delay — correct and intentional.
//
// RESPONSE TIMING:
//   Strobe now fires RESP_DELAY ticks (~3.5us) after the stop bit's rising
//   edge, not 1 cycle after it. A real 3rd-party GC controller was captured
//   waiting ~3.5us of idle-high line before responding to 0x00; the CPLD's
//   previous near-instant (~1-2 cycle) response is suspected of truncating
//   the Wii's own view of its stop bit / not giving its SI receiver time to
//   settle before seeing a new edge. No new counter added -- reuses the
//   existing high_count idle counter, which already starts counting from
//   the stop bit's rising edge for free.
//   top.v wires directly to gc_tx send_* inputs — no registered path beyond
//   this one.
//   For 3-byte commands: response begins ~600 ticks + RESP_DELAY after
//   command start. Wii window: ~62µs (3100 ticks). Comfortable margin
//   remains even with the added delay.
// =============================================================================

module gc_rx (
    input  wire       clk,         // 50MHz — 20ns per tick
    input  wire       data_in,     // GC JoyBus Line B (1kΩ pull-up, idles HIGH)
    output reg        poll_short,  // one-cycle strobe: Wii sent 0x40 (3 bytes)
    output reg        poll_long,   // one-cycle strobe: Wii sent 0x43 (3 bytes)
    output reg        info_req,    // one-cycle strobe: Wii sent 0x00 or 0xFF
    output reg        origin_req,  // one-cycle strobe: Wii sent 0x41
    output reg  [7:0] cmd_out,     // first command byte, stable after cmd_ready
    output reg        cmd_ready    // one-cycle strobe: cmd_out valid
);

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    parameter THRESHOLD   = 8'd44;    // ticks — midpoint between 22(1us) and 66(3us) at 22MHz
    parameter IDLE_THRESH = 10'd220;  // ticks (~10us at 22MHz), fits 10 bits (max 1023)
    parameter RESP_DELAY  = 10'd77;   // ticks (~3.5us at 22MHz) — matches real
                                       // controller's measured identify-response
                                       // turnaround. Must stay < IDLE_THRESH so
                                       // the deferred fire always lands before
                                       // the re-arm timeout.

    // -------------------------------------------------------------------------
    // Stage 1: Two-stage synchronizer — prevents metastability on async input
    // Both init HIGH — line idles HIGH via 1kOhm pull-up.
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
    // -------------------------------------------------------------------------
    reg [7:0] low_count = 8'd0;

    // -------------------------------------------------------------------------
    // Stage 3: High-time idle counter
    // Resets on any falling edge. Saturates at IDLE_THRESH.
    // When saturated: re-arm receiver (clear packet_done, reset counters).
    // -------------------------------------------------------------------------
    reg [9:0] high_count = 10'd0;

    // -------------------------------------------------------------------------
    // Stage 4: Bit-level pipeline registers (NBA pattern from n64_rx)
    // bit_valid/bit_val set on rising edge, read by shift reg next cycle.
    // -------------------------------------------------------------------------
    reg bit_valid = 1'b0;
    reg bit_val   = 1'b0;

    // -------------------------------------------------------------------------
    // Stage 5: Byte shift register, bit counter, framing
    //
    // bit_count counts DATA bits across the WHOLE command (0-23), not per
    // byte. total_bits is decided right after byte 0 completes (bit_count
    // hits 8): 8 for single-byte commands, 24 for 0x40/0x43. Once bit_count
    // reaches total_bits, the next bit received is the terminal stop bit.
    //
    // cmd_reg holds byte 0 only -- it stops being written to after its
    // first 8 bits land (bit_count > 7), so bytes 1-2 of a multi-byte
    // command are counted through but not stored (unused: report mode is
    // always 0x03, rumble is unused -- no rumble motor).
    //
    // packet_done: set when the stop bit is decoded (command fully
    //              received). Blocks further bit decoding until idle gap
    //              resets it.
    // -------------------------------------------------------------------------
    reg [7:0] cmd_reg     = 8'd0;
    reg [4:0] bit_count   = 5'd0;   // 5 bits: counts 0-24
    reg [4:0] total_bits  = 5'd8;   // 8 (single-byte) or 24 (0x40/0x43),
                                     // decided when bit_count hits 8 -- self-
                                     // correcting per command, no reset needed
    reg       packet_done = 1'b0;

    // -------------------------------------------------------------------------
    // Main clocked logic
    // -------------------------------------------------------------------------
    always @(posedge clk) begin

        // Default: all strobes LOW
        bit_valid  <= 1'b0;
        poll_short <= 1'b0;
        poll_long  <= 1'b0;
        info_req   <= 1'b0;
        origin_req <= 1'b0;
        cmd_ready  <= 1'b0;

        // --- Low counter: reset on falling edge, count while low ---
        if (falling) begin
            low_count  <= 8'd0;
            high_count <= 10'd0;
        end else if (d1 == 1'b0) begin
            low_count  <= low_count + 8'd1;
        end

        // --- Idle counter: count while HIGH, saturate at IDLE_THRESH ---
        if (d1 == 1'b1 && !falling) begin
            if (high_count < IDLE_THRESH)
                high_count <= high_count + 10'd1;
        end

        // --- Deferred response fire ---
        // packet_done was set when the stop bit was decoded, but we hold off
        // decoding/firing until the line has sat idle-high for RESP_DELAY
        // ticks, matching real controller turnaround. high_count is already
        // counting from the stop bit's rising edge (it resets on the falling
        // edge just before it), so no new counter is needed here — this
        // condition is true for exactly one cycle per packet since high_count
        // passes through RESP_DELAY once, monotonically, before it would ever
        // reach IDLE_THRESH.
        if (packet_done && (high_count == RESP_DELAY)) begin
            cmd_out   <= cmd_reg;
            cmd_ready <= 1'b1;

            case (cmd_reg)
                8'h00: info_req    <= 1'b1;
                8'hFF: info_req    <= 1'b1;  // Reset = same response as probe
                8'h41: origin_req  <= 1'b1;
                8'h40: poll_short  <= 1'b1;
                8'h43: poll_long   <= 1'b1;
                default: ;  // Unknown command: cmd_ready fires, no action
            endcase
        end

        // --- Inter-command gap: re-arm receiver ---
        if (high_count == IDLE_THRESH) begin
            packet_done <= 1'b0;
            bit_count   <= 5'd0;
        end

        // --- Classify bit on rising edge ---
        // Gated by !packet_done: no bits decoded after command is complete.
        if (rising && !packet_done) begin
            bit_val   <= (low_count < THRESHOLD) ? 1'b1 : 1'b0;
            bit_valid <= 1'b1;
        end

        // --- Shift register and command framing ---
        // bit_valid here reads the value from the END of the previous cycle (NBA).
        if (bit_valid) begin

            if (bit_count < total_bits) begin
                // --- Data bit (anywhere in the command, byte 0 through the
                // last byte) ---
                if (bit_count <= 5'd7) begin
                    // Byte 0 only -- shift into cmd_reg MSB-first.
                    cmd_reg <= {cmd_reg[6:0], bit_val};
                end
                // bit_count 8-23 (bytes 1-2 of a multi-byte command):
                // counted through, not stored -- content unused.

                bit_count <= bit_count + 5'd1;

                if (bit_count == 5'd7) begin
                    // Byte 0 just completed. Decide how many total data
                    // bits this command has -- {cmd_reg[6:0], bit_val} is
                    // the byte-0 value being shifted in THIS cycle (cmd_reg
                    // itself won't show it until next edge, NBA).
                    if ({cmd_reg[6:0], bit_val} == 8'h40 ||
                        {cmd_reg[6:0], bit_val} == 8'h43)
                        total_bits <= 5'd24;
                    else
                        total_bits <= 5'd8;
                end

            end else begin
                // --- Terminal stop bit: all data bits for this command
                // have been received. Per jefflongo.dev, this is always an
                // ordinary "1" bit for a well-formed command, but classify
                // it the same as any other bit rather than assuming.
                packet_done <= 1'b1;
                bit_count   <= 5'd0;
                total_bits  <= 5'd8;  // reset for the next command's byte 0
            end
        end

    end

endmodule
