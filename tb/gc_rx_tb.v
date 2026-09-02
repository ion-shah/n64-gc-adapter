`timescale 1ns / 1ps
// =============================================================================
// gc_rx_tb.v — Testbench for gc_rx (9-bit byte encoding)
// =============================================================================
// JoyBus 9-bit byte encoding:
//   Each byte = 8 data bits + 1 continue/stop bit (same 1µs/3µs encoding)
//   Continue bit = 0 (logic-0, 3µs low): more bytes follow
//   Continue bit = 1 (logic-1, 1µs low): last byte of command
//
// Commands and their byte sequences:
//   0x00: [0x00 | stop=1]                    → info_req
//   0xFF: [0xFF | stop=1]                    → info_req
//   0x41: [0x41 | stop=1]                    → origin_req
//   0x40: [0x40 | cont=0][0x03 | cont=0][0x00 | stop=1] → poll_short
//   0x43: [0x43 | cont=0][0x03 | cont=0][0x00 | stop=1] → poll_long
//
// Tests:
//   1.  0x00 single byte, stop=1   → info_req
//   2.  0xFF single byte, stop=1   → info_req (same as 0x00)
//   3.  0x41 single byte, stop=1   → origin_req
//   4.  0x40 three bytes            → poll_short fires after 3rd byte
//   5.  0x43 three bytes            → poll_long fires after 3rd byte
//   6.  Unknown 0x42, stop=1       → cmd_ready only, no action strobe
//   7.  Multi-byte unknown: 0x42 + 0x03 + stop=1 → cmd_ready, no action
//   8.  poll_short does NOT fire after byte 0 (continue=0 must be handled)
//   9.  Back-to-back commands after idle gap
//   10. Idle line — no false triggers in 50µs
//   11. cmd_out holds first command byte correctly for multi-byte commands
// =============================================================================

module gc_rx_tb;

    reg        clk     = 0;
    reg        data_in = 1;

    wire       poll_short;
    wire       poll_long;
    wire       info_req;
    wire       origin_req;
    wire [7:0] cmd_out;
    wire       cmd_ready;

    gc_rx dut (
        .clk       (clk),
        .data_in   (data_in),
        .poll_short(poll_short),
        .poll_long (poll_long),
        .info_req  (info_req),
        .origin_req(origin_req),
        .cmd_out   (cmd_out),
        .cmd_ready (cmd_ready)
    );

    always #10 clk = ~clk;

    initial begin
        $dumpfile("sim/gc_rx_tb.vcd");
        $dumpvars(0, gc_rx_tb);
    end

    integer pass_count = 0;
    integer fail_count = 0;

    // -------------------------------------------------------------------------
    // Latches — catch one-cycle pulses reliably
    // -------------------------------------------------------------------------
    reg seen_poll_short  = 0;
    reg seen_poll_long   = 0;
    reg seen_info_req    = 0;
    reg seen_origin_req  = 0;
    reg seen_cmd_ready   = 0;
    reg [7:0] latched_cmd = 8'h00;
    // Count how many times cmd_ready fired (detect double-fire in multi-byte)
    integer cmd_ready_count = 0;

    always @(posedge clk) begin
        if (poll_short)  seen_poll_short  <= 1;
        if (poll_long)   seen_poll_long   <= 1;
        if (info_req)    seen_info_req    <= 1;
        if (origin_req)  seen_origin_req  <= 1;
        if (cmd_ready) begin
            seen_cmd_ready    <= 1;
            latched_cmd       <= cmd_out;
            cmd_ready_count   <= cmd_ready_count + 1;
        end
    end

    task clear_latches;
        begin
            @(posedge clk); #1;
            seen_poll_short  = 0;
            seen_poll_long   = 0;
            seen_info_req    = 0;
            seen_origin_req  = 0;
            seen_cmd_ready   = 0;
            latched_cmd      = 8'h00;
            cmd_ready_count  = 0;
        end
    endtask

    // -------------------------------------------------------------------------
    // JoyBus 9-bit byte stimulus tasks
    // -------------------------------------------------------------------------
    // send_bit_1: 1µs low + 3µs high (logic 1)
    // send_bit_0: 3µs low + 1µs high (logic 0)
    // continue bit = 0 means "more bytes" → send as logic-0 (3µs low)
    // stop bit     = 1 means "last byte"  → send as logic-1 (1µs low)

    task send_bit_1;
        begin data_in = 0; #1000; data_in = 1; #3000; end
    endtask

    task send_bit_0;
        begin data_in = 0; #3000; data_in = 1; #1000; end
    endtask

    // Send 8 data bits MSB-first, then the continue/stop bit
    // stop: 1 = last byte of command, 0 = more bytes follow
    task send_byte;
        input [7:0] b;
        input       stop;   // 1 = stop bit (last byte), 0 = continue (more bytes)
        integer i;
        begin
            // 8 data bits MSB first
            for (i = 7; i >= 0; i = i - 1) begin
                if (b[i]) send_bit_1;
                else      send_bit_0;
            end
            // 9th bit: continue/stop
            if (stop) send_bit_1;  // stop=1 → logic-1 (1µs low)
            else      send_bit_0;  // cont=0 → logic-0 (3µs low)
        end
    endtask

    // Single-byte command (stop=1 after the byte)
    task send_cmd_single;
        input [7:0] cmd;
        begin
            send_byte(cmd, 1);
        end
    endtask

    // Three-byte poll command: cmd byte0 (cont=0), 0x03 (cont=0), 0x00 (stop=1)
    // rumble is always 0x00 — no rumble motor
    task send_cmd_poll;
        input [7:0] cmd;   // 0x40 or 0x43
        begin
            send_byte(cmd,  0);    // byte 0: command, continue
            send_byte(8'h03, 0);   // byte 1: report mode 3, continue
            send_byte(8'h00, 1);   // byte 2: rumble=0, stop
        end
    endtask

    // Wait for cmd_ready with timeout
    task wait_for_cmd;
        input integer timeout_ticks;
        begin : wfc
            integer t;
            for (t = 0; t < timeout_ticks; t = t + 1) begin
                @(posedge clk);
                if (seen_cmd_ready) disable wfc;
            end
        end
    endtask

    // Standard check helpers
    task assert_strobe;
        input       seen;
        input [8*32-1:0] name;
        begin
            if (!seen) begin
                $display("  FAIL [%0s]: strobe never fired", name);
                fail_count = fail_count + 1;
            end else begin
                $display("  PASS [%0s]", name);
                pass_count = pass_count + 1;
            end
        end
    endtask

    task assert_no_strobe;
        input       seen;
        input [8*32-1:0] name;
        begin
            if (seen) begin
                $display("  FAIL [%0s]: strobe fired unexpectedly", name);
                fail_count = fail_count + 1;
            end else begin
                $display("  PASS [%0s]: not fired (correct)", name);
                pass_count = pass_count + 1;
            end
        end
    endtask

    task assert_cmd;
        input [7:0] expected;
        input [8*32-1:0] name;
        begin
            if (!seen_cmd_ready) begin
                $display("  FAIL [%0s]: cmd_ready never fired", name);
                fail_count = fail_count + 1;
            end else if (latched_cmd !== expected) begin
                $display("  FAIL [%0s]: cmd_out=0x%02X expected 0x%02X",
                         name, latched_cmd, expected);
                fail_count = fail_count + 1;
            end else begin
                $display("  PASS [%0s]: cmd_out=0x%02X", name, latched_cmd);
                pass_count = pass_count + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Tests
    // -------------------------------------------------------------------------
    initial begin
        $display("============================================");
        $display("  gc_rx Testbench (9-bit byte encoding)");
        $display("  Ref: YAGCD §9, n64brew.dev, joypad-os docs");
        $display("============================================");

        #500;

        // -----------------------------------------------------------------
        // TEST 1: 0x00 single byte, stop=1 → info_req
        // -----------------------------------------------------------------
        $display("\n[TEST 1] 0x00 (probe) single byte → info_req");
        clear_latches;
        send_cmd_single(8'h00);
        wait_for_cmd(300);
        #15000;

        assert_cmd(8'h00,   "cmd_out=0x00");
        assert_strobe(seen_info_req,   "info_req fired");
        assert_no_strobe(seen_poll_short, "poll_short not fired");
        assert_no_strobe(seen_poll_long,  "poll_long not fired");
        assert_no_strobe(seen_origin_req, "origin_req not fired");

        // -----------------------------------------------------------------
        // TEST 2: 0xFF single byte, stop=1 → info_req (Reset command)
        // -----------------------------------------------------------------
        $display("\n[TEST 2] 0xFF (reset) single byte → info_req");
        clear_latches;
        send_cmd_single(8'hFF);
        wait_for_cmd(300);
        #15000;

        assert_cmd(8'hFF,   "cmd_out=0xFF");
        assert_strobe(seen_info_req,   "info_req fired (reset=probe response)");
        assert_no_strobe(seen_poll_short, "poll_short not fired");

        // -----------------------------------------------------------------
        // TEST 3: 0x41 single byte, stop=1 → origin_req
        // -----------------------------------------------------------------
        $display("\n[TEST 3] 0x41 (origin) single byte → origin_req");
        clear_latches;
        send_cmd_single(8'h41);
        wait_for_cmd(300);
        #15000;

        assert_cmd(8'h41,   "cmd_out=0x41");
        assert_strobe(seen_origin_req, "origin_req fired");
        assert_no_strobe(seen_info_req,   "info_req not fired");
        assert_no_strobe(seen_poll_short, "poll_short not fired");

        // -----------------------------------------------------------------
        // TEST 4: 0x40 three bytes → poll_short fires after THIRD byte
        // Sequence: [0x40|cont=0] [0x03|cont=0] [0x00|stop=1]
        // -----------------------------------------------------------------
        $display("\n[TEST 4] 0x40 three bytes → poll_short after 3rd byte");
        clear_latches;
        send_cmd_poll(8'h40);
        wait_for_cmd(1000);  // longer timeout: 3 bytes × ~9 bits × 200 ticks
        #15000;

        assert_cmd(8'h40,   "cmd_out=0x40 (first byte saved)");
        assert_strobe(seen_poll_short, "poll_short fired");
        assert_no_strobe(seen_poll_long,  "poll_long not fired");
        assert_no_strobe(seen_info_req,   "info_req not fired");

        // cmd_ready should fire exactly ONCE (not once per byte)
        if (cmd_ready_count !== 1) begin
            $display("  FAIL [cmd_ready once]: fired %0d times, expected 1",
                     cmd_ready_count);
            fail_count = fail_count + 1;
        end else begin
            $display("  PASS [cmd_ready once]: fired exactly 1 time");
            pass_count = pass_count + 1;
        end

        // -----------------------------------------------------------------
        // TEST 5: 0x43 three bytes → poll_long fires after THIRD byte
        // -----------------------------------------------------------------
        $display("\n[TEST 5] 0x43 three bytes → poll_long after 3rd byte");
        clear_latches;
        send_cmd_poll(8'h43);
        wait_for_cmd(1000);
        #15000;

        assert_cmd(8'h43,   "cmd_out=0x43 (first byte saved)");
        assert_strobe(seen_poll_long,  "poll_long fired");
        assert_no_strobe(seen_poll_short, "poll_short not fired");

        if (cmd_ready_count !== 1) begin
            $display("  FAIL [cmd_ready once]: fired %0d times", cmd_ready_count);
            fail_count = fail_count + 1;
        end else begin
            $display("  PASS [cmd_ready once]: fired exactly 1 time");
            pass_count = pass_count + 1;
        end

        // -----------------------------------------------------------------
        // TEST 6: poll_short does NOT fire after byte 0 alone
        // Verify: send only byte 0 of 0x40 (with continue=0), wait, check
        // no strobe fires yet. Then send bytes 1-2 to complete.
        // -----------------------------------------------------------------
        $display("\n[TEST 6] 0x40 byte 0 alone (cont=0) — no strobe yet");
        clear_latches;
        // Send only byte 0 with continue=0
        send_byte(8'h40, 0);
        // Wait a few cycles — strobe must NOT fire yet
        repeat(20) @(posedge clk);
        assert_no_strobe(seen_poll_short, "poll_short silent after byte 0 only");
        assert_no_strobe(seen_cmd_ready,  "cmd_ready silent after byte 0 only");

        // Now complete the command (bytes 1-2)
        send_byte(8'h03, 0);
        send_byte(8'h00, 1);
        wait_for_cmd(300);
        #15000;
        assert_strobe(seen_poll_short, "poll_short fires after all 3 bytes");

        // -----------------------------------------------------------------
        // TEST 7: Unknown single-byte command 0x42 → cmd_ready, no action
        // -----------------------------------------------------------------
        $display("\n[TEST 7] 0x42 (calibrate, unknown) → cmd_ready only");
        clear_latches;
        send_cmd_single(8'h42);
        wait_for_cmd(300);
        #15000;

        assert_strobe(seen_cmd_ready, "cmd_ready fired for unknown cmd");
        assert_no_strobe(seen_poll_short, "poll_short not fired");
        assert_no_strobe(seen_poll_long,  "poll_long not fired");
        assert_no_strobe(seen_info_req,   "info_req not fired");
        assert_no_strobe(seen_origin_req, "origin_req not fired");

        // -----------------------------------------------------------------
        // TEST 8: Unknown multi-byte command (0x42 with continue bytes)
        // -----------------------------------------------------------------
        $display("\n[TEST 8] 0x42 multi-byte → cmd_ready only after last byte");
        clear_latches;
        send_byte(8'h42, 0);   // cont=0
        send_byte(8'h03, 0);   // cont=0
        send_byte(8'h00, 1);   // stop=1
        wait_for_cmd(1000);
        #15000;

        assert_cmd(8'h42,     "cmd_out=0x42 (first byte)");
        assert_strobe(seen_cmd_ready,   "cmd_ready fired");
        assert_no_strobe(seen_poll_short, "poll_short not fired");
        assert_no_strobe(seen_poll_long,  "poll_long not fired");

        if (cmd_ready_count !== 1) begin
            $display("  FAIL [cmd_ready once]: fired %0d times", cmd_ready_count);
            fail_count = fail_count + 1;
        end else begin
            $display("  PASS [cmd_ready once]: fired exactly 1 time");
            pass_count = pass_count + 1;
        end

        // -----------------------------------------------------------------
        // TEST 9: Back-to-back — 0x00 then 0x40 after idle gap
        // -----------------------------------------------------------------
        $display("\n[TEST 9] Back-to-back: 0x00 then 0x40 after idle gap");
        clear_latches;
        send_cmd_single(8'h00);
        wait_for_cmd(300);
        #15000;

        clear_latches;
        send_cmd_poll(8'h40);
        wait_for_cmd(1000);
        #15000;

        assert_cmd(8'h40,   "cmd_out=0x40 (second command)");
        assert_strobe(seen_poll_short, "poll_short fires for second command");
        assert_no_strobe(seen_info_req, "info_req not fired for second command");

        // -----------------------------------------------------------------
        // TEST 10: Idle line — no false triggers in 50µs
        // -----------------------------------------------------------------
        $display("\n[TEST 10] Idle line — no false triggers in 50µs");
        clear_latches;
        begin : idle_test
            integer t10;
            reg spurious;
            spurious = 0;
            for (t10 = 0; t10 < 2500; t10 = t10 + 1) begin
                @(posedge clk);
                if (poll_short || poll_long || info_req ||
                    origin_req || cmd_ready)
                    spurious = 1;
            end
            if (spurious) begin
                $display("  FAIL [idle]: spurious trigger");
                fail_count = fail_count + 1;
            end else begin
                $display("  PASS [idle]: no spurious triggers in 50µs");
                pass_count = pass_count + 1;
            end
        end

        // -----------------------------------------------------------------
        // TEST 11: Rumble byte variation — 0x40 with rumble=1 in stop bit
        // The stop bit of byte 2 encodes rumble: 0=off, 1=on.
        // Either way poll_short must fire — we have no rumble motor so
        // both rumble values produce the same response.
        // -----------------------------------------------------------------
        $display("\n[TEST 11] 0x40 with rumble=1 in stop bit");
        clear_latches;
        send_byte(8'h40, 0);   // cmd, cont=0
        send_byte(8'h03, 0);   // mode, cont=0
        send_byte(8'h01, 1);   // rumble=1, stop=1 (stop bit encodes rumble value)
        wait_for_cmd(1000);
        #15000;

        assert_cmd(8'h40,   "cmd_out=0x40 (rumble variation)");
        assert_strobe(seen_poll_short, "poll_short fires regardless of rumble");

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        $display("\n============================================");
        $display("  Results: %0d passed, %0d failed",
                 pass_count, fail_count);
        if (fail_count == 0)
            $display("  *** ALL TESTS PASSED ***");
        else
            $display("  FAILED — open sim/gc_rx_tb.vcd in GTKWave");
        $display("============================================\n");

        $finish;
    end

endmodule
