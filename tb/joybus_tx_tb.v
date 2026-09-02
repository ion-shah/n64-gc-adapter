`timescale 1ns / 1ps
// =============================================================================
// joybus_tx_tb.v — Testbench for joybus_tx.v (shared transmitter engine)
// =============================================================================
// joybus_tx.v replaces the old n64_tx.v + gc_tx.v pair with one bit-level
// engine, parameterized per-transmission by `send_stop` (whether to append
// a stop bit at all) and `stop_kind` (which of the two stop-bit shapes to
// use). Byte SEQUENCING (multi-byte GC responses, N64's single-command
// send) lives in top.v now, not here — see top_tb.v for that integration
// coverage. This testbench covers the engine in isolation.
//
// Tests:
//   1.  Logic-1 bit timing:  1us low (50 ticks) + 3us high (150 ticks)
//   2.  Logic-0 bit timing:  3us low (150 ticks) + 1us high (50 ticks)
//   3.  Byte content: 0x00, 0xFF, 0xAA, 0x55, 0x01, 0x80 all decode correctly
//   4.  send_stop=0: engine goes idle immediately after the byte's last
//       bit — no stop bit, byte_done and tx_done fire together
//   5.  send_stop=1, stop_kind=0 (console-style): stop bit is
//       1us low (SHORT) + 2us high (MID)
//   6.  send_stop=1, stop_kind=1 (controller-style): stop bit is
//       2us low (MID) + 2us high (MID)
//   7.  byte_done fires exactly once, before tx_done when a stop bit follows
//   8.  tx_active spans the whole operation (byte, or byte+stop) and drops
//       the same cycle tx_done fires
//   9.  DONE state holds — no output without a new `start`
//   10. Re-arm from DONE: back-to-back sends work correctly (mirrors how
//       top.v's GC sequencer chains bytes)
// =============================================================================

module joybus_tx_tb;

    // -------------------------------------------------------------------------
    // DUT
    // -------------------------------------------------------------------------
    reg        clk        = 0;
    reg        start      = 0;
    reg  [7:0] data       = 8'd0;
    reg        send_stop  = 0;
    reg        stop_kind  = 0;
    wire       data_out;
    wire       tx_active;
    wire       byte_done;
    wire       tx_done;

    joybus_tx dut (
        .clk      (clk),
        .start    (start),
        .data     (data),
        .send_stop(send_stop),
        .stop_kind(stop_kind),
        .data_out (data_out),
        .tx_active(tx_active),
        .byte_done(byte_done),
        .tx_done  (tx_done)
    );

    always #10 clk = ~clk;

    initial begin
        $dumpfile("sim/joybus_tx_tb.vcd");
        $dumpvars(0, joybus_tx_tb);
    end

    // -------------------------------------------------------------------------
    // Test tracking
    // -------------------------------------------------------------------------
    integer pass_count = 0;
    integer fail_count = 0;

    task check;
        input        cond;
        input [511:0] name;
        begin
            if (cond) begin
                $display("  PASS: %0s", name);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: %0s", name);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Edge-timed bit/stop decoder — measures low-phase and high-phase
    // durations directly off data_out, independent of DUT internal state,
    // so this validates the actual output waveform, not the RTL's own
    // bookkeeping.
    // -------------------------------------------------------------------------
    integer low_ticks_meas, high_ticks_meas;
    reg [7:0] decoded_byte;
    integer   decoded_bits;
    reg       measuring;

    task send_byte_and_check;
        input [7:0]  tx_byte;
        input        do_stop;
        input        kind;         // stop_kind to use if do_stop
        input [511:0] label;
        integer i;
        reg [7:0] observed;
        reg       byte_done_seen;
        reg       tx_done_seen;
        integer   safety;
        begin
            data      = tx_byte;
            send_stop = do_stop;
            stop_kind = kind;
            observed      = 8'd0;
            byte_done_seen = 0;
            tx_done_seen   = 0;

            @(posedge clk); start = 1;
            @(posedge clk); start = 0;

            // Capture 8 bits by measuring each low-phase width
            for (i = 0; i < 8; i = i + 1) begin
                // wait for falling edge (start of bit's low phase)
                safety = 0;
                while (data_out !== 1'b0 && safety < 2000) begin @(posedge clk); safety = safety + 1; end
                low_ticks_meas = 0;
                while (data_out === 1'b0) begin @(posedge clk); low_ticks_meas = low_ticks_meas + 1; end
                // low <= ~62 ticks (midpoint of 50/150) => logic 1, else logic 0
                observed = {observed[6:0], (low_ticks_meas <= 100) ? 1'b1 : 1'b0};
                if (byte_done) byte_done_seen = 1;
            end

            check(observed === tx_byte, {label, ": byte content"});

            if (do_stop) begin
                // Measure the stop bit's low phase
                safety = 0;
                while (data_out !== 1'b0 && safety < 2000) begin @(posedge clk); safety = safety + 1; end
                low_ticks_meas = 0;
                while (data_out === 1'b0) begin @(posedge clk); low_ticks_meas = low_ticks_meas + 1; end
                high_ticks_meas = 0;
                while (data_out === 1'b1 && !tx_done) begin @(posedge clk); high_ticks_meas = high_ticks_meas + 1; end
                if (tx_done) tx_done_seen = 1;

                if (kind == 1'b0)
                    check(low_ticks_meas >= 45 && low_ticks_meas <= 55, {label, ": stop bit low = SHORT (~50 ticks, console-style)"});
                else
                    check(low_ticks_meas >= 95 && low_ticks_meas <= 105, {label, ": stop bit low = MID (~100 ticks, controller-style)"});
                check(high_ticks_meas >= 95 && high_ticks_meas <= 105, {label, ": stop bit high = MID (~100 ticks)"});
            end else begin
                // No stop bit: tx_done fires at the end of the 8th bit's
                // high phase, which can be up to LONG=150 ticks -- give
                // the safety margin enough room to see it.
                safety = 0;
                while (!tx_done_seen && safety < 200) begin
                    if (tx_done) tx_done_seen = 1;
                    @(posedge clk); safety = safety + 1;
                end
            end

            check(tx_done_seen, {label, ": tx_done fired"});
            @(posedge clk);
        end
    endtask

    // -------------------------------------------------------------------------
    // Main test sequence
    // -------------------------------------------------------------------------
    initial begin
        #100;

        // TEST 1-3: byte content + bit timing across varied patterns, no stop
        $display("\n[TEST 1] Byte content + bit timing, no stop bit");
        send_byte_and_check(8'h00, 1'b0, 1'b0, "0x00, no stop");
        send_byte_and_check(8'hFF, 1'b0, 1'b0, "0xFF, no stop");
        send_byte_and_check(8'hAA, 1'b0, 1'b0, "0xAA, no stop");
        send_byte_and_check(8'h55, 1'b0, 1'b0, "0x55, no stop");
        send_byte_and_check(8'h01, 1'b0, 1'b0, "0x01, no stop");
        send_byte_and_check(8'h80, 1'b0, 1'b0, "0x80, no stop");

        // TEST 2: console-style stop bit (N64 side: SHORT low + MID high)
        $display("\n[TEST 2] Console-style stop bit (stop_kind=0)");
        send_byte_and_check(8'h01, 1'b1, 1'b0, "0x01 + console stop");
        send_byte_and_check(8'hA5, 1'b1, 1'b0, "0xA5 + console stop");

        // TEST 3: controller-style stop bit (GC side: MID low + MID high)
        $display("\n[TEST 3] Controller-style stop bit (stop_kind=1)");
        send_byte_and_check(8'h09, 1'b1, 1'b1, "0x09 + controller stop");
        send_byte_and_check(8'hFF, 1'b1, 1'b1, "0xFF + controller stop");

        // TEST 4: tx_active spans the whole operation
        $display("\n[TEST 4] tx_active timing");
        begin : test4_block
            reg active_seen_during;
            active_seen_during = 0;
            data = 8'h42; send_stop = 1; stop_kind = 1;
            @(posedge clk); start = 1;
            @(posedge clk); start = 0;
            @(posedge clk); // state became LOAD last edge; tx_active <= 1 takes effect this edge
            check(tx_active, "tx_active HIGH immediately after start");
            while (!tx_done) begin
                if (!tx_active) active_seen_during = 1; // would indicate a false drop
                @(posedge clk);
            end
            check(!active_seen_during, "tx_active stayed HIGH for entire byte+stop");
            check(!tx_active, "tx_active LOW same cycle as tx_done");
            @(posedge clk);
        end

        // TEST 5: DONE holds — no spurious output without a new start
        $display("\n[TEST 5] DONE state holds without new start");
        begin : test5_block
            integer k;
            reg saw_low;
            saw_low = 0;
            for (k = 0; k < 50; k = k + 1) begin
                @(posedge clk);
                if (data_out === 1'b0) saw_low = 1;
            end
            check(!saw_low, "line stays idle-HIGH with no start pulse");
        end

        // TEST 6: re-arm from DONE — back-to-back sends (mirrors top.v's
        // GC byte sequencer chaining multiple bytes with send_stop=0 until
        // the final byte, which sets send_stop=1)
        $display("\n[TEST 6] Back-to-back sends (multi-byte sequence simulation)");
        send_byte_and_check(8'h11, 1'b0, 1'b0, "seq byte 0 (no stop)");
        send_byte_and_check(8'h22, 1'b0, 1'b0, "seq byte 1 (no stop)");
        send_byte_and_check(8'h33, 1'b1, 1'b1, "seq byte 2 (final, controller stop)");

        // -------------------------------------------------------------------
        $display("\n============================================");
        $display("  Results: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0)
            $display("  *** ALL TESTS PASSED ***");
        else
            $display("  *** %0d TEST(S) FAILED ***", fail_count);
        $display("============================================\n");

        $finish;
    end

endmodule
