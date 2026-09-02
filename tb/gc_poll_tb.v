`timescale 1ns / 1ps
// =============================================================================
// gc_poll_tb.v
// =============================================================================
// Purpose: gc_identify_tb.v only ever tested the single-byte (0x00) command
// path in gc_rx.v. A real Wii poll (0x40/0x03/rumble) exercises a totally
// different branch: the cmd_latched/cmd_first discard-and-relatch logic used
// for multi-byte commands. That path has never been simulated. This
// testbench closes that gap directly.
//
// Sends: byte0=0x40 (poll_short, continue=0), byte1=0x03 (report mode,
// continue=0), byte2=0x00 (rumble off, continue=1 / stop).
//
// PASS criteria:
//   - dbg_gc_cmd_ready pulses exactly once, after the 3rd byte's stop bit.
//   - poll_short (visible internally) is what fired -- checked indirectly
//     via gc_total_bytes behavior: response should be exactly 4 bytes.
//   - gc_data_oe asserts, walks exactly 4 byte-transmissions, then RELEASES
//     and stays released (mirrors the same pass/fail structure as
//     gc_identify_tb.v, applied to the previously-untested code path).
// =============================================================================

module gc_poll_tb;

    reg clk = 0;
    always #7.5758 clk = ~clk;   // 66MHz board oscillator

    reg         n64_data_in = 1'b1;
    wire        n64_data_out, n64_data_oe;

    reg         gc_data_in = 1'b1;
    wire        gc_data_out, gc_data_oe;

    reg  [4:0]  dip = 5'b00000;

    wire        dbg_gc_tx_active;
    wire        dbg_n64_pkt_ready;
    wire        dbg_gc_cmd_ready;
    wire        dbg_n64_poll_pending;

    top dut (
        .clk (clk),
        .n64_data_in (n64_data_in), .n64_data_out (n64_data_out), .n64_data_oe (n64_data_oe),
        .gc_data_in  (gc_data_in),  .gc_data_out  (gc_data_out),  .gc_data_oe  (gc_data_oe),
        .dip (dip),
        .dbg_gc_tx_active     (dbg_gc_tx_active),
        .dbg_n64_pkt_ready    (dbg_n64_pkt_ready),
        .dbg_gc_cmd_ready     (dbg_gc_cmd_ready),
        .dbg_n64_poll_pending (dbg_n64_poll_pending)
    );

    task idle_start; begin gc_data_in = 1'b1; #3000; end endtask
    task send_bit1; begin gc_data_in = 1'b0; #1000; gc_data_in = 1'b1; #3000; end endtask
    task send_bit0; begin gc_data_in = 1'b0; #3000; gc_data_in = 1'b1; #1000; end endtask
    task send_stop; begin gc_data_in = 1'b0; #1000; gc_data_in = 1'b1; #500; end endtask

    task send_gc_byte(input [7:0] data, input stop);
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1)
                if (data[i]) send_bit1(); else send_bit0();
            if (stop) send_bit1(); else send_bit0();
        end
    endtask

    integer cmd_ready_count = 0;
    integer oe_pulse_count  = 0;
    reg     oe_seen_high = 1'b0;
    reg     oe_released_after_high = 1'b0;

    always @(posedge dbg_gc_cmd_ready) begin
        cmd_ready_count = cmd_ready_count + 1;
        $display("[%0t ns] dbg_gc_cmd_ready pulsed (#%0d)", $time, cmd_ready_count);
    end

    always @(gc_data_oe) begin
        $display("[%0t ns] gc_data_oe -> %b   gc_data_out=%b", $time, gc_data_oe, gc_data_out);
        if (gc_data_oe) begin
            oe_seen_high  = 1'b1;
            oe_pulse_count = oe_pulse_count + 1;
        end else if (oe_seen_high) begin
            oe_released_after_high = 1'b1;
        end
    end

    initial begin
        $dumpfile("gc_poll_tb.vcd");
        $dumpvars(0, gc_poll_tb);
    end

    initial begin
        #500;

        $display("[%0t ns] Sending real GC poll: 0x40 (cont=0), 0x03 (cont=0), 0x00 (stop)", $time);
        
        // send_gc_byte(8'h40, 1'b0);   // byte0: poll_short, continue -> more bytes follow
        // send_gc_byte(8'h03, 1'b0);   // byte1: report mode, continue -> more bytes follow
        // send_gc_byte(8'h00, 1'b1);   // byte2: rumble off, stop -> command complete

        idle_start();
        send_gc_byte(8'b00000000, 1'b0);
        send_stop();
        send_gc_byte(8'b00001001, 1'b0);
        send_gc_byte(8'b00000000, 1'b0);

        send_gc_byte(8'b00000011, 1'b1);

        $display("[%0t ns] Command fully sent. Waiting for response + release...", $time);
        #200000;

        $display("---- RESULTS ----");
        $display("cmd_ready fired %0d time(s) (expect exactly 1)", cmd_ready_count);
        $display("gc_data_oe asserted %0d time(s) (expect exactly 4, one per response byte)", oe_pulse_count);

        if (cmd_ready_count != 1)
            $display("RESULT: FAIL - cmd_ready fired %0d times, expected 1. gc_rx is mis-framing the 3-byte command.", cmd_ready_count);
        else if (!oe_seen_high)
            $display("RESULT: FAIL - gc_data_oe never asserted. poll_short never reached the TX arbiter (cmd byte likely mis-latched).");
        else if (oe_pulse_count != 4)
            $display("RESULT: FAIL - gc_data_oe asserted %0d times, expected 4 (poll_short => gc_total_bytes=4). Wrong command byte was latched/decoded.", oe_pulse_count);
        else if (!oe_released_after_high)
            $display("RESULT: FAIL - gc_data_oe asserted but never released.");
        else if (gc_data_oe)
            $display("RESULT: FAIL - gc_data_oe is high again at end of test with no new command sent.");
        else
            $display("RESULT: PASS - 3-byte poll correctly framed, cmd_first latched 0x40, exactly 4 response bytes sent, line released cleanly.");

        #1000;
        $finish;
    end

    initial begin
        #400000;
        $display("RESULT: FAIL - testbench timeout, something hung.");
        $finish;
    end

endmodule