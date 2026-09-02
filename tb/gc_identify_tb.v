`timescale 1ns / 1ps
// =============================================================================
// gc_identify_tb.v
// =============================================================================
// Purpose: isolate "does the RTL's shared TX engine ever correctly assert
// then RELEASE (tristate) gc_data_oe" from the hardware symptom ("gc_data
// reads stuck high once the CPLD is wired in"). This testbench has no
// ground/signal-integrity problem by construction — it's pure logic — so if
// gc_data_oe still gets stuck (asserts and never deasserts, or never
// asserts at all despite a clean command), that's a genuine RTL bug and NOT
// explained by the ground-reference theory. If it behaves correctly here,
// that's strong evidence the fault is purely in the physical layer.
//
// Sequence:
//   1. Reset / settle.
//   2. Bit-bang a GC 0x00 "identify" command onto gc_data_in — 8 data bits
//      (all logic-0: 3us low / 1us high) + a continue/stop bit = 1
//      (logic-1: 1us low / 3us high), per gc_rx.v's 9-bit byte encoding.
//   3. Watch gc_data_oe / gc_data_out / gc_tx_active (via dbg_gc_tx_active)
//      and dbg_gc_cmd_ready.
//   4. PASS criteria:
//        - dbg_gc_cmd_ready pulses once, shortly after the 9th bit.
//        - dbg_gc_tx_active / gc_data_oe rise within a few clk_core cycles
//          after that (info_req -> gc_pending -> TXARB -> tx_dest=1).
//        - gc_data_oe stays high only for the duration of the 3-byte
//          response + stop bit, then drops back to 0 (release/tristate)
//          and STAYS low afterward (no controller polling it further).
// =============================================================================

module gc_identify_tb;

    // -------------------------------------------------------------------
    // Clock: 66MHz board oscillator, ~15.1515ns period
    // -------------------------------------------------------------------
    reg clk = 0;
    always #7.5758 clk = ~clk;

    // -------------------------------------------------------------------
    // DUT connections
    // -------------------------------------------------------------------
    reg         n64_data_in = 1'b1;   // idle high, no N64 controller attached
    wire        n64_data_out, n64_data_oe;

    reg         gc_data_in = 1'b1;    // idle high (pull-up)
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

    // -------------------------------------------------------------------
    // GC-side bit tasks (real time, not tick-counted — matches the actual
    // 1us/3us/2us pulse widths regardless of internal clk_core divider)
    // -------------------------------------------------------------------
    task send_bit1; // logic-1: 1us low, 3us high
        begin
            gc_data_in = 1'b0; #1000;
            gc_data_in = 1'b1; #3000;
        end
    endtask

    task send_bit0; // logic-0: 3us low, 1us high
        begin
            gc_data_in = 1'b0; #3000;
            gc_data_in = 1'b1; #1000;
        end
    endtask

    // Send one 9-bit GC command byte: 8 data bits MSB-first + continue/stop
    task send_gc_byte(input [7:0] data, input stop);
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                if (data[i]) send_bit1(); else send_bit0();
            end
            if (stop) send_bit1(); else send_bit0();
        end
    endtask

    // -------------------------------------------------------------------
    // Trace gc_data_oe transitions with a timestamp, so we can literally
    // see whether it releases or gets stuck.
    // -------------------------------------------------------------------
    reg oe_seen_high = 1'b0;
    reg oe_released_after_high = 1'b0;

    always @(gc_data_oe) begin
        $display("[%0t ns] gc_data_oe -> %b   gc_data_out=%b  gc_tx_active=%b",
                  $time, gc_data_oe, gc_data_out, dbg_gc_tx_active);
        if (gc_data_oe) oe_seen_high = 1'b1;
        else if (oe_seen_high) oe_released_after_high = 1'b1;
    end

    always @(posedge dbg_gc_cmd_ready)
        $display("[%0t ns] dbg_gc_cmd_ready pulsed (gc_rx decoded a byte)", $time);

    // -------------------------------------------------------------------
    // VCD dump
    // -------------------------------------------------------------------
    initial begin
        $dumpfile("gc_identify_tb.vcd");
        $dumpvars(0, gc_identify_tb);
    end

    // -------------------------------------------------------------------
    // Test sequence
    // -------------------------------------------------------------------
    initial begin
        // Let the clock divider / reset state settle
        #500;

        $display("[%0t ns] Sending GC identify command 0x00 (single byte, stop=1)", $time);
        send_gc_byte(8'h00, 1'b1);

        $display("[%0t ns] Command fully sent. Waiting for response + release...", $time);

        // Response is 3 bytes (info_req) at up to LONG(3us)+MID(2us) worst
        // case per bit -> generous margin: wait well past that.
        #100000;

        if (!oe_seen_high) begin
            $display("RESULT: FAIL — gc_data_oe never asserted.");
            $display("        gc_rx never decoded the command into info_req/gc_pending,");
            $display("        or the TXARB sequencer never granted the engine to GC side.");
        end else if (!oe_released_after_high) begin
            $display("RESULT: FAIL — gc_data_oe asserted but NEVER released.");
            $display("        This would be a genuine RTL tristate bug (stuck driving),");
            $display("        NOT explained by a hardware ground/reference issue.");
        end else if (gc_data_oe) begin
            $display("RESULT: FAIL — gc_data_oe is high again at end of test with no new command sent.");
        end else begin
            $display("RESULT: PASS — gc_data_oe asserted for the response and cleanly released.");
            $display("        RTL tristate behavior is correct in simulation.");
            $display("        This points the 'stuck high' hardware symptom at the physical");
            $display("        layer (ground reference / signal integrity), not this logic.");
        end

        #1000;
        $finish;
    end

    // Safety timeout
    initial begin
        #200000;
        $display("RESULT: FAIL — testbench timeout, something hung.");
        $finish;
    end

endmodule