`timescale 1ns / 1ps
// =============================================================================
// n64_rx_tb.v — Testbench for n64_rx
// =============================================================================
// Reference: https://n64brew.dev/wiki/Joybus_Protocol
//
// JoyBus timing per n64brew.dev spec (all controller bits 4us total at 50MHz):
//
//   Logic 1          : 1us low  ( 50 ticks) + 3us high (150 ticks) = 4us
//   Logic 0          : 3us low  (150 ticks) + 1us high ( 50 ticks) = 4us
//   Controller Stop  : 2us low  (100 ticks) + 2us high (100 ticks) = 4us
//   Console Stop Bit : 1us low  ( 50 ticks) + 2us high (100 ticks) = 3us
//     (Console stop sent by CPLD after 0x01 — handled by n64_tx, not here)
//
//   Controller Stop Bit sits at exactly THRESHOLD (100 ticks) — boundary case.
//   packet_done blocks it by position after 32 data bits so classification
//   of the stop bit pulse width is irrelevant in real operation.
//
// Tests:
//   1. Single logic-1 bit  (1us low → bit_out=1)
//   2. Single logic-0 bit  (3us low → bit_out=0)
//   3. Controller stop bit standalone (2us low — hits THRESHOLD boundary)
//   4. Full packet 32'hABCD_1234 with correct 2us stop bit
//   5. A-button only: 32'h8000_0000
//   6. Stick packet X=+40 Y=-40: 32'h0000_28D8
//   7. Back-to-back second packet: 32'h1400_0000
//   8. Idle line — no false triggers in 50us
//   9. Stop bit isolation — verify stop bit does NOT corrupt packet
// =============================================================================

module n64_rx_tb;

    // -------------------------------------------------------------------------
    // DUT
    // -------------------------------------------------------------------------
    reg         clk     = 0;
    reg         data_in = 1;   // idles HIGH (open-drain, 1kΩ pull-up to 3.3V)

    wire        bit_out;
    wire        bit_valid;
    wire [31:0] packet;
    wire        pkt_ready;

    n64_rx dut (
        .clk      (clk),
        .data_in  (data_in),
        .bit_out  (bit_out),
        .bit_valid(bit_valid),
        .packet   (packet),
        .pkt_ready(pkt_ready)
    );

    // -------------------------------------------------------------------------
    // Clock: 50MHz = 20ns period
    // -------------------------------------------------------------------------
    always #10 clk = ~clk;

    // -------------------------------------------------------------------------
    // VCD dump for GTKWave
    // -------------------------------------------------------------------------
    initial begin
        $dumpfile("sim/n64_rx_tb.vcd");
        $dumpvars(0, n64_rx_tb);
    end

    // -------------------------------------------------------------------------
    // Test tracking
    // -------------------------------------------------------------------------
    integer pass_count = 0;
    integer fail_count = 0;

    // -------------------------------------------------------------------------
    // JoyBus stimulus tasks
    // -------------------------------------------------------------------------
    // All delays match spec at 50MHz (20ns/tick).
    // To update for measured hardware values: change #1000 and #3000 to match
    // your actual measured pulse widths from PulseView (e.g. #980 for 980ns).

    task send_bit_1;
        // Spec: 1us low + 3us high = 4us total
        begin
            data_in = 0; #1000;    // 1us low  = 50 ticks
            data_in = 1; #3000;    // 3us high = 150 ticks
        end
    endtask

    task send_bit_0;
        // Spec: 3us low + 1us high = 4us total
        begin
            data_in = 0; #3000;    // 3us low  = 150 ticks
            data_in = 1; #1000;    // 1us high = 50 ticks
        end
    endtask

    task send_controller_stop_bit;
        // Spec (n64brew.dev): Controller Stop Bit = 2us low + 2us high = 4us total
        // This is NOT the same as logic-0 (3us low). It sits at the THRESHOLD
        // midpoint (100 ticks). packet_done blocks it by position after 32 bits —
        // pulse-width classification of the stop bit is intentionally irrelevant.
        // After the 2us high phase the line returns to idle until the next poll.
        begin
            data_in = 0; #2000;    // 2us low  = 100 ticks (exactly at THRESHOLD)
            data_in = 1; #2000;    // 2us high = 100 ticks, then line idles HIGH
        end
    endtask

    task send_byte;
        input [7:0] b;
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                if (b[i]) send_bit_1;
                else      send_bit_0;
            end
        end
    endtask

    // Send full 32-bit N64 controller response packet + controller stop bit
    // This is what the real controller sends in response to a 0x01 command.
    task send_controller_response;
        input [31:0] pkt;
        begin
            send_byte(pkt[31:24]);
            send_byte(pkt[23:16]);
            send_byte(pkt[15:8]);
            send_byte(pkt[7:0]);
            send_controller_stop_bit;
        end
    endtask

    // -------------------------------------------------------------------------
    // Latch: catches 1-cycle pulses reliably
    // -------------------------------------------------------------------------
    reg seen_bit_valid  = 0;
    reg seen_pkt_ready  = 0;
    reg latched_bit_out = 0;

    always @(posedge clk) begin
        if (bit_valid) begin
            seen_bit_valid  <= 1;
            latched_bit_out <= bit_out;
        end
        if (pkt_ready)
            seen_pkt_ready <= 1;
    end

    task clear_latches;
        begin
            @(posedge clk); #1;
            seen_bit_valid  = 0;
            seen_pkt_ready  = 0;
            latched_bit_out = 0;
        end
    endtask

    // -------------------------------------------------------------------------
    // Assertion helpers
    // -------------------------------------------------------------------------
    task assert_bit;
        input expected;
        input [8*32-1:0] name;
        begin
            repeat(10) @(posedge clk);
            if (!seen_bit_valid) begin
                $display("  FAIL [%0s]: bit_valid never asserted", name);
                fail_count = fail_count + 1;
            end else if (latched_bit_out !== expected) begin
                $display("  FAIL [%0s]: bit_out=%b expected=%b", name, latched_bit_out, expected);
                fail_count = fail_count + 1;
            end else begin
                $display("  PASS [%0s]: bit_out=%b", name, latched_bit_out);
                pass_count = pass_count + 1;
            end
        end
    endtask

    task assert_packet;
        input [31:0] expected;
        input [8*32-1:0] name;
        begin : wait_loop
            integer timeout;
            timeout = 0;
            while (!seen_pkt_ready && timeout < 300000) begin
                @(posedge clk);
                timeout = timeout + 20;
            end
            if (!seen_pkt_ready) begin
                $display("  FAIL [%0s]: pkt_ready never asserted (timeout)", name);
                fail_count = fail_count + 1;
            end else if (packet !== expected) begin
                $display("  FAIL [%0s]: packet=32'h%08X expected=32'h%08X",
                         name, packet, expected);
                fail_count = fail_count + 1;
            end else begin
                $display("  PASS [%0s]: packet=32'h%08X", name, packet);
                pass_count = pass_count + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Test sequence
    // -------------------------------------------------------------------------
    initial begin
        $display("============================================");
        $display("  n64_rx Testbench");
        $display("  Ref: n64brew.dev/wiki/Joybus_Protocol");
        $display("============================================");

        #500;

        // -----------------------------------------------------------------
        // TEST 1: Logic-1 bit (1us low)
        // -----------------------------------------------------------------
        $display("\n[TEST 1] Logic-1 bit: 1us low + 3us high → bit_out=1");
        clear_latches;
        send_bit_1;
        assert_bit(1'b1, "logic-1");
        #2000;

        // -----------------------------------------------------------------
        // TEST 2: Logic-0 bit (3us low)
        // -----------------------------------------------------------------
        $display("\n[TEST 2] Logic-0 bit: 3us low + 1us high → bit_out=0");
        clear_latches;
        send_bit_0;
        assert_bit(1'b0, "logic-0");
        #2000;

        // -----------------------------------------------------------------
        // TEST 3: Controller stop bit in isolation (2us low + 2us high)
        // Per spec: controller stop bit = 2us low (100 ticks) = THRESHOLD boundary.
        // When standalone (packet_done=0), bit_valid will fire — value is
        // boundary case (could be 0 or 1 depending on exact tick count).
        // In real use packet_done blocks this after 32 bits — irrelevant there.
        // We just verify bit_valid fires and no crash occurs.
        // -----------------------------------------------------------------
        $display("\n[TEST 3] Controller stop bit (2us low — THRESHOLD boundary, standalone)");
        $display("         bit_valid should fire; value is boundary case (0 or 1 ok)");
        clear_latches;
        send_controller_stop_bit;
        repeat(10) @(posedge clk);
        if (!seen_bit_valid) begin
            $display("  FAIL [stop-bit-standalone]: bit_valid never asserted");
            fail_count = fail_count + 1;
        end else begin
            $display("  PASS [stop-bit-standalone]: bit_valid fired, bit_out=%b",
                     latched_bit_out);
            pass_count = pass_count + 1;
        end
        #2000;

        // Flush stray bits from tests 1-3 before packet tests
        // Need >10us (500 ticks) HIGH to trigger IDLE_THRESH reset
        #15000;

        // -----------------------------------------------------------------
        // TEST 4: Full packet 32'hABCD_1234 + correct stop bit
        // 0xAB = 1010_1011: A, Z, Start, D-Down, D-Right pressed
        // 0xCD = 1100_1101: L, C-Down, C-Left pressed (bits 23:22 reserved=1 here)
        // 0x12 = 0001_0010: Analog X = +18
        // 0x34 = 0011_0100: Analog Y = +52
        // -----------------------------------------------------------------
        $display("\n[TEST 4] Full packet 32'hABCD_1234 + controller stop bit");
        clear_latches;
        send_controller_response(32'hABCD_1234);
        assert_packet(32'hABCD_1234, "packet ABCD1234");

        // -----------------------------------------------------------------
        // TEST 5: A-button only, all else zero, sticks centered
        // -----------------------------------------------------------------
        $display("\n[TEST 5] A-button only: 32'h8000_0000");
        #15000;
        clear_latches;
        send_controller_response(32'h8000_0000);
        assert_packet(32'h8000_0000, "A-button only");

        // -----------------------------------------------------------------
        // TEST 6: Stick only — X=+40 (0x28), Y=-40 (0xD8 two's complement)
        // Verifies signed axis decoding. Y=-40: 0xD8 = 1101_1000 = -40 signed.
        // -----------------------------------------------------------------
        $display("\n[TEST 6] Stick X=+40 Y=-40: 32'h0000_28D8");
        #15000;
        clear_latches;
        send_controller_response(32'h000028D8);
        assert_packet(32'h000028D8, "stick X+40 Y-40");
        if (seen_pkt_ready) begin
            $display("         Analog X = %0d (expect +40)", $signed(packet[15:8]));
            $display("         Analog Y = %0d (expect -40)", $signed(packet[7:0]));
        end

        // -----------------------------------------------------------------
        // TEST 7: Back-to-back second packet (32'h1400_0000 = Z+Start pressed)
        // -----------------------------------------------------------------
        $display("\n[TEST 7] Back-to-back 2nd packet: 32'h1400_0000");
        #15000;
        clear_latches;
        send_controller_response(32'h14000000);
        assert_packet(32'h14000000, "back-to-back");

        // -----------------------------------------------------------------
        // TEST 8: Idle line — no false triggers for 50us
        // -----------------------------------------------------------------
        $display("\n[TEST 8] Idle line: no false bit_valid or pkt_ready in 50us");
        clear_latches;
        begin : idle_watch
            integer t;
            reg false_seen;
            false_seen = 0;
            for (t = 0; t < 2500; t = t + 1) begin
                @(posedge clk);
                if (bit_valid || pkt_ready) false_seen = 1;
            end
            if (false_seen) begin
                $display("  FAIL [idle]: spurious trigger during 50us idle");
                fail_count = fail_count + 1;
            end else begin
                $display("  PASS [idle]: no spurious triggers in 50us");
                pass_count = pass_count + 1;
            end
        end

        // -----------------------------------------------------------------
        // TEST 9: Stop bit does NOT corrupt packet
        // Send a known packet, verify packet register holds correct value
        // after the 3us stop bit has been transmitted and received.
        // This is the key regression test for the packet_done mechanism.
        // -----------------------------------------------------------------
        $display("\n[TEST 9] Stop bit isolation: packet stable after stop bit");
        #15000;
        clear_latches;
        send_controller_response(32'hDEADBEEF);
        // Wait for pkt_ready first
        begin : wait9
            integer t9;
            t9 = 0;
            while (!seen_pkt_ready && t9 < 300000) begin
                @(posedge clk); t9 = t9 + 20;
            end
        end
        if (!seen_pkt_ready) begin
            $display("  FAIL [stop isolation]: pkt_ready never fired");
            fail_count = fail_count + 1;
        end else begin
            // Stop bit already sent as part of send_controller_response.
            // Wait a few extra clocks for any pipeline to settle, then check.
            repeat(20) @(posedge clk);
            if (packet !== 32'hDEADBEEF) begin
                $display("  FAIL [stop isolation]: packet=32'h%08X after stop bit, expected 32'hDEADBEEF",
                         packet);
                $display("         Stop bit corrupted the packet register!");
                fail_count = fail_count + 1;
            end else begin
                $display("  PASS [stop isolation]: packet=32'h%08X stable after stop bit", packet);
                pass_count = pass_count + 1;
            end
        end

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        $display("\n============================================");
        $display("  Results: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0)
            $display("  *** ALL TESTS PASSED ***");
        else
            $display("  FAILED — open sim/n64_rx_tb.vcd in GTKWave");
        $display("============================================\n");

        $finish;
    end

endmodule
