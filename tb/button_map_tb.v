`timescale 1ns / 1ps
// =============================================================================
// button_map_tb.v — Testbench for button_map.v
// =============================================================================
// Pure combinational module — no clock needed. Tests apply inputs and check
// outputs after a small propagation delay (#10).
//
// Tests cover:
//   1.  Default (000): Z/L swap, C-stick all 4 buttons, X/Y not mapped
//   2.  L map   (001): Z passthrough, C-stick all 4 buttons
//   3.  R map   (010): C-Right→X, C-Left→Y, C-stick X centered (buttons promoted)
//   4.  L+R map (011): Same as R but Z/L swap from L map
//   5.  C-Up    (100): C-Down→X, C-Left→Y, Z←R, L←Z, R←L
//   6.  C-Down  (101): R←D-Up, C-Down→X, C-Left→Y
//   7.  C-Left  (110): C-Down→X, Y not mapped, Z←R, R←L
//   8.  C-Right (111): C-Left→X, C-Down→Y, Z←R, L←L, R←Z
//   9.  Analog stick scaling: center, +40, -40, max positive, max negative
//   10. C-stick: all four directions + both pressed = center
//   11. RST bit [23] and Reserved [22] are NOT mapped to any GC output
//   12. D-pad CDN special: D-Up not mapped to GC D-Up, maps to GC R instead
// =============================================================================

module button_map_tb;

    reg  [31:0] n64_state;
    reg  [2:0]  dip;
    wire [63:0] gc_state;

    button_map dut (
        .n64_state(n64_state),
        .dip      (dip),
        .gc_state (gc_state)
    );

    // Convenience: extract gc_state fields
    wire [7:0] gc_byte0   = gc_state[63:56];
    wire [7:0] gc_byte1   = gc_state[55:48];
    wire [7:0] gc_stick_x = gc_state[47:40];
    wire [7:0] gc_stick_y = gc_state[39:32];
    wire [7:0] gc_cstk_x  = gc_state[31:24];
    wire [7:0] gc_cstk_y  = gc_state[23:16];
    wire [7:0] gc_l_ana   = gc_state[15:8];
    wire [7:0] gc_r_ana   = gc_state[7:0];

    // Byte 0 bits
    wire gc_start = gc_byte0[4];
    wire gc_y     = gc_byte0[3];
    wire gc_x     = gc_byte0[2];
    wire gc_b     = gc_byte0[1];
    wire gc_a     = gc_byte0[0];

    // Byte 1 bits
    wire gc_l_dig  = gc_byte1[6];
    wire gc_r_dig  = gc_byte1[5];
    wire gc_z      = gc_byte1[4];
    wire gc_dup    = gc_byte1[3];
    wire gc_ddown  = gc_byte1[2];
    wire gc_dright = gc_byte1[1];
    wire gc_dleft  = gc_byte1[0];

    integer pass_count = 0;
    integer fail_count = 0;

    task check;
        input actual;
        input expected;
        input [8*48-1:0] name;
        begin
            if (actual !== expected) begin
                $display("  FAIL [%0s]: got %b expected %b", name, actual, expected);
                fail_count = fail_count + 1;
            end else begin
                $display("  PASS [%0s]", name);
                pass_count = pass_count + 1;
            end
        end
    endtask

    task check8;
        input [7:0] actual;
        input [7:0] expected;
        input [8*48-1:0] name;
        begin
            if (actual !== expected) begin
                $display("  FAIL [%0s]: got 0x%02X expected 0x%02X",
                         name, actual, expected);
                fail_count = fail_count + 1;
            end else begin
                $display("  PASS [%0s]: 0x%02X", name, actual);
                pass_count = pass_count + 1;
            end
        end
    endtask

    // Build an N64 state with only specific bits set
    // Bit positions match n64_rx packet format
    localparam A      = 32'h8000_0000;  // [31]
    localparam B      = 32'h4000_0000;  // [30]
    localparam Z      = 32'h2000_0000;  // [29]
    localparam START  = 32'h1000_0000;  // [28]
    localparam DUP    = 32'h0800_0000;  // [27]
    localparam DDOWN  = 32'h0400_0000;  // [26]
    localparam DLEFT  = 32'h0200_0000;  // [25]
    localparam DRIGHT = 32'h0100_0000;  // [24]
    localparam RST    = 32'h0080_0000;  // [23] — should NOT appear in output
    localparam L      = 32'h0020_0000;  // [21]
    localparam R      = 32'h0010_0000;  // [20]
    localparam CUP    = 32'h0008_0000;  // [19]
    localparam CDOWN  = 32'h0004_0000;  // [18]
    localparam CLEFT  = 32'h0002_0000;  // [17]
    localparam CRIGHT = 32'h0001_0000;  // [16]

    initial begin
        $display("============================================");
        $display("  button_map Testbench");
        $display("  Ref: raphnet-tech.com N64-to-Wii mappings");
        $display("============================================");

        // =================================================================
        // TEST 1: Default mapping (dip=000)
        // Z/L swap: GC Z←N64 L, GC L←N64 Z
        // X/Y not mapped, C-all on C-stick
        // =================================================================
        $display("\n[TEST 1] Default mapping (dip=000)");
        dip = 3'd0;

        // A, B passthrough
        n64_state = A; #10;
        check(gc_a, 1, "A passthrough");
        check(gc_b, 0, "B when only A");

        n64_state = B; #10;
        check(gc_b, 1, "B passthrough");

        // Z/L swap
        n64_state = Z; #10;
        check(gc_z,    0, "Z: N64 Z does NOT drive GC Z (default swap)");
        check(gc_l_dig, 1, "Z: N64 Z drives GC L (default swap)");

        n64_state = L; #10;
        check(gc_z,    1, "L: N64 L drives GC Z (default swap)");
        check(gc_l_dig, 0, "L: N64 L does NOT drive GC L");

        n64_state = R; #10;
        check(gc_r_dig, 1, "R passthrough");
        check8(gc_r_ana, 8'hFF, "R analog full when R pressed");

        // X/Y not mapped
        n64_state = CRIGHT; #10;
        check(gc_x, 0, "C-Right: GC X NOT mapped in default mode");

        n64_state = CLEFT; #10;
        check(gc_y, 0, "C-Left: GC Y NOT mapped in default mode");

        // C-stick all four directions
        n64_state = CRIGHT; #10;
        check8(gc_cstk_x, 8'hFF, "C-Right → C-stick X = 0xFF");

        n64_state = CLEFT; #10;
        check8(gc_cstk_x, 8'h00, "C-Left → C-stick X = 0x00");

        n64_state = CUP; #10;
        check8(gc_cstk_y, 8'hFF, "C-Up → C-stick Y = 0xFF");

        n64_state = CDOWN; #10;
        check8(gc_cstk_y, 8'h00, "C-Down → C-stick Y = 0x00");

        n64_state = 32'h0; #10;
        check8(gc_cstk_x, 8'h80, "no C-buttons → C-stick X centered");
        check8(gc_cstk_y, 8'h80, "no C-buttons → C-stick Y centered");

        // Start passthrough
        n64_state = START; #10;
        check(gc_start, 1, "Start passthrough");

        // =================================================================
        // TEST 2: L mapping (dip=001)
        // Z passthrough: GC Z←N64 Z, GC L←N64 L
        // =================================================================
        $display("\n[TEST 2] L mapping (dip=001)");
        dip = 3'd1;

        n64_state = Z; #10;
        check(gc_z,     1, "L-map: N64 Z → GC Z (passthrough)");
        check(gc_l_dig, 0, "L-map: N64 Z does NOT drive GC L");

        n64_state = L; #10;
        check(gc_z,     0, "L-map: N64 L does NOT drive GC Z");
        check(gc_l_dig, 1, "L-map: N64 L → GC L (passthrough)");

        // X/Y still not mapped
        n64_state = CRIGHT; #10;
        check(gc_x, 0, "L-map: X still not mapped");

        // =================================================================
        // TEST 3: R mapping (dip=010)
        // Z/L swap, C-Right→X, C-Left→Y
        // C-Right and C-Left removed from C-stick when used as face buttons
        // =================================================================
        $display("\n[TEST 3] R mapping (dip=010)");
        dip = 3'd2;

        // Z/L swap same as default
        n64_state = L; #10;
        check(gc_z, 1, "R-map: N64 L → GC Z");

        // C-Right promoted to X
        n64_state = CRIGHT; #10;
        check(gc_x, 1, "R-map: C-Right → GC X");
        // C-Right promoted — should NOT deflect C-stick X
        check8(gc_cstk_x, 8'h80, "R-map: C-Right promoted, C-stick X centered");

        // C-Left promoted to Y
        n64_state = CLEFT; #10;
        check(gc_y, 1, "R-map: C-Left → GC Y");
        check8(gc_cstk_x, 8'h80, "R-map: C-Left promoted, C-stick X still centered");

        // C-Up/C-Down still drive C-stick Y
        n64_state = CUP; #10;
        check8(gc_cstk_y, 8'hFF, "R-map: C-Up still drives C-stick Y");

        n64_state = CDOWN; #10;
        check8(gc_cstk_y, 8'h00, "R-map: C-Down still drives C-stick Y");

        // =================================================================
        // TEST 4: L+R mapping (dip=011)
        // Z passthrough, C-Right→X, C-Left→Y
        // =================================================================
        $display("\n[TEST 4] L+R mapping (dip=011)");
        dip = 3'd3;

        n64_state = Z; #10;
        check(gc_z,     1, "L+R-map: Z passthrough");
        check(gc_l_dig, 0, "L+R-map: Z not on L");

        n64_state = CRIGHT; #10;
        check(gc_x, 1, "L+R-map: C-Right → X");
        check8(gc_cstk_x, 8'h80, "L+R-map: C-Right promoted, C-stick X centered");

        // =================================================================
        // TEST 5: C-Up mapping (dip=100)
        // Z←R, L←Z, R←L, X←C-Down, Y←C-Left
        // =================================================================
        $display("\n[TEST 5] C-Up mapping (dip=100)");
        dip = 3'd4;

        n64_state = R; #10;
        check(gc_z,     1, "CUP-map: N64 R → GC Z");
        check(gc_r_dig, 0, "CUP-map: N64 R not on GC R");

        n64_state = Z; #10;
        check(gc_l_dig, 1, "CUP-map: N64 Z → GC L");

        n64_state = L; #10;
        check(gc_r_dig, 1, "CUP-map: N64 L → GC R");
        check8(gc_r_ana, 8'hFF, "CUP-map: N64 L → GC R analog full");

        n64_state = CDOWN; #10;
        check(gc_x, 1, "CUP-map: C-Down → GC X");
        check8(gc_cstk_y, 8'h80, "CUP-map: C-Down promoted, C-stick Y centered");

        n64_state = CLEFT; #10;
        check(gc_y, 1, "CUP-map: C-Left → GC Y");

        // C-Up still drives C-stick Y up
        n64_state = CUP; #10;
        check8(gc_cstk_y, 8'hFF, "CUP-map: C-Up still drives C-stick Y");

        // =================================================================
        // TEST 6: C-Down mapping (dip=101)
        // R←D-Up special case, X←C-Down, Y←C-Left
        // =================================================================
        $display("\n[TEST 6] C-Down mapping (dip=101)");
        dip = 3'd5;

        // D-Up maps to GC R
        n64_state = DUP; #10;
        check(gc_r_dig, 1,    "CDN-map: N64 D-Up → GC R digital");
        check8(gc_r_ana, 8'hFF, "CDN-map: N64 D-Up → GC R analog full");
        check(gc_dup,   0,    "CDN-map: N64 D-Up NOT on GC D-Up");

        // D-Down still maps normally
        n64_state = DDOWN; #10;
        check(gc_ddown, 1, "CDN-map: D-Down still passthrough");

        // =================================================================
        // TEST 7: C-Left mapping (dip=110)
        // Z←R, L←Z, R←L, X←C-Down, Y not mapped
        // =================================================================
        $display("\n[TEST 7] C-Left mapping (dip=110)");
        dip = 3'd6;

        n64_state = CLEFT; #10;
        check(gc_y, 0, "CLT-map: Y not mapped");
        // C-Left still on C-stick (not promoted)
        check8(gc_cstk_x, 8'h00, "CLT-map: C-Left still on C-stick X");

        n64_state = CDOWN; #10;
        check(gc_x, 1, "CLT-map: C-Down → GC X");
        check8(gc_cstk_y, 8'h80, "CLT-map: C-Down promoted, C-stick Y centered");

        // =================================================================
        // TEST 8: C-Right mapping (dip=111)
        // Z←R, L←L, R←Z, X←C-Left, Y←C-Down
        // =================================================================
        $display("\n[TEST 8] C-Right mapping (dip=111)");
        dip = 3'd7;

        n64_state = R; #10;
        check(gc_z,     1, "CRT-map: N64 R → GC Z");

        n64_state = L; #10;
        check(gc_l_dig, 1, "CRT-map: N64 L → GC L");

        n64_state = Z; #10;
        check(gc_r_dig, 1, "CRT-map: N64 Z → GC R");

        n64_state = CLEFT; #10;
        check(gc_x, 1, "CRT-map: C-Left → GC X");
        check8(gc_cstk_x, 8'h80, "CRT-map: C-Left promoted, C-stick X centered");

        n64_state = CDOWN; #10;
        check(gc_y, 1, "CRT-map: C-Down → GC Y");
        check8(gc_cstk_y, 8'h80, "CRT-map: C-Down promoted, C-stick Y centered");

        // =================================================================
        // TEST 9: Analog stick scaling (dip=000, mapping doesn't affect stick)
        // =================================================================
        $display("\n[TEST 9] Analog stick scaling");
        dip = 3'd0;

        // Center: X=0, Y=0 → GC 0x80 both
        n64_state = 32'h0000_0000; #10;
        check8(gc_stick_x, 8'h80, "stick X center (0 → 128)");
        check8(gc_stick_y, 8'h80, "stick Y center (0 → 128)");

        // X=+40 (0x28): 40 + 10 + 128 = 178 = 0xB2
        n64_state = 32'h0000_2800; #10;
        check8(gc_stick_x, 8'hB2, "stick X = +40 → 178 (0xB2)");

        // X=-40 (0xD8 = -40 signed): -40 + (-10) + 128 = 78 = 0x4E
        n64_state = 32'h0000_D800; #10;
        check8(gc_stick_x, 8'h4E, "stick X = -40 → 78 (0x4E)");

        // Y=+40 (0x28)
        n64_state = 32'h0000_0028; #10;
        check8(gc_stick_y, 8'hB2, "stick Y = +40 → 178 (0xB2)");

        // Clamp test: X=+127 (0x7F): 127 + 31 = 158 + 128 = 286 → clamp 255
        n64_state = 32'h0000_7F00; #10;
        check8(gc_stick_x, 8'hFF, "stick X = +127 → clamped to 255");

        // Clamp test: X=-128 (0x80): -128 + (-32) = -160 + 128 = -32 → clamp 0
        n64_state = 32'h0000_8000; #10;
        check8(gc_stick_x, 8'h00, "stick X = -128 → clamped to 0");

        // =================================================================
        // TEST 10: C-stick both directions pressed = center
        // =================================================================
        $display("\n[TEST 10] C-stick: both directions = center");
        dip = 3'd0;

        // C-Right AND C-Left both pressed
        n64_state = CRIGHT | CLEFT; #10;
        check8(gc_cstk_x, 8'h80, "C-Right+C-Left both → C-stick X centered");

        // C-Up AND C-Down both pressed
        n64_state = CUP | CDOWN; #10;
        check8(gc_cstk_y, 8'h80, "C-Up+C-Down both → C-stick Y centered");

        // =================================================================
        // TEST 11: RST bit [23] and Reserved [22] not mapped
        // =================================================================
        $display("\n[TEST 11] RST and Reserved bits not mapped");
        dip = 3'd0;

        // Set RST=1, Reserved=1, all else 0
        n64_state = RST | 32'h0040_0000; #10;
        // gc_byte0 should have ONLY origin_unchecked=1 set (bit [5]=0x20)
        // gc_byte1 should have ONLY High1=1 set (bit [7]=0x80)
        // No button bits should be set — RST and Reserved are not mapped
        if (gc_byte0 !== 8'h20 || gc_byte1 !== 8'h80) begin
            $display("  FAIL [RST not mapped]: gc_byte0=0x%02X (expected 0x20) gc_byte1=0x%02X (expected 0x80)",
                     gc_byte0, gc_byte1);
            fail_count = fail_count + 1;
        end else begin
            $display("  PASS [RST/Reserved not mapped]: byte0=0x20 (origin only) byte1=0x80 (High1 only)");
            pass_count = pass_count + 1;
        end
        check8(gc_stick_x, 8'h80, "RST set: stick X still centered");
        check8(gc_stick_y, 8'h80, "RST set: stick Y still centered");

        // =================================================================
        // TEST 12: CDN D-Up special — D-Up goes to R, not D-Up in gc
        // =================================================================
        $display("\n[TEST 12] CDN mapping D-Up special case");
        dip = 3'd5;

        n64_state = DUP; #10;
        check(gc_dup,   0, "CDN D-Up: GC D-Up NOT set");
        check(gc_r_dig, 1, "CDN D-Up: GC R digital IS set");

        // Other D-pad still maps normally in CDN mode
        n64_state = DLEFT; #10;
        check(gc_dleft, 1, "CDN: D-Left still passthrough");

        // =================================================================
        // Summary
        // =================================================================
        $display("\n============================================");
        $display("  Results: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0)
            $display("  *** ALL TESTS PASSED ***");
        else
            $display("  FAILED — review output above");
        $display("============================================\n");

        $finish;
    end

endmodule
