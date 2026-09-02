`timescale 1ns / 1ps
//
// stick_map.v -- inverse stick correction, built from real OoT Wii-VC
// sweep data (see fit_octant_patches.py).
//
// v2 CHANGES FROM FIRST PASS (after counting actual register bits and
// finding it was ~143 bits, well over the 80-macrocell target):
//   1. Removed ax/ay as separate registers (16 bits) -- computed as
//      combinational wires and written directly into major/minor.
//   2. Replaced the abs-then-negate MAC scheme (term + abs_coef +
//      term_a + term_b = 57 bits of scratch) with a direct two's-
//      complement WEIGHTED-BIT accumulation: for a signed coefficient's
//      bits 0..7, add (val <<< bit) per set bit as before, but for the
//      final SIGN bit (bit 8), SUBTRACT (val <<< 8) instead of adding.
//      This is the standard two's-complement bit decomposition
//      (value = -sign*2^8 + sum of lower bits*2^i) and computes a
//      correctly-signed product directly into one running accumulator
//      -- no separate abs-value or negate-at-the-end step needed at all.
//   3. Single output byte (val_out) with an axis_sel input, matching
//      the byte-serial JoyBus flow, instead of two output registers.
//      val_out is COMBINATIONAL (a mux over the already-registered
//      mx_reg/mn_reg/swapped/sign bits), not itself a clocked register --
//      so once `done` has pulsed, a caller can toggle axis_sel and read
//      both axes immediately without re-asserting start (no need to
//      recompute the shared major/minor math twice).
//
// Net effect: register bit count drops from ~143 to ~70 (see tally in
// the accompanying analysis) -- back under the 80-macrocell target for
// registers, though actual macrocell count depends on how comparators/
// adders/case-statement ROMs pack, and can only be confirmed by running
// this through XST.
//
// ARCHITECTURE (unchanged from v1, see prior notes):
//   8-fold (D4) symmetry confirmed on real data (~0.01 unit mean error)
//   means only one octant (ax>=ay>=0) needs characterizing; everything
//   else is sign-flip/swap, free in hardware. 4 affine patches fit from
//   real sweep data, selected by the larger-axis ("major") value.
//
// Interface: latch val_x, val_y and pulse start; done pulses once when
// the computation is complete. val_out is combinational from that point
// on -- read it with axis_sel=0 for gc_x, then toggle axis_sel and read
// again on the next cycle for gc_y, with NO need to re-assert start.
// Values are signed, N64/GC common range -56..56 (this data's captured
// N64 max); adjust patch table for other games.
//
module stick_map (
    input  wire        clk,
    input  wire        start,
    input  wire        axis_sel,     // 0 = want gc_x, 1 = want gc_y
    input  wire signed [7:0] val_x,
    input  wire signed [7:0] val_y,
    output wire signed [7:0] val_out,
    output reg          done
);

    localparam N_PATCH = 4;
    localparam SHIFT    = 6;

    function [7:0] MAJ_HI(input [1:0] i);
        case (i)
            0: MAJ_HI = 8;   1: MAJ_HI = 20;
            2: MAJ_HI = 35;  default: MAJ_HI = 56;
        endcase
    endfunction

    function signed [8:0] A_MX(input [1:0] i);
        case (i) 0: A_MX=55; 1: A_MX=59; 2: A_MX=62; default: A_MX=88; endcase
    endfunction
    function signed [8:0] B_MX(input [1:0] i);
        case (i) 0: B_MX=67; 1: B_MX=33; 2: B_MX=28; default: B_MX=38; endcase
    endfunction
    function signed [7:0] C_MX(input [1:0] i);
        case (i) 0: C_MX=19; 1: C_MX=21; 2: C_MX=21; default: C_MX=3; endcase
    endfunction

    function signed [8:0] A_MN(input [1:0] i);
        case (i) 0: A_MN=-59; 1: A_MN=-25; 2: A_MN=-13; default: A_MN=2; endcase
    endfunction
    function signed [8:0] B_MN(input [1:0] i);
        case (i) 0: B_MN=234; 1: B_MN=126; 2: B_MN=105; default: B_MN=107; endcase
    endfunction
    function signed [7:0] C_MN(input [1:0] i);
        case (i) 0: C_MN=15; 1: C_MN=20; 2: C_MN=20; default: C_MN=10; endcase
    endfunction

    // ---- registers (see header comment for the reduction from v1) ----
    reg sign_x, sign_y, swapped;
    reg [7:0] major, minor;
    reg [1:0]  patch;
    reg        which_output;    // 0 = computing mx, 1 = computing mn
    reg signed [7:0] mx_reg, mn_reg;
    reg signed [15:0] acc;       // single running accumulator, both MAC phases
    reg [3:0]  bit_cnt;
    reg [3:0]  state;

    localparam S_IDLE      = 0,
               S_ABS_SWAP  = 1,
               S_ZERO_CHK  = 2,
               S_FIND_SEG  = 3,
               S_MAC_A     = 4,
               S_MAC_B     = 5,
               S_COMBINE   = 6,
               S_WRITEBACK = 7;

    wire signed [8:0] cur_a = which_output ? A_MN(patch) : A_MX(patch);
    wire signed [8:0] cur_b = which_output ? B_MN(patch) : B_MX(patch);
    wire signed [7:0] cur_c = which_output ? C_MN(patch) : C_MX(patch);

    // combinational abs -- feeds straight into major/minor in S_ABS_SWAP,
    // no separate ax/ay registers needed
    wire [7:0] ax_w = sign_x ? (-val_x) : val_x;
    wire [7:0] ay_w = sign_y ? (-val_y) : val_y;

    always @(posedge clk) begin
        done <= 1'b0;
        case (state)
            S_IDLE: begin
                if (start) begin
                    sign_x <= val_x[7];
                    sign_y <= val_y[7];
                    state  <= S_ABS_SWAP;
                end
            end

            S_ABS_SWAP: begin
                if (ax_w >= ay_w) begin
                    swapped <= 1'b0;
                    major   <= ax_w;
                    minor   <= ay_w;
                end else begin
                    swapped <= 1'b1;
                    major   <= ay_w;
                    minor   <= ax_w;
                end
                state <= S_ZERO_CHK;
            end

            // Correctness requirement: stick at rest must map to stick at
            // rest, regardless of what the fitted patches say near the
            // degenerate deadzone region.
            S_ZERO_CHK: begin
                if (ax_w == 8'd0 && ay_w == 8'd0) begin
                    mx_reg <= 8'sd0;
                    mn_reg <= 8'sd0;
                    state  <= S_WRITEBACK;
                end else begin
                    patch        <= 2'd0;
                    which_output <= 1'b0;
                    state        <= S_FIND_SEG;
                end
            end

            S_FIND_SEG: begin
                if (patch == N_PATCH-1 || major <= MAJ_HI(patch)) begin
                    acc     <= 16'sd0;
                    bit_cnt <= 4'd0;
                    state   <= S_MAC_A;
                end else begin
                    patch <= patch + 1;
                end
            end

            // Two's-complement weighted-bit MAC: bits 0..7 add, bit 8
            // (sign) subtracts -- computes major*cur_a directly into acc,
            // no abs-value/negate-afterward step required.
            S_MAC_A: begin
                if (bit_cnt < 4'd8) begin
                    if (cur_a[bit_cnt])
                        acc <= acc + ($signed({8'd0, major}) <<< bit_cnt);
                    bit_cnt <= bit_cnt + 1;
                end else begin
                    if (cur_a[8])
                        acc <= acc - ($signed({8'd0, major}) <<< 8);
                    bit_cnt <= 4'd0;
                    state   <= S_MAC_B;
                end
            end

            // Continues accumulating minor*cur_b into the SAME acc (so
            // acc ends up holding major*A + minor*B, ready for >>>SHIFT).
            S_MAC_B: begin
                if (bit_cnt < 4'd8) begin
                    if (cur_b[bit_cnt])
                        acc <= acc + ($signed({8'd0, minor}) <<< bit_cnt);
                    bit_cnt <= bit_cnt + 1;
                end else begin
                    if (cur_b[8])
                        acc <= acc - ($signed({8'd0, minor}) <<< 8);
                    state <= S_COMBINE;
                end
            end

            S_COMBINE: begin
                if (!which_output) begin
                    mx_reg       <= (acc >>> SHIFT) + cur_c;
                    which_output <= 1'b1;
                    acc          <= 16'sd0;
                    bit_cnt      <= 4'd0;
                    state        <= S_MAC_A;   // second pass: compute mn
                end else begin
                    mn_reg <= (acc >>> SHIFT) + cur_c;
                    state  <= S_WRITEBACK;
                end
            end

            S_WRITEBACK: begin
                // mx_reg/mn_reg/swapped/sign_x/sign_y all now hold stable
                // values -- val_out reads them out combinationally below,
                // so a caller can toggle axis_sel and read both axes off
                // this ONE computation without re-asserting start.
                done  <= 1'b1;
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end

    // Combinational output mux: un-swap, apply sign, select axis.
    // Valid once mx_reg/mn_reg are stable (i.e. after `done` has pulsed);
    // reading before that returns stale/undefined data, same as before.
    wire signed [7:0] x_mag = swapped ? mn_reg : mx_reg;
    wire signed [7:0] y_mag = swapped ? mx_reg : mn_reg;
    assign val_out = axis_sel ? (sign_y ? -y_mag : y_mag)
                              : (sign_x ? -x_mag : x_mag);

endmodule
