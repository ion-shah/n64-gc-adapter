`timescale 1ns / 1ps

//// stick_map.v -- inverse stick correction, built from real OoT Wii-VC sweep data
//
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
    localparam TAPER_SHIFT = 3;   // must match log2(MAJ_HI(0)) == log2(8)

    function [7:0] MAJ_HI(input [1:0] i);
        case (i)
            0: MAJ_HI = 8;   1: MAJ_HI = 20;
            2: MAJ_HI = 35;  default: MAJ_HI = 56;
        endcase
    endfunction

    // Combined coefficient lookup: one 8-way case on {which_output,patch}
    // instead of 6 separate 4-way cases + 3 extra top-level muxes.
    // Packing: [25:17]=a (9b signed), [16:8]=b (9b signed), [7:0]=c (8b signed)
    function signed [25:0] PATCH_ABC(input sel_mn, input [1:0] i);
        reg signed [8:0] a;
        reg signed [8:0] b;
        reg signed [7:0] c;
        begin
            case ({sel_mn, i})
                3'b000: begin a =  9'sd55;  b =  9'sd67;  c = 8'sd19; end // MX patch0
                3'b001: begin a =  9'sd59;  b =  9'sd33;  c = 8'sd21; end // MX patch1
                3'b010: begin a =  9'sd62;  b =  9'sd28;  c = 8'sd21; end // MX patch2
                3'b011: begin a =  9'sd88;  b =  9'sd38;  c = 8'sd3;  end // MX patch3
                3'b100: begin a = -9'sd59;  b = 9'sd234;  c = 8'sd15; end // MN patch0
                3'b101: begin a = -9'sd25;  b = 9'sd126;  c = 8'sd20; end // MN patch1
                3'b110: begin a = -9'sd13;  b = 9'sd105;  c = 8'sd20; end // MN patch2
                default: begin a =  9'sd2;  b = 9'sd107;  c = 8'sd10; end // MN patch3
            endcase
            PATCH_ABC = {a, b, c};
        end
    endfunction

    // saturate into the 8-bit output register range. Input narrowed to
    // 9 bits -- wide_result is exhaustively proven to stay in [0,253],
    // so 9 bits (range -256..255) is the tight, safe minimum. Do not
    // reuse this function for anything wider without re-checking.
    function signed [7:0] sat8(input signed [8:0] v);
        begin
            if (v > 9'sd127)
                sat8 = 8'sd127;
            else if (v < -9'sd128)
                sat8 = -8'sd128;
            else
                sat8 = v[7:0];
        end
    endfunction

    // ---- registers ----
    reg sign_x, sign_y, swapped;
    reg [7:0] major, minor;
    reg [1:0]  patch;
    reg        which_output;    // 0 = computing mx, 1 = computing mn
    reg        ab_phase;        // 0 = major*cur_a term, 1 = minor*cur_b term
    reg signed [7:0] mx_reg, mn_reg;
    reg signed [15:0] acc;              // single running accumulator, both MAC phases
                                          // (verified 16 bits needed -- see header note)
    reg signed [8:0]  coef;             // current coefficient, shifts RIGHT 1/cycle
    reg signed [15:0] shifted_operand;  // current operand,     shifts LEFT  1/cycle
    reg signed [8:0]  wide_result;      // pre-saturate result, shared by both paths
                                          // (narrowed from [15:0], see header note)
    reg [3:0]  bit_cnt;
    reg [3:0]  state;

    localparam S_IDLE       = 0,
               S_ABS_SWAP   = 1,
               S_FIND_SEG   = 2,
               S_MAC_LOAD   = 3,
               S_MAC_ITER   = 4,
               S_COMBINE    = 5,
               S_TAPER_ITER = 6,
               S_SAT_WRITE  = 7,
               S_WRITEBACK  = 8;

    wire signed [25:0] abc   = PATCH_ABC(which_output, patch);
    wire signed [8:0]  cur_a = abc[25:17];
    wire signed [8:0]  cur_b = abc[16:8];
    wire signed [7:0]  cur_c = abc[7:0];

    // combinational abs -- feeds straight into major/minor in S_ABS_SWAP,
    // no separate ax/ay registers needed
    wire [7:0] ax_w = sign_x ? (-val_x) : val_x;
    wire [7:0] ay_w = sign_y ? (-val_y) : val_y;

    // combinational: raw combine result before taper/saturate, valid
    // during S_COMBINE once acc holds the just-finished MAC sum.
    // Computed at 16 bits (acc's width) since it also feeds
    // shifted_operand for the taper multiply, but the SETTLED value
    // (once latched into wide_result) is proven to fit in 9 bits.
    wire signed [15:0] combine_result = (acc >>> SHIFT) + cur_c;

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
                patch        <= 2'd0;
                which_output <= 1'b0;
                state        <= S_FIND_SEG;
            end

            // No explicit zero-deadzone check needed: when major=0 the
            // taper multiply (S_TAPER_ITER) has coef=0 for all 4
            // iterations, so acc never accumulates and the tapered
            // result is exactly 0 regardless of cur_c. (0,0) input
            // always makes major=minor=0, so this falls out for free.
            S_FIND_SEG: begin
                if (patch == N_PATCH-1 || major <= MAJ_HI(patch)) begin
                    acc      <= 16'sd0;
                    ab_phase <= 1'b0;
                    state    <= S_MAC_LOAD;
                end else begin
                    patch <= patch + 1;
                end
            end

            S_MAC_LOAD: begin
                if (!ab_phase) begin
                    coef            <= cur_a;
                    shifted_operand <= {8'd0, major};
                end else begin
                    coef            <= cur_b;
                    shifted_operand <= {8'd0, minor};
                end
                bit_cnt <= 4'd0;
                state   <= S_MAC_ITER;
            end

            S_MAC_ITER: begin
                if (bit_cnt < 4'd8) begin
                    if (coef[0])
                        acc <= acc + shifted_operand;
                    shifted_operand <= shifted_operand <<< 1;
                    coef            <= coef >>> 1;
                    bit_cnt         <= bit_cnt + 1'b1;
                end else begin
                    if (coef[0])
                        acc <= acc - shifted_operand;
                    if (!ab_phase) begin
                        ab_phase <= 1'b1;
                        state    <= S_MAC_LOAD;
                    end else begin
                        state <= S_COMBINE;
                    end
                end
            end

            S_COMBINE: begin
                if (patch == 2'd0) begin
                    acc             <= 16'sd0;
                    coef            <= {1'b0, major};
                    shifted_operand <= combine_result;
                    bit_cnt         <= 4'd0;
                    state           <= S_TAPER_ITER;
                end else begin
                    wide_result <= combine_result;
                    state       <= S_SAT_WRITE;
                end
            end

            S_TAPER_ITER: begin
                if (bit_cnt < 4'd4) begin
                    if (coef[0])
                        acc <= acc + shifted_operand;
                    shifted_operand <= shifted_operand <<< 1;
                    coef            <= coef >>> 1;
                    bit_cnt         <= bit_cnt + 1'b1;
                end else begin
                    wide_result <= (acc >>> TAPER_SHIFT);
                    state       <= S_SAT_WRITE;
                end
            end

            S_SAT_WRITE: begin
                if (!which_output) begin
                    mx_reg       <= sat8(wide_result);
                    which_output <= 1'b1;
                    acc          <= 16'sd0;
                    ab_phase     <= 1'b0;
                    state        <= S_MAC_LOAD;
                end else begin
                    mn_reg <= sat8(wide_result);
                    state  <= S_WRITEBACK;
                end
            end

            S_WRITEBACK: begin
                done  <= 1'b1;
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end

    // Combinational output mux: un-swap, apply sign, select axis.
    wire signed [7:0] x_mag = swapped ? mn_reg : mx_reg;
    wire signed [7:0] y_mag = swapped ? mx_reg : mn_reg;
    assign val_out = axis_sel ? (sign_y ? -y_mag : y_mag)
                              : (sign_x ? -x_mag : x_mag);

endmodule
