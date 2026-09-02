`timescale 1ns / 1ps
// =============================================================================
// top.v — N64 to GameCube CPLD Adapter — Top Level Integration
// =============================================================================
// Target:  XC9572XL-10VQG44C (50MHz, 72 macrocells)
//
// ARCHITECTURE — Two independent parallel JoyBus conversations:
//
//   LINE A (N64 Controller side):
//     CPLD acts as CONSOLE — polls the N64 controller at ~60Hz.
//     n64_tx sends 0x01 command every POLL_PERIOD ticks.
//     n64_rx captures the 32-bit controller response.
//     n64_state register latches the result when pkt_ready fires.
//
//   LINE B (Wii GameCube port side):
//     CPLD acts as CONTROLLER — responds to Wii polls immediately.
//     gc_rx decodes Wii commands (0x00, 0xFF, 0x41, 0x40, 0x43).
//     gc_tx transmits the appropriate response from gc_state.
//     Response begins within 3-4 clock cycles (~60-80ns) of stop bit.
//
//   DECOUPLING POINT:
//     n64_state (32-bit register) latches the N64 packet once per poll.
//     button_map converts it combinationally to gc_state (64-bit wire).
//     gc_tx reads gc_state at the moment it responds — always current.
//     Worst case staleness: ~16.7ms (one poll period). Imperceptible.
//
//   GUARD FLAG:
//     gc_tx is blocked while n64_tx is active.
//     Prevents bus collision during the ~32µs n64_tx window.
//     Fewer than 0.2% of Wii polls affected at 60Hz.
//
// PORT MODEL — Split data_in/data_out/data_oe (simulation compatible):
//   Icarus Verilog cannot resolve multiple drivers on inout nets.
//   For real hardware synthesis, use a wrapper module:
//
//     module top_hw (...);
//       wire n64_data, gc_data;
//       assign n64_data    = n64_data_oe ? n64_data_out : 1'bz;
//       assign n64_data_in = n64_data;
//       assign gc_data     = gc_data_oe  ? gc_data_out  : 1'bz;
//       assign gc_data_in  = gc_data;
//       top core (...split ports...);
//     endmodule
//
//   ISE synthesizes the tristate assign to the CPLD's native OE logic.
//   The UCF only needs one NET per physical pin.
//
// LOOPBACK PREVENTION:
//   n64_rx.data_in is gated HIGH while n64_tx is active — prevents the
//   CPLD from decoding its own poll command as a controller response.
//   gc_rx.data_in is gated HIGH while gc_tx is active — prevents the
//   CPLD from decoding its own response as a new Wii command.
//   A 2-cycle synchronizer race exists on the first edge of each
//   transmission, but both receivers are in packet_done=1 state at
//   that moment so the spurious edge is harmless.
//
// DIP SWITCH:
//   dip[2:0] → button mapping mode (8 raphnet-style modes, button_map.v)
//   dip[4:3] → reserved. stick_map.v currently implements a single fixed
//              profile (OoT); per-game profile selection via dip[4:3] is
//              a future extension, not yet wired up.
// =============================================================================

module top (
    input  wire       clk,          // 50MHz crystal oscillator

    // Line A — N64 JoyBus (split for Icarus simulation compatibility)
    input  wire       n64_data_in,
    output wire       n64_data_out,
    output wire       n64_data_oe,

    // Line B — GC JoyBus (split for Icarus simulation compatibility)
    input  wire       gc_data_in,
    output wire       gc_data_out,
    output wire       gc_data_oe,

    // DIP switch — [2:0] mapping mode, [4:3] reserved
    input  wire [4:0] dip
);

    // =========================================================================
    // 60Hz Poll Timer
    // =========================================================================
    // 50_000_000 / 60 = 833_333 ticks per period.
    // Fires n64_send for exactly one clock cycle per period.
    // Free-running — no synchronization with Wii poll rate.
    // =========================================================================
    parameter POLL_PERIOD = 20'd833_333;

    reg [19:0] poll_counter = 20'd0;
    reg        n64_send     = 1'b0;

    always @(posedge clk) begin
        n64_send <= 1'b0;
        if (poll_counter >= POLL_PERIOD - 20'd1) begin
            poll_counter <= 20'd0;
            n64_send     <= 1'b1;
        end else begin
            poll_counter <= poll_counter + 20'd1;
        end
    end

    // =========================================================================
    // Line A — N64 JoyBus Wires
    // =========================================================================
    wire       n64_tx_data_out;
    wire       n64_tx_active;
    wire       n64_tx_done;

    wire       n64_bit_out;
    wire       n64_bit_valid;
    wire [31:0] n64_packet;
    wire       n64_pkt_ready;

    // Pin drives LOW only when n64_tx is transmitting
    assign n64_data_oe  = n64_tx_active;
    assign n64_data_out = n64_tx_data_out;

    n64_tx n64_transmitter (
        .clk      (clk),
        .send     (n64_send),
        .cmd      (8'h01),
        .data_out (n64_tx_data_out),
        .tx_active(n64_tx_active),
        .tx_done  (n64_tx_done)
    );

    n64_rx n64_receiver (
        .clk      (clk),
        // Loopback gate: hold HIGH during n64_tx so n64_rx ignores own command
        .data_in  (n64_tx_active ? 1'b1 : n64_data_in),
        .bit_out  (n64_bit_out),
        .bit_valid(n64_bit_valid),
        .packet   (n64_packet),
        .pkt_ready(n64_pkt_ready)
    );

    // =========================================================================
    // N64 State Register
    // =========================================================================
    // Initialized to neutral (all zeros = buttons released, sticks at 0).
    // Signed zero maps to GC center (128) via button_map's offset conversion.
    //
    // N64 packet layout:
    //   [31]=A  [30]=B  [29]=Z  [28]=Start
    //   [27]=D-Up  [26]=D-Down  [25]=D-Left  [24]=D-Right
    //   [23]=RST (not mapped — controller handles internally)
    //   [22]=Reserved
    //   [21]=L  [20]=R
    //   [19]=C-Up  [18]=C-Down  [17]=C-Left  [16]=C-Right
    //   [15:8]=Analog X (signed 8-bit, center=0)
    //   [7:0] =Analog Y (signed 8-bit, center=0)
    // =========================================================================
    reg [31:0] n64_state = 32'h0000_0000;

    always @(posedge clk) begin
        if (n64_pkt_ready)
            n64_state <= n64_packet;
    end

    // =========================================================================
    // Stick Correction — stick_map.v run-once controller
    // =========================================================================
    // stick_map is a multi-cycle FSM (see stick_map.v) that computes one
    // corrected axis per invocation, selected by axis_sel. Once `done`
    // pulses, val_out is purely combinational — a single `start` pulse
    // yields BOTH corrected axes: capture axis_sel=0's result the cycle
    // `done` fires, then flip axis_sel and capture again the next cycle,
    // with no need to re-assert start.
    //
    // Triggered once per new N64 packet (n64_pkt_ready), not every clock —
    // the shared major/minor math only needs recomputing when the sticks
    // actually change. Runs in well under 100 cycles, negligible against
    // the ~16.7ms N64 poll period.
    //
    // stick_map's val_out is signed, range -56..56. GC expects unsigned
    // 0-255 (center=128). "+ 8'sd128" performs that offset: for any 8-bit
    // signed value, two's-complement addition of 128 modulo 256 is exact
    // (equivalent to flipping the sign bit) — no separate clamp needed.
    // =========================================================================
    wire        sm_done;
    wire signed [7:0] sm_val_out;
    reg         sm_start    = 1'b0;
    reg         sm_axis_sel = 1'b0;
    reg  [1:0]  sm_state    = 2'd0;

    localparam SM_IDLE    = 2'd0,
               SM_WAIT    = 2'd1,
               SM_LATCH_Y = 2'd2;

    reg [7:0] gc_stick_x_reg = 8'h80;  // centered until first correction runs
    reg [7:0] gc_stick_y_reg = 8'h80;

    stick_map stick_correction (
        .clk     (clk),
        .start   (sm_start),
        .axis_sel(sm_axis_sel),
        .val_x   (n64_state[15:8]),
        .val_y   (n64_state[7:0]),
        .val_out (sm_val_out),
        .done    (sm_done)
    );

    always @(posedge clk) begin
        sm_start <= 1'b0;
        case (sm_state)
            SM_IDLE: begin
                if (n64_pkt_ready) begin
                    sm_start    <= 1'b1;
                    sm_axis_sel <= 1'b0;
                    sm_state    <= SM_WAIT;
                end
            end

            SM_WAIT: begin
                if (sm_done) begin
                    // val_out currently holds the corrected X axis (axis_sel=0)
                    gc_stick_x_reg <= sm_val_out + 8'sd128;
                    sm_axis_sel    <= 1'b1;  // val_out becomes Y next cycle
                    sm_state       <= SM_LATCH_Y;
                end
            end

            SM_LATCH_Y: begin
                gc_stick_y_reg <= sm_val_out + 8'sd128;
                sm_state       <= SM_IDLE;
            end

            default: sm_state <= SM_IDLE;
        endcase
    end

    // =========================================================================
    // Button/Axis Translation
    // =========================================================================
    // Pure combinational — gc_state updates instantly when n64_state or the
    // latched stick registers change. gc_tx latches gc_state into its shift
    // register at response time.
    // =========================================================================
    wire [63:0] gc_state;

    button_map mapper (
        .n64_state    (n64_state),
        .dip          (dip[2:0]),
        .gc_stick_x_in(gc_stick_x_reg),
        .gc_stick_y_in(gc_stick_y_reg),
        .gc_state     (gc_state)
    );

    // =========================================================================
    // Line B — GC JoyBus Wires
    // =========================================================================
    wire       gc_rx_poll_short;
    wire       gc_rx_poll_long;
    wire       gc_rx_info_req;
    wire       gc_rx_origin_req;
    wire [7:0] gc_rx_cmd_out;    // available for debug LEDs
    wire       gc_rx_cmd_ready;  // available for debug LEDs

    wire       gc_tx_data_out;
    wire       gc_tx_active;
    wire       gc_tx_done;       // available for debug LEDs

    assign gc_data_oe  = gc_tx_active;
    assign gc_data_out = gc_tx_data_out;

    // Guard flag: block gc_tx while n64_tx is transmitting
    wire gc_send_info   = gc_rx_info_req   && !n64_tx_active;
    wire gc_send_origin = gc_rx_origin_req && !n64_tx_active;
    wire gc_send_short  = gc_rx_poll_short && !n64_tx_active;
    wire gc_send_long   = gc_rx_poll_long  && !n64_tx_active;

    gc_rx gc_receiver (
        .clk       (clk),
        // Loopback gate: hold HIGH during gc_tx so gc_rx ignores own response
        .data_in   (gc_tx_active ? 1'b1 : gc_data_in),
        .poll_short(gc_rx_poll_short),
        .poll_long (gc_rx_poll_long),
        .info_req  (gc_rx_info_req),
        .origin_req(gc_rx_origin_req),
        .cmd_out   (gc_rx_cmd_out),
        .cmd_ready (gc_rx_cmd_ready)
    );

    gc_tx gc_transmitter (
        .clk        (clk),
        .send_info  (gc_send_info),
        .send_origin(gc_send_origin),
        .send_short (gc_send_short),
        .send_long  (gc_send_long),
        .gc_state   (gc_state),
        .data_out   (gc_tx_data_out),
        .tx_active  (gc_tx_active),
        .tx_done    (gc_tx_done)
    );

endmodule
