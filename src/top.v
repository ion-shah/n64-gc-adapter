`timescale 1ns / 1ps
// =============================================================================
// top.v — N64 to GameCube CPLD Adapter — Top Level Integration
// =============================================================================
//
//
// ARCHITECTURE — Two independent parallel JoyBus conversations, one shared
// transmitter engine:
//
//   LINE A (N64 Controller side):
//     CPLD acts as CONSOLE — polls the N64 controller at ~60Hz.
//     Poll timer requests the shared engine send 8'h01 + a console-style
//     stop bit every POLL_PERIOD ticks.
//     n64_rx captures the 32-bit controller response.
//     n64_state register latches the result when pkt_ready fires.
//
//   LINE B (Wii GameCube port side):
//     CPLD acts as CONTROLLER — responds to Wii polls immediately.
//     gc_rx decodes Wii commands (0x00, 0xFF, 0x41, 0x40, 0x43).
//     A small sequencer (below) walks byte_index through gc_state, feeding
//     the shared engine one byte at a time, ending with a controller-style
//     stop bit on the last byte.
//
//   SHARED TRANSMITTER ENGINE (joybus_tx.v):
//     n64_tx.v and gc_tx.v used to be two independent copies of the same
//     bit-level FSM (differing only in stop-bit shape) — never active at
//     the same time (see GUARD, below), which is exactly the precondition
//     for time-sharing one physical engine instead of paying for two.
//     joybus_tx does ONE byte + optional stop bit per `start` pulse;
//     sequencing (which byte, how many, gc_state's byte-select mux) lives
//     here in top.v since it genuinely differs per direction.
//
//   DECOUPLING POINT:
//     n64_state (32-bit register) latches the N64 packet once per poll.
//     button_map converts it combinationally to gc_state (64-bit wire).
//     The GC sequencer reads gc_state at the moment it responds — always
//     current. Worst case staleness: ~16.7ms (one poll period).
//
//   GUARD / ARBITRATION:
//     Because both directions now share ONE engine, mutual exclusion is
//     automatic — a request simply waits until the engine is free, no
//     manual "block gc_tx while n64_tx active" flag needed anymore. The
//     N64 poll is given priority if both happen to want the engine on the
//     same cycle (astronomically rare given a 16.7ms poll period vs a
//     response that completes in well under 100us).
//
// PORT MODEL — Split data_in/data_out/data_oe (simulation compatible):
//   Icarus Verilog cannot resolve multiple drivers on inout nets.
//   For real hardware synthesis, top_hw.v is the wrapper module:
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
//   n64_rx.data_in is gated HIGH while the engine is transmitting to the
//   N64 side — prevents the CPLD from decoding its own poll command as a
//   controller response.
//   gc_rx.data_in is gated HIGH while the engine is transmitting to the
//   GC side — prevents the CPLD from decoding its own response as a new
//   Wii command.
//   A 2-cycle synchronizer race exists on the first edge of each
//   transmission, but both receivers are in packet_done=1 state at
//   that moment so the spurious edge is harmless.
//
// DIP SWITCH:
//   dip[2:0] → button mapping mode (8 raphnet-style modes, button_map.v)
//   dip[4:3] → empty. stick_map.v currently implements a single fixed
//              profile (OoT); per-game profile selection via dip[4:3] is
//              a future extension, not yet wired up. not enough area
// =============================================================================

module top (
    input  wire       clk,          // 66MHz crystal oscillator
    input  wire       n64_data_in,
    output wire       n64_data_out,
    output wire       n64_data_oe,
    input  wire       gc_data_in,
    output wire       gc_data_out,
    output wire       gc_data_oe,
    input  wire [4:0] dip,

    // ---- Debug taps (drive onboard LEDs via top_hw.v) ----------------------
    // dbg_gc_tx_active     : HIGH while the shared TX engine is driving the
    //                        GC line. If this never lights, gc_rx is never
    //                        decoding a valid Wii command (points at the
    //                        ground/input-threshold issue) rather than a
    //                        stuck-output bug.
    // dbg_n64_pkt_ready     : one-cycle pulse each time a full N64 controller
    //                        packet is captured — confirms Line A is alive.
    // dbg_gc_cmd_ready      : one-cycle pulse each time gc_rx finishes
    //                        decoding a Wii command byte — confirms Line B
    //                        RX is seeing valid edges at all, independent of
    //                        whether a reply ever gets sent.
    // dbg_n64_poll_pending  : mirrors n64_poll_pending — toggles/pulses once
    //                        per POLL_PERIOD, a cheap "core clock is alive
    //                        and the poll timer is running" heartbeat.
    // --------------------------------------------------------------------
    output wire       dbg_gc_tx_active,
    output wire       dbg_n64_pkt_ready,
    output wire       dbg_gc_cmd_ready,
    output wire       dbg_n64_poll_pending,
    output wire       dbg_clk_core
);
    // =========================================================================
    // Clock Divider — 66MHz board oscillator -> 22MHz core clock
    // =========================================================================
    // Need to fix setup time violations, so we just slow down the clock
    //
    // Every synchronous block in this design, and every submodule instance,
    // must run on clk_core, NOT the raw 66MHz clk -- clk itself should only
    // ever appear in this one always block. Duty cycle is asymmetric (1 fast
    // cycle high, 2 low) by construction -- irrelevant here since everything
    // downstream is posedge-triggered; only the period between clk_core's
    // rising edges matters for timing.
    // =========================================================================

    reg [1:0] div_cnt  = 2'd0;
    reg       clk_core = 1'b0;

    always @(posedge clk) begin
        div_cnt  <= (div_cnt == 2'd2) ? 2'd0 : div_cnt + 2'd1;
        clk_core <= (div_cnt == 2'd2);
    end

    parameter POLL_PERIOD = 20'd366_667;

    reg [19:0] poll_counter = 20'd0;
    wire poll_fire = (poll_counter >= POLL_PERIOD - 20'd1);

    always @(posedge clk_core) begin
        if (poll_fire)
            poll_counter <= 20'd0;
        else
            poll_counter <= poll_counter + 20'd1;
    end

    reg n64_poll_pending = 1'b0;

    wire       n64_bit_out;
    wire       n64_bit_valid;
    wire [31:0] n64_packet;
    wire       n64_pkt_ready;
    wire       n64_tx_active;

    n64_rx n64_receiver (
        .clk      (clk_core),
        .data_in  (n64_tx_active ? 1'b1 : n64_data_in),
        .bit_out  (n64_bit_out),
        .bit_valid(n64_bit_valid),
        .packet   (n64_packet),
        .pkt_ready(n64_pkt_ready)
    );

    reg [31:0] n64_state = 32'h0000_0000;

    always @(posedge clk_core) begin
        if (n64_pkt_ready)
            n64_state <= n64_packet;
    end

    wire        sm_done;
    wire signed [7:0] sm_val_out;
    reg         sm_start    = 1'b0;
    reg         sm_axis_sel = 1'b0;
    reg  [1:0]  sm_state    = 2'd0;

    localparam SM_IDLE    = 2'd0,
               SM_WAIT    = 2'd1,
               SM_LATCH_Y = 2'd2;

    reg [7:0] gc_stick_x_reg = 8'h80;
    reg [7:0] gc_stick_y_reg = 8'h80;

    stick_map stick_correction (
        .clk     (clk_core),
        .start   (sm_start),
        .axis_sel(sm_axis_sel),
        .val_x   (n64_state[15:8]),
        .val_y   (n64_state[7:0]),
        .val_out (sm_val_out),
        .done    (sm_done)
    );

    always @(posedge clk_core) begin
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
                    gc_stick_x_reg <= sm_val_out + 8'd128;
                    sm_axis_sel    <= 1'b1;
                    sm_state       <= SM_LATCH_Y;
                end
            end
            SM_LATCH_Y: begin
                gc_stick_y_reg <= sm_val_out + 8'd128;
                sm_state       <= SM_IDLE;
            end
            default: sm_state <= SM_IDLE;
        endcase
    end

    wire [63:0] gc_state;

    // Latches HIGH the moment a 0x41 ORIGIN request is seen from gc_rx,
    // and stays HIGH for the rest of the session. Drives button_map's
    // origin_unchecked bit (byte 0 [5]) -- per jefflongo.dev, this bit
    // must be cleared starting with (and including) the origin response
    // itself, not just poll responses after it. Setting it the same
    // cycle origin_req strobes covers the origin response too, since
    // gc_state is read combinationally whenever gc_tx actually loads it.
    reg origin_checked = 1'b0;
    always @(posedge clk_core) begin
        if (gc_rx_origin_req)
            origin_checked <= 1'b1;
    end

    button_map mapper (
        .n64_state     (n64_state),
        .dip           (dip[2:0]),
        .gc_stick_x_in (gc_stick_x_reg),
        .gc_stick_y_in (gc_stick_y_reg),
        .origin_checked(origin_checked),
        .gc_state      (gc_state)
    );

    wire       gc_rx_poll_short;
    wire       gc_rx_poll_long;
    wire       gc_rx_info_req;
    wire       gc_rx_origin_req;
    wire [7:0] gc_rx_cmd_out;
    wire       gc_rx_cmd_ready;
    wire       gc_tx_active;

    gc_rx gc_receiver (
        .clk       (clk_core),
        .data_in   (gc_tx_active ? 1'b1 : gc_data_in),
        .poll_short(gc_rx_poll_short),
        .poll_long (gc_rx_poll_long),
        .info_req  (gc_rx_info_req),
        .origin_req(gc_rx_origin_req),
        .cmd_out   (gc_rx_cmd_out),
        .cmd_ready (gc_rx_cmd_ready)
    );

    reg [3:0] gc_byte_idx    = 4'd0;
    reg [3:0] gc_total_bytes = 4'd0;
    reg       gc_pending     = 1'b0;

    reg [7:0] gc_next_byte;
    always @* begin
        case (gc_byte_idx)
            4'd0:    gc_next_byte = (gc_total_bytes == 4'd3) ? 8'h09 : gc_state[63:56];
            4'd1:    gc_next_byte = (gc_total_bytes == 4'd3) ? 8'h00 : gc_state[55:48];
            4'd2:    gc_next_byte = (gc_total_bytes == 4'd3) ? 8'h60 : gc_state[47:40];
            4'd3:    gc_next_byte = gc_state[39:32];
            4'd4:    gc_next_byte = gc_state[31:24];
            4'd5:    gc_next_byte = gc_state[23:16];
            4'd6:    gc_next_byte = gc_state[15:8];
            4'd7:    gc_next_byte = gc_state[7:0];
            default: gc_next_byte = 8'h00;
        endcase
    end

    wire       tx_data_out;
    wire       tx_active;
    wire       tx_done;

    reg        tx_start     = 1'b0;
    reg  [7:0] tx_data      = 8'd0;
    reg        tx_send_stop = 1'b0;
    reg        tx_stop_kind = 1'b0;
    reg        tx_dest      = 1'b0;

    joybus_tx tx_engine (
        .clk      (clk_core),
        .start    (tx_start),
        .data     (tx_data),
        .send_stop(tx_send_stop),
        .stop_kind(tx_stop_kind),
        .data_out (tx_data_out),
        .tx_active(tx_active),
        .byte_done(),
        .tx_done  (tx_done)
    );

    assign n64_tx_active = tx_active && !tx_dest;
    assign gc_tx_active  = tx_active &&  tx_dest;

    assign n64_data_oe  = n64_tx_active;
    assign n64_data_out = tx_data_out;
    assign gc_data_oe   = gc_tx_active;
    assign gc_data_out  = tx_data_out;


    localparam TXARB_IDLE = 1'b0,
               TXARB_WAIT = 1'b1;
    reg txarb_state = TXARB_IDLE;

    wire gc_last_byte = (gc_byte_idx == gc_total_bytes - 4'd1);

    always @(posedge clk_core) begin
        tx_start <= 1'b0;

        if (poll_fire)
            n64_poll_pending <= 1'b1;

        if (!gc_pending) begin
            if (gc_rx_info_req) begin
                gc_total_bytes <= 4'd3;  gc_byte_idx <= 4'd0;  gc_pending <= 1'b1;
            end else if (gc_rx_origin_req) begin
                gc_total_bytes <= 4'd10; gc_byte_idx <= 4'd0;  gc_pending <= 1'b1;
            end else if (gc_rx_poll_short) begin
                // 0x40 is the real GC standard poll command. Its response is
                // 8 bytes (buttons0, buttons1, xAxis, yAxis, cxAxis, cyAxis,
                // left, right) per NicoHood/Nintendo's hardware-validated
                // timing math: 4us * 8 * (3 cmd + 8 resp) = 352us for
                // "0x40, 0x03, 0x00". Previously truncated to 4 bytes here.
                gc_total_bytes <= 4'd8;  gc_byte_idx <= 4'd0;  gc_pending <= 1'b1;
            end else if (gc_rx_poll_long) begin
                gc_total_bytes <= 4'd8;  gc_byte_idx <= 4'd0;  gc_pending <= 1'b1;
            end
        end

        case (txarb_state)
            TXARB_IDLE: begin
                if (n64_poll_pending) begin
                    tx_dest      <= 1'b0;
                    tx_data      <= 8'h01;
                    tx_send_stop <= 1'b1;
                    tx_stop_kind <= 1'b0;
                    tx_start     <= 1'b1;
                    txarb_state  <= TXARB_WAIT;
                end else if (gc_pending) begin
                    tx_dest      <= 1'b1;
                    tx_data      <= gc_next_byte;
                    tx_send_stop <= gc_last_byte;
                    tx_stop_kind <= 1'b1;
                    tx_start     <= 1'b1;
                    txarb_state  <= TXARB_WAIT;
                end
            end

            TXARB_WAIT: begin
                if (tx_done) begin
                    if (!tx_dest) begin
                        n64_poll_pending <= 1'b0;
                        txarb_state      <= TXARB_IDLE;
                    end else if (gc_last_byte) begin
                        gc_pending  <= 1'b0;
                        txarb_state <= TXARB_IDLE;
                    end else begin
                        gc_byte_idx <= gc_byte_idx + 4'd1;
                        txarb_state <= TXARB_IDLE;
                    end
                end
            end

            default: txarb_state <= TXARB_IDLE;
        endcase
    end

    // =========================================================================
    // Debug taps — combinational, no effect on core logic or timing.
    // =========================================================================
    assign dbg_gc_tx_active    = gc_tx_active;
    assign dbg_n64_pkt_ready   = n64_pkt_ready;
    assign dbg_gc_cmd_ready    = gc_rx_cmd_ready;
    assign dbg_n64_poll_pending = n64_poll_pending;
    assign dbg_clk_core        = clk_core;

endmodule
