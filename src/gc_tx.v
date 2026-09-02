`timescale 1ns / 1ps
// =============================================================================
// gc_tx.v — GameCube JoyBus Transmitter (byte-at-a-time architecture)
// =============================================================================
// Reference: https://n64brew.dev/wiki/Joybus_Protocol
//
// ARCHITECTURE CHANGE vs previous version:
//   OLD: 80-bit shift register loaded with full response upfront (80+ FFs)
//   NEW: 8-bit shift register + 4-bit byte counter (12 FFs for data path)
//        Combinational mux selects next byte from gc_state based on byte_index.
//        No performance difference — wire timing is identical.
//        Saves ~82 flip-flops.
//
// RESPONSE TYPES:
//   send_info   (0x00/0xFF) → 3 bytes:  0x09, 0x00, 0x03
//   send_origin (0x41)      → 10 bytes: gc_state bytes 0-7 + 0x00, 0x00
//   send_short  (0x40)      → 4 bytes:  gc_state bytes 0-3
//   send_long   (0x43)      → 8 bytes:  gc_state bytes 0-7
//
// CONTROLLER STOP BIT: 2µs low + 2µs high (MID=100 ticks each)
//
// REGISTER COUNT (for LE estimation):
//   state[2:0]        3
//   shift_reg[7:0]    8   ← was 80, now 8
//   byte_index[3:0]   4   ← was total_bits[6:0]=7 + bit_count[6:0]=7 = 14
//   bit_count[2:0]    3   ← bits within current byte (0-7)
//   phase_count[7:0]  8
//   low_ticks[7:0]    8
//   high_ticks[7:0]   8
//   data_out          1
//   tx_active         1
//   tx_done           1
//   total_bytes[3:0]  4   ← number of bytes to send (max 10)
//   TOTAL:           49 FFs  (was 124 FFs — saving 75 FFs)
// =============================================================================

module gc_tx (
    input  wire        clk,
    input  wire        send_info,    // one-cycle strobe: respond to 0x00/0xFF
    input  wire        send_origin,  // one-cycle strobe: respond to 0x41
    input  wire        send_short,   // one-cycle strobe: respond to 0x40
    input  wire        send_long,    // one-cycle strobe: respond to 0x43
    input  wire [63:0] gc_state,     // pre-translated GC controller state
    output reg         data_out,     // 0=drive LOW, 1=release (tristate in top.v)
    output reg         tx_active,    // HIGH throughout entire transmission
    output reg         tx_done       // one-cycle strobe: transmission complete
);

    // -------------------------------------------------------------------------
    // Timing parameters
    // -------------------------------------------------------------------------
    parameter SHORT = 8'd50;   // 1µs — logic-1 low phase
    parameter LONG  = 8'd150;  // 3µs — logic-0 low phase / logic-1 high phase
    parameter MID   = 8'd100;  // 2µs — controller stop bit (both phases)

    // -------------------------------------------------------------------------
    // FSM states
    // -------------------------------------------------------------------------
    localparam IDLE      = 3'd0;
    localparam LOAD_BYTE = 3'd1;  // load next byte into shift_reg, pull LOW
    localparam BIT_LOW   = 3'd2;
    localparam BIT_HIGH  = 3'd3;
    localparam STOP_LOW  = 3'd4;
    localparam STOP_HIGH = 3'd5;
    localparam DONE      = 3'd6;

    reg [2:0] state = IDLE;

    // -------------------------------------------------------------------------
    // Data path registers — small and efficient
    // -------------------------------------------------------------------------
    reg [7:0] shift_reg   = 8'd0;  // current byte being transmitted
    reg [3:0] byte_index  = 4'd0;  // which byte we're on (0..total_bytes-1)
    reg [3:0] total_bytes = 4'd0;  // how many bytes to send (3/4/8/10)
    reg [2:0] bit_count   = 3'd0;  // bit within current byte (0..7)
    reg [7:0] phase_count = 8'd0;
    reg [7:0] low_ticks   = 8'd0;
    reg [7:0] high_ticks  = 8'd0;

    // -------------------------------------------------------------------------
    // Byte source — combinational mux selecting the correct byte to send
    // Based on current byte_index and response type (encoded in total_bytes).
    //
    // gc_state layout (from button_map.v):
    //   [63:56] byte 0 — buttons A/B/X/Y/Start + origin_unchecked
    //   [55:48] byte 1 — High1 + L/R/Z + D-pad
    //   [47:40] byte 2 — main stick X
    //   [39:32] byte 3 — main stick Y
    //   [31:24] byte 4 — C-stick X
    //   [23:16] byte 5 — C-stick Y
    //   [15:8]  byte 6 — L analog
    //   [7:0]   byte 7 — R analog
    //
    // Info bytes are hardcoded constants.
    // Origin appends 0x00, 0x00 as bytes 8 and 9 (deadzone).
    // -------------------------------------------------------------------------
    reg [7:0] next_byte;

    always @(*) begin
        case (byte_index)
            4'd0: begin
                // For info responses, override with hardcoded bytes
                if (total_bytes == 4'd3)
                    next_byte = 8'h09;         // info byte 0
                else
                    next_byte = gc_state[63:56]; // controller byte 0
            end
            4'd1: begin
                if (total_bytes == 4'd3)
                    next_byte = 8'h00;         // info byte 1
                else
                    next_byte = gc_state[55:48];
            end
            4'd2: begin
                if (total_bytes == 4'd3)
                    next_byte = 8'h03;         // info byte 2
                else
                    next_byte = gc_state[47:40];
            end
            4'd3:  next_byte = gc_state[39:32];
            4'd4:  next_byte = gc_state[31:24];
            4'd5:  next_byte = gc_state[23:16];
            4'd6:  next_byte = gc_state[15:8];
            4'd7:  next_byte = gc_state[7:0];
            4'd8:  next_byte = 8'h00;          // origin deadzone byte 0
            4'd9:  next_byte = 8'h00;          // origin deadzone byte 1
            default: next_byte = 8'h00;
        endcase
    end

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        tx_done <= 1'b0;

        case (state)

            IDLE: begin
                data_out  <= 1'b1;
                tx_active <= 1'b0;

                if (send_info) begin
                    total_bytes <= 4'd3;
                    byte_index  <= 4'd0;
                    bit_count   <= 3'd0;
                    state       <= LOAD_BYTE;
                end else if (send_origin) begin
                    total_bytes <= 4'd10;
                    byte_index  <= 4'd0;
                    bit_count   <= 3'd0;
                    state       <= LOAD_BYTE;
                end else if (send_short) begin
                    total_bytes <= 4'd4;
                    byte_index  <= 4'd0;
                    bit_count   <= 3'd0;
                    state       <= LOAD_BYTE;
                end else if (send_long) begin
                    total_bytes <= 4'd8;
                    byte_index  <= 4'd0;
                    bit_count   <= 3'd0;
                    state       <= LOAD_BYTE;
                end
            end

            // -----------------------------------------------------------------
            // LOAD_BYTE: fetch next_byte combinationally, load into shift_reg,
            // compute timing for bit 7 (MSB), pull line LOW.
            // -----------------------------------------------------------------
            LOAD_BYTE: begin
                tx_active   <= 1'b1;
                phase_count <= 8'd0;
                shift_reg   <= next_byte;

                // Pre-compute timing for MSB of incoming byte
                if (next_byte[7]) begin
                    low_ticks  <= SHORT;
                    high_ticks <= LONG;
                end else begin
                    low_ticks  <= LONG;
                    high_ticks <= SHORT;
                end

                data_out <= 1'b0;
                state    <= BIT_LOW;
            end

            // -----------------------------------------------------------------
            // BIT_LOW: hold LOW for low_ticks cycles
            // -----------------------------------------------------------------
            BIT_LOW: begin
                data_out <= 1'b0;
                if (phase_count < low_ticks - 8'd1) begin
                    phase_count <= phase_count + 8'd1;
                end else begin
                    phase_count <= 8'd0;
                    data_out    <= 1'b1;
                    state       <= BIT_HIGH;
                end
            end

            // -----------------------------------------------------------------
            // BIT_HIGH: hold HIGH, then advance to next bit or next byte
            // -----------------------------------------------------------------
            BIT_HIGH: begin
                data_out <= 1'b1;
                if (phase_count < high_ticks - 8'd1) begin
                    phase_count <= phase_count + 8'd1;
                end else begin
                    phase_count <= 8'd0;

                    if (bit_count == 3'd7) begin
                        // Last bit of this byte — move to next byte or stop bit
                        bit_count <= 3'd0;
                        if (byte_index == total_bytes - 4'd1) begin
                            // All bytes sent — controller stop bit
                            data_out <= 1'b0;
                            state    <= STOP_LOW;
                        end else begin
                            // Load next byte
                            byte_index <= byte_index + 4'd1;
                            state      <= LOAD_BYTE;
                            // data_out stays HIGH until LOAD_BYTE pulls it LOW
                        end
                    end else begin
                        // More bits in this byte — shift and continue
                        bit_count <= bit_count + 3'd1;
                        shift_reg <= {shift_reg[6:0], 1'b0};

                        // Pre-compute timing for next bit (shift_reg[6] = new MSB)
                        if (shift_reg[6]) begin
                            low_ticks  <= SHORT;
                            high_ticks <= LONG;
                        end else begin
                            low_ticks  <= LONG;
                            high_ticks <= SHORT;
                        end

                        data_out <= 1'b0;
                        state    <= BIT_LOW;
                    end
                end
            end

            // -----------------------------------------------------------------
            // STOP_LOW / STOP_HIGH: controller stop bit (2µs each)
            // -----------------------------------------------------------------
            STOP_LOW: begin
                data_out <= 1'b0;
                if (phase_count < MID - 8'd1) begin
                    phase_count <= phase_count + 8'd1;
                end else begin
                    phase_count <= 8'd0;
                    data_out    <= 1'b1;
                    state       <= STOP_HIGH;
                end
            end

            STOP_HIGH: begin
                data_out <= 1'b1;
                if (phase_count < MID - 8'd1) begin
                    phase_count <= phase_count + 8'd1;
                end else begin
                    phase_count <= 8'd0;
                    tx_done     <= 1'b1;
                    tx_active   <= 1'b0;
                    state       <= DONE;
                end
            end

            // -----------------------------------------------------------------
            // DONE: hold until next send strobe
            // -----------------------------------------------------------------
            DONE: begin
                data_out  <= 1'b1;
                tx_active <= 1'b0;

                if (send_info) begin
                    total_bytes <= 4'd3;
                    byte_index  <= 4'd0;
                    bit_count   <= 3'd0;
                    state       <= LOAD_BYTE;
                end else if (send_origin) begin
                    total_bytes <= 4'd10;
                    byte_index  <= 4'd0;
                    bit_count   <= 3'd0;
                    state       <= LOAD_BYTE;
                end else if (send_short) begin
                    total_bytes <= 4'd4;
                    byte_index  <= 4'd0;
                    bit_count   <= 3'd0;
                    state       <= LOAD_BYTE;
                end else if (send_long) begin
                    total_bytes <= 4'd8;
                    byte_index  <= 4'd0;
                    bit_count   <= 3'd0;
                    state       <= LOAD_BYTE;
                end
            end

            default: state <= IDLE;
        endcase
    end

endmodule