`timescale 1ns / 1ps

// =============================================================================
// joybus_tx.v — Shared JoyBus Bit-Level Transmitter Engine
// =============================================================================
//
// Shared bit-level transmitter for both the N64 and GameCube directions.
// n64_tx (CPLD -> real N64 controller, polling it) and gc_tx (CPLD -> Wii
// GC port, responding to it) never transmit at the same time, so they
// time-share this one engine instead of each paying for their own copy.
// Byte sequencing (which byte comes next, total byte count per response
// type) lives in top.v, since that's genuinely direction-specific; this
// module only handles one byte (plus an optional trailing stop bit) per
// `start` pulse.
//
// BIT ENCODING (JoyBus, both directions): a data bit is either
//   logic 1: SHORT low + LONG high   (1us + 3us at 22MHz)
//   logic 0: LONG low + SHORT high   (3us + 1us at 22MHz)
// `cur_bit` tracks which shape the bit currently being sent uses.
//
// STOP BIT: only the N64 and GC directions differ here, and only in the
// HIGH phase -- both use a SHORT (1us) low phase:
//   N64 (stop_kind=0): SHORT low + MID high  (1us + 2us) -- distinctly
//     shaped, not an ordinary data bit. Matches n64brew wiki's N64-specific
//     framing.
//   GC  (stop_kind=1): SHORT low + LONG high (1us + 3us) -- an ORDINARY
//     "1" bit, not a unique waveform. Per jefflongo.dev's GC controller
//     reverse-engineering writeup, a single such bit terminates the whole
//     multi-byte response (nothing sent between non-final bytes). This
//     matters beyond just protocol correctness: gc_rx.v classifies bits
//     against a 2us threshold, so a stop bit shaped MID/MID (2us low)
//     would sit exactly on that boundary -- ambiguous by construction.
//
// low_ticks/high_ticks are derived combinationally from `state` (are we in
// a stop bit or not) + cur_bit/stop_kind_latched, rather than latched into
// their own registers -- saves the two 8-bit tick registers a byte-by-byte
// FSM would otherwise need, in exchange for a small mux. Together with
// phase_count narrowed to 7 bits (max needed value is LONG-1=65) and
// cur_bit replacing separate low/high registers, this keeps the shared
// engine's footprint small against a tight LC budget.
// =============================================================================

module joybus_tx (
    input  wire       clk,        // clk_core, ~22MHz
    input  wire       start,      // one-cycle strobe: begin transmitting `data`
    input  wire [7:0] data,       // byte to transmit, MSB first
    input  wire       send_stop,  // 1: append a stop bit after this byte
                                   // 0: go idle immediately after the byte's
                                   //    last bit (used between non-final
                                   //    bytes of a multi-byte GC response)
    input  wire       stop_kind,  // stop bit shape when send_stop=1:
                                   // 0 = N64-style (SHORT low + MID high)
                                   // 1 = GC-style  (SHORT low + LONG high)
                                   // sampled at LOAD, ignored if send_stop=0
    output reg        data_out,   // 0=drive LOW, 1=release (caller applies OE)
    output reg        tx_active,  // HIGH for the whole operation
    output reg        byte_done,  // one-cycle strobe: last DATA bit sent,
                                   // fires BEFORE any stop bit
    output reg        tx_done     // one-cycle strobe: fully done (byte,
                                   // plus stop bit if send_stop was set)
);

    parameter SHORT = 7'd22;   // 1us at 22MHz
    parameter LONG  = 7'd66;   // 3us at 22MHz
    parameter MID   = 7'd44;   // 2us at 22MHz

    localparam IDLE      = 3'd0;
    localparam LOAD      = 3'd1;
    localparam BIT_LOW   = 3'd2;
    localparam BIT_HIGH  = 3'd3;
    localparam STOP_LOW  = 3'd4;
    localparam STOP_HIGH = 3'd5;
    localparam DONE      = 3'd6;

    reg [2:0] state = IDLE;

    reg [7:0] shift_reg   = 8'd0;
    reg [2:0] bit_count   = 3'd0;
    reg [6:0] phase_count = 7'd0;    // narrowed 8->7 (max needed: LONG-1=65)

    reg cur_bit = 1'b0;              // 1 = SHORT-low/LONG-high, 0 = LONG-low/SHORT-high
                                      // (replaces separate low_ticks/high_ticks regs)
    reg send_stop_latched = 1'b0;
    reg stop_kind_latched = 1'b0;

    // Combinationally derived -- state already tells us whether we're in a
    // stop bit; cur_bit/stop_kind_latched are already registers, so no new
    // latching needed, just a mux. Stop bit's low phase is SHORT either
    // way (see header) -- only the high phase depends on stop_kind.
    wire in_stop = (state == STOP_LOW) || (state == STOP_HIGH);
    wire [6:0] low_ticks  = in_stop ? SHORT
                                     : (cur_bit ? SHORT : LONG);
    wire [6:0] high_ticks = in_stop ? (stop_kind_latched ? LONG : MID)
                                     : (cur_bit ? LONG : SHORT);

    always @(posedge clk) begin
        byte_done <= 1'b0;
        tx_done   <= 1'b0;

        case (state)

            // -----------------------------------------------------------------
            IDLE: begin
                data_out  <= 1'b1;
                tx_active <= 1'b0;
                if (start) state <= LOAD;
            end

            // -----------------------------------------------------------------
            // LOAD: latch data + this operation's stop-bit request, set up
            // timing for bit 7 (MSB), begin transmission.
            // -----------------------------------------------------------------
            LOAD: begin
                shift_reg         <= data;
                bit_count         <= 3'd0;
                phase_count       <= 7'd0;
                tx_active         <= 1'b1;
                send_stop_latched <= send_stop;
                stop_kind_latched <= stop_kind;
                cur_bit           <= data[7];

                data_out <= 1'b0;
                state    <= BIT_LOW;
            end

            // -----------------------------------------------------------------
            BIT_LOW: begin
                data_out <= 1'b0;
                if (phase_count < low_ticks - 7'd1) begin
                    phase_count <= phase_count + 7'd1;
                end else begin
                    phase_count <= 7'd0;
                    data_out    <= 1'b1;
                    state       <= BIT_HIGH;
                end
            end

            // -----------------------------------------------------------------
            BIT_HIGH: begin
                data_out <= 1'b1;
                if (phase_count < high_ticks - 7'd1) begin
                    phase_count <= phase_count + 7'd1;
                end else begin
                    phase_count <= 7'd0;

                    if (bit_count == 3'd7) begin
                        bit_count <= 3'd0;
                        byte_done <= 1'b1;

                        if (send_stop_latched) begin
                            // low_ticks/high_ticks for the stop bit are
                            // derived live from state+stop_kind_latched --
                            // nothing to latch here.
                            data_out <= 1'b0;
                            state    <= STOP_LOW;
                        end else begin
                            tx_done   <= 1'b1;
                            tx_active <= 1'b0;
                            state     <= DONE;
                        end
                    end else begin
                        bit_count <= bit_count + 3'd1;
                        shift_reg <= {shift_reg[6:0], 1'b0};
                        cur_bit   <= shift_reg[6];

                        data_out <= 1'b0;
                        state    <= BIT_LOW;
                    end
                end
            end

            // -----------------------------------------------------------------
            STOP_LOW: begin
                data_out <= 1'b0;
                if (phase_count < low_ticks - 7'd1) begin
                    phase_count <= phase_count + 7'd1;
                end else begin
                    phase_count <= 7'd0;
                    data_out    <= 1'b1;
                    state       <= STOP_HIGH;
                end
            end

            STOP_HIGH: begin
                data_out <= 1'b1;
                if (phase_count < high_ticks - 7'd1) begin
                    phase_count <= phase_count + 7'd1;
                end else begin
                    phase_count <= 7'd0;
                    tx_done     <= 1'b1;
                    tx_active   <= 1'b0;
                    state       <= DONE;
                end
            end

            // -----------------------------------------------------------------
            DONE: begin
                data_out  <= 1'b1;
                tx_active <= 1'b0;
                if (start) state <= LOAD;
            end

            default: state <= IDLE;
        endcase
    end

endmodule
