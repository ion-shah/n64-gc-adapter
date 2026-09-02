`timescale 1ns / 1ps
// =============================================================================
// n64_tx.v — N64 JoyBus Transmitter
// =============================================================================
// Reference: https://n64brew.dev/wiki/Joybus_Protocol
//
// Transmits a single JoyBus command byte to the N64 controller, followed by
// the console stop bit. Used by top.v to send the 0x01 (Controller State)
// poll command once per frame.
//
// PROTOCOL — Console-side bit encoding at 50MHz (20ns/tick):
//
//   Logic 1:  ___
//            |   |_____________
//            1us      3us        → 50 ticks low, 150 ticks high
//
//   Logic 0:  _____________
//            |             |___
//                 3us       1us  → 150 ticks low, 50 ticks high
//
//   Console Stop Bit:  ___
//                     |   |______
//                      1us  2us  → 50 ticks low, 100 ticks high (3us total)
//
//   NOTE: Console stop bit is 3us total (1us low + 2us high).
//   This is DIFFERENT from the controller stop bit (4us total, 3us low).
//   The console stop bit always follows the command byte.
//
// OPEN-DRAIN BEHAVIOR:
//   JoyBus is open-drain — the line is never actively driven HIGH.
//   data_out=0 means drive the pin LOW.
//   data_out=1 means release the pin (tristate) — pull-up holds it HIGH.
//   The tristate assignment belongs in top.v:
//     assign n64_data = tx_active ? data_out : 1'bz;
//   While tx_active=0, top.v should tristate so n64_rx can hear the controller.
//
// INTERFACE:
//   send     — one-cycle strobe from top.v: begin transmitting cmd
//   cmd      — byte to transmit (will be 8'h01 for Controller State poll)
//   data_out — drive LOW or release (see open-drain note above)
//   tx_active — HIGH for entire duration of transmission including stop bit
//   tx_done  — one-cycle strobe when stop bit completes
//              module holds in DONE state until next send strobe
//
// TIMING PARAMETERS (update if clock frequency changes):
//   SHORT = 50 ticks = 1us  (logic-1 low phase, stop bit low phase)
//   LONG  = 150 ticks = 3us (logic-0 low phase, logic-1 high phase)
//   MID   = 100 ticks = 2us (console stop bit high phase)
// =============================================================================

module n64_tx (
    input  wire       clk,       // 50MHz — 20ns per tick
    input  wire       send,      // one-cycle strobe: begin transmission
    input  wire [7:0] cmd,       // command byte to send (e.g. 8'h01)
    output reg        data_out,  // 0=drive LOW, 1=release (tristate in top.v)
    output reg        tx_active, // HIGH throughout transmission
    output reg        tx_done    // one-cycle strobe: transmission complete
);

    // -------------------------------------------------------------------------
    // Timing parameters — all in 50MHz clock ticks (20ns each)
    // -------------------------------------------------------------------------
    parameter SHORT = 8'd50;    // 1us — logic-1 low, stop bit low
    parameter LONG  = 8'd150;   // 3us — logic-0 low, logic-1 high
    parameter MID   = 8'd100;   // 2us — console stop bit high phase

    // -------------------------------------------------------------------------
    // FSM states
    // -------------------------------------------------------------------------
    localparam IDLE     = 3'd0;  // waiting for send strobe
    localparam LOAD     = 3'd1;  // latch cmd into shift register, one cycle
    localparam BIT_LOW  = 3'd2;  // driving line LOW for current bit
    localparam BIT_HIGH = 3'd3;  // releasing line HIGH for current bit
    localparam STOP_LOW = 3'd4;  // console stop bit: 1us low
    localparam STOP_HIGH= 3'd5;  // console stop bit: 2us high
    localparam DONE     = 3'd6;  // tx_done pulsed, hold until next send

    reg [2:0] state = IDLE;

    // -------------------------------------------------------------------------
    // Shift register and bit counter
    // -------------------------------------------------------------------------
    reg [7:0] shift_reg  = 8'd0; // command byte, shifts MSB first
    reg [2:0] bit_count  = 3'd0; // counts 0–7 (8 bits per byte)

    // -------------------------------------------------------------------------
    // Phase counter — measures ticks within the current low or high phase
    // -------------------------------------------------------------------------
    reg [7:0] phase_count = 8'd0;

    // -------------------------------------------------------------------------
    // Low-phase duration for current bit
    // Set in LOAD/BIT_HIGH based on the next bit to send.
    // -------------------------------------------------------------------------
    reg [7:0] low_ticks  = 8'd0;  // SHORT(50) for bit=1, LONG(150) for bit=0
    reg [7:0] high_ticks = 8'd0;  // LONG(150) for bit=1, SHORT(50) for bit=0

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    always @(posedge clk) begin

        // Default: tx_done is a one-cycle strobe
        tx_done <= 1'b0;

        case (state)

            // -----------------------------------------------------------------
            IDLE: begin
                data_out  <= 1'b1;   // release line (tristate via top.v)
                tx_active <= 1'b0;
                if (send) begin
                    state <= LOAD;
                end
            end

            // -----------------------------------------------------------------
            // LOAD: latch cmd, set up timing for bit 7 (MSB), begin transmission
            // -----------------------------------------------------------------
            LOAD: begin
                shift_reg   <= cmd;
                bit_count   <= 3'd0;
                phase_count <= 8'd0;
                tx_active   <= 1'b1;

                // Pre-compute timing for the first bit (MSB of cmd)
                // cmd[7] is the MSB — checked here since shift hasn't happened yet
                if (cmd[7]) begin
                    low_ticks  <= SHORT;   // logic-1: 1us low
                    high_ticks <= LONG;    // logic-1: 3us high
                end else begin
                    low_ticks  <= LONG;    // logic-0: 3us low
                    high_ticks <= SHORT;   // logic-0: 1us high
                end

                data_out <= 1'b0;    // pull line LOW — start of first bit
                state    <= BIT_LOW;
            end

            // -----------------------------------------------------------------
            // BIT_LOW: hold line LOW for low_ticks cycles
            // -----------------------------------------------------------------
            BIT_LOW: begin
                data_out <= 1'b0;
                if (phase_count < low_ticks - 8'd1) begin
                    phase_count <= phase_count + 8'd1;
                end else begin
                    // Low phase done — release line HIGH
                    phase_count <= 8'd0;
                    data_out    <= 1'b1;
                    state       <= BIT_HIGH;
                end
            end

            // -----------------------------------------------------------------
            // BIT_HIGH: hold line HIGH for high_ticks cycles, then next bit
            // -----------------------------------------------------------------
            BIT_HIGH: begin
                data_out <= 1'b1;
                if (phase_count < high_ticks - 8'd1) begin
                    phase_count <= phase_count + 8'd1;
                end else begin
                    phase_count <= 8'd0;
                    bit_count   <= bit_count + 3'd1;

                    if (bit_count == 3'd7) begin
                        // All 8 bits sent — move to console stop bit
                        data_out <= 1'b0;      // stop bit low phase starts
                        state    <= STOP_LOW;
                    end else begin
                        // Shift register: advance to next bit
                        shift_reg <= {shift_reg[6:0], 1'b0};

                        // Pre-compute timing for next bit
                        if (shift_reg[6]) begin   // [6] will be new MSB after shift
                            low_ticks  <= SHORT;
                            high_ticks <= LONG;
                        end else begin
                            low_ticks  <= LONG;
                            high_ticks <= SHORT;
                        end

                        data_out <= 1'b0;   // pull LOW for next bit
                        state    <= BIT_LOW;
                    end
                end
            end

            // -----------------------------------------------------------------
            // STOP_LOW: console stop bit — 1us low (SHORT = 50 ticks)
            // -----------------------------------------------------------------
            STOP_LOW: begin
                data_out <= 1'b0;
                if (phase_count < SHORT - 8'd1) begin
                    phase_count <= phase_count + 8'd1;
                end else begin
                    phase_count <= 8'd0;
                    data_out    <= 1'b1;   // release line
                    state       <= STOP_HIGH;
                end
            end

            // -----------------------------------------------------------------
            // STOP_HIGH: console stop bit — 2us high (MID = 100 ticks)
            // -----------------------------------------------------------------
            STOP_HIGH: begin
                data_out <= 1'b1;
                if (phase_count < MID - 8'd1) begin
                    phase_count <= phase_count + 8'd1;
                end else begin
                    // Transmission complete
                    phase_count <= 8'd0;
                    tx_done     <= 1'b1;   // one-cycle strobe
                    tx_active   <= 1'b0;
                    state       <= DONE;
                end
            end

            // -----------------------------------------------------------------
            // DONE: hold here until top.v sends next poll strobe
            // -----------------------------------------------------------------
            DONE: begin
                data_out  <= 1'b1;   // release line
                tx_active <= 1'b0;
                if (send) begin
                    state <= LOAD;
                end
            end

            default: state <= IDLE;

        endcase
    end

endmodule
