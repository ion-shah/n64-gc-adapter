// =============================================================================
// top_hw.v — Hardware top-level wrapper (adds tristate I/O + debug LEDs)
// =============================================================================
//
// This hardware wrapper is built around the UnoProLogic CPLD board
// which has a 66MHz oscillator, onboard RGB LEDs, and PMOD connectors for the GPIO
// The dev board specs and data sheet are found at https://earthpeopletechnology.com
//
// UnoProLogic onboard RGB LEDs are current-sink, active-LOW:
//   assert 0 on the pin -> LED segment lit
//   assert 1 (or Hi-Z)  -> LED segment off
// (see UNO_USB_CPLD_DEV_SYS_DS_V44.pdf section 7, LEDs)
//
// Full pin chart (silkscreen -> CPLD pin):
//   D1: RED=55(LED_RD_1_N)  BLUE=56(LED_BL_1_N)  GREEN=57(LED_GR_1_N)
//   D2: RED=52(LED_RD_2_N)  BLUE=53(LED_BL_2_N)  GREEN=54(LED_GR_2_N)
//   D3: RED=49(LED_RD_3_N)  BLUE=50(LED_BL_3_N)  GREEN=51(LED_GR_3_N)
//   D4: RED=43(LED_RD_4_N)  BLUE=47(LED_BL_4_N)  GREEN=48(LED_GR_43_N)
//
// Organized by subsystem, one LED per subsystem:
//  D1 = POWER    -> hardwired "alive" indicator
//  D2 = GAMECUBE -> RED=dbg_gc_tx_active, GREEN=dbg_gc_cmd_ready
//  D3 = N64      -> BLUE=dbg_n64_poll_pending, GREEN=dbg_n64_pkt_ready
//  D4 = OFF      -> not wired to anything yet, all three channels held off
//
// dbg_clk_core_pin (PIN_84) -> direct copy of clk_core, for probing the
//                               divide-by-3 output with the logic analyzer.
//                               NOT gated by anything -- always toggling.
//
// All of these are pure combinational taps (see top.v's dbg_* assigns) —
// they do not affect core timing or LC count in any meaningful way.
// =============================================================================

module top_hw (
    input  wire       clk,
    inout  wire       n64_data,
    inout  wire       gc_data,
    input  wire [4:0] dip,

    output wire       led1_r,   // PIN_55 (off)
    output wire       led1_g,   // PIN_57 (POWER: always on when configured)
    output wire       led1_b,   // PIN_56 (off)
    output wire       led2_r,   // PIN_52 (GC: dbg_gc_tx_active)
    output wire       led2_g,   // PIN_54 (GC: dbg_gc_cmd_ready)
    output wire       led2_b,   // PIN_53 (off)
    output wire       led3_r,   // PIN_49 (off)
    output wire       led3_g,   // PIN_51 (N64: dbg_n64_pkt_ready)
    output wire       led3_b,   // PIN_50 (N64: dbg_n64_poll_pending)
    output wire       led4_r,   // PIN_43 (off, unused)
    output wire       led4_g,   // PIN_48 (off, unused)
    output wire       led4_b,   // PIN_47 (off, unused)
    output wire       dbg_clk_core_pin // PIN_84
);
    wire n64_data_in, n64_data_out, n64_data_oe;
    wire gc_data_in, gc_data_out, gc_data_oe;

    assign n64_data    = n64_data_oe ? n64_data_out : 1'bz;
    assign n64_data_in = n64_data;
    assign gc_data     = gc_data_oe ? gc_data_out : 1'bz;
    assign gc_data_in  = gc_data;

    wire dbg_gc_tx_active;
    wire dbg_n64_pkt_ready;
    wire dbg_gc_cmd_ready;
    wire dbg_n64_poll_pending;
    wire dbg_clk_core;

    top core (
        .clk        (clk),
        .n64_data_in (n64_data_in), .n64_data_out (n64_data_out), .n64_data_oe (n64_data_oe),
        .gc_data_in  (gc_data_in),  .gc_data_out  (gc_data_out),  .gc_data_oe  (gc_data_oe),
        .dip        (dip),

        .dbg_gc_tx_active    (dbg_gc_tx_active),
        .dbg_n64_pkt_ready   (dbg_n64_pkt_ready),
        .dbg_gc_cmd_ready    (dbg_gc_cmd_ready),
        .dbg_n64_poll_pending(dbg_n64_poll_pending),
        .dbg_clk_core        (dbg_clk_core)
    );

    // Active-low drive: signal=1 -> pin=0 -> LED lit

    // D1 = POWER -- hardwired, no protocol logic dependency
    assign led1_r = 1'b1;  // off
    assign led1_g = 1'b0;  // always on: "CPLD powered + configured" indicator
    assign led1_b = 1'b1;  // off

    // D2 = GAMECUBE
    assign led2_r = dbg_gc_tx_active ? 1'b0 : 1'b1;
    assign led2_g = dbg_gc_cmd_ready ? 1'b0 : 1'b1;
    assign led2_b = 1'b1;  // off

    // D3 = N64
    assign led3_r = 1'b1;  // off
    assign led3_g = dbg_n64_pkt_ready    ? 1'b0 : 1'b1;
    assign led3_b = dbg_n64_poll_pending ? 1'b0 : 1'b1;

    // D4 = OFF -- unused, all channels held off
    assign led4_r = 1'b1;
    assign led4_g = 1'b1;
    assign led4_b = 1'b1;

    assign dbg_clk_core_pin = dbg_clk_core;

endmodule
