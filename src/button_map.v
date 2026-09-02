`timescale 1ns / 1ps
// =============================================================================
// button_map.v — N64 to GameCube Button and Axis Mapping
// =============================================================================
// Reference: https://www.raphnet-tech.com/support/mappings/n64_to_wii_v2_mapping.php
//
// Pure combinational module — no clock, no state, no FSM.
// Instantiated by top.v. Updates gc_state instantly when n64_state changes.
//
// INPUTS:
//   n64_state     [31:0] — latched N64 controller packet from n64_rx
//   dip           [2:0]  — 3-bit mapping selection from DIP switch
//                          (bits 0-2 of the 5-position DIP switch)
//   gc_stick_x_in [7:0]  — main stick X, already corrected + offset to
//                          unsigned GC range (center=0x80). Computed once
//                          per N64 packet by stick_map.v in top.v and
//                          passed straight through here — button_map does
//                          no stick math of its own.
//   gc_stick_y_in [7:0]  — main stick Y, same as above.
//   origin_checked       — 0 until the CPLD has serviced a 0x41 ORIGIN
//                          request at least once, then latched 1 for the
//                          rest of the session (see top.v). Drives byte 0
//                          bit [5] below.
//
// OUTPUT:
//   gc_state [63:0]  — formatted GC controller response for gc_tx
//
// N64 PACKET BIT LAYOUT:
//   [31]=A   [30]=B   [29]=Z   [28]=Start
//   [27]=D-Up  [26]=D-Down  [25]=D-Left  [24]=D-Right
//   [23]=RST (L+R+Start combo — not mapped, controller handles it internally)
//   [22]=Reserved (always 0 — not mapped)
//   [21]=L   [20]=R
//   [19]=C-Up  [18]=C-Down  [17]=C-Left  [16]=C-Right
//   [15:8]=Analog X (signed 8-bit, center=0, range ~±80)
//   [7:0] =Analog Y (signed 8-bit, center=0, range ~±80)
//
// GC STATE BYTE LAYOUT:
//   [63:56] Byte 0: [7:5]=000 [4]=Start [3]=Y [2]=X [1]=B [0]=A
//   [55:48] Byte 1: [7]=0 [6]=L_dig [5]=R_dig [4]=Z [3]=D-Up [2]=D-Down
//                   [1]=D-Right [0]=D-Left
//   [47:40] Byte 2: Main stick X (unsigned, center=128)
//   [39:32] Byte 3: Main stick Y (unsigned, center=128)
//   [31:24] Byte 4: C-stick X   (unsigned, center=128)
//   [23:16] Byte 5: C-stick Y   (unsigned, center=128)
//   [15:8]  Byte 6: L trigger analog (0x00=released, 0xFF=full press)
//   [7:0]   Byte 7: R trigger analog (0x00=released, 0xFF=full press)
//
// MAPPING MODES (dip[2:0]):
// ┌─────┬──────────────┬───────┬───────┬───────┬───────┬───────┬─────────────────────┐
// │ DIP │ Name         │ GC Z  │ GC L  │ GC R  │ GC X  │ GC Y  │ C-stick uses        │
// ├─────┼──────────────┼───────┼───────┼───────┼───────┼───────┼─────────────────────┤
// │ 000 │ Default      │ N64 L │ N64 Z │ N64 R │  —    │  —    │ C-Up/Dn/L/R         │
// │ 001 │ L mapping    │ N64 Z │ N64 L │ N64 R │  —    │  —    │ C-Up/Dn/L/R         │
// │ 010 │ R mapping    │ N64 L │ N64 Z │ N64 R │ C-Rt  │ C-Lt  │ C-Up/Dn only        │
// │ 011 │ L+R mapping  │ N64 Z │ N64 L │ N64 R │ C-Rt  │ C-Lt  │ C-Up/Dn only        │
// │ 100 │ C-Up mapping │ N64 R │ N64 Z │ N64 L │ C-Dn  │ C-Lt  │ C-Up/Rt only        │
// │ 101 │ C-Dn mapping │ N64 R │ N64 Z │D-Up→R │ C-Dn  │ C-Lt  │ C-Up/Rt only        │
// │ 110 │ C-Lt mapping │ N64 R │ N64 Z │ N64 L │ C-Dn  │  —    │ C-Up/L/R            │
// │ 111 │ C-Rt mapping │ N64 R │ N64 L │ N64 Z │ C-Lt  │ C-Dn  │ C-Up/Rt only        │
// └─────┴──────────────┴───────┴───────┴───────┴───────┴───────┴─────────────────────┘
//
// DEFAULT MAPPING RATIONALE (dip=000):
//   The raphnet default swaps N64 Z↔L because on GameCube, Z is a small trigger
//   button while L is a large shoulder. On N64, Z is the large front trigger.
//   Swapping makes the physical feel more natural for Wii VC N64 games.
//   For games that need N64 Z as GC Z (e.g. some non-VC games), use dip=001.
// 
//
// ANALOG STICK SOURCE:
//   Stick correction (octant-affine fit against real OoT Wii-VC sweep
//   data) now lives entirely in stick_map.v, instantiated once in top.v.
//   top.v runs stick_map once per new N64 packet (n64_pkt_ready), captures
//   both corrected axes into gc_stick_x_reg/gc_stick_y_reg (already offset
//   to unsigned GC range, center=0x80), and feeds them in here as
//   gc_stick_x_in/gc_stick_y_in. button_map just passes them through into
//   the gc_state byte layout below — no scaling math in this module.
//
// C-BUTTON DUAL MAPPING NOTE:
//   Currently C-buttons map to C-stick only (not GC face buttons X/Y).
//   Some mappings promote C-Right→X and C-Left→Y (or C-Down→X, C-Left→Y).
//   When a C-button is promoted to a face button, it is REMOVED from the
//   C-stick mapping for that mode (to avoid unintended C-stick deflection
//   when pressing a face button).
//
//   To re-enable dual mapping (C-button acts as BOTH face button AND C-stick):
//   In the C-stick assignment for that axis, change the condition from:
//     (signal && !promoted) ? 8'hFF : ...
//   to:
//     signal ? 8'hFF : ...
//   where `promoted` is 1 when that C-button maps to a face button in the
//   current mode. This was a deliberate design choice — adjust per preference.
// =============================================================================

module button_map (
    input  wire [31:0] n64_state,     // latched N64 packet from n64_rx
    input  wire [2:0]  dip,           // mapping select from DIP switch bits [2:0]
    input  wire [7:0]  gc_stick_x_in, // pre-corrected main stick X from stick_map.v (top.v)
    input  wire [7:0]  gc_stick_y_in, // pre-corrected main stick Y from stick_map.v (top.v)
    input  wire        origin_checked,// 0 until 0x41 has been serviced once (see top.v)
    output wire [63:0] gc_state       // formatted GC response for gc_tx
);

    // =========================================================================
    // Convenience aliases — makes mapping table readable
    // =========================================================================
    wire n64_a      = n64_state[31];
    wire n64_b      = n64_state[30];
    wire n64_z      = n64_state[29];
    wire n64_start  = n64_state[28];
    wire n64_dup    = n64_state[27];
    wire n64_ddown  = n64_state[26];
    wire n64_dleft  = n64_state[25];
    wire n64_dright = n64_state[24];
    // [23] = RST — not aliased, not mapped
    // [22] = Reserved — not aliased, not mapped
    wire n64_l      = n64_state[21];
    wire n64_r      = n64_state[20];
    wire n64_cup    = n64_state[19];
    wire n64_cdown  = n64_state[18];
    wire n64_cleft  = n64_state[17];
    wire n64_cright = n64_state[16];

    // =========================================================================
    // Analog Stick — pass-through from stick_map.v (see header comment)
    // =========================================================================
    wire [7:0] gc_stick_x = gc_stick_x_in;
    wire [7:0] gc_stick_y = gc_stick_y_in;

    // =========================================================================
    // Per-mode Signal Routing
    // =========================================================================
    // Each GC output is driven by a different N64 source depending on dip[2:0].
    // All assignments are purely combinational (wire selects via ternary).
    //
    // Signals that change per mode:
    //   gc_z_src    — what drives GC Z button
    //   gc_l_src    — what drives GC L digital + L analog
    //   gc_r_src    — what drives GC R digital + R analog
    //   gc_x_src    — what drives GC X face button (0 if not mapped)
    //   gc_y_src    — what drives GC Y face button (0 if not mapped)
    //   cstick_x_rt — C-Right contributes to C-stick X (0 when promoted to face btn)
    //   cstick_x_lt — C-Left  contributes to C-stick X (0 when promoted to face btn)
    //   cstick_y_dn — C-Down  contributes to C-stick Y (0 when promoted to face btn)
    // =========================================================================

    // --- GC Z button source ---
    // Default/R/CDN/CLt: GC Z = N64 L (swap)
    // L/LR:              GC Z = N64 Z (passthrough)
    // CUP/CRIGHT:        GC Z = N64 R
    // CDN/CLEFT/CRIGHT:  GC Z = N64 R
    wire gc_z_src =
        (dip == 3'd0) ? n64_l     :  // Default:   Z←L
        (dip == 3'd1) ? n64_z     :  // L map:     Z←Z
        (dip == 3'd2) ? n64_l     :  // R map:     Z←L
        (dip == 3'd3) ? n64_z     :  // L+R map:   Z←Z
        (dip == 3'd4) ? n64_r     :  // C-Up map:  Z←R
        (dip == 3'd5) ? n64_r     :  // C-Dn map:  Z←R
        (dip == 3'd6) ? n64_r     :  // C-Lt map:  Z←R
                        n64_r;       // C-Rt map:  Z←R

    // --- GC L digital button source ---
    wire gc_l_dig_src =
        (dip == 3'd0) ? n64_z     :  // Default:   L←Z
        (dip == 3'd1) ? n64_l     :  // L map:     L←L
        (dip == 3'd2) ? n64_z     :  // R map:     L←Z
        (dip == 3'd3) ? n64_l     :  // L+R map:   L←L
        (dip == 3'd4) ? n64_z     :  // C-Up map:  L←Z
        (dip == 3'd5) ? n64_z     :  // C-Dn map:  L←Z
        (dip == 3'd6) ? n64_z     :  // C-Lt map:  L←Z
                        n64_l;       // C-Rt map:  L←L

    // --- GC R digital button source ---
    // CDN mapping uniquely maps GC R ← N64 D-Up (unusual but matches raphnet)
    wire gc_r_dig_src =
        (dip == 3'd0) ? n64_r     :  // Default:   R←R
        (dip == 3'd1) ? n64_r     :  // L map:     R←R
        (dip == 3'd2) ? n64_r     :  // R map:     R←R
        (dip == 3'd3) ? n64_r     :  // L+R map:   R←R
        (dip == 3'd4) ? n64_l     :  // C-Up map:  R←L
        (dip == 3'd5) ? n64_dup   :  // C-Dn map:  R←D-Up (raphnet special)
        (dip == 3'd6) ? n64_l     :  // C-Lt map:  R←L
                        n64_z;       // C-Rt map:  R←Z

    // --- GC X face button source ---
    // 0 when not mapped in this mode
    wire gc_x_src =
        (dip == 3'd0) ? 1'b0       :  // Default:   X not mapped
        (dip == 3'd1) ? 1'b0       :  // L map:     X not mapped
        (dip == 3'd2) ? n64_cright :  // R map:     X←C-Right
        (dip == 3'd3) ? n64_cright :  // L+R map:   X←C-Right
        (dip == 3'd4) ? n64_cdown  :  // C-Up map:  X←C-Down
        (dip == 3'd5) ? n64_cdown  :  // C-Dn map:  X←C-Down
        (dip == 3'd6) ? n64_cdown  :  // C-Lt map:  X←C-Down
                        n64_cleft;    // C-Rt map:  X←C-Left

    // --- GC Y face button source ---
    // 0 when not mapped in this mode
    wire gc_y_src =
        (dip == 3'd0) ? 1'b0       :  // Default:   Y not mapped
        (dip == 3'd1) ? 1'b0       :  // L map:     Y not mapped
        (dip == 3'd2) ? n64_cleft  :  // R map:     Y←C-Left
        (dip == 3'd3) ? n64_cleft  :  // L+R map:   Y←C-Left
        (dip == 3'd4) ? n64_cleft  :  // C-Up map:  Y←C-Left
        (dip == 3'd5) ? n64_cleft  :  // C-Dn map:  Y←C-Left
        (dip == 3'd6) ? 1'b0       :  // C-Lt map:  Y not mapped
                        n64_cdown;    // C-Rt map:  Y←C-Down

    // =========================================================================
    // D-pad routing
    // =========================================================================
    // CDN mapping uniquely unmaps D-Up (it goes to GC R instead).
    // All other mappings pass D-pad through 1:1.
    // =========================================================================
    wire gc_dup    = (dip == 3'd5) ? 1'b0     : n64_dup;    // CDN: D-Up→R, not D-Up
    wire gc_ddown  = n64_ddown;
    wire gc_dleft  = n64_dleft;
    wire gc_dright = n64_dright;

    // =========================================================================
    // C-Stick Mapping
    // =========================================================================
    // When a C-button is promoted to a GC face button (X or Y), it is removed
    // from the C-stick to prevent simultaneous unintended C-stick deflection.
    //
    // Which C-buttons are promoted per mode:
    //   dip=000,001: none promoted → all four C-buttons drive C-stick
    //   dip=010,011: C-Right→X, C-Left→Y → only C-Up/C-Down drive C-stick Y
    //                C-stick X has no driver (stays centered at 0x80)
    //   dip=100,101: C-Down→X, C-Left→Y  → only C-Up/C-Right drive C-stick
    //   dip=110:     C-Down→X only        → C-Up/C-Left/C-Right drive C-stick
    //                Y not mapped, so C-Left still on C-stick
    //   dip=111:     C-Left→X, C-Down→Y  → only C-Up/C-Right drive C-stick
    //
    // C-stick X axis (positive = right):
    //   C-Right drives X+ (0xFF) when not promoted to face button
    //   C-Left  drives X- (0x00) when not promoted to face button
    //   Both or neither = center (0x80)
    //
    // TO ENABLE DUAL MAPPING (C-button acts as both face button AND C-stick):
    //   Remove the `&& cstick_xr_active` / `&& cstick_xl_active` guards below.
    //   See header comment for full explanation.
    // =========================================================================

    // Flags: is this C-button still active on the C-stick (not promoted)?
    // Promotion flags: is this C-button used as a face button in current mode?
    // Based on DIP mode, not on current button state — a C-button is either
    // always promoted in a given mode or never promoted.
    // C-Right→X: dip=010,011 (R, L+R maps)
    wire cright_promoted = (dip == 3'd2) || (dip == 3'd3);
    // C-Left→Y:  dip=010,011,100,101 (R, L+R, C-Up, C-Down maps)
    wire cleft_promoted  = (dip == 3'd2) || (dip == 3'd3) ||
                           (dip == 3'd4) || (dip == 3'd5);
    // C-Down→X:  dip=100,101,110 (C-Up, C-Down, C-Left maps)
    //            also dip=111 promotes C-Down→Y
    wire cdown_promoted  = (dip == 3'd4) || (dip == 3'd5) ||
                           (dip == 3'd6) || (dip == 3'd7);
    // C-Left→X:  dip=111 (C-Right map)
    // Note: cleft_promoted already handles Y case; for X in dip=111 use cright promoted slot
    wire cleft_x_promoted = (dip == 3'd7);  // C-Left→X in C-Right map

    wire cstick_xr_active = !cright_promoted && !cleft_x_promoted; // C-Right on C-stick when not X
    wire cstick_xl_active = !cleft_promoted  && !cleft_x_promoted; // C-Left on C-stick when not Y/X
    wire cstick_yd_active = !cdown_promoted;                       // C-Down on C-stick when not X/Y
                                                      // C-Up is never promoted → always active

    // C-stick X: C-Right → 0xFF, C-Left → 0x00, center → 0x80
    // Priority: C-Right wins if both pressed (arbitrary, could be changed)
    wire cstick_xr = n64_cright && cstick_xr_active;
    wire cstick_xl = n64_cleft  && cstick_xl_active;

    // Deflect only when exactly one direction is pressed.
    // Both pressed simultaneously = center (cancel out).
    wire [7:0] gc_cstick_x = (cstick_xr && !cstick_xl) ? 8'hFF :
                             (cstick_xl && !cstick_xr) ? 8'h00 :
                             8'h80;

    // C-stick Y: C-Up → 0xFF, C-Down → 0x00, center → 0x80
    // C-Up is never promoted so always active.
    wire cstick_yd = n64_cdown && cstick_yd_active;

    // C-Up never promoted, always active. Both C-Up+C-Down = center.
    wire [7:0] gc_cstick_y = (n64_cup && !cstick_yd) ? 8'hFF :
                             (cstick_yd && !n64_cup) ? 8'h00 :
                             8'h80;

    // =========================================================================
    // L/R Analog Trigger Bytes
    // =========================================================================
    // N64 L/R are digital. GC expects analog (0–255) + digital.
    // Pressed = 0xFF full pull, released = 0x00.
    // The analog byte and digital bit must both be set for the Wii to
    // register a full trigger press in most games.
    // =========================================================================
    wire [7:0] gc_l_analog = gc_l_dig_src ? 8'hFF : 8'h00;
    wire [7:0] gc_r_analog = gc_r_dig_src ? 8'hFF : 8'h00;

    // =========================================================================
    // Assemble GC Byte 0 and Byte 1
    // =========================================================================
    // Byte 0 per YAGCD / libdragon:
    //   [7:6] = ErrStat, ErrLatch (always 0 — no error)
    //   [5]   = origin_unchecked (1 until a 0x41 ORIGIN request has been
    //           serviced at least once, then stays 0 for the rest of the
    //           session). Per jefflongo.dev's GC controller reverse-
    //           engineering writeup: "the Origin has been sent to console
    //           bit in the ID, origin, and status commands are cleared"
    //           once origin has been sent -- confirmed independently by
    //           the author's own GC+ implementation notes: "the get
    //           origin bit (in byte 0 of the status/get origin packet)
    //           is cleared before the origin packet is written on the
    //           bus." Previously hardcoded to 1 here always (see prior
    //           comment history) -- meaning every response, including
    //           the origin response itself, told the Wii "origin not
    //           yet sent," even after actually answering 0x41. Now
    //           driven from top.v's origin_checked, which latches once
    //           origin_req has been serviced (see top.v).
    //   [4]   = Start
    //   [3]   = Y    [2] = X    [1] = B    [0] = A
    wire [7:0] gc_byte0 = {2'b00,
                           !origin_checked, // [5] origin_unchecked
                           n64_start,      // [4] Start
                           gc_y_src,       // [3] Y
                           gc_x_src,       // [2] X
                           n64_b,          // [1] B
                           n64_a};         // [0] A

    // Byte 1 per YAGCD / libdragon:
    //   [7]   = High1 / unused2 (always 1 — protocol marker)
    //   [6]   = L digital   [5] = R digital   [4] = Z
    //   [3]   = D-Up    [2] = D-Down   [1] = D-Right   [0] = D-Left
    wire [7:0] gc_byte1 = {1'b1,           // [7] High1 always 1
                           gc_l_dig_src,   // [6] L digital
                           gc_r_dig_src,   // [5] R digital
                           gc_z_src,       // [4] Z
                           gc_dup,         // [3] D-Up
                           gc_ddown,       // [2] D-Down
                           gc_dright,      // [1] D-Right
                           gc_dleft};      // [0] D-Left

    // =========================================================================
    // Assemble 64-bit gc_state output
    // =========================================================================
    assign gc_state = {
        gc_byte0,    // [63:56] Byte 0
        gc_byte1,    // [55:48] Byte 1
        gc_stick_x,  // [47:40] Byte 2: main stick X
        gc_stick_y,  // [39:32] Byte 3: main stick Y
        gc_cstick_x, // [31:24] Byte 4: C-stick X
        gc_cstick_y, // [23:16] Byte 5: C-stick Y
        gc_l_analog, // [15:8]  Byte 6: L analog
        gc_r_analog  // [7:0]   Byte 7: R analog
    };

endmodule