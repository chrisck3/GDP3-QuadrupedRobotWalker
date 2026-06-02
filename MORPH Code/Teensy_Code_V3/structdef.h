// gait_struct holds all pre-computed foot trajectory tables for one gait.
// Instantiated as 'crawl' in the Teensy_Code.ino; used exclusively by create_mat().
// To switch gaits, swap the active struct definition (crawl/trot blocks below).
#ifndef STRUCTDEF_H
#define STRUCTDEF_H

#define legn  4   // number of legs
#define coor  3   // x, y, z
#define itr  16   // iterations per gait cycle (must be multiple of legn)
#define timer 50  // legacy define, not used by firmware; see loop_timer in main .ino

// =============================================================================
// Crawl gait — 16-phase, one leg at a time, FR->BL->FL->BR
//
// Leg index:  0=FR, 1=BL, 2=FL, 3=BR
// Swing windows: FR=phases 1-3, BL=phases 5-7, FL=phases 9-11, BR=phases 13-15
// Phases 0,4,8,12: all feet grounded (stability buffer between lifts)
//
// FB table designed for FBrat=1.0 → 60 mm total stride (±30 mm from home)
//   Swing  : foot moves +20 mm/phase in y over 3 phases (uniform)
//   Stance : foot moves  -5 mm/phase in y over 12 phases (uniform)
//   Result : zero body-roll contribution from uneven stance steps
//
// To revert to FBrat=0.5 operation, use FBrat=0.5 in firmware and stride = 30 mm.

//
struct gait_struct {

  // --------------------------------------------------------------------------
  // March: vertical lift arc — 3-phase triangle per leg
  //   rise to 15 mm → peak 30 mm → descend to 15 mm
  //   (reduce peak from 30 to 20 if ground clearance allows)
  // --------------------------------------------------------------------------
  double March[itr][legn][coor] = {
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0, 0}},  //  0  all grounded
    {{0,0,10},{0,0, 0},{0,0, 0},{0,0, 0}},  //  1  FR lift
    {{0,0,20},{0,0, 0},{0,0, 0},{0,0, 0}},  //  2  FR peak   (effective lift = 20×2.5 = 50 mm at h=300)
    {{0,0,10},{0,0, 0},{0,0, 0},{0,0, 0}},  //  3  FR descend
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0, 0}},  //  4  all grounded
    {{0,0, 0},{0,0,10},{0,0, 0},{0,0, 0}},  //  5  BL lift
    {{0,0, 0},{0,0,20},{0,0, 0},{0,0, 0}},  //  6  BL peak
    {{0,0, 0},{0,0,10},{0,0, 0},{0,0, 0}},  //  7  BL descend
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0, 0}},  //  8  all grounded
    {{0,0, 0},{0,0, 0},{0,0,10},{0,0, 0}},  //  9  FL lift
    {{0,0, 0},{0,0, 0},{0,0,20},{0,0, 0}},  // 10  FL peak
    {{0,0, 0},{0,0, 0},{0,0,10},{0,0, 0}},  // 11  FL descend
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0, 0}},  // 12  all grounded
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0,10}},  // 13  BR lift
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0,20}},  // 14  BR peak
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0,10}}   // 15  BR descend
  };

  // --------------------------------------------------------------------------
  // FB: forward/backward foot positions (y-axis only, x=0, z=0)
  //
  // At FBrat=1.0: effective foot motion = table value × 1.0
  //   Stride = 60 mm  |  swing step = +20 mm/phase  |  stance step = -5 mm/phase
  //
  // Per-leg y trajectory (FR as example, others are cyclic shifts by 4 phases):
  //   Phase 0      : y=-30  (last stance, foot at back)
  //   Phases 1,2,3 : y=-10,+10,+30  (swing, foot flies forward)
  //   Phase 4      : y=+30  (landed, foot at front — y unchanged from phase 3)
  //   Phases 5-15  : y decreasing by 5mm/phase back toward -30
  //   Phase 0 next : y=-30  (cycle closes cleanly)
  // --------------------------------------------------------------------------
  double FB[itr][legn][coor] = {
    {{0,-30,0},{0,-10,0},{0, 10,0},{0, 30,0}},  //  0
    {{0,-10,0},{0,-15,0},{0,  5,0},{0, 25,0}},  //  1  FR swing-1
    {{0, 10,0},{0,-20,0},{0,  0,0},{0, 20,0}},  //  2  FR swing-2 (peak z)
    {{0, 30,0},{0,-25,0},{0, -5,0},{0, 15,0}},  //  3  FR swing-3
    {{0, 30,0},{0,-30,0},{0,-10,0},{0, 10,0}},  //  4  FR landed
    {{0, 25,0},{0,-10,0},{0,-15,0},{0,  5,0}},  //  5  BL swing-1
    {{0, 20,0},{0, 10,0},{0,-20,0},{0,  0,0}},  //  6  BL swing-2 (peak z)
    {{0, 15,0},{0, 30,0},{0,-25,0},{0, -5,0}},  //  7  BL swing-3
    {{0, 10,0},{0, 30,0},{0,-30,0},{0,-10,0}},  //  8  BL landed
    {{0,  5,0},{0, 25,0},{0,-10,0},{0,-15,0}},  //  9  FL swing-1
    {{0,  0,0},{0, 20,0},{0, 10,0},{0,-20,0}},  // 10  FL swing-2 (peak z)
    {{0, -5,0},{0, 15,0},{0, 30,0},{0,-25,0}},  // 11  FL swing-3
    {{0,-10,0},{0, 10,0},{0, 30,0},{0,-30,0}},  // 12  FL landed
    {{0,-15,0},{0,  5,0},{0, 25,0},{0,-10,0}},  // 13  BR swing-1
    {{0,-20,0},{0,  0,0},{0, 20,0},{0, 10,0}},  // 14  BR swing-2 (peak z)
    {{0,-25,0},{0, -5,0},{0, 15,0},{0, 30,0}}   // 15  BR swing-3
  };

  // --------------------------------------------------------------------------
  // LR: lateral (x-axis) offsets — ±30 mm amplitude matching FB
  //
  // Each leg's x follows its own y trajectory from FB (4-phase cyclic shift).
  // Every leg independently sweeps +60 mm in x during its 3-phase swing
  // and returns at −5 mm/phase over 12 stance phases → closed-loop strafe.
  // Scale by LRrat in firmware to set strafe speed.
  // --------------------------------------------------------------------------
  double LR[itr][legn][coor] = {
    {{ -30,0,0},{ -10,0,0},{  10,0,0},{  30,0,0}},  //  0
    {{ -10,0,0},{ -15,0,0},{   5,0,0},{  25,0,0}},  //  1  FR swing-1
    {{  10,0,0},{ -20,0,0},{   0,0,0},{  20,0,0}},  //  2  FR swing-2
    {{  30,0,0},{ -25,0,0},{  -5,0,0},{  15,0,0}},  //  3  FR swing-3
    {{  30,0,0},{ -30,0,0},{ -10,0,0},{  10,0,0}},  //  4  FR landed
    {{  25,0,0},{ -10,0,0},{ -15,0,0},{   5,0,0}},  //  5  BL swing-1
    {{  20,0,0},{  10,0,0},{ -20,0,0},{   0,0,0}},  //  6  BL swing-2
    {{  15,0,0},{  30,0,0},{ -25,0,0},{  -5,0,0}},  //  7  BL swing-3
    {{  10,0,0},{  30,0,0},{ -30,0,0},{ -10,0,0}},  //  8  BL landed
    {{   5,0,0},{  25,0,0},{ -10,0,0},{ -15,0,0}},  //  9  FL swing-1
    {{   0,0,0},{  20,0,0},{  10,0,0},{ -20,0,0}},  // 10  FL swing-2
    {{  -5,0,0},{  15,0,0},{  30,0,0},{ -25,0,0}},  // 11  FL swing-3
    {{ -10,0,0},{  10,0,0},{  30,0,0},{ -30,0,0}},  // 12  FL landed
    {{ -15,0,0},{   5,0,0},{  25,0,0},{ -10,0,0}},  // 13  BR swing-1
    {{ -20,0,0},{   0,0,0},{  20,0,0},{  10,0,0}},  // 14  BR swing-2
    {{ -25,0,0},{  -5,0,0},{  15,0,0},{  30,0,0}}   // 15  BR swing-3
  };

  // --------------------------------------------------------------------------
  // RT: yaw-rotation foot offsets (x, y) — derived from previous group geometry
  //
  // Each leg independently sweeps its rotation arc during its 3-phase swing
  // and returns linearly over 12 stance phases (stretched from the previous
  // group's 4-phase return to our 12-phase return).
  //   FR: max−neg phase 0, max+pos phase 4,  return phases  4→15
  //   BL: max−neg phase 4, max+pos phase 8,  return phases  8→3
  //   FL: max−neg phase 8, max+pos phase 12, return phases 12→7
  //   BR: max−neg phase 12,max+pos phase 0,  return phases  0→11
  // Scale by RTrat in firmware to set turn speed.
  // --------------------------------------------------------------------------
  double RT[itr][legn][coor] = {
    {{-30.51, 18.68,0},{ 11.41, -4.55,0},{  7.70,  9.57,0},{-26.80,-23.69,0}},  //  0
    {{-14.84, 10.01,0},{ 16.19, -8.08,0},{  2.92,  6.04,0},{-22.02,-20.16,0}},  //  1  FR swing-1
    {{  0.00,  0.00,0},{ 20.97,-11.61,0},{ -1.86,  2.51,0},{-17.25,-16.63,0}},  //  2  FR swing-2
    {{ 13.92,-11.26,0},{ 25.74,-15.14,0},{ -6.64, -1.02,0},{-12.47,-13.10,0}},  //  3  FR swing-3
    {{ 26.80,-23.69,0},{ 30.51,-18.68,0},{-11.41, -4.55,0},{ -7.70, -9.57,0}},  //  4  FR landed
    {{ 22.02,-20.16,0},{ 14.84,-10.01,0},{-16.19, -8.08,0},{ -2.92, -6.04,0}},  //  5  BL swing-1
    {{ 17.25,-16.63,0},{  0.00,  0.00,0},{-20.97,-11.61,0},{  1.86, -2.51,0}},  //  6  BL swing-2
    {{ 12.47,-13.10,0},{-13.92, 11.26,0},{-25.74,-15.14,0},{  6.64,  1.02,0}},  //  7  BL swing-3
    {{  7.70, -9.57,0},{-26.80, 23.69,0},{-30.51,-18.68,0},{  11.41, 4.55,0}},  //  8  BL landed
    {{  2.92, -6.04,0},{-22.02, 20.16,0},{-14.84,-10.01,0},{  16.19, 8.08,0}},  //  9  FL swing-1
    {{ -1.86, -2.51,0},{-17.25, 16.63,0},{  0.00,  0.00,0},{  20.97,11.61,0}},  // 10  FL swing-2
    {{ -6.64,  1.02,0},{-12.47, 13.10,0},{ 13.92, 11.26,0},{  25.74,15.14,0}},  // 11  FL swing-3
    {{-11.41,  4.55,0},{ -7.70,  9.57,0},{ 26.80, 23.69,0},{  30.51,18.68,0}},  // 12  FL landed
    {{-16.19,  8.08,0},{ -2.92,  6.04,0},{ 22.02, 20.16,0},{  14.84,10.01,0}},  // 13  BR swing-1
    {{-20.97, 11.61,0},{  1.86,  2.51,0},{ 17.25, 16.63,0},{   0.00, 0.00,0}},  // 14  BR swing-2
    {{-25.74, 15.14,0},{  6.64, -1.02,0},{ 12.47, 13.10,0},{ -13.92,-11.26,0}}  // 15  BR swing-3
  };

  // --------------------------------------------------------------------------
  // WS: weight shift offsets — disabled via ws_gain=0.0 in firmware
  // Zeroed here for clarity; restore and re-enable ws_gain once gait is stable
  // --------------------------------------------------------------------------
  double WS[itr][legn][coor] = {
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  0
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  1
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  2
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  3
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  4
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  5
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  6
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  7
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  8
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  9
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  // 10
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  // 11
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  // 12
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  // 13
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  // 14
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}}   // 15
  };

};
//=============================================================================
/*

// =============================================================================
// Trot gait — 16-phase, diagonal pairs, FR+BL <-> FL+BR
//
// Leg index:  0=FR, 1=BL, 2=FL, 3=BR
// Diagonal pair A: FR+BL — swing phases 1-3
// Diagonal pair B: FL+BR — swing phases 9-11
// Phases 0,4,8,12: all feet grounded (stability buffer)
//
// FB table designed for FBrat=1.0 → 60 mm total stride (±30 mm from home)
//   Swing  : foot moves +20 mm/phase in y over 3 phases (uniform)
//   Stance : foot moves  -5 mm/phase in y over 12 phases (uniform)
//
// To switch from crawl to trot:
//   1. Comment out the crawl gait_struct block above
//   2. Uncomment this block
//   (instance name 'crawl' in main .ino and all create_mat calls stay unchanged)


struct gait_struct {

  // --------------------------------------------------------------------------
  // March: vertical lift arc — 3-phase triangle per diagonal pair
  //   rise to 10 mm → peak 20 mm → descend to 10 mm
  // --------------------------------------------------------------------------
  double March[itr][legn][coor] = {
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0, 0}},  //  0  all grounded
    {{0,0,10},{0,0,10},{0,0, 0},{0,0, 0}},  //  1  FR+BL lift
    {{0,0,20},{0,0,20},{0,0, 0},{0,0, 0}},  //  2  FR+BL peak
    {{0,0,10},{0,0,10},{0,0, 0},{0,0, 0}},  //  3  FR+BL descend
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0, 0}},  //  4  all grounded
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0, 0}},  //  5  all stance
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0, 0}},  //  6  all stance
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0, 0}},  //  7  all stance
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0, 0}},  //  8  all grounded
    {{0,0, 0},{0,0, 0},{0,0,10},{0,0,10}},  //  9  FL+BR lift
    {{0,0, 0},{0,0, 0},{0,0,20},{0,0,20}},  // 10  FL+BR peak
    {{0,0, 0},{0,0, 0},{0,0,10},{0,0,10}},  // 11  FL+BR descend
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0, 0}},  // 12  all grounded
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0, 0}},  // 13  all stance
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0, 0}},  // 14  all stance
    {{0,0, 0},{0,0, 0},{0,0, 0},{0,0, 0}}   // 15  all stance
  };

  // --------------------------------------------------------------------------
  // FB: forward/backward foot positions (y-axis only, x=0, z=0)
  //
  // At FBrat=1.0: stride = 60 mm | swing = +20 mm/phase | stance = -5 mm/phase
  //
  // FR/BL start at y=-30 (back), swing to +30 in phases 1-3, then
  // walk back at -5mm/phase over phases 4-15 (12 stance phases).
  // FL/BR are half a cycle ahead: start at y=+10 and swing in phases 9-11.
  // --------------------------------------------------------------------------
  double FB[itr][legn][coor] = {
    {{0,-30,0},{0,-30,0},{0, 10,0},{0, 10,0}},  //  0
    {{0,-10,0},{0,-10,0},{0,  5,0},{0,  5,0}},  //  1  FR+BL swing-1
    {{0, 10,0},{0, 10,0},{0,  0,0},{0,  0,0}},  //  2  FR+BL swing-2 (peak z)
    {{0, 30,0},{0, 30,0},{0, -5,0},{0, -5,0}},  //  3  FR+BL swing-3
    {{0, 30,0},{0, 30,0},{0,-10,0},{0,-10,0}},  //  4  FR+BL landed
    {{0, 25,0},{0, 25,0},{0,-15,0},{0,-15,0}},  //  5
    {{0, 20,0},{0, 20,0},{0,-20,0},{0,-20,0}},  //  6
    {{0, 15,0},{0, 15,0},{0,-25,0},{0,-25,0}},  //  7
    {{0, 10,0},{0, 10,0},{0,-30,0},{0,-30,0}},  //  8
    {{0,  5,0},{0,  5,0},{0,-10,0},{0,-10,0}},  //  9  FL+BR swing-1
    {{0,  0,0},{0,  0,0},{0, 10,0},{0, 10,0}},  // 10  FL+BR swing-2 (peak z)
    {{0, -5,0},{0, -5,0},{0, 30,0},{0, 30,0}},  // 11  FL+BR swing-3
    {{0,-10,0},{0,-10,0},{0, 30,0},{0, 30,0}},  // 12  FL+BR landed
    {{0,-15,0},{0,-15,0},{0, 25,0},{0, 25,0}},  // 13
    {{0,-20,0},{0,-20,0},{0, 20,0},{0, 20,0}},  // 14
    {{0,-25,0},{0,-25,0},{0, 15,0},{0, 15,0}}   // 15
  };

  // --------------------------------------------------------------------------
  // LR: lateral (x-axis) offsets — ±30 mm amplitude matching FB
  //
  // FR+BL x follows the FR+BL y trajectory from FB (diagonal pair A).
  // FL+BR x follows the FL+BR y trajectory from FB (diagonal pair B).
  // Both pairs step in the same lateral direction → closed-loop strafe.
  //   Swing  : foot sweeps +60 mm in x over 3 phases (from −30 to +30)
  //   Stance : foot returns  −5 mm/phase over 12 phases
  // Scale by LRrat in firmware to set strafe speed.
  // --------------------------------------------------------------------------
  double LR[itr][legn][coor] = {
    {{-30,0,0},{-30,0,0},{ 10,0,0},{ 10,0,0}},  //  0
    {{ -10,0,0},{ -10,0,0},{  5,0,0},{  5,0,0}},  //  1  FR+BL swing-1
    {{  10,0,0},{  10,0,0},{  0,0,0},{  0,0,0}},  //  2  FR+BL swing-2
    {{  30,0,0},{  30,0,0},{ -5,0,0},{ -5,0,0}},  //  3  FR+BL swing-3
    {{  30,0,0},{  30,0,0},{ -10,0,0},{ -10,0,0}},  //  4  FR+BL landed
    {{  25,0,0},{  25,0,0},{ -15,0,0},{ -15,0,0}},  //  5
    {{  20,0,0},{  20,0,0},{ -20,0,0},{ -20,0,0}},  //  6
    {{  15,0,0},{  15,0,0},{ -25,0,0},{ -25,0,0}},  //  7
    {{  10,0,0},{  10,0,0},{ -30,0,0},{ -30,0,0}},  //  8
    {{   5,0,0},{   5,0,0},{ -10,0,0},{ -10,0,0}},  //  9  FL+BR swing-1
    {{   0,0,0},{   0,0,0},{  10,0,0},{  10,0,0}},  // 10  FL+BR swing-2
    {{  -5,0,0},{  -5,0,0},{  30,0,0},{  30,0,0}},  // 11  FL+BR swing-3
    {{ -10,0,0},{ -10,0,0},{  30,0,0},{  30,0,0}},  // 12  FL+BR landed
    {{ -15,0,0},{ -15,0,0},{  25,0,0},{  25,0,0}},  // 13
    {{ -20,0,0},{ -20,0,0},{  20,0,0},{  20,0,0}},  // 14
    {{ -25,0,0},{ -25,0,0},{  15,0,0},{  15,0,0}}   // 15
  };

  // --------------------------------------------------------------------------
  // RT: yaw-rotation foot offsets (x, y) — derived from previous group geometry
  //
  // Each foot traces a circular arc as the body yaws. Values represent the
  // (Δx, Δy) displacement for each leg at a given rotation angle.
  // FR+BL swing arc in phases 1–3; FL+BR swing arc in phases 9–11.
  // Swing values match the previous group's 3-phase arc directly.
  // Stance return is linear over 12 phases (stretched from 4→12 phases).
  //   FR/BL: max−neg at phase 0, max+pos at phase 4, linear return phases 4–15
  //   FL/BR: max−neg at phase 8, max+pos at phase 12, linear return phases 12–8
  // Scale by RTrat in firmware to set turn speed.
  // --------------------------------------------------------------------------
  double RT[itr][legn][coor] = {
    {{-30.51, 18.68,0},{ 30.51,-18.68,0},{  7.70,  9.57,0},{ -7.70, -9.57,0}},  //  0
    {{-14.84, 10.01,0},{ 14.84,-10.01,0},{  2.92,  6.04,0},{ -2.92, -6.04,0}},  //  1  FR+BL swing-1
    {{  0.00,  0.00,0},{  0.00,  0.00,0},{ -1.86,  2.51,0},{  1.86, -2.51,0}},  //  2  FR+BL swing-2
    {{ 13.92,-11.26,0},{-13.92, 11.26,0},{ -6.64, -1.02,0},{  6.64,  1.02,0}},  //  3  FR+BL swing-3
    {{ 26.80,-23.69,0},{-26.80, 23.69,0},{-11.41, -4.55,0},{ 11.41,  4.55,0}},  //  4  FR+BL landed
    {{ 22.02,-20.16,0},{-22.02, 20.16,0},{-16.19, -8.08,0},{ 16.19,  8.08,0}},  //  5
    {{ 17.25,-16.63,0},{-17.25, 16.63,0},{-20.97,-11.61,0},{ 20.97, 11.61,0}},  //  6
    {{ 12.47,-13.10,0},{-12.47, 13.10,0},{-25.74,-15.14,0},{ 25.74, 15.14,0}},  //  7
    {{  7.70, -9.57,0},{ -7.70,  9.57,0},{-30.51,-18.68,0},{ 30.51, 18.68,0}},  //  8
    {{  2.92, -6.04,0},{ -2.92,  6.04,0},{-14.84,-10.01,0},{ 14.84, 10.01,0}},  //  9  FL+BR swing-1
    {{ -1.86, -2.51,0},{  1.86,  2.51,0},{  0.00,  0.00,0},{  0.00,  0.00,0}},  // 10  FL+BR swing-2
    {{ -6.64,  1.02,0},{  6.64, -1.02,0},{ 13.92, 11.26,0},{-13.92,-11.26,0}},  // 11  FL+BR swing-3
    {{-11.41,  4.55,0},{ 11.41, -4.55,0},{ 26.80, 23.69,0},{-26.80,-23.69,0}},  // 12  FL+BR landed
    {{-16.19,  8.08,0},{ 16.19, -8.08,0},{ 22.02, 20.16,0},{-22.02,-20.16,0}},  // 13
    {{-20.97, 11.61,0},{ 20.97,-11.61,0},{ 17.25, 16.63,0},{-17.25,-16.63,0}},  // 14
    {{-25.74, 15.14,0},{ 25.74,-15.14,0},{ 12.47, 13.10,0},{-12.47,-13.10,0}}   // 15
  };

  // --------------------------------------------------------------------------
  // WS: weight shift offsets. disabled via ws_gain=0.0 in firmware
  // --------------------------------------------------------------------------
  double WS[itr][legn][coor] = {
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  0
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  1
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  2
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  3
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  4
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  5
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  6
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  7
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  8
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  //  9
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  // 10
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  // 11
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  // 12
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  // 13
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}},  // 14
    {{0,0,0},{0,0,0},{0,0,0},{0,0,0}}   // 15
  };

};

// =============================================================================
*/
#endif
