// ---------------------------------------------------------
// Body levelling
// converts terrain pitch/roll to per-leg
// height corrections and injects them into hleg[]
// Call once per imu_loop() update, before kinematics_mat()
//
// Slope memory strategy (pure integrator):
//
//   Previous behaviour: hleg[i] = hleg_base[i] + dz
//     hleg_base is the FIXED encoder reference (265/280 mm). When the body
//     successfully levels (dz → 0), hleg snapped back to hleg_base and the
//     legs lost the slope correction, causing the body to tilt again → loop.
//
//   New behaviour:
//     hleg_slope_base[i] is a MOVING reference initialised to the walking
//     height (300 mm) that slowly INTEGRATES the slope correction:
//
//       hleg_slope_base[i] += SLOPE_ADAPT_RATE * dz
//       hleg[i] = hleg_slope_base[i]         ← integrator output only, NO +dz
//
//     WHY no +dz (P term):
//       Adding dz a second time on top of hleg_slope_base caused one leg toextend continuously
//       the integrator AND the proportional term both
//       push hleg in the same direction. If the IMU has any persistent offset
//       (even small), hleg_slope_base grows to the clamp limit and then hleg
//       = clamp + dz_bias, exceeding mechanical travel.
//       With the integrator alone: hleg can only change by SLOPE_ADAPT_RATE*dz
//       per iteration and is always equal to hleg_slope_base (clamped). Safe.
//
//     When body levels on a slope: dz → 0, hleg_slope_base stops and retains
//     the correction. No snap-back.
//     When the robot leaves the slope: body tilts the other way → dz reverses
//     → hleg_slope_base bleeds back toward neutral automatically.
// ---------------------------------------------------------

// Foot positions relative to body centre (mm)
static const float legX[4] = { +142.5f, -142.5f, -142.5f, +142.5f }; // FR BL FL BR
static const float legY[4] = { +165.0f, -165.0f, +165.0f, -165.0f };

// Integration rate for slope memory.
// At 0.10 and ~80 ms loop period: time-constant ≈ 80 * (1/0.10) = 800 ms
// Reduce to 0.05 for ~1600 ms (slower, smoother). Increase cautiously
// Too high will make the integrator overshoot on rapid slope changes.
static const float SLOPE_ADAPT_RATE = 0.10f;

// Maximum allowed drift of hleg_slope_base away from the neutral walking height (mm).
// Hard limit. Prevents runaway from IMU bias or sustained disturbances.
// 50 mm gives ~10° correction headroom at 280 mm leg radius.
static const float SLOPE_ADAPT_MAX_MM = 50.0f;

// Per-leg slope-memory base, initialised from hleg[] on the first call
// (setup() sets hleg[] to 300 mm before loop() runs, so this picks that up).
// hleg_neutral[] captures the same initial value and never changes;
// it is the centre for the ±SLOPE_ADAPT_MAX_MM safety clamp.
// NOTE: hleg_base[] (the encoder reference, 265/280 mm) is NEVER touched here.
static float hleg_slope_base[4];
static float hleg_neutral[4];
static bool  hleg_slope_init = false;


void body_level_update(float terrain_pitch_deg, float terrain_roll_deg) {
        
    static uint32_t bl_enable_ms = 0;
    if (bl_enable_ms == 0) bl_enable_ms = millis();
    if (millis() - bl_enable_ms < 8000) return;  // 8s settle time
    // One-time initialisation, capture walking height set by setup()
    if (!hleg_slope_init) {
        for (int i = 0; i < 4; i++) {
            hleg_slope_base[i] = hleg[i];
            hleg_neutral[i]    = hleg[i];
        }
        hleg_slope_init = true;
    }

    // Deadband, ignore tiny angles (IMU noise floor)
    const float deadband = 1.0f;  // degrees (May have to increase it to make more stable)
    if (fabs(terrain_pitch_deg) < deadband) terrain_pitch_deg = 0.0f;
    if (fabs(terrain_roll_deg)  < deadband) terrain_roll_deg  = 0.0f;

    float pitch_rad = terrain_pitch_deg * DEG_TO_RAD;
    float roll_rad  = terrain_roll_deg  * DEG_TO_RAD;

    for (int i = 0; i < 4; i++) {
        // Levelling correction needed for this leg
        float dz_pitch = legY[i] * tan(pitch_rad);
        float dz_roll  = legX[i] * tan(roll_rad);
        float dz = dz_pitch + dz_roll;

        // Integrate dz into hleg_slope_base.
        // When body is level (dz = 0): base holds its accumulated value, no snap-back.
        // When body tilts from a new slope: base integrates further in that direction.
        // When IMU reads 0 because the body IS level (not because slope is gone):
        // dz = 0, base stays put, hleg stays at the corrected height.
        hleg_slope_base[i] += SLOPE_ADAPT_RATE * dz;

        // Clamp. keep within ±SLOPE_ADAPT_MAX_MM of the neutral walking height
        hleg_slope_base[i] = constrain(hleg_slope_base[i],
                                        hleg_neutral[i] - SLOPE_ADAPT_MAX_MM,
                                        hleg_neutral[i] + SLOPE_ADAPT_MAX_MM);

        // hleg is the integrator output only. No additional dz term
        hleg[i] = hleg_slope_base[i];
    }
}
