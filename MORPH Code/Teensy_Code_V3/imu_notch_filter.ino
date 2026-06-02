// ---------------------------------------------------------
// IMU Notch Filter – Cascaded 2nd-order IIR notch filters
// ---------------------------------------------------------
// Removes gait-induced periodic body oscillations from IMU pitch/roll.
//
// Filter chain:
//   Kalman pitch --> Notch @ f0 (1st harmonic) --> Notch @ 2*f0 (2nd harmonic) --> EMA LPF --> output
//
// Design parameters (fs = 100 Hz):
//   1st notch:  f0 = 0.70 Hz,  Q = 6    (fundamental gait frequency)
//   2nd notch:  f0 = 1.61 Hz,  Q = 9    (2nd harmonic)
//   EMA LPF:    alpha = 0.20
//
// Each notch is a biquad:  y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
//                                         - a1*y[n-1] - a2*y[n-2]
// ---------------------------------------------------------

#ifndef IMU_NOTCH_FILTER_H
#define IMU_NOTCH_FILTER_H

//Biquad (2nd-order IIR) section
struct Biquad {
  // Coefficients
  float b0, b1, b2;
  float a1, a2;
  // State (direct-form I)
  float x1, x2;   // x[n-1], x[n-2]
  float y1, y2;   // y[n-1], y[n-2]

  void reset() {
    x1 = x2 = y1 = y2 = 0.0f;
  }

  // Seed the state so the filter starts at a known value
  // (avoids large transient at startup)
  void seed(float val) {
    x1 = x2 = val;
    y1 = y2 = val;
  }

  float process(float x0) {
    float y0 = b0 * x0 + b1 * x1 + b2 * x2
                        - a1 * y1 - a2 * y2;
    // Shift state
    x2 = x1;  x1 = x0;
    y2 = y1;  y1 = y0;
    return y0;
  }
};

//Exponential Moving Average LP filter
struct EmaLpf {
  float alpha;
  float y_prev;
  bool  initialised;

  void reset() {
    y_prev = 0.0f;
    initialised = false;
  }

  float process(float x) {
    if (!initialised) {
      y_prev = x;
      initialised = true;
      return x;
    }
    y_prev += alpha * (x - y_prev);
    return y_prev;
  }
};

//Complete dual-notch + LPF pipeline
class ImuNotchFilter {
public:
  Biquad notch1;   // 1st harmonic  (0.70 Hz)
  Biquad notch2;   // 2nd harmonic  (1.61 Hz)
  EmaLpf lpf;

  void init() {
    // 1st harmonic notch: f0 = 0.70 Hz, Q = 6, fs = 100 Hz
    notch1.b0 =  0.9963493661f;
    notch1.b1 = -1.9907716624f;
    notch1.b2 =  0.9963493661f;
    notch1.a1 = -1.9907716624f;
    notch1.a2 =  0.9926987322f;
    notch1.reset();

    //2nd harmonic notch: f0 = 1.61 Hz, Q = 9, fs = 100 Hz
    notch2.b0 =  0.9944209207f;
    notch2.b1 = -1.9786744075f;
    notch2.b2 =  0.9944209207f;
    notch2.a1 = -1.9786744075f;
    notch2.a2 =  0.9888418414f;
    notch2.reset();

    //EMA low-pass: alpha = 0.20
    lpf.alpha = 0.20f;
    lpf.reset();
  }

  // Call once at startup with the first IMU reading so the
  // filters don't ring from a zero initial condition.
  void seed(float first_reading) {
    notch1.seed(first_reading);
    notch2.seed(first_reading);
    lpf.y_prev = first_reading;
    lpf.initialised = true;
  }

  // Feed a new Kalman-filtered pitch (or roll) sample.
  // Returns the cleaned terrain estimate.
  float update(float kalman_angle) {
    float after_n1 = notch1.process(kalman_angle);
    float after_n2 = notch2.process(after_n1);
    float filtered  = lpf.process(after_n2);
    return filtered;
  }
};

//Global instances for pitch and roll
ImuNotchFilter pitchFilter;
ImuNotchFilter rollFilter;

// Call from setup() after the first valid IMU read
void imu_filter_init(float initial_pitch, float initial_roll) {
  pitchFilter.init();
  rollFilter.init();
  pitchFilter.seed(initial_pitch);
  rollFilter.seed(initial_roll);
}

// Call every IMU sample (~10 ms at 100 Hz).
// Writes the cleaned terrain pitch & roll into the output pointers.
void imu_filter_update(float raw_pitch, float raw_roll,
                       float *terrain_pitch, float *terrain_roll) {
  *terrain_pitch = pitchFilter.update(raw_pitch);
  *terrain_roll  = rollFilter.update(raw_roll);
}

#endif // IMU_NOTCH_FILTER_H
