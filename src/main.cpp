// =============================================================================
//  HFI bring-up — B-G431B-ESC1 + SunnySky X4108S (Rama's rig)
//
//  v11 (2026-11-27): spin PLL faster (15 Hz instead of 3 Hz) so it can follow the jerky
//     cogging motion; speed loop a bit faster; safety trip needs 3 bad encoder samples
//     in a row (one I2C glitch no longer stops the run).
//
//  v10 (2026-11-27): spin polarity threshold |z| >= 2.5 (n keeps 3.0). Weak spot gave
//     z = -3.0 rounded after 180 rounds, same sign in every block.
//
//  v9 (2026-11-27): spin polarity keeps adding rounds (60 at a time, up to 180) until
//     the decision is clear. z grows with sqrt(rounds), so a weak position still works.
//
//  v8 (2026-11-27): spin uses the SAME polarity routine as n (+d -d +q -q, 60 rounds,
//     decide only if |z| >= 3 and d difference > 2x q control). The quick 20-round
//     +d/-d version was too weak (z ~ -0.5).
//
//  v7 (2026-11-26): command s = first SPIN WITH HFI (rotor FREE, nothing attached).
//     1) HFI locks (iq = 0), 2) quick polarity (20 pulse pairs), 3) slow speed control
//     using ONLY the HFI angle and the HFI speed: +30, 0, -30 rad/s el (~26 rpm mech).
//     Current loop runs once per HF period on period-averaged currents (the average
//     over exactly one 1 kHz period removes the injection). The HFI error signal is
//     also integrated over exactly one period, which cancels the torque current.
//     The encoder only watches: it is logged and it trips the run if HFI and encoder
//     disagree by > 60 deg el or the speed gets too high.
//
//  v6 (2026-11-26): better encoder offset (command a). Stronger pull (0.6 V, ~2 A)
//     against cogging, and 8 angles FORWARD then 8 BACKWARD. The rotor stops a bit
//     short in the direction it came from; forward and backward errors point opposite
//     ways, so their average cancels it.
//
//  v5 (2026-11-26): Uh default 0.3 V, B_PER_VOLT = 0.17 (sweep at 0.3 V),
//     new command m = automatic accuracy map (rotor FREE, no hand clamp):
//     a DC voltage drags the rotor to 8 electrical angles (45 deg apart), then it is
//     released, HFI locks, and the HFI axis is compared with the encoder truth.
//     Needs the encoder offset first (o -28.1).
//
//  v4 (2026-11-26): PWM-centre fix.
//   - driver.voltage_limit = supply voltage, so SimpleFOC centres every phase at
//     Vdc/2 (~50 % duty) instead of 0.5 V (3.6 % duty, where the 0.8 us dead time
//     ate the pulses and the phases clipped at 0 V).
//   - Safety moved to a software clamp: setV() limits |Ud,Uq| to U_SAFE = 1 V.
//     Every voltage command in this file goes through setV().
//   - Polarity test now reports how far the rotor moved (hand clamp check).
//   - Uh range widened to 0.6 V (still under U_SAFE).
//   - After the first new sweep: update B_PER_VOLT (PLL gain depends on it).
//
//  MODE 2 (command g): saliency sweep. Rotor clamped. Inject Uh*cos(wh*t) along
//          a FIXED angle th_hat, demodulate, step th_hat 0..2*pi.
//          RESULT 2026-11-20: real saliency confirmed (pattern follows rotor at 2x).
//  MODE 3 (command p): PLL at standstill. Rotor clamped. The PLL steers th_hat
//          by itself from 4 different start angles. PASS = all runs settle at
//          the same angle (mod 180 deg) and w_hat -> 0.
//  MODE 3b (command t): PLL tracking. Unclamp, turn the rotor SLOWLY by hand.
//          PASS = th_hat follows the encoder (mod 180 deg, plus a constant offset).
//
//  MODE 4a (command a): ALIGN. Rotor FREE. A small DC voltage vector drags the rotor
//          to known electrical angles. Gives the encoder offset = encoder angle minus
//          true d-axis angle (this is the "constant 26 deg" we saw in tracking).
//  MODE 4b (command n): POLARITY. Rotor clamped. PLL locks, then +/- DC voltage pulses
//          along the locked axis. The pulse that adds to the magnet flux saturates the
//          iron, lowers L, and makes the current rise faster -> smaller rise time tau.
//          A q-axis pulse pair is the control (should show no difference).
//          The answer is checked against the encoder truth from Mode 4a.
//
//  Error signal (from the sweep data): e = LPF( 2*cos(wh*t) * iq_hat ).
//  With POSITIVE PLL gain it locks on the axis where |Xd| is largest (checked
//  on both sweep runs: lock 151.5 vs |Xd| max 149.2 deg, and 35.1 vs 37.4 deg).
//
//  Safety  : software clamp U_SAFE = 1 V on |Ud,Uq| (setV), 3 A trip, 5 A fuse.
//            Only HF voltage is applied (Uq = 0): no torque command at all.
//            Gates are OFF except while a test is running.
//  Encoder : Rama's setup counts opposite to the electrical angle, so in PLL logs
//            enc_el = normalize(-11 * mechanical angle). Constant offset unknown.
//
//  SERIAL COMMANDS (1000000 baud, send with Enter):
//    ?        settings + loop rate (gates stay off)
//    u 0.1    set Uh [V]   (0.02 .. 0.6)
//    f 1000   set fh [Hz]  (200 .. 2500)
//    g        Mode 2: saliency sweep (2 sweeps)
//    p        Mode 3: PLL at standstill, 4 start angles x 2 s
//    t        Mode 3b: PLL tracking, 15 s, logs the encoder
//    a        Mode 4a: align + encoder offset (rotor FREE, ~8 s)
//    o 12.3   set the encoder offset by hand [deg] (after a power cycle)
//    n        Mode 4b: polarity test (rotor clamped, ~5 s)
//    m        Accuracy map: 8 positions, automatic (rotor FREE, ~40 s)
//    s        Spin with HFI: +30 / 0 / -30 rad/s el (rotor FREE, ~20 s)
// =============================================================================
#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/as5600/MagneticSensorAS5600.h"

// ---------------- Hardware (copied from Rama) --------------------------------
BLDCMotor motor = BLDCMotor(11);                       // 11 pole pairs
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL,
                                       A_PHASE_VH, A_PHASE_VL,
                                       A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003f, -64.0f / 7.0f,
                                       A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
MagneticSensorAS5600 sensor_as5600;                    // reference only

// ---------------- Test parameters (changeable by serial command) -------------
float uh_v  = 0.30f;               // HF voltage amplitude [V] (0.3 V: 2.5x bigger signal, 26 Nov)
float fh_hz = 1000.0f;             // HF frequency [Hz]

// Sweep (Mode 2)
const int   N_STEPS     = 72;
const float STEP_TIME_S = 0.10f;
const int   N_SWEEPS    = 2;

// PLL (Mode 3) — values introduced here, derived from the sweep:
//   measured error-signal amplitude B ~ 0.18 A per volt of Uh,
//   small-angle slope Ke = 2*B*Uh  [A/rad],
//   closed loop s^2 + Ke*Kp*s + Ke*Ki  ->  Ke*Ki = wn^2, Ke*Kp = 2*zeta*wn.
const float B_PER_VOLT  = 0.17f;   // [A/V] from the 2026-11-26 sweep at Uh = 0.3 V
const float PLL_FN_HZ   = 3.0f;    // PLL natural frequency [Hz] (slow = robust)
const float PLL_ZETA    = 0.9f;    // damping
const float LPF_FC_HZ   = 50.0f;   // demodulation low-pass cutoff [Hz] (removes 2*fh ripple)
const float W_MAX       = 100.0f;  // clamp on w_hat [rad/s electrical]
const int   LOG_N       = 450;     // RAM log size (450 x 28 B = 12.6 KB)

// Common
const float I_TRIP_A    = 3.0f;
const float L_NOM_H     = 41.39e-6f;
const float R_NOM_OHM   = 0.285f;
const float PWM_HZ      = 25000.0f;
const float V_SUPPLY    = 13.8f;   // MEASURE your battery and put the real value here
const float U_SAFE      = 1.0f;    // software clamp on |Ud,Uq| [V] (replaces the old driver clamp)
const float ENC_MOVE_MAX_DEG = 10.0f; // polarity: max allowed rotor motion [deg el] (~0.9 deg mech)

// Mode 4 (new values, background in the reply):
const float V_ALIGN   = 0.40f;     // DC voltage for the map drag [V] -> ~1.4 A (V/R)
const float V_ALIGN2  = 0.60f;     // DC voltage for encoder-offset align [V] -> ~2.1 A
const float V_POL     = 0.60f;     // polarity pulse voltage [V]   -> ~2.1 A final value
const unsigned long T_POL_US = 1000;  // pulse length [us]; 7 time constants (L/R = 145 us)
const int   N_POL     = 60;        // number of +/- pulse pairs

// ---------------- State -------------------------------------------------------
float off_a = 0, off_b = 0, off_c = 0;                 // residual current offsets [A]
float enc_off = 0; bool have_off = false;              // encoder offset from Mode 4a [rad]
float th_last = 0;                                     // last th_hat of runPLL [rad]

struct LogPt { float t, th, w, e, enc, iq, x; };
LogPt logbuf[LOG_N];

// ---------------- Helpers -----------------------------------------------------
// The ONLY place that talks to SimpleFOC's setPhaseVoltage. Clamps |Ud,Uq| to U_SAFE.
static inline void setV(float uq, float ud, float th) {
  const float mag = sqrtf(ud * ud + uq * uq);
  if (mag > U_SAFE) { const float k = U_SAFE / mag; ud *= k; uq *= k; }
  motor.setPhaseVoltage(uq, ud, th);                  // SimpleFOC order: (Uq, Ud, angle)
}
static inline PhaseCurrent_s readI() {                 // currents with residual offset removed
  PhaseCurrent_s c = current_sense.getPhaseCurrents();
  c.a -= off_a; c.b -= off_b; c.c -= off_c;
  return c;
}
static inline void clarke(const PhaseCurrent_s &c, float &ia, float &ib) {
  ia = c.a;
  ib = 0.57735026919f * (c.a + 2.0f * c.b);           // (a + 2b)/sqrt(3)
}
static inline float imaxOf(const PhaseCurrent_s &c) {
  return fmaxf(fabsf(c.a), fmaxf(fabsf(c.b), fabsf(c.c)));
}
static inline float wrapPi(float x) {                  // wrap to (-pi, pi]
  return x - 2.0f * PI * floorf((x + PI) / (2.0f * PI));
}
float encElRev() {                                     // electrical angle, reversed direction
  sensor_as5600.update();
  return _normalizeAngle(-11.0f * sensor_as5600.getMechanicalAngle());
}

float encAvg() {                                       // circular mean of 20 encoder readings [rad el]
  float x = 0, y = 0;
  for (int i = 0; i < 20; i++) { const float e = encElRev(); x += cosf(e); y += sinf(e); delay(2); }
  return atan2f(y, x);
}
static inline float wrapHalfPi(float x) {              // wrap to (-pi/2, pi/2]: axis error, N/S ignored
  return x - PI * floorf((x + 0.5f * PI) / PI);
}

void stopAll(const char *why) {
  setV(0, 0, 0);
  motor.disable();
  Serial.print(F("# STOP: ")); Serial.println(why);
  Serial.println(F("# Power-cycle the board to continue."));
  while (1) {}
}

float measureLoopRate() {
  unsigned long t0 = micros(); long n = 0;
  while (micros() - t0 < 200000UL) {
    PhaseCurrent_s c = current_sense.getPhaseCurrents(); (void)c;
    setV(0, 0, 0);
    n++;
  }
  return n / 0.2f;
}

void printInfo() {
  float rate = measureLoopRate();
  float wh   = 2.0f * PI * fh_hz;
  float fs   = fminf(rate, PWM_HZ);
  float zmag = sqrtf(R_NOM_OHM * R_NOM_OHM + (wh * L_NOM_H) * (wh * L_NOM_H));
  Serial.print(F("# Uh [V] = "));                Serial.println(uh_v, 3);
  Serial.print(F("# fh [Hz] = "));               Serial.println(fh_hz, 0);
  Serial.print(F("# loop rate [Hz] = "));        Serial.println(rate, 0);
  Serial.print(F("# samples per HF period = ")); Serial.println(fs / fh_hz, 1);
  Serial.print(F("# predicted HF current amplitude [A] = ")); Serial.println(uh_v / zmag, 3);
  Serial.print(F("# PWM centre duty [%] = "));  Serial.println(100.0f * 0.5f * driver.voltage_limit / V_SUPPLY, 1);
  Serial.print(F("# software clamp U_SAFE [V] = ")); Serial.println(U_SAFE, 2);
  if (fs / fh_hz < 10.0f) Serial.println(F("# WARNING: <10 samples/period, lower fh"));
}

void printHelp() {
  Serial.println(F("# Commands: ? | u <V> | f <Hz> | g (sweep) | p (PLL standstill) | t (PLL tracking) | a (align) | o <deg> | n (polarity) | m (accuracy map) | s (spin with HFI)"));
}

// ---- Residual offset calibration: gates on, zero voltage, no current ----
void calibOffsets() {
  off_a = off_b = off_c = 0;
  setV(0, 0, 0);
  delay(50);
  float sa = 0, sb = 0, sc = 0; long m = 0;
  unsigned long t0 = micros();
  while (micros() - t0 < 200000UL) {
    PhaseCurrent_s c = current_sense.getPhaseCurrents();
    sa += c.a; sb += c.b; sc += c.c; m++;
  }
  off_a = sa / m; off_b = sb / m; off_c = sc / m;
  Serial.print(F("# residual offsets a,b,c [A] = "));
  Serial.print(off_a, 4); Serial.print(','); Serial.print(off_b, 4); Serial.print(',');
  Serial.println(off_c, 4);
}

// ---- DC check: phase mapping (expect ia>0 largest, ib~ic~-ia/2, sum~0) ----
void dcCheck() {
  unsigned long t0 = micros();
  float sa = 0, sb = 0, sc = 0; long m = 0;
  while (micros() - t0 < 300000UL) {
    setV(0.0f, 0.1f, 0.0f);
    PhaseCurrent_s c = readI();
    if (imaxOf(c) > I_TRIP_A) stopAll("overcurrent in DC check");
    if (micros() - t0 > 100000UL) { sa += c.a; sb += c.b; sc += c.c; m++; }
  }
  setV(0, 0, 0);
  Serial.print(F("# DC check ia,ib,ic,sum [A] = "));
  Serial.print(sa / m, 3); Serial.print(','); Serial.print(sb / m, 3); Serial.print(',');
  Serial.print(sc / m, 3); Serial.print(','); Serial.println((sa + sb + sc) / m, 3);
  delay(300);
}

// =============================== MODE 2: SWEEP ===============================
void runSweep(int sweep) {
  const float wh = 2.0f * PI * fh_hz;
  for (int k = 0; k < N_STEPS; k++) {
    const float th_hat = _2PI * k / N_STEPS;
    const float c_h = _cos(th_hat), s_h = _sin(th_hat);
    float xd_s = 0, xd_c = 0, xq_s = 0, xq_c = 0, i_peak = 0;
    long  n = 0;
    const unsigned long t_start = micros();
    while (micros() - t_start < (unsigned long)(STEP_TIME_S * 1e6f)) {
      const float t   = (micros() - t_start) * 1e-6f;
      const float ph  = wh * t; 
      const float sph = _sin(ph), cph = _cos(ph);
      setV(0.0f, uh_v * cph, th_hat);
      PhaseCurrent_s c = readI();
      float ia, ib; clarke(c, ia, ib);
      const float id_h =  ia * c_h + ib * s_h;
      const float iq_h = -ia * s_h + ib * c_h;
      const float imax = imaxOf(c);
      if (imax > i_peak) i_peak = imax;
      if (imax > I_TRIP_A) stopAll("overcurrent");
      xd_s += 2.0f * id_h * sph;  xd_c += 2.0f * id_h * cph;
      xq_s += 2.0f * iq_h * sph;  xq_c += 2.0f * iq_h * cph;
      n++;
    }
    setV(0, 0, th_hat);
    sensor_as5600.update();
    const float th_enc_el = _normalizeAngle(sensor_as5600.getMechanicalAngle() * 11.0f); // same as earlier logs
    Serial.print(sweep);        Serial.print(',');
    Serial.print(th_hat, 4);    Serial.print(',');
    Serial.print(xd_s / n, 5);  Serial.print(',');
    Serial.print(xd_c / n, 5);  Serial.print(',');
    Serial.print(xq_s / n, 5);  Serial.print(',');
    Serial.print(xq_c / n, 5);  Serial.print(',');
    Serial.print(n);            Serial.print(',');
    Serial.print(th_enc_el, 4); Serial.print(',');
    Serial.println(i_peak, 3);
  }
}

void testSweep() {
  Serial.println(F("# ---- MODE 2 SWEEP (rotor clamped) ----"));
  printInfo();
  motor.enable();
  calibOffsets();
  dcCheck();
  Serial.println(F("START_LOG"));
  Serial.println(F("sweep,th_hat,Xd_s,Xd_c,Xq_s,Xq_c,n,th_enc_el,i_peak"));
  for (int s = 0; s < N_SWEEPS; s++) runSweep(s);
  setV(0, 0, 0);
  motor.disable();
  Serial.println(F("STOP_LOG"));
}

// ================================ MODE 3: PLL =================================
// Runs the HFI PLL for T seconds starting from th0. Logs into RAM every log_dt_s.
// If with_enc is true, the encoder is read at each log point (costs ~0.5 ms).
// Returns the number of logged points.
int runPLL(float th0, float T, float log_dt_s, bool with_enc) {
  const float wh  = 2.0f * PI * fh_hz;
  const float Ke  = 2.0f * B_PER_VOLT * uh_v;          // [A/rad]
  const float wn  = 2.0f * PI * PLL_FN_HZ;
  const float Ki  = wn * wn / Ke;
  const float Kp  = 2.0f * PLL_ZETA * wn / Ke;
  const float tau = 1.0f / (2.0f * PI * LPF_FC_HZ);

  float th_hat = _normalizeAngle(th0), w_hat = 0, e_f = 0, integ = 0;
  int nlog = 0;
  const unsigned long t0 = micros();
  unsigned long tprev = t0, tlog = t0;
  const unsigned long Tus = (unsigned long)(T * 1e6f), LOGus = (unsigned long)(log_dt_s * 1e6f);

  while (true) {
    const unsigned long now = micros();
    if (now - t0 >= Tus) break;
    float dt = (now - tprev) * 1e-6f; tprev = now;
    if (dt <= 0 || dt > 0.001f) dt = 1e-5f;            // guard after slow steps (encoder read)
    const float t  = (now - t0) * 1e-6f;
    const float ch = _cos(wh * t);

    // 1) inject along the current estimate
    setV(0.0f, uh_v * ch, th_hat);

    // 2) measure, rotate into d^q^ (only q^ is needed)
    PhaseCurrent_s c = readI();
    if (imaxOf(c) > I_TRIP_A) stopAll("overcurrent in PLL");
    float ia, ib; clarke(c, ia, ib);
    const float iq_h = -ia * _sin(th_hat) + ib * _cos(th_hat);

    // 3) demodulate (cos, as found in the sweep) + low-pass
    const float prod = 2.0f * iq_h * ch;
    e_f += (dt / (tau + dt)) * (prod - e_f);

    // 4) PI -> speed, integrate -> angle (positive gain locks on the |Xd|-max axis)
    integ += Ki * e_f * dt;
    integ  = _constrain(integ, -W_MAX, W_MAX);
    w_hat  = _constrain(Kp * e_f + integ, -W_MAX, W_MAX);
    th_hat = _normalizeAngle(th_hat + w_hat * dt);

    // 5) log
    if (nlog < LOG_N && (now - tlog >= LOGus || nlog == 0)) {
      tlog = now;
      logbuf[nlog].t = t; logbuf[nlog].th = th_hat; logbuf[nlog].w = w_hat; logbuf[nlog].e = e_f;
      logbuf[nlog].enc = with_enc ? encElRev() : NAN;
      nlog++;
    }
  }
  th_last = th_hat;
  setV(0, 0, 0);
  return nlog;
}

void dumpLog(int run, int nlog) {
  for (int i = 0; i < nlog; i++) {
    Serial.print(run);              Serial.print(',');
    Serial.print(logbuf[i].t, 4);   Serial.print(',');
    Serial.print(logbuf[i].th, 4);  Serial.print(',');
    Serial.print(logbuf[i].w, 3);   Serial.print(',');
    Serial.print(logbuf[i].e, 5);   Serial.print(',');
    Serial.println(logbuf[i].enc, 4);
  }
}

void printPllGains() {
  const float Ke = 2.0f * B_PER_VOLT * uh_v, wn = 2.0f * PI * PLL_FN_HZ;
  Serial.print(F("# PLL Ke [A/rad] = ")); Serial.println(Ke, 4);
  Serial.print(F("# PLL Kp = "));         Serial.println(2.0f * PLL_ZETA * wn / Ke, 1);
  Serial.print(F("# PLL Ki = "));         Serial.println(wn * wn / Ke, 1);
}

void testPLLStandstill() {
  Serial.println(F("# ---- MODE 3 PLL STANDSTILL (rotor clamped) ----"));
  printInfo(); printPllGains();
  motor.enable();
  calibOffsets();
  Serial.print(F("# encoder el (reversed) before = ")); Serial.println(encElRev(), 4);
  Serial.println(F("START_LOG"));
  Serial.println(F("run,t,th_hat,w_hat,e,enc_el"));
  const float starts[4] = {0.0f, 0.5f * PI, PI, 1.5f * PI};
  for (int r = 0; r < 4; r++) {
    int n = runPLL(starts[r], 2.0f, 0.01f, false);    // 2 s, log every 10 ms
    dumpLog(r, n);
  }
  Serial.println(F("STOP_LOG"));
  Serial.print(F("# encoder el (reversed) after = ")); Serial.println(encElRev(), 4);
  motor.disable();
  Serial.println(F("# PASS if all 4 runs end at the same th_hat (mod 3.14) and w_hat ~ 0."));
}

void testPLLTracking() {
  Serial.println(F("# ---- MODE 3b PLL TRACKING: unclamp, wait 2 s, then turn SLOWLY by hand ----"));
  printInfo(); printPllGains();
  motor.enable();
  calibOffsets();
  Serial.println(F("START_LOG"));
  Serial.println(F("run,t,th_hat,w_hat,e,enc_el"));
  int n = runPLL(0.0f, 15.0f, 0.03f, true);           // 15 s, log every 30 ms with encoder
  dumpLog(9, n);
  Serial.println(F("STOP_LOG"));
  motor.disable();
  Serial.println(F("# PASS if th_hat follows enc_el (mod 3.14) with a constant offset."));
}


// ============================ MODE 4a: ALIGN v2 (rotor FREE) ===================
// A DC voltage pulls the magnet (d-axis) onto a chosen electrical angle. At rest:
// encoder angle - applied angle = encoder offset.
// Problem seen on 26 Nov: cogging + friction make the rotor stop a little short, and
// the offset changed between runs (-28, -33, -11 deg). Fix:
//   - stronger pull (V_ALIGN2 = 0.6 V, ~2 A),
//   - 8 angles going FORWARD, then 8 going BACKWARD; the stop-short error has opposite
//     sign in the two directions, so the average of all 16 cancels it.
float holdAndRead(float th, float V, unsigned long ms_total, unsigned long ms_avg, float &ipk) {
  float ex = 0, ey = 0; int ne = 0; ipk = 0;
  const unsigned long t0 = millis(); unsigned long tenc = 0;
  while (millis() - t0 < ms_total) {
    setV(0.0f, V, th);
    PhaseCurrent_s c = readI();
    const float im = imaxOf(c);
    if (im > I_TRIP_A) stopAll("overcurrent in align");
    if (im > ipk) ipk = im;
    if (millis() - t0 > ms_total - ms_avg && millis() - tenc >= 5) {
      tenc = millis();
      const float e = encElRev(); ex += cosf(e); ey += sinf(e); ne++;
    }
  }
  return atan2f(ey, ex);
}

void testAlign() {
  Serial.println(F("# ---- MODE 4a ALIGN v2: rotor must be FREE, do not touch it (~25 s) ----"));
  motor.enable();
  calibOffsets();
  const int NA = 8;
  float ipk;
  // pre-position: come to angle 0 from below, so the first point is approached FORWARD
  holdAndRead(_normalizeAngle(-_2PI / NA), V_ALIGN2, 800, 50, ipk);
  Serial.println(F("START_LOG"));
  Serial.println(F("k,dir,applied_deg,enc_el_deg,diff_deg,i_peak_A"));
  float fx = 0, fy = 0, bx = 0, by = 0;
  for (int pass = 0; pass < 2; pass++) {                 // 0 = forward, 1 = backward
    for (int j = 0; j < NA; j++) {
      const int idx = (pass == 0) ? j : (NA - 1 - j);
      const float th = _2PI * idx / NA;
      const float em = holdAndRead(th, V_ALIGN2, 1200, 400, ipk);
      const float diff = wrapPi(em - th);
      if (pass == 0) { fx += cosf(diff); fy += sinf(diff); }
      else           { bx += cosf(diff); by += sinf(diff); }
      Serial.print(pass * NA + j);          Serial.print(',');
      Serial.print(pass == 0 ? F("fwd") : F("back")); Serial.print(',');
      Serial.print(th * 57.2958f, 1);       Serial.print(',');
      Serial.print(em * 57.2958f, 1);       Serial.print(',');
      Serial.print(diff * 57.2958f, 1);     Serial.print(',');
      Serial.println(ipk, 2);
    }
    // before going backward: overshoot one step, so the first backward point comes from above
    if (pass == 0) holdAndRead(_2PI, V_ALIGN2, 800, 50, ipk);
  }
  setV(0, 0, 0);
  motor.disable();
  Serial.println(F("STOP_LOG"));
  const float off_f = atan2f(fy, fx), off_b = atan2f(by, bx);
  enc_off = atan2f(fy + by, fx + bx); have_off = true;
  Serial.print(F("# offset forward  [deg] = ")); Serial.println(off_f * 57.2958f, 1);
  Serial.print(F("# offset backward [deg] = ")); Serial.println(off_b * 57.2958f, 1);
  Serial.print(F("# forward - backward [deg] = ")); Serial.println(wrapPi(off_f - off_b) * 57.2958f, 1);
  Serial.print(F("# encoder offset (average, USED) [deg] = ")); Serial.println(enc_off * 57.2958f, 1);
  Serial.println(F("# Do NOT type o afterwards: this offset is already set. Next: m"));
}

// ============================ MODE 4b: POLARITY ================================
// v3: we no longer compare current AREAS (they were fooled by a ~1 A measurement bias
// that depends on the pulse direction). We compare the RISE TIME instead:
//     tau = integral( i_final - i(t) ) dt / i_final        [us]
// A constant offset cancels (it is in both i_final and i(t)); a gain error cancels
// (it is in the numerator and in the denominator). Physically tau ~ L/R, so the N side
// (saturated, smaller L) should give the SMALLER tau.
// CONTROL: the same pulses along the q axis (th_lock +-90 deg). Saturation is the same
// on both q directions, so any tau difference there is pure hardware asymmetry.
const int PSAMP = 128;
float firePulse(float th_dir, float V, float &i_ss) {
  const float c = _cos(th_dir), s = _sin(th_dir);
  setV(0, 0, th_dir);
  float b = 0; int nb = 0;
  unsigned long t0 = micros();
  while (micros() - t0 < 5000UL) {
    PhaseCurrent_s cc = readI(); float ia, ib; clarke(cc, ia, ib);
    b += ia * c + ib * s; nb++;
  }
  b /= (float)nb;

  static float ts[PSAMP], is_[PSAMP];
  int n = 0;
  const unsigned long tp = micros();
  while (n < PSAMP) {
    const unsigned long now = micros();
    if (now - tp >= T_POL_US) break;
    setV(0.0f, V, th_dir);
    PhaseCurrent_s cc = readI();
    if (imaxOf(cc) > I_TRIP_A) stopAll("overcurrent in polarity pulse");
    float ia, ib; clarke(cc, ia, ib);
    ts[n] = (float)(now - tp); is_[n] = ia * c + ib * s - b; n++;
  }
  setV(0, 0, th_dir);                   // zero volts: current decays in < 1 ms
  delay(10);

  float sum = 0; int m = 0;
  for (int i = 0; i < n; i++) if (ts[i] > 700.0f) { sum += is_[i]; m++; }
  i_ss = (m > 0) ? sum / (float)m : 0.0f;                // final value = mean of the last 0.3 ms
  if (i_ss < 0.3f) return NAN;
  float miss = 0, tprev = 0;
  for (int i = 0; i < n; i++) { miss += (i_ss - is_[i]) * (ts[i] - tprev); tprev = ts[i]; }
  return miss / i_ss;                                    // tau [us]
}

struct PairStat { float s = 0, s2 = 0; int n = 0;
  void add(float d) { s += d; s2 += d * d; n++; }
  float mean() const { return n ? s / n : 0; }
  float z() const { if (n < 3) return 0; float m = mean();
    float v = fmaxf(0.0f, (s2 - n * m * m) / (n - 1)); float se = sqrtf(v / n); return se > 0 ? m / se : 0; } };

void testPolarity() {
  Serial.println(F("# ---- MODE 4b POLARITY v3 (rotor CLAMPED) ----"));
  printInfo(); printPllGains();
  motor.enable();
  calibOffsets();

  runPLL(0.0f, 3.0f, 0.1f, false);                       // 1) lock the PLL (3 s)
  const float th_lock = th_last;
  float ex = 0, ey = 0;
  for (int i = 0; i < 20; i++) { const float e = encElRev(); ex += cosf(e); ey += sinf(e); delay(2); }
  const float enc_now = atan2f(ey, ex);

  Serial.println(F("START_LOG"));                         // 2) four pulses per round: +d -d +q -q
  Serial.println(F("k,tau_dp,tau_dm,tau_qp,tau_qm,iss_dp,iss_dm,iss_qp,iss_qm"));
  PairStat D, Q; float tdp = 0, tdm = 0, tqp = 0, tqm = 0;
  for (int k = 0; k < N_POL; k++) {
    float i1, i2, i3, i4;
    const float t1 = firePulse(th_lock, V_POL, i1);
    const float t2 = firePulse(_normalizeAngle(th_lock + PI), V_POL, i2);
    const float t3 = firePulse(_normalizeAngle(th_lock + 0.5f * PI), V_POL, i3);
    const float t4 = firePulse(_normalizeAngle(th_lock + 1.5f * PI), V_POL, i4);
    Serial.print(k);         Serial.print(',');
    Serial.print(t1, 1);     Serial.print(','); Serial.print(t2, 1); Serial.print(',');
    Serial.print(t3, 1);     Serial.print(','); Serial.print(t4, 1); Serial.print(',');
    Serial.print(i1, 3);     Serial.print(','); Serial.print(i2, 3); Serial.print(',');
    Serial.print(i3, 3);     Serial.print(','); Serial.println(i4, 3);
    if (isnan(t1) || isnan(t2) || isnan(t3) || isnan(t4)) continue;
    D.add(t1 - t2); Q.add(t3 - t4);
    tdp += t1; tdm += t2; tqp += t3; tqm += t4;
  }
  setV(0, 0, 0);
  float mx = 0, my = 0;                                  // encoder after the pulses
  for (int i = 0; i < 20; i++) { const float e = encElRev(); mx += cosf(e); my += sinf(e); delay(2); }
  const float moved_deg = wrapPi(atan2f(my, mx) - enc_now) * 57.2958f;
  motor.disable();
  Serial.println(F("STOP_LOG"));
  Serial.print(F("# rotor moved during pulses [deg el] = ")); Serial.print(moved_deg, 1);
  Serial.print(F("  (= ")); Serial.print(moved_deg / 11.0f, 2); Serial.println(F(" deg mech)"));
  if (fabsf(moved_deg) > ENC_MOVE_MAX_DEG)
    Serial.println(F("# WARNING: rotor moved too much -> clamp better and repeat; do not trust this run."));

  const float n = (float)D.n;
  Serial.print(F("# valid pulse rounds = ")); Serial.println(D.n);
  Serial.print(F("# th_lock [deg] = "));      Serial.println(th_lock * 57.2958f, 1);
  if (D.n < 20) { Serial.println(F("# DECISION: NONE (too few valid rounds)")); return; }
  Serial.print(F("# mean tau +d, -d [us] = ")); Serial.print(tdp / n, 1); Serial.print(", "); Serial.println(tdm / n, 1);
  Serial.print(F("# mean tau +q, -q [us] = ")); Serial.print(tqp / n, 1); Serial.print(", "); Serial.println(tqm / n, 1);
  Serial.print(F("# d-axis: tau(+d)-tau(-d) [us] = ")); Serial.print(D.mean(), 1);
  Serial.print(F("   z = ")); Serial.println(D.z(), 1);
  Serial.print(F("# q-axis CONTROL: tau(+q)-tau(-q) [us] = ")); Serial.print(Q.mean(), 1);
  Serial.print(F("   z = ")); Serial.println(Q.z(), 1);

  const bool strong = fabsf(D.z()) >= 3.0f && fabsf(D.mean()) > 2.0f * fabsf(Q.mean());
  if (!strong)
    Serial.println(F("# DECISION: NONE (d-axis difference is not clearly bigger than the q-axis control)."));
  else if (D.mean() < 0)
    Serial.println(F("# DECISION: +d_hat has the SMALLER tau (more saturated) -> th_lock points to the N pole."));
  else
    Serial.println(F("# DECISION: -d_hat has the SMALLER tau -> th_lock points to the S pole. Use th_lock + 180 deg."));

  if (have_off) {                                       // 3) compare with the encoder truth
    const float true_d = wrapPi(enc_now - enc_off);
    const float rel    = wrapPi(th_lock - true_d) * 57.2958f;
    Serial.print(F("# TRUTH: true d-axis [deg] = ")); Serial.println(true_d * 57.2958f, 1);
    Serial.print(F("# th_lock - true d [deg] = "));   Serial.println(rel, 1);
    Serial.println(F("#   near 0   -> locked on N pole (decision should say N)"));
    Serial.println(F("#   near 180 -> locked on S pole (decision should say S)"));
    Serial.println(F("#   near +-90 -> locked on the q-axis, not d (pulse test is then meaningless)"));
  } else {
    Serial.println(F("# (no encoder offset yet: run command a first to get the truth check)"));
  }
}

// ============================ ACCURACY MAP (command m) ========================
// Rotor FREE. For 8 target angles (45 deg el apart):
//   1) drag the rotor there with a DC voltage (like Mode 4a),
//   2) switch it off and let the rotor settle (it may snap to a cogging spot: fine,
//      the encoder measures where it really is),
//   3) let HFI lock from a start 57 deg away, so it has to find the axis by itself,
//   4) error = HFI axis - true d axis, wrapped to +-90 deg (N/S is ignored here;
//      polarity is checked separately with n).
void testMap() {
  Serial.println(F("# ---- ACCURACY MAP: rotor FREE, do not touch it (~40 s) ----"));
  if (!have_off) { Serial.println(F("# Need the encoder offset first: send  o -28.1")); return; }
  printInfo(); printPllGains();
  motor.enable();
  calibOffsets();
  Serial.println(F("START_LOG"));
  Serial.println(F("k,target_deg,true_d_deg,th_lock_deg,err_deg,moved_deg"));
  const int NPOS = 8;
  float sum = 0, sum2 = 0, emax = 0; int n = 0;
  for (int k = 0; k < NPOS; k++) {
    const float th = _2PI * k / NPOS;
    const unsigned long t0 = millis();                    // 1) drag
    while (millis() - t0 < 1000) {
      setV(0.0f, V_ALIGN, th);
      PhaseCurrent_s c = readI();
      if (imaxOf(c) > I_TRIP_A) stopAll("overcurrent in map drag");
    }
    setV(0, 0, 0);                                        // 2) release + settle
    delay(300);
    const float enc0 = encAvg();
    runPLL(_normalizeAngle(th + 1.0f), 3.0f, 0.1f, false); // 3) HFI lock (3 s)
    const float enc1 = encAvg();
    const float true_d = wrapPi(enc1 - enc_off);          // 4) compare
    const float err    = wrapHalfPi(th_last - true_d);
    const float moved  = wrapPi(enc1 - enc0);
    Serial.print(k);                          Serial.print(',');
    Serial.print(th * 57.2958f, 1);           Serial.print(',');
    Serial.print(true_d * 57.2958f, 1);       Serial.print(',');
    Serial.print(th_last * 57.2958f, 1);      Serial.print(',');
    Serial.print(err * 57.2958f, 1);          Serial.print(',');
    Serial.println(moved * 57.2958f, 1);
    sum += err; sum2 += err * err; n++;
    if (fabsf(err) > emax) emax = fabsf(err);
  }
  setV(0, 0, 0);
  motor.disable();
  Serial.println(F("STOP_LOG"));
  const float m  = sum / n;
  const float sd = sqrtf(fmaxf(0.0f, sum2 / n - m * m));
  Serial.print(F("# mean error [deg el] = "));      Serial.println(m * 57.2958f, 1);
  Serial.print(F("# spread (std) [deg el] = "));    Serial.println(sd * 57.2958f, 1);
  Serial.print(F("# worst |error| [deg el] = "));   Serial.println(emax * 57.2958f, 1);
  Serial.println(F("# GOOD if worst |error| < ~10 deg. Check that true_d values spread around the circle."));
  Serial.println(F("# If moved_deg is large (> ~10), the rotor drifted during HFI: note which k."));
}

// ============================ MODE 5: SPIN WITH HFI (command s) ===============
const float W_REF      = 30.0f;              // speed reference [rad/s el] (= ~26 rpm mech)
const float IQ_MAX     = 0.40f;              // torque current limit [A] (small on purpose)
const float KP_W       = 0.003f;             // speed P [A per rad/s el] (first guess, kept slow:
                                             //  the speed loop must be slower than the 3 Hz PLL)
const float KI_W       = 0.015f;             // speed I [A per (rad/s el) per s] (first guess)
const float R_FF       = 0.25f;              // resistance for voltage feedforward [ohm] (26 Nov fit)
const float KI_C_STEP  = 0.05f;              // current I step [V per A, once per HF period]
const float UD_I_MAX   = 0.30f;              // limits of the current integrators [V]
const float UQ_I_MAX   = 0.50f;
const float ERR_TRIP   = 60.0f * PI / 180.0f;// stop if |HFI - encoder| > 60 deg el
const float WENC_TRIP  = 100.0f;             // stop if encoder speed > 100 rad/s el
const int   N_POL_SPIN = 60;                 // rounds for the polarity check (same as n)
const float T_SPIN     = 11.0f;              // total run time [s]
const float PLL_FN_SPIN = 15.0f;             // PLL natural frequency during the spin [Hz]
const int   TRIP_N      = 3;                 // bad samples in a row before the safety stop
const float Z_SPIN     = 2.5f;               // polarity confidence for the spin test (n uses 3.0;
                                             //  the encoder trip is the safety net here)

float wrefAt(float t) {                                 // speed profile [rad/s el]
  if (t < 1.0f)  return 0.0f;
  if (t < 5.0f)  return  W_REF;
  if (t < 6.0f)  return 0.0f;
  if (t < 10.0f) return -W_REF;
  return 0.0f;
}

// Polarity for the spin: same routine as command n (+d -d +q -q per round).
// Returns false if not clear.
bool polarityQuick(float th_lock, bool &isS) {
  PairStat D, Q;
  for (int block = 0; block < 3; block++) {
  for (int k = 0; k < N_POL_SPIN; k++) {
    float i1, i2, i3, i4;
    const float t1 = firePulse(th_lock, V_POL, i1);
    const float t2 = firePulse(_normalizeAngle(th_lock + PI), V_POL, i2);
    const float t3 = firePulse(_normalizeAngle(th_lock + 0.5f * PI), V_POL, i3);
    const float t4 = firePulse(_normalizeAngle(th_lock + 1.5f * PI), V_POL, i4);
    if (isnan(t1) || isnan(t2) || isnan(t3) || isnan(t4)) continue;
    D.add(t1 - t2); Q.add(t3 - t4);
  }
  Serial.print(F("# polarity: rounds = ")); Serial.print(D.n);
  Serial.print(F(", d diff [us] = ")); Serial.print(D.mean(), 1);
  Serial.print(F(" (z = ")); Serial.print(D.z(), 1);
  Serial.print(F("), q control [us] = ")); Serial.println(Q.mean(), 1);
  if (D.n >= 20 && fabsf(D.z()) >= Z_SPIN && fabsf(D.mean()) > 2.0f * fabsf(Q.mean())) {
    isS = D.mean() > 0;                                  // -d faster -> th_lock is on S
    return true;
  }
  if (block < 2) Serial.println(F("# not clear yet -> adding 60 more rounds"));
  }
  return false;
}

void dumpSpin(int nlog) {
  Serial.println(F("START_LOG"));
  Serial.println(F("t,w_ref,th_hat_deg,w_hat,w_enc,err_deg,iq_avg,iq_ref"));
  for (int i = 0; i < nlog; i++) {
    Serial.print(logbuf[i].t, 3);              Serial.print(',');
    Serial.print(wrefAt(logbuf[i].t), 1);      Serial.print(',');
    Serial.print(logbuf[i].th * 57.2958f, 1);  Serial.print(',');
    Serial.print(logbuf[i].w, 1);              Serial.print(',');
    Serial.print(logbuf[i].enc, 1);            Serial.print(',');
    Serial.print(logbuf[i].e * 57.2958f, 1);   Serial.print(',');
    Serial.print(logbuf[i].iq, 3);             Serial.print(',');
    Serial.println(logbuf[i].x, 3);
  }
  Serial.println(F("STOP_LOG"));
  // summary: mean speeds in the middle of each run phase, worst angle error
  float sp = 0, se = 0, sn = 0, sne = 0, emax = 0; int np = 0, nn = 0;
  for (int i = 0; i < nlog; i++) {
    const float t = logbuf[i].t;
    if (fabsf(logbuf[i].e) > emax) emax = fabsf(logbuf[i].e);
    if (t > 2.5f && t < 5.0f) { sp += logbuf[i].w; se += logbuf[i].enc; np++; }
    if (t > 7.5f && t < 10.0f) { sn += logbuf[i].w; sne += logbuf[i].enc; nn++; }
  }
  Serial.print(F("# worst |HFI - encoder| [deg el] = ")); Serial.println(emax * 57.2958f, 1);
  if (np) { Serial.print(F("# + phase: w_ref, mean w_hat, mean w_enc = ")); Serial.print(W_REF, 1);
            Serial.print(", "); Serial.print(sp / np, 1); Serial.print(", "); Serial.println(se / np, 1); }
  if (nn) { Serial.print(F("# - phase: w_ref, mean w_hat, mean w_enc = ")); Serial.print(-W_REF, 1);
            Serial.print(", "); Serial.print(sn / nn, 1); Serial.print(", "); Serial.println(sne / nn, 1); }
  Serial.println(F("# PASS if the rotor turned both ways, w_enc follows w_ref, and the angle error stays small."));
}

void testSpin() {
  Serial.println(F("# ---- MODE 5 SPIN WITH HFI: rotor FREE, nothing attached, hands off (~20 s) ----"));
  if (!have_off) { Serial.println(F("# Need the encoder offset first: run a (it is the safety judge).")); return; }
  printInfo(); printPllGains();
  motor.enable();
  calibOffsets();

  // 1) lock the axis with iq = 0
  runPLL(0.0f, 2.0f, 0.1f, false);
  float th_hat = th_last;

  // 2) polarity
  bool isS = false;
  if (!polarityQuick(th_hat, isS)) {
    setV(0, 0, 0); motor.disable();
    Serial.println(F("# ABORT: polarity not clear, no torque applied.")); return;
  }
  if (isS) th_hat = _normalizeAngle(th_hat + PI);
  {
    const float true_d = wrapPi(encAvg() - enc_off);
    Serial.print(F("# polarity says ")); Serial.print(isS ? F("S (+180)") : F("N"));
    Serial.print(F(", start error HFI - true d [deg el] = "));
    Serial.println(wrapPi(th_hat - true_d) * 57.2958f, 1);
  }

  // 3) spin: everything from HFI, encoder only watches
  const float wh = 2.0f * PI * fh_hz;
  const float Ke = 2.0f * B_PER_VOLT * uh_v;
  const float wn = 2.0f * PI * PLL_FN_SPIN;
  const float Ki = wn * wn / Ke, Kp = 2.0f * PLL_ZETA * wn / Ke;
  float w_hat = 0, integ = 0, iw = 0, iq_ref = 0, ud_i = 0, uq_i = 0;
  float acc_e = 0, acc_d = 0, acc_q = 0, acc_T = 0, iq_avg = 0;
  long per_prev = 0; int nlog = 0; float enc_prev = NAN; int nbad = 0;
  const char *why = nullptr;
  const unsigned long t0 = micros(); unsigned long tprev = t0, tlog = t0;

  while (true) {
    const unsigned long now = micros();
    const float t = (now - t0) * 1e-6f;
    if (t >= T_SPIN) break;
    float dt = (now - tprev) * 1e-6f; tprev = now;
    if (dt <= 0 || dt > 0.001f) dt = 1e-5f;
    const float ch = _cos(wh * t);

    // voltage = feedforward + current integrators + injection on d^
    setV(R_FF * iq_ref + uq_i, ud_i + uh_v * ch, th_hat);

    PhaseCurrent_s c = readI();
    if (imaxOf(c) > I_TRIP_A) { why = "overcurrent"; break; }
    float ia, ib; clarke(c, ia, ib);
    const float sh = _sin(th_hat), chh = _cos(th_hat);
    const float id =  ia * chh + ib * sh;
    const float iq = -ia * sh  + ib * chh;

    // integrate over exactly one HF period (removes the injection from id/iq,
    // and removes the torque current from the HFI error)
    acc_e += 2.0f * iq * ch * dt; acc_d += id * dt; acc_q += iq * dt; acc_T += dt;
    th_hat = _normalizeAngle(th_hat + w_hat * dt);

    const long per = (long)(t * fh_hz);
    if (per != per_prev) {                               // once per HF period (1 kHz)
      per_prev = per;
      const float Tp = acc_T;
      const float e = acc_e / Tp, id_avg = acc_d / Tp; iq_avg = acc_q / Tp;
      acc_e = acc_d = acc_q = acc_T = 0;
      // PLL (same law as runPLL)
      integ = _constrain(integ + Ki * e * Tp, -W_MAX, W_MAX);
      w_hat = _constrain(Kp * e + integ, -W_MAX, W_MAX);
      // speed PI -> iq_ref
      const float werr = wrefAt(t) - w_hat;
      iw     = _constrain(iw + KI_W * werr * Tp, -IQ_MAX, IQ_MAX);
      iq_ref = _constrain(KP_W * werr + iw, -IQ_MAX, IQ_MAX);
      // current integrators (id_ref = 0)
      ud_i = _constrain(ud_i + KI_C_STEP * (0.0f - id_avg), -UD_I_MAX, UD_I_MAX);
      uq_i = _constrain(uq_i + KI_C_STEP * (iq_ref - iq_avg), -UQ_I_MAX, UQ_I_MAX);
    }

    // log + safety every 25 ms (encoder read ~0.5 ms)
    if (now - tlog >= 25000UL || nlog == 0) {
      const float enc = encElRev();
      const float true_d = wrapPi(enc - enc_off);
      const float aerr = wrapPi(th_hat - true_d);
      float wenc = 0;
      if (!isnan(enc_prev)) wenc = wrapPi(enc - enc_prev) / ((now - tlog) * 1e-6f);
      enc_prev = enc; tlog = now;
      if (nlog < LOG_N) {
        logbuf[nlog].t = t; logbuf[nlog].th = th_hat; logbuf[nlog].w = w_hat; logbuf[nlog].e = aerr;
        logbuf[nlog].enc = wenc; logbuf[nlog].iq = iq_avg; logbuf[nlog].x = iq_ref; nlog++;
      }
      nbad = (t > 0.2f && fabsf(aerr) > ERR_TRIP) ? nbad + 1 : 0;
      if (nbad >= TRIP_N) { why = "HFI and encoder disagree > 60 deg (3 samples in a row)"; break; }
      if (fabsf(wenc) > WENC_TRIP)            { why = "encoder speed too high"; break; }
    }
  }
  setV(0, 0, 0);
  motor.disable();
  if (why) { Serial.print(F("# SAFETY STOP: ")); Serial.println(why); }
  dumpSpin(nlog);
}

// ================================= SETUP/LOOP =================================
void setup() {
  Serial.begin(1000000);
  delay(1000);

  driver.voltage_power_supply = V_SUPPLY;
  driver.voltage_limit        = V_SUPPLY;             // v4: PWM centre at Vdc/2 (~50 % duty)
                                                      // safety is the software clamp in setV()
  driver.pwm_frequency        = 25000;
  if (!driver.init()) stopAll("driver init");
  motor.linkDriver(&driver);

  current_sense.linkDriver(&driver);
  current_sense.skip_align = true;
  if (!current_sense.init()) stopAll("current sense init");
  motor.linkCurrentSense(&current_sense);

  sensor_as5600.init();

  motor.voltage_limit = 1.0f;
  motor.controller    = MotionControlType::torque;
  motor.init();
  motor.disable();                                    // gates OFF until a command

  Serial.println(F("# HFI bring-up ready. Gates OFF."));
  printHelp();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '?') {
    printInfo(); printPllGains();
  } else if (c == 'u') {
    float v = Serial.parseFloat();
    if (v >= 0.02f && v <= 0.6f) { uh_v = v; Serial.print(F("# Uh set to ")); Serial.println(uh_v, 3); }
    else Serial.println(F("# Uh rejected: allowed 0.02 .. 0.6 V"));
  } else if (c == 'f') {
    float v = Serial.parseFloat();
    if (v >= 200.0f && v <= 2500.0f) { fh_hz = v; Serial.print(F("# fh set to ")); Serial.println(fh_hz, 0); }
    else Serial.println(F("# fh rejected: allowed 200 .. 2500 Hz"));
  } else if (c == 'g') {
    testSweep();
  } else if (c == 'p') {
    testPLLStandstill();
  } else if (c == 't') {
    testPLLTracking();
  } else if (c == 'a') {
    testAlign();
  } else if (c == 'o') {
    const float v = Serial.parseFloat();
    enc_off = wrapPi(v * PI / 180.0f); have_off = true;
    Serial.print(F("# encoder offset set to [deg] ")); Serial.println(enc_off * 57.2958f, 1);
  } else if (c == 'n') {
    testPolarity();
  } else if (c == 'm') {
    testMap();
  } else if (c == 's') {
    testSpin();
  } else if (c == '\r' || c == '\n' || c == ' ') {
    // ignore
  } else {
    printHelp();
  }
}