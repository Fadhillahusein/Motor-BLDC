// Plain FOC bring-up for B-G431B-ESC1 + SunnySky X4108S (no load) with SimpleFOC.
// NOT compiled or tested on hardware as a whole: STAGE 2 is your tested AS5600 sketch, the rest is new.
//
// Work through the stages IN ORDER. Change STAGE, rebuild, pass the check, then move on.
//   STAGE 1: open-loop spin   (no sensor, proves driver + motor wiring)
//   STAGE 2: read AS5600      (your tested sketch; turn the shaft by hand, no motor drive)
//   STAGE 3: closed-loop velocity FOC (voltage mode, uses the AS5600 angle)
#include <SimpleFOC.h>
#include <Arduino.h>

#define STAGE 3

#define POLE_PAIRS 11       // pole PAIRS (not poles). 11 matches Rama's thesis table (Tabel 3.1).
#define SUPPLY_V   13.89f   // [V] battery pack voltage, measured with a multimeter. SimpleFOC uses it to turn volts into PWM duty.
                            // Re-measure before each session (a battery drops as it discharges) and update this number.
                            // A battery has NO current limit: put a 5 A fuse in series with the + wire.

// --- Safe voltages come from the motor resistance, NOT from a fixed number ---
// Idea: at standstill / low speed the phase current is about  V / Rph  (Rph = resistance of ONE phase).
// So choose the current you accept, then compute the voltage:  V = I * Rph.
#define R_LL     0.285f     // [ohm] resistance BETWEEN TWO motor wires. 0.285 = Rama's table value (not measured by us).
                            // If Rama's value is per phase, the real R_LL is 0.57 and every current below is half. MEASURE IT.
#define I_ALIGN  4.0f       // [A] phase current accepted during alignment (initFOC)
#define I_RUN    6.0f       // [A] phase current accepted if the rotor is blocked while running
#define V_BEMF   0.4f      // [V] room for back-EMF at about 20 rad/s (about 0.3 V for 380 KV, rounded up)
const float R_PH    = R_LL / 2.0f;              // wye motor: one phase = half of the wire-to-wire value
const float V_ALIGN = I_ALIGN * R_PH;           // 4 A * 0.1425 ohm = 0.57 V
const float V_RUN   = I_RUN * R_PH + V_BEMF;    // 6 A * 0.1425 ohm + 0.4 V = 1.26 V

// Pin names: newer board packages use A_PHASE_*, older custom variants use PHASE_*.
#if defined(A_PHASE_UH)
  #define P_UH A_PHASE_UH
  #define P_UL A_PHASE_UL
  #define P_VH A_PHASE_VH
  #define P_VL A_PHASE_VL
  #define P_WH A_PHASE_WH
  #define P_WL A_PHASE_WL
#else
  #define P_UH PHASE_UH
  #define P_UL PHASE_UL
  #define P_VH PHASE_VH
  #define P_VL PHASE_VL
  #define P_WH PHASE_WH
  #define P_WL PHASE_WL
#endif

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(P_UH, P_UL, P_VH, P_VL, P_WH, P_WL);

// AS5600 on I2C (default Wire pins, as in your tested sketch).
// Also: in most cases you need external pull-ups on SDA and SCL.
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

Commander command = Commander(Serial);

bool drv_ok = false;   // becomes true only if driver.init() succeeded (checked in loop, so a failure cannot be missed)
int  foc_ok = -1;      // STAGE 3: result of initFOC(). 1 = alignment OK, 0 = failed (motor disabled)

float target = 10.0f;  // rad/s (mechanical). Change at run time: type  T20  then Enter
void doTarget(char* cmd) { command.scalar(&target, cmd); }

void setup() {
  Serial.begin(115200);
  command.add('T', doTarget, "target velocity [rad/s]");

#if STAGE >= 2
  // configure I2C, then initialise the magnetic sensor hardware (same order as your tested sketch)
  Wire.setClock(400000);
  sensor.init();
#endif

#if STAGE == 2
  Serial.println("Sensor ready");
  _delay(1000);
  return;  // sensor only; no driver, no motor
#endif

  SimpleFOCDebug::enable(&Serial);     // print SimpleFOC's own init messages (driver/motor), to find where it fails
  _delay(2000);                        // time to open the Serial Monitor before init starts
  driver.voltage_power_supply = SUPPLY_V;
  drv_ok = driver.init();                          // 1 = PWM timers started, 0 = failed
  if (!drv_ok) { Serial.println("driver.init() FAILED"); return; }
  motor.linkDriver(&driver);

#if STAGE == 1
  motor.voltage_limit = V_RUN;                      // [V] from the resistance above. Raise slowly (0.1 V steps).
  motor.controller = MotionControlType::velocity_openloop;
  motor.init();
  motor.initFOC();                                  // SimpleFOC v2.4: needed even in open loop (found on the bench)
#elif STAGE == 3
  motor.linkSensor(&sensor);
  // --- parameters (see the table in the chat for what each one means) ---
  motor.voltage_sensor_align = V_ALIGN; // [V] used ONLY inside initFOC(). Library default is 3 V = too high for a low-R motor.
  motor.voltage_limit = V_RUN;         // [V] max voltage the controller may apply. Raise slowly if the motor stalls.
  motor.velocity_limit = 30.0f;        // [rad/s] mechanical speed ceiling (30 rad/s = about 290 rpm). Raise later, together with V_RUN.
  motor.controller = MotionControlType::velocity;   // closed-loop speed control (needs the sensor)
  motor.torque_controller = TorqueControlType::voltage; // the speed PI outputs a VOLTAGE, not a current
  motor.PID_velocity.P = 0.05f;        // [V per rad/s of speed error]. Docs value 0.2 gives ~2-4 deg phase margin with this motor (rings).
  motor.PID_velocity.I = 1.0f;         // [V per rad]. With P=0.05: ~55 deg phase margin, ~10-20 Hz speed loop (model estimate).
  motor.PID_velocity.D = 0.0f;
  motor.LPF_velocity.Tf = 0.01f;       // [s] speed filter time constant, cutoff = 1/(2*pi*Tf) = about 16 Hz
  motor.init();
  foc_ok = motor.initFOC();            // finds the electrical zero offset and the direction (the rotor will move a little)
  if (!foc_ok) Serial.println("initFOC() FAILED - alignment did not work, the motor is disabled");
#endif
  Serial.println("ready");
}

void loop() {
#if STAGE != 2
  if (!drv_ok) {                                    // driver not started: say so every second, never pretend to run
    static uint32_t tf = 0;
    if (millis() - tf > 1000) { tf = millis(); Serial.println("DRIVER NOT INITIALISED - the motor is NOT driven"); }
    return;
  }
#endif
#if STAGE == 2
  // iterative function updating the sensor internal variables (motor.loopFOC() does this in STAGE 3)
  sensor.update();
  Serial.print(sensor.getMechanicalAngle() * 180.0f / PI);  // 0 to 360 degrees
  Serial.print("\t");
  Serial.println(sensor.getVelocity());                      // rad/s
#elif STAGE == 1
  motor.loopFOC();                                  // v2.4: this call applies the phase voltages; without it Ua=Ub=Uc=0
  motor.move(target);
  command.run();
  static uint32_t t1 = 0;
  if (millis() - t1 > 500) {                        // every 0.5 s: commanded speed and the open-loop angle (rad)
    t1 = millis();
    Serial.print("T="); Serial.print(target); Serial.print("  angle="); Serial.println(motor.shaft_angle);
  }
#elif STAGE == 3
  motor.loopFOC();                                  // the FOC step. Call it as often as possible.
  motor.move(target);
  command.run();
  static uint32_t t0 = 0;
  if (millis() - t0 > 100) {                        // print target and measured speed for logging
    t0 = millis();
    // columns: target [rad/s], measured speed [rad/s], Uq [V] (controller output), foc_ok, enabled
    Serial.print(target); Serial.print(',');
    Serial.print(motor.shaft_velocity); Serial.print(',');
    Serial.print(motor.voltage.q); Serial.print(',');
    Serial.print(foc_ok); Serial.print(',');
    Serial.println(motor.enabled);
  }
#endif
}