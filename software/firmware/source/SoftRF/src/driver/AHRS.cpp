/*
 * AHRS.cpp
 * Fork: LSM6DSO six-axis attitude for the Card Edition Mk III (Seeed X1).
 *
 * Driver is register level over Wire (no library), fusion is Mahony,
 * heading comes from GPS track (the BMM350 magnetometer is not used yet).
 * Output: GDL90 message 0x4C in the iLevil layout AvareX/ForeFlight read,
 * at AHRS_OUT_INTERVAL_MS, independent of the 1 Hz export loop.
 */

#include "../system/SoC.h"
#include "../../SoftRF.h"
#include "EEPROM.h"
#include "GNSS.h"
#include "AHRS.h"
#include "../protocol/data/GDL90.h"
#include <Wire.h>
#include <math.h>

const ahrs_chip_ops_t *ahrs_chip = NULL;
static const ahrs_chip_ops_t lsm6dso_ops = { "LSM6DSO" };

#if defined(ENABLE_AHRS)

#define LSM6DSO_ADDR        0x6A
#define LSM6DSO_WHO_AM_I    0x0F
#define LSM6DSO_ID          0x6C
#define LSM6DSO_CTRL1_XL    0x10
#define LSM6DSO_CTRL2_G     0x11
#define LSM6DSO_CTRL3_C     0x12
#define LSM6DSO_OUTX_L_G    0x22
#define LSM6DSO_OUTX_L_A    0x28

#define AHRS_SAMPLE_MS      10     /* ~104 Hz ODR, read every 10 ms */
#define AHRS_OUT_INTERVAL_MS 200   /* 5 Hz to the EFB */
#define AHRS_KP             0.8f
#define AHRS_KI             0.002f

/*
 * Axis mapping from sensor frame to aircraft frame (x forward, y right,
 * z down). Unknown board orientation: start with the card flat, long axis
 * along the fuselage. Adjust the signs here after a bench check.
 */
#define AHRS_MAP_X(ax, ay, az)  ( (ax))
#define AHRS_MAP_Y(ax, ay, az)  ( (ay))
#define AHRS_MAP_Z(ax, ay, az)  (-(az))

static bool          ahrs_present   = false;
static bool          ahrs_valid     = false;
static unsigned long ahrs_sample_ms = 0;
static unsigned long ahrs_out_ms    = 0;
static unsigned long ahrs_ok_ms     = 0;

static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   /* orientation quaternion */
static float ei_x = 0, ei_y = 0, ei_z = 0;      /* integral feedback */
static float roll_deg = 0, pitch_deg = 0, turn_dps = 0, slip_deg = 0, g_load = 1;

static bool lsm_write(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(LSM6DSO_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool lsm_read(uint8_t reg, uint8_t *dst, size_t n)
{
  Wire.beginTransmission(LSM6DSO_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t) LSM6DSO_ADDR, (uint8_t) n) != n) return false;
  for (size_t i = 0; i < n; i++) dst[i] = Wire.read();
  return true;
}

byte AHRS_setup(void)
{
  if (hw_info.model != SOFTRF_MODEL_CARD_MK3) {
    return IMU_NONE;
  }

  uint8_t id = 0;
  if (!lsm_read(LSM6DSO_WHO_AM_I, &id, 1) || id != LSM6DSO_ID) {
    Serial.print(F("LSM6DSO not found, WHO_AM_I = 0x")); Serial.println(id, HEX);
    return IMU_NONE;
  }

  lsm_write(LSM6DSO_CTRL3_C,  0x44);  /* BDU, register auto-increment */
  lsm_write(LSM6DSO_CTRL1_XL, 0x48);  /* accel 104 Hz, +/-4 g */
  lsm_write(LSM6DSO_CTRL2_G,  0x44);  /* gyro  104 Hz, 500 dps */

  ahrs_present = true;
  ahrs_chip = &lsm6dso_ops;
  Serial.println(F("LSM6DSO: AHRS enabled"));
  return IMU_LSM6DSO;
}

/* Mahony update, aircraft frame, gyro in rad/s, accel in g */
static void mahony(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm > 0.5f && norm < 1.5f) {           /* trust accel only near 1 g */
    ax /= norm; ay /= norm; az /= norm;
    /* estimated gravity direction from the quaternion (body frame) */
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    /* error is cross product between measured and estimated gravity */
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);
    ei_x += AHRS_KI * ex * dt; ei_y += AHRS_KI * ey * dt; ei_z += AHRS_KI * ez * dt;
    gx += AHRS_KP * ex + ei_x;
    gy += AHRS_KP * ey + ei_y;
    gz += AHRS_KP * ez + ei_z;
  }
  /* integrate rate of change of quaternion */
  float hd = 0.5f * dt;
  float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * hd;
  float dq1 = ( q0 * gx + q2 * gz - q3 * gy) * hd;
  float dq2 = ( q0 * gy - q1 * gz + q3 * gx) * hd;
  float dq3 = ( q0 * gz + q1 * gy - q2 * gx) * hd;
  q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;
  norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (norm > 0) { q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm; }
}

void AHRS_loop(void)
{
  if (!ahrs_present) return;

  unsigned long now = millis();
  if (now - ahrs_sample_ms < AHRS_SAMPLE_MS) return;
  float dt = (ahrs_sample_ms == 0) ? 0.01f : (now - ahrs_sample_ms) / 1000.0f;
  if (dt > 0.2f) dt = 0.2f;
  ahrs_sample_ms = now;

  uint8_t raw[12];
  if (!lsm_read(LSM6DSO_OUTX_L_G, raw, sizeof(raw))) {
    if (now - ahrs_ok_ms > 2000) ahrs_valid = false;
    return;
  }
  ahrs_ok_ms = now;

  int16_t gxr = (int16_t) (raw[1]  << 8 | raw[0]);
  int16_t gyr = (int16_t) (raw[3]  << 8 | raw[2]);
  int16_t gzr = (int16_t) (raw[5]  << 8 | raw[4]);
  int16_t axr = (int16_t) (raw[7]  << 8 | raw[6]);
  int16_t ayr = (int16_t) (raw[9]  << 8 | raw[8]);
  int16_t azr = (int16_t) (raw[11] << 8 | raw[10]);

  /* sensor frame: 500 dps -> 17.5 mdps/LSB, 4 g -> 0.122 mg/LSB */
  float sgx = gxr * 0.0175f, sgy = gyr * 0.0175f, sgz = gzr * 0.0175f;   /* deg/s */
  float sax = axr * 0.000122f, say = ayr * 0.000122f, saz = azr * 0.000122f; /* g */

  float gx = AHRS_MAP_X(sgx, sgy, sgz), gy = AHRS_MAP_Y(sgx, sgy, sgz), gz = AHRS_MAP_Z(sgx, sgy, sgz);
  float ax = AHRS_MAP_X(sax, say, saz), ay = AHRS_MAP_Y(sax, say, saz), az = AHRS_MAP_Z(sax, say, saz);

  mahony(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az, dt);

  /* aerospace Euler angles from the quaternion */
  roll_deg  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
  float sp  = 2.0f * (q0 * q2 - q3 * q1);
  if (sp > 1) sp = 1; if (sp < -1) sp = -1;
  pitch_deg = asinf(sp) * RAD_TO_DEG;
  turn_dps  = gz;
  /* lateral acceleration as a slip angle; one g sideways ~ 90 deg scale */
  slip_deg  = atan2f(ay, (az < 0 ? -az : 1.0f)) * RAD_TO_DEG;
  g_load    = -az;                      /* z is down in the aircraft frame */
  ahrs_valid = true;

  if (now - ahrs_out_ms >= AHRS_OUT_INTERVAL_MS && settings->gdl90 != GDL90_OFF) {
    ahrs_out_ms = now;
    uint8_t buf[64];
    size_t size = AHRS_GDL90(buf);
    GDL90_Out_Ext(buf, size);
  }
}

bool  AHRS_valid(void)     { return ahrs_present && ahrs_valid; }
float AHRS_roll(void)      { return roll_deg; }
float AHRS_pitch(void)     { return pitch_deg; }
float AHRS_turn_rate(void) { return turn_dps; }
float AHRS_slip(void)      { return slip_deg; }
float AHRS_g(void)         { return g_load; }

static void put16(uint8_t *p, int v)
{
  p[0] = (v >> 8) & 0xFF; p[1] = v & 0xFF;
}

/*
 * iLevil AHRS message, as parsed by AvareX / ForeFlight:
 *  ID 0x4C, 0x45, 0x01, 0x01, then big-endian int16 x10:
 *  roll, pitch, yaw(heading), slip, turn rate, g, airspeed(kt),
 *  altitude ft + 5000 (uint16), vsi fpm. 0x7FFF = not available.
 */
size_t AHRS_GDL90(uint8_t *buf)
{
  if (!AHRS_valid()) return 0;

  uint8_t msg[3 + 18];
  msg[0] = 0x45; msg[1] = 0x01; msg[2] = 0x01;
  uint8_t *p = msg + 3;

  put16(p, (int) lroundf(roll_deg  * 10));  p += 2;
  put16(p, (int) lroundf(pitch_deg * 10));  p += 2;
  bool hdg = isValidFix() && ThisAircraft.speed > 5;
  put16(p, hdg ? (int) lroundf(ThisAircraft.course * 10) : 0x7FFF); p += 2;
  put16(p, (int) lroundf(slip_deg  * 10));  p += 2;
  put16(p, (int) lroundf(turn_dps  * 10));  p += 2;
  put16(p, (int) lroundf(g_load    * 10));  p += 2;
  put16(p, 0x7FFF);                          p += 2;  /* no airspeed */
  if (ThisAircraft.pressure_altitude != 0.0f) {
    int alt_ft = (int) lroundf(ThisAircraft.pressure_altitude * 3.28084f) + 5000;
    if (alt_ft < 0) alt_ft = 0; if (alt_ft > 0xFFFE) alt_ft = 0xFFFE;
    put16(p, alt_ft);
  } else {
    put16(p, 0xFFFF);
  }
  p += 2;
  put16(p, isValidFix() ? (int) lroundf(ThisAircraft.vs) : 0x7FFF);

  return GDL90_Frame(buf, 0x4C, msg, sizeof(msg));
}

void AHRS_NMEA(void) { }   /* no NMEA attitude sentence in this fork */

#else
byte   AHRS_setup(void)        { return IMU_NONE; }
void   AHRS_loop(void)         { }
size_t AHRS_GDL90(uint8_t *b)  { (void) b; return 0; }
void   AHRS_NMEA(void)         { }
bool   AHRS_valid(void)        { return false; }
#endif /* ENABLE_AHRS */
