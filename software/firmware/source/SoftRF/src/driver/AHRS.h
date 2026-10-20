/*
 * AHRS.h
 * Fork: attitude from the Seeed MeshTracker X1's LSM6DSO (accel + gyro),
 * six-axis Mahony fusion, emitted as an iLevil-style GDL90 AHRS message.
 * SoftRF upstream keeps its AHRS in the closed "premium" package.
 */
#ifndef AHRSHELPER_H
#define AHRSHELPER_H

#include <stdint.h>
#include <stddef.h>

/* upstream call sites expect these */
#define AHRS_INTERVAL 100   /* ms, NMEA attitude cadence (unused: no NMEA attitude here) */
typedef struct { const char name[12]; } ahrs_chip_ops_t;
extern const ahrs_chip_ops_t *ahrs_chip;   /* NULL when no IMU */

byte   AHRS_setup(void);
void   AHRS_loop(void);
size_t AHRS_GDL90(uint8_t *buf);
void   AHRS_NMEA(void);

/* latest attitude, degrees; valid only when AHRS_valid() */
bool  AHRS_valid(void);
float AHRS_roll(void);
float AHRS_pitch(void);
float AHRS_turn_rate(void);  /* deg/s, positive right */
float AHRS_slip(void);       /* deg, positive = skid right */
float AHRS_g(void);          /* vertical load factor */

#endif /* AHRSHELPER_H */
