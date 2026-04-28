#ifndef ACCEL_CALIBRATION_DATA_H
#define ACCEL_CALIBRATION_DATA_H

// This file keeps the measured ADXL345 calibration constants separate from the demo logic.
// After running the six-face calibration mode once, paste the printed values into
// accel_calibration_data.cpp so future firmware changes can reuse the same sensor correction.

typedef struct {
    float x_mg;
    float y_mg;
    float z_mg;
} accel_demo_face_average_t;

typedef struct {
    // These offsets remove the zero-g bias from each ADXL345 axis.
    float x_offset_mg;
    float y_offset_mg;
    float z_offset_mg;

    // These scale factors normalize each axis so the measured gravity span is close to +/-1000 mg.
    float x_scale;
    float y_scale;
    float z_scale;

    // The face averages are stored for traceability so the team can see what produced the constants.
    accel_demo_face_average_t plus_x_up_average_mg;
    accel_demo_face_average_t minus_x_up_average_mg;
    accel_demo_face_average_t plus_y_up_average_mg;
    accel_demo_face_average_t minus_y_up_average_mg;
    accel_demo_face_average_t plus_z_up_average_mg;
    accel_demo_face_average_t minus_z_up_average_mg;
} accel_demo_calibration_t;

extern const accel_demo_calibration_t ACCEL_DEMO_STORED_CALIBRATION;

#endif
