#include "accel_calibration_data.h"

// Default constants keep the demo buildable before the first real six-face calibration.
// Run with ACCEL_DEMO_RUN_SIX_FACE_CALIBRATION set to 1, then replace this initializer with
// the values printed in the terminal for this exact ADXL345 module and mounting setup.
const accel_demo_calibration_t ACCEL_DEMO_STORED_CALIBRATION = {
    7.160004f,     // x_offset_mg
    -20.759979f,     // y_offset_mg
    -17.839996f,     // z_offset_mg

    -0.98332286f,     // x_scale
    -0.96261215f,     // y_scale
    1.01034594f,     // z_scale

    {-1009.799988f, -11.200000f, -58.200001f},    // plus_x_up_average_mg
    {1024.119995f, -6.640000f, 49.000000f},    // minus_x_up_average_mg
    {-3.920000f, -1059.599976f, 14.240000f},    // plus_y_up_average_mg
    {-58.000000f, 1018.080017f, -34.400002f},    // minus_y_up_average_mg
    {-11.200000f, -9.400000f, 971.919983f},    // plus_z_up_average_mg
    {145.000000f, -164.679993f, -1007.599976f},    // minus_z_up_average_mg
};
