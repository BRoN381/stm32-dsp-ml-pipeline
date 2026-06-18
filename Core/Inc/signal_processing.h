/**
 * @file signal_processing.h
 * @brief FIR filter, sliding window, and feature extraction for IMU data.
 */
#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <stdint.h>

/* ====================================================================
 * 1. Macros & Constants
 * ==================================================================== */
#define FIR_TAPS 31                 // Number of filter taps (order)
#define COMPLEMENTARY_ALPHA 0.98f   // Weight for gyroscope trust

/* ====================================================================
 * 2. Extern Constants
 * Actual values should be defined in signal_processing.c
 * ==================================================================== */
extern const float FIR_COEFFS_LOWPASS[FIR_TAPS]; 

/* ====================================================================
 * 3. Data Structures
 * ==================================================================== */
/**
 * @brief State structure for a single FIR filter channel.
 */
typedef struct {
    float history[FIR_TAPS]; // Ring buffer for historical data
    int head;                // Write index for the ring buffer
} FIR_FilterState;

/**
 * @brief State structure for the complementary filter (attitude estimation).
 */
typedef struct {
    float pitch; // X-axis rotation angle in degrees
    float roll;  // Y-axis rotation angle in degrees
} Complementary_FilterState;

/**
 * @brief Comprehensive state manager for 6-axis IMU processing.
 */
typedef struct {
    FIR_FilterState acc_x_filter;
    FIR_FilterState acc_y_filter;
    FIR_FilterState acc_z_filter;
    FIR_FilterState gyro_x_filter;
    FIR_FilterState gyro_y_filter;
    FIR_FilterState gyro_z_filter;
    Complementary_FilterState attitude;

    /* Most recent FIR-filtered accelerometer outputs (for inspection/debug). */
    float filt_acc_x;
    float filt_acc_y;
    float filt_acc_z;
} IMU_ProcessingState;

/* ====================================================================
 * 4. Function Prototypes
 * ==================================================================== */

/**
 * @brief Initializes all filter states (clears ring buffers).
 * @param state Pointer to the IMU processing state structure.
 */
void SignalProcessing_Init(IMU_ProcessingState* state);

/**
 * @brief Processes a single sample through an FIR filter.
 * @param state Pointer to the specific FIR filter state.
 * @param input New sensor data sample.
 * @param coeffs Array of FIR filter coefficients.
 * @return The filtered output value.
 */
float FIR_ProcessSingle(FIR_FilterState* state, float input, const float* coeffs);

/**
 * @brief Fuses accelerometer and gyroscope data to calculate attitude.
 * @param state Pointer to the complementary filter state.
 * @param acc_x Filtered X-axis accelerometer data.
 * @param acc_y Filtered Y-axis accelerometer data.
 * @param acc_z Filtered Z-axis accelerometer data.
 * @param gyro_x Filtered X-axis gyroscope data (pitch rate).
 * @param gyro_y Filtered Y-axis gyroscope data (roll rate).
 * @param dt Sampling interval in seconds (e.g., 0.002 for 500Hz).
 */
void Complementary_Process(Complementary_FilterState* state, 
                           float acc_x, float acc_y, float acc_z, 
                           float gyro_x, float gyro_y, 
                           float dt);

/**
 * @brief High-level update function to be called on every new IMU sample.
 * Executes FIR filtering for all axes and updates the complementary filter.
 * @param state Pointer to the IMU processing state structure.
 * @param raw_ax Raw X-axis accelerometer data.
 * @param raw_ay Raw Y-axis accelerometer data.
 * @param raw_az Raw Z-axis accelerometer data.
 * @param raw_gx Raw X-axis gyroscope data.
 * @param raw_gy Raw Y-axis gyroscope data.
 * @param raw_gz Raw Z-axis gyroscope data.
 * @param dt Sampling interval in seconds.
 */
void SignalProcessing_Update(IMU_ProcessingState* state, 
                             float raw_ax, float raw_ay, float raw_az, 
                             float raw_gx, float raw_gy, float raw_gz,
                             float dt);

// Existing declarations kept for feature extraction/onset detection placeholders
void SignalProcessing_ExtractFeatures(float* input, float* features);
void SignalProcessing_OnsetDetection(float* input, int* onset_detected);

#endif // SIGNAL_PROCESSING_H