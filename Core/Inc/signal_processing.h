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

/* --- Gesture recognition (rule-based) tunables ---
 * Accel units are raw LSB at +/-2g full scale: 16384 LSB ~= 1 g.
 * Sampling is 500 Hz, so sample counts below map to time as N/500 seconds.
 * These are starting points — tune on hardware via the UART output. */
#define GRAV_ALPHA          0.995f  // Gravity EMA factor (~0.4 Hz cutoff -> removes tilt/DC)
#define GESTURE_ONSET_LSB   2000.0f // AC magnitude to START a gesture (~0.12 g)
#define GESTURE_OFFSET_LSB  1000.0f // AC magnitude below which motion is "stopped" (hysteresis)
#define GESTURE_MIN_LEN     50      // Min active samples to accept (~100 ms)
#define GESTURE_MAX_LEN     750     // Force classification after this many samples (~1.5 s)
#define GESTURE_COOLDOWN    250     // Debounce after a gesture (~0.5 s)
#define GESTURE_MIN_ZC      2       // Min zero-crossings on dominant axis = oscillation

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
 * @brief Recognized gesture labels.
 * Axis -> physical direction depends on how the board is mounted; rename
 * the strings in Gesture_Name() to match your orientation.
 */
typedef enum {
    GESTURE_NONE = 0,
    GESTURE_SHAKE_X,   // dominant motion along accel X (e.g. forward-back)
    GESTURE_SHAKE_Y,   // dominant motion along accel Y (e.g. left-right)
    GESTURE_SHAKE_Z,   // dominant motion along accel Z (e.g. up-down)
} GestureType;

/**
 * @brief State for the rule-based shake recognizer.
 * Driven one filtered-accel sample at a time; runs a small onset/active/
 * cooldown state machine and classifies by dominant-axis AC energy.
 */
typedef struct {
    /* Slow EMA estimate of gravity (DC) per axis, subtracted to get AC. */
    float grav_x, grav_y, grav_z;
    int   grav_init;        // 0 until the EMA is seeded with the first sample

    int   state;            // internal: idle / active / cooldown
    int   active_count;     // samples elapsed in the current gesture
    int   cooldown_count;   // samples elapsed since the last gesture

    /* Per-axis accumulators, reset at the start of each gesture. */
    float energy_x, energy_y, energy_z;   // sum of AC^2
    int   zc_x, zc_y, zc_z;               // AC zero-crossings (sign changes)
    float prev_ac_x, prev_ac_y, prev_ac_z;
} GestureState;

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

    GestureState gesture;       // shake recognizer state
    GestureType  last_gesture;  // result of the most recent Update (GESTURE_NONE if none)
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

/* ====================================================================
 * Gesture recognition (rule-based)
 * ==================================================================== */

/**
 * @brief Resets the gesture recognizer to idle. (Also covered by
 *        SignalProcessing_Init, which zeroes the whole state.)
 */
void Gesture_Init(GestureState* g);

/**
 * @brief Feeds one FIR-filtered accelerometer sample to the recognizer.
 * @return The recognized gesture on the sample where a gesture completes,
 *         otherwise GESTURE_NONE.
 */
GestureType Gesture_ProcessSample(GestureState* g, float ax, float ay, float az);

/**
 * @brief Human-readable name for a gesture (for UART/display output).
 */
const char* Gesture_Name(GestureType t);

// Existing declarations kept for feature extraction/onset detection placeholders
void SignalProcessing_ExtractFeatures(float* input, float* features);
void SignalProcessing_OnsetDetection(float* input, int* onset_detected);

#endif // SIGNAL_PROCESSING_H