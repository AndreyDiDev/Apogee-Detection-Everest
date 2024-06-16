/**
 * @co-author Andrey Dimanchev
 * @file infusion.cpp
 * @author Seb Madgwick
 * @brief AHRS algorithm to combine gyroscope, accelerometer, and magnetometer
 * measurements into a filtered orientation relative to the Earth's frame of reference
 * 
 * Modified Implementation of Madgwick's IMU and AHRS algorithms.
 * See: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 */

//------------------------------------------------------------------------------
// Includes

#include <float.h> // FLT_MAX
#include "infusion.hpp"
#include <math.h> // atan2f, cosf, fabsf, powf, sinf
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <ctime>
#include <string>

using namespace std;

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Initial gain used during the initialisation.
 */
#define INITIAL_GAIN (10.0f)

/**
 * @brief Initialisation period in seconds.
 */
#define INITIALISATION_PERIOD (3.0f)

/**
 * @brief Cutoff frequency in Hz.
 */
#define CUTOFF_FREQUENCY (0.02f)

/**
 * @brief Timeout in seconds.
 */
#define TIMEOUT (5)

/**
 * @brief Threshold in degrees per second.
 */
#define THRESHOLD (3.0f)

#define MAX_LINE_LENGTH 1024

// Functions--------------------------------------------------------------------
/**
 * @brief Initialises the AHRS algorithm structure.
 * @param ahrs AHRS algorithm structure.
 */
void Infusion::madAhrsInitialise(madAhrs *const ahrs) {
    const madAhrsSettings settings = {
            .convention = EarthConventionNed,
            .gain = 0.5f,
            .gyroscopeRange = 0.0f,
            .accelerationRejection = 90.0f,
            .magneticRejection = 90.0f,
            .recoveryTriggerPeriod = 0,
    };
    madAhrsSetSettings(ahrs, &settings);
    madAhrsReset(ahrs);
}

/**
 * @brief Resets the AHRS algorithm.  This is equivalent to reinitialising the
 * algorithm while maintaining the current settings.
 * @param ahrs AHRS algorithm structure.
 */
void madAhrsReset(madAhrs *const ahrs) {
    ahrs->quaternion = IDENTITY_QUATERNION;
    ahrs->accelerometer = VECTOR_ZERO;
    ahrs->initialising = true;
    ahrs->rampedGain = INITIAL_GAIN;
    ahrs->angularRateRecovery = false;
    ahrs->halfAccelerometerFeedback = VECTOR_ZERO;
    ahrs->halfMagnetometerFeedback = VECTOR_ZERO;
    ahrs->accelerometerIgnored = false;
    ahrs->accelerationRecoveryTrigger = 0;
    ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
    ahrs->magnetometerIgnored = false;
    ahrs->magneticRecoveryTrigger = 0;
    ahrs->magneticRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
}

/**
 * @brief Sets the AHRS algorithm settings.
 * @param ahrs AHRS algorithm structure.
 * @param settings Settings.
 */
void Infusion::madAhrsSetSettings(madAhrs *const ahrs, const madAhrsSettings *const settings) {
    ahrs->settings.convention = settings->convention;
    ahrs->settings.gain = settings->gain;
    ahrs->settings.gyroscopeRange = settings->gyroscopeRange == 0.0f ? FLT_MAX : 0.98f * settings->gyroscopeRange;
    ahrs->settings.accelerationRejection = settings->accelerationRejection == 0.0f ? FLT_MAX : powf(0.5f * sinf(DegreesToRadians(settings->accelerationRejection)), 2);
    ahrs->settings.magneticRejection = settings->magneticRejection == 0.0f ? FLT_MAX : powf(0.5f * sinf(DegreesToRadians(settings->magneticRejection)), 2);
    ahrs->settings.recoveryTriggerPeriod = settings->recoveryTriggerPeriod;
    ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
    ahrs->magneticRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
    if ((settings->gain == 0.0f) || (settings->recoveryTriggerPeriod == 0)) { // disable acceleration and magnetic rejection features if gain is zero
        ahrs->settings.accelerationRejection = FLT_MAX;
        ahrs->settings.magneticRejection = FLT_MAX;
    }
    if (ahrs->initialising == false) {
        ahrs->rampedGain = ahrs->settings.gain;
    }
    ahrs->rampedGainStep = (INITIAL_GAIN - ahrs->settings.gain) / INITIALISATION_PERIOD;
}

void Infusion::reinitialiseGyro(madAhrs *const ahrs, const madVector gyroscope){

    if ((fabsf(gyroscope.axis.x) > ahrs->settings.gyroscopeRange) || (fabsf(gyroscope.axis.y) > ahrs->settings.gyroscopeRange) || (fabsf(gyroscope.axis.z) > ahrs->settings.gyroscopeRange)) {
        const madQuaternion quaternion = ahrs->quaternion;
        madAhrsReset(ahrs);
        ahrs->quaternion = quaternion;
        ahrs->angularRateRecovery = true;
    }
}

void Infusion::rampDownGain(madAhrs *const ahrs, const float deltaTime){
    if (ahrs->initialising) {
        ahrs->rampedGain -= ahrs->rampedGainStep * deltaTime;
        if ((ahrs->rampedGain < ahrs->settings.gain) || (ahrs->settings.gain == 0.0f)) {
            ahrs->rampedGain = ahrs->settings.gain;
            ahrs->initialising = false;
            ahrs->angularRateRecovery = false;
        }
    }
}

madVector Infusion::accelerometerFeedback(madAhrs *const ahrs, const madVector accelerometer, madVector halfGravity, madVector halfAccelerometerFeedback){

    if (madVectorIsZero(accelerometer) == false) {

        // Calculate accelerometer feedback scaled by 0.5
        ahrs->halfAccelerometerFeedback = Feedback(madVectorNormalise(accelerometer), halfGravity);

        // Don't ignore accelerometer if acceleration error below threshold
        if (ahrs->initialising || ((madVectorMagnitudeSquared(ahrs->halfAccelerometerFeedback) <= ahrs->settings.accelerationRejection))) {
            ahrs->accelerometerIgnored = false;
            ahrs->accelerationRecoveryTrigger -= 9;
        } else {
            ahrs->accelerationRecoveryTrigger += 1;
        }

        // Don't ignore accelerometer during acceleration recovery
        if (ahrs->accelerationRecoveryTrigger > ahrs->accelerationRecoveryTimeout) {
            ahrs->accelerationRecoveryTimeout = 0;
            ahrs->accelerometerIgnored = false;
        } else {
            ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
        }
        ahrs->accelerationRecoveryTrigger = Clamp(ahrs->accelerationRecoveryTrigger, 0, ahrs->settings.recoveryTriggerPeriod);

        // Apply accelerometer feedback
        if (ahrs->accelerometerIgnored == false) {
            halfAccelerometerFeedback = ahrs->halfAccelerometerFeedback;
        }
    }

    return halfAccelerometerFeedback;
}

madVector Infusion::magnetometerFeedback(madAhrs *const ahrs, const madVector magnetometer, madVector halfGravity, madVector halfMagnetometerFeedback){
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if (madVectorIsZero(magnetometer) == false) {

        // printf("Mag: (%.6f, %.6f, %.6f) uT\n", magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z);

        // Calculate direction of magnetic field indicated by algorithm
        const madVector halfMagnetic = HalfMagnetic(ahrs);

        // Calculate magnetometer feedback scaled by 0.5
        ahrs->halfMagnetometerFeedback = Feedback(madVectorNormalise(madVectorCrossProduct(halfGravity, magnetometer)), halfMagnetic);

        // Don't ignore magnetometer if magnetic error below threshold
        if (ahrs->initialising || ((madVectorMagnitudeSquared(ahrs->halfMagnetometerFeedback) <= ahrs->settings.magneticRejection))) {
            ahrs->magnetometerIgnored = false;
            ahrs->magneticRecoveryTrigger -= 9;
        } else {
            ahrs->magneticRecoveryTrigger += 1;
        }

        // Don't ignore magnetometer during magnetic recovery
        if (ahrs->magneticRecoveryTrigger > ahrs->magneticRecoveryTimeout) {
            ahrs->magneticRecoveryTimeout = 0;
            ahrs->magnetometerIgnored = false;
        } else {
            ahrs->magneticRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
        }
        ahrs->magneticRecoveryTrigger = Clamp(ahrs->magneticRecoveryTrigger, 0, ahrs->settings.recoveryTriggerPeriod);

        // Apply magnetometer feedback
        if (ahrs->magnetometerIgnored == false) {
            halfMagnetometerFeedback = ahrs->halfMagnetometerFeedback;
        }
    }
    return halfMagnetometerFeedback;
}


/**
 * @brief Updates the AHRS algorithm using the gyroscope, accelerometer, and
 * magnetometer measurements. Acts more like a wrapper for the low and 
 * high pass filter, all of the MARG operations are abstracted in other 
 * functions
 * 
 * MARG Operations
 * 0. Do repetitive multiplication
 * 1. Normalise the accelerometer measurement
 * 2. Normalise the magnetometer measurement
 * 3. Compute the objective function and Jacobian
 * 4. Compute the gradient (matrix multiplication)
 * 5. Normalise the gradient to estimate direction of the gyroscope error
 * 6. Compute angular estimated direction of the gyroscope error
 * 7. Compute and remove the gyroscope biases
 * 8. Compute the quaternion rate measured by gyroscopes
 * 9. Compute then integrate the estimated quaternion rate
 * 10.Normalise quaternion
 * 11.Compute flux in the earth frame
 * 12.Normalise the flux vector to have only components in the x and z
 * 
 * @param ahrs AHRS algorithm structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @param accelerometer Accelerometer measurement in g.
 * @param magnetometer Magnetometer measurement in arbitrary units.
 * @param deltaTime Delta time in seconds.
 */
void Infusion::madAhrsUpdate(madAhrs *const ahrs, const madVector gyroscope, const madVector accelerometer, const madVector magnetometer, const float deltaTime) {
#define Q ahrs->quaternion.element
// Store accelerometer
    ahrs->accelerometer = accelerometer;

    // Reinitialise if gyroscope range exceeded
    if ((fabsf(gyroscope.axis.x) > ahrs->settings.gyroscopeRange) || (fabsf(gyroscope.axis.y) > ahrs->settings.gyroscopeRange) || (fabsf(gyroscope.axis.z) > ahrs->settings.gyroscopeRange)) {
        const madQuaternion quaternion = ahrs->quaternion;
        madAhrsReset(ahrs);
        ahrs->quaternion = quaternion;
        ahrs->angularRateRecovery = true;
    }

    // Ramp down gain during initialisation
    if (ahrs->initialising) {
        ahrs->rampedGain -= ahrs->rampedGainStep * deltaTime;
        // printf("%f\n", ahrs->rampedGain);
        if ((ahrs->rampedGain < ahrs->settings.gain) || (ahrs->settings.gain == 0.0f)) {
            // printf("adte");
            // printf("%d", ahrs->rampedGain);
            ahrs->rampedGain = ahrs->settings.gain;
            ahrs->initialising = false;
            ahrs->angularRateRecovery = false;
        }
    }

    // Calculate direction of gravity indicated by algorithm
    const madVector halfGravity = HalfGravity(ahrs);

    // Calculate accelerometer feedback
    madVector halfAccelerometerFeedback = VECTOR_ZERO;
    ahrs->accelerometerIgnored = true;
    if (madVectorIsZero(accelerometer) == false) {

        // Calculate accelerometer feedback scaled by 0.5
        ahrs->halfAccelerometerFeedback = Feedback(madVectorNormalise(accelerometer), halfGravity);

        // Don't ignore accelerometer if acceleration error below threshold
        if (ahrs->initialising || ((madVectorMagnitudeSquared(ahrs->halfAccelerometerFeedback) <= ahrs->settings.accelerationRejection))) {
            ahrs->accelerometerIgnored = false;
            ahrs->accelerationRecoveryTrigger -= 9;
        } else {
            ahrs->accelerationRecoveryTrigger += 1;
        }

        // Don't ignore accelerometer during acceleration recovery
        if (ahrs->accelerationRecoveryTrigger > ahrs->accelerationRecoveryTimeout) {
            ahrs->accelerationRecoveryTimeout = 0;
            ahrs->accelerometerIgnored = false;
        } else {
            ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
        }
        ahrs->accelerationRecoveryTrigger = Clamp(ahrs->accelerationRecoveryTrigger, 0, ahrs->settings.recoveryTriggerPeriod);

        // Apply accelerometer feedback
        if (ahrs->accelerometerIgnored == false) {
            halfAccelerometerFeedback = ahrs->halfAccelerometerFeedback;
        }
    }

    // Calculate magnetometer feedback
    madVector halfMagnetometerFeedback = VECTOR_ZERO;
    ahrs->magnetometerIgnored = true;
    if (madVectorIsZero(magnetometer) == false) {

        // Calculate direction of magnetic field indicated by algorithm
        const madVector halfMagnetic = HalfMagnetic(ahrs);

        // Calculate magnetometer feedback scaled by 0.5
        ahrs->halfMagnetometerFeedback = Feedback(madVectorNormalise(madVectorCrossProduct(halfGravity, magnetometer)), halfMagnetic);

        // Don't ignore magnetometer if magnetic error below threshold
        if (ahrs->initialising || ((madVectorMagnitudeSquared(ahrs->halfMagnetometerFeedback) <= ahrs->settings.magneticRejection))) {
            ahrs->magnetometerIgnored = false;
            ahrs->magneticRecoveryTrigger -= 9;
        } else {
            ahrs->magneticRecoveryTrigger += 1;
        }

        // Don't ignore magnetometer during magnetic recovery
        if (ahrs->magneticRecoveryTrigger > ahrs->magneticRecoveryTimeout) {
            ahrs->magneticRecoveryTimeout = 0;
            ahrs->magnetometerIgnored = false;
        } else {
            ahrs->magneticRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
        }
        ahrs->magneticRecoveryTrigger = Clamp(ahrs->magneticRecoveryTrigger, 0, ahrs->settings.recoveryTriggerPeriod);

        // Apply magnetometer feedback
        if (ahrs->magnetometerIgnored == false) {
            halfMagnetometerFeedback = ahrs->halfMagnetometerFeedback;
        }
    }

    // Convert gyroscope to radians per second scaled by 0.5
    const madVector halfGyroscope = madVectorMultiplyScalar(gyroscope, DegreesToRadians(0.5f));

    // Apply feedback to gyroscope
    const madVector adjustedHalfGyroscope = madVectorAdd(halfGyroscope, madVectorMultiplyScalar(madVectorAdd(halfAccelerometerFeedback, halfMagnetometerFeedback), ahrs->rampedGain));

    // Integrate rate of change of quaternion
    ahrs->quaternion = madQuaternionAdd(ahrs->quaternion, madQuaternionMultiplyVector(ahrs->quaternion, madVectorMultiplyScalar(adjustedHalfGyroscope, deltaTime)));

    // Normalise quaternion
    ahrs->quaternion = madQuaternionNormalise(ahrs->quaternion);

//---------------------------------------------------------------------------------------------------------------------
    // // Store accelerometer
    // ahrs->accelerometer = accelerometer;

    // // Reinitialise if gyroscope range exceeded
    // this->reinitialiseGyro(ahrs, gyroscope);

    // // Ramp down gain during initialisation
    // this->rampDownGain(ahrs, deltaTime);

    // // Calculate direction of gravity indicated by algorithm
    // const madVector halfGravity = HalfGravity(ahrs);

    // // Calculate accelerometer feedback
    // madVector halfAccelerometerFeedback = VECTOR_ZERO;
    // ahrs->accelerometerIgnored = true;

    // // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    // halfAccelerometerFeedback = this->accelerometerFeedback(ahrs, accelerometer, halfGravity, halfAccelerometerFeedback);

    // // Calculate magnetometer feedback
    // madVector halfMagnetometerFeedback = VECTOR_ZERO;
    // ahrs->magnetometerIgnored = true;

    // halfMagnetometerFeedback = this->magnetometerFeedback(ahrs, magnetometer, halfGravity, halfMagnetometerFeedback);

    // // Convert gyroscope to radians per second scaled by 0.5
    // const madVector halfGyroscope = madVectorMultiplyScalar(gyroscope, DegreesToRadians(0.5f));

    // // Apply feedback to gyroscope
    // const madVector adjustedHalfGyroscope = madVectorAdd(halfGyroscope, madVectorMultiplyScalar(madVectorAdd(halfAccelerometerFeedback, halfMagnetometerFeedback), ahrs->rampedGain));

    // // Integrate rate of change of quaternion
    // ahrs->quaternion = madQuaternionAdd(ahrs->quaternion, madQuaternionMultiplyVector(ahrs->quaternion, madVectorMultiplyScalar(adjustedHalfGyroscope, deltaTime)));

    // // Normalise quaternion
    // ahrs->quaternion = madQuaternionNormalise(ahrs->quaternion);
#undef Q
}

/**
 * @brief Returns the direction of gravity scaled by 0.5.
 * @param ahrs AHRS algorithm structure.
 * @return Direction of gravity scaled by 0.5.
 */
static inline madVector HalfGravity(const madAhrs *const ahrs) {
#define Q ahrs->quaternion.element
    switch (ahrs->settings.convention) {
        case EarthConventionNwu:
        case EarthConventionEnu: {
            const madVector halfGravity = {.axis = {
                    .x = Q.x * Q.z - Q.w * Q.y,
                    .y = Q.y * Q.z + Q.w * Q.x,
                    .z = Q.w * Q.w - 0.5f + Q.z * Q.z,
            }}; // third column of transposed rotation matrix scaled by 0.5
            return halfGravity;
        }
        case EarthConventionNed: {
            const madVector halfGravity = {.axis = {
                    .x = Q.w * Q.y - Q.x * Q.z,
                    .y = -1.0f * (Q.y * Q.z + Q.w * Q.x),
                    .z = 0.5f - Q.w * Q.w - Q.z * Q.z,
            }}; // third column of transposed rotation matrix scaled by -0.5
            return halfGravity;
        }
    }
    return VECTOR_ZERO; // avoid compiler warning
#undef Q
}

/**
 * @brief Returns the direction of the magnetic field scaled by 0.5.
 * @param ahrs AHRS algorithm structure.
 * @return Direction of the magnetic field scaled by 0.5.
 */
static inline madVector HalfMagnetic(const madAhrs *const ahrs) {
#define Q ahrs->quaternion.element
    switch (ahrs->settings.convention) {
        case EarthConventionNwu: {
            const madVector halfMagnetic = {.axis = {
                    .x = Q.x * Q.y + Q.w * Q.z,
                    .y = Q.w * Q.w - 0.5f + Q.y * Q.y,
                    .z = Q.y * Q.z - Q.w * Q.x,
            }}; // second column of transposed rotation matrix scaled by 0.5
            return halfMagnetic;
        }
        case EarthConventionEnu: {
            const madVector halfMagnetic = {.axis = {
                    .x = 0.5f - Q.w * Q.w - Q.x * Q.x,
                    .y = Q.w * Q.z - Q.x * Q.y,
                    .z = -1.0f * (Q.x * Q.z + Q.w * Q.y),
            }}; // first column of transposed rotation matrix scaled by -0.5
            return halfMagnetic;
        }
        case EarthConventionNed: {
            const madVector halfMagnetic = {.axis = {
                    .x = -1.0f * (Q.x * Q.y + Q.w * Q.z),
                    .y = 0.5f - Q.w * Q.w - Q.y * Q.y,
                    .z = Q.w * Q.x - Q.y * Q.z,
            }}; // second column of transposed rotation matrix scaled by -0.5
            return halfMagnetic;
        }
    }
    return VECTOR_ZERO; // avoid compiler warning
#undef Q
}

/**
 * @brief Returns the feedback.
 * @param sensor Sensor.
 * @param reference Reference.
 * @return Feedback.
 */
static inline madVector Feedback(const madVector sensor, const madVector reference) {
    if (madVectorDotProduct(sensor, reference) < 0.0f) { // if error is >90 degrees
        return madVectorNormalise(madVectorCrossProduct(sensor, reference));
    }
    return madVectorCrossProduct(sensor, reference);
}

/**
 * @brief Returns a value limited to maximum and minimum.
 * @param value Value.
 * @param min Minimum value.
 * @param max Maximum value.
 * @return Value limited to maximum and minimum.
 */
static inline int Clamp(const int value, const int min, const int max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

/**
 * @brief Updates the AHRS algorithm using the gyroscope and accelerometer
 * measurements only.
 * @param ahrs AHRS algorithm structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @param accelerometer Accelerometer measurement in g.
 * @param deltaTime Delta time in seconds.
 */
void Infusion::madAhrsUpdateNoMagnetometer(madAhrs *const ahrs, const madVector gyroscope, const madVector accelerometer, const float deltaTime) {

    // Update AHRS algorithm
    this->madAhrsUpdate(ahrs, gyroscope, accelerometer, VECTOR_ZERO, deltaTime);

    // Zero heading during initialisation
    if (ahrs->initialising) {
        madAhrsSetHeading(ahrs, 0.0f);
    }
}

/**
 * @brief Updates the AHRS algorithm using the gyroscope, accelerometer, and
 * heading measurements.
 * @param ahrs AHRS algorithm structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @param accelerometer Accelerometer measurement in g.
 * @param heading Heading measurement in degrees.
 * @param deltaTime Delta time in seconds.
 */
void Infusion::madAhrsUpdateExternalHeading(madAhrs *const ahrs, const madVector gyroscope, const madVector accelerometer, const float heading, const float deltaTime) {
#define Q ahrs->quaternion.element

    // Calculate roll
    const float roll = atan2f(Q.w * Q.x + Q.y * Q.z, 0.5f - Q.y * Q.y - Q.x * Q.x);

    // Calculate magnetometer
    const float headingRadians = DegreesToRadians(heading);
    const float sinHeadingRadians = sinf(headingRadians);
    const madVector magnetometer = {.axis = {
            .x = cosf(headingRadians),
            .y = -1.0f * cosf(roll) * sinHeadingRadians,
            .z = sinHeadingRadians * sinf(roll),
    }};

    // Update AHRS algorithm
    this->madAhrsUpdate(ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
#undef Q
}

/**
 * @brief Returns the quaternion describing the sensor relative to the Earth.
 * @param ahrs AHRS algorithm structure.
 * @return Quaternion describing the sensor relative to the Earth.
 */
madQuaternion Infusion::madAhrsGetQuaternion(const madAhrs *const ahrs) {
    return ahrs->quaternion;
}

/**
 * @brief Sets the quaternion describing the sensor relative to the Earth.
 * @param ahrs AHRS algorithm structure.
 * @param quaternion Quaternion describing the sensor relative to the Earth.
 */
void madAhrsSetQuaternion(madAhrs *const ahrs, const madQuaternion quaternion) {
    ahrs->quaternion = quaternion;
}

/**
 * @brief Returns the linear acceleration measurement equal to the accelerometer
 * measurement with the 1 g of gravity removed.
 * @param ahrs AHRS algorithm structure.
 * @return Linear acceleration measurement in g.
 */
madVector madAhrsGetLinearAcceleration(const madAhrs *const ahrs) {
#define Q ahrs->quaternion.element

    // Calculate gravity in the sensor coordinate frame
    const madVector gravity = {.axis = {
            .x = 2.0f * (Q.x * Q.z - Q.w * Q.y),
            .y = 2.0f * (Q.y * Q.z + Q.w * Q.x),
            .z = 2.0f * (Q.w * Q.w - 0.5f + Q.z * Q.z),
    }}; // third column of transposed rotation matrix

    // Remove gravity from accelerometer measurement
    switch (ahrs->settings.convention) {
        case EarthConventionNwu:
        case EarthConventionEnu: {
            return madVectorSubtract(ahrs->accelerometer, gravity);
        }
        case EarthConventionNed: {
            return madVectorAdd(ahrs->accelerometer, gravity);
        }
    }
    return VECTOR_ZERO; // avoid compiler warning
#undef Q
}

/**
 * @brief Returns the Earth acceleration measurement equal to accelerometer
 * measurement in the Earth coordinate frame with the 1 g of gravity removed.
 * @param ahrs AHRS algorithm structure.
 * @return Earth acceleration measurement in g.
 */
madVector Infusion::madAhrsGetEarthAcceleration(const madAhrs *const ahrs) {
#define Q ahrs->quaternion.element
#define A ahrs->accelerometer.axis

    // Calculate accelerometer measurement in the Earth coordinate frame
    const float qwqw = Q.w * Q.w; // calculate common terms to avoid repeated operations
    const float qwqx = Q.w * Q.x;
    const float qwqy = Q.w * Q.y;
    const float qwqz = Q.w * Q.z;
    const float qxqy = Q.x * Q.y;
    const float qxqz = Q.x * Q.z;
    const float qyqz = Q.y * Q.z;
    madVector accelerometer = {.axis = {
            .x = 2.0f * ((qwqw - 0.5f + Q.x * Q.x) * A.x + (qxqy - qwqz) * A.y + (qxqz + qwqy) * A.z),
            .y = 2.0f * ((qxqy + qwqz) * A.x + (qwqw - 0.5f + Q.y * Q.y) * A.y + (qyqz - qwqx) * A.z),
            .z = 2.0f * ((qxqz - qwqy) * A.x + (qyqz + qwqx) * A.y + (qwqw - 0.5f + Q.z * Q.z) * A.z),
    }}; // rotation matrix multiplied with the accelerometer

    // Remove gravity from accelerometer measurement
    switch (ahrs->settings.convention) {
        case EarthConventionNwu:
        case EarthConventionEnu:
            accelerometer.axis.z -= 1.0f;
            break;
        case EarthConventionNed:
            accelerometer.axis.z += 1.0f;
            break;
    }
    return accelerometer;
#undef Q
#undef A
}

/**
 * @brief Returns the AHRS algorithm internal states.
 * @param ahrs AHRS algorithm structure.
 * @return AHRS algorithm internal states.
 */
madAhrsInternalStates Infusion::madAhrsGetInternalStates(const madAhrs *const ahrs) {
    const madAhrsInternalStates internalStates = {
            .accelerationError = RadiansToDegrees(Asin(2.0f * madVectorMagnitude(ahrs->halfAccelerometerFeedback))),
            .accelerometerIgnored = ahrs->accelerometerIgnored,
            .accelerationRecoveryTrigger = ahrs->settings.recoveryTriggerPeriod == 0 ? 0.0f : (float) ahrs->accelerationRecoveryTrigger / (float) ahrs->settings.recoveryTriggerPeriod,
            .magneticError = RadiansToDegrees(Asin(2.0f * madVectorMagnitude(ahrs->halfMagnetometerFeedback))),
            .magnetometerIgnored = ahrs->magnetometerIgnored,
            .magneticRecoveryTrigger = ahrs->settings.recoveryTriggerPeriod == 0 ? 0.0f : (float) ahrs->magneticRecoveryTrigger / (float) ahrs->settings.recoveryTriggerPeriod,
    };
    return internalStates;
}

/**
 * @brief Returns the AHRS algorithm flags.
 * @param ahrs AHRS algorithm structure.
 * @return AHRS algorithm flags.
 */
madAhrsFlags Infusion::madAhrsGetFlags(const madAhrs *const ahrs) {
    const madAhrsFlags flags = {
            .initialising = ahrs->initialising,
            .angularRateRecovery = ahrs->angularRateRecovery,
            .accelerationRecovery = ahrs->accelerationRecoveryTrigger > ahrs->accelerationRecoveryTimeout,
            .magneticRecovery= ahrs->magneticRecoveryTrigger > ahrs->magneticRecoveryTimeout,
    };
    return flags;
}

/**
 * @brief Sets the heading of the orientation measurement provided by the AHRS
 * algorithm.  This function can be used to reset drift in heading when the AHRS
 * algorithm is being used without a magnetometer.
 * @param ahrs AHRS algorithm structure.
 * @param heading Heading angle in degrees.
 */
void madAhrsSetHeading(madAhrs *const ahrs, const float heading) {
#define Q ahrs->quaternion.element
    const float yaw = atan2f(Q.w * Q.z + Q.x * Q.y, 0.5f - Q.y * Q.y - Q.z * Q.z);
    const float halfYawMinusHeading = 0.5f * (yaw - DegreesToRadians(heading));
    const madQuaternion rotation = {.element = {
            .w = cosf(halfYawMinusHeading),
            .x = 0.0f,
            .y = 0.0f,
            .z = -1.0f * sinf(halfYawMinusHeading),
    }};
    ahrs->quaternion = madQuaternionMultiply(rotation, ahrs->quaternion);
#undef Q
}

//------------------------------------------------------------------------------
/**
 * @brief Tilt-compensated compass to calculate the magnetic heading using
 * accelerometer and magnetometer measurements.
 */

// Functions--------------------------------------------------------------------
/**
 * @brief Calculates the magnetic heading.
 * @param convention Earth axes convention.
 * @param accelerometer Accelerometer measurement in any calibrated units.
 * @param magnetometer Magnetometer measurement in any calibrated units.
 * @return Heading angle in degrees.
 */
float compassCalculateHeading(const EarthConvention convention, const madVector accelerometer, const madVector magnetometer) {
    switch (convention) {
        case EarthConventionNwu: {
            const madVector west = madVectorNormalise(madVectorCrossProduct(accelerometer, magnetometer));
            const madVector north = madVectorNormalise(madVectorCrossProduct(west, accelerometer));
            return RadiansToDegrees(atan2f(west.axis.x, north.axis.x));
        }
        case EarthConventionEnu: {
            const madVector west = madVectorNormalise(madVectorCrossProduct(accelerometer, magnetometer));
            const madVector north = madVectorNormalise(madVectorCrossProduct(west, accelerometer));
            const madVector east = madVectorMultiplyScalar(west, -1.0f);
            return RadiansToDegrees(atan2f(north.axis.x, east.axis.x));
        }
        case EarthConventionNed: {
            const madVector up = madVectorMultiplyScalar(accelerometer, -1.0f);
            const madVector west = madVectorNormalise(madVectorCrossProduct(up, magnetometer));
            const madVector north = madVectorNormalise(madVectorCrossProduct(west, up));
            return RadiansToDegrees(atan2f(west.axis.x, north.axis.x));
        }
    }
    return 0; // avoid compiler warning
}

//------------------------------------------------------------------------------

/**
 * @brief Gyroscope offset correction algorithm for run-time calibration of the
 * gyroscope offset.
 */
//------------------------------------------------------------------------------

// Functions--------------------------------------------------------------------
/**
 * @brief Initialises the gyroscope offset algorithm.
 * @param offset Gyroscope offset algorithm structure.
 * @param sampleRate Sample rate in Hz.
 */
void Infusion::madOffsetInitialise(madOffset *const offset, const unsigned int sampleRate) {
    offset->filterCoefficient = 2.0f * (float) M_PI * CUTOFF_FREQUENCY * (1.0f / (float) sampleRate);
    offset->timeout = TIMEOUT * sampleRate;
    offset->timer = 0;
    offset->gyroscopeOffset = VECTOR_ZERO;
}

/**
 * @brief Updates the gyroscope offset algorithm and returns the corrected
 * gyroscope measurement.
 * @param offset Gyroscope offset algorithm structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @return Corrected gyroscope measurement in degrees per second.
 */
madVector Infusion::madOffsetUpdate(madOffset *const offset, madVector gyroscope) {

    // Subtract offset from gyroscope measurement
    gyroscope = madVectorSubtract(gyroscope, offset->gyroscopeOffset);

    // Reset timer if gyroscope not stationary
    if ((fabsf(gyroscope.axis.x) > THRESHOLD) || (fabsf(gyroscope.axis.y) > THRESHOLD) || (fabsf(gyroscope.axis.z) > THRESHOLD)) {
        offset->timer = 0;
        return gyroscope;
    }

    // Increment timer while gyroscope stationary
    if (offset->timer < offset->timeout) {
        offset->timer++;
        return gyroscope;
    }

    // Adjust offset if timer has elapsed
    offset->gyroscopeOffset = madVectorAdd(offset->gyroscopeOffset, madVectorMultiplyScalar(gyroscope, offset->filterCoefficient));
    return gyroscope;
}

// Test Class-------------------------------------------------------------------

#define SAMPLE_RATE (100) // replace this with actual sample rate

// void test(madMatrix gyroscopeMisalignment,
//     madVector gyroscopeSensitivity,
//     madVector gyroscopeOffset,
//     madMatrix accelerometerMisalignment,
//     madVector accelerometerSensitivity,
//     madVector accelerometerOffset,
//     madMatrix softIronMatrix,
//     madVector hardIronOffset, 
//     madOffset offset, 
//     madAhrs *ahrs,
//     SensorData data,
//     FILE *file){
//         // Acquire latest sensor data
//         // const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp
//         const float timestamp = data.time;
//         madVector gyroscope = {data.gyroX, data.gyroY, data.gyroZ}; // replace this with actual gyroscope data in degrees/s
//         madVector accelerometer = {data.accelX, data.accelY, data.accelZ}; // replace this with actual accelerometer data in g
//         madVector magnetometer = {data.magX, data.magY, data.magZ}; // replace this with actual magnetometer data in arbitrary units

//         madEuler euler = madQuaternionToEuler(madAhrsGetQuaternion(ahrs));
//         madVector earth = madAhrsGetEarthAcceleration(ahrs);

//         // printf("Before anything Time: %.6f s, Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g, Mag: (%.6f, %.4f, %.6f) uT\n",
//         //     data.time, data.gyroX, data.gyroY, data.gyroZ, data.accelX, data.accelY, data.accelZ, data.magX, data.magY, data.magZ);

//         // printf("Roll %0.3f, Pitch %0.3f, Yaw %0.3f, X %0.3f, Y %0.3f, Z %0.3f\n",
//         //        euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
//         //        earth.axis.x, earth.axis.y, earth.axis.z);

//         // Apply calibration
//         // gyroscope = madCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
//         // accelerometer = madCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
//         // magnetometer = madCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

//         // Update gyroscope offset correction algorithm
//         gyroscope = madOffsetUpdate(&offset, gyroscope);

//         // printf("Offset update Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g, Mag: (%.6f, %.4f, %.6f) uT\n",
//         //     data.gyroX, data.gyroY, data.gyroZ, data.accelX, data.accelY, data.accelZ, data.magX, data.magY, data.magZ);

//         // printf("Roll %0.3f, Pitch %0.3f, Yaw %0.3f, X %0.3f, Y %0.3f, Z %0.3f\n",
//         //        euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
//         //        earth.axis.x, earth.axis.y, earth.axis.z);

//         // Calculate delta time (in seconds) to account for gyroscope sample clock error
//         static float previousTimestamp;
//         float deltaTime = (float) (timestamp - previousTimestamp);
//         previousTimestamp = timestamp;

//         // Update gyroscope AHRS algorithm
//         madAhrsUpdate(ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

//         // printf("After Update - Time: %.6f s, Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g, Mag: (%.6f, %.4f, %.6f) uT, deltaT: %.10f\n",
//         //     data.time, data.gyroX, data.gyroY, data.gyroZ, data.accelX, data.accelY, data.accelZ, data.magX, data.magY, data.magZ, deltaTime);

//         // Print algorithm outputs
//         madAhrsInternalStates internal;
//         madAhrsFlags flags;

//         euler = madQuaternionToEuler(madAhrsGetQuaternion(ahrs));

//         internal = madAhrsGetInternalStates(ahrs);
//         flags = madAhrsGetFlags(ahrs);

//         fprintf(file, "%f,", timestamp);

//         fprintf(file, "%f,%f,%f,", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

//         fprintf(file, "%f,%d,%.0f,%.0f,%d,%.0f,%d,%d,%d,%d", internal.accelerationError,  
//         internal.accelerometerIgnored, internal.accelerationRecoveryTrigger, internal.magneticError, 
//         internal.magnetometerIgnored, internal.magneticRecoveryTrigger, flags.initialising, 
//         flags.angularRateRecovery, flags.accelerationRecovery, flags.magneticRecovery);

//         fprintf(file, "\n");

//         // printf("%f,%d,%.0f,%.0f,%d,%.0f,%d,%d,%d,%d", internal.accelerationError, 
//         // internal.accelerometerIgnored, internal.accelerationRecoveryTrigger, 
//         // internal.magneticError, internal.magnetometerIgnored, internal.magneticRecoveryTrigger, 
//         // flags.initialising, flags.angularRateRecovery, flags.accelerationRecovery, flags.magneticRecovery);

//         // printf("\n");
// }

// void testNoMagInternal(madOffset offset, 
//     madAhrs *ahrs,
//     SensorDataNoMag data,
//     FILE *file)
// {
//     // Acquire latest sensor data
//         // const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp
//         const double timestamp = data.time;
//         madVector gyroscope = {data.gyroX, data.gyroY, data.gyroZ}; // replace this with actual gyroscope data in degrees/s
//         madVector accelerometer = {data.accelX, data.accelY, data.accelZ}; // replace this with actual accelerometer data in g
//         // madVector magnetometer = {data.magX, data.magY, data.magZ}; // replace this with actual magnetometer data in arbitrary units

//         madEuler euler = madQuaternionToEuler(madAhrsGetQuaternion(ahrs));
//         madVector earth = madAhrsGetEarthAcceleration(ahrs);

//         // printf("Before anything Time: %.6f s, Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g, Mag: (%.6f, %.4f, %.6f) uT\n",
//         //     data.time, data.gyroX, data.gyroY, data.gyroZ, data.accelX, data.accelY, data.accelZ, data.magX, data.magY, data.magZ);

//         // printf("Roll %0.3f, Pitch %0.3f, Yaw %0.3f, X %0.3f, Y %0.3f, Z %0.3f\n",
//         //        euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
//         //        earth.axis.x, earth.axis.y, earth.axis.z);

//         // Apply calibration
//         // gyroscope = madCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
//         // accelerometer = madCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
//         // magnetometer = madCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

//         // Update gyroscope offset correction algorithm
//         gyroscope = madOffsetUpdate(&offset, gyroscope);

//         // printf("Offset update Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g, Mag: (%.6f, %.4f, %.6f) uT\n",
//         //     data.gyroX, data.gyroY, data.gyroZ, data.accelX, data.accelY, data.accelZ, data.magX, data.magY, data.magZ);

//         // printf("Roll %0.3f, Pitch %0.3f, Yaw %0.3f, X %0.3f, Y %0.3f, Z %0.3f\n",
//         //        euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
//         //        earth.axis.x, earth.axis.y, earth.axis.z);

//         // Calculate delta time (in seconds) to account for gyroscope sample clock error
//         static double previousTimestamp;
//         double deltaTime = (double) (timestamp - previousTimestamp);
//         previousTimestamp = timestamp;

//         // Update gyroscope AHRS algorithm
//         madAhrsUpdateNoMagnetometer(ahrs, gyroscope, accelerometer, deltaTime);

//         // printf("After Update - Time: %.6f s, Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g, Mag: (%.6f, %.4f, %.6f) uT, deltaT: %.10f\n",
//         //     data.time, data.gyroX, data.gyroY, data.gyroZ, data.accelX, data.accelY, data.accelZ, data.magX, data.magY, data.magZ, deltaTime);

//         // Print algorithm outputs
//         madAhrsInternalStates internal;
//         madAhrsFlags flags;

//         euler = madQuaternionToEuler(madAhrsGetQuaternion(ahrs));

//         internal = madAhrsGetInternalStates(ahrs);
//         flags = madAhrsGetFlags(ahrs);

//         fprintf(file, "%f,", timestamp);

//         fprintf(file, "%f,%f,%f,", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

//         fprintf(file, "%f,%d,%.0f,%.0f,%d,%.0f,%d,%d,%d,%d", internal.accelerationError,  
//         internal.accelerometerIgnored, internal.accelerationRecoveryTrigger, internal.magneticError, 
//         internal.magnetometerIgnored, internal.magneticRecoveryTrigger, flags.initialising, 
//         flags.angularRateRecovery, flags.accelerationRecovery, flags.magneticRecovery);

//         fprintf(file, "%f,%f,%f", earth.axis.x, earth.axis.y, earth.axis.z);

//         fprintf(file, "\n");

// }

// void testNoMag(){

//     FILE *fileOut = fopen("noMag1.txt", "w+"); 
//     if (!fileOut) {
//         fprintf(stderr, "Error opening file...exiting\n");
//         exit(1);
//     }

//     // std::ifstream inputFile(fileOutName); // Opens the file
//     // if (inputFile.is_open()) {
//     //     // File opened successfully
//     //     // Read data from the file
//     //     // ...
//     //     inputFile.close(); // Close the file when done
//     // } else {
//     //     std::cout << "Error opening file." << std::endl;
//     // }

//     // Initialise algorithms
//     madOffset offset;
//     madAhrs ahrs;

//     madOffsetInitialise(&offset, SAMPLE_RATE);
//     madAhrsInitialise(&ahrs);

//     // Set AHRS algorithm settings
//     const madAhrsSettings settings = {
//             .convention = EarthConventionNed,
//             .gain = 0.5f,
//             .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
//             .accelerationRejection = 10.0f,
//             .magneticRejection = 10.0f,
//             .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
//     };

//     madAhrsSetSettings(&ahrs, &settings);

//     FILE *fileIn = fopen("C:/Users/Andrey/Documents/EverestRepo/Apogee-Detection-Everest/MadgwickLibrary/sensor_data.csv", "r");

//     // if (!fileIn) {
//     //     perror("Error opening file");
//     //     // return 1;
//     // }
//     // read first line and preset the deltaTime to timestamp 
//     char line[MAX_LINE_LENGTH];
//     std::clock_t start;
//     double duration;
    
//     while (fgets(line, sizeof(line), fileIn)) {
//         // Tokenize the line using strtok
//         char *token = strtok(line, ",");
//         float time = atof(token); // Convert the time value to float

//         // Parse accelerometer readings (X, Y, Z)
//         token = strtok(NULL, ",");
//         float accelX = atof(token);
//         token = strtok(NULL, ",");
//         float accelY = atof(token);
//         token = strtok(NULL, ",");
//         float accelZ = atof(token);

//         // Parse gyroscope readings (X, Y, Z)
//         token = strtok(NULL, ",");
//         float gyroX = atof(token);
//         token = strtok(NULL, ",");
//         float gyroY = atof(token);
//         token = strtok(NULL, ",");
//         float gyroZ = atof(token);

//         // Parse magnetometer readings (X, Y, Z)
//         // token = strtok(NULL, ",");
//         // float magX = atof(token);
//         // token = strtok(NULL, ",");
//         // float magY = atof(token);
//         // token = strtok(NULL, ",");
//         // float magZ = atof(token);

//         SensorDataNoMag sensorData = {
//             time,
//             gyroX,
//             gyroY,
//             gyroZ,
//             accelX,
//             accelY,
//             accelZ,
//             // magX,
//             // magY,
//             // magZ,
//         };

//         // Example: Print all sensor readings
//         printf("Time: %.6f s, Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g\n",
//                time, gyroX, gyroY, gyroZ, accelX, accelY, accelZ);

//         start = std::clock();

//         testNoMagInternal(offset, &ahrs, sensorData, fileOut);

//         clock_t endTime = std::clock();

//         duration += endTime - start;

//         printf("Time for noMag (seconds): %f\n", duration/CLOCKS_PER_SEC);
//     }

//     printf("Final for noMag: %f", duration/CLOCKS_PER_SEC);

//     fclose(fileIn);
//     fclose(fileOut);

// }

// int main() {

//     // Define calibration (replace with actual calibration data if available)
//     const madMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
//     const madVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
//     const madVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
//     const madMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
//     const madVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
//     const madVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
//     const madMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

//     madAhrsInternalStates internal;
//     madAhrsFlags flags;
//     const madVector hardIronOffset = {0.0f, 0.0f, 0.0f};

//     FILE *file = fopen("infusion.txt", "w+"); // Open the file for appending or create it if it doesn't exist
//     if (!file) {
//         fprintf(stderr, "Error opening file...exiting\n");
//         exit(1);
//     }


//     // Initialise algorithms
//     madOffset offset;
//     madAhrs ahrs;

//     madOffsetInitialise(&offset, SAMPLE_RATE);
//     madAhrsInitialise(&ahrs);

//     // Set AHRS algorithm settings
//     const madAhrsSettings settings = {
//             .convention = EarthConventionNed,
//             .gain = 0.5f,
//             .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
//             .accelerationRejection = 10.0f,
//             .magneticRejection = 10.0f,
//             .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
//     };
//     madAhrsSetSettings(&ahrs, &settings);

//     FILE *file1 = fopen("C:/Users/Andrey/Documents/EverestRepo/Apogee-Detection-Everest/MadgwickLibrary/sensor_data.csv", "r");
//     if (!file1) {
//         perror("Error opening file");
//         return 1;
//     }
//     // read first line and preset the deltaTime to timestamp 
//     char line[MAX_LINE_LENGTH];
//     std::clock_t start;
//     double duration;
    
//     while (fgets(line, sizeof(line), file1)) {
//         // Tokenize the line using strtok
//         char *token = strtok(line, ",");
//         float time = atof(token); // Convert the time value to float

//         // Parse gyroscope readings (X, Y, Z)
//         token = strtok(NULL, ",");
//         float gyroX = atof(token);
//         token = strtok(NULL, ",");
//         float gyroY = atof(token);
//         token = strtok(NULL, ",");
//         float gyroZ = atof(token);

//         // Parse accelerometer readings (X, Y, Z)
//         token = strtok(NULL, ",");
//         float accelX = atof(token);
//         token = strtok(NULL, ",");
//         float accelY = atof(token);
//         token = strtok(NULL, ",");
//         float accelZ = atof(token);

//         // Parse magnetometer readings (X, Y, Z)
//         token = strtok(NULL, ",");
//         float magX = atof(token);
//         token = strtok(NULL, ",");
//         float magY = atof(token);
//         token = strtok(NULL, ",");
//         float magZ = atof(token);

//         SensorData sensorData = {
//             time,
//             gyroX,
//             gyroY,
//             gyroZ,
//             accelX,
//             accelY,
//             accelZ,
//             magX,
//             magY,
//             magZ,
//         };

//         // Example: Print all sensor readings
//         // printf("Time: %.6f s, Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g, Mag: (%.6f, %.6f, %.6f) uT\n",
//         //        time, gyroX, gyroY, gyroZ, accelX, accelY, accelZ, magX, magY, magZ);
//         start = std::clock();

//         test(gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset, 
//         accelerometerMisalignment,accelerometerSensitivity,accelerometerOffset,
//         softIronMatrix, hardIronOffset, offset, &ahrs, sensorData, file);

//         clock_t endTime = std::clock();

//         duration += endTime - start;

//         printf("Time for one more (seconds): %f\n", duration/CLOCKS_PER_SEC);
//     }

//     printf("Overall for (13k samples): %f", duration/CLOCKS_PER_SEC);

//     fclose(file1);
//     fclose(file);

//     testNoMag();

//     // testCircle();

// }
