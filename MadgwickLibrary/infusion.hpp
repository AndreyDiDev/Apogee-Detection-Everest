/**
 * @co-author Andrey Dimanchev
 * @file infusion.hpp
 * @author Seb Madgwick
 * 
 * /**
 * @brief AHRS algorithm to combine gyroscope, accelerometer, and magnetometer
 * measurements into a filtered orientation relative to the Earth's frame of reference
 */
// -----------------------------------------------------------------------------

//Math--------------------------------------------------------------------------
// Includes

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Earth axes convention.
 */
typedef enum {
    EarthConventionNwu, /* North-West-Up */
    EarthConventionEnu, /* East-North-Up */
    EarthConventionNed, /* North-East-Down */
} EarthConvention;

/**
 * @brief 3D vector.
 */
typedef union {
    float array[3];

    struct {
        float x;
        float y;
        float z;
    } axis;
} madVector;

/**
 * @brief Quaternion.
 */
typedef union {
    float array[4];

    struct {
        float w;
        float x;
        float y;
        float z;
    } element;
} madQuaternion;

/**
 * @brief 3x3 matrix in row-major order.
 * See http://en.wikipedia.org/wiki/Row-major_order
 */
typedef union {
    float array[3][3];

    struct {
        float xx;
        float xy;
        float xz;
        float yx;
        float yy;
        float yz;
        float zx;
        float zy;
        float zz;
    } element;
} madMatrix;

/**
 * @brief Euler angles.  Roll, pitch, and yaw correspond to rotations around
 * X, Y, and Z respectively.
 */
typedef union {
    float array[3];

    struct {
        float roll;
        float pitch;
        float yaw;
    } angle;
} madEuler;

#define VECTOR_ZERO ((madVector){ .array = {0.0f, 0.0f, 0.0f} })

#define VECTOR_ONES ((madVector){ .array = {1.0f, 1.0f, 1.0f} })

#define IDENTITY_QUATERNION ((madQuaternion){ .array = {1.0f, 0.0f, 0.0f, 0.0f} })

#define IDENTITY_MATRIX ((madMatrix){ .array = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}} })

#define EULER_ZERO ((madEuler){ .array = {0.0f, 0.0f, 0.0f} })

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

/**
 * @brief Axes alignment describing the sensor axes relative to the body axes.
 * For example, if the body X axis is aligned with the sensor Y axis and the
 * body Y axis is aligned with sensor X axis but pointing the opposite direction
 * then alignment is +Y-X+Z.
 */
typedef enum {
    MadAxesAlignmentPXPYPZ, /* +X+Y+Z */
    MadAxesAlignmentPXNZPY, /* +X-Z+Y */
    MadAxesAlignmentPXNYNZ, /* +X-Y-Z */
    MadAxesAlignmentPXPZNY, /* +X+Z-Y */
    MadAxesAlignmentNXPYNZ, /* -X+Y-Z */
    MadAxesAlignmentNXPZPY, /* -X+Z+Y */
    MadAxesAlignmentNXNYPZ, /* -X-Y+Z */
    MadAxesAlignmentNXNZNY, /* -X-Z-Y */
    MadAxesAlignmentPYNXPZ, /* +Y-X+Z */
    MadAxesAlignmentPYNZNX, /* +Y-Z-X */
    MadAxesAlignmentPYPXNZ, /* +Y+X-Z */
    MadAxesAlignmentPYPZPX, /* +Y+Z+X */
    MadAxesAlignmentNYPXPZ, /* -Y+X+Z */
    MadAxesAlignmentNYNZPX, /* -Y-Z+X */
    MadAxesAlignmentNYNXNZ, /* -Y-X-Z */
    MadAxesAlignmentNYPZNX, /* -Y+Z-X */
    MadAxesAlignmentPZPYNX, /* +Z+Y-X */
    MadAxesAlignmentPZPXPY, /* +Z+X+Y */
    MadAxesAlignmentPZNYPX, /* +Z-Y+X */
    MadAxesAlignmentPZNXNY, /* +Z-X-Y */
    MadAxesAlignmentNZPYPX, /* -Z+Y+X */
    MadAxesAlignmentNZNXPY, /* -Z-X+Y */
    MadAxesAlignmentNZNYNX, /* -Z-Y-X */
    MadAxesAlignmentNZPXNY, /* -Z+X-Y */
} MadAxesAlignment;

/**
 * @brief Gyroscope offset algorithm structure.  Structure members are used
 * internally and must not be accessed by the application.
 */
typedef struct {
    float filterCoefficient;
    unsigned int timeout;
    unsigned int timer;
    madVector gyroscopeOffset;
} madOffset;

// Definitions
typedef struct {
    float time;
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;
    float magX, magY, magZ;
}SensorData;

typedef struct {
    double time;
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;
    float magX, magY, magZ;
}SensorDataNoMag;

/**
 * @brief AHRS algorithm settings.
 */
typedef struct {
    EarthConvention convention;
    float gain;
    float gyroscopeRange;
    float accelerationRejection;
    float magneticRejection;
    unsigned int recoveryTriggerPeriod;
} madAhrsSettings;

/**
 * @brief AHRS algorithm structure.  Structure members are used internally and
 * must not be accessed by the application.
 */
typedef struct {
    madAhrsSettings settings;
    madQuaternion quaternion;
    madVector accelerometer;
    bool initialising;
    float rampedGain;
    float rampedGainStep;
    bool angularRateRecovery;
    madVector halfAccelerometerFeedback;
    madVector halfMagnetometerFeedback;
    bool accelerometerIgnored;
    int accelerationRecoveryTrigger;
    int accelerationRecoveryTimeout;
    bool magnetometerIgnored;
    int magneticRecoveryTrigger;
    int magneticRecoveryTimeout;
} madAhrs;

/**
 * @brief AHRS algorithm internal states.
 */
typedef struct {
    float accelerationError;
    bool accelerometerIgnored;
    float accelerationRecoveryTrigger;
    float magneticError;
    bool magnetometerIgnored;
    float magneticRecoveryTrigger;
} madAhrsInternalStates;

/**
 * @brief AHRS algorithm flags.
 */
typedef struct {
    bool initialising;
    bool angularRateRecovery;
    bool accelerationRecovery;
    bool magneticRecovery;
} madAhrsFlags;


//------------------------------------------------------------------------------
/**
 * @brief Converts degrees to radians.
 * @param degrees Degrees.
 * @return Radians.
 */
static inline float DegreesToRadians(const float degrees) {
    return degrees * ((float) M_PI / 180.0f);
}

/**
 * @brief Converts radians to degrees.
 * @param radians Radians.
 * @return Degrees.
 */
static inline float RadiansToDegrees(const float radians) {
    return radians * (180.0f / (float) M_PI);
}

//------------------------------------------------------------------------------

/**
 * @brief Returns the arc sine of the value.
 * @param value Value.
 * @return Arc sine of the value.
 */
static inline float Asin(const float value) {
    if (value <= -1.0f) {
        return (float) M_PI / -2.0f;
    }
    if (value >= 1.0f) {
        return (float) M_PI / 2.0f;
    }
    return asinf(value);
}

//------------------------------------------------------------------------------

#ifndef NORMAL_SQRT

/**
 * @brief Calculates the reciprocal of the square root.
 * See https://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/
 * @param x Operand.
 * @return Reciprocal of the square root of x.
 */
static inline float fastInverseSqrt(const float x) {

    typedef union {
        float f;
        int32_t i;
    } Union32;

    Union32 union32 = {.f = x};
    union32.i = 0x5F1F1412 - (union32.i >> 1);
    return union32.f * (1.69000231f - 0.714158168f * x * union32.f * union32.f);
}

#endif

//Vector Operations-------------------------------------------------------------------------
/**
 * @brief Returns true if the vector is zero.
 * @param vector Vector.
 * @return True if the vector is zero.
 */
static inline bool madVectorIsZero(const madVector vector) {
    return (vector.axis.x == 0.0f) && (vector.axis.y == 0.0f) && (vector.axis.z == 0.0f);
}

/**
 * @brief Returns the sum of two vectors.
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Sum of two vectors.
 */
static inline madVector madVectorAdd(const madVector vectorA, const madVector vectorB) {
    const madVector result = {.axis = {
            .x = vectorA.axis.x + vectorB.axis.x,
            .y = vectorA.axis.y + vectorB.axis.y,
            .z = vectorA.axis.z + vectorB.axis.z,
    }};
    return result;
}

/**
 * @brief Returns vector B subtracted from vector A.
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Vector B subtracted from vector A.
 */
static inline madVector madVectorSubtract(const madVector vectorA, const madVector vectorB) {
    const madVector result = {.axis = {
            .x = vectorA.axis.x - vectorB.axis.x,
            .y = vectorA.axis.y - vectorB.axis.y,
            .z = vectorA.axis.z - vectorB.axis.z,
    }};
    return result;
}

/**
 * @brief Returns the sum of the elements.
 * @param vector Vector.
 * @return Sum of the elements.
 */
static inline float madVectorSum(const madVector vector) {
    return vector.axis.x + vector.axis.y + vector.axis.z;
}

/**
 * @brief Returns the multiplication of a vector by a scalar.
 * @param vector Vector.
 * @param scalar Scalar.
 * @return Multiplication of a vector by a scalar.
 */
static inline madVector madVectorMultiplyScalar(const madVector vector, const float scalar) {
    const madVector result = {.axis = {
            .x = vector.axis.x * scalar,
            .y = vector.axis.y * scalar,
            .z = vector.axis.z * scalar,
    }};
    return result;
}

/**
 * @brief Calculates the Hadamard product (element-wise multiplication).
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Hadamard product.
 */
static inline madVector madVectorHadamardProduct(const madVector vectorA, const madVector vectorB) {
    const madVector result = {.axis = {
            .x = vectorA.axis.x * vectorB.axis.x,
            .y = vectorA.axis.y * vectorB.axis.y,
            .z = vectorA.axis.z * vectorB.axis.z,
    }};
    return result;
}

/**
 * @brief Returns the cross product.
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Cross product.
 */
static inline madVector madVectorCrossProduct(const madVector vectorA, const madVector vectorB) {
#define A vectorA.axis
#define B vectorB.axis
    const madVector result = {.axis = {
            .x = A.y * B.z - A.z * B.y,
            .y = A.z * B.x - A.x * B.z,
            .z = A.x * B.y - A.y * B.x,
    }};
    return result;
#undef A
#undef B
}

/**
 * @brief Returns the dot product.
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Dot product.
 */
static inline float madVectorDotProduct(const madVector vectorA, const madVector vectorB) {
    return madVectorSum(madVectorHadamardProduct(vectorA, vectorB));
}

/**
 * @brief Returns the vector magnitude squared.
 * @param vector Vector.
 * @return Vector magnitude squared.
 */
static inline float madVectorMagnitudeSquared(const madVector vector) {
    return madVectorSum(madVectorHadamardProduct(vector, vector));
}

/**
 * @brief Returns the vector magnitude.
 * @param vector Vector.
 * @return Vector magnitude.
 */
static inline float madVectorMagnitude(const madVector vector) {
    return sqrtf(madVectorMagnitudeSquared(vector));
}

/**
 * @brief Returns the normalised vector.
 * @param vector Vector.
 * @return Normalised vector.
 */
static inline madVector madVectorNormalise(const madVector vector) {
#ifdef NORMAL_SQRT
    const float magnitudeReciprocal = 1.0f / sqrtf(madVectorMagnitudeSquared(vector));
#else
    const float magnitudeReciprocal = fastInverseSqrt(madVectorMagnitudeSquared(vector));
#endif
    return madVectorMultiplyScalar(vector, magnitudeReciprocal);
}

//Quaternion operations-----------------------------------------------------------------
/**
 * @brief Returns the sum of two quaternions.
 * @param quaternionA Quaternion A.
 * @param quaternionB Quaternion B.
 * @return Sum of two quaternions.
 */
static inline madQuaternion madQuaternionAdd(const madQuaternion quaternionA, const madQuaternion quaternionB) {
    const madQuaternion result = {.element = {
            .w = quaternionA.element.w + quaternionB.element.w,
            .x = quaternionA.element.x + quaternionB.element.x,
            .y = quaternionA.element.y + quaternionB.element.y,
            .z = quaternionA.element.z + quaternionB.element.z,
    }};
    return result;
}

/**
 * @brief Returns the multiplication of two quaternions.
 * @param quaternionA Quaternion A (to be post-multiplied).
 * @param quaternionB Quaternion B (to be pre-multiplied).
 * @return Multiplication of two quaternions.
 */
static inline madQuaternion madQuaternionMultiply(const madQuaternion quaternionA, const madQuaternion quaternionB) {
#define A quaternionA.element
#define B quaternionB.element
    const madQuaternion result = {.element = {
            .w = A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z,
            .x = A.w * B.x + A.x * B.w + A.y * B.z - A.z * B.y,
            .y = A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x,
            .z = A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w,
    }};
    return result;
#undef A
#undef B
}

/**
 * @brief Returns the multiplication of a quaternion with a vector.  This is a
 * normal quaternion multiplication where the vector is treated a
 * quaternion with a W element value of zero.  The quaternion is post-
 * multiplied by the vector.
 * @param quaternion Quaternion.
 * @param vector Vector.
 * @return Multiplication of a quaternion with a vector.
 */
static inline madQuaternion madQuaternionMultiplyVector(const madQuaternion quaternion, const madVector vector) {
#define Q quaternion.element
#define V vector.axis
    const madQuaternion result = {.element = {
            .w = -Q.x * V.x - Q.y * V.y - Q.z * V.z,
            .x = Q.w * V.x + Q.y * V.z - Q.z * V.y,
            .y = Q.w * V.y - Q.x * V.z + Q.z * V.x,
            .z = Q.w * V.z + Q.x * V.y - Q.y * V.x,
    }};
    return result;
#undef Q
#undef V
}

/**
 * @brief Returns the normalised quaternion.
 * @param quaternion Quaternion.
 * @return Normalised quaternion.
 */
static inline madQuaternion madQuaternionNormalise(const madQuaternion quaternion) {
#define Q quaternion.element
#ifdef NORMAL_SQRT
    const float magnitudeReciprocal = 1.0f / sqrtf(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
#else
    const float magnitudeReciprocal = fastInverseSqrt(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
#endif
    const madQuaternion result = {.element = {
            .w = Q.w * magnitudeReciprocal,
            .x = Q.x * magnitudeReciprocal,
            .y = Q.y * magnitudeReciprocal,
            .z = Q.z * magnitudeReciprocal,
    }};
    return result;
#undef Q
}

// Matrix operations---------------------------------------------------------------
/**
 * @brief Returns the multiplication of a matrix with a vector.
 * @param matrix Matrix.
 * @param vector Vector.
 * @return Multiplication of a matrix with a vector.
 */
static inline madVector madMatrixMultiplyVector(const madMatrix matrix, const madVector vector) {
#define R matrix.element
    const madVector result = {.axis = {
            .x = R.xx * vector.axis.x + R.xy * vector.axis.y + R.xz * vector.axis.z,
            .y = R.yx * vector.axis.x + R.yy * vector.axis.y + R.yz * vector.axis.z,
            .z = R.zx * vector.axis.x + R.zy * vector.axis.y + R.zz * vector.axis.z,
    }};
    return result;
#undef R
}

//Conversion Operations----------------------------------------------------------------
/**
 * @brief Converts a quaternion to a rotation matrix.
 * @param quaternion Quaternion.
 * @return Rotation matrix.
 */
static inline madMatrix madQuaternionToMatrix(const madQuaternion quaternion) {
#define Q quaternion.element
    const float qwqw = Q.w * Q.w; // calculate common terms to avoid repeated operations
    const float qwqx = Q.w * Q.x;
    const float qwqy = Q.w * Q.y;
    const float qwqz = Q.w * Q.z;
    const float qxqy = Q.x * Q.y;
    const float qxqz = Q.x * Q.z;
    const float qyqz = Q.y * Q.z;
    const madMatrix matrix = {.element = {
            .xx = 2.0f * (qwqw - 0.5f + Q.x * Q.x),
            .xy = 2.0f * (qxqy - qwqz),
            .xz = 2.0f * (qxqz + qwqy),
            .yx = 2.0f * (qxqy + qwqz),
            .yy = 2.0f * (qwqw - 0.5f + Q.y * Q.y),
            .yz = 2.0f * (qyqz - qwqx),
            .zx = 2.0f * (qxqz - qwqy),
            .zy = 2.0f * (qyqz + qwqx),
            .zz = 2.0f * (qwqw - 0.5f + Q.z * Q.z),
    }};
    return matrix;
#undef Q
}

/**
 * @brief Converts a quaternion to ZYX Euler angles in degrees.
 * @param quaternion Quaternion.
 * @return Euler angles in degrees.
 */
static inline madEuler madQuaternionToEuler(const madQuaternion quaternion) {
#define Q quaternion.element
    const float halfMinusQySquared = 0.5f - Q.y * Q.y; // calculate common terms to avoid repeated operations
    const madEuler euler = {.angle = {
            .roll = RadiansToDegrees(atan2f(Q.w * Q.x + Q.y * Q.z, halfMinusQySquared - Q.x * Q.x)),
            .pitch = RadiansToDegrees(Asin(2.0f * (Q.w * Q.y - Q.z * Q.x))),
            .yaw = RadiansToDegrees(atan2f(Q.w * Q.z + Q.x * Q.y, halfMinusQySquared - Q.z * Q.z)),
    }};
    return euler;
#undef Q
}

// Function declarations--------------------------------------------------------------

void madAhrsInitialise(madAhrs *const ahrs);

void madAhrsReset(madAhrs *const ahrs);

void madAhrsSetSettings(madAhrs *const ahrs, const madAhrsSettings *const settings);

void madAhrsUpdate(madAhrs *const ahrs, const madVector gyroscope, const madVector accelerometer, const madVector magnetometer, const float deltaTime);

void madAhrsUpdateNoMagnetometer(madAhrs *const ahrs, const madVector gyroscope, const madVector accelerometer, const float deltaTime);

void madAhrsUpdateExternalHeading(madAhrs *const ahrs, const madVector gyroscope, const madVector accelerometer, const float heading, const float deltaTime);

void madAhrsSetQuaternion(madAhrs *const ahrs, const madQuaternion quaternion);

madAhrsInternalStates madAhrsGetInternalStates(const madAhrs *const ahrs);

madAhrsFlags madAhrsGetFlags(const madAhrs *const ahrs);

void madAhrsSetHeading(madAhrs *const ahrs, const float heading);

static inline madVector HalfGravity(const madAhrs *const ahrs);

static inline madVector HalfMagnetic(const madAhrs *const ahrs);

static inline madVector Feedback(const madVector sensor, const madVector reference);

static inline int Clamp(const int value, const int min, const int max);


//------------------------------------------------------------------------------
/**
 * @brief Swaps sensor axes for alignment with the body axes.
 */

//------------------------------------------------------------------------------

//Functions---------------------------------------------------------------------
/**
 * @brief Swaps sensor axes for alignment with the body axes.
 * @param sensor Sensor axes.
 * @param alignment Axes alignment.
 * @return Sensor axes aligned with the body axes.
 */
static inline madVector AxesSwitch(const madVector sensor, const MadAxesAlignment alignment) {
    madVector result;
    switch (alignment) {
        case MadAxesAlignmentPXPYPZ:
            break;
        case MadAxesAlignmentPXNZPY:
            result.axis.x = +sensor.axis.x;
            result.axis.y = -sensor.axis.z;
            result.axis.z = +sensor.axis.y;
            return result;
        case MadAxesAlignmentPXNYNZ:
            result.axis.x = +sensor.axis.x;
            result.axis.y = -sensor.axis.y;
            result.axis.z = -sensor.axis.z;
            return result;
        case MadAxesAlignmentPXPZNY:
            result.axis.x = +sensor.axis.x;
            result.axis.y = +sensor.axis.z;
            result.axis.z = -sensor.axis.y;
            return result;
        case MadAxesAlignmentNXPYNZ:
            result.axis.x = -sensor.axis.x;
            result.axis.y = +sensor.axis.y;
            result.axis.z = -sensor.axis.z;
            return result;
        case MadAxesAlignmentNXPZPY:
            result.axis.x = -sensor.axis.x;
            result.axis.y = +sensor.axis.z;
            result.axis.z = +sensor.axis.y;
            return result;
        case MadAxesAlignmentNXNYPZ:
            result.axis.x = -sensor.axis.x;
            result.axis.y = -sensor.axis.y;
            result.axis.z = +sensor.axis.z;
            return result;
        case MadAxesAlignmentNXNZNY:
            result.axis.x = -sensor.axis.x;
            result.axis.y = -sensor.axis.z;
            result.axis.z = -sensor.axis.y;
            return result;
        case MadAxesAlignmentPYNXPZ:
            result.axis.x = +sensor.axis.y;
            result.axis.y = -sensor.axis.x;
            result.axis.z = +sensor.axis.z;
            return result;
        case MadAxesAlignmentPYNZNX:
            result.axis.x = +sensor.axis.y;
            result.axis.y = -sensor.axis.z;
            result.axis.z = -sensor.axis.x;
            return result;
        case MadAxesAlignmentPYPXNZ:
            result.axis.x = +sensor.axis.y;
            result.axis.y = +sensor.axis.x;
            result.axis.z = -sensor.axis.z;
            return result;
        case MadAxesAlignmentPYPZPX:
            result.axis.x = +sensor.axis.y;
            result.axis.y = +sensor.axis.z;
            result.axis.z = +sensor.axis.x;
            return result;
        case MadAxesAlignmentNYPXPZ:
            result.axis.x = -sensor.axis.y;
            result.axis.y = +sensor.axis.x;
            result.axis.z = +sensor.axis.z;
            return result;
        case MadAxesAlignmentNYNZPX:
            result.axis.x = -sensor.axis.y;
            result.axis.y = -sensor.axis.z;
            result.axis.z = +sensor.axis.x;
            return result;
        case MadAxesAlignmentNYNXNZ:
            result.axis.x = -sensor.axis.y;
            result.axis.y = -sensor.axis.x;
            result.axis.z = -sensor.axis.z;
            return result;
        case MadAxesAlignmentNYPZNX:
            result.axis.x = -sensor.axis.y;
            result.axis.y = +sensor.axis.z;
            result.axis.z = -sensor.axis.x;
            return result;
        case MadAxesAlignmentPZPYNX:
            result.axis.x = +sensor.axis.z;
            result.axis.y = +sensor.axis.y;
            result.axis.z = -sensor.axis.x;
            return result;
        case MadAxesAlignmentPZPXPY:
            result.axis.x = +sensor.axis.z;
            result.axis.y = +sensor.axis.x;
            result.axis.z = +sensor.axis.y;
            return result;
        case MadAxesAlignmentPZNYPX:
            result.axis.x = +sensor.axis.z;
            result.axis.y = -sensor.axis.y;
            result.axis.z = +sensor.axis.x;
            return result;
        case MadAxesAlignmentPZNXNY:
            result.axis.x = +sensor.axis.z;
            result.axis.y = -sensor.axis.x;
            result.axis.z = -sensor.axis.y;
            return result;
        case MadAxesAlignmentNZPYPX:
            result.axis.x = -sensor.axis.z;
            result.axis.y = +sensor.axis.y;
            result.axis.z = +sensor.axis.x;
            return result;
        case MadAxesAlignmentNZNXPY:
            result.axis.x = -sensor.axis.z;
            result.axis.y = -sensor.axis.x;
            result.axis.z = +sensor.axis.y;
            return result;
        case MadAxesAlignmentNZNYNX:
            result.axis.x = -sensor.axis.z;
            result.axis.y = -sensor.axis.y;
            result.axis.z = -sensor.axis.x;
            return result;
        case MadAxesAlignmentNZPXNY:
            result.axis.x = -sensor.axis.z;
            result.axis.y = +sensor.axis.x;
            result.axis.z = -sensor.axis.y;
            return result;
    }
    return sensor; // avoid compiler warning
}

//------------------------------------------------------------------------------

/**
 * @brief Gyroscope, accelerometer, and magnetometer calibration models.
 */

//Functions---------------------------------------------------------------------
/**
 * @brief Gyroscope and accelerometer calibration model.
 * @param uncalibrated Uncalibrated measurement.
 * @param misalignment Misalignment matrix.
 * @param sensitivity Sensitivity.
 * @param offset Offset.
 * @return Calibrated measurement.
 */
static inline madVector madCalibrationInertial(const madVector uncalibrated, const madMatrix misalignment, const madVector sensitivity, const madVector offset) {
    return madMatrixMultiplyVector(misalignment, madVectorHadamardProduct(madVectorSubtract(uncalibrated, offset), sensitivity));
}

/**
 * @brief Magnetometer calibration model.
 * @param uncalibrated Uncalibrated measurement.
 * @param softIronMatrix Soft-iron matrix.
 * @param hardIronOffset Hard-iron offset.
 * @return Calibrated measurement.
 */
static inline madVector madCalibrationMagnetic(const madVector uncalibrated, const madMatrix softIronMatrix, const madVector hardIronOffset) {
    return madMatrixMultiplyVector(softIronMatrix, madVectorSubtract(uncalibrated, hardIronOffset));
}

//------------------------------------------------------------------------------

/**
 * @brief Tilt-compensated compass to calculate the magnetic heading using
 * accelerometer and magnetometer measurements.
 */

// Function declarations--------------------------------------------------------

float compassCalculateHeading(const EarthConvention convention, const madVector accelerometer, const madVector magnetometer);

//------------------------------------------------------------------------------
/**
 * @brief Gyroscope offset correction algorithm for run-time calibration of the
 * gyroscope offset.
 */

//------------------------------------------------------------------------------

//Function declarations---------------------------------------------------------

void madOffsetInitialise(madOffset *const offset, const unsigned int sampleRate);

madVector madOffsetUpdate(madOffset *const offset, madVector gyroscope);

// The End