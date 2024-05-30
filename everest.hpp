
/**
 * @file everest.hpp
 * @brief 
*/


// Definitions

/**
 * @brief Keeps whole system's states, including apogee detection results and confidence values 
 * for each system
*/
typedef union {
    bool apogeeDetected_IMU;
    bool apogeeDetected_Baros;
    bool apogeeDetected_Real_Baro;

    float confidence_IMU;
    float confidence_Baros;
    float confidence_Real_Baro;

    bool finalApogeeDetected;

    IMUData avgIMU;
} systemState;

/**
 * @brief Keeps the data from the IMU sensor #1
*/
typedef struct{
    float time;
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;
} IMUData;

typedef struct{
    float time;
    float pressure;
} BarosData;


