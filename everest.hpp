
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

class Everest{
    public:
        Everest();
        ~Everest();

        void IMU_Update(const IMUData& imu1, const IMUData& imu2, int whichOne);

    private:
        IMUData internalIMU_1;
        IMUData internalIMU_2;

        BarosData baro1, baro2, baro3, realBaro;

        // Initialize system state
        systemState state; 
};



