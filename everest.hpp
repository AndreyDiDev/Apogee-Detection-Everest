
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

    SensorDataNoMag avgIMU;
} systemState;

/**
 * @brief Keeps the data from the IMU sensor #1
*/
// typedef struct{
//     float time;
//     float gyroX, gyroY, gyroZ;
//     float accelX, accelY, accelZ;
// } IMUData;

typedef struct{
    float time;
    float pressure;
} BarosData;

class Everest{
    public:
        Everest();
        ~Everest();

        void IMU_Update(const SensorDataNoMag& imu1, const SensorDataNoMag& imu2, int whichOne);

    private:
        SensorDataNoMag internalIMU_1;
        SensorDataNoMag internalIMU_2;

        BarosData baro1, baro2, baro3, realBaro;

        // Initialize system state
        systemState state;

        // Initialize Madgwick Library
        Infusion madgwick = Infusion();
        
        void initialize(systemState& state);
};

void initialize(systemState& state){
    // Initially we trust systems equally
    state.confidence_IMU = 1/3.0;
    state.confidence_Baros = 1/3.0;
    state.confidence_Real_Baro = 1/3.0;

    // Initially no apogee detected
    state.apogeeDetected_IMU = false;
    state.apogeeDetected_Baros = false;
    state.apogeeDetected_Real_Baro = false;
    state.finalApogeeDetected = false;
}

Infusion Everest::Everest(){
    initialize(state);
    Infusion madgwick = Infusion();
    return madgwick;
}

// Infusion getMadgwick(){
//     return madgwick;
// }



