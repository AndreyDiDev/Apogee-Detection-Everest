
/**
 * @file everest.hpp
 * @brief 
*/
#ifndef EVEREST_HPP
#define EVEREST_HPP

#include "C:/Users/Andrey/Documents/EverestRepo/Apogee-Detection-Everest/MadgwickLibrary/infusion.hpp"
// Definitions

typedef struct{
    float initialVelo;
    float initialAlt;
    float finalAltitude;
} kinematics;

/**
 * @brief Keeps whole system's states, including apogee detection results and confidence values 
 * for each system
*/
typedef union {
    float gain_IMU;
    float gain_Baro1;
    float gain_Baro2;
    float gain_Baro3;
    float gain_Real_Baro;

    float std_IMU;
    float std_Baro1;
    float std_Baro2;
    float std_Baro3;
    float std_Real_Baro;

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

        void IMU_Update(const SensorDataNoMag& imu1, const SensorDataNoMag& imu2);

        Infusion Initialize();

        static Everest getEverest();

        // Initialize system state
        systemState state;

        void Baro_Update(const BarosData& baro1, const BarosData& baro2, const BarosData& baro3, const BarosData& realBaro);

        systemState dynamite();

    protected:
        SensorDataNoMag internalIMU_1, internalIMU_2;

        BarosData baro1, baro2, baro3, realBaro;

        // Initialize Madgwick Library
        Infusion madgwick = Infusion();
        
        void initialize(systemState& state);
};

void Everest::initialize(systemState& state){
    // Initially we trust systems equally
    state.gain_IMU = 1/3.0; // change to actual initial trusts 
    state.gain_Baro1 = 1/3.0;
    state.gain_Baro2 = 1/3.0;
    state.gain_Baro3 = 1/3.0;
    state.gain_Real_Baro = 1/3.0;

    state.std_IMU = 0.0;
    state.std_Baro1 = 0.0;
    state.std_Baro2 = 0.0;
    state.std_Baro3 = 0.0;
    state.std_Real_Baro = 0.0;

}

Infusion Everest::Initialize(){
    initialize(state);
    Infusion madgwick = Infusion();

    return madgwick;
}

Everest Everest::getEverest(){
    Everest everest = Everest();
    return everest;
}

#endif

