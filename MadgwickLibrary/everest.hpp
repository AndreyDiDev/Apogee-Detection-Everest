
/**
 * @file everest.hpp
 * @brief 
*/
#ifndef EVEREST_HPP
#define EVEREST_HPP

#include <stdio.h>
#include <ctime>
#include <string.h>

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>

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
typedef struct {
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
    float deltaTimeIMU;
} systemState;

typedef struct{
    float secondLastAltitude;
    float lastAltitude;
} altitudeList;

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
    float altitude;

    float deltaTime;
    float previousTime;
} BarosData;

class Everest{
    public:
        // Everest();
        // ~Everest();

        void IMU_Update(const SensorDataNoMag& imu1, const SensorDataNoMag& imu2);

        Infusion* Initialize();

        static Everest getEverest();

        // Initialize system state
        systemState state = {};

        void Baro_Update(const BarosData& baro1, const BarosData& baro2, const BarosData& baro3, const BarosData& realBaro);

        double dynamite();

        kinematics Kinematics;

        kinematics* getKinematics();

        Infusion madgwick;

        // Infusion madgwick2;

        altitudeList AltitudeList;

        void recalculateGain(double estimate);

        double deriveForVelocity(double estimate);

        void MadgwickWrapper(SensorDataNoMag data);

        // void IMU_Update(const SensorDataNoMag& imu1, const SensorDataNoMag& imu2, float magX, float magY, float magZ);

        double ExternalUpdate(SensorDataNoMag imu1, SensorDataNoMag imu2, BarosData baro1, 
                                BarosData baro2, BarosData baro3, BarosData realBaro);

        double deriveForAltitudeIMU(SensorDataNoMag avgIMU);

        double AlignedExternalUpdate(SensorDataNoMag imu1, SensorDataNoMag imu2, 
                    BarosData baro1, BarosData baro2, BarosData baro3, BarosData realBaro, MadAxesAlignment alignment);
    

    protected:
        SensorDataNoMag internalIMU_1, internalIMU_2;

        BarosData baro1, baro2, baro3, realBaro;

        // Initialize Madgwick Library
        // Infusion madgwick = Infusion();
        
        void initialize(systemState& state);
};

void Everest::initialize(systemState& state){
    // Initially we trust systems equally
    this->state.gain_IMU = 4/10.0; // change to actual initial trusts 
    this->state.gain_Baro1 = 1/10.0;
    state.gain_Baro2 = 1/10.0;
    state.gain_Baro3 = 1/10.0;
    state.gain_Real_Baro = 3/10.0;

    state.std_IMU = 0.05;
    state.std_Baro1 = 0.1;
    state.std_Baro2 = 0.1;
    state.std_Baro3 = 0.1;
    state.std_Real_Baro = 0.05;

    Kinematics.initialVelo = 0;
    Kinematics.initialAlt = 0;
    Kinematics.finalAltitude = 0;

    printf("Initialized\n");
}

Infusion* Everest::Initialize(){
    initialize(state);
    return &madgwick;
}

// Infusion* Everest::Initialize2(){
//     return &madgwick2;
// }

Everest Everest::getEverest(){
    Everest everest = Everest();
    return everest;
}

kinematics* Everest::getKinematics(){
    return &Kinematics;
}

#endif


