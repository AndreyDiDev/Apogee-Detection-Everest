
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

#include "C:\Users\Andrey\Documents\EverestRepo\Apogee-Detection-Everest\EverestLibrary\infusion.hpp"
// Definitions
// CHANGE
#define SAMPLE_RATE (3) // replace this with actual sample rate
#define DELTA_TIME (1.0f / 3.0f)
#define RATE_BARO (3)
#define CALIBRATION_TIME (2)

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

    // prev gains
    float prev_gain_IMU;
    float prev_gain_Baro1;
    float prev_gain_Baro2;
    float prev_gain_Baro3;
    float prev_gain_Real_Baro;

    float std_IMU;
    float std_Baro1;
    float std_Baro2;
    float std_Baro3;
    float std_Real_Baro;

    SensorDataNoMag avgIMU;
    float deltaTimeIMU;
    float earthAcceleration;
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

        void IMU_Update(const SensorDataNoMag& imu1, const SensorDataNoMag& imu2);

        Infusion* ExternalInitialize();

        static Everest getEverest();

        systemState state = {};

        void Baro_Update(const BarosData& baro1, const BarosData& baro2, const BarosData& baro3, const BarosData& realBaro);

        double dynamite();

        kinematics Kinematics;

        kinematics* getKinematics();

        Infusion madgwick;

        altitudeList AltitudeList;

        void recalculateGain(double estimate);

        double deriveChangeInVelocityToGetAltitude(double estimate);

        void MadgwickWrapper(SensorDataNoMag data);

        // void IMU_Update(const SensorDataNoMag& imu1, const SensorDataNoMag& imu2, float magX, float magY, float magZ);

        double ExternalUpdate(SensorDataNoMag imu1, SensorDataNoMag imu2, BarosData baro1, 
                                BarosData baro2, BarosData baro3, BarosData realBaro);

        double deriveForAltitudeIMU(SensorDataNoMag avgIMU);

        double AlignedExternalUpdate(SensorDataNoMag imu1, SensorDataNoMag imu2, 
                    BarosData baro1, BarosData baro2, BarosData baro3, BarosData realBaro, MadAxesAlignment alignment);

        void tare(SensorDataNoMag &imu1, SensorDataNoMag &imu2, BarosData baro1, BarosData baro2, BarosData baro3, BarosData realBaro);

        void MadgwickSetup();

        void initialize1(systemState& state, kinematics& Kinematics);

        void calculateSTDCoefficients();

    protected:
        SensorDataNoMag internalIMU_1, internalIMU_2;

        BarosData baro1, baro2, baro3, realBaro;
        
        
};

void Everest::initialize1(systemState& state, kinematics& kinematics){
    // Initially we trust systems equally
    this->state.gain_IMU = 4/10.0; // change to actual initial trusts 
    this->state.gain_Baro1 = 1/10.0;
    this->state.gain_Baro2 = 1/10.0;
    this->state.gain_Baro3 = 1/10.0;
    this->state.gain_Real_Baro = 3/10.0;

    state.std_IMU = 0.05;
    state.std_Baro1 = 0.1;
    state.std_Baro2 = 0.1;
    state.std_Baro3 = 0.1;
    state.std_Real_Baro = 0.05;

    Kinematics.initialVelo = 0;
    this->Kinematics.initialAlt = 1111; // average of new mexico
    Kinematics.finalAltitude = 0;

    printf("Initialized\n");
}

Infusion* Everest::ExternalInitialize(){
    initialize1(state, this->Kinematics);
    return &madgwick;
}

Everest Everest::getEverest(){
    Everest everest = Everest();
    return everest;
}

kinematics* Everest::getKinematics(){
    return &this->Kinematics;
}

#endif


