/**
 * @file everest.hpp
 * @brief 
*/
#ifndef EVEREST_TASK_HPP
#define EVEREST_TASK_HPP

#include "infusion.hpp"

#include <stdio.h>
#include <ctime>
#include <string.h>

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include "C:\Users\andin\OneDrive\Documents\AllRepos\UnscentedKalmanFilter\EverestLibrary_HALO\EverestL\EverestLibrary\HALO.hpp"


// task specific
// #include "Task.hpp"
// #include "Data.h"
// #include "SystemDefines.hpp"


// Definitions
// CHANGE
#define SAMPLE_RATE (3) // replace this with actual sample rate of baros 
#define DELTA_TIME (1.0f / 3.0f)
#define RATE_BARO (3)
#define CALIBRATION_TIME (2)

#define LAPTOP

/* Macros/Enums ------------------------------------------------------------*/
enum EVEREST_TASK_COMMANDS {
    EVEREST_NONE = 0,
	UPDATE,
	TEST,
    RETARE
};

/*Defines------------------------------------------------------------------*/
typedef struct{
    float initialVelo;
    float initialAlt;
    float finalAltitude;
} kinematics;

/**
 * @brief Keeps the data from the IMU sensor #1 time, gyroXYZ, accelXYZ, magXYZ
*/
typedef struct{
    float time;
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;
    float magX, magY, magZ;
    float altitude;
} IMUData;

/**
 * @brief Keeps whole system's states, including apogee detection results and confidence values 
 * for each system
*/
typedef struct {
    float gain_IMU;
    float gain_Baro1;
    float gain_Baro2;

    // prev gains
    float prev_gain_IMU;
    float prev_gain_Baro1;
    float prev_gain_Baro2;

    float std_IMU;
    float std_Baro1;
    float std_Baro2;

    IMUData avgIMU;
    float deltaTimeIMU;
    float earthAcceleration;
} systemState;

typedef struct{
    float secondLastAltitude;
    float lastAltitude;
} altitudeList;

typedef struct{
    float time;
    float pressure;
    float altitude;

    float deltaTime;
    float previousTime;
} BarosData;

/**
 * @brief Just to carry all data between tasks
*/
typedef struct{
    float timeIMU1;
    float timeIMU2;
    float timeBaro1;
    float timeBaro2;

    float pressure1;
    float pressure2;

    float accelX1;
    float accelY1;
    float accelZ1;
    float gyroX1;
    float gyroY1;
    float gyroZ1;
    float magX1;
    float magY1;
    float magZ1;

    float accelX2;
    float accelY2;
    float accelZ2;
    float gyroX2;
    float gyroY2;
    float gyroZ2;
    float magX2;
    float magY2;
    float magZ2;

} EverestData;

class EverestTask
{
    public:

        // static EverestTask& Inst() {
		// 	static EverestTask inst;
		// 	return inst;
		// }

		// void InitTask();

        void IMU_Update(const IMUData& imu1, const IMUData& imu2);

        Infusion* ExternalInitialize();

        static EverestTask getEverest();

        // Initialize system state
        systemState state = {};

        void Baro_Update(const BarosData& baro1, const BarosData& baro2);

        double dynamite();

        kinematics Kinematics;

        kinematics* getKinematics();

        Infusion madgwick;

        altitudeList AltitudeList;

        void recalculateGain(double estimate);

        double deriveChangeInVelocityToGetAltitude(double estimate);

        void MadgwickWrapper(IMUData data);

        // void IMU_Update(const SensorDataNoMag& imu1, const SensorDataNoMag& imu2, float magX, float magY, float magZ);

        double ExternalUpdate(IMUData imu1, IMUData imu2, BarosData baro1, 
                                BarosData baro2);

        double deriveForAltitudeIMU(IMUData avgIMU);

        double AlignedExternalUpdate(IMUData imu1, IMUData imu2, 
                    BarosData baro1, BarosData baro2, MadAxesAlignment alignment);

        void tare(IMUData &imu1, IMUData &imu2, BarosData baro1, BarosData baro2);

        void MadgwickSetup();

        void initialize1(systemState& state);

        void calculateSTDCoefficients();

        double TaskWrapper(EverestData everestData, MadAxesAlignment alignment, MadAxesAlignment alignment2);

        double finalWrapper(    
                float accelX1,  float accelY1,      float accelZ1,
                float gyroX1,   float gyroY1,       float gyroZ1,       float accelX2,
                float magX1,    float magY1,        float magZ1,        float accelY2,  
                float accelZ2,  float gyroX2,       float gyroY2,       float gyroZ2,   
                float magX2,    float magY2,        float magZ2,        float pressure1,
                float pressure2,float timeIMU1,     float timeIMU2,     float timeBaro1,    
                float timeBaro2,MadAxesAlignment alignment, MadAxesAlignment alignment2);

    protected:
        IMUData internalIMU_1, internalIMU_2;

        BarosData baro1, baro2;

        std::vector<double> zeroOffsetAccel = {0, 0, 0};
        std::vector<double> zeroOffsetAccel2= {0, 0, 0};
        std::vector<double> zeroOffsetGyro  = {0, 0, 0};
        std::vector<double> zeroOffsetGyro2 = {0, 0, 0};

        // static void RunTask(void* pvParams) { EverestTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

	    // void Run(void* pvParams);    // Main run code

	    // void HandleCommand(Command& cm);
	    // void HandleRequestCommand(uint16_t taskCommand);

    private:
        // EverestTask();                                        // Private constructor
	    // EverestTask(const EverestTask&);                    // Prevent copy-construction
	    // EverestTask& operator=(const EverestTask&);            // Prevent assignment
};

void EverestTask::initialize1(systemState& state){
    // Initially we trust systems equally
    this->state.gain_IMU = 4/10.0; // change to actual initial trusts 
    this->state.gain_Baro1 = 3/10.0;
    state.gain_Baro2 = 3/10.0;

    Kinematics.initialVelo = 0;
    Kinematics.initialAlt = 0;
    Kinematics.finalAltitude = 0;

    printf("Initialized\n");
}

Infusion* EverestTask::ExternalInitialize(){
    initialize1(state);
    return &madgwick;
}

EverestTask EverestTask::getEverest(){
    EverestTask everest = EverestTask();
    return everest;
}

kinematics* EverestTask::getKinematics(){
    return &Kinematics;
}

#endif


