// Altitude estimation using multiple sensors
#include "everestTaskHPP.hpp"
#include <stdio.h>
#include <ctime>
#include <string.h>

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>

// task specific defines
#include "main.h"
#include "Data.h"
#include "DebugTask.hpp"
#include "Task.hpp"
#include "DMBProtocolTask.hpp"
#include "TelemetryMessage.hpp"
#include "FlashTask.hpp"
#include <string.h>


using namespace std;

// SETTINGS (mostly for debugging, keep default for run)
enum debug_level{
    RAW = 0,        // raw data
    Secondary = 1,  // all operations before dynamite
    Dynamite = 2,   //everything during dynamite
    Third = 3,      // after dynamite
    ALL = 4,        // all
    NONE = 5        // none
};
bool isTared = false;
debug_level debug = ALL;
bool firstSampleAfterCalibration = true;
bool useSTD = false;

// INTERNAL VARIABLES
// double timeInSeconds = 2;
double theTime = CALIBRATION_TIME * RATE_BARO;
double sum = 0;
double pressureSum = 0;
static float previousTimestamp = 0;

// FILE *file;

// Instantiate Everest
madAhrs *ahrs;
Infusion *infusion;

EverestTask everest = EverestTask::getEverest();
kinematics *Kinematics = everest.getKinematics(); // tare to ground

madAhrsFlags flags;
madAhrsInternalStates internalStates;
EverestData everestData;


//----------------------------------Task Integration----------------------------------//
EverestTask::EverestTask() : Task(TASK_EVEREST_QUEUE_DEPTH_OBJS)
{
    // Initialize the task
    MadgwickSetup();
    everestData = (EverestData*)soar_malloc(sizeof(EverestData));
}

/**
 * @brief Creates a task for the FreeRTOS Scheduler
 */
void EverestTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Everest task twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)EverestTask::RunTask,
            (const char*)"EverestTask",
            (uint16_t)TASK_EVEREST_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_EVEREST_TASK_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    //Ensure creation succeded
    SOAR_ASSERT(rtValue == pdPASS, "EverestTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief EverestTask run loop
 * @param pvParams Currently unused task context
 */
void EverestTask::Run(void* pvParams)
{

    //Task run loop
    while (1) {

        Command cm;

        //Wait forever for a command
        qEvtQueue->ReceiveWait(cm);

        //Process the command
        HandleCommand(cm);

        cm.Reset();
    }


}

/**
 * @brief Handles a command
 * @param cm Command reference to handle
 */
void EverestTask::HandleCommand(Command& cm)
{
    //TODO: Since this task will stall for a few milliseconds, we may need a way to eat the whole queue (combine similar eg. REQUEST commands and eat to WDG command etc)
    //TODO: Maybe a HandleEvtQueue instead that takes in the whole queue and eats the whole thing in order of non-blocking to blocking

    //Switch for the GLOBAL_COMMAND
    switch (cm.GetCommand()) {
    case TASK_SPECIFIC_COMMAND: {
    	if(cm.GetTaskCommand()==COPY_DATA){
    		*everestData = *(EverestData*) cm.GetDataPointer();
    	}else{
            	*everestData = *(EverestData*) cm.GetDataPointer();
		HandleRequestCommand(cm.GetTaskCommand());
    	}
        break;
    }
    default:
        SOAR_PRINT("EverestTask - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what happens, we must reset allocated data
    cm.Reset();
}

/**
 * @brief Handles a Request Command
 * @param taskCommand The command to handle
 */
void EverestTask::HandleRequestCommand(uint16_t taskCommand)
{
    //Switch for task specific command within DATA_COMMAND
    switch (taskCommand) {
    case UPDATE:
        TaskWrapper(everestData, MadAxesAlignmentPXPYNZ, MadAxesAlignmentPXPYNZ);
        break;
    case TEST:
    	break;
    case RETARE:
        setIsTared(false);
    default:
        SOAR_PRINT("EverestTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
        break;
    }
}

/**
 * @brief Calls finalWrapper with data and alignment
 * 		IMPORTANT: PASS 0s FOR NOT UPDATED MEASUREMENTS (BAROS), 
 * 		PASS INFINITY FOR IMUs (https://en.cppreference.com/w/cpp/numeric/math/isinf) 
 * 		DEFINE LIKE: const double inf = std::numeric_limits<double>::infinity();
*/
double TaskWrapper(EverestData everestData, MadAxesAlignment alignment, MadAxesAlignment alignment2){

    finalWrapper(everestData.accelX1, everestData.accelY1, everestData.accelZ1,
    everestData.gyroX1, everestData.gyroY1, everestData.gyroZ1, everestData.accelX2,
    everestData.accelY2, everestData.accelZ2, everestData.gyroX2, everestData.gyroY2,
    everestData.gyroZ2, everestData.pressure1, everestData.pressure2, everestData.pressure3, 
    everestData.altitudeReal, everestData.timeIMU, everestData.timeIMU, everestData.timeBaro1, 
    everestData.Baro2, everestData.timeBaro3, everestData.timeBaroReal, alignment, alignment2);

}


//----------------------------------EVEREST-------------------------------------------//
/**
 * @brief Only done once. Sets pointers for Madgwick
 *     Internal
*/
void EverestTask::MadgwickSetup()
{
    // Attaches Madgwick to Everest
    infusion = everest.ExternalInitialize();
    ahrs = infusion->getMadAhrs();

    // calculateSTDCoefficients();

    // Define calibration (replace with actual calibration data if available)
    const madMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const madVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const madVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    const madMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const madVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const madVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
    const madMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

    internalStates = infusion->madAhrsGetInternalStates(ahrs);
    flags  = infusion->madAhrsGetFlags(ahrs);

    const madVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    // Initialise algorithms
    madOffset offset = infusion->getOffset();

    infusion->madOffsetInitialise(&offset, SAMPLE_RATE);
    infusion->madAhrsInitialise(ahrs);

    // Set AHRS algorithm settings
    madAhrsSettings settings = {
            EarthConventionEnu,
            0.5f,
            2000.0f, /* replace this with actual gyroscope range in degrees/s */
            10.0f,
            10.0f,
            5 * SAMPLE_RATE, /* 5 seconds */
    };

    infusion->madAhrsSetSettings(ahrs, &settings);

}

/**
 * @brief Wrapper for Madgwick, does offset calc and passes
 *       data to Madgwick
 * 
 *      Internal
 * @param data SensorDataNoMag struct
 * 
*/
void EverestTask::MadgwickWrapper(SensorDataNoMag data){
    // Infusion infusion = infusion;
    const float timestamp = data.time;
    madVector gyroscope = {data.gyroX, data.gyroY, data.gyroZ}; // replace this with actual gyroscope data in degrees/s
    madVector accelerometer = {data.accelX, data.accelY, data.accelZ}; // replace this with actual accelerometer data in g

    // Update gyroscope offset correction algorithm
    madOffset offset = infusion->getOffset();
    gyroscope = infusion->madOffsetUpdate(&offset, gyroscope);

    // printf("Roll %0.3f, Pitch %0.3f, Yaw %0.3f, X %0.3f, Y %0.3f, Z %0.3f\n",
    //        euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
    //        earth.axis.x, earth.axis.y, earth.axis.z);

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    // static float previousTimestamp;
    float deltaTime = (float) (timestamp - previousTimestamp);
    previousTimestamp = timestamp;

    this->state.deltaTimeIMU = deltaTime;

    if(debug == Secondary || debug == ALL){
        SOAR_PRINT("Averaged: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g Time: %f\n",
            data.gyroX, data.gyroY, data.gyroZ, data.accelX, data.accelY, data.accelZ, deltaTime);
    }

    // madVector mag = {.axis = {x, y, z,}};

    // printf("Mag: (%.6f, %.6f, %.6f) uT\n", mag.axis.x, mag.axis.y, mag.axis.z);

    // madVector mag = axis.{x, y, z};

    // Update gyroscope AHRS algorithm
    infusion->madAhrsUpdateNoMagnetometer(ahrs, gyroscope, accelerometer, deltaTime);
    // infusion->madAhrsUpdate(ahrs, gyroscope, accelerometer, mag, deltaTime);

    // madAhrsInternalStates internal;
    // madAhrsFlags flags;

    madEuler euler = infusion->getEuler(ahrs);
    madVector earth = infusion->madAhrsGetEarthAcceleration(ahrs);

    internalStates = infusion->madAhrsGetInternalStates(infusion->getMadAhrs());
    flags = infusion->madAhrsGetFlags(infusion->getMadAhrs());

    // write to file
//    fprintf(file, "%f,", timestamp);
//
//    fprintf(file, "%f,%f,%f,", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
//
//    fprintf(file, "%f,%d,%.0f,%.0f,%d,%.0f,%d,%d,%d,%d,%f", internalStates.accelerationError,
//    internalStates.accelerometerIgnored, internalStates.accelerationRecoveryTrigger, internalStates.magneticError,
//    internalStates.magnetometerIgnored, internalStates.magneticRecoveryTrigger, flags.initialising,
//    flags.angularRateRecovery, flags.accelerationRecovery, flags.magneticRecovery, earth.axis.z);

    everest.state.earthAcceleration = earth.axis.z;

    // fprintf(file, "\n");

//    if(debug == Secondary || debug == ALL){
//        SOAR_PRINT("%f,%d,%.0f,%.0f,%d,%.0f,%d,%d,%d,%d\n", internalStates.accelerationError,
//        internalStates.accelerometerIgnored, internalStates.accelerationRecoveryTrigger,
//        internalStates.magneticError, internalStates.magnetometerIgnored, internalStates.magneticRecoveryTrigger,
//        flags.initialising, flags.angularRateRecovery, flags.accelerationRecovery, flags.magneticRecovery);
//    }

}

/**
 * @brief Averages IMUs and feeds them to Madgwick wrapper
 *      Should be called every time IMU data is updated
 * 
 *    Internal
*/
void EverestTask::IMU_Update(const SensorDataNoMag& imu1, const SensorDataNoMag& imu2)
{
    int numberOfSamples = 2;
    // Update IMU1
    this->internalIMU_1.time = imu1.time;
    this->internalIMU_1.gyroX = imu1.gyroX;
    this->internalIMU_1.gyroZ = imu1.gyroZ;
    this->internalIMU_1.gyroY = imu1.gyroY;

    this->internalIMU_1.accelX = imu1.accelX;
    this->internalIMU_1.accelY = imu1.accelY;
    this->internalIMU_1.accelZ = imu1.accelZ;  

    // Update IMU2
    this->internalIMU_2.time = imu2.time;

    this->internalIMU_2.gyroX = imu2.gyroX;
    this->internalIMU_2.gyroY = imu2.gyroY;
    this->internalIMU_2.gyroZ = imu2.gyroZ;

    this->internalIMU_2.accelX = imu2.accelX;
    this->internalIMU_2.accelY = imu2.accelY;
    this->internalIMU_2.accelZ = imu2.accelZ;

    if (isinf(internalIMU_1.accelX)){
        numberOfSamples -= 1;
        // printf("%d",numberOfSamples);
        this->internalIMU_1.gyroX = 0;
        this->internalIMU_1.gyroY = 0;
        this->internalIMU_1.gyroZ = 0;

        this->internalIMU_1.accelX = 0;
        this->internalIMU_1.accelY = 0;
        this->internalIMU_1.accelZ = 0;
    }

    if (isinf(internalIMU_2.accelX)){
        numberOfSamples -= 1;
        // printf("%d",numberOfSamples);
        this->internalIMU_2.gyroX = 0;
        this->internalIMU_2.gyroY = 0;
        this->internalIMU_2.gyroZ = 0;

        this->internalIMU_2.accelX = 0;
        this->internalIMU_2.accelY = 0;
        this->internalIMU_2.accelZ = 0;
    }

    // Calculate average of IMU parameters
    #define averageIMU this->state.avgIMU

    averageIMU.gyroX = (this->internalIMU_1.gyroX + this->internalIMU_2.gyroX) / numberOfSamples;
    averageIMU.gyroY = (this->internalIMU_1.gyroY + this->internalIMU_2.gyroY) / numberOfSamples;
    averageIMU.gyroZ = (this->internalIMU_1.gyroZ + this->internalIMU_2.gyroZ) / numberOfSamples;

    averageIMU.accelX = (this->internalIMU_1.accelX + this->internalIMU_2.accelX) / numberOfSamples;
    averageIMU.accelY = (this->internalIMU_1.accelY + this->internalIMU_2.accelY) / numberOfSamples;
    averageIMU.accelZ = (this->internalIMU_1.accelZ + this->internalIMU_2.accelZ) / numberOfSamples;

    averageIMU.time = (this->internalIMU_1.time + this->internalIMU_2.time) / numberOfSamples;

    #undef averageIMU

    if(numberOfSamples == 0){
        this->state.avgIMU = {imu1.time,0,0,0,0,0,0};
    }
    // feed to Madgwick
    // MadgwickWrapper(state.avgIMU);
    this->MadgwickWrapper(state.avgIMU);

}

/**
 * @brief updates baro and delta time
 *     Should be called every time baro data is updated
 *      Internal
*/
void EverestTask::Baro_Update(const BarosData& Baro1, const BarosData& Baro2, const BarosData& Baro3, const BarosData& RealBaro)
{
    // Update Baros
    this->baro1.time = Baro1.time;
    this->baro1.pressure = Baro1.pressure;
    this->baro1.deltaTime = Baro1.time - this->baro1.previousTime;
    this->baro1.previousTime = Baro1.time;

    this->baro2.time = Baro2.time;
    this->baro2.pressure = Baro2.pressure;
    this->baro2.deltaTime = Baro2.time - this->baro2.previousTime;
    this->baro2.previousTime = Baro2.time;

    this->baro3.time = Baro3.time;
    this->baro3.pressure = Baro3.pressure;
    this->baro3.deltaTime = Baro3.time - this->baro3.previousTime;
    this->baro3.previousTime = Baro3.time;

    this->realBaro.time = RealBaro.time;
    this->realBaro.pressure = RealBaro.pressure;
    this->realBaro.altitude = RealBaro.altitude;
    this->realBaro.deltaTime = RealBaro.time - this->realBaro.previousTime;
    this->realBaro.previousTime = RealBaro.time;

    if(debug == RAW || debug == ALL){
        SOAR_PRINT("Baro1: %.f Pa, Baro2: %.f Pa, Baro3: %.f Pa, RealBaro: %.f m\n",
            baro1.pressure, baro2.pressure, baro3.pressure, realBaro.altitude);
    }

}

/**
 * @brief Calls IMU and Baro update functions and calculates altitude
 *      calls Dynamite and updates altitude list
 * 
 * @return calculated altitude
 * 
 *    External (only function that should be called after instantiation of Everest to pass
 *  sensor data to Everest for altitude calculation)
*/
double EverestTask::ExternalUpdate(SensorDataNoMag imu1, SensorDataNoMag imu2, BarosData baro1, BarosData baro2, BarosData baro3, BarosData realBaro){
    if(!isTared){
        everest.tare(imu1, imu2, baro1, baro2, baro3, realBaro);
        SOAR_PRINT("Taring in progress\n)");
        return 0;
    }

    everest.IMU_Update(imu1, imu2);

    if(debug == Third || debug == ALL){
        SOAR_PRINT("After IMU Update IMU Altitude: %f\n", everest.state.avgIMU.altitude);
    }

    everest.Baro_Update(baro1, baro2, baro3, realBaro);

    double finalAlt = everest.dynamite();

//    if(debug == Dynamite || debug == ALL){
//        printf("After Dynamite: %f\n", finalAlt);
//    }

    // Update altitude list
    this->AltitudeList.secondLastAltitude = this->AltitudeList.lastAltitude;
    this->AltitudeList.lastAltitude = finalAlt;

//    fprintf(file, ",%f\n", finalAlt); // write to file

    return finalAlt;
}

/**
 * @brief (Currently does not work, use final wrapper) Wraps External Update with alignment, returns External Update with aligned data
*/
double EverestTask::AlignedExternalUpdate(SensorDataNoMag imu1, SensorDataNoMag imu2, 
            BarosData baro1, BarosData baro2, BarosData baro3, BarosData realBaro, MadAxesAlignment alignment){
    // align
    madVector alignedIMU1 = infusion->AxesSwitch({imu1.accelX, imu1.accelY, imu1.accelZ}, alignment);
    madVector alignedIMUGyro1 = infusion->AxesSwitch({imu1.gyroX, imu1.gyroY, imu1.gyroZ}, alignment);

    madVector alignedIMU2 = infusion->AxesSwitch({imu2.accelX, imu2.accelY, imu2.accelZ}, alignment);
    madVector alignedIMUGyro2 = infusion->AxesSwitch({imu2.gyroX, imu2.gyroY, imu2.gyroZ}, alignment);

    if(debug == Secondary || debug == ALL){
        SOAR_PRINT("Unaligned IMU1: (%.6f, %.6f, %.6f) g, (%.6f, %.6f, %.6f) deg/s\n",
            imu1.accelX, imu1.accelY, imu1.accelZ, imu1.gyroX, imu1.gyroY, imu1.gyroZ);

        SOAR_PRINT("Unaligned IMU2: (%.6f, %.6f, %.6f) g, (%.6f, %.6f, %.6f) deg/s\n",
            imu2.accelX, imu2.accelY, imu2.accelZ, imu2.gyroX, imu2.gyroY, imu2.gyroZ);

        SOAR_PRINT("Alignment: %d\n", alignment);
    }

    // put aligned data into SensorDataNoMag struct
    imu1.accelX = alignedIMU1.axis.x;
    imu1.accelY = alignedIMU1.axis.y;
    imu1.accelZ = alignedIMU1.axis.z;

    imu1.gyroX = alignedIMUGyro1.axis.x;
    imu1.gyroY = alignedIMUGyro1.axis.y;
    imu1.gyroZ = alignedIMUGyro1.axis.z;

    // IMU 2
    imu2.accelX = alignedIMU2.axis.x;
    imu2.accelY = alignedIMU2.axis.y;
    imu2.accelZ = alignedIMU2.axis.z;

    imu2.gyroX = alignedIMUGyro2.axis.x;
    imu2.gyroY = alignedIMUGyro2.axis.y;
    imu2.gyroZ = alignedIMUGyro2.axis.z;

    if(debug == Secondary || debug == ALL){
        SOAR_PRINT("Aligned IMU1: (%.6f, %.6f, %.6f) g, (%.6f, %.6f, %.6f) deg/s\n",
            imu1.accelX, imu1.accelY, imu1.accelZ, imu1.gyroX, imu1.gyroY, imu1.gyroZ);

        SOAR_PRINT("Aligned IMU2: (%.6f, %.6f, %.6f) g, (%.6f, %.6f, %.6f) deg/s\n",
            imu2.accelX, imu2.accelY, imu2.accelZ, imu2.gyroX, imu2.gyroY, imu2.gyroZ);
    }

    return ExternalUpdate(imu1, imu2, baro1, baro2, baro3, realBaro);
}


/**
 * Calculates altitude using IMU sensor data and kinematic equations.
 * 
 * @param avgIMU with the average sensor data from the IMU
 * 
 * @category Internal | ASYNCHRONOUS
 * 
 * @return calculated altitude
 */
double EverestTask::deriveForAltitudeIMU(SensorDataNoMag avgIMU){
    // double accelerationZ = avgIMU.accelX * -9.81;
    double accelerationZ = everest.state.earthAcceleration * -9.81;
    double initialVelocity = this->getKinematics()->initialVelo;
    double initialAltitude = this->Kinematics.initialAlt;
    double deltaTime = this->state.deltaTimeIMU;

    // Derive altitude from IMU
    double finalVelocity = initialVelocity + accelerationZ * deltaTime;
    double altitude = initialAltitude + (initialVelocity + finalVelocity) * deltaTime / 2.0;

    if(debug == Secondary || debug == ALL){
        SOAR_PRINT("\nKinematics\n");
        SOAR_PRINT("IMU Initial Altitude: %f\n", initialAltitude);
        SOAR_PRINT("IMU Velocity: %f\n", initialVelocity);
        SOAR_PRINT("IMU Acceleration: %f\n", accelerationZ);
        SOAR_PRINT("IMU Delta Time: %f\n", deltaTime);
        SOAR_PRINT("Derived Altitude: %f\n", altitude);
    }

    // return altitude
    return altitude;
}

/**
 * Calculates the altitude based on the given pressure using the barometric formula
 * 
 * @param pressure pressure from baros in Pa
 * 
 * @return altitude in meters
 * 
 * @category Internal
 */
double convertToAltitude(double pressure){

    double seaLevelPressure = 1013.25 ; // sea level pressure in hPa
    pressure = pressure / 100.0; // convert to hPa
    double altitude = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 1/5.2558)); // barometric formula

    // If pressure is less than 100, altitude is 0
    if(pressure < 100){
        altitude = 0;
    }

    if(debug == Dynamite || debug == ALL){
        SOAR_PRINT("\nConversion \n");
        SOAR_PRINT("Pressure: %.f hPa, Altitude: %.f m\n", pressure, altitude);
    }

    return altitude;
}

/**
 * @brief Multi-system trust algorithm. Assumes measurements are updated
 * @returns normalised altitude
 * 
 * @category Internal | Asynchronous 
*/
double EverestTask::dynamite(){
    double IMUAltitude = deriveForAltitudeIMU(everest.state.avgIMU);
    this->state.avgIMU.altitude = IMUAltitude;

    double BaroAltitude1 = convertToAltitude(this->baro1.pressure);
    this->baro1.altitude = BaroAltitude1;

    double BaroAltitude2 = convertToAltitude(everest.baro2.pressure);
    this->baro2.altitude = BaroAltitude2;

    double BaroAltitude3 = convertToAltitude(everest.baro3.pressure);
    this->baro3.altitude = BaroAltitude3;

    double RealBaroAltitude = this->realBaro.altitude;
    // this->realBaro.altitude = RealBaroAltitude;

    if(debug == Dynamite || debug == ALL){
        SOAR_PRINT("\nDynamite\n");
        SOAR_PRINT("Baro1 Altitude: %f\n", BaroAltitude1);
        SOAR_PRINT("Baro2 Altitude: %f\n", BaroAltitude2);
        SOAR_PRINT("Baro3 Altitude: %f\n", BaroAltitude3);
        SOAR_PRINT("Real Baro Altitude: %f\n", RealBaroAltitude);
        SOAR_PRINT("IMU Altitude: %f\n", IMUAltitude);
    }

    // // distributing measurement
    // double distributed_IMU_Altitude = (IMUAltitude * everest.state.gain_IMU)/pow(everest.state.std_IMU, 2);
    // double distributed_Baro_Altitude1 = (BaroAltitude1 * everest.state.gain_Baro1)/pow(everest.state.std_Baro1, 2);
    // double distributed_Baro_Altitude2 = (BaroAltitude2 * everest.state.gain_Baro2)/pow(everest.state.std_Baro2, 2);
    // double distributed_Baro_Altitude3 = (BaroAltitude3 * everest.state.gain_Baro3)/pow(everest.state.std_Baro3,2);
    // double distributed_RealBaro_Altitude = (RealBaroAltitude * everest.state.gain_Real_Baro)/pow(everest.state.std_Real_Baro,2);
    
    // if pressure is zero, set gain to zero
    if(everest.realBaro.altitude == 0){
        everest.state.gain_Real_Baro = 0;
    }else if (everest.state.gain_Real_Baro == 0){
        // if not zero, set gain to previous non-zero gain
        everest.state.gain_Real_Baro = everest.state.prev_gain_Real_Baro;
    }

    // if pressure is zero, set gain to zero
    if(everest.baro1.pressure == 0){
        everest.state.gain_Baro1 = 0;
    }else if (everest.state.gain_Baro1 == 0){
        // if not zero, set gain to previous gain
        everest.state.gain_Baro1 = everest.state.prev_gain_Baro1;
    }

    // if not zero, set gain to zero
    if(everest.baro2.pressure == 0){
        everest.state.gain_Baro2 = 0;
    }else if(everest.state.gain_Baro2 == 0){
        // if not zero, set gain to previous gain
        everest.state.gain_Baro2 = everest.state.prev_gain_Baro2;
    }

    // if pressure is zero, set gain to zero
    if(everest.baro3.pressure == 0){
        everest.state.gain_Baro3 = 0;
    }else if(everest.state.gain_Baro3 == 0){
        // if measurement not zero, set gain to previous gain
        everest.state.gain_Baro3 = everest.state.prev_gain_Baro3;
    }

    // distribute measurements based on gain
    double distributed_IMU_Altitude = IMUAltitude * everest.state.gain_IMU;
    double distributed_Baro_Altitude1 = (BaroAltitude1 * everest.state.gain_Baro1);
    double distributed_Baro_Altitude2 = (BaroAltitude2 * everest.state.gain_Baro2);
    double distributed_Baro_Altitude3 = (BaroAltitude3 * everest.state.gain_Baro3);
    double distributed_RealBaro_Altitude = (RealBaroAltitude * everest.state.gain_Real_Baro);

    if(debug == Dynamite || debug == ALL){
        SOAR_PRINT("\nDistributed\n");
        SOAR_PRINT("Distributed IMU Altitude: %f\n", distributed_IMU_Altitude);
        SOAR_PRINT("Gain IMU: %f\n", everest.state.gain_IMU);
        SOAR_PRINT("STD IMU: %f\n\n", everest.state.std_IMU);

        SOAR_PRINT("Distributed Baro1 Altitude: %f\n", distributed_Baro_Altitude1);
        SOAR_PRINT("Gain Baro1: %f\n", everest.state.gain_Baro1);
        SOAR_PRINT("STD Baro1: %f\n\n", everest.state.std_Baro1);

        SOAR_PRINT("Distributed Baro2 Altitude: %f\n", distributed_Baro_Altitude2);
        SOAR_PRINT("Gain Baro2: %f\n", everest.state.gain_Baro2);
        SOAR_PRINT("STD Baro2: %f\n\n", everest.state.std_Baro2);

        SOAR_PRINT("Distributed Baro3 Altitude: %f\n", distributed_Baro_Altitude3);
        SOAR_PRINT("Gain Baro3: %f\n", everest.state.gain_Baro3);
        SOAR_PRINT("STD Baro3: %f\n\n", everest.state.std_Baro3);

        SOAR_PRINT("Distributed Real Baro Altitude: %f\n", distributed_RealBaro_Altitude);
        SOAR_PRINT("Gain Real Baro: %f\n", everest.state.gain_Real_Baro);
        SOAR_PRINT("STD Real Baro: %f\n\n", everest.state.std_Real_Baro);
    }

    // assumes stds are already converted to coefficients -> variances
    // distributed_IMU_Altitude = distributed_IMU_Altitude / everest.state.std_IMU;
    // distributed_Baro_Altitude1 = distributed_Baro_Altitude1 / everest.state.std_Baro1;
    // distributed_Baro_Altitude2 = distributed_Baro_Altitude2 / everest.state.std_Baro2;
    // distributed_Baro_Altitude3 = distributed_Baro_Altitude3 / everest.state.std_Baro3;
    // distributed_RealBaro_Altitude = distributed_RealBaro_Altitude / everest.state.std_Real_Baro;

    // summation of distributed measurements
    double distributed_Sum = distributed_IMU_Altitude + distributed_Baro_Altitude1 + distributed_Baro_Altitude2 
                            + distributed_Baro_Altitude3 + distributed_RealBaro_Altitude;

    if(debug == Dynamite || debug == ALL){
        SOAR_PRINT("Distributed Sum: %f\n\n", distributed_Sum);
    }

    // summation of standard deviations
    // double sumSTD1 = pow(everest.state.std_IMU + everest.state.std_Baro1 + everest.state.std_Baro2
    //                 + everest.state.std_Baro3 + everest.state.std_Real_Baro, 2);
    // double sumSTD1 = pow(everest.state.gain_IMU, 2) + pow(everest.state.gain_Baro1, 2) + pow(everest.state.gain_Baro2, 2)
    //                 + pow(everest.state.gain_Baro3, 2) + pow(everest.state.gain_Real_Baro, 2);

//    if(debug == Dynamite || debug == ALL){
//        // printf("Sum STD: %f\n\n", sumSTD1);
//    }

    // summation of gains
    double sumGain = everest.state.gain_IMU + everest.state.gain_Baro1 + everest.state.gain_Baro2 
                    + everest.state.gain_Baro3 + everest.state.gain_Real_Baro;

    if(debug == Dynamite || debug == ALL){
        SOAR_PRINT("Sum Gain: %f\n\n", sumGain);
    }

    // normalised altitude
    double normalised_Altitude = (distributed_Sum)/sumGain;
    // normalised_Altitude = normalised_Altitude / sumSTD1;

    // double normalised_Altitude = (distributed_Sum*sumSTD)/(everest.state.gain_IMU);

    if(debug == Dynamite || debug == ALL){
        SOAR_PRINT("Normalised Altitude: %f\n\n", normalised_Altitude);
    }

    // Update Kinematics
    Kinematics.finalAltitude = normalised_Altitude;

    if(debug == Dynamite || debug == ALL){
        SOAR_PRINT("Final Altitude: %f\n\n", Kinematics.finalAltitude);
    }

    // update velocity
    Kinematics.initialVelo = (Kinematics.finalAltitude - Kinematics.initialAlt)/(this->state.deltaTimeIMU);

    if(debug == Dynamite || debug == ALL){
        SOAR_PRINT("Initial Velocity: %f\n", Kinematics.initialVelo);
    }

    // update altitude
    Kinematics.initialAlt = Kinematics.finalAltitude;

    recalculateGain(normalised_Altitude);

    // Save the gains that are not zero as previous gains
    // so once we have recovery phase these old gains are used
    if (everest.state.gain_IMU != 0) {
        everest.state.prev_gain_IMU = everest.state.gain_IMU;
    }
    if (everest.state.gain_Baro1 != 0) {
        everest.state.prev_gain_Baro1 = everest.state.gain_Baro1;
    }
    if (everest.state.gain_Baro2 != 0) {
        everest.state.prev_gain_Baro2 = everest.state.gain_Baro2;
    }
    if (everest.state.gain_Baro3 != 0) {
        everest.state.prev_gain_Baro3 = everest.state.gain_Baro3;
    }
    if (everest.state.gain_Real_Baro != 0) {
        everest.state.prev_gain_Real_Baro = everest.state.gain_Real_Baro;
    }

    if(debug == Dynamite || debug == ALL){
        SOAR_PRINT("Previous Gains\n");
        SOAR_PRINT("Prev Gain IMU: %f\n", everest.state.prev_gain_IMU);
        SOAR_PRINT("Prev Gain Baro1: %f\n", everest.state.prev_gain_Baro1);
        SOAR_PRINT("Prev Gain Baro2: %f\n", everest.state.prev_gain_Baro2);
        SOAR_PRINT("Prev Gain Baro3: %f\n", everest.state.prev_gain_Baro3);
        SOAR_PRINT("Prev Gain Real Baro: %f\n\n", everest.state.prev_gain_Real_Baro);
    }

    return normalised_Altitude;

}

// @brief calculation - new gain = 1 / abs(estimate - measurement)
void EverestTask::recalculateGain(double estimate){
    double gainedEstimate = deriveChangeInVelocityToGetAltitude(estimate); // pre integrated for altitude

    double gain_IMU = 1/fabsf(gainedEstimate-this->state.avgIMU.altitude); // change to previous trusts
    double gain_Baro1 = 1/fabsf(gainedEstimate-this->baro1.altitude);
    double gain_Baro2 = 1/fabsf(gainedEstimate-this->baro2.altitude);
    double gain_Baro3 = 1/fabsf(gainedEstimate-this->baro3.altitude);
    double gain_Real_Baro = 1/fabsf(gainedEstimate-this->realBaro.altitude);

//    if(debug == Third || debug == ALL){
//        printf("\nRecalculate Gain - Before normalization\n");
//        printf("Gain IMU: %f\n", gain_IMU);
//        printf("Gain Baro1: %f\n", gain_Baro1);
//        printf("Gain Baro2: %f\n", gain_Baro2);
//        printf("Gain Baro3: %f\n", gain_Baro3);
//        printf("Gain Real Baro: %f\n", gain_Real_Baro);
//        printf("Gained Estimate: %f\n", gainedEstimate);

//        printf("Altitude: %f\n", estimate);
//        printf("Baro1: %f\n", this->baro1.altitude);
//        printf("Baro2: %f\n", this->baro2.altitude);
//        printf("Baro3: %f\n", this->baro3.altitude);
//        printf("Real Baro: %f\n\n", this->realBaro.altitude);
//    }

    // normalise
    this->state.gain_IMU = gain_IMU / (gain_IMU + gain_Baro1 + gain_Baro2 + gain_Baro3 + gain_Real_Baro);
    this->state.gain_Baro1 = gain_Baro1 / (gain_IMU + gain_Baro1 + gain_Baro2 + gain_Baro3 + gain_Real_Baro);
    this->state.gain_Baro2 = gain_Baro2 / (gain_IMU + gain_Baro1 + gain_Baro2 + gain_Baro3 + gain_Real_Baro);
    this->state.gain_Baro3 = gain_Baro3 / (gain_IMU + gain_Baro1 + gain_Baro2 + gain_Baro3 + gain_Real_Baro);
    this->state.gain_Real_Baro = gain_Real_Baro / (gain_IMU + gain_Baro1 + gain_Baro2 + gain_Baro3 + gain_Real_Baro);

    if(debug == Dynamite || debug == ALL){
        SOAR_PRINT("\nRecalculate Gain\n");
        SOAR_PRINT("New Gain IMU: %f\n", this->state.gain_IMU);
        SOAR_PRINT("New Gain Baro1: %f\n", this->state.gain_Baro1);
        SOAR_PRINT("New Gain Baro2: %f\n", this->state.gain_Baro2);
        SOAR_PRINT("New Gain Baro3: %f\n", this->state.gain_Baro3);
        SOAR_PRINT("New Gain Real Baro: %f\n\n", this->state.gain_Real_Baro);
    }
}

/**
 * @brief Converts the STDs to coefficients
 */ 
void EverestTask::calculateSTDCoefficients(){
    // calculate standard deviation coefficients
    double std_IMU = this->state.gain_IMU;
    double std_Baro1 = this->state.gain_Baro1;
    double std_Baro2 = this->state.gain_Baro2;
    double std_Baro3 = this->state.gain_Baro3;
    double std_Real_Baro = this->state.gain_Real_Baro;

    double sumSTD1 = pow(everest.state.gain_IMU, 2) + pow(everest.state.gain_Baro1, 2) + pow(everest.state.gain_Baro2, 2)
                    + pow(everest.state.gain_Baro3, 2) + pow(everest.state.gain_Real_Baro, 2);

    // normalise
    this->state.std_IMU = pow(std_IMU, 2) / sumSTD1;
    this->state.std_Baro1 = pow(std_Baro1, 2) / sumSTD1;
    this->state.std_Baro2 = pow(std_Baro2, 2) / sumSTD1;
    this->state.std_Baro3 = pow(std_Baro3, 2) / sumSTD1;
    this->state.std_Real_Baro = pow(std_Real_Baro, 2) / sumSTD1;

//    if(debug == Dynamite || debug == ALL){
//        printf("\nStandard Deviation Coefficients\n");
//        printf("STD IMU: %f\n", this->state.std_IMU);
//        printf("STD Baro1: %f\n", this->state.std_Baro1);
//        printf("STD Baro2: %f\n", this->state.std_Baro2);
//        printf("STD Baro3: %f\n", this->state.std_Baro3);
//        printf("STD Real Baro: %f\n\n", this->state.std_Real_Baro);
//    }

}

/**
 * @brief Calculates the derivative of the altitude
 * 
 * @param estimate the estimated altitude
 * 
 * @return velocity
 * 
 *   Internal
 */
double EverestTask::deriveChangeInVelocityToGetAltitude(double estimate){

    double deltaTimeAverage = (this->baro1.deltaTime + this->baro2.deltaTime 
                                + this->baro3.deltaTime + this->realBaro.deltaTime + this->state.deltaTimeIMU)/5.0;

    double velocityZ = (this->AltitudeList.secondLastAltitude - 4 * this->AltitudeList.lastAltitude + 3*estimate)/(2.0 * deltaTimeAverage);

    double newAltitude = this->AltitudeList.lastAltitude + velocityZ * deltaTimeAverage;

    if(debug == Dynamite || debug == ALL){
        SOAR_PRINT("\nDerivative for new gain\n");
        SOAR_PRINT("Velocity: %f\n", velocityZ);
        SOAR_PRINT("New Altitude: %f\n", newAltitude);
        SOAR_PRINT("Delta Time Average: %f\n\n", deltaTimeAverage);
    }

    return newAltitude;
}

/**
 * @brief Getter for finalAltitude
 * 
 * @return kinematics struct
 * 
 */
double getFinalAltitude(){
    return Kinematics->finalAltitude;
}

/**
 * @brief Tares the altitude to the ground
 * 
 * @category Internal | Asynchronous 
 * 
 * Call function 10*RefreshRate times to get the initial altitude
 * 
 * Once finished will print the tared altitude and set it as the initial altitude
*/
void EverestTask::tare(SensorDataNoMag &imu1, SensorDataNoMag &imu2, BarosData baro1, BarosData baro2, BarosData baro3, BarosData realBaro){
    // for 10 seconds collect baro
    // decrement the time

    // average pressures   
    // have to do since function is weird
    double average = 0;
    int numberOfSamples = 0;

    if(baro1.pressure != 0){
        average = average + convertToAltitude(baro1.pressure);
        numberOfSamples++;
	if(debug == Secondary || debug == ALL){
        	SOAR_PRINT("average: %f number: %d \n", average, numberOfSamples);
        }
    }

    if(baro2.pressure != 0){
        average = average + convertToAltitude(baro2.pressure);
        numberOfSamples++;
	if(debug == Secondary || debug == ALL){
        	SOAR_PRINT("average: %f number: %d \n", average, numberOfSamples);
        }
    }

    if(baro3.pressure != 0){
        average = average + convertToAltitude(baro3.pressure);
        numberOfSamples++;
	if(debug == Secondary || debug == ALL){
        	SOAR_PRINT("average: %f number: %d \n", average, numberOfSamples);
        }
    }

    if(realBaro.altitude != 0){
        average = average + realBaro.altitude;
        numberOfSamples++;
	if(debug == Secondary || debug == ALL){
        	SOAR_PRINT("average: %f number: %d \n", average, numberOfSamples);
        }
    }

    if(numberOfSamples != 0){
        sum += average/numberOfSamples;
    }

    if(debug == Secondary || debug == ALL){
        SOAR_PRINT("Tare Sum: %f\n", sum);
        SOAR_PRINT("Number of samples %f\n", numberOfSamples);
    }

    if(theTime == 0){
        sum = sum/(CALIBRATION_TIME*RATE_BARO);
        this->Kinematics.initialAlt = sum;
        SOAR_PRINT("Tare Initial Altitude: %f\n", this->Kinematics.initialAlt);
        isTared = true;

        
        // ExternalUpdate(imu1, imu2, baro1, baro2, baro3, realBaro);
    }

    // call to update time for these structs
    IMU_Update(imu1, imu2);

    theTime -= 1;
}

/**
 * @brief accel is in m/s -> gs, gyro is passed in dps, pressure is in Pa, real is for altitude ONLY in m
 *        Aligns before sending to update
*/
double finalWrapper(float accelX1,  float accelY1,      float accelZ1,
    		float gyroX1,   float gyroY1,       float gyroZ1,       float accelX2,
    		float accelY2,  float accelZ2,      float gyroX2,       float gyroY2,
    		float gyroZ2,   float pressure1,    float pressure2, float pressure3, 
		float altitudeReal,	float timeIMU, float timeIMU2, float timeBaro1, 
		float timeBaro2, float timeBaro3, float timeRealBaro,
                MadAxesAlignment alignment, MadAxesAlignment alignment2){

    SensorDataNoMag sensorData = {
        timeIMU1,
        gyroX1,
        gyroY1,
        gyroZ1,
        (float) (accelX1/9.81),
        (float) (accelY1/9.81),
        (float) (accelZ1/9.81),
    };

    SensorDataNoMag sensorData2 = {
        timeIMU2,
        gyroX2,
        gyroY2,
        gyroZ2,
        (float) (accelX2/9.81),
        (float) (accelY2/9.81),
        (float) (accelZ2/9.81),
    };

    BarosData baro1 = {
        timeBaro1,
        pressure1,
        0,
        0
    };

    BarosData baro2 = {
        timeBaro2,
        pressure2,
        0,
        0
    };

    BarosData baro3 = {
        timeBaro3,
        pressure3,
        0,
        0
    };

    BarosData realBaro = {
        timeRealBaro,
        0,
        0,
        altitudeReal
    };

    // align
    madVector imu1Gyro = {sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ};
    madVector imu1Accel = {sensorData.accelX, sensorData.accelY, sensorData.accelZ};

    madVector imu1GyroAligned = infusion->AxesSwitch(imu1Gyro, alignment);
    madVector imu1AccelAligned = infusion->AxesSwitch(imu1Accel, alignment);

    madVector imu2Gyro = {sensorData2.gyroX, sensorData2.gyroY, sensorData2.gyroZ};
    madVector imu2Accel = {sensorData2.accelX, sensorData2.accelY, sensorData2.accelZ};

    madVector imu2GyroAligned = infusion->AxesSwitch(imu2Gyro, alignment2);
    madVector imu2AccelAligned = infusion->AxesSwitch(imu2Accel, alignment2);

//    if(debug == Secondary || debug == ALL){
//        printf("Aligned: Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g\n",
//            imu1GyroAligned.axis.x, imu1GyroAligned.axis.y, imu1GyroAligned.axis.z, imu1AccelAligned.axis.x, imu1AccelAligned.axis.y, imu1AccelAligned.axis.z);
//    }

    // feed vectors into sensorData structs
    sensorData.gyroX = imu1GyroAligned.axis.x;
    sensorData.gyroY = imu1GyroAligned.axis.y;
    sensorData.gyroZ = imu1GyroAligned.axis.z;

    sensorData.accelX = imu1AccelAligned.axis.x;
    sensorData.accelY = imu1AccelAligned.axis.y;
    sensorData.accelZ = imu1AccelAligned.axis.z;

    // second IMU
    sensorData2.gyroX = imu2GyroAligned.axis.x;
    sensorData2.gyroY = imu2GyroAligned.axis.y;
    sensorData2.gyroZ = imu2GyroAligned.axis.z;

    sensorData2.accelX = imu2AccelAligned.axis.x;
    sensorData2.accelY = imu2AccelAligned.axis.y;
    sensorData2.accelZ = imu2AccelAligned.axis.z;

    double eAltitude = everest.ExternalUpdate(sensorData, sensorData2, baro1, baro2, baro3, realBaro);

    SOAR_PRINT("Altitude: %f\n", eAltitude);

    return eAltitude;

}

/**
 * @brief Resets isTared flag to re-initialize the tare
 */
void setIsTare(bool isTare){
    isTared = isTare;
}

// --------------------------------------------------- END OF EVEREST ---------------------------------------------------//





// #define MAX_LINE_LENGTH 1024

/**
 * Serves to just initialize structs 
*/
//int main()
//{
//    // Setup Madgwick
//    // Attach Madgwick to Everest
//    everest.MadgwickSetup();
//
//    // test purposes
//    file = fopen("everest3.txt", "w+"); // Open the file for appending or create it if it doesn't exist
//    if (!file) {
//        fprintf(stderr, "Error opening file...exiting\n");
//        exit(1);
//    }
//
//    FILE *file1 = fopen("C:/Users/Andrey/Documents/EverestRepo/Apogee-Detection-Everest/MadgwickLibrary/IMU_BARO.csv", "r");
//    if (!file1) {
//        perror("Error opening file");
//        return 1;
//    }
//    // read first line and preset the deltaTime to timestamp
//    char line[MAX_LINE_LENGTH];
//    std::clock_t start;
//    double duration;
//
//    int howMany = 1;
//
//    int i = 0;
//
//    while (fgets(line, sizeof(line), file1)) {
//        // Tokenize the line using strtok
//        // Parse accelerometer readings (X, Y, Z)
//        char *token = strtok(line, ",");
//        float accelX = atof(token); // Convert the time value to float
//        token = strtok(NULL, ",");
//        float accelY = atof(token);
//        token = strtok(NULL, ",");
//        float accelZ = atof(token);
//
//        // Parse gyroscope readings (X, Y, Z)
//        token = strtok(NULL, ",");
//        float gyroX = atof(token);
//        token = strtok(NULL, ",");
//        float gyroY = atof(token);
//        token = strtok(NULL, ",");
//        float gyroZ = atof(token);
//
//        // Parse magnetometer readings (X, Y, Z)
//        token = strtok(NULL, ",");
//        float magX = atof(token);
//        token = strtok(NULL, ",");
//        float magY = atof(token);
//        token = strtok(NULL, ",");
//        float magZ = atof(token);
//
//        token = strtok(NULL, ",");
//        float time = atof(token); // Convert the time value to float
//
//        token = strtok(NULL, ",");
//        float pressure = atof(token);
//
//        time = i * DELTA_TIME;
//
//        i++;
//
//        // SensorDataNoMag sensorData = {
//        //     time,
//        //     gyroX/1000,
//        //     gyroY/1000,
//        //     gyroZ/1000,
//        //     accelX/1000,
//        //     accelY/1000,
//        //     accelZ/1000,
//        // };
//
//        // SensorDataNoMag sensorData2 = {
//        //     time,
//        //     gyroX/1000,
//        //     gyroY/1000,
//        //     gyroZ/1000,
//        //     accelX/1000,
//        //     accelY/1000,
//        //     accelZ/1000,
//        // };
//
//        // start = std::clock();
//
//        // BarosData baro1 = {
//        //     time,
//        //     pressure,
//        //     0,
//        //     0
//        // };
//
//        // BarosData baro2 = {
//        //     time,
//        //     pressure,
//        //     0,
//        //     0
//        // };
//
//        // BarosData baro3 = {
//        //     time,
//        //     pressure,
//        //     0,
//        //     0
//        // };
//
//        // BarosData realBaro = {
//        //     time,
//        //     pressure,
//        //     0,
//        //     0
//        // };
//
//        // // if(howMany <= 10){
//
//        printf("\n#%d Sample--------------------------------------------------------------------------\n\n", howMany);
//
//        // // Example: Print all sensor readings
//        // if(debug == RAW || debug == ALL){
//        //     printf("Raw Time: %.6f s, Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g Pressure: (%.f, %.f, %.f, %.f)\n",
//        //         time, sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ, sensorData.accelX, sensorData.accelY, sensorData.accelZ,
//        //         baro1.pressure, baro2.pressure, baro3.pressure, realBaro.pressure);
//        // }
//
//        // madVector imu1Gyro = {sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ};
//        // madVector imu1Accel = {sensorData.accelX, sensorData.accelY, sensorData.accelZ};
//
//        // madVector imu1GyroAligned = infusion->AxesSwitch(imu1Gyro, MadAxesAlignmentPXPYNZ);
//        // madVector imu1AccelAligned = infusion->AxesSwitch(imu1Accel, MadAxesAlignmentPXPYNZ);
//
//        // madVector imu2Gyro = {sensorData2.gyroX, sensorData2.gyroY, sensorData2.gyroZ};
//        // madVector imu2Accel = {sensorData2.accelX, sensorData2.accelY, sensorData2.accelZ};
//
//        // madVector imu2GyroAligned = infusion->AxesSwitch(imu2Gyro, MadAxesAlignmentPXPYNZ);
//        // madVector imu2AccelAligned = infusion->AxesSwitch(imu2Accel, MadAxesAlignmentPXPYNZ);
//
//        // if(debug == Secondary || debug == ALL){
//        //     printf("Aligned: Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g\n",
//        //         imu1GyroAligned.axis.x, imu1GyroAligned.axis.y, imu1GyroAligned.axis.z, imu1AccelAligned.axis.x, imu1AccelAligned.axis.y, imu1AccelAligned.axis.z);
//        // }
//
//        // // feed vectors into sensorData structs
//        // sensorData.gyroX = imu1GyroAligned.axis.x;
//        // sensorData.gyroY = imu1GyroAligned.axis.y;
//        // sensorData.gyroZ = imu1GyroAligned.axis.z;
//
//        // sensorData.accelX = imu1AccelAligned.axis.x;
//        // sensorData.accelY = imu1AccelAligned.axis.y;
//        // sensorData.accelZ = imu1AccelAligned.axis.z;
//
//        // // second IMU
//        // sensorData2.gyroX = imu2GyroAligned.axis.x;
//        // sensorData2.gyroY = imu2GyroAligned.axis.y;
//        // sensorData2.gyroZ = imu2GyroAligned.axis.z;
//
//        // sensorData2.accelX = imu2AccelAligned.axis.x;
//        // sensorData2.accelY = imu2AccelAligned.axis.y;
//        // sensorData2.accelZ = imu2AccelAligned.axis.z;
//
//        // everest.IMU_Update(sensorData, sensorData2);
//
//            // double eAltitude = everest.AlignedExternalUpdate(sensorData, sensorData2, baro1, baro2, baro3, realBaro, MadAxesAlignmentPXPYNZ);
//        // double eAltitude = everest.ExternalUpdate(sensorData, sensorData2, baro1, baro2, baro3, realBaro);
//        double eAltitude = finalWrapper(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, pressure, pressure, pressure, pressure, time, time, time, time, time, time, MadAxesAlignmentPXPYNZ, MadAxesAlignmentPXPYNZ);
//
//        printf("Altitude: %f\n", eAltitude);
//
//
//        // }
//
//        howMany++;
//
//        clock_t endTime = std::clock();
//
//        duration += endTime - start;
//
//        // printf("Time for one more (seconds): %f\n", duration/CLOCKS_PER_SEC);
//    }
//
//    // printf("Overall for (13k samples): %f", duration/CLOCKS_PER_SEC);
//
//    fclose(file1);
//    fclose(file);
//
//    return 0;
//}




