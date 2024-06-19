// Altitude estimation using multiple sensors
#include "everest.hpp"
#include <stdio.h>
#include <ctime>
#include <string.h>

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

bool firstSampleAfterCalibration = true;

bool isTared = false;

// double timeInSeconds = 2;
double theTime = CALIBRATION_TIME * RATE_BARO;
double sum = 0;
double pressureSum = 0;

static float previousTimestamp = 0;

FILE *file;

enum debug_level{
    RAW = 0,        // raw data
    Secondary = 1,  // all operations before dynamite
    Dynamite = 2,   //everything during dynamite
    Third = 3,      // after dynamite
    ALL = 4,        // all
    NONE = 5        // none
};

debug_level debug = NONE;

// Instantiate Everest
madAhrs *ahrs;
Infusion *infusion;

Everest everest = Everest::getEverest();
kinematics *Kinematics = everest.getKinematics(); // tare to ground

madAhrsFlags flags;
madAhrsInternalStates internalStates;


//----------------------------------Task Integration----------------------------------//



//----------------------------------EVEREST-------------------------------------------//
/**
 * @brief Only done once. Sets pointers for Madgwick
 *     Internal
*/
void Everest::MadgwickSetup()
{
    // Attaches Madgwick to Everest
    infusion = everest.ExternalInitialize();
    ahrs = infusion->getMadAhrs();

    calculateSTDCoefficients();

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
            EarthConventionNed,
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
void Everest::MadgwickWrapper(SensorDataNoMag data){
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
        printf("Averaged: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g Time: %f\n",
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
    fprintf(file, "%f,", timestamp);

    fprintf(file, "%f,%f,%f,", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

    fprintf(file, "%f,%d,%.0f,%.0f,%d,%.0f,%d,%d,%d,%d,%f", internalStates.accelerationError,  
    internalStates.accelerometerIgnored, internalStates.accelerationRecoveryTrigger, internalStates.magneticError, 
    internalStates.magnetometerIgnored, internalStates.magneticRecoveryTrigger, flags.initialising, 
    flags.angularRateRecovery, flags.accelerationRecovery, flags.magneticRecovery, earth.axis.z);

    everest.state.earthAcceleration = earth.axis.z;

    // fprintf(file, "\n");

    if(debug == Secondary || debug == ALL){
        printf("%f,%d,%.0f,%.0f,%d,%.0f,%d,%d,%d,%d\n", internalStates.accelerationError, 
        internalStates.accelerometerIgnored, internalStates.accelerationRecoveryTrigger, 
        internalStates.magneticError, internalStates.magnetometerIgnored, internalStates.magneticRecoveryTrigger, 
        flags.initialising, flags.angularRateRecovery, flags.accelerationRecovery, flags.magneticRecovery); 
    }

}

/**
 * @brief Averages IMUs and feeds them to Madgwick wrapper
 *      Should be called every time IMU data is updated
 * 
 *    Internal
*/
void Everest::IMU_Update(const SensorDataNoMag& imu1, const SensorDataNoMag& imu2)
{
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

    // Calculate average of IMU parameters
    #define averageIMU this->state.avgIMU

    averageIMU.gyroX = (this->internalIMU_1.gyroX + this->internalIMU_2.gyroX) / 2.0;
    averageIMU.gyroY = (this->internalIMU_1.gyroY + this->internalIMU_2.gyroY) / 2.0;
    averageIMU.gyroZ = (this->internalIMU_1.gyroZ + this->internalIMU_2.gyroZ) / 2.0;

    averageIMU.accelX = (this->internalIMU_1.accelX + this->internalIMU_2.accelX) / 2.0;
    averageIMU.accelY = (this->internalIMU_1.accelY + this->internalIMU_2.accelY) / 2.0;
    averageIMU.accelZ = (this->internalIMU_1.accelZ + this->internalIMU_2.accelZ) / 2.0;

    averageIMU.time = (this->internalIMU_1.time + this->internalIMU_2.time) / 2.0;

    #undef averageIMU

    // feed to Madgwick
    // MadgwickWrapper(state.avgIMU);
    this->MadgwickWrapper(state.avgIMU);

}

/**
 * @brief updates baro and delta time
 *     Should be called every time baro data is updated
 *      Internal
*/
void Everest::Baro_Update(const BarosData& Baro1, const BarosData& Baro2, const BarosData& Baro3, const BarosData& RealBaro)
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
    this->realBaro.deltaTime = RealBaro.time - this->realBaro.previousTime;
    this->realBaro.previousTime = RealBaro.time;

    if(debug == RAW || debug == ALL){
        printf("Baro1: %.f Pa, Baro2: %.f Pa, Baro3: %.f Pa, RealBaro: %.f Pa\n",
            baro1.pressure, baro2.pressure, baro3.pressure, realBaro.pressure);
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
double Everest::ExternalUpdate(SensorDataNoMag imu1, SensorDataNoMag imu2, BarosData baro1, BarosData baro2, BarosData baro3, BarosData realBaro){
    if(!isTared){
        everest.tare(imu1, imu2, baro1, baro2, baro3, realBaro);
        printf("Taring in progress\n)");
        return 0;
    }

    everest.IMU_Update(imu1, imu2);

    if(debug == Third || debug == ALL){
        printf("After IMU Update IMU Altitude: %f\n", everest.state.avgIMU.altitude);
    }

    everest.Baro_Update(baro1, baro2, baro3, realBaro);

    double finalAlt = everest.dynamite();

    if(debug == Dynamite || debug == ALL){
        printf("After Dynamite: %f\n", finalAlt);
    }

    // Update altitude list
    this->AltitudeList.secondLastAltitude = this->AltitudeList.lastAltitude;
    this->AltitudeList.lastAltitude = finalAlt;

    fprintf(file, ",%f\n", finalAlt); // write to file

    return finalAlt;
}

/**
 * @brief Wraps External Update with alignment, returns External Update with aligned data
*/
double Everest::AlignedExternalUpdate(SensorDataNoMag imu1, SensorDataNoMag imu2, 
            BarosData baro1, BarosData baro2, BarosData baro3, BarosData realBaro, MadAxesAlignment alignment){
    // align
    madVector alignedIMU1 = infusion->AxesSwitch({imu1.accelX, imu1.accelY, imu1.accelZ}, alignment);
    madVector alignedIMUGyro1 = infusion->AxesSwitch({imu1.gyroX, imu1.gyroY, imu1.gyroZ}, alignment);

    madVector alignedIMU2 = infusion->AxesSwitch({imu2.accelX, imu2.accelY, imu2.accelZ}, alignment);
    madVector alignedIMUGyro2 = infusion->AxesSwitch({imu2.gyroX, imu2.gyroY, imu2.gyroZ}, alignment);

    if(debug == Secondary || debug == ALL){
        printf("Unaligned IMU1: (%.6f, %.6f, %.6f) g, (%.6f, %.6f, %.6f) deg/s\n",
            imu1.accelX, imu1.accelY, imu1.accelZ, imu1.gyroX, imu1.gyroY, imu1.gyroZ);

        printf("Unaligned IMU2: (%.6f, %.6f, %.6f) g, (%.6f, %.6f, %.6f) deg/s\n", 
            imu2.accelX, imu2.accelY, imu2.accelZ, imu2.gyroX, imu2.gyroY, imu2.gyroZ);

        printf("Alignment: %d\n", alignment);
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
        printf("Aligned IMU1: (%.6f, %.6f, %.6f) g, (%.6f, %.6f, %.6f) deg/s\n",
            imu1.accelX, imu1.accelY, imu1.accelZ, imu1.gyroX, imu1.gyroY, imu1.gyroZ);

        printf("Aligned IMU2: (%.6f, %.6f, %.6f) g, (%.6f, %.6f, %.6f) deg/s\n",   
            imu2.accelX, imu2.accelY, imu2.accelZ, imu2.gyroX, imu2.gyroY, imu2.gyroZ);
    }

    return ExternalUpdate(imu1, imu2, baro1, baro2, baro3, realBaro);
}


/**
 * Calculates altitude using IMU sensor data and kinematic equations.
 * 
 * @param avgIMU with the average sensor data from the IMU
 * 
 *  Internal
 * 
 * @return calculated altitude
 */

// TO DO - fix the negative and accelX to accelZ
double Everest::deriveForAltitudeIMU(SensorDataNoMag avgIMU){
    // double accelerationZ = avgIMU.accelX * -9.81;
    double accelerationZ = everest.state.earthAcceleration * -9.81;
    double initialVelocity = this->getKinematics()->initialVelo;
    double initialAltitude = this->Kinematics.initialAlt;
    double deltaTime = this->state.deltaTimeIMU;

    // Derive altitude from IMU
    double finalVelocity = initialVelocity + accelerationZ * deltaTime;
    double altitude = initialAltitude + (initialVelocity + finalVelocity) * deltaTime / 2.0;

    if(debug == Secondary || debug == ALL){
        printf("\nKinematics\n");
        printf("IMU Initial Altitude: %f\n", initialAltitude);
        printf("IMU Velocity: %f\n", initialVelocity);
        printf("IMU Acceleration: %f\n", accelerationZ);
        printf("IMU Delta Time: %f\n", deltaTime);
        printf("Derived Altitude: %f\n", altitude);
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
        printf("\nConversion \n");
        printf("Pressure: %.f hPa, Altitude: %.f m\n", pressure, altitude);
    }

    return altitude;
}

/**
 * @brief Multi-system trust algorithm. Assumes measurements are updated
 * @returns normalised altitude
 * 
 *   Internal
*/
double Everest::dynamite(){
    double IMUAltitude = deriveForAltitudeIMU(everest.state.avgIMU);
    this->state.avgIMU.altitude = IMUAltitude;

    double BaroAltitude1 = convertToAltitude(this->baro1.pressure);
    this->baro1.altitude = BaroAltitude1;

    double BaroAltitude2 = convertToAltitude(everest.baro2.pressure);
    this->baro2.altitude = BaroAltitude2;

    double BaroAltitude3 = convertToAltitude(everest.baro3.pressure);
    this->baro3.altitude = BaroAltitude3;

    double RealBaroAltitude = convertToAltitude(everest.realBaro.pressure);
    this->realBaro.altitude = RealBaroAltitude;

    if(debug == Dynamite || debug == ALL){
        printf("\nDynamite\n");
        printf("Baro1 Altitude: %f\n", BaroAltitude1);
        printf("Baro2 Altitude: %f\n", BaroAltitude2);
        printf("Baro3 Altitude: %f\n", BaroAltitude3);
        printf("Real Baro Altitude: %f\n", RealBaroAltitude);
        printf("IMU Altitude: %f\n", IMUAltitude);
    }

    // // distributing measurement
    // double distributed_IMU_Altitude = (IMUAltitude * everest.state.gain_IMU)/pow(everest.state.std_IMU, 2);
    // double distributed_Baro_Altitude1 = (BaroAltitude1 * everest.state.gain_Baro1)/pow(everest.state.std_Baro1, 2);
    // double distributed_Baro_Altitude2 = (BaroAltitude2 * everest.state.gain_Baro2)/pow(everest.state.std_Baro2, 2);
    // double distributed_Baro_Altitude3 = (BaroAltitude3 * everest.state.gain_Baro3)/pow(everest.state.std_Baro3,2);
    // double distributed_RealBaro_Altitude = (RealBaroAltitude * everest.state.gain_Real_Baro)/pow(everest.state.std_Real_Baro,2);
    
    // if pressure is zero, set gain to zero
    if(everest.realBaro.pressure == 0){
        everest.state.gain_Real_Baro = 0;
    }else if (everest.state.gain_Real_Baro == 0){
        // if not zero, set gain to previous gain
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
        printf("\nDistributed\n");
        printf("Distributed IMU Altitude: %f\n", distributed_IMU_Altitude);
        printf("Gain IMU: %f\n", everest.state.gain_IMU);
        printf("STD IMU: %f\n\n", everest.state.std_IMU);

        printf("Distributed Baro1 Altitude: %f\n", distributed_Baro_Altitude1);
        printf("Gain Baro1: %f\n", everest.state.gain_Baro1);
        printf("STD Baro1: %f\n\n", everest.state.std_Baro1);

        printf("Distributed Baro2 Altitude: %f\n", distributed_Baro_Altitude2);
        printf("Gain Baro2: %f\n", everest.state.gain_Baro2);
        printf("STD Baro2: %f\n\n", everest.state.std_Baro2);

        printf("Distributed Baro3 Altitude: %f\n", distributed_Baro_Altitude3);
        printf("Gain Baro3: %f\n", everest.state.gain_Baro3);
        printf("STD Baro3: %f\n\n", everest.state.std_Baro3);

        printf("Distributed Real Baro Altitude: %f\n", distributed_RealBaro_Altitude);
        printf("Gain Real Baro: %f\n", everest.state.gain_Real_Baro);
        printf("STD Real Baro: %f\n\n", everest.state.std_Real_Baro);
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
        printf("Distributed Sum: %f\n\n", distributed_Sum);
    }

    // summation of standard deviations
    // double sumSTD1 = pow(everest.state.std_IMU + everest.state.std_Baro1 + everest.state.std_Baro2
    //                 + everest.state.std_Baro3 + everest.state.std_Real_Baro, 2);
    // double sumSTD1 = pow(everest.state.gain_IMU, 2) + pow(everest.state.gain_Baro1, 2) + pow(everest.state.gain_Baro2, 2)
    //                 + pow(everest.state.gain_Baro3, 2) + pow(everest.state.gain_Real_Baro, 2);

    if(debug == Dynamite || debug == ALL){
        // printf("Sum STD: %f\n\n", sumSTD1);
    }

    // summation of gains
    double sumGain = everest.state.gain_IMU + everest.state.gain_Baro1 + everest.state.gain_Baro2 
                    + everest.state.gain_Baro3 + everest.state.gain_Real_Baro;

    if(debug == Dynamite || debug == ALL){
        printf("Sum Gain: %f\n\n", sumGain);
    }

    // normalised altitude
    double normalised_Altitude = (distributed_Sum)/sumGain;
    // normalised_Altitude = normalised_Altitude / sumSTD1;

    // double normalised_Altitude = (distributed_Sum*sumSTD)/(everest.state.gain_IMU);

    if(debug == Dynamite || debug == ALL){
        printf("Normalised Altitude: %f\n\n", normalised_Altitude);
    }

    // Update Kinematics
    Kinematics.finalAltitude = normalised_Altitude;

    if(debug == Dynamite || debug == ALL){
        printf("Final Altitude: %f\n\n", Kinematics.finalAltitude);
    }

    // update velocity
    Kinematics.initialVelo = (Kinematics.finalAltitude - Kinematics.initialAlt)/(this->state.deltaTimeIMU);

    if(debug == Dynamite || debug == ALL){
        printf("Initial Velocity: %f\n", Kinematics.initialVelo);
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
        printf("Previous Gains\n");
        printf("Prev Gain IMU: %f\n", everest.state.prev_gain_IMU);
        printf("Prev Gain Baro1: %f\n", everest.state.prev_gain_Baro1);
        printf("Prev Gain Baro2: %f\n", everest.state.prev_gain_Baro2);
        printf("Prev Gain Baro3: %f\n", everest.state.prev_gain_Baro3);
        printf("Prev Gain Real Baro: %f\n\n", everest.state.prev_gain_Real_Baro);
    }

    return normalised_Altitude;

}

// @brief calculation - new gain = 1 / abs(estimate - measurement)
void Everest::recalculateGain(double estimate){
    double gainedEstimate = deriveChangeInVelocityToGetAltitude(estimate); // pre integrated for altitude

    double gain_IMU = 1/fabsf(gainedEstimate-this->state.avgIMU.altitude); // change to previous trusts
    double gain_Baro1 = 1/fabsf(gainedEstimate-this->baro1.altitude);
    double gain_Baro2 = 1/fabsf(gainedEstimate-this->baro2.altitude);
    double gain_Baro3 = 1/fabsf(gainedEstimate-this->baro3.altitude);
    double gain_Real_Baro = 1/fabsf(gainedEstimate-this->realBaro.altitude);

    if(debug == Third || debug == ALL){
        printf("\nRecalculate Gain - Before normalization\n");
        printf("Gain IMU: %f\n", gain_IMU);
        printf("Gain Baro1: %f\n", gain_Baro1);
        printf("Gain Baro2: %f\n", gain_Baro2);
        printf("Gain Baro3: %f\n", gain_Baro3);
        printf("Gain Real Baro: %f\n", gain_Real_Baro);
        printf("Gained Estimate: %f\n", gainedEstimate);

        printf("Altitude: %f\n", estimate);
        printf("Baro1: %f\n", this->baro1.altitude);
        printf("Baro2: %f\n", this->baro2.altitude);
        printf("Baro3: %f\n", this->baro3.altitude);
        printf("Real Baro: %f\n\n", this->realBaro.altitude);
    }

    // normalise
    this->state.gain_IMU = gain_IMU / (gain_IMU + gain_Baro1 + gain_Baro2 + gain_Baro3 + gain_Real_Baro);
    this->state.gain_Baro1 = gain_Baro1 / (gain_IMU + gain_Baro1 + gain_Baro2 + gain_Baro3 + gain_Real_Baro);
    this->state.gain_Baro2 = gain_Baro2 / (gain_IMU + gain_Baro1 + gain_Baro2 + gain_Baro3 + gain_Real_Baro);
    this->state.gain_Baro3 = gain_Baro3 / (gain_IMU + gain_Baro1 + gain_Baro2 + gain_Baro3 + gain_Real_Baro);
    this->state.gain_Real_Baro = gain_Real_Baro / (gain_IMU + gain_Baro1 + gain_Baro2 + gain_Baro3 + gain_Real_Baro);

    if(debug == Dynamite || debug == ALL){
        printf("\nRecalculate Gain\n");
        printf("New Gain IMU: %f\n", this->state.gain_IMU);
        printf("New Gain Baro1: %f\n", this->state.gain_Baro1);
        printf("New Gain Baro2: %f\n", this->state.gain_Baro2);
        printf("New Gain Baro3: %f\n", this->state.gain_Baro3);
        printf("New Gain Real Baro: %f\n\n", this->state.gain_Real_Baro);
    }
}

/**
 * @brief Converts the STDs to coefficients
 */ 
void Everest::calculateSTDCoefficients(){
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

    if(debug == Dynamite || debug == ALL){
        printf("\nStandard Deviation Coefficients\n");
        printf("STD IMU: %f\n", this->state.std_IMU);
        printf("STD Baro1: %f\n", this->state.std_Baro1);
        printf("STD Baro2: %f\n", this->state.std_Baro2);
        printf("STD Baro3: %f\n", this->state.std_Baro3);
        printf("STD Real Baro: %f\n\n", this->state.std_Real_Baro);
    }

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
double Everest::deriveChangeInVelocityToGetAltitude(double estimate){
    double deltaTimeAverage = (this->baro1.deltaTime + this->baro2.deltaTime 
                                + this->baro3.deltaTime + this->realBaro.deltaTime + this->state.deltaTimeIMU)/5.0;

    double velocityZ = (this->AltitudeList.secondLastAltitude - 4 * this->AltitudeList.lastAltitude + 3*estimate)/(2.0 * deltaTimeAverage);

    double newAltitude = this->AltitudeList.lastAltitude + velocityZ * deltaTimeAverage;

    if(debug == Dynamite || debug == ALL){
        printf("\nDerivative for new gain\n");
        printf("Velocity: %f\n", velocityZ);
        printf("New Altitude: %f\n", newAltitude);
        printf("Delta Time Average: %f\n\n", deltaTimeAverage);
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
 * Call function 10*RefreshRate times to get the initial altitude
 * 
 * Once finished will print the tared altitude and set it as the initial altitude
*/
void Everest::tare(SensorDataNoMag &imu1, SensorDataNoMag &imu2, BarosData baro1, BarosData baro2, BarosData baro3, BarosData realBaro){
    // average pressures   
    double average = 0;
    int numberOfSamples = 0;

    if(baro1.pressure != 0){
        average = convertToAltitude(baro1.pressure);
        numberOfSamples++;
    }

    if(baro2.pressure != 0){
        average = average + convertToAltitude(baro2.pressure);
        numberOfSamples++;
    }

    if(baro3.pressure != 0){
        average = average + convertToAltitude(baro3.pressure);
        numberOfSamples++;
    }

    if(realBaro.altitude != 0){
        average = average + realBaro.altitude;
        numberOfSamples++;
    }

    sum += average/numberOfSamples;

    if(debug == Secondary || debug == ALL){
        printf("Sum: %f\n", sum);
    }

    if(theTime == 0){
        sum = sum/(CALIBRATION_TIME*RATE_BARO);
        this->Kinematics.initialAlt = sum;
        printf("Tare: %f\n", this->Kinematics.initialAlt);
        isTared = true;

        // call to update time for these structs
        // ExternalUpdate(imu1, imu2, baro1, baro2, baro3, realBaro);
        // everest.IMU_Update(imu1, imu2);

        // everest.Baro_Update(baro1, baro2, baro3, realBaro);

        // double finalAlt = everest.dynamite();
    }

    everest.IMU_Update(imu1, imu2);

    theTime -= 1;
}

/**
 * @brief accel is in milli-gs, gyro is in milli-dps, pressure is in Pa
 *        Aligns before sending to update
*/
double finalWrapper( float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ, 
                    float pressure1, float pressure2, float pressure3, float pressureReal,
                    float timeIMU, float timeIMU2, float timeBaro1, float timeBaro2, float timeBaro3, float timeRealBaro,
                    MadAxesAlignment alignment, MadAxesAlignment alignment2){

    SensorDataNoMag sensorData = {
        timeIMU,
        gyroX/1000,
        gyroY/1000,
        gyroZ/1000,
        accelX/1000,
        accelY/1000,
        accelZ/1000,
    };

    SensorDataNoMag sensorData2 = {
        timeIMU2,
        gyroX/1000,
        gyroY/1000,
        gyroZ/1000,
        accelX/1000,
        accelY/1000,
        accelZ/1000,
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
        pressureReal,
        0,
        0
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

    if(debug == Secondary || debug == ALL){
        printf("Aligned: Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g\n",
            imu1GyroAligned.axis.x, imu1GyroAligned.axis.y, imu1GyroAligned.axis.z, imu1AccelAligned.axis.x, imu1AccelAligned.axis.y, imu1AccelAligned.axis.z);
    }

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

    printf("Altitude: %f\n", eAltitude);

    return eAltitude;

}

#define MAX_LINE_LENGTH 1024

/**
 * Serves to just initialize structs 
*/
int main()
{
    // Setup Madgwick
    // Attach Madgwick to Everest
    everest.MadgwickSetup();

    // test purposes
    file = fopen("everest3.txt", "w+"); // Open the file for appending or create it if it doesn't exist
    if (!file) {
        fprintf(stderr, "Error opening file...exiting\n");
        exit(1);
    }

    // FILE *file1 = fopen("C:/Users/Andrey/Documents/EverestRepo/Apogee-Detection-Everest/EverestLibrary/IMU_BARO1.txt", "r");
    FILE *file1 = fopen("C:/Users/Andrey/Downloads/theSims2.csv", "r");
    if (!file1) {
        perror("Error opening file");
        return 1;
    }
    // read first line and preset the deltaTime to timestamp 
    char line[MAX_LINE_LENGTH];
    std::clock_t start;
    double duration;

    int howMany = 1;

    int i = 0;
    
    while (fgets(line, sizeof(line), file1)) {
        // Tokenize the line using strtok
        // Parse accelerometer readings (X, Y, Z)
        char *token = strtok(line, ",");
        float time = atof(token); // Convert the time value to float

        token = strtok(NULL, ",");
        float accelX = atof(token); // Convert the time value to float

        token = strtok(NULL, ",");
        float accelY = atof(token);
        token = strtok(NULL, ",");
        float accelZ = atof(token);

        // Parse gyroscope readings (X, Y, Z)
        token = strtok(NULL, ",");
        float gyroX = atof(token);
        token = strtok(NULL, ",");
        float gyroY = atof(token);
        token = strtok(NULL, ",");
        float gyroZ = atof(token);

        // Parse magnetometer readings (X, Y, Z)
        token = strtok(NULL, ",");
        float magX = atof(token);
        token = strtok(NULL, ",");
        float magY = atof(token);
        token = strtok(NULL, ",");
        float magZ = atof(token);

        // token = strtok(NULL, ",");
        // float time = atof(token); // Convert the time value to float

        token = strtok(NULL, ",");
        float pressure = atof(token);

        // time = i * DELTA_TIME;

        i++;

        pressure = 0;
        magX = magY = magZ = 0;

        SensorDataNoMag sensorData = {
            time,
            gyroX/1000,
            gyroY/1000,
            gyroZ/1000,
            accelX/1000,
            accelY/1000,
            accelZ/1000,
        };

        SensorDataNoMag sensorData2 = {
            time,
            gyroX/1000,
            gyroY/1000,
            gyroZ/1000,
            accelX/1000,
            accelY/1000,
            accelZ/1000,
        };

        start = std::clock();

        BarosData baro1 = {
            time,
            pressure,
            0,
            0
        };

        BarosData baro2 = {
            time,
            pressure,
            0,
            0
        };

        BarosData baro3 = {
            time,
            pressure,
            0,
            0
        };

        BarosData realBaro = {
            time,
            pressure,
            0,
            0
        };

        // if(howMany <= 10){

        printf("\n#%d Sample--------------------------------------------------------------------------\n\n", howMany);

        // Example: Print all sensor readings
        if(debug == RAW || debug == ALL){
            printf("Raw Time: %.6f s, Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g Pressure: (%.f, %.f, %.f, %.f)\n",
                time, sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ, sensorData.accelX, sensorData.accelY, sensorData.accelZ, 
                baro1.pressure, baro2.pressure, baro3.pressure, realBaro.pressure);
        }

        madVector imu1Gyro = {sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ};
        madVector imu1Accel = {sensorData.accelX, sensorData.accelY, sensorData.accelZ};

        madVector imu1GyroAligned = infusion->AxesSwitch(imu1Gyro, MadAxesAlignmentPXPYNZ);
        madVector imu1AccelAligned = infusion->AxesSwitch(imu1Accel, MadAxesAlignmentPXPYNZ);

        madVector imu2Gyro = {sensorData2.gyroX, sensorData2.gyroY, sensorData2.gyroZ};
        madVector imu2Accel = {sensorData2.accelX, sensorData2.accelY, sensorData2.accelZ};

        madVector imu2GyroAligned = infusion->AxesSwitch(imu2Gyro, MadAxesAlignmentPXPYNZ);
        madVector imu2AccelAligned = infusion->AxesSwitch(imu2Accel, MadAxesAlignmentPXPYNZ);

        if(debug == Secondary || debug == ALL){
            printf("Aligned: Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g\n",
                imu1GyroAligned.axis.x, imu1GyroAligned.axis.y, imu1GyroAligned.axis.z, imu1AccelAligned.axis.x, imu1AccelAligned.axis.y, imu1AccelAligned.axis.z);
        }

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

        // everest.IMU_Update(sensorData, sensorData2);
            
        double eAltitude = everest.AlignedExternalUpdate(sensorData, sensorData2, baro1, baro2, baro3, realBaro, MadAxesAlignmentPXPYNZ);
        // double eAltitude = everest.ExternalUpdate(sensorData, sensorData2, baro1, baro2, baro3, realBaro);
        // double eAltitude = finalWrapper(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, pressure, pressure, pressure, pressure, time, time, time, time, time, time, MadAxesAlignmentPXPYNZ, MadAxesAlignmentPXPYNZ);

        printf("Altitude: %f\n", eAltitude);


        howMany++;

        // }


        clock_t endTime = std::clock();

        duration += endTime - start;

        // printf("Time for one more (seconds): %f\n", duration/CLOCKS_PER_SEC);
    }

    // printf("Overall for (13k samples): %f", duration/CLOCKS_PER_SEC);

    fclose(file1);
    fclose(file);

    return 0;
}




