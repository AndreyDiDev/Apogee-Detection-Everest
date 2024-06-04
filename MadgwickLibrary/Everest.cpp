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

#define SAMPLE_RATE (3) // replace this with actual sample rate
#define DELTA_TIME (1.0f / 3.0f)
FILE *file;

enum debug_level{
    RAW = 0,        // raw data
    Secondary = 1,  // all operations before dynamite
    Dynamite = 2,   //everything during dynamite
    Third = 3,      // after dynamite
    ALL = 4         // all
};

debug_level debug = ALL;

// Instantiate Everest
madAhrs *ahrs;
Infusion *infusion;

// madAhrs *ahrs2;
// Infusion *infusion2;

Everest everest = Everest::getEverest();
kinematics *Kinematics = everest.getKinematics(); // tare to ground

madAhrsFlags flags;
madAhrsInternalStates internalStates;

/**
 * @brief Only done once. Sets pointers for Madgwick
 *     Internal
*/
void MadgwickSetup()
{
    // Attaches Madgwick to Everest
    infusion = everest.Initialize();
    ahrs = infusion->getMadAhrs();

    // infusion2 = everest.Initialize2();
    // ahrs2 = infusion2->getMadAhrs();

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

    // madAhrsInternalStates internal2 = infusion2->madAhrsGetInternalStates(ahrs2);
    // madAhrsFlags flags2 = infusion2->madAhrsGetFlags(ahrs2);

    // Initialise algorithms
    madOffset offset = infusion->getOffset();
    // madOffset offset2 = infusion2->getOffset();
    // madOffset offset2 = infusion->getOffset();

    // *ahrs = infusion.getMadAhrs();

    infusion->madOffsetInitialise(&offset, SAMPLE_RATE);
    infusion->madAhrsInitialise(ahrs);

    // infusion2->madOffsetInitialise(&offset2, SAMPLE_RATE);
    // infusion2->madAhrsInitialise(ahrs2);

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
    // infusion2->madAhrsSetSettings(ahrs2, &settings);

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

    if(debug == Secondary || debug == ALL){
        printf("Averaged: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g\n",
            data.gyroX, data.gyroY, data.gyroZ, data.accelX, data.accelY, data.accelZ);
    }

    // printf("Roll %0.3f, Pitch %0.3f, Yaw %0.3f, X %0.3f, Y %0.3f, Z %0.3f\n",
    //        euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
    //        earth.axis.x, earth.axis.y, earth.axis.z);

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    static float previousTimestamp;
    float deltaTime = (float) (timestamp - previousTimestamp);
    previousTimestamp = timestamp;

    this->state.deltaTimeIMU = deltaTime;

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

    // euler = infusion->getEuler(ahrs);

    internalStates = infusion->madAhrsGetInternalStates(infusion->getMadAhrs());
    flags = infusion->madAhrsGetFlags(infusion->getMadAhrs());

    // write to file
    fprintf(file, "%f,", timestamp);

    fprintf(file, "%f,%f,%f,", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

    fprintf(file, "%f,%d,%.0f,%.0f,%d,%.0f,%d,%d,%d,%d", internalStates.accelerationError,  
    internalStates.accelerometerIgnored, internalStates.accelerationRecoveryTrigger, internalStates.magneticError, 
    internalStates.magnetometerIgnored, internalStates.magneticRecoveryTrigger, flags.initialising, 
    flags.angularRateRecovery, flags.accelerationRecovery, flags.magneticRecovery);

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
        printf("Baro1: %.6f hPa, Baro2: %.6f hPa, Baro3: %.6f hPa, RealBaro: %.6f hPa\n",
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
 * Calculates altitude using IMU sensor data and kinematic equations.
 * 
 * @param avgIMU with the average sensor data from the IMU
 * 
 *  Internal
 * 
 * @return calculated altitude
 */
double Everest::deriveForAltitudeIMU(SensorDataNoMag avgIMU){
    double accelerationZ = avgIMU.accelZ;
    double initialVelocity = this->getKinematics()->initialVelo;
    double initialAltitude = this->Kinematics.initialAlt;
    double deltaTime = this->state.deltaTimeIMU;

    // Derive altitude from IMU
    double altitude = (double) (initialAltitude + initialVelocity * (deltaTime) + 0.5 * accelerationZ * pow((deltaTime), 2));

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
 * @param pressure pressure from baros in hPa
 * 
 * @return altitude in meters
 */
double convertToAltitude(double pressure){
    // Convert pressure to altitude
    // Convert from 100*millibars to m
    double seaLevelPressure = 1013.25 * 1000; // sea level pressure in hPa
    double altitude = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.190284));

    if(debug == Dynamite || debug == ALL){
        printf("\n Conversion \n");
        printf("Pressure: %f hPa, Altitude: %f m\n", pressure, altitude);
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

    double BaroAltitude1 = convertToAltitude(everest.baro1.pressure);
    this->baro1.altitude = BaroAltitude1;

    double BaroAltitude2 = convertToAltitude(everest.baro2.pressure);
    this->baro2.altitude = BaroAltitude2;

    double BaroAltitude3 = convertToAltitude(everest.baro3.pressure);
    this->baro3.altitude = BaroAltitude3;

    double RealBaroAltitude = convertToAltitude(everest.realBaro.pressure);
    this->realBaro.altitude = RealBaroAltitude;

    if(debug == Dynamite || debug == ALL){
        printf("\n Dynamite\n");
        printf("Baro1 Altitude: %f\n", BaroAltitude1);
        printf("Baro2 Altitude: %f\n", BaroAltitude2);
        printf("Baro3 Altitude: %f\n", BaroAltitude3);
        printf("Real Baro Altitude: %f\n", RealBaroAltitude);
        printf("IMU Altitude: %f\n", IMUAltitude);
    }

    // distributing measurement
    double distributed_IMU_Altitude = (IMUAltitude * everest.state.gain_IMU)/everest.state.std_IMU;
    double distributed_Baro_Altitude1 = (BaroAltitude1 * everest.state.gain_Baro1)/everest.state.std_Baro1;
    double distributed_Baro_Altitude2 = (BaroAltitude2 * everest.state.gain_Baro2)/everest.state.std_Baro2;
    double distributed_Baro_Altitude3 = (BaroAltitude3 * everest.state.gain_Baro3)/everest.state.std_Baro3;
    double distributed_RealBaro_Altitude = (RealBaroAltitude * everest.state.gain_Real_Baro)/everest.state.std_Real_Baro;

    double distributed_Sum = distributed_IMU_Altitude + distributed_Baro_Altitude1 + distributed_Baro_Altitude2 
                            + distributed_Baro_Altitude3 + distributed_RealBaro_Altitude;

    double sumSTD = everest.state.std_IMU + everest.state.std_Baro1 + everest.state.std_Baro2 
                    + everest.state.std_Baro3 + everest.state.std_Real_Baro;

    double sumGain = everest.state.gain_IMU + everest.state.gain_Baro1 + everest.state.gain_Baro2 
                    + everest.state.gain_Baro3 + everest.state.gain_Real_Baro;

    double normalised_Altitude = (distributed_Sum/sumSTD)/sumGain;

    // Update Kinematics
    Kinematics.finalAltitude = normalised_Altitude;

    // update velocity
    Kinematics.initialVelo = (Kinematics.finalAltitude - Kinematics.initialAlt)/(this->state.deltaTimeIMU);

    // update altitude
    Kinematics.initialAlt = Kinematics.finalAltitude;

    recalculateGain(normalised_Altitude);

    return normalised_Altitude;

}

// new gain = 1 / abs(estimate - measurement)
// TO DO : put the derivative of the altitude in the recalculateGain function
// do first derivative estimated altitude and times it by time then 1/(new - old)
void Everest::recalculateGain(double estimate){
    double gainedEstimate = deriveForVelocity(estimate); // pre integrated for altitude
    // gainedEstimate = gainedEstimate * (1.0/SAMPLE_RATE); // integrate to get altitude

    this->state.gain_IMU = 1/fabsf(gainedEstimate-this->state.avgIMU.altitude); // change to previous trusts
    this->state.gain_Baro1 = 1/fabsf(gainedEstimate-this->baro1.altitude);
    this->state.gain_Baro2 = 1/fabsf(gainedEstimate-this->baro2.altitude);
    this->state.gain_Baro3 = 1/fabsf(gainedEstimate-this->baro3.altitude);
    this->state.gain_Real_Baro = 1/fabsf(gainedEstimate-this->realBaro.altitude);
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
double Everest::deriveForVelocity(double estimate){
    double deltaTimeAverage = (this->baro1.deltaTime + this->baro2.deltaTime + this->baro3.deltaTime + this->realBaro.deltaTime + this->state.deltaTimeIMU)/5.0;

    double velocityZ = (this->AltitudeList.secondLastAltitude - 4 * this->AltitudeList.lastAltitude + 3*estimate)/(2.0 * deltaTimeAverage);

    double newAltitude = velocityZ * deltaTimeAverage;

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

#define MAX_LINE_LENGTH 1024

/**
 * Serves to just initialize structs 
*/
int main()
{
    // Setup Madgwick
    // Attach Madgwick to Everest
    MadgwickSetup();

    // test purposes

    // everest.ExternalUpdate(imu1, imu2, baro1, baro2, baro3, realBaro);

    // printf("Altitude: %d\n", everest.ExternalUpdate(imu1, imu2, baro1, baro2, baro3, realBaro));

    file = fopen("everest2.txt", "w+"); // Open the file for appending or create it if it doesn't exist
    if (!file) {
        fprintf(stderr, "Error opening file...exiting\n");
        exit(1);
    }

    FILE *file1 = fopen("C:/Users/Andrey/Documents/EverestRepo/Apogee-Detection-Everest/MadgwickLibrary/IMU_BARO.csv", "r");
    if (!file1) {
        perror("Error opening file");
        return 1;
    }
    // read first line and preset the deltaTime to timestamp 
    char line[MAX_LINE_LENGTH];
    std::clock_t start;
    double duration;

    // correct for varying refresh rate 

    int i = 0;
    
    while (fgets(line, sizeof(line), file1)) {
        // Tokenize the line using strtok
        char *token = strtok(line, ",");
        float accelX = atof(token); // Convert the time value to float

        // Parse accelerometer readings (X, Y, Z)
        // token = strtok(NULL, ",");
        // float accelX = atof(token);
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

        token = strtok(NULL, ",");
        float time = atof(token); // Convert the time value to float

        token = strtok(NULL, ",");
        float pressure = atof(token);

        // token = strtok(NULL, ",");
        // float realPressure = atof(token);

        time = i * DELTA_TIME;

        i++;

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

        // Example: Print all sensor readings
        printf("Raw Time: %.6f s, Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g Pressure: (%.f, %.f, %.f, %.f)\n",
            time, sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ, sensorData.accelX, sensorData.accelY, sensorData.accelZ, baro1.pressure, baro2.pressure, baro3.pressure, realBaro.pressure);

        // everest.IMU_Update(sensorData, sensorData2);
        double eAltitude = everest.ExternalUpdate(sensorData, sensorData2, baro1, baro2, baro3, realBaro);

        printf("Altitude: %f\n", eAltitude);

        clock_t endTime = std::clock();

        duration += endTime - start;

        // printf("Time for one more (seconds): %f\n", duration/CLOCKS_PER_SEC);
    }

    // printf("Overall for (13k samples): %f", duration/CLOCKS_PER_SEC);

    fclose(file1);
    fclose(file);

    return 0;
}
