// Altitude estimation using multiple sensors
#include "everest.hpp"
#include <stdio.h>
using namespace std;

#define SAMPLE_RATE (100) // replace this with actual sample rate

// Instantiate Everest
madAhrs *ahrs;
Infusion infusion;
Everest everest = Everest::getEverest();
kinematics Kinematics = {0.0, 0.0, 0.0}; // tare to ground

void MadgwickSetup()
{
    // Attaches Madgwick to Everest
    infusion = everest.Initialize();
    ahrs = infusion.getMadAhrs();

    // Define calibration (replace with actual calibration data if available)
    const madMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const madVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const madVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    const madMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const madVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const madVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
    const madMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

    madAhrsInternalStates internal = infusion.madAhrsGetInternalStates(ahrs);
    madAhrsFlags flags = infusion.madAhrsGetFlags(ahrs);
    const madVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    // Initialise algorithms
    madOffset offset = infusion.getOffset();
    // *ahrs = infusion.getMadAhrs();

    madOffsetInitialise(&offset, SAMPLE_RATE);
    madAhrsInitialise(ahrs);

    // Set AHRS algorithm settings
    madAhrsSettings settings = {
            EarthConventionNed,
            0.5f,
            2000.0f, /* replace this with actual gyroscope range in degrees/s */
            10.0f,
            5 * SAMPLE_RATE, /* 5 seconds */
    };

    madAhrsSetSettings(ahrs, &settings);
}

void MadgwickWrapper(SensorDataNoMag data){
// #define ahrs infusion->getMadAhrs(infusion)
    // Infusion infusion = infusion;
    const float timestamp = data.time;
    madVector gyroscope = {data.gyroX, data.gyroY, data.gyroZ}; // replace this with actual gyroscope data in degrees/s
    madVector accelerometer = {data.accelX, data.accelY, data.accelZ}; // replace this with actual accelerometer data in g

    madEuler euler = infusion.getEuler(ahrs);
    madVector earth = infusion.madAhrsGetEarthAcceleration(ahrs);

    // Update gyroscope offset correction algorithm
    madOffset offset = infusion.getOffset();
    gyroscope = madOffsetUpdate(&offset, gyroscope);

    // printf("Offset update Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g, Mag: (%.6f, %.4f, %.6f) uT\n",
    //     data.gyroX, data.gyroY, data.gyroZ, data.accelX, data.accelY, data.accelZ, data.magX, data.magY, data.magZ);

    // printf("Roll %0.3f, Pitch %0.3f, Yaw %0.3f, X %0.3f, Y %0.3f, Z %0.3f\n",
    //        euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
    //        earth.axis.x, earth.axis.y, earth.axis.z);

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    static float previousTimestamp;
    float deltaTime = (float) (timestamp - previousTimestamp);
    previousTimestamp = timestamp;

    // Update gyroscope AHRS algorithm
    infusion.madAhrsUpdateNoMagnetometer(ahrs, gyroscope, accelerometer, deltaTime);

// #undef ahrs

    // return accelerationZ
    // or run numerical integration on accelerationZ
}

// 2 IMUs report on same refresh rate
// dont know if they are in sync
// Asynchronous complementary filter
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

    averageIMU.gyroX = (internalIMU_1.gyroX + internalIMU_2.gyroX) / 2.0;
    averageIMU.gyroY = (internalIMU_1.gyroY + internalIMU_2.gyroY) / 2.0;
    averageIMU.gyroZ = (internalIMU_1.gyroZ + internalIMU_2.gyroZ) / 2.0;

    averageIMU.accelX = (internalIMU_1.accelX + internalIMU_2.accelX) / 2.0;
    averageIMU.accelY = (internalIMU_1.accelY + internalIMU_2.accelY) / 2.0;
    averageIMU.accelZ = (internalIMU_1.accelZ + internalIMU_2.accelZ) / 2.0;

    #undef averageIMU

    // fed to Madgwick
    MadgwickWrapper(state.avgIMU);
}

void Everest::Baro_Update(const BarosData& baro1, const BarosData& baro2, const BarosData& baro3, const BarosData& realBaro)
{
    // Update Baros
    this->baro1.time = baro1.time;
    this->baro1.pressure = baro1.pressure;

    this->baro2.time = baro2.time;
    this->baro2.pressure = baro2.pressure;

    this->baro3.time = baro3.time;
    this->baro3.pressure = baro3.pressure;

    this->realBaro.time = realBaro.time;
    this->realBaro.pressure = realBaro.pressure;

}

double deriveForAltitudeIMU(SensorDataNoMag avgIMU){
    double accelerationZ = avgIMU.accelZ;
    double initialVelocity = Kinematics.initialVelo;
    double initialAltitude = Kinematics.initialAlt;

    // Derive altitude from IMU
    double altitude = (double) (initialAltitude + initialVelocity * (1.0/SAMPLE_RATE) + 0.5 * accelerationZ * pow((1.0/SAMPLE_RATE), 2));

    // return altitude
    return altitude;
}

double convertToAltitude(double pressure){
    // Convert pressure to altitude
    // Convert from 100*millibars to m
    double seaLevelPressure = 1013.25; // sea level pressure in hPa
    double altitude = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.190284));
    return altitude;
}

/**
 * @brief Multi-system trust algorithm. Assumes measurements are updated
*/
systemState Everest::dynamite(){
    double IMUAltitude = deriveForAltitudeIMU(everest.state.avgIMU);

    double BaroAltitude1 = convertToAltitude(everest.baro1.pressure);
    double BaroAltitude2 = convertToAltitude(everest.baro2.pressure);
    double BaroAltitude3 = convertToAltitude(everest.baro3.pressure);
    double RealBaroAltitude = convertToAltitude(everest.realBaro.pressure);

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
    Kinematics.initialVelo = (Kinematics.finalAltitude - Kinematics.initialAlt)/(1.0/SAMPLE_RATE);

    // update altitude
    Kinematics.initialAlt = Kinematics.finalAltitude;

}

double getFinalAltitude(){
    return Kinematics.finalAltitude;
}

/**
 * Serves to just initialize structs 
*/
int main()
{
    // Instantiate Everest
    // Everest everest = getEverest();

    // Setup Madgwick
    // Attach Madgwick to Everest
    MadgwickSetup();

    // test purposes
    SensorDataNoMag imu1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    SensorDataNoMag imu2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    everest.IMU_Update(imu1, imu2);
    everest.Baro_Update({0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0});

    printf("Altitude: %f\n", getFinalAltitude());

    return 0;
}