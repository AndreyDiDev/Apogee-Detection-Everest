// Asynchornous complementary filter 
#include "everest.hpp"
#include "C:/Users/Andrey/Documents/EverestRepo/Apogee-Detection-Everest/MadgwickLibrary/infusion.hpp"

#define SAMPLE_RATE (100) // replace this with actual sample rate

void MadgwickSetup()
{
    // Define calibration (replace with actual calibration data if available)
    const madMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const madVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const madVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    const madMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const madVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const madVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
    const madMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

    madAhrsInternalStates internal;
    madAhrsFlags flags;
    const madVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    // Initialise algorithms
    madOffset offset;
    madAhrs ahrs;

    madOffsetInitialise(&offset, SAMPLE_RATE);
    madAhrsInitialise(&ahrs);
}

void MadgwickWrapper(SensorDataNoMag data, madAhrs ahrs){
    const float timestamp = data.time;
    madVector gyroscope = {data.gyroX, data.gyroY, data.gyroZ}; // replace this with actual gyroscope data in degrees/s
    madVector accelerometer = {data.accelX, data.accelY, data.accelZ}; // replace this with actual accelerometer data in g

    madEuler euler = madQuaternionToEuler(madAhrsGetQuaternion(ahrs));
    madVector earth = madAhrsGetEarthAcceleration(ahrs);
}

// 2 IMUs report on same refresh rate
// dont know if they are in sync
// Asynchronous complementary filter
void Everest::IMU_Update(const SensorDataNoMag& imu1, const SensorDataNoMag& imu2, int whichOne)
{
    if(whichOne == 1)
    {
        // Update IMU1
        internalIMU_1.time = imu1.time;
        internalIMU_1.gyroX = imu1.gyroX;
        internalIMU_1.gyroY = imu1.gyroY;
        internalIMU_1.gyroZ = imu1.gyroZ;

        internalIMU_1.accelX = imu1.accelX;
        internalIMU_1.accelY = imu1.accelY;
        internalIMU_1.accelZ = imu1.accelZ;  

    }
    else
    {
        // Update IMU2
        internalIMU_2.time = imu2.time;

        internalIMU_2.gyroX = imu2.gyroX;
        internalIMU_2.gyroY = imu2.gyroY;
        internalIMU_2.gyroZ = imu2.gyroZ;

        internalIMU_2.accelX = imu2.accelX;
        internalIMU_2.accelY = imu2.accelY;
        internalIMU_2.accelZ = imu2.accelZ;

    }

    // Update system state

    // Wait for both IMUs to update

    // Calculate average of IMU parameters
    #define averageIMU state.avgIMU

    averageIMU.gyroX = (internalIMU_1.gyroX + internalIMU_2.gyroX) / 2.0;
    averageIMU.gyroY = (internalIMU_1.gyroY + internalIMU_2.gyroY) / 2.0;
    averageIMU.gyroZ = (internalIMU_1.gyroZ + internalIMU_2.gyroZ) / 2.0;

    averageIMU.accelX = (internalIMU_1.accelX + internalIMU_2.accelX) / 2.0;
    averageIMU.accelY = (internalIMU_1.accelY + internalIMU_2.accelY) / 2.0;
    averageIMU.accelZ = (internalIMU_1.accelZ + internalIMU_2.accelZ) / 2.0;

    #undef averageIMU

    // feed to Madgwick
    // MadgwickAHRSupdateIMU(ahrs, averageIMU.gyroX, averageIMU.gyroY, 
    // averageIMU.gyroZ, averageIMU.accelX, averageIMU.accelY, averageIMU.accelZ);
    

}

/**
 * Serves to just initialize structs 
*/
int main()
{
    Everest everest = Everest();

    SensorDataNoMag imu1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    SensorDataNoMag imu2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    everest.IMU_Update(imu1, imu2, 1);

    // printf()
    return 0;
}