// Asynchornous complementary filter 
#include "everest.hpp"

// 2 IMUs report on same refresh rate
// dont know if they are in sync
// Asynchronous complementary filter
void IMU_Update(const IMUData& imu1, const IMUData& imu2, int whichOne)
{
    if(whichOne == 1)
    {
        // Update IMU1
        internalIMU1.time = imu1.time;
        internalIMU1.gyroX = imu.gyroX;
        internalIMU1.gyroY = imu.gyroY;
        internalIMU1.gyroZ = imu.gyroZ;

        internalIMU1.accelX = imu.accelX;
        internalIMU1.accelY = imu.accelY;
        internalIMU1.accelZ = imu.accelZ;  

    }
    else
    {
        // Update IMU2
        internalIMU2.time = imu2.time;

        internalIMU2.gyroX = imu2.gyroX;
        internalIMU2.gyroY = imu2.gyroY;
        internalIMU2.gyroZ = imu2.gyroZ;

        internalIMU2.accelX = imu2.accelX;
        internalIMU2.accelY = imu2.accelY;
        internalIMU2.accelZ = imu2.accelZ;

    }

    // Update system state

    // Wait for both IMUs to update

    // Calculate average of IMU parameters
    // #define Q ahrs->quaternion.element
    #define averageIMU state.avgIMU

    averageIMU.gyroX = (internalIMU1.gyroX + internalIMU2.gyroX) / 2.0;
    averageIMU.gyroY = (internalIMU1.gyroY + internalIMU2.gyroY) / 2.0;
    averageIMU.gyroZ = (internalIMU1.gyroZ + internalIMU2.gyroZ) / 2.0;

    averageIMU.accelX = (internalIMU1.accelX + internalIMU2.accelX) / 2.0;
    averageIMU.accelY = (internalIMU1.accelY + internalIMU2.accelY) / 2.0;
    averageIMU.accelZ = (internalIMU1.accelZ + internalIMU2.accelZ) / 2.0;


    // Calculate confidence values



    // feed to Madgwick

    

}

/**
 * Serves to just initialize structs 
*/
int main()
{
    // Initialize structs 
    IMUData internalIMU_1, internalIMU_2;

    // Initialize system state
    systemState state;

    // Initialize baros
    BarosData baro1, baro2, baro3, realBaro;

    // Initially we trust systems equally
    state.confidence_IMU = 1/3.0;
    state.confidence_Baros = 1/3.0;
    state.confidence_Real_Baro = 1/3.0;

    // Initially no apogee detected
    state.apogeeDetected_IMU = false;
    state.apogeeDetected_Baros = false;
    state.apogeeDetected_Real_Baro = false;
    state.finalApogeeDetected = false;

    return 0;
}