// Asynchornous complementary filter 
#include "everest.hpp"
#include "C:/Users/Andrey/Documents/EverestRepo/Apogee-Detection-Everest/MadgwickLibrary/infusion.hpp"



// Initialize Madgwick filter


// 2 IMUs report on same refresh rate
// dont know if they are in sync
// Asynchronous complementary filter
void Everest::IMU_Update(const IMUData& imu1, const IMUData& imu2, int whichOne)
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
    return 0;
}