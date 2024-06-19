#include "runner.hpp"


#define MAX_LINE_LENGTH 1024
FILE *file;


#define MAX_LINE_LENGTH 1024
FILE *file;
int main() {


// extern Everest everest;
// Everest& everest = Everest::idk();

    // initialization
    // Everest everest = Everest();
    // Everest* everest = Everest::getPointer();
    // everest.setPointer(&everest);
    // everest.createEverest();
    everest1->MadgwickSetup();

    Infusion *infusion = everest1->ExternalInitialize();

        // test purposes
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

    int howMany = 1;

    int i = 0;
    
    while (fgets(line, sizeof(line), file1)) {
        // Tokenize the line using strtok
        // Parse accelerometer readings (X, Y, Z)
        char *token = strtok(line, ",");
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

        token = strtok(NULL, ",");
        float time = atof(token); // Convert the time value to float

        token = strtok(NULL, ",");
        float pressure = atof(token);

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

        // if(howMany <= 10){

        printf("\n#%d Sample--------------------------------------------------------------------------\n\n", howMany);

        // Example: Print all sensor readings
        // if(debug == RAW || debug == ALL){
            printf("Raw Time: %.6f s, Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g Pressure: (%.f, %.f, %.f, %.f)\n",
                time, sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ, sensorData.accelX, sensorData.accelY, sensorData.accelZ, 
                baro1.pressure, baro2.pressure, baro3.pressure, realBaro.pressure);
        // }

        madVector imu1Gyro = {sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ};
        madVector imu1Accel = {sensorData.accelX, sensorData.accelY, sensorData.accelZ};

        madVector imu1GyroAligned = infusion->AxesSwitch(imu1Gyro, MadAxesAlignmentPXPYNZ);
        madVector imu1AccelAligned = infusion->AxesSwitch(imu1Accel, MadAxesAlignmentPXPYNZ);

        madVector imu2Gyro = {sensorData2.gyroX, sensorData2.gyroY, sensorData2.gyroZ};
        madVector imu2Accel = {sensorData2.accelX, sensorData2.accelY, sensorData2.accelZ};

        madVector imu2GyroAligned = infusion->AxesSwitch(imu2Gyro, MadAxesAlignmentPXPYNZ);
        madVector imu2AccelAligned = infusion->AxesSwitch(imu2Accel, MadAxesAlignmentPXPYNZ);

        // if(debug == Secondary || debug == ALL){
            printf("Aligned: Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g\n",
                imu1GyroAligned.axis.x, imu1GyroAligned.axis.y, imu1GyroAligned.axis.z, imu1AccelAligned.axis.x, imu1AccelAligned.axis.y, imu1AccelAligned.axis.z);
        // }

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
            
        // double eAltitude = everest.AlignedExternalUpdate(sensorData, sensorData2, baro1, baro2, baro3, realBaro, MadAxesAlignmentPXPYNZ);
        double eAltitude = everest1->ExternalUpdate(sensorData, sensorData2, baro1, baro2, baro3, realBaro);

        printf("Altitude: %f\n", eAltitude);


        // }

        howMany++;
    }

    fclose(file1);
    fclose(file);

/**
 * Serves to just initialize structs 
*/
int main()
{
    // Setup Madgwick
    // Attach Madgwick to Everest
    Everest everest = Everest();
    everest.MadgwickSetup();

    // test purposes
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

    int howMany = 1;

    int i = 0;
    
    while (fgets(line, sizeof(line), file1)) {
        // Tokenize the line using strtok
        // Parse accelerometer readings (X, Y, Z)
        char *token = strtok(line, ",");
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

        token = strtok(NULL, ",");
        float time = atof(token); // Convert the time value to float

        token = strtok(NULL, ",");
        float pressure = atof(token);

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

        // if(howMany <= 10){

        printf("\n#%d Sample--------------------------------------------------------------------------\n\n", howMany);

        // Example: Print all sensor readings
        // if(debug == RAW || debug == ALL){
            printf("Raw Time: %.6f s, Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g Pressure: (%.f, %.f, %.f, %.f)\n",
                time, sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ, sensorData.accelX, sensorData.accelY, sensorData.accelZ, 
                baro1.pressure, baro2.pressure, baro3.pressure, realBaro.pressure);
        // }

        madVector imu1Gyro = {sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ};
        madVector imu1Accel = {sensorData.accelX, sensorData.accelY, sensorData.accelZ};

        madVector imu1GyroAligned = AxesSwitch(imu1Gyro, MadAxesAlignmentPXPYNZ);
        madVector imu1AccelAligned = AxesSwitch(imu1Accel, MadAxesAlignmentPXPYNZ);

        madVector imu2Gyro = {sensorData2.gyroX, sensorData2.gyroY, sensorData2.gyroZ};
        madVector imu2Accel = {sensorData2.accelX, sensorData2.accelY, sensorData2.accelZ};

        madVector imu2GyroAligned = AxesSwitch(imu2Gyro, MadAxesAlignmentPXPYNZ);
        madVector imu2AccelAligned = AxesSwitch(imu2Accel, MadAxesAlignmentPXPYNZ);

        // if(debug == Secondary || debug == ALL){
            printf("Aligned: Gyro: (%.6f, %.6f, %.6f) deg/s, Accel: (%.6f, %.6f, %.6f) g\n",
                imu1GyroAligned.axis.x, imu1GyroAligned.axis.y, imu1GyroAligned.axis.z, imu1AccelAligned.axis.x, imu1AccelAligned.axis.y, imu1AccelAligned.axis.z);
        // }

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
            
            // double eAltitude = everest.AlignedExternalUpdate(sensorData, sensorData2, baro1, baro2, baro3, realBaro, MadAxesAlignmentPXPYNZ);
            double eAltitude = everest.ExternalUpdate(sensorData, sensorData2, baro1, baro2, baro3, realBaro);

            printf("Altitude: %f\n", eAltitude);


        // }

        howMany++;

        clock_t endTime = std::clock();

        // duration += endTime - start;

        // printf("Time for one more (seconds): %f\n", duration/CLOCKS_PER_SEC);
    }

    // printf("Overall for (13k samples): %f", duration/CLOCKS_PER_SEC);

    fclose(file1);
    fclose(file);

    return 0;
}