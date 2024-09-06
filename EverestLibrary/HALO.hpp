#ifndef HALO_HPP
#define HALO_HPP

#include <cmath>
#include <string>
#include <vector>
#include <iostream>

// home
#ifdef HOME
    #include "C:\Users\andin\OneDrive\Documents\AllRepos\UnscentedKalmanFilter\eigen-3.4.0\Eigen\Cholesky"
    #include "C:\Users\andin\OneDrive\Documents\AllRepos\UnscentedKalmanFilter\eigen-3.4.0\Eigen\Dense"
#endif

// away
#ifndef HOME
    #include "C:\Users\Andrey\Documents\UKFRepo\UnscentedKalmanFilter\eigen-3.4.0\eigen-3.4.0\Eigen\Cholesky"
    #include "C:\Users\Andrey\Documents\UKFRepo\UnscentedKalmanFilter\eigen-3.4.0\eigen-3.4.0\Eigen\Dense"
#endif



// bool isBeforeApogeeBool = false;

using namespace Eigen;

/**
 * Keeps track of the kinematics of HALO. Updated
 * with UKF state update
 */
// typedef struct{
//     float initialVelo;
//     float initialAlt;
//     float finalAltitude;
// } kinematics;

/**
 * @brief Scenario struct to store the coefficients of the 3rd degree polynomial for acceleration, velocity and altitude
 *      before and after apogee, also evaluates the acceleration, velocity and altitude at a given time
 */
struct Scenario {
    std::vector<float> beforeApogeeAccel;
    std::vector<float> afterApogeeAccel;

    std::vector<float> beforeApogeeVelo;
    std::vector<float> afterApogeeVelo;

    std::vector<float> beforeApogeeAlt;
    std::vector<float> afterApogeeAlt;

    std::vector<std::vector<float>> BeforeList;
    std::vector<std::vector<float>> AfterList;

    int name;

    std::vector<float> measurement;
    bool isBeforeApogeeBool = true;

    Scenario(std::vector<std::vector<float>> beforelList, std::vector<std::vector<float>> afterList, int Name)
        :BeforeList(beforelList), AfterList(afterList), name(Name){

        // auto [firstPart, secondPart] = findAndSplitVector(list);


        // FILE *file = fopen("Sims.txt", "w+"); // Open the file for appending or create it if it doesn't exist
        // if (!file) {
        //     fprintf(stderr, "Error opening Sims.txt...exiting\n");
        //     exit(1);
        // }

        printf("%d\n", name);

        // // Print the first part
        // // std::cout << "First Part:" << std::endl;
        // fprintf(file, "%s\n", "First Part:");
        // for (const auto& vec : firstPart) {
        //     for (float value : vec) {
        //         // std::cout << value << " ";
        //         fprintf(file, "%f,%f,%f,%f\n", value);
        //     }
        //     // std::cout << std::endl;
        // }

        // // Print the second part
        // // std::cout << "Second Part:" << std::endl;
        // fprintf(file, "%s\n", "Second Part:");
        // for (const auto& vec : secondPart) {
        //     for (float value : vec) {
        //         // std::cout << value << " ";
        //         fprintf(file, "%f,%f,%f,%f\n", value);
        //     }
        //     // std::cout << std::endl;
        // }

    }

    // Function to find the vector and split the list
    std::pair<std::vector<std::vector<float>>, std::vector<std::vector<float>>> findAndSplitVector(
        const std::vector<std::vector<float>>& inputList) {
        
        std::vector<std::vector<float>> firstPart;
        std::vector<std::vector<float>> secondPart;
        bool splitPointFound = false;

        for (const auto& vec : inputList) {
            if (!splitPointFound && vec.size() >= 3 && vec[1] < 1.0f && vec[2] < 1.0f) {
                splitPointFound = true;
            }

            if (splitPointFound) {
                secondPart.push_back(vec);
            } else {
                firstPart.push_back(vec);
            }
        }

        return {firstPart, secondPart};
    }


    /**
     * {Altitude, Velocity, Acceleration}
     */
    void setMeasurement(std::vector<float> measurementVector){
        this->measurement = measurementVector;
    }

    void setIsBeforeApogee(bool isBeforeApogee){
        isBeforeApogeeBool = isBeforeApogee;
    }

    /**
     * Returns list of vectors of scenario {Altitude, Velocity, Acceleration} before or after apogee
     */
    std::vector<std::vector<float>> getLists(){
        if(isBeforeApogeeBool){
            // for(std::vector<float> vec : BeforeList){
            //     for(float value : vec){
            //         printf("%f ", value);
            //     }
            //     printf("\n");
            // }
            return BeforeList;
        }
        else{
            // for(std::vector<float> vec : AfterList){
            //     for(float value : vec){
            //         printf("%f ", value);
            //     }
            //     printf("after\n");
            // }
            return AfterList;
        }

    }

    /** finds vector at specified index **/
    std::vector<float> evaluateVectorAt(int index){
        return getLists()[index];
    }

    /** finds vector at specified time **/
    std::vector<float> evaluateVectorAtTime(float time){
        std::vector<std::vector<float>> list = getLists();
        int index = 0;
        printf("starting to evaluate vector at time %f\n", time);
        std::vector<float> vect = {0,0,0,0};

        for(std::vector<float> vec : list){
            // std::vector<float> vect = {0,0,0,0};
            int valueIndex = 0;

            for(float value : vec){
                vect[valueIndex] = value;
                valueIndex++;
            }

            if(time < vect[3]){
                break;
            }

            index++;

            // printf("Considered: (%f)", vect[3]);
        }


        // while(time > list[index][2]){
        //     printf("Considered: (%f)", list[index][2]);
        //     index++;
        // }

        return vect;
    }

    // state machine on apogee
    float evaluateAcceleration(float time) {
        float measuredAcceleration = measurement[0];

        if(isBeforeApogeeBool){
            // evaluate acceleration at timestep before apogee by using the coefficients at 3rd degree polynomial
            return beforeApogeeAccel[0] * pow(time, 3) + beforeApogeeAccel[1] * pow(time, 2) 
            + beforeApogeeAccel[2] * time + beforeApogeeAccel[3];
        }
        else{
            // evaluate acceleration at timestep after apogee by using the coefficients at 3rd degree polynomial
            return afterApogeeAccel[0] * pow(time, 3) + afterApogeeAccel[1] * pow(time, 2) 
            + afterApogeeAccel[2] * time + afterApogeeAccel[3];
        } 
    }

    float evaluateVelocity(float time){
        if(isBeforeApogeeBool){
            // evaluate velocity at timestep before apogee by using the coefficients at 3rd degree polynomial
            return beforeApogeeVelo[0] * pow(time, 3) + beforeApogeeVelo[1] * pow(time, 2)
            + beforeApogeeVelo[2] * time + beforeApogeeVelo[3];
        }
        else{
            // evaluate velocity at timestep after apogee by using the coefficients at 3rd degree polynomial
            return afterApogeeVelo[0] * pow(time, 3) + afterApogeeVelo[1] * pow(time, 2) 
            + afterApogeeVelo[2] * time + afterApogeeVelo[3];
        }
    }

    float evaluateAltitude(float time){
        if(isBeforeApogeeBool){
            // evaluate altitude at timestep before apogee by using the coefficients at 3rd degree polynomial
            return beforeApogeeAlt[0] * pow(time, 3) + beforeApogeeAlt[1] * pow(time, 2) + beforeApogeeAlt[2] * time + beforeApogeeAlt[3];
        }
        else{
            // evaluate altitude at timestep after apogee by using the coefficients at 3rd degree polynomial
            return afterApogeeAlt[0] * pow(time, 3) + afterApogeeAlt[1] * pow(time, 2) + afterApogeeAlt[2] * time + afterApogeeAlt[3];
        }
    }
    
};

/**
 * @brief Kinematics struct to store the kinematics of the rocket
 */
struct kinematicsHalo {
    float altitudeStore;
};


class HALO{
    public:
        void init(VectorXf &X0, MatrixXf &P0, MatrixXf Q_input, MatrixXf &R0);

        void update();

        void unscentedTransform();

        void stateUpdate();

        void prediction();

        float fAccel, fVelo, fAlt, GPS_Alt;

        // static void setFilteredValues(float FAccel, float fVelo, float fAlt);

        float getFAlt();

        float getFVelo();

        float getFAccel();

        float getGPSAlt();

        void setAlt(float gps_alt);

        // std::vector<float> getGains(float x, float scenario1Distance, float scenario2Distance);

        VectorXf predictNextValues(std::vector<std::vector<float>> &vectors, VectorXf &X_in);

        void setStateVector(float filteredAcc, float filteredVelo, float filteredAlt);

        float interpolate(float x, float scenario1Distance, float scenario2Distance);

        std::vector<std::vector<float>> findNearestScenarios(std::vector<Scenario>& scenarios, VectorXf &measurement);

        float interpolateScenarios(VectorXf &X_in, std::vector<Scenario> &scenarios);

        void calculateSigmaPoints();

        VectorXf X;  // state vector

        VectorXf X0; // current state vector

        MatrixXf observe(MatrixXf sigmaPoints);

        float lambda;

        float N1;

        // MatrixXf predict(MatrixXf sigmaPoints);

        void init(MatrixXf &X0, MatrixXf &P0, MatrixXf Q_input, VectorXf &Z_input, MatrixXf &F);

        VectorXf Z; // measurement vector

        VectorXf dynamicModel(VectorXf &X);

        void setScenarios(std::vector<Scenario> &scenarios){
            this->scenarios = scenarios;
        };

        std::vector<Scenario> getScenarios(){
            return this->scenarios;
        };

        std::vector<Scenario> scenarios;

        bool isBeforeApogeeBoolHALO = false;

        bool isBeforeApogee(float acceleration, float velocity, float altitude, float lastAltitude);

        float deltaTime = 1/3;

        void setDeltaTime(float deltaTime){
            this->deltaTime;
        }

        float getDeltaTime(){
            return this->deltaTime;
        }

        float time = 0;

        void setTime(float time){
            this->time;
        }

        float euclideanDistance(const std::vector<float>& vec1, const VectorXf& vec2);

        std::vector<std::pair<std::vector<float>, std::vector<float>>> listOfGainsSigmaPoints = 
        {{{0.4, 0.4, 0.4}, {0.6, 0.6, 0.6}}, {{0.6, 0.6, 0.6}, {0.4, 0.4, 0.4}}, 
        {{0.2, 0.2, 0.2}, {0.8, 0.8, 0.8}}, {{0.7, 0.7, 0.7}, {0.3, 0.3, 0.3}},
        {{0.3, 0.3, 0.3}, {0.7, 0.7, 0.7}}, {{0.1, 0.1, 0.1}, {0.9, 0.9, 0.9}},
        {{0.8, 0.8, 0.8}, {0.2, 0.2, 0.2}}};
        // {{{0.5, 0.5, 0.5}, {0.5, 0.5, 0.5}}, {{0.5, 0.5, 0.5}, {0.5, 0.5, 0.5}}, 
        // {{0.5, 0.5, 0.5}, {0.5, 0.5, 0.5}}, {{0.5, 0.5, 0.5}, {0.5, 0.5, 0.5}},
        // {{0.5, 0.5, 0.5}, {0.5, 0.5, 0.5}}, {{0.5, 0.5, 0.5}, {0.5, 0.5, 0.5}},
        // {{0.5, 0.5, 0.5}, {0.5, 0.5, 0.5}}};

        void overrideStateWithGPS(float GPS);

        std::vector<int> firstTime = {1, 1, 1, 1, 1, 1 ,1};

        int firstTimeForPoint = 1;

    private:

        float Uaccel;
        float Ualt;
        float Uvelo;

        VectorXf X_in;
        VectorXf X_pred;

        float timeStep = 1/3;

        std::vector<float> prevGain1 = {0.5, 0.5, 0.5};
        std::vector<float> prevGain2 = {0.5, 0.5, 0.5};

    protected:

        MatrixXf sigmaPoints;
        MatrixXf Xprediction;
        MatrixXf Pprediction;
        MatrixXf P;
        MatrixXf Q;
        MatrixXf projectError;
        
        MatrixXf WeightsUKF;
        VectorXf WeightsForSigmaPoints;

        MatrixXf F; // state to next state transition matrix
        MatrixXf H; // state to measurement matrix
        MatrixXf R; // measurement noise covariance matrix
        MatrixXf K; // Kalman gain matrix

        kinematicsHalo KinematicsHalo;

        // kinematics* getKinematics();

        MatrixXf sigPoints;

};

// bool getIsBeforeApogee(){
//     return isBeforeApogeeBool;
// };

// void HALO::setFilteredValues(float FAccel, float FVelo, float FAlt){
//     this->fAccel = FAccel;
//     this->fVelo = FVelo;
//     this->fAlt = FAlt;
// }

// float HALO::getFAlt(){
//     return this->fAlt;
// }

// float HALO::getFVelo(){
//     return this->fVelo;
// }

// float HALO::getFAccel(){
//     return this->fAccel;
// }

// void HALO::setAlt(float gps_alt){
//     this->GPS_Alt = gps_alt;
// }

// float HALO::getGPSAlt(){
//     return this->GPS_Alt;
// }

// kinematics* HALO::getKinematics(){
//     return &Kinematics;
// }

// HALO getHALO(){
//     return 
// }

#endif