#include "HALO.hpp"
// #include "everestTaskHPP.hpp"
#include <fstream>

// #define LOGON

// home
#ifdef HOME
    #include "C:\Users\andin\OneDrive\Documents\AllRepos\UnscentedKalmanFilter\eigen-3.4.0\Eigen\Dense"
#endif

// away
#ifndef HOME
    #include "C:\Users\Andrey\Documents\UKFRepo\UnscentedKalmanFilter\eigen-3.4.0\eigen-3.4.0\Eigen\Dense"
#endif

#ifndef HALO_CPP
#define HALO_CPP

// Integration of Everest with UKF = HALO

#define REFRESH_RATE 20

#define printf(...) ;

// Constants for the UKF
#define N 6
#define dim 6
#define alpha 0.1
#define beta 2
#define k 3 - dim // 3 dimensions

using namespace Eigen;

// Madgwick -> IMUs, Baros

// UKF -> GPS, Dynamic Model = augmented state vector

// Madgwick -(Xi = Filtered Altitude)> z = GPS

void HALO::init(VectorXf &X0, MatrixXf &P0, MatrixXf Q_input, MatrixXf &R0){
    // Input: Estimate Uncertainty -> system state
    // Initial Guess
    this->X0 = X0;
    this->P = P0;
    this->Q = Q_input;
    this->R = R0;

    // std::cout << "X0: \n" << X0 << std::endl;
    // std::cout << "P0: \n" << P0 << std::endl;
    // std::cout << "Q: \n" << Q_input << std::endl;

    // F is for state to next state transition
    // P0 = initial guess of state covariance matrix
    // Q = process noise covariance matrix -> dynamic model std
    // R = measurement noise covariance matrix -> sensor std

    // std::cout << "k: " << k << std::endl;

    // std::cout << "N: " << N << std::endl;

    // Weights for sigma points
    float dime = 3;
    float k1 = 3 - dime;
    float w0_m = k1 / (dime + k1);    // weight for first sPoint when cal covar
    float w_i = 1/ (2 * ( dime + k1));
    
    // std::cout << "w0_m: " << w0_m << std::endl;
    
    // std::cout << "w_i: " << w_i << std::endl;

    MatrixXf Weights(7, 7);
    VectorXf W(7, 1);

    W.setConstant(7, w_i);

    for(int i = 1; i < 7; i++){
        Weights.diagonal()[i] = w_i;
    }

    Weights(0) = w0_m;

    this->WeightsUKF = Weights;

    // std::cout << "Weights: \n" << Weights << std::endl;

    // errors can be because you didnt instantiate the matrix
    // or trying to make a vector and declaring as a matrix
    VectorXf WeightsForSigmaPoints(7, 1);
    WeightsForSigmaPoints.setConstant(7, w_i);
    WeightsForSigmaPoints(0) = w0_m;
    this->WeightsForSigmaPoints = WeightsForSigmaPoints;

    // std::cout << "WeightsForSigmaPoints: \n" << WeightsForSigmaPoints << std::endl;

    this->KinematicsHalo.altitudeStore = X0(0);

    // Z_in = [GPS altitude]
    // Z_in << this->getGPSAlt();
    this->N1 = 3;

    calculateSigmaPoints();
}

// Update Step-------------------------------------
void HALO::update(){
    unscentedTransform();

    // stateUpdate();
}

void HALO::unscentedTransform(){
    // measurement vector
    // Z = h (X) = altitude

    // N = number of dimensions
    // number of s points = 2N +1
    // all others = xn,n + sqrt((N+k)Pn,n) for 1 to N
    // change sign to negative when i = N+1, .... 2N

    //  propagate s points measurment to state equation

    // compute weights of s points

    // w0 = k/(N+k) for the first mean s point

    // wi = 1/2(N+k)

    // Zn = sigma(wi Zn)

    // approx mean and covar pf output distribution

    // mean = xhat n+1, n = sum 2N for wi Xn+1,n

    // covar P n+1, n = sum to 2N for wi (Xn+1,n - xhatn+1,n)(same transposed)

    // for gaussian distribution, set N + k = 3

}

void HALO::stateUpdate(){
    // Xn = Xn-1 + K (Zn - EstZn)
    // std::cout << "\n---- State Update ---- \n" << std::endl;

    MatrixXf observedValues(3, 7);
    observedValues.setZero(3, 7);

    for (int i = 0; i < (2 * this->N1) + 1; i++){
        observedValues.col(i) = sigPoints.col(i);
    }

    // std::cout << "Observed Values: \n" << observedValues << std::endl;
    // std::cout << "WeightsForSigmaPoints: \n" << WeightsForSigmaPoints << std::endl;

    // calculate the mean of the observed values
    VectorXf zMean(3);
    zMean.setZero(3);
    zMean = observedValues * WeightsForSigmaPoints;

    // std::cout << "\nZ mean:\n " << zMean << std::endl;
    this->Z = zMean;

    // calculate covariance of Z
    MatrixXf zCovar(3, 7);
    zCovar.setZero(3, (2 * 3) + 1);

    for (int i = 0; i < 3; i++)
    {
        zCovar.row(i) = (observedValues.row(i).array() - zMean.row(i).value()).matrix();
    }

    // std::cout << "\nZ Covar:\n " << zCovar << std::endl;
    // std::cout << "R: " << this->R << std::endl;

    // calculate the innovation covariance, measurement covariance
    MatrixXf Pz(3, 3);
    Pz.setZero(3, 3);

    Pz = (zCovar * WeightsForSigmaPoints.asDiagonal() * zCovar.transpose()) + this->R;

    // std::cout << "\nPz:\n " << Pz << std::endl;

    // calculate the cross covariance
    MatrixXf Pxz(3,2);
    Pxz.setZero();

    // std::cout << "\nprojectError: \n" << projectError << std::endl;

    Pxz = projectError * WeightsForSigmaPoints.asDiagonal() * zCovar.transpose(); 

    // std::cout << "\nPxz:\n " << Pxz << std::endl;

    // calculate the Kalman gain
    MatrixXf K(3, 3);
    K.setZero();

    K = Pxz * Pz.inverse();

    // std::cout << "X: \n" << this->X << std::endl;
    // std::cout << "\nKalman Gain: \n" << K << std::endl;

    // update the state vector
    printf("\nthis->Xprediction + K * (this->X - zMean)\n\n");

    // std::cout << "Xprediction\n" <<  this->Xprediction << std::endl;
    // std::cout << "\nthis->X\n" << this->X;
    // std::cout << "\n\nzMean\n\n" << zMean;

    bool kZero = false;

    for(int row = 0; row < 3; row++){
        for(int col = 0; col < 3; col++){
            if(std::isnan(K(row, col))){
                std::cout << "NAN detected in the Kalman Gain, defaulting to 0" << std::endl;

                FILE* log = fopen("log.txt", "a+"); // Open the file for appending or create it if it doesn't exist

                kZero = true;

                if (!log) {
                    fprintf(stderr, "Error opening log.txt...exiting\n");
                    exit(1);
                }

                fprintf(log, "At time %f s NAN detected in the Kalman Gain, " , this->time);

                fclose(log);

                K(row, col) = 0;
            }
        }
    }

    if(kZero){

        FILE* log = fopen("log.txt", "a+"); // Open the file for appending or create it if it doesn't exist

        if (!log) {
            fprintf(stderr, "Error opening log.txt...exiting\n");
            exit(1);
        }

        fprintf(log, "\n");
        fclose(log);
    }

    // std::cout << "\n\nK\n" << K;

    VectorXf difference(3,1);
    difference.setZero();
    difference << this->X[2] - zMean(0), this->X[1] - zMean(1), this->X[0] - zMean(2);

    // std::cout << "\n\n difference\n" << difference << std::endl;

    X0 = this->Xprediction + K * difference;

    // check and update before apogee bool
    if(!this->isBeforeApogeeBoolHALO){
        std::vector<Scenario> scenarios = this->getScenarios();

        for(Scenario scenario : scenarios){
            scenario.setIsBeforeApogee(false);
        }

    }else{
        // check if the rocket is before apogee
        this->isBeforeApogeeBoolHALO = isBeforeApogee(this->Uaccel, this->Uvelo, this->Ualt, this->KinematicsHalo.altitudeStore);
    }

    if(std::isnan(X0(0)) || std::isnan(X0(1)) || std::isnan(X0(2))){

        FILE* log = fopen("log.txt", "a+"); // Open the file for appending or create it if it doesn't exist

        if (!log) {
            fprintf(stderr, "Error opening log.txt...exiting\n");
            exit(1);
        }

        fprintf(log, "At time %f s NAN detected in the state update, defaulting to Prediction as Estimation\n" , this->time);

        std::cout << "NAN detected in the state update, defaulting to Prediction as Estimation" << std::endl;

        X0 = this->Xprediction;

        fclose(log);
    }

    // std::cout << "\nEstimated X (HALO): \n" << X0 << std::endl;

    this->KinematicsHalo.altitudeStore = X0(0);

    // update the estimate covariance matrix
    // when variations in the estimate are small then its good
    MatrixXf P1(3,3);
    P1.setZero();

    // // std::cout << "Pprediction: " << Pprediction << std::endl;
    // // std::cout << "K: " << K << std::endl;
    // // std::cout << "Pz: " << Pz << std::endl;

    P1 = Pprediction - (K * Pz * K.transpose());

    this->P = P1;

    // std::cout << "\nP(1,1):\n " << P << std::endl;

    // std::cout << "\n end of state Update\n " << std::endl;

    calculateSigmaPoints();

}
// ------------------------------------------------

// Prediction--------------------------------------
void HALO::prediction(){

    // initialize scenario to default model-A(vg)

    // given the Madgwick acc, velo, alt

    // MatrixXf sigmaPoints(2*N+1, 3);
    // sigmaPoints.setZero(2*N+1, 3);

    // calculate sigma points
    // sigmaPoints = calculateSigmaPoints();

    // interpolate between 2 models


    // predict scenario t+1 based on interpolated values

}


void HALO::calculateSigmaPoints() {

    // std::cout << "X0: " << X0 << std::endl;
    // std::cout << " ---- Predict Step ---- \n" << std::endl;

    // std::cout << "Q: " << Q << std::endl;

    float mutliplier = 3; // N - lambda

    // std::cout << "Multiplier: " << mutliplier << std::endl;

    MatrixXf L( ((mutliplier) *P).llt().matrixL());
    // std::cout << L.col(0) << std::endl;

    // std::cout << "N " << this->N1 << std::endl;

    // std::cout << "L: \n" << L << std::endl;

    // Initialize sigma points matrix
    MatrixXf sigmaPoints(3, 7);
    sigmaPoints.setZero();

    // Set the first sigma point
    sigmaPoints.col(0) = X0;

    // Set the remaining sigma points
    for (int i = 1; i < this->N1 + 1; i++) {
        sigmaPoints.col(i) = X0 + L.col(i - 1);
    }

    for(int j = this->N1 + 1; j < (2 * this->N1) + 1; j++){
        sigmaPoints.col(j) = X0 - L.col(j - this->N1 - 1);
    }

    // before dynamics
    // std::cout << "before dynamics sPoints: \n" << sigmaPoints << std::endl;

    #ifdef LOGON
    FILE* file = fopen("gains.txt", "a+");
    if (!file) {
        fprintf(stderr, "Error opening gains.txt...exiting\n");
        exit(1);
    }

    FILE* sigmaPointsFile = fopen("sigmaPoints.txt", "a+");
    if (!sigmaPointsFile) {
        fprintf(stderr, "Error opening sigmaPoints.txt...exiting\n");
        exit(1);
    }

    FILE* sigmaPointsFile1 = fopen("sigmaPoints1.txt", "a+");
    if (!sigmaPointsFile) {
        fprintf(stderr, "Error opening sigmaPoints.txt...exiting\n");
        exit(1);
    }

    FILE* sigmaPointsFile2 = fopen("sigmaPoints2.txt", "a+");
    if (!sigmaPointsFile) {
        fprintf(stderr, "Error opening sigmaPoints.txt...exiting\n");
        exit(1);
    }

    FILE* sigmaPointsFile3 = fopen("sigmaPoints3.txt", "a+");
    if (!sigmaPointsFile) {
        fprintf(stderr, "Error opening sigmaPoints.txt...exiting\n");
        exit(1);
    }

    FILE* sigmaPointsFile4 = fopen("sigmaPoints4.txt", "a+");
    if (!sigmaPointsFile) {
        fprintf(stderr, "Error opening sigmaPoints.txt...exiting\n");
        exit(1);
    }

    FILE* sigmaPointsFile5 = fopen("sigmaPoints5.txt", "a+");
    if (!sigmaPointsFile) {
        fprintf(stderr, "Error opening sigmaPoints.txt...exiting\n");
        exit(1);
    }

    FILE* sigmaPointsFile6 = fopen("sigmaPoints6.txt", "a+");
    if (!sigmaPointsFile) {
        fprintf(stderr, "Error opening sigmaPoints.txt...exiting\n");
        exit(1);
    }

    #endif

    // propagate sigma points through the dynamic model
    for (int i = 0; i < (2 * this->N1) + 1; i++){
        // load variables
        VectorXf column = sigmaPoints.col(i);
        this->firstTimeForPoint = firstTime[i];
        this->prevGain1 = this->listOfGainsSigmaPoints[i].first;
        this->prevGain2 = this->listOfGainsSigmaPoints[i].second;

        printf("\nprevGain1 (%f,%f,%f)\n", this->prevGain1[0], this->prevGain1[1], this->prevGain1[2]);
        printf("\nsPoint[%d]: \n", i);

        // update
        sigmaPoints.col(i) = dynamicModel(column);
        this->listOfGainsSigmaPoints[i] = {this->prevGain1, this->prevGain2};
        this->firstTime[i] = this->firstTimeForPoint;
        #ifdef LOGON
        fprintf(file, " ");
        #endif
        printf("List of Gains\n {(%f, %f, %f), (%f, %f, %f)},\n {(%f, %f, %f), (%f, %f, %f)},\n {(%f, %f, %f), (%f, %f, %f)},\n {(%f, %f, %f), (%f, %f, %f)},\n {(%f, %f, %f), (%f, %f, %f)},\n {(%f, %f, %f), (%f, %f, %f)}\n",
        listOfGainsSigmaPoints[0].first[0], listOfGainsSigmaPoints[0].first[1], listOfGainsSigmaPoints[0].first[2], listOfGainsSigmaPoints[0].second[0], listOfGainsSigmaPoints[0].second[1], listOfGainsSigmaPoints[0].second[2],
        listOfGainsSigmaPoints[1].first[0], listOfGainsSigmaPoints[1].first[1], listOfGainsSigmaPoints[1].first[2], listOfGainsSigmaPoints[1].second[0], listOfGainsSigmaPoints[1].second[1], listOfGainsSigmaPoints[1].second[2],
        listOfGainsSigmaPoints[2].first[0], listOfGainsSigmaPoints[2].first[1], listOfGainsSigmaPoints[2].first[2], listOfGainsSigmaPoints[2].second[0], listOfGainsSigmaPoints[2].second[1], listOfGainsSigmaPoints[2].second[2],
        listOfGainsSigmaPoints[3].first[0], listOfGainsSigmaPoints[3].first[1], listOfGainsSigmaPoints[3].first[2], listOfGainsSigmaPoints[3].second[0], listOfGainsSigmaPoints[3].second[1], listOfGainsSigmaPoints[3].second[2],
        listOfGainsSigmaPoints[4].first[0], listOfGainsSigmaPoints[4].first[1], listOfGainsSigmaPoints[4].first[2], listOfGainsSigmaPoints[4].second[0], listOfGainsSigmaPoints[4].second[1], listOfGainsSigmaPoints[4].second[2],
        listOfGainsSigmaPoints[5].first[0], listOfGainsSigmaPoints[5].first[1], listOfGainsSigmaPoints[5].first[2], listOfGainsSigmaPoints[5].second[0], listOfGainsSigmaPoints[5].second[1], listOfGainsSigmaPoints[5].second[2]);
    }

    #ifdef LOGON

    fprintf(file, "\n");

    fprintf(sigmaPointsFile,  "%f,%f,%f\n", sigmaPoints(0,0), sigmaPoints(1,0), sigmaPoints(2,0));
    fprintf(sigmaPointsFile1, "%f,%f,%f\n", sigmaPoints(0,1), sigmaPoints(1,1), sigmaPoints(2,1));
    fprintf(sigmaPointsFile2, "%f,%f,%f\n", sigmaPoints(0,2), sigmaPoints(1,2), sigmaPoints(2,2));
    fprintf(sigmaPointsFile3, "%f,%f,%f\n", sigmaPoints(0,3), sigmaPoints(1,3), sigmaPoints(2,3));
    fprintf(sigmaPointsFile4, "%f,%f,%f\n", sigmaPoints(0,4), sigmaPoints(1,4), sigmaPoints(2,4));
    fprintf(sigmaPointsFile5, "%f,%f,%f\n", sigmaPoints(0,5), sigmaPoints(1,5), sigmaPoints(2,5));
    fprintf(sigmaPointsFile6, "%f,%f,%f\n",sigmaPoints(0,6), sigmaPoints(1,6), sigmaPoints(2,6));

    fclose(file);
    fclose(sigmaPointsFile);
    fclose(sigmaPointsFile1);
    fclose(sigmaPointsFile2);
    fclose(sigmaPointsFile3);
    fclose(sigmaPointsFile4);
    fclose(sigmaPointsFile5);
    fclose(sigmaPointsFile6);

    #endif


    // std::cout << "\nafter predict sPoints: \n" << sigmaPoints << std::endl;

    // std::cout << "Sigma Points row: " << sigmaPoints.rows() << " col: " << sigmaPoints.cols() << std::endl;
    // std::cout << "Sigma Points row 0 \n" << sigmaPoints.row(0) << std::endl;
    // std::cout << "Sigma Points row 0\n" << sigmaPoints(all, all) << std::endl;

    // std::cout << "WeightsForSigmaPoints row: " << WeightsForSigmaPoints.rows() 
    // << " col: " << WeightsForSigmaPoints.cols() << std::endl;

    // calculate the mean and covariance of the sigma points
    VectorXf xPreMean(3,1);
    for(int row = 0; row < this->N1; row++){
        float sum00 = 0;
        for(int col = 0; col < 2*this->N1 + 1; col++){
            // std::cout << "sP (" << row << ", " << col << ")" << "= " << sigmaPoints(row, col) << std::endl;
            sum00 += sigmaPoints(row, col) * WeightsForSigmaPoints(col);
        }
        xPreMean(row) = sum00;
        // std::cout << "XpreMean: \n" << xPreMean << std::endl;
    }

    // std::cout << "\nXprediction: \n" << xPreMean << std::endl;
    // std::cout << "Xprediction: \n" << Xprediction << std::endl;
    this->Xprediction = xPreMean;

    MatrixXf projError(3, 7);
    projError.setZero(3, 7);
    
    // std::cout << "Sigma Points row: " << sigmaPoints.rows() << " col: " << sigmaPoints.cols() << std::endl;
    // std::cout << "xPreMean row: " << xPreMean.rows() << " col: " << xPreMean.cols() << std::endl;
    // std::cout << "sigmaPoints (0,3) " << projError(0,3) << std::endl;

    for (int i = 0; i < this->N1; i++)
    {
        projError.row(i) = (sigmaPoints.row(i).array() - (this->Xprediction).row(i).value()).matrix();
    }

    // std::cout << "\nProject Error: \n" << projError << std::endl;

    this->projectError = projError;

    MatrixXf Pprediction(3,3);
    Pprediction.setZero(3,3);

    Pprediction = projError * WeightsForSigmaPoints.asDiagonal() * projError.transpose() + this->Q;

    // std::cout << "\nPprediction: \n" << Pprediction << std::endl;

    this->Pprediction = Pprediction;

    this->sigPoints =  sigmaPoints;

    // std::cout << " ---- End Predict Step ---- \n" << std::endl;
}

// Function to interpolate between two nearest scenarios
float HALO::interpolateScenarios(VectorXf &X_in, std::vector<Scenario> &scenarios) {

    // Find the nearest 2 scenarios to the current state for each measure
    // auto predicted_acc = findNearestScenarios(scenarios, X_in(0), X_in(1), 'a');
    // auto predicted_velo = findNearestScenarios(scenarios, X_in(0), X_in(1), 'v');
    // auto predicted_alt = findNearestScenarios(scenarios, X_in(0), X_in(1), 'h');

    // Interpolate between the two scenarios
    // float interpolated_acc = interpolate(X_in(1), predicted_acc[0].first, predicted_acc[1].first);
    // float interpolated_velo = interpolate(X_in(2), predicted_velo[0].first, predicted_velo[1].first);
    // float interpolated_alt = interpolate(X_in(3), predicted_alt[0].first, predicted_alt[1].first);

    // X_in << interpolated_acc, interpolated_velo, interpolated_alt;

    // // Save the interpolated scenarios for the next iteration
    // Scenario scenario1(predicted_acc[0].second, predicted_velo[0].second, predicted_alt[0].second);
    // Scenario scenario2(predicted_acc[1].second, predicted_velo[1].second, predicted_alt[1].second);

    // Store the scenarios in a vector or any other suitable data structure
    std::vector<Scenario> nextScenarios;
    // nextScenarios.push_back(scenario1);
    // nextScenarios.push_back(scenario2);

    // predictNextValues(time, nextScenarios);

    // return X_in;
    return 0;

}

// Function to calculate the Euclidean distance between two 3D vectors
float HALO::euclideanDistance(const std::vector<float>& vec1, const VectorXf& vec2) {
    float x1, y1, z1, x2, y2, z2;

    x1 = vec1[0];
    y1 = vec1[1];
    z1 = vec1[2];

    x2 = vec2(0);
    y2 = vec2(1);
    z2 = vec2(2);

    // printf("vec (%f,%f,%f) meas (%f,%f,%f)\n", x1, y1, z1, x2, y2, z2);

    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2));
}

/**
 * @brief Given a list of scenarios, find the nearest 2 scenarios and returns the vectors of the nearest scenarios
 */
std::vector<std::vector<float>> HALO::findNearestScenarios(std::vector<Scenario>& scenarios, VectorXf &measurement) {
    // printf("findNearestScenarios\n");
    std::vector<std::pair<float, std::pair<float, Scenario>>> distances;
    float minDistance = std::numeric_limits<float>::max();
    int i = 0;

    for(Scenario scenario : scenarios){
        i = 0;
        int lowestDistanceIndex = 0;
        std::vector<float> lowestVector = scenario.getLists()[0];
        minDistance = std::numeric_limits<float>::max();

        std::pair<std::vector<float>, size_t> vect = {{0,0,0}, 0};

        std::vector<float> measurementVec = {measurement(0), measurement(1), measurement(2)};

        printf("Asking tree for measurementVec: %f, %f, %f\n", measurementVec[0], measurementVec[1], measurementVec[2]);

        vect = scenario.nearestKDTree(measurementVec);

        minDistance = euclideanDistance(vect.first, measurement);

        lowestDistanceIndex = vect.second;

        // for(std::vector<float> vec : scenario.getLists()){
        //     int valueIndex = 0;

        //     for(float value : vec){
        //         vect[valueIndex] = value;
        //         valueIndex++;
        //     }

        //     // printf(" NS vec: %f, %f, %f\n", vect[0], vect[1], vect[2]);
            
        //     float distance = euclideanDistance(vect, measurement);
        //     // printf("eucledian distance %f\n", distance);

        //     if (distance < minDistance) {
        //         minDistance = distance;
        //         lowestDistanceIndex = i;
        //         lowestVector = scenario.getLists()[i];
        //     }

        //     i++;
        // }

        // minDistance to order the list and get lowest 2
        // index go evaluate scenario at n+1
        std::pair<float, std::pair<float, Scenario>> vector = {minDistance, {lowestDistanceIndex, scenario}};
        distances.emplace_back(vector);
        
    }

    // print distances list
    for(int i = 0; i < 6; i++){
        printf("minDistance: %f, lowestIndex: %d, scenario %d\n", distances[i].first, distances[i].second.first, 
        distances[i].second.second.name);
    }

    float lowestDistance = std::numeric_limits<int>::max();
    int lowestDistanceIndex = 0;
    float secondLowestDistance = std::numeric_limits<int>::max();
    int secondLowestDistanceIndex = 0;

    for(int i = 0; i < 6; i++){
        if(distances[i].first < lowestDistance){
            printf("new lowest distance %f\n", distances[i].first);

            secondLowestDistance = lowestDistance;
            secondLowestDistanceIndex = lowestDistanceIndex;

            lowestDistance = distances[i].first;
            lowestDistanceIndex = i;
        } else if (distances[i].first < secondLowestDistance){
            printf("new second lowest distance %f\n", distances[i].first);
            secondLowestDistance = distances[i].first;
            secondLowestDistanceIndex = i;
        }
    }

    printf("lowest distance(%d) = %f, second lowest distance(%d) = %f\n", lowestDistanceIndex, lowestDistance, secondLowestDistanceIndex, secondLowestDistance);

    #ifdef LOGON

    FILE* file = fopen("nearestScenarios.txt", "a+");
    if (!file) {
        fprintf(stderr, "Error opening nearestScenarios.txt...exiting\n");
        exit(1);
    }

    FILE* file2 = fopen("nearestScenariosFormatted.txt", "a+");
    if (!file2) {
        fprintf(stderr, "Error opening nearestScenariosFormatted.txt...exiting\n");
        exit(1);
    }

    fprintf(file, "%f,%f,", lowestDistance, secondLowestDistance);
    fprintf(file, "%d,%d,", lowestDistanceIndex, secondLowestDistanceIndex);
    fprintf(file, "%d,%d", distances[lowestDistanceIndex].second.second.name, distances[secondLowestDistanceIndex].second.second.name);
    fprintf(file, "%f,%f\n", distances[lowestDistanceIndex].first, distances[secondLowestDistanceIndex].first);

    fprintf(file2, "For Meas(%f,%f,%f) V1(%f) lowest(%f),secondL(%f),", measurement[0], measurement[1], measurement[2], lowestDistance, secondLowestDistance);
    fprintf(file2, "lowestIndex(%d),secondLI(%d),", lowestDistanceIndex, secondLowestDistanceIndex);
    fprintf(file2, "lowestName(%d),secondLN(%d)\n", distances[lowestDistanceIndex].second.second.name, distances[secondLowestDistanceIndex].second.second.name);
    fprintf(file2, "list: %f,%f,%f,%f,%f,%f\n", distances[0].first, distances[1].first, distances[2].first, 
                        distances[3].first, distances[4].first, distances[5].first, distances[6].first);

    fclose(file);
    fclose(file2);

    #endif

    // find current vector (by index) and future vector (by time)
    int indexFirst = distances[lowestDistanceIndex].second.first;
    printf("indexFirst: %d\n", indexFirst);
    std::vector<float> currentVector1 = distances[lowestDistanceIndex].second.second.evaluateVectorAt(indexFirst);
    float deltaTime = 0.333333;
    float nextTimeStep = currentVector1[3] + deltaTime;
    printf("time %f, delta %f\n", currentVector1[3], deltaTime);
    std::vector<float> futureVector1 = distances[lowestDistanceIndex].second.second.evaluateVectorAtTime(nextTimeStep);

    int indexSecond = distances[secondLowestDistanceIndex].second.first;
    std::vector<float> currentVector2 = distances[secondLowestDistanceIndex].second.second.evaluateVectorAt(indexSecond);
    float nextTimeStep2 = currentVector2[3] + deltaTime;
    std::vector<float> futureVector2 = distances[secondLowestDistanceIndex].second.second.evaluateVectorAtTime(nextTimeStep2);

    std::vector<std::vector<float>> nearestVectors;
    nearestVectors.push_back(currentVector1);
    nearestVectors.push_back(futureVector1);
    nearestVectors.push_back(currentVector2);
    nearestVectors.push_back(futureVector2);

    #ifdef LOGON

    FILE* file3 = fopen("predictedValues.txt", "a+");
    if (!file3) {
        fprintf(stderr, "Error opening predictedValues.txt...exiting\n");
        exit(1);
    }

    fprintf(file3, "%f,%f,%f,%f,", currentVector1[0], currentVector1[1], currentVector1[2], currentVector1[3]);
    fprintf(file3, "%f,%f,%f,%f,", futureVector1[0], futureVector1[1], futureVector1[2], futureVector1[3]);
    fprintf(file3, "%f,%f,%f,%f,", currentVector2[0], currentVector2[1], currentVector2[2], currentVector2[3]);
    fprintf(file3, "%f,%f,%f,%f\n", futureVector2[0], futureVector2[1], futureVector2[2], futureVector2[3]);

    fclose(file3);

    #endif

    return nearestVectors;
}

/**
 * @brief Interpolates between two values based on a given x value
 */
// float HALO::interpolate(float x, float scenario1Distance, float scenario2Distance) {
//     // Get gains for scenarios
//     std::vector<float> gains = getGains(x, scenario1Distance, scenario2Distance);
//
//     double gain1 = gains[0];
//     double gain2 = 1.0 - gain1;
//
//     return gain1 * scenario1Distance + gain2 * scenario2Distance;
// }

/**
 * @brief Get gains for scenarios
 */
// std::vector<float> HALO::getGains(float x, float scenario1Distance, float scenario2Distance) {
//     float gain1 = 1.0 / std::abs(x - scenario1Distance);
//     float gain2 = 1.0 - gain1;
//     this->scenarioWeights = {gain1, gain2};
//     return {gain1, gain2};
// }

/**
 * @brief Interpolates between two scenarios based on the gains
 */
// float interpolateWithgains(float gain1, float gain2, float scenario1Distance, float scenario2Distance) {
//     return gain1 * scenario1Distance + gain2 * scenario2Distance;
// }

/**
 * @brief Predicts the next values based on the interpolated scenarios
 */
VectorXf HALO::predictNextValues(std::vector<std::vector<float>> &vectors, VectorXf &X_in){

    std::vector<float> gainV1 = {0, 0, 0};
    std::vector<float> gainV2 = {0, 0, 0};
    // vectors = vector1, vector1Future, vector2, vector2Future
    std::vector<float> vector1 = vectors[0];
    std::vector<float> vector1Future = vectors[1];

    std::vector<float> vector2 = vectors[2];
    std::vector<float> vector2Future = vectors[3];
    bool both = false;
    bool vector1Further = false;
    
    for(int i = 0; i < 3; i++){
        printf("[%d], v1 (%f), v2 (%f)\n", i, vector1[i], vector2[i]);

        float distance = std::abs(vector1[i] - vector2[i]);

        if(distance < 1){
            gainV1[i] = 0.5;
            gainV2[i] = 0.5;
            printf("similar vectors gains default");
        }else{
            if(vector1[i] > X_in(i)){
                printf("---below v1----\n");
                // below vector1
                if(vector2[i] > X_in(i)){
                    // point below both lines
                    both = true;
                    if(vector1[i] > vector2[i]){
                        // vector1 on top
                        printf("point below both, v1 on top\n\n");
                        vector1Further = true;
                    }else{
                        vector1Further = false;
                        printf("point below both, v2 on top\n\n");
                    }
                }else{
                    // point between lines
                    printf("point between lines\n\n");

                }
            }else{
                if(vector2[i] > X_in(i)){
                    // point between lines
                    printf("point between\n");
                }else{
                    // point above both lines
                    if(vector1[i] < vector2[i]){
                        vector1Further = true;
                        printf("point above both lines, vector1 further\n\n");
                    }else{
                        vector1Further = false;
                        printf("point above both lines, vector2 further\n\n");
                    }
                    both = true;
                }
            }

            // point below both lines -----------------------------------------------------------
            if(both){
                float longestDistance = 0;
                float distance1 = 0;

                if(vector1Further){
                    longestDistance = std::abs(vector1[i] - X_in(i));
                    distance1 = std::abs(vector2[i] - X_in(i));
                }else{
                    longestDistance = std::abs(vector2[i] - X_in(i));
                    distance1 = std::abs(vector1[i] - X_in(i));
                }

                float relativeFactor = longestDistance / distance1;

                if(!vector1Further){
                    gainV1[i] = relativeFactor / (relativeFactor + 1);
                }else{
                    gainV1[i] = 1 - (relativeFactor / (relativeFactor + 1));
                }

                printf("gainV1[%d]: %f\n", i, gainV1[i]);
                gainV2[i] = 1 - gainV1[i];
            }else{
                // point between lines ------------------------------------------------------------
                printf("distance: %f\n", distance);
                gainV1[i] = 1 - (std::abs(vector1[i] - X_in(i))/distance);  // get distance between vector1 and current state
                // printf("gainV1[%d]: %f\n", i, gainV1[i]);
                gainV2[i] = 1 - gainV1[i];
                //--------------------------------------------------------------------------------
            }
        }
    }

    if(this->firstTimeForPoint == 1){
        FILE* file = fopen("log.txt", "a+");

        if (!file) {
            fprintf(stderr, "Error opening log.txt...exiting\n");
            exit(1);
        }

        fprintf(file, "First time for point (%f, %f, %f)\n", X_in(0), X_in(1), X_in(2));

        fclose(file);

        this->prevGain1 = gainV1;
        this->prevGain2 = gainV2;
        this->firstTimeForPoint = 0;
    }

    printf("prevGain1 (%f,%f,%f), gainV1 (%f,%f,%f)\n", this->prevGain1[0], this->prevGain1[1], this->prevGain1[2], gainV1[0], gainV1[1], gainV1[2]);
    printf("prevGain1 (%f,%f,%f), gainV2 (%f,%f,%f)\n", this->prevGain2[0], this->prevGain2[1], this->prevGain2[2], gainV2[0], gainV2[1], gainV2[2]);

    #ifdef LOGON

    FILE* file = fopen("gains.txt", "a+");
    if (!file) {
        fprintf(stderr, "Error opening gains.txt...exiting\n");
        exit(1);
    }

    fprintf(file, "%f, %f, %f,", gainV1[0], gainV1[1], gainV1[2]);
    fprintf(file, "%f, %f, %f", gainV2[0], gainV2[1], gainV2[2]);

    fclose(file);

    #endif

    // interpolate between the two scenarios to get predicted values0
    float predicted_interpolated_alt  = this->prevGain1[0] * vector1Future[0] + this->prevGain2[0] * vector2Future[0];
    float predicted_interpolated_velo = this->prevGain1[1] * vector1Future[1] + this->prevGain2[1] * vector2Future[1];
    float predicted_interpolated_acc  = this->prevGain1[2] * vector1Future[2] + this->prevGain2[2] * vector2Future[2];

    this->prevGain1 = gainV1;
    this->prevGain2 = gainV2;

    VectorXf X_pred(3,1);
    X_pred << predicted_interpolated_alt, predicted_interpolated_velo, predicted_interpolated_acc;

    return X_pred;
}

/**
 * @brief Check if the rocket is before apogee, based on Everest filter values
 */
bool HALO::isBeforeApogee(float acceleration, float velocity, float altitude, float lastAltitude){

    if(acceleration < -9.81 || velocity < 0.5 || altitude < lastAltitude){
        printf("Apogee at %f\n", altitude);

        FILE* file = fopen("log.txt", "a+");
        if (!file) {
            fprintf(stderr, "Error opening log.txt...exiting\n");
            exit(1);
        }

        fprintf(file, "Apogee at %f\n", altitude);

        return false;
    }

    return true;
}

/**
 * @brief Take the filtered values from Everest filter
*/
void HALO::setStateVector(float filteredAcc, float filteredVelo, float filteredAlt){
    this->Uaccel = filteredAcc;
    this->Uvelo = filteredVelo;
    this->Ualt = filteredAlt;

    VectorXf X_in(3);
    X_in << this->Uaccel, this->Uvelo, this->Ualt;

    /** X_in = [acceleration, velocity, altitude] */
    this->X = X_in;

    // std::cout << "X from Everest: \n" << this->X << std::endl;

    this->stateUpdate();
}

void HALO::overrideStateWithGPS(float GPS){
    float lowest = std::numeric_limits<float>::max();
    float highest = std::numeric_limits<float>::min();

    for(int i = 0; i <= 7; i++){
        if(this->sigmaPoints(0, i) < lowest){
            lowest = this->sigmaPoints(0, i);
        }

        if(this->sigmaPoints(0, i) > highest){
            highest = this->sigmaPoints(0, i);
        }
    }

    if(GPS > (lowest) && GPS < highest){
        this->X[0] = GPS;
        printf("Override GPS (%f, %f, %f)", this->X[0], this->X[1], this->X[2]);

        FILE* file = fopen("log.txt", "a+");
        if (!file) {
            fprintf(stderr, "Error opening log.txt...exiting\n");
            exit(1);
        }
        
        fprintf(file, "Override GPS (%f, %f, %f), where GPS(\n", this->X[0], this->X[1], this->X[2]);

        fclose(file);
    }
}

// prediction step based on the dynamic model
VectorXf HALO::dynamicModel(VectorXf &X){
    // X = [acceleration, velocity, altitude]
    VectorXf Xprediction(3, 1);

    // for every scenario get lists and find nearest 2 vectors to the current state
    std::vector<Scenario> scenarios = this->getScenarios();

    // for(Scenario scenario : scenarios){
    //     printf("Scenario\n");
    //     for(std::vector<float> vec : scenario.getLists()){
    //         for(float value : vec){
    //             printf("%f ", value);
    //         }
    //         printf("\n");
    //     }
    // }

    if(std::isnan(X(0)) || std::isnan(X(1)) || std::isnan(X(2))){
        FILE* file = fopen("log.txt", "a+");
        fprintf(file, "At %f X is nan, defaulting to static integration\n", this->time);
        fclose(file);


        printf("X is nan, defaulting to static integration\n");

        double finalVelocity = X(1) + X(0) * deltaTime;
        double altitude = X(2) + (X(1) + finalVelocity) * deltaTime / 2.0;

        Xprediction(0) = altitude;
        Xprediction(1) = finalVelocity;
        Xprediction(2) = X(0);

        return Xprediction;
    }

    std::vector<std::vector<float>> nearestVectors = this->findNearestScenarios(scenarios, X);

    std::vector<float> vector1 = nearestVectors[0];
    std::vector<float> vector2 = nearestVectors[1];
    std::vector<float> vector3 = nearestVectors[2];
    std::vector<float> vector4 = nearestVectors[3];

    printf("vector1 (%f,%f,%f)\n", vector1[0], vector1[1], vector1[2]);
    printf("futureV1 (%f,%f,%f)\n", vector2[0], vector2[1], vector2[2]);
    printf("vector2 (%f,%f,%f)\n", vector3[0], vector3[1], vector3[2]);
    printf("futureV2 (%f,%f,%f)\n", vector4[0], vector4[1], vector4[2]);

    printf("Spoint being propagated X (%f,%f,%f)\n", X(0), X(1), X(2));

    Xprediction = predictNextValues(nearestVectors, X);

    printf("\nXPrediction of Model: (%f,%f,%f)\n\n", Xprediction(0), Xprediction(1), Xprediction(2));

    return Xprediction;
}

// int main(){
//     // only able to measure angle and extrapolate for velocity
//     MatrixXf X0(6, 1);
//     X0 << 400, 0, 0, -300, 0, 0;

//     int deltaT = 1;

//     MatrixXf F(6 , 6);
//     F << 1, deltaT, 0.5 * std::pow(deltaT,2), 0, 0, 0,
//          0, 1, deltaT, 0, 0, 0,
//          0, 0, 1, 0, 0, 0,
//          0, 0, 0, 1, deltaT, 0.5 * std::pow(deltaT, 2),
//          0, 0, 0, 0, 1, deltaT,
//          0, 0, 0, 0, 0, 1;

//     MatrixXf X(2, 35);
//     X << 502.55, 477.34, 457.21, 442.94, 427.27, 406.05, 400.73, 377.32, 360.27, 345.93, 333.34, 328.07, 315.48,
//                             301.41, 302.87, 304.25, 294.46, 294.29, 299.38, 299.37, 300.68, 304.1, 301.96, 300.3, 301.9, 296.7, 297.07,
//                             295.29, 296.31, 300.62, 292.3, 298.11, 298.07, 298.92, 298.04,

//         -0.9316, -0.8977, -0.8512, -0.8114, -0.7853, -0.7392, -0.7052, -0.6478, -0.59, -0.5183, -0.4698, -0.3952, -0.3026,
//                             -0.2445, -0.1626, -0.0937, 0.0085, 0.0856, 0.1675, 0.2467, 0.329, 0.4149, 0.504, 0.5934, 0.667, 0.7537, 0.8354,
//                             0.9195, 1.0039, 1.0923, 1.1546, 1.2564, 1.3274, 1.409, 1.5011;

//     MatrixXf P(6, 6);
//     P <<500, 0, 0, 0, 0, 0,
//         0, 500, 0, 0, 0, 0,
//         0, 0, 500, 0, 0, 0,
//         0, 0, 0, 500, 0, 0,
//         0, 0, 0, 0, 500, 0,
//         0, 0, 0, 0, 0, 500;


//     VectorXf Z_in(2,1);
//     Z_in << 0, 0;

//     MatrixXf Q(6,6);
//     Q << 0.25, 0.5, 0.5, 0, 0, 0,
//             0.5, 1, 1, 0, 0, 0,
//             0.5, 1, 1, 0, 0, 0,
//             0, 0, 0, 0.25, 0.5, 0.5,
//             0, 0, 0, 0.5, 1, 1,
//             0, 0, 0, 0.5, 1, 1;

//     Q = Q * std::pow(0.2, 2);

//     std::cout << "Q:\n" << Q << std::endl;

//     std::cout << "X:\n" << X << std::endl;

//     // Open a file for writing
//     std::ofstream outFile("filtered_values_2D_HALO.csv");

//     // Check if the file is open
//     if (!outFile.is_open()) {
//         std::cerr << "Failed to open filtered_values_2D.csv" << std::endl;
//         return 1;
//     }

//     // Write the header to the file
//     outFile << "Time,X,Y,XVelo,YVelo\n";

//     HALO halo = HALO();

//     halo.init(X0, P, Q, Z_in, F);

//     VectorXf X1(2,1);
//     halo.X = X1;

//     for(int i = 0; i < 35; i++){
//         std::cout << "\n\nIteration: " << i + 1 << std::endl;

//         halo.X << X(0, i), X(1, i);

//         halo.stateUpdate();

//         // Write the filtered values to the file
//         outFile << i << "," << halo.X0(0) << "," << halo.X0(3) << "," << halo.X0(1) << "," << halo.X0(4) << "\n";
//     }

//     // Close the file
//     outFile.close();

//     return 0;

// }

#endif