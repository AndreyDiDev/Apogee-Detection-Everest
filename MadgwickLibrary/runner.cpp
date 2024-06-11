#include "everest.hpp"


int main() {
    kinematics kin;
    systemState sys;
    altitudeList alt;

    printf("Hello, World!\n");

    Everest everest = Everest::getEverest();
    everest.MadgwickSetup();



    return 0;
}