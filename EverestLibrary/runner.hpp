#include "everest.hpp"

class Runner {
private:
    Everest* everestInstance;

public:
    Runner() {
        everestInstance = new Everest();
    }

    ~Runner() {
        delete everestInstance;
    }

    Everest* getThat() {
        return everestInstance;
    }

    // ...
};