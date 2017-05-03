#include <iostream>
#include "GraphColoringModule.h"

int main() {

    GraphColoringModule gcm{};
    gcm.solve("../dataFile1");
    gcm.solve("../dataFile2");


    std::cout << "\nFinished..." << std::endl;
    return 0;
}