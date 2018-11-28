#ifndef willc_utils_utils_1
#define willc_utils_utils_1

#include <iostream>

namespace willc{

    void print_willc () {
        std::cout << "WillC" << std::endl;
    }

    namespace utils{
        void print_array(const int an_array[], const int size) {
            for(int ii_ = 0; ii_ < size ; ii_++) {
                std::cout << an_array[ii_];
                if (ii_ < size-1) {std::cout << ","; }
            }
            std::cout << std::endl;
        }
    }

    void print_full_name() {
        std::cout << "William Chamberlain" << std::endl;
    }

}

#endif