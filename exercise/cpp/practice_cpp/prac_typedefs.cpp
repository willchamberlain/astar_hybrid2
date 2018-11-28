#include "utils_1.cpp"

// typedef int[4] int_array;
typedef int int_array_4t[4];
using int_array_4u = int[4];



int main(int argc, char const *argv[])
{
    willc::print_willc();
    willc::print_full_name();

    int array_0_plain_int[4] = {2, -2, 4, 9};
    //willc::utils::print_array(array_0_plain_int, array_0_plain_int.size()); // needs to be  #include <array>   std::array<int,4> myints;  myints.size()  or  sizeof(myints) 
    willc::utils::print_array(array_0_plain_int, sizeof(array_0_plain_int)/sizeof(array_0_plain_int[0]) );

    int_array_4t array_1 = {3, 5, 7, 9} ;
    willc::utils::print_array(array_1, sizeof(array_1)/sizeof(array_1[0]) );

    int_array_4u array_2 = {2, 4, 6, -8}; 
    willc::utils::print_array(array_2, sizeof(array_2)/sizeof(array_2[0]) );


    return 0;
}
