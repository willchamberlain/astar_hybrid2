#include <iostream>
#include <tuple>
#include <string>

int main(int argc, char const *argv[])
{
    std::tuple<int, std::string, int> my_tuple;

    // std::cout << my_tuple << std::endl;  // cannot print directly

    std::cout<< "std::get<1>(my_tuple) = '" << std::get<1>(my_tuple) << "'" << std::endl;

    std::get<1>(my_tuple) = "update #1";
    std::cout<< "std::get<1>(my_tuple) = '" << std::get<1>(my_tuple) << "'" << std::endl;

    std::string& s = std::get<1>(my_tuple) ;
    std::cout << "std::string&   s  = '"<< s<<"'"<<std::endl;

    std::string s2 = std::get<1>(my_tuple) ;
    std::cout << "std::string    s2 = '"<< s2 <<"'"<<std::endl;

    // std::string s_update_2 = "update #2 by pointer to string";
    // std::get<1>(my_tuple) = &s_update_2;     // cannot pass pointer into reference argument
    // std::cout<< "std::get<1>(my_tuple) = '" << std::get<1>(my_tuple) << "'" << std::endl;
    
    return 0;
}
