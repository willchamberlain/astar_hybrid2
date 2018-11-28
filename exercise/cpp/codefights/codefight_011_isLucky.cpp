#include <iostream>
#include <algorithm>

bool isLucky(int n) {
     
    while(0 < n/10)  {
        num_ = n%10;
        n = n/10;
    }
    return false;
}

int main(int argc, char const *argv[])
{
    std::cout << isLucky(1230) << std::endl;
    std::cout << isLucky(239017) << std::endl;

    int num = 239017;
    num=2;
    std::cout << num/10 << "  ,  " << num%10 << std::endl;
    
    return 0;
}
