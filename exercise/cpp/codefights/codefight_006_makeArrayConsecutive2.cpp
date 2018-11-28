#include <iostream>
#include <vector>
#include <algorithm>

void print(std::vector<int> statues) {
    for(auto& elem : statues) {
        std::cout << elem << "," ;
    }
    std::cout << std::endl;
}

int makeArrayConsecutive2(std::vector<int> statues) {
    int num_statues_required = 0;
    print(statues);
    std::sort(statues.begin(), statues.end());
    print(statues);
    for(int ii_ = 0; ii_<statues.size()-1; ii_++) {
        int diff = std::abs(statues[ii_] - statues[ii_+1]);
        if(diff > 1) {
            num_statues_required += diff-1;
        }
    }
    // std::for_each(statues.begin(), statues.end()-1, [](int i) {
    //     std::cout << i << "," ;
    // }
    // std::cout << std::endl;
    return num_statues_required;
}

int main(int argc, char const *argv[])
{
    int expected = 0;
    std::cout << "makeArrayConsecutive2: expect " << expected << ", got " << makeArrayConsecutive2(std::vector<int> {1,2,3,4} ) << std::endl;
    expected = 0;
    std::cout << "makeArrayConsecutive2: expect " << expected << ", got " << makeArrayConsecutive2(std::vector<int> {4,3,2,1} ) << std::endl;
    expected = 1;
    std::cout << "makeArrayConsecutive2: expect " << expected << ", got " << makeArrayConsecutive2(std::vector<int> {1,2,4} ) << std::endl;
    expected = 1;
    std::cout << "makeArrayConsecutive2: expect " << expected << ", got " << makeArrayConsecutive2(std::vector<int> {1,4,2} ) << std::endl;
    expected = 10;
    std::cout << "makeArrayConsecutive2: expect " << expected << ", got " << makeArrayConsecutive2(std::vector<int> {1,2+1+10,2} ) << std::endl;

    return 0;
}
