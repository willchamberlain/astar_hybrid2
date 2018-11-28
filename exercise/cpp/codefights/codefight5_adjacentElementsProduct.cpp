#include <iostream>
#include <vector>
#include <algorithm>

static int max_pos_input_value = 1000;
static int max_neg_input_value = -1000;

struct AdjacentElements {
    int product=max_neg_input_value*max_pos_input_value;
    int index_in_list=-1;
};

int adjacentElementsProduct(std::vector<int> inputArray) {
    AdjacentElements highestProductAdjacentElements;
    for (unsigned int ii_ = 0;  ii_<inputArray.size()-1; ii_++) {
        if( inputArray[ii_]*inputArray[ii_+1] > highestProductAdjacentElements.product ) {
            highestProductAdjacentElements.index_in_list = ii_;
            highestProductAdjacentElements.product = inputArray[ii_]*inputArray[ii_+1];
        }
    }
    // std::for_each(inputArray.begin(), inputArray.end()-1, [](int i) {
    // }

    std::cout << "adjacentElementsProduct: " << highestProductAdjacentElements.product << " at " << highestProductAdjacentElements.index_in_list << std::endl;
    return highestProductAdjacentElements.product;
}

int main(int argc, char const *argv[])
{
    std::vector<int> a_vector;
    a_vector.push_back(1);
    a_vector.push_back(10);
    a_vector.push_back(100);
    a_vector.push_back(2);
    a_vector.push_back(20);
    a_vector.push_back(200);
    adjacentElementsProduct(a_vector);

    adjacentElementsProduct( std::vector<int>{10,20,30} );

    adjacentElementsProduct( std::vector<int>{10,20,40,10,20,-1000,10,20,30} );

    adjacentElementsProduct( std::vector<int>{-23, 4, -3, 8, -12} );
    return 0;
}
