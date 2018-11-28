#include <iostream>
#include <vector>
#include <algorithm>

int adjacentElementsProduct(std::vector<int> inputArray) {
    int ans = -987654321;
    for (int i = 1; i < inputArray.size(); ++i)
        ans = std::max(ans, inputArray[i] * inputArray[i - 1]);
    return ans; 
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
    std::cout << "adjacentElementsProduct: " << adjacentElementsProduct(a_vector) << std::endl ;

    
    std::cout << "adjacentElementsProduct: " << adjacentElementsProduct( std::vector<int>{10,20,30} ) << std::endl ;

    std::cout << "adjacentElementsProduct: " << adjacentElementsProduct( std::vector<int>{10,20,40,10,20,-1000,10,20,30} ) << std::endl ;

    std::cout << "adjacentElementsProduct: " << adjacentElementsProduct( std::vector<int>{-23, 4, -3, 8, -12} ) << std::endl ;
    return 0;
}
