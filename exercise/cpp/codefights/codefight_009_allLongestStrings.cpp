#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

std::vector<std::string> allLongestStrings(std::vector<std::string> inputArray) {
    int stringLengths[inputArray.size()];
    int maxLength = -1;
    std::vector<std::string> longestStrings;
    for(int ii_ = 0; ii_<inputArray.size(); ii_++ ) {
        stringLengths[ii_] = inputArray[ii_].size();
        if(maxLength<stringLengths[ii_]) {
            maxLength=stringLengths[ii_];
            longestStrings.clear();
            longestStrings.push_back(inputArray[ii_]);
        } else if(maxLength==stringLengths[ii_]){
            longestStrings.push_back(inputArray[ii_]);
        }
    }
    return longestStrings;
}


int main(int argc, char const *argv[])
{
    int anarray[]={1,2,3,4,12,1};

    std::vector<std::string> inputArray = {"aba", "aa", "ad", "vcd", "aba"};
    std::vector<std::string> outputArray = allLongestStrings(inputArray);
    for(int ii_ = 0; ii_<outputArray.size(); ii_++ ) {
        std::cout<<outputArray[ii_]<<",";
    }
    std::cout << std::endl;
    inputArray = {"aba", "zzzzz", "ad", "zzzzz", "aba"};outputArray = allLongestStrings(inputArray);
    for(int ii_ = 0; ii_<outputArray.size(); ii_++ ) {
        std::cout<<outputArray[ii_]<<",";
    }
    std::cout << std::endl;
    inputArray = {"aba", "zzzzz", "aaaaaa", "zzzzz", "aba"};outputArray = allLongestStrings(inputArray);
    for(int ii_ = 0; ii_<outputArray.size(); ii_++ ) {
        std::cout<<outputArray[ii_]<<",";
    }
    std::cout << std::endl;
    inputArray = {"aba", "zzzzz", "aaaaaa", "zzzzz", "99999999"};outputArray = allLongestStrings(inputArray);
    for(int ii_ = 0; ii_<outputArray.size(); ii_++ ) {
        std::cout<<outputArray[ii_]<<",";
    }
    std::cout << std::endl;
    inputArray = {"111111111111111", "zzzzz", "aaaaaa", "zzzzz", "aba"};outputArray = allLongestStrings(inputArray);
    for(int ii_ = 0; ii_<outputArray.size(); ii_++ ) {
        std::cout<<outputArray[ii_]<<",";
    }
    std::cout << std::endl;
    return 0;
}
