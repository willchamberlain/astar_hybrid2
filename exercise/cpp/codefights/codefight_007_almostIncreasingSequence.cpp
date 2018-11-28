
/*  Given a sequence of integers as an array, determine whether it is possible to obtain a strictly increasing sequence by removing no more than one element from the array.

Example
    For sequence = [1, 3, 2, 1], the output should be
        almostIncreasingSequence(sequence) = false.
    There is no one element in this array that can be removed in order to get a strictly increasing sequence.
    For sequence = [1, 3, 2], the output should be
        almostIncreasingSequence(sequence) = true.
    You can 
        remove 3 from the array to get the strictly increasing sequence [1, 2]. Alternately, you can 
        remove 2 to get the strictly increasing sequence [1, 3].
Input/Output
    [execution time limit] 0.5 seconds (cpp)
    [input] array.integer sequence
    Guaranteed constraints:
        2 ≤ sequence.length ≤ 105,
        -105 ≤ sequence[i] ≤ 105.
    [output] boolean
        Return true if it is possible to remove one element from the array in order to get a strictly increasing sequence, otherwise return false.
*/

#include <iostream>
#include <vector>

int max_allowed_skips() {
    return 1;
}

int max_allowed_bad() {
    return 0;
}

bool num_skips_is_OK(int num_skips, int num_bad) {
    return num_skips <= max_allowed_skips() && num_bad <= max_allowed_bad();
}

void print(std::vector<int> statues) {
    for(auto& elem : statues) {
        std::cout << elem << "," ;
    }
    //std::cout << std::endl;
}





bool almostIncreasingSequence(std::vector<int> sequence) {
    int good=0, num_skipped=0, num_bad=0;
    for(int ii_=0;ii_<sequence.size()-1;) {
        if(sequence[ii_+1] > sequence[ii_]) {
            good++;
            ii_+=1;
        } else if(ii_>0 && sequence[ii_+1] > sequence[ii_-1]) {
            num_skipped++;
            ii_+=1;
        } else if(ii_==0) {
            num_skipped++;
            ii_+=1;
        } else if(ii_==sequence.size()-2) {
            num_skipped++;
            ii_+=1;
        } else  {
            if(ii_+2 < sequence.size()) {
                if(sequence[ii_+2] > sequence[ii_]) {
                    num_skipped++;
                    ii_+=2;
                } else {
                    num_bad++;
                    ii_+=1;
                }
            } else {
                    num_bad++;
                    ii_+=1;
            }
        }        
    }
    std::cout<<num_skips_is_OK(num_skipped, num_bad)<<":  "; print(sequence);  std::cout<<":    ";
    return num_skips_is_OK(num_skipped, num_bad);
}


int main(int argc, char const *argv[])
{
    
    std::cout << almostIncreasingSequence(std::vector<int>{1,2,3})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{1,3,5})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{10,12,13})  << std::endl;
    std::cout << "---------------"  << std::endl;

    std::cout << almostIncreasingSequence(std::vector<int>{3,2,1})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{5,3,1})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{10,1,-10})  << std::endl;
    std::cout << "---------------"  << std::endl;
    
    std::cout << almostIncreasingSequence(std::vector<int>{1,1,2,3})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{1,1,3,5})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{10,1,12,13})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{10,12,1,13})  << std::endl;
    std::cout << "---------------"  << std::endl;

    std::cout << almostIncreasingSequence(std::vector<int>{1,2,1,3})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{1,3,1,5})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{10,12,1,13})  << std::endl;
    std::cout << "---------------"  << std::endl;

    std::cout << almostIncreasingSequence(std::vector<int>{-3,-2,-1})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{-30,-20,-10})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{-30,-40,-20,-10})  << std::endl;
    std::cout << "---------------"  << std::endl;

    std::cout << almostIncreasingSequence(std::vector<int>{-30,40,-20,-10})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{-30,-20,-40,-10})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{-30,-20,40,-10})  << std::endl;
    std::cout << "---------------"  << std::endl;

    std::cout << almostIncreasingSequence(std::vector<int>{-30,-40,-20,-40,-10})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{1, 3, 2, 1})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{-30,40,-20,-10})  << std::endl;
    std::cout << "---------------"  << std::endl;

    std::cout << almostIncreasingSequence(std::vector<int>{123, -17, -5, 1, 2, 3, 12, 43, 45})  << std::endl;
    std::cout << almostIncreasingSequence(std::vector<int>{3, 5, 67, 98, 3})  << std::endl;
    
    std::cout << "---------------"  << std::endl;
    


    return 0;
}





bool almostIncreasingSequence1(std::vector<int> sequence) {
    bool debug = true;
    if(debug){print(sequence);}
    int num_skips = 0, num_bad = 0;
    // if sorted(sequence) == sequence return true
    for(int ii_=0 ; ii_<sequence.size()-1 ; ii_++) {
        if( sequence[ii_] < sequence[ii_+1] ) {
            if(debug){std::cout << "good: "<<ii_<<" :" << sequence[ii_] << " , " << sequence[ii_+1] << std::endl;}
            continue;
        } 
        if( ii_ < sequence.size()-2) {
            if( sequence[ii_] < sequence[ii_+2] ) {
                num_skips++;
                if(debug){std::cout << "num_skips++ 1: "<<ii_<<" :" << sequence[ii_] << " , " << sequence[ii_+2] << std::endl;}
            } else {
                num_bad++;
                if(debug){std::cout << "num_bad++ 1: "<<ii_<<" :" << sequence[ii_] << " , " << sequence[ii_+2] << std::endl;}
            }
        } else {
            if(sequence.size()>2 && sequence[ii_-1] < sequence[ii_+1]) {
                num_skips++;
                if(debug){std::cout << "num_skips++ 2: "<<ii_<<" :" << sequence[ii_] << " , " << sequence[ii_+2] << std::endl;}
            }
            else {
                num_bad++;
                if(debug){std::cout << "num_bad++ 2: "<<ii_<<" :" << sequence[ii_] << " , " << sequence[ii_+2] << std::endl;}
            }
        }
    }
    // if(num_skips>max_allowed_skips()) { return false; }
    // return true;
    return num_skips_is_OK(num_skips, num_bad);
}

bool almostIncreasingSequence2(std::vector<int> sequence) {
    bool debug = true;
    if(debug){std::cout<<"-------------"<<std::endl;print(sequence);}
    int num_skips = 0, num_bad = 0;
    // if sorted(sequence) == sequence return true
    for(int ii_=0 ; ii_<sequence.size()-1 ; ii_++) {
        if( sequence[ii_] < sequence[ii_+1] ) {
            if(debug){std::cout << "good        : "<<ii_<<" :" << sequence[ii_] << " , " << sequence[ii_-1] << std::endl;}
            continue;
        } 
        if( ii_ < sequence.size()-2) {
            if( sequence[ii_] < sequence[ii_+2] ) {
                num_skips++;
                ii_++;
                if(debug){std::cout << "num_skips++ 1: "<<ii_<<" :" << sequence[ii_] << " , " << sequence[ii_-2] << std::endl;}
            } else {
                num_bad++;
                if(debug){std::cout << "num_bad++ 1 : "<<ii_<<" :" << sequence[ii_] << " , " << sequence[ii_-2] << std::endl;}
            }
        } else {
            num_bad++;
            if(debug){std::cout << "num_bad++ 2 : "<<ii_<<" :" << sequence[ii_] << " , " << sequence[ii_-1] << std::endl;}
        }
    }
    // if(num_skips>max_allowed_skips()) { return false; }
    // return true;
    return num_skips_is_OK(num_skips, num_bad);
}