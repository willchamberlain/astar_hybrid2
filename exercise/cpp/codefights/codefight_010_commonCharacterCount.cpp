#include <iostream>
#include <string>
#include <algorithm>
#include <set>
#include <iterator>

int commonCharacterCount2(std::string s1, std::string s2) {
    std::sort(s1.begin(),s1.end());
    std::unique(s1.begin(),s1.end());
    std::sort(s2.begin(),s2.end());
    std::unique(s2.begin(),s2.end());

    std::set<char> s1set(s1.begin(), s1.end());
    // std::pair<std::iterator, bool> did_it_work;
    std::string in_common = "";
    int num_common_chars = 0;
    for(int ii_=0; ii_<s2.size(); ii_++) {
        auto did_it_work = s1set.insert(s2[ii_]);
        if(!did_it_work.second){
            // in_common.append(s2[ii_]);
            num_common_chars++;
        }
    }

    return num_common_chars;
}


int commonCharacterCount(std::string s1, std::string s2) {
    int all_chars[256];
    std::fill_n(all_chars,256,0);
    for(int ii_=0; ii_<s1.length(); ii_++) {
        // std::cout << s1[ii_] << std::endl;
        // std::cout << (int)s1[ii_] << std::endl;
        all_chars[(int)s1[ii_]]++;
    }
    int all_chars2[256];
    std::fill_n(all_chars2,256,0);
    for(int ii_=0; ii_<s2.length(); ii_++) {
        // std::cout << s2[ii_] << std::endl;
        // std::cout << (int)s2[ii_] << std::endl;
        all_chars2[(int)s2[ii_]]++;
    }


    // std::cout << "all_chars:   ";
    for(int ii_=0; ii_< sizeof(all_chars)/sizeof(*all_chars); ii_++) {
        // std::cout << all_chars[ii_] << ",";
    }
    // std::cout << std::endl;
    // std::cout << "all_chars2:  ";
    for(int ii_=0; ii_< sizeof(all_chars)/sizeof(*all_chars); ii_++) {
        // std::cout << all_chars2[ii_] << ",";
    }
    // std::cout << std::endl;

    int all_chars_min[256];
    std::fill_n(all_chars_min,256,0);
    // std::cout << "all_chars_min:  ";
    for(int ii_=0; ii_< sizeof(all_chars)/sizeof(*all_chars); ii_++) {
        all_chars_min[ii_] = std::min(all_chars[ii_],all_chars2[ii_]);
        // std::cout << all_chars_min[ii_] << ",";
    }
    // std::cout << std::endl;

    int num_chars_in_common = 0;
    for(int ii_=0; ii_< sizeof(all_chars)/sizeof(*all_chars); ii_++) {
        // if(all_chars_min[ii_]>0) {
            num_chars_in_common+=all_chars_min[ii_];
        // }
    }
    

    return num_chars_in_common ;
}

int main(int argc, char const *argv[])
{

    std::string s1 = "aabcc";  std::string s2 = "adcaa";
    // std::cout << commonCharacterCount(s1,s2) << std::endl;
    std::sort(s1.begin(),s1.end()); // std::cout<< s1 <<std::endl;
    std::sort(s2.begin(),s2.end()); // std::cout<< s2 <<std::endl;
    // std::unique_copy(s2.begin(),s2.end()); // std::cout<< s2 <<std::endl;

    s1= "abca";
    s2= "xyzbac";
    // std::cout << commonCharacterCount(s1,s2) << std::endl;

    s1= "zzzz";
    s2= "zzzzzzz";
    // std::cout << commonCharacterCount(s1,s2) << std::endl;

    return 0;
}
