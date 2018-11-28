#include <iostream>
using std::cout;  using std::endl;

void printInt(int a) { cout << a << endl;}
void printIntRef(int& a) { cout << a << endl;}

class Dog {
    private:
        std::string doggy_noise = "doooog";
    public:
        Dog() {
            cout << "I'm a default dog - " << doggy_noise << endl;
        }
        Dog(std::string my_noise_) {
            doggy_noise = my_noise_;
            cout << "I'm a special dog - " << doggy_noise << endl;
        }

        std::string noise() {
            return doggy_noise;
        }
};

void printInt(Dog a) { cout << a.noise() << endl;}
void printIntRef(Dog& a) { cout << "ref - " << a.noise() << endl;}



int main(int argc, char const *argv[])
{
    printInt(4);
    
    int a = 5; // a is lvalue
    printInt(a);
    printIntRef(a);

    Dog doggy("bob");
    printInt(doggy);
    printIntRef(doggy);

    std::string robby = "robby";
    Dog robbydoggy(robby);
    printInt(robbydoggy);
    printIntRef(robbydoggy);


    
    return 0;
}
