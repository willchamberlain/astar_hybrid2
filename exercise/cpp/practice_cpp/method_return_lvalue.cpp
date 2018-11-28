#include <iostream>
using std::cout;
using std::endl;


int someGlobal;

class DogWithStatic {
    private:
        static int theStatic;
        int theLocal;
    public:
        DogWithStatic(const int initialLocal) {
            theLocal = initialLocal;
        }

        int& getTheStatic() const {
            return theStatic;
        }

        int& getTheGlobal() {
            // int someGlobalwas = someGlobal;  // can't return reference to function-local variable
            someGlobal = theLocal;
            // return someGlobalwas;  // can't return reference to function-local variable
            return someGlobal;
        }

        void print()  {
            // cout << "DogWithStatic: theStatic=" << DogWithStatic::theStatic << ", theLocal=" << theLocal << endl;
            cout << "DogWithStatic: theStatic=" << ", theLocal=" << theLocal << endl;
        }
};

int main(int argc, char const *argv[])
{
    DogWithStatic d_01(1);  d_01.print();  cout << d_01.getTheGlobal() << endl;  // cout << d_01.getTheStatic() << endl;
    DogWithStatic d_10(10); d_10.print();  cout << d_10.getTheGlobal() << endl;
        d_10.getTheGlobal() = 10000;  cout << someGlobal << endl;
        cout << d_10.getTheGlobal() << endl;
    DogWithStatic d_15(15); d_15.print();  cout << d_15.getTheGlobal() << endl;
    
    return 0;
}
