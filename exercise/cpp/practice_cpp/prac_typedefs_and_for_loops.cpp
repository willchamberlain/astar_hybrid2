// https://stackoverflow.com/a/44990326/1200764

#include <iostream>
using std::cout; using std::endl;

int main()
{
    // /* same effect as below */ int ia[3][4] = { 10, 11, 12, 13, 24, 25, 26, 27, 33, 34, 35, 36 };
    /* same effect as above */ int ia[3][4] = { {10, 11, 12, 13}, {24, 25, 26, 27}, {33, 34, 35, 36} };
    // Does not compile:  int ia[3][4] = { {10, 11, 12},{13, 24, 25}, {26, 27, 33},{34, 35, 36} };

    // a range for to manage the iteration
    // use type alias
    // using int_array = int[4];
    typedef int int_array[4];
    cout << "\n\n start: count iterations through array with a range  sized via a using/type alias ( i.e. in units of sizeof(int_array) :  the same size as the rows of the 2D array) , and a range within that of ints in the variable of type <type alias> " << "\n";
    int outer_loop_count = 0;
    for (int_array& p : ia) {
        outer_loop_count++;
        cout << "| outer_loop_count=" << outer_loop_count << ":  ";
        for (int q : p) {
            cout << "q=" << q << " ";
        }
        cout << "\n";
    }
    cout << endl;

    // ordinary for loop using subscripts
    cout << "\n\n start: count iterations through array in ordinary for loop using subscripts " << "\n";
    for (size_t i = 0; i != 3; ++i) {
        for (size_t j = 0; j != 4; ++j) {
            cout << "| i,j=" << i << "," << j << ": ia[i][j]=" << ia[i][j] << " ";
        }
        cout << "\n";
    }
    cout << endl;

    // using pointers.
    // use type alias
    cout << "\n\n start: using pointer of <type alias> stepping through up to start row+number of rows-1  :  then by pointer-to-int up from start of <type alias var from outer loop> to start+known number of columns-1"<<"\n";
    for (int_array* p = ia; p != ia + 3; ++p) {
        for (int *q = *p; q != *p + 4; ++q) {
            cout << "| p=" << p << "  ";
            cout << "q=" << q << " ";
            cout << "*q=" << *q << " ";
            cout << "\n";
        }
    }
    cout << endl;

    return 0;
}