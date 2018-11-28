#include<iostream>
//#include<conio>

void swap_by_value_doesnt_work(int a, int b)
{
 int temp;
 temp=a;
 a=b;
 b=temp;
}

void swap_by_ref(int &a, int &b)
{
 int temp;
 temp=a;
 a=b;
 b=temp;
}

void swap_by_ptr_1(int *a, int *b)
{
 int *temp;
    std::cout << "a=" << a << ", b=" << b<< ", temp=" << temp  << std::endl;
 temp=a;
    std::cout << "a=" << a << ", b=" << b<< ", temp=" << temp  << std::endl;
 a=b;
    std::cout << "a=" << a << ", b=" << b << ", temp=" << temp << std::endl;
 b=temp;
    std::cout << "a=" << a << ", b=" << b<< ", temp=" << temp  << std::endl;
}

void swap_by_ptr_2(int *a, int *b)
{
 int temp;
 temp=*a;
 *a=*b;
 *b=temp;
}



int main()
{
 int a=100, b=200;
 //clrscr();
 swap_by_value_doesnt_work(a, b);  // passing value to function
 std::cout<<"Value of a = "<<a<<std::endl;
 std::cout<<"Value of b = "<<b<<std::endl;
 std::cout<<"------------"<<std::endl;
 
 a=100, b=200;
 swap_by_ref(a, b);  // passing refs to function
 std::cout<<"Value of a = "<<a<<std::endl;
 std::cout<<"Value of b = "<<b<<std::endl;
 std::cout<<"------------"<<std::endl;
 
 a=100, b=200;
 int *a_ptr=&a, *b_ptr=&b ;
 swap_by_ptr_1(a_ptr, b_ptr);  // passing pointers to function to change what the pointers point to
 std::cout<<"Value of a = "<<a<<std::endl;
 std::cout<<"Value of b = "<<b<<std::endl;
 std::cout<<"Value of *a_ptr = "<<*a_ptr<<std::endl;
 std::cout<<"Value of *b_ptr = "<<*b_ptr<<std::endl;
    std::cout << "a_ptr=" << a_ptr << ", b_ptr=" << b_ptr << std::endl;
 std::cout<<"------------"<<std::endl;
 
 a=100, b=200;
 a_ptr=&a; b_ptr=&b ;
 swap_by_ptr_2(a_ptr, b_ptr);  // passing pointers to function to change what the pointers point to
 std::cout<<"Value of a = "<<a<<std::endl;
 std::cout<<"Value of b = "<<b<<std::endl;
 std::cout<<"Value of *a_ptr = "<<*a_ptr<<std::endl;
 std::cout<<"Value of *b_ptr = "<<*b_ptr<<std::endl;
 std::cout<<"------------"<<std::endl;

 a=100, b=200;
 a_ptr=&a; b_ptr=&b ;
 swap_by_ptr_2(&a, &b);  // passing address-of to function to change what is at those addresses / what the 'pointers' point to 
 std::cout<<"Value of a = "<<a<<std::endl;
 std::cout<<"Value of b = "<<b<<std::endl;
 std::cout<<"Value of *a_ptr = "<<*a_ptr<<std::endl;
 std::cout<<"Value of *b_ptr = "<<*b_ptr<<std::endl;
 std::cout<<"------------"<<std::endl;
 //getch();
 return 0;
}