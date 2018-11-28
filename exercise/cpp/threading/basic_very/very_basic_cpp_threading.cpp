#include <iostream>
#include <thread>

void worker(int a_number)
{
    std::cout << "\n another thread : num=" << a_number << std::endl;
}
void worker2()
{
    std::cout << "\n another thread worker2" << std::endl;
}
void worker3()
{
    std::cout << "\n another thread worker3" << std::endl;
}

int main()
{
    std::thread t2(worker2);
    std::thread t(worker,1);
    std::cout << "\n main thread" << std::endl;
    std::thread t3(worker3);
    t.join();
    t3.join();
    t2.join();
    return 0;
}