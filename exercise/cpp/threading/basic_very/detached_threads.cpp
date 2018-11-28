#include <iostream>       // std::cout
#include <thread>         // std::thread, std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
 
void pause_thread(int n) 
{
  std::this_thread::sleep_for (std::chrono::seconds(n));
  std::cout << "pause of " << n << " seconds ended\n";
}
 
int main() 
{
  std::cout << "Spawning and detaching 3 threads...\n";
  std::thread t1(pause_thread,1);
  t1.detach();
  std::thread (pause_thread,2).detach();
  std::thread t3(pause_thread,3);
  t3.detach();
  std::cout << "Done spawning threads.\n";

    int main_thread_pause = 5;  //  1;

  std::cout << "(the main thread will now pause for "<< main_thread_pause <<" seconds)\n";
  // give the detached threads time to finish (but not guaranteed!):
  pause_thread(main_thread_pause);

  std::cout << "(the main thread finished pausing for "<< main_thread_pause <<" seconds)\n";
  return 0;
}



