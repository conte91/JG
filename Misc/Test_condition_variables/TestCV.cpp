#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

boost::condition_variable cond;
boost::mutex mut;

boost::condition_variable cond1;
boost::mutex mut1;

int count=0;

void incrementCounter(){
  while(1){
    sleep(1);
    count++;
    cond.notify_one();
    sleep(1);
    count++;
    cond1.notify_one();
  }
}

int main(){

  boost::thread t(incrementCounter);

  for(int i=0; i<100; ++i){
    std::cout << "Value of count: " << count << std::endl;
    boost::unique_lock<boost::mutex> lock(mut);
    cond.wait(lock);
    std::cout << "Value of count: " << count << std::endl;
    cond1.wait(lock);
    std::cout << "Value of count: " << count << std::endl;

    cond1.wait(lock);
    std::cout << "Value of count: " << count << std::endl;
    cond.wait(lock);
    std::cout << "Value of count: " << count << std::endl;
  }

  return 0;
}

