#include <iostream>
#include <rigtorp/SPSCQueue.h>
#include <thread>

int main(int argc, char *argv[]) {
  (void)argc, (void)argv;

  using namespace rigtorp;

  SPSCQueue<int> q(1001);

  auto t = std::thread([&] {
      while(true)
      {
          if (q.front())
          {
              std::cout << *q.front() << std::endl;
              q.pop();
          }
      }
  });

  auto t1 = std::thread([&]
  {
      for (int i = 0; i < 200000; ++i)
      {
          q.push(i);
      }
  });

  t.join();
  t1.join();

  return 0;
}
