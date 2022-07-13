#ifndef __processess_h_
#define __processess_h_

#include <vector>
#include <queue>
#include <algorithm>

#include "process.h"

template<typename T> void pretty_print(std::queue<T> q);
template<typename T> void pretty_print(std::vector<T> q);
template<typename T> void pretty_print(T q);


class CompareIO {
  public:
  bool operator() ( Process a, Process b) {
    return a.current().second > b.current().second;
  }
};

class CompareCPU {
  public:
  bool operator() ( Process a, Process b) {
    return a.current().first > b.current().first;
  }
};

class Processes {
public:
  Processes( std::vector<Process> processes, int cs ): processes(processes), P(processes), contextSw(cs) { }

  // Process nextProcess() { // returns the next process in the queue
    
  // }

  void start() {
    std::cout << std::endl;
    // int x = 0;
    // std::cout << "time 0ms: Simulator started for FCFS ";
    // printQ(-2);
    // Process p = addNewProcess();
    // time += p.getArrivalTime();

Process p = addNewProcess();
p.next();
CPUworkingQueue.push(p);
IOworkingQueue.push(p);
p = addNewProcess();
p.next();
CPUworkingQueue.push(p);
IOworkingQueue.push(p);
pretty_print(CPUworkingQueue);
pretty_print(IOworkingQueue);


    // while ( !finished( P ) ) {
    // while ( x++ < 5 ) {
    //   // start the burst of the first process
    //   if ( !(Q.empty() )) { Q.pop(); }
    //   p.next();
    //   time += contextSw/2;
    //   std::cout << prefix( time, p.getID() ) << "started using the CPU for " << p.current().first << "ms burst ";
    //   printQ( p.current().first );
    //   time += p.current().first;
    //   p.cpuDone( time );
    //   printQ( 0 );
    //   std::cout << prefix( time, p.getID() ) << "switching out of CPU; will block on I/O until time " << p.current().second + time + contextSw/2 << "ms ";
    //   printQ( p.current().second );
    //   time += contextSw/2;
    //   if ( !(Q.empty() )) {
    //     p = Q.front();
    //   }
    // }
  }
  friend int mian();

private:

  bool finished( std::vector<Process> P ) {
    for ( Process p : P ) {
      if ( !p.empty() ) { return false; }
    }
    return true;
  }

  std::string prefix( int time, char pid ) {
    std::string out = "time ";
    out += std::to_string(time);
    out += "ms: Process ";
    out += pid;
    out += " ";
    return out;
  }

  void printQ( int nextTime ) {
    std::queue<Process> q = Q;
    std::cout << "[Q:";
    if ( q.empty() ) {
      std::cout << " empty";
    } else {
      while ( !(q.empty() ) ) {
        std::cout << " " << q.front().getID();
        q.pop();
      }
    }
    std::cout << "]\n";
    if ( nextTime == -2 || processes.empty() ) {
      return;
    }

    // new arrivals
    Process p = processes.front();
    if ( time + nextTime > p.getArrivalTime() || nextTime == -1 ) {
      addNewProcess();
    }
  }

  Process addNewProcess() {
    Process p = processes.front();
    Q.push(p);
    processes.erase( processes.begin() );

    std::cout << prefix( p.getArrivalTime(), p.getID() ) << "arrived; added to ready queue ";
    printQ(-2);
    return p;
  }
  


  int time = 0;
  std::queue<Process> Q;
  std::vector<Process> processes;
  std::priority_queue<Process, std::vector<Process>, CompareCPU> CPUworkingQueue;
  std::priority_queue<Process, std::vector<Process>, CompareIO> IOworkingQueue;
  const std::vector<Process> P;
  int contextSw;
};

template<typename T>
void pretty_print(T q) { // NB: pass by value so the print uses a copy
    while(!q.empty()) {
        std::cout << q.top() << std::endl;
        q.pop();
    }
    std::cout << '\n';
}

template<typename T>
void pretty_print( std::vector<T> v ) {
  for ( T t : v ) {
    std::cout << t << std::endl;
  }
}

template<typename T>
void pretty_print( std::queue<T> q ) {
  while ( !(q.empty())) {
    std::cout << q.front() << std::endl;
    q.pop();
  }
}



#endif


// for testing
int main() {
  int seed = 19;
  float lambda = 0.01;
  int upperBound = 4096;
  srand48(seed);
  // Process('A', seed, lambda, upperBound);
  std::vector<Process> v;
  for ( int i = 0; i < 2; i++ ) {
    v.push_back( Process( 'A' + i, seed, lambda, upperBound ) );
  }
  std::sort( v.begin(), v.end(), std::greater<>() );
  Processes processes = Processes(v, 4);
  processes.start();
  return 0;
}

