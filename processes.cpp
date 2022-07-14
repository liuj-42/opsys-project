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

  void start() {
    std::cout << std::endl;
    int x = 0;
    std::cout << "time 0ms: Simulator started for FCFS ";
    printQ();
    Process p = addNewProcess();
    time += p.getArrivalTime();

// Process p = addNewProcess();
// p.next();
// p.cpuDone(time); p.ioDone(time); p.next();
// // std::cout << p << std::endl;
// CPUworkingQueue.push(p);
// IOworkingQueue.push(p);
// p = addNewProcess();
// p.next();
// CPUworkingQueue.push(p);
// IOworkingQueue.push(p);
// pretty_print(CPUworkingQueue);
// pretty_print(IOworkingQueue);
// printQ( 3500 );
    bool res;
    int finishedProcesses = 0;
    // pretty_print( P );
    while ( finishedProcesses != P.size() ) {
    // while ( x++ < 3 ) {
      // std::cout << "x:" << x << std::endl;
      if ( !(Q.empty()) ) { Q.pop(); }
      p.next();
      time += contextSw/2;
      // this should only print once both working queues empty themselves with stuff that wouldve run
      std::cout << prefix( time, p.getID() ) << "started using the CPU for " << p.current().first << "ms burst ";
      CPUworkingQueue.push( p );
      printQ();
      checkArrivals( p.current().first );
      checkWorkingQueues( p.current().first );

      if ( !Q.empty() ) {
        // if there is stuff in the queue then look at it instead
        p = Q.front();
        Q.pop();
        time += contextSw/2;
        continue;
      }

      if ( p.getRemainingBursts() == 0 ) {
        std::cout << prefix( time, p.getID() ) << "terminated ";
        printQ();
        // std::cerr << p << std::endl;
        finishedProcesses++;
      }
      time += contextSw/2;
      checkWorkingQueues( p.current().second );
      p.next();
      if ( !(Q.empty()) ) {
        p = Q.front();
      }

    }
    std::cout << "time " << time << "ms: Simulator ended for FCFS ";
    printQ();

  }
  friend int main();

private:

  bool finished( std::vector<Process> P ) {
    for ( Process p : P ) {
      if ( p.getRemainingBursts() != 0 ) { 
        return false;
      }
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

  void printQ() {
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
  }

  void checkWorkingQueues( int nextTime ) {
    bool done = false;
    if ( !(IOworkingQueue.empty()) || !(CPUworkingQueue.empty()) ) {
      bool ioDone = false;
      int cpuDone = 0;
      
      while ( (!ioDone || !cpuDone ) ) {
        Process cpu, io;
        if ( CPUworkingQueue.empty() || IOworkingQueue.empty() ) { // io only
          cpuDone = cpuQueue( nextTime );
          if ( cpuDone == -1 ) {
            done = true;
            break;
          }
          ioDone = ioQueue( nextTime );

        }
        else {  // both
          bool CPU, IO;
          while ( !cpuDone && !ioDone ) {
            cpu = CPUworkingQueue.top();
            io = IOworkingQueue.top();
            if (cpu.current().first < io.current().second) {
              cpuDone = cpuQueue( nextTime );
            } else {
              ioDone = ioQueue( nextTime );
            }
          }
        }
      }

    } 

  }

  bool ioQueue( int nextTime ) {
    if ( IOworkingQueue.empty()) { return true ;}
    Process io = IOworkingQueue.top();
    while ( io.current().second <= nextTime ) {
      // std::cout << "here io\n";
      std::cout << prefix( io.current().second + time, io.getID() ) << "completed I/O; added to ready queue ";
      io.ioDone( io.current().second + time );
      Q.push( io );
      io.ioDone( time + io.current().second );
      IOworkingQueue.pop();
      printQ();
      
      time += io.current().second;
      if ( !IOworkingQueue.empty() ) {

        io = IOworkingQueue.top();
      } else { break; }
    }
    bool ioDone = ( IOworkingQueue.empty() || io.current().second > nextTime );
    return ioDone;
  }
  int cpuQueue( int nextTime ) {
    if ( CPUworkingQueue.empty()) { return true ;}
    Process cpu = CPUworkingQueue.top();
    while ( cpu.current().first <= nextTime ) { 
      // std::cout << "here cpu\n";
      if ( cpu.getRemainingBursts() == 0 ) {
        time += cpu.current().first;
        return -1; // true
      } 
      // "completed a cpu burst ... "
      cpu.cpuDone( time + cpu.current().first );
      printQ();

      std::cout << prefix( cpu.current().first + time, cpu.getID() ) << "switching out of CPU; will block on I/O until time " << cpu.current().first + time + cpu.current().second + contextSw/2 << "ms ";
      printQ();
      // look at the ready queue and if theres stuff there then start them

      time += cpu.current().first;
      IOworkingQueue.push( cpu );
      CPUworkingQueue.pop();

      if ( !CPUworkingQueue.empty() ) {

        cpu = CPUworkingQueue.top();
      } else { break ; }
    } 
    int cpuDone = (int) ( CPUworkingQueue.empty() || cpu.current().first > nextTime );
    return cpuDone;
  }

  void checkArrivals( int nextTime ) {
    if ( processes.empty() ) {
      return;
    }
    // new arrivals

    Process p = processes.front();
    if ( time + nextTime > p.getArrivalTime() ) {
      addNewProcess();
    }
  }

  Process addNewProcess() {
    Process p = processes.front();
    Q.push(p);
    processes.erase( processes.begin() );

    std::cout << prefix( p.getArrivalTime(), p.getID() ) << "arrived; added to ready queue ";
    printQ();
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

