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



// class WorkingQueue {
// public:
// private:
//   std::priority_queue<Process, std::vector<Process>, Comp> Q;
// }


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
p.cpuDone(); p.ioDone(); p.next();
CPUworkingQueue.push(p);
IOworkingQueue.push(p);
p = addNewProcess();
p.next();
CPUworkingQueue.push(p);
IOworkingQueue.push(p);
p = addNewProcess();


pretty_print(CPUworkingQueue);
pretty_print(IOworkingQueue);
printQ( 3500 );

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
#if 0
    // std::cout << "time: " << time << std::endl;
    // std::cout << "processes: " << std::endl;
    // pretty_print(processes);
#endif 
    if ( nextTime == -2 ) { return; }
    // look at the cpu/io working queue

    // todo: could break the io/cpu stuff into separate functions
    if ( !(IOworkingQueue.empty()) || !(CPUworkingQueue.empty()) ) {
      // look at all of the items in the io working queue and check to see if theyre done with their burst or not
      // std::cout << "both queues have stuff\n";
      bool noCPU = CPUworkingQueue.empty();
      bool noIO = IOworkingQueue.empty(); 
      bool ioDone = false;
      bool cpuDone = false;
      
      while ( (!noCPU || !noIO) && (!ioDone || cpuDone ) ) {
        // std::cout << "io working queue: \n";
        // pretty_print(IOworkingQueue);
        noCPU = CPUworkingQueue.empty();
        noIO = IOworkingQueue.empty();
        Process cpu, io;

        if ( noCPU ) {
          io = IOworkingQueue.top();
          // io only
          while ( io.current().second <= nextTime ) {
            std::cout << prefix( io.current().second + time, io.getID() ) << " completed I/O; added to ready queue ";
            Q.push( io );
            IOworkingQueue.pop();
            printQ( -2 );
            if ( !IOworkingQueue.empty() ) {
              io = IOworkingQueue.top();
            } else { break; }
          }
          ioDone =  ( IOworkingQueue.empty() || io.current().second > nextTime );
        } else if ( noIO ) {
          // cpu only
          cpu = CPUworkingQueue.top();

          while ( cpu.current().first <= nextTime ) {
            std::cout << prefix( cpu.current().first + time, cpu.getID() ) << "completed a CPU burst; " << cpu.getRemainingBursts() << (cpu.getRemainingBursts() == 1 ? " burst to go " : " bursts to go ");
            printQ( -2 );
            CPUworkingQueue.pop();
            if ( !CPUworkingQueue.empty() ) {
              cpu = CPUworkingQueue.top();
            } else { break ; }
          } 
          cpuDone = ( CPUworkingQueue.empty() || cpu.current().first > nextTime );
        } else {
          cpu = CPUworkingQueue.top();
          io = IOworkingQueue.top();
          bool CPU = true;
          bool IO = false;
          // exit this loop once we run out of either cpu or io bursts
          while ( !cpuDone && !ioDone ) {
            CPU = (cpu.current().first < io.current().second);
            IO = !CPU;
            if (CPU) {
              while ( cpu.current().first <= nextTime ) {
                std::cout << prefix( cpu.current().first + time, cpu.getID() ) << "completed a CPU burst; " << cpu.getRemainingBursts() << (cpu.getRemainingBursts() == 1 ? " burst to go " : " bursts to go ");
                printQ( -2 );
                CPUworkingQueue.pop();
                if ( !CPUworkingQueue.empty() ) {
                  cpu = CPUworkingQueue.top();
                } else { noCPU = true; break; }
              } 
              cpuDone = ( CPUworkingQueue.empty() || cpu.current().first > nextTime );
            } else {
              // std::cout << "doing io\n";
              // if ( !IOworkingQueue.empty() ) {
              //   IOworkingQueue.pop();
              //   io = IOworkingQueue.top();
              // } else { noIO = true; }
              // noIO = true;
              while ( io.current().second <= nextTime ) {
                std::cout << prefix( io.current().second + time, io.getID() ) << " completed I/O; added to ready queue ";
                Q.push( io );
                IOworkingQueue.pop();
                printQ( -2 );
                if ( !IOworkingQueue.empty() ) {
                  io = IOworkingQueue.top();
                } else { noIO = true; break; }
              }
              ioDone =  ( IOworkingQueue.empty() || io.current().second > nextTime );
            }
            // noCPU = CPUworkingQueue.empty();
            noIO = IOworkingQueue.empty();
            cpuDone = ( noCPU || cpu.current().first > nextTime );
            ioDone = ( noIO || io.current().second > nextTime );
          }


        }
        noCPU = CPUworkingQueue.empty();
        noIO = IOworkingQueue.empty();
      }
    } 


    if ( processes.empty() ) {
      return;
    }
    // new arrivals
    //TODO:  check to see if i can remove the nextTime == -1
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

