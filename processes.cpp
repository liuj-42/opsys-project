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

void sanity() {
  std::cout << "sanity check" << std::endl;
}


template<typename T>
void push(T q, Process p) { // NB: pass by value so the print uses a copy
    std::cout << "process " << p.getID() << " was added to the working queue" << std::endl;
    q.push(p);
}

class Processes {
public:
  Processes( std::vector<Process> processes, int cs ): processes(processes), P(processes), contextSw(cs) { }

  void start() {
    bool once = true;
    std::cout << std::endl;
    int x = 0;
    std::cout << "time 0ms: Simulator started for FCFS ";
    printQ();
    Process p = addNewProcess( 0 );
    // time += p.getArrivalTime();
    bool res;
    int finishedProcesses = 0;
    while ( finishedProcesses != P.size() ) {
    // while ( x++ < 3 ) {
      // std::cout << "x:" << x << std::endl;
      if ( !(Q.empty()) ) { Q.pop(); }
      p.next(); 
      // time += contextSw/2;
      // this should only print once both working queues empty themselves with stuff that wouldve run
      p.startCpu( time );
      // std::cout << prefix( time, p.getID() ) << "started using the CPU for " << p.current().first << "ms burst ";
      CPUworkingQueue.push( p );
      printQ();


      checkArrivals( p.current().first, p.getTime()  );

      checkWorkingQueues( p.current().first );
      p.setTime( time );


      if ( !Q.empty() ) {
        // if there is stuff in the queue then look at it instead
        time = p.getTime();
        p = Q.front();
        Q.pop();
        p.setTime( time );
        p.incrementContextSwitches();
        if ( once ) {
          once = false;
          p.incrementContextSwitches();
        }

        // time += contextSw/2;
        continue;
      }

      if ( p.getRemainingBursts() == 0 ) {
        time = p.done();
        // std::cout << prefix( p.getTime(), p.getID() ) << "terminated ";
        printQ();
        // std::cerr << p << std::endl;
        finishedProcesses++;
      }
      // time += contextSw/2;
      checkWorkingQueues( p.current().second );
      p.setTime( time );

      p.next();
      if ( !(Q.empty()) ) {
        time = p.getTime();
        // std::cout << "time: " << time << std::endl;
        p = Q.front();
        Q.pop();
        p.setTime( time );
        p.incrementContextSwitches();
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
      int ioDone = 0;
      int cpuDone = 0;
      
      while ( (!ioDone || !cpuDone ) ) {

      // printWorkingQueues();

        Process cpu, io;
        if ( CPUworkingQueue.empty() || IOworkingQueue.empty() ) { // io only
          cpuDone = cpuQueue( nextTime );
          if ( cpuDone == -1 ) {
            done = true;
            break;
          }
          ioDone = ioQueue( nextTime );
          if ( ioDone == -1 ) {
            // std::cout << "leaving \n";
            break;
          }

        }
        else {  // both
          bool CPU, IO;
          while ( !cpuDone && !ioDone ) {
            cpu = CPUworkingQueue.top();
            io = IOworkingQueue.top();
            if (cpu.current().first < io.current().second) {
              cpuDone = cpuQueue( nextTime );
              if ( cpuDone == -1 ) {
                done = true;
                break;
              }
            } else {
              ioDone = ioQueue( nextTime );
              if ( ioDone == -1 ) {
                break;
              }
            }
          }
        }
      }

    } 

  }

  int ioQueue( int nextTime ) {
    // sanity();
    // std::cout << "Q:\n";
    // pretty_print(Q);
    if ( IOworkingQueue.empty()) { return true ;}

    Process io = IOworkingQueue.top();
    // std::cout << "nextTime: " << nextTime << std::endl;
    while ( io.current().second <= nextTime ) {
      // std::cout << "here io\n";
    if ( !Q.empty() ) {   // todo: more checks
      return -1;
    }
      io.ioDone();
      time = io.getTime();
      Q.push( io );
      printQ();
      IOworkingQueue.pop();

      if ( !IOworkingQueue.empty() ) {
        io = IOworkingQueue.top();
      } else { break; }
    }
    int ioDone = ( IOworkingQueue.empty() || io.current().second > nextTime );
    return ioDone;
  }

  int cpuQueue( int nextTime ) {
    if ( CPUworkingQueue.empty()) { return true ;}
    Process cpu = CPUworkingQueue.top();
    while ( cpu.current().first <= nextTime ) { 
      // std::cout << "here cpu\n";
      if ( cpu.getRemainingBursts() == 0 ) {
        // time += cpu.current().first;
        return -1; // true
      } 

      // "completed a cpu burst ... "
      cpu.cpuDone();
      printQ();
      time = cpu.getTime();

      cpu.startIo( time );

      // std::cout << prefix( cpu.current().first + time, cpu.getID() ) << "switching out of CPU; will block on I/O until time " << cpu.current().first + time + cpu.current().second + contextSw/2 << "ms ";
      printQ();
      // look at the ready queue and if theres stuff there then start them

      // time += cpu.current().first;
      IOworkingQueue.push( cpu );
      CPUworkingQueue.pop();

      if ( !CPUworkingQueue.empty() ) {

        cpu = CPUworkingQueue.top();
      } else { break ; }
    } 
    int cpuDone = (int) ( CPUworkingQueue.empty() || cpu.current().first > nextTime );
    return cpuDone;
  }

  void printWorkingQueues() {
    std::cout << "cpu\n";
    pretty_print( CPUworkingQueue );
    std::cout << "io\n";
    pretty_print( IOworkingQueue );
  }

  void checkArrivals( int nextTime, int pTime ) {
    if ( processes.empty() ) {
      return;
    }
    // new arrivals

    Process p = processes.front();
    if ( time + nextTime > p.getArrivalTime() ) {
      addNewProcess( pTime );
    }
  }

  Process addNewProcess( int pTime ) {
    Process p = processes.front();
    Q.push(p);
    processes.erase( processes.begin() );

    std::cout << prefix( p.getArrivalTime(), p.getID() ) << "arrived; added to ready queue ";
    if ( pTime != 0 ) {
      p.setTime( pTime );
    }
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
    v.push_back( Process( 'A' + i, seed, lambda, upperBound, 4 ) );
  }
  std::sort( v.begin(), v.end(), std::greater<>() );
  Processes processes = Processes(v, 4);
  processes.start();
  return 0;
}

