#include <iostream>
#include <fstream>
#include <string>
#include <queue>
#include <vector>
#include <algorithm>

// #include "next_exp.cpp"
#include "process.h"
char alphabet[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'};

int fcfs( std::vector<Process> processes, int contextSwitch );

int main( int argc, char** argv ) {
  if ( argc != 8 ) {
    std::cerr << "invalid arguments" << std::endl;
    return EXIT_FAILURE;
  }
  int numProcesses = atoi(*(argv + 1));
  int seed = atoi(*(argv + 2));
  double lambda = strtod(*(argv + 3), NULL);
  int upperBound = atoi(*(argv + 4));
  float contextSwitch = atof( *(argv + 5) );
  // float alpha = atof( *(argv + 6) );
  // float timeSlice = atof( *(argv + 7) );

#if 0 // debugging
  std::string names[7] = { "processes\t\t\t", "seed\t\t\t\t", "lambda\t\t\t\t", "upper bound\t\t\t", "context switch time\t", "alpha\t\t\t\t", "time slice\t\t\t" };

  for ( int i = 0; i < 7; i++ ) {
    std::cout << *(names + i) << "| " << *(argv + i + 1) << std::endl;
  }

  std::cout << "\n\n------------------------------\n";
#endif

  srand48(seed);

  // generate processes
  std::vector<Process> processes;
  for ( int i = 0; i < numProcesses; i++ ) {
    processes.push_back( Process(alphabet[i], seed, lambda, upperBound) );
  }


  int time = 0;
  // FCFS
  time += fcfs( processes, contextSwitch );

}

std::string prefix( int time, char pid ) {
  std::string out = "time ";
  out += std::to_string(time);
  out += "ms: Process ";
  out += pid;
  out += " ";
  return out;
}
void printQ( std::priority_queue<Process> Q );
void printQ( std::priority_queue<Process> &Q, std::vector<Process> &processes, int &time, int nextTime );
Process addNewProcess( std::vector<Process> &processes, std::priority_queue<Process> &Q );

void printQ( std::priority_queue<Process> Q ) {
  std::cout << "[Q:";
  if ( Q.empty() ) {
    std::cout << " empty]\n";
    return;
  }
  while (!(Q.empty())) {
    Process el = Q.top();
    std::cout << " " << el.getID();
    Q.pop();
  }
  std::cout<< "]\n";
}

void printQ( std::priority_queue<Process> &Q, std::vector<Process> &processes, int &time, int nextTime ) {
  std::cout << "test ";
  std::cout << "[Q:";
  if ( Q.empty() ) {
    std::cout << " empty]\n";
  } else {
    while (!(Q.empty())) {
      Process el = Q.top();
      std::cout << " " << el.getID();
      Q.pop();
    };
    std::cout<< "]\n";
  }

  Process p = processes.front();
  if ( time + nextTime > p.getArrivalTime() ) {
    addNewProcess( processes, Q );
    // time = p.getArrivalTime();
  }

}

Process addNewProcess( std::vector<Process> &processes, std::priority_queue<Process> &Q ) {
  Process p = processes.front();
  Q.push(p);
  processes.erase( processes.begin() );

  std::cout << prefix( p.getArrivalTime(), p.getID() ) << "arrived; added to ready queue ";
  printQ(Q);
  return p;
}

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

bool finished( std::vector<Process> P ) {
  for ( Process p : P ) {
    if ( !p.empty() ) { return false; }
  }
  return true;
}


int fcfs( std::vector<Process> processes, int contextSwitch ) {
  // int time = 0;
  const std::vector<Process> P = processes;
  std::cout << "time 0ms: Simulator started for FCFS [Q: empty]\n";
  std::sort( processes.begin(), processes.end(), std::greater<>() );

  pretty_print( processes );


  std::priority_queue<Process> Q;

  Process p = addNewProcess( processes, Q );

  int time = p.getArrivalTime();

  // while ( !(finished( P ) ) ) {
    Q.pop();
    time += contextSwitch/2;
    std::cout << prefix( time, p.getID() ) << "started using the CPU for " << p.current().first << "ms burst ";
    printQ(Q, processes, time, p.current().first );
    time += p.current().first;
    std::cout << prefix( time, p.getID() ) << "completed a CPU burst; " << p.getRemainingBursts() << (p.getRemainingBursts() == 1 ? " burst to go " : " bursts to go ");
    printQ(Q);
    std::cout << prefix( time, p.getID() ) << "switching out of CPU; will block on I/O until time " << time + p.current().second + contextSwitch/2<< "ms ";
    printQ(Q, processes, time, p.current().second );
    if ( !(Q.empty() )) {
      // if there are other processes in the queue that can do something then have them do stuff
      //  first come first serve -> we only look at the next process in the queue
      p = Q.top();
      Q.pop();
      // we remember the time that the other process has finished 
      // continue;
    }


  // }




#if 0 // this works for one input
  while ( !(Q.empty()) ) {
    time += contextSwitch/2;
    p.next();
    std::cout << prefix( time, p.getID() ) << "started using the CPU for " << p.current().first << "ms burst ";
    Q.pop();
    printQ(Q);
    time += p.current().first;
    p.cpuDone();
    if ( p.current().second ) {
      std::cout << prefix( time, p.getID() ) << "completed a CPU burst; " << p.getRemainingBursts() << (p.getRemainingBursts() == 1 ? " burst to go " : " bursts to go ");
      printQ(Q);
      std::cout << prefix( time, p.getID() ) << "switching out of CPU; will block on I/O until time " << time + p.current().second + contextSwitch/2 << "ms ";
      printQ(Q);
      time += p.current().second;
      p.ioDone();
      time += contextSwitch/2;
      std::cout << prefix( time, p.getID() ) << "completed I/O; added to ready queue ";
      Q.push(p);
      printQ(Q);
    }
    p.next();
    if ( p.empty() ) {
      std::cout << prefix( time, p.getID() ) << "terminated ";
      printQ(Q);
      time += contextSwitch/2;
    }
  }
#endif

  std::cout << "time " << time << "ms: Simulator ended for FCFS [Q: empty]\n";

  return time;
}

