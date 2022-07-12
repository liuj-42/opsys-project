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
  float alpha = atof( *(argv + 6) );
  float timeSlice = atof( *(argv + 7) );

#if 1 // debugging
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

template<typename T>
void pretty_print(T q) { // NB: pass by value so the print uses a copy
    while(!q.empty()) {
        std::cout << q.top() << std::endl;
        q.pop();
    }
    std::cout << '\n';
}


// std::string



int fcfs( std::vector<Process> processes, int contextSwitch ) {
  int time = 0;
  // std::priority_queue<Process> Q;
  std::cout << "time 0ms: Simulator started for FCFS [Q: empty]\n";
  std::sort( processes.begin(), processes.end(), [](Process a, Process b) {
        return a.getArrivalTime() < b.getArrivalTime();
  });
  auto cmp = [](Process a, Process b) { return a.getArrivalTime() > b.getArrivalTime(); };

  std::priority_queue<Process> Q;
  // std::priority_queue<Process, std::vector<Process>, decltype(comp)> Q(comp);


  // for ( Process p : processes ) { Q.push(p); }



  Process p = processes.front();
  Q.push(p);
  time += p.getArrivalTime();
  std::cout << prefix( time, p.getID() ) << "arrived, added to ready queue";
  printQ(Q);


  // while ( !(Q.empty()) ) {
  //   // check for processes

    
  // }
  

  

#if 0
  for ( Process p : processes ) {
    time += p.getArrivalTime();
    std::cout << prefix( time, p.getID() ) << "arrived, added to ready queue";
    std::cout << "[Q: " << p.getID() << "]\n";
    std::list<std::pair<int, int>> bursts = p.getBursts();
    int index = 0;
    for ( auto burstItr = bursts.begin(); burstItr != bursts.end(); burstItr++, index++ ) {
      time += contextSwitch/2;
      std::cout << prefix( time, p.getID() ) << "started using the CPU for " << burstItr->first << "ms burst ";
      std::cout << "[Q: empty]\n";
      time += burstItr->first;
      std::cout << prefix( time, p.getID() ) << "completed a CPU burst; " << p.getRemainingBursts(index) << (p.getRemainingBursts(index) == 1 ? " burst to go " : " bursts to go ");  
      std::cout << "[Q: empty]\n";
      if ( burstItr->second ) {
        std::cout << prefix( time, p.getID() ) << "switching out of CPU, will block on I/O until time " << time + burstItr->second << "ms ";
        std::cout << "[Q: empty]\n";
        time += burstItr->second;
        time += contextSwitch/2;
        std::cout << prefix( time, p.getID() )  << "completed I/O; added to ready queue ";
        std::cout << "[Q: " << p.getID() << "]\n";
      }
    }
    std::cout << prefix( time, p.getID() ) << "terminated ";
    std::cout << "[Q: empty]\n";
    time += contextSwitch/2;
  }
#endif
  

  // for ( Process p : processes ) {
  //   std::cout << "Process " << p.getID() << " with arrival time " << p.getArrivalTime() << std::endl;
  // }


  // std::cout << "time " << time << "ms: Simulator ended for FCFS [Q: empty]\n";
  // std::cout << "total time: " << time << "ms\n";

  return time;
}

