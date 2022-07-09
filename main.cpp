#include <iostream>
#include <fstream>
#include <string>

// #include "next_exp.cpp"
#include "process.h"
char alphabet[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'};

int fcfs( std::vector<Process> processes );

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
  time += fcfs( processes );


  // for(int i = 0; i < numProcesses; i++) {
  //   // std::cout << "Process ID: " << i << "\n";
  //   // std::cout << "Arrival Time: " << arrivalTimes[i] << "ms\n";
  //   // std::cout << "Num. of CPU Bursts: " << cpuBurstsNums[i] << "\n";
  //   // std::cout << "CPU Bursts: \n\n";
  //   char pid = alphabet[i];
  //   std::cout << "time 0ms: Simulator started for FCFS [Q: empty]" << std::endl;
  //   std::cout << "time " << time << "ms: Process " << pid << " arrived; added to ready queue [Q: " << pid << "]\n";

  //   for(int c = 0; c < cpuBurstsNums[i]; c++) {
  //     // std::cout << "CPU Burst #" << c << "\n";
  //     // std::cout << "CPU Burst Time: " << cpuBursts[i][c].cpu_burst_time << "ms\n";
  //     // std::cout << "I/O Burst Time: " << cpuBursts[i][c].io_burst_time << "ms\n";
  //     // std::cout << "\n";
  //     time += contextSwitch/2;
  //     int cpu = cpuBursts[i][c].cpu_burst_time;
  //     int io = cpuBursts[i][c].io_burst_time;
  //     std::cout << "time " << time << "ms: Process " << alphabet[i] << "started using the CPU for " << cpu << "ms burst [Q: empty]\n";
  //     time += cpu;
  //     std::cout << "time " << time << "ms: Process " << pid << " completed a CPU burst; " << cpuBurstsNums[i]-c << " bursts to go [Q: empty]\n";
  //     std::cout << "time " << time << "ms: Process " << pid <<  " switching out of CPU; will block on I/O until time " << time + io << "ms [Q: empty]\n";
  //     time += io;
  //     std::cout << "time " << time << "ms: Process " << pid << " completed I/O; added to ready queue [Q: " << pid << "]\n";
  //     time += contextSwitch/2;

  //   }
  // }

  // std::cout<< "total time: " << time << std::endl;

}


int fcfs( std::vector<Process> processes ) {
  int time = 0;
  for ( Process p : processes ) {
    std::cout<< "new process " << p.getID() << std::endl;
    std::list<std::pair<int, int>> bursts = p.getBursts();
    int index = 0;
    for ( auto burstItr = bursts.begin(); burstItr != bursts.end(); burstItr++, index++ ) {
      std::cout << "CPU Burst: " << burstItr->first << "ms  \tIO Burst: " << burstItr->second << "ms\n";
    }
  }
}