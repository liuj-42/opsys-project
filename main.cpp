#include <iostream>
#include <fstream>
#include <string>
#include <queue>
#include <vector>
#include <algorithm>
#include <map>
#include <unistd.h>

// #include "next_exp.cpp"
#include "process.h"
char alphabet[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'};

int fcfs( std::vector<Process> processes, int contextSwitch );
int rr  ( std::vector<Process> processes, int contextSwitch, float timeSlice );

int main( int argc, char** argv ) {
  if ( argc != 8 ) {
    std::cerr << "invalid arguments" << std::endl;
    return EXIT_FAILURE;
  }
  int numProcesses = atoi(*(argv + 1));
  int seed = atoi(*(argv + 2));
  double lambda = atof(*(argv + 3));
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
  std::cout << std::endl;


  int time;
  // FCFS
  //time = 0;
  //time += fcfs( processes, contextSwitch );
  // RR
  std::cout << "RR ALGORITHM START\n";
  time = 0;
  time += rr( processes, contextSwitch, timeSlice);
}

std::string prefix( int time, char pid ) {
  std::string out = "time ";
  out += std::to_string(time);
  out += "ms: Process ";
  out += pid;
  out += " ";
  return out;
}

void printQ( std::queue<Process> Q ) {
  std::queue<Process> temp = Q;
  std::cout << "[Q:";
  if ( temp.empty() ) {
    std::cout << " empty]\n";
    return;
  }
  while (!(temp.empty())) {
    Process el = temp.front();
    std::cout << " " << el.getID();
    temp.pop();
  }
  std::cout << "]\n";
}

void printQ(std::queue<Process*> Q) {
  std::queue<Process*> temp = Q;
  std::cout << "[Q:";
  if ( temp.empty() ) {
    std::cout << " empty]\n";
    return;
  }
  while (!(temp.empty())) {
    Process el = *temp.front();
    std::cout << " " << el.getID();
    temp.pop();
  }
  std::cout << "]\n";
}

template<typename T>
void print_pqueue(T q) { // NB: pass by value so the print uses a copy
    while(!q.empty()) {
        std::cout << q.top() << ' ';
        q.pop();
    }
    std::cout << '\n';
}

template<typename T>
void print_queue(T q) { // NB: pass by value so the print uses a copy
    while(!q.empty()) {
        std::cout << q.front() << ' ';
        q.pop();
    }
    std::cout << '\n';
}

std::string time_string(int time) {
  std::string out = "time ";
  out += std::to_string(time);
  out += "ms: ";
  return out;
}

int fcfs( std::vector<Process> processes, int contextSwitch ) {
  int time = 0;
  // std::priority_queue<Process> Q;
  std::cout << "time 0ms: Simulator started for FCFS [Q: empty]\n";
  std::sort( processes.begin(), processes.end(), [](Process a, Process b) {
        return a.getArrivalTime() < b.getArrivalTime();
  });
  auto cmp = [](Process a, Process b) { return a.getArrivalTime() < b.getArrivalTime(); };
  std::priority_queue<Process, std::vector<Process>, decltype(cmp)> Q(cmp);
  for ( Process p : processes ) { Q.push(p); }

  //print_pqueue(Q);
  

#if 0
  for ( Process p : processes ) {
    time += p.getArrivalTime();
    std::cout << prefix( time, p.getID() ) << "arrived, added to ready queue";
    std::cout << " [Q: " << p.getID() << "]\n";
    std::list<std::pair<int, int>> bursts = p.getBursts();
    int index = 1;
    for ( auto burstItr = bursts.begin(); burstItr != bursts.end(); burstItr++, index++ ) {
      time += contextSwitch/2;
      std::cout << prefix( time, p.getID() ) << "started using the CPU for " << burstItr->first << "ms burst ";
      std::cout << "[Q: empty]\n";
      time += burstItr->first;
      std::cout << prefix( time, p.getID() ) << "completed a CPU burst; " << p.getRemainingBursts(index) << (p.getRemainingBursts(index) == 1 ? " burst to go " : " bursts to go ");  
      std::cout << "[Q: empty]\n";
      if ( burstItr->second ) {
        std::cout << prefix( time, p.getID() ) << "switching out of CPU, will block on I/O until time " << time + burstItr->second + (contextSwitch/2) << "ms ";
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


  std::cout << "time " << time << "ms: Simulator ended for FCFS [Q: empty]\n";
  // std::cout << "total time: " << time << "ms\n";

  return time;
}

Process* getProcessWithState(std::vector<Process> &processes, int targetState, std::map<char,int> ioStartTimes) {
  std::vector<Process*> targetProcesses;
  for(int i = 0; i < (int) processes.size(); i++) {
    Process *p = &processes[i];
    if(p->getState() == targetState) targetProcesses.push_back(p);
  }
  if(targetProcesses.size() == 0) return NULL;
  if(targetState != 3) return targetProcesses[0];

  // return process with earliest I/O burst end time
  Process* returnProcess = targetProcesses[0];
  int earliest_io_burst_end = returnProcess->getCurrentIOBurst() + ioStartTimes[returnProcess->getID()];
  for(int i = 0; i < (int) targetProcesses.size(); i++) {
    if(targetProcesses[i]->getCurrentIOBurst() + ioStartTimes[targetProcesses[i]->getID()] < earliest_io_burst_end) {
      earliest_io_burst_end = targetProcesses[i]->getCurrentIOBurst() + ioStartTimes[targetProcesses[i]->getID()];
      returnProcess = targetProcesses[i];
    }
  }

  return returnProcess;
}

bool allProcessesTerminated(std::vector<Process> processes) {
  for(Process p: processes) {
    if(p.getState() != 4) return false;
  }
  return true;
}

int rr( std::vector<Process> processes, int contextSwitch, float timeSlice ) {
  int time = 0;

  // sort processes initially by arrival times
  std::sort( processes.begin(), processes.end(), [](Process a, Process b) {
        return a.getArrivalTime() < b.getArrivalTime();
  });
  
  std::queue<Process*> ready_queue;

  // current time slice expirations for each process
  std::map<char, int> timeSliceExpirations;
  for(Process p: processes) timeSliceExpirations[p.getID()] = 0;

  // start times for I/O bursts for each process
  std::map<char, int> ioStartTimes;
  for(Process p: processes) ioStartTimes[p.getID()] = 0;

  // start simulation
  std::cout << time_string(time) << "Simulator started for RR with time slice ";
  std::cout << timeSlice << "ms ";
  printQ(ready_queue);

  int pIndex = 0;
  while(!allProcessesTerminated(processes)) {
    if(pIndex < (int) processes.size()) {
      Process *p = &processes[pIndex];
      char id = p->getID();
      pIndex++;

      // process arrives
      if(p->getState() == 0) {
        time = p->getArrivalTime();
        std::cout << time_string(time) << "Process " << id << " arrived; added to ready queue ";
        p->addToReadyQueue(ready_queue);
        printQ(ready_queue);
      }
    }

    Process *nextProcess;
    if(pIndex < (int) processes.size()) nextProcess = &processes[pIndex];
    
    // process uses CPU
    Process *p_cpu = ready_queue.front();
    if(p_cpu->getState() == 1 || p_cpu->getState() == 2) {
      char p_cpu_id = p_cpu->getID();
      ready_queue.pop();
      time += (contextSwitch / 2);
      timeSliceExpirations[p_cpu_id] = time + timeSlice;
      int timeSliceExpiration = timeSliceExpirations[p_cpu_id];
      int cpu_burst = p_cpu->getCurrentCPUBurst();

      int original_cpu_burst = p_cpu->getOriginalBursts()[p_cpu->getBurstsCompleted()].first;

      if(cpu_burst == original_cpu_burst) {
        std::cout << time_string(time) << "Process " << p_cpu_id << " started using the CPU for ";
        std::cout << cpu_burst << "ms burst ";
        printQ(ready_queue);
      }
      else {
        std::cout << time_string(time) << "Process " << p_cpu_id << " started using the CPU for ";
        std::cout << "remaining " << cpu_burst << "ms of " << original_cpu_burst << "ms burst ";
        printQ(ready_queue);
      }

      // original time for when CPU burst started
      int cpu_burst_start = time;

      // next process is ready to arrive
      if(pIndex < (int) processes.size() && (nextProcess->getArrivalTime() < timeSliceExpiration
        || nextProcess->getArrivalTime() < (time + cpu_burst))) {
          char next_process_id = nextProcess->getID();
          time = nextProcess->getArrivalTime();
          std::cout << time_string(time) << "Process " << next_process_id << " arrived; added to ready queue ";
          nextProcess->addToReadyQueue(ready_queue);
          printQ(ready_queue);
      }

      bool preemption = false;

      // time slice will expire before CPU burst completes
      if(timeSliceExpiration < (time + cpu_burst)) {
        time = p_cpu->doCPUBurst(cpu_burst_start, timeSlice);
        cpu_burst = p_cpu->getCurrentCPUBurst();
        // process can be preempted
        if(!ready_queue.empty()) {
          std::cout << time_string(time) << "Time slice expired; process " << p_cpu_id << " preempted with ";
          std::cout << cpu_burst << "ms remaining ";
          printQ(ready_queue);
          ready_queue.push(p_cpu);
          preemption = true;
          time += (contextSwitch / 2);
        }
        // process cannot be preempted
        else {
          int numTimeSliceExpirations = (int) floor(cpu_burst / timeSlice) + 1;
          for(int i = 0; i < numTimeSliceExpirations; i++) {
            std::cout << time_string(time + (timeSlice * i)) << "Time slice expired; no preemption because ready queue is empty ";
            printQ(ready_queue);
          }
        }
      }
      // CPU burst will complete
      if(!preemption) {
        time = p_cpu->doCPUBurst(time, cpu_burst);
        if(p_cpu->getState() == 4) {
          std::cout << time_string(time) << "Process " << p_cpu_id << " terminated ";
          printQ(ready_queue);
          time += (contextSwitch / 2);
        }
        else {
          std::cout << time_string(time) << "Process " << p_cpu_id << " completed a CPU burst; ";
          std::cout << p_cpu->getRemainingBursts() << " bursts to go ";
          printQ(ready_queue);
          std::cout << time_string(time) << "Process " << p_cpu_id << " switching out of CPU; ";
          int io_burst = p_cpu->getCurrentIOBurst();
          time += (contextSwitch / 2);
          std::cout << "will block on I/O until time " << (time + io_burst) << "ms ";
          ioStartTimes[p_cpu->getID()] = time;
          printQ(ready_queue);
        }
      }

      // there is a process waiting to use CPU
      if(!ready_queue.empty() && ((time + (contextSwitch / 2)) < (p_cpu->getCurrentIOBurst() + time)))
        continue;
    }

    // process does I/O
    Process *p_io = getProcessWithState(processes, 3, ioStartTimes);
    if(p_io) {
      time = p_io->doIOBurst(ioStartTimes[p_io->getID()]);
      char p_io_id = p_io->getID();
      std::cout << time_string(time) << "Process " << p_io_id << " completed I/O; added to ready queue ";
      ready_queue.push(p_io);
      printQ(ready_queue);
    }
  }

  // end simulation
  std::cout << time_string(time) << "Simulator ended for RR ";
  printQ(ready_queue);

  return time;
}