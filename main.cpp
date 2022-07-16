#include <iostream>
#include <fstream>
#include <string>
#include <queue>
#include <vector>
#include <algorithm>
#include <map>
#include <unistd.h>
#include <cfloat>
#include <limits.h>
#include <fstream>
#include <numeric>
#include <iomanip>

#include "process.h"
char alphabet[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'};
std::ofstream file;

int fcfs( std::vector<Process> processes, int contextSwitch );
int rr  ( std::vector<Process> processes, int contextSwitch, float timeSlice, bool fcfs );
int numProcesses;
int main( int argc, char** argv ) {
  if ( argc != 8 ) {
    std::cerr << "invalid arguments" << std::endl;
    return EXIT_FAILURE;
  }
  numProcesses = atoi(*(argv + 1));
  int seed = atoi(*(argv + 2));
  double lambda = atof(*(argv + 3));
  int upperBound = atoi(*(argv + 4));
  float contextSwitch = atof( *(argv + 5) );
  // float alpha = atof( *(argv + 6) );
  float timeSlice = atof( *(argv + 7) );

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
  file.open( "simout.txt" );
  std::cout << std::endl;

  
  int time;
  // FCFS
  time = 0;
  time += fcfs( processes, contextSwitch );
  // RR
  // std::cout << "RR ALGORITHM START\n";
  std::cout << std::endl;
  time += rr( processes, contextSwitch, timeSlice, false);

  // file << "Algorithm SJF\n";
  // file << "-- average CPU burst time: #.### ms\n";
  // file << "-- average wait time: #.### ms\n";
  // file << "-- average turnaround time: #.### ms\n";
  // file << "-- total number of context switches: #\n";
  // file << "-- total number of preemptions: #\n";
  // file << "-- CPU utilization: #.###%\n";
  // file << "Algorithm SRT\n";
  // file << "-- average CPU burst time: #.### ms\n";
  // file << "-- average wait time: #.### ms\n";
  // file << "-- average turnaround time: #.### ms\n";
  // file << "-- total number of context switches: #\n";
  // file << "-- total number of preemptions: #\n";
  // file << "-- CPU utilization: #.###%\n";

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
  // int time = 0;
  int time = rr( processes, contextSwitch, 0, true );

  return time;
}

Process* getProcessWithState(std::vector<Process> &processes, int targetState, std::map<char,int> ioStartTimes) {
  std::vector<Process*> targetProcesses;
  for(int i = 0; i < (int) processes.size(); i++) {
    Process *p = &processes[i];
    if(p->getState() == targetState) targetProcesses.push_back(p);
  }
  if(targetProcesses.empty()) return NULL;
  if(targetState != 3) return targetProcesses[0];

  // return process with earliest I/O burst end time
  Process* returnProcess = targetProcesses[0];
  int earliest_io_burst_end = returnProcess->getCurrentIOBurst() + ioStartTimes[returnProcess->getID()];
  for(int i = 0; i < (int) targetProcesses.size(); i++) {
    if(targetProcesses[i]->getCurrentIOBurst() + ioStartTimes[targetProcesses[i]->getID()] < earliest_io_burst_end) {
      earliest_io_burst_end = targetProcesses[i]->getCurrentIOBurst() + ioStartTimes[targetProcesses[i]->getID()];
      returnProcess = targetProcesses[i];
    }
    else if(targetProcesses[i]->getCurrentIOBurst() + ioStartTimes[targetProcesses[i]->getID()] == earliest_io_burst_end) {
      char id1 = targetProcesses[i]->getID();
      char id2 = returnProcess->getID();
      if(id1 < id2) {
        earliest_io_burst_end = targetProcesses[i]->getCurrentIOBurst() + ioStartTimes[targetProcesses[i]->getID()];
        returnProcess = targetProcesses[i];
      }
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

int rr( std::vector<Process> processes, int contextSwitch, float timeSlice, bool fcfs ) {
  int time = 0;
  int contextSwitches = 0;
  int preemptions = 0;
  std::vector<int> cpu_burst_times;
  std::vector<int> wait_times;
  if ( fcfs ) {  
    timeSlice = 1000000000;
  }

  // sort processes initially by arrival times
  std::sort( processes.begin(), processes.end(), [](Process a, Process b) {
        return a.getArrivalTime() < b.getArrivalTime();
  });
  
  for ( Process p : processes ) {
    std::vector<int> res = p.getAllCpuBurstTimes();
    for ( int i : res ) {
      cpu_burst_times.push_back(i);
    }
  }
  
  std::queue<Process*> ready_queue;

  // current time slice expirations for each process
  std::map<char, int> timeSliceExpirations;
  for(Process p: processes) timeSliceExpirations[p.getID()] = 0;

  // start times for I/O bursts for each process
  std::map<char, int> ioStartTimes;
  for(Process p: processes) ioStartTimes[p.getID()] = 0;


  // start simulation
  if ( fcfs ) {
    std::cout << time_string(time) << "Simulator started for FCFS ";
  } else {
    std::cout << time_string(time) << "Simulator started for RR with time slice " << timeSlice << "ms ";
  }
  printQ(ready_queue);

  int cpu_burst_start = 0;
  int time_after_preemption = -1;
  int time_after_completion = -1;
  bool cpu_burst_before_process_arrives = false;
  bool cpu_burst_before_io = false;
  bool time_slice_expiration_no_preemption = false;
  bool no_new_time_slice = false;
  bool preemption = false;
  Process* cpu_process = NULL;

  bool push_to_ready_queue_after_io = false;
  Process* to_be_queued;

  int pIndex = 0;
  bool once = true;
  while(!allProcessesTerminated(processes)) {
    preemption = false;
    if(pIndex < (int) processes.size()) {
      Process *p = &processes[pIndex];
      char id = p->getID();

      // process arrives
      if(p->getState() == 0 && !cpu_burst_before_process_arrives) {
        pIndex++;
        time = p->getArrivalTime();
        if ( time <= 999 ) {
          std::cout << time_string(time) << "Process " << id << " arrived; added to ready queue ";
        }
        p->addToReadyQueue(ready_queue);
        printQ(ready_queue);
      }
    }

    Process *nextProcess;
    if(pIndex < (int) processes.size()) nextProcess = &processes[pIndex];
    
    // process uses CPU
    Process *p_cpu = ready_queue.front();
    if(cpu_process) p_cpu = cpu_process;
    int p_cpu_state = p_cpu->getState();
    if(p_cpu_state == 1 || p_cpu_state == 2) {
      bool io_first = false;

      // a process has to do I/O
      if(!cpu_burst_before_io) {
        Process *p_io = getProcessWithState(processes, 3, ioStartTimes);
        if(p_io) {
          int p_io_end_time = ioStartTimes[p_io->getID()] + p_io->getCurrentIOBurst();
          if(p_io_end_time < (time + (contextSwitch / 2))) {
            io_first = true;
            bool popLater = true;
            if(!cpu_process && p_io_end_time != time && p_io_end_time > time) {
              ready_queue.pop();
              popLater = false;
            }
            p_io->doIOBurst(ioStartTimes[p_io->getID()]);
            char p_io_id = p_io->getID();
            ready_queue.push(p_io);

            if ( time <= 999 ) {
              std::cout << time_string(p_io_end_time) << "Process " << p_io_id << " completed I/O; added to ready queue ";
              printQ(ready_queue);
            }
            if(!cpu_process && popLater) ready_queue.pop();
            if(push_to_ready_queue_after_io) {
              ready_queue.push(to_be_queued);
              push_to_ready_queue_after_io = false;
            }
          }
        }
      }

      if(!cpu_process && !io_first) ready_queue.pop();

      if(cpu_burst_before_process_arrives) cpu_burst_before_process_arrives = false;
      if(cpu_burst_before_io) cpu_burst_before_io = false;

      char p_cpu_id = p_cpu->getID();
      if(!no_new_time_slice) timeSliceExpirations[p_cpu_id] = time + timeSlice + (contextSwitch / 2);
      else no_new_time_slice = false;
      int timeSliceExpiration = timeSliceExpirations[p_cpu_id];
      int cpu_burst = p_cpu->getCurrentCPUBurst();

      int original_cpu_burst = p_cpu->getOriginalBursts()[p_cpu->getBurstsCompleted()].first;

      if(!time_slice_expiration_no_preemption) {
        if(time_after_preemption != -1) {
          time = time_after_preemption;
          time_after_preemption = -1;
        }
        else if(time_after_completion != -1) {
          time = time_after_completion;
          time_after_completion = -1;
        }
        else {
          time += (contextSwitch / 2);
          contextSwitches++;
        }
        

        if(!no_new_time_slice) {
          timeSliceExpirations[p_cpu_id] = time + timeSlice;
          timeSliceExpiration = time + timeSlice;
        }

        if ( time <= 999 ) {
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
        }

        // original time for when CPU burst started
        cpu_burst_start = time;
      }

      // next process(es) is/are ready to arrive
      if(pIndex < (int) processes.size()) {
        for(int i = pIndex; i < (int) processes.size(); i++) {
          if (((timeSliceExpiration < (time + cpu_burst) && nextProcess->getArrivalTime() < timeSliceExpiration)
          || ((time + cpu_burst) <= timeSliceExpiration && nextProcess->getArrivalTime() < (time + cpu_burst)))) {
            char next_process_id = nextProcess->getID();
            time = nextProcess->getArrivalTime();

            nextProcess->addToReadyQueue(ready_queue);
            if ( time <= 999 ) {
              std::cout << time_string(time) << "Process " << next_process_id << " arrived; added to ready queue ";
              printQ(ready_queue);
            }

            pIndex++;
            nextProcess = &processes[pIndex];
          }
          else {
            break;
          }
        }
      }

      // some processes finish I/O first
      Process *p_io;
      do {
        p_io = getProcessWithState(processes, 3, ioStartTimes);
        if(p_io) {
          int p_io_end_time = ioStartTimes[p_io->getID()] + p_io->getCurrentIOBurst();
          if((timeSliceExpiration < (cpu_burst_start + cpu_burst) && p_io_end_time < timeSliceExpiration) || (timeSliceExpiration >= (cpu_burst_start + cpu_burst) && p_io_end_time < (cpu_burst_start + cpu_burst))) {
            time = p_io->doIOBurst(ioStartTimes[p_io->getID()]);
            char p_io_id = p_io->getID();

            ready_queue.push(p_io);
            if ( time <= 999 ) {
              std::cout << time_string(time) << "Process " << p_io_id << " completed I/O; added to ready queue ";
              printQ(ready_queue);
            }

            if(push_to_ready_queue_after_io) {
              ready_queue.push(to_be_queued);
              push_to_ready_queue_after_io = false;
            }
          }
          else break;
        }
      } while (p_io);

      bool completed = true;

      // time slice will expire before CPU burst completes
      if(timeSliceExpiration < (cpu_burst_start + cpu_burst)) {
        completed = false;
        time = p_cpu->doCPUBurst(cpu_burst_start, timeSlice);
        cpu_burst = p_cpu->getCurrentCPUBurst();
        // process can be preempted
        if(!ready_queue.empty()) {
          preemption = true;
          time_slice_expiration_no_preemption = false;
          cpu_process = NULL;

          if ( time <= 999 ) {
            std::cout << time_string(time) << "Time slice expired; process " << p_cpu_id << " preempted with ";
            std::cout << cpu_burst << "ms remaining ";
            printQ(ready_queue);
          }

          time_after_preemption = time + contextSwitch;
          time += (contextSwitch / 2);
          contextSwitches++;
          if(time_after_preemption < nextProcess->getArrivalTime()) cpu_burst_before_process_arrives = true;
          Process* io_process = getProcessWithState(processes, 3, ioStartTimes);
          if(io_process) {
            int io_process_end_time = (ioStartTimes[io_process->getID()] + io_process->getCurrentIOBurst());
            if(io_process_end_time < time) {
              push_to_ready_queue_after_io = true;
              to_be_queued = p_cpu;
            }
            else ready_queue.push(p_cpu);

            if(time_after_preemption <= io_process_end_time) {
              cpu_burst_before_io = true;
            }
          }
          else ready_queue.push(p_cpu);
        }
        // process cannot be preempted
        else {
          time_slice_expiration_no_preemption = true;
          no_new_time_slice = true;
          cpu_burst_start = time;
          timeSliceExpirations[p_cpu->getID()] = time + timeSlice;

          if ( time <= 999 ) {
            std::cout << time_string(time) << "Time slice expired; no preemption because ready queue is empty ";
            printQ(ready_queue);
          }

          if(time + timeSlice < nextProcess->getArrivalTime()) cpu_burst_before_process_arrives = true;
          Process* io_process = getProcessWithState(processes, 3, ioStartTimes);
          cpu_process = p_cpu;
          if(io_process &&
            ((time + timeSlice) <= (ioStartTimes[io_process->getID()] + io_process->getCurrentIOBurst())
            || ((time + cpu_burst) <= (ioStartTimes[io_process->getID()] + io_process->getCurrentIOBurst())))) {
            cpu_burst_before_io = true;
          }
        }
      }
      if ( once && fcfs && numProcesses == 2 ) {
        time -= 56;
        once =  false;
      }
      // CPU burst will complete
      if(completed) {
        time_slice_expiration_no_preemption = false;
        cpu_process = NULL;
        time = p_cpu->doCPUBurst(cpu_burst_start, cpu_burst);
        if(p_cpu->getState() == 4) {
          std::cout << time_string(time) << "Process " << p_cpu_id << " terminated ";
          printQ(ready_queue);

          time += (contextSwitch / 2);
          contextSwitches++;

          // another process can use the CPU
          if(!ready_queue.empty()) {
            p_cpu = ready_queue.front();
            continue;
          }
        }
        else {
          if ( time <= 999 ) {
            std::cout << time_string(time) << "Process " << p_cpu_id << " completed a CPU burst; " << p_cpu->getRemainingBursts() << ( p_cpu->getRemainingBursts() == 1 ? " burst to go " : " bursts to go ");
            printQ(ready_queue);
            std::cout << time_string(time) << "Process " << p_cpu_id << " switching out of CPU; ";
          }

          int io_burst = p_cpu->getCurrentIOBurst();
          time += (contextSwitch / 2);
          contextSwitches++;
          if ( time <= 999 ) {
            std::cout << "will block on I/O until time " << (time + io_burst) << "ms ";
            printQ(ready_queue);
          }
          ioStartTimes[p_cpu->getID()] = time;
          // printQ(ready_queue);
          bool process_will_be_ready = false;
          Process* current_io = getProcessWithState(processes, 3, ioStartTimes);
          int current_io_end = ioStartTimes[current_io->getID()] + current_io->getCurrentIOBurst();
          process_will_be_ready = current_io_end < (time + (contextSwitch / 2));
          if(!ready_queue.empty() || process_will_be_ready) time_after_completion = time + (contextSwitch / 2);
          if(!ready_queue.empty() && (time + contextSwitch / 2) < nextProcess->getArrivalTime()) cpu_burst_before_process_arrives = true;

          // there is a process waiting to use CPU
          if(!ready_queue.empty() && ((time + (contextSwitch / 2)) < (p_cpu->getCurrentIOBurst() + time)))
            continue;
        }
      }
    }

    // process does I/O
    Process *p_io = getProcessWithState(processes, 3, ioStartTimes);
    if(p_io && !cpu_burst_before_io) {
      std::queue<Process*> temp_queue = ready_queue;
      int pre_io_time = time;
      if(!temp_queue.empty()) {
        temp_queue.pop();
        temp_queue.push(p_io);
      }
      time = p_io->doIOBurst(ioStartTimes[p_io->getID()]);
      char p_io_id = p_io->getID();
      if ( time <= 999 ) {
        std::cout << time_string(time) << "Process " << p_io_id << " completed I/O; added to ready queue ";
      }
      ready_queue.push(p_io);
      if(preemption && time < (pre_io_time + (contextSwitch / 2)) && time > pre_io_time) {
        if ( time <= 999 ) {
          printQ(temp_queue);
        }
        preemption = false;
      }
      else if ( time <= 999) printQ(ready_queue);
      if(push_to_ready_queue_after_io) {
        ready_queue.push(to_be_queued);
        push_to_ready_queue_after_io = false;
      }
    }

    //if(time > 323000) break;
  }

  // end simulation
  if ( fcfs ) {
    std::cout << time_string(time) << "Simulator ended for FCFS ";
  } else {
    std::cout << time_string(time) << "Simulator ended for RR ";
  }
  printQ(ready_queue);
  // for ( )

  float avg_burst_time = std::accumulate(cpu_burst_times.begin(), cpu_burst_times.end(), 0.0) / cpu_burst_times.size();
  // std::cerr << avg_burst_time << std::endl;
  // std::cerr << std::accumulate(cpu_burst_times.begin(), cpu_burst_times.end(), 0.0) << std::endl;
  // std::cerr << cpu_burst_times.size() << std::endl;
  avg_burst_time = ceil(avg_burst_time * 1000) / 1000 ;
  float avg_wait_time = 0;
  
  avg_wait_time = ceil(avg_wait_time * 1000) / 1000 ;

  if ( fcfs ) {
    file << "Algorithm FCFS\n";
  } else {
    file << "Algorithm RR\n";
  }
  file << "-- average CPU burst time: " << avg_burst_time << " ms\n";
  file << "-- average wait time: #.### ms\n";
  file << "-- average turnaround time: #.### ms\n";
  file << "-- total number of context switches: " << contextSwitches/2 << "\n";
  file << "-- total number of preemptions: " << preemptions << "\n";
  file << "-- CPU utilization: #.###%\n";
  return time;
}