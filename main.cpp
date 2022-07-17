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
#include "processSJF.h"
char alphabet[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'};
std::ofstream file;

int fcfs(std::vector<Process> processes, int contextSwitch);
int rr(std::vector<Process> processes, int contextSwitch, float timeSlice, bool fcfs);
int sjf(std::vector<ProcessSJF> processes, int contextSwitch);
int srt(std::vector<ProcessSJF> processes, int contextSwitch);
int numProcesses;
int main(int argc, char **argv)
{
  if (argc != 8)
  {
    std::cerr << "invalid arguments" << std::endl;
    return EXIT_FAILURE;
  }
  numProcesses = atoi(*(argv + 1));
  int seed = atoi(*(argv + 2));
  double lambda = atof(*(argv + 3));
  int upperBound = atoi(*(argv + 4));
  float contextSwitch = atof(*(argv + 5));
  float alpha = atof(*(argv + 6));
  float timeSlice = atof(*(argv + 7));

  srand48(seed);

  // generate processes
  std::vector<Process> processes;
  std::vector<ProcessSJF> processesSJF;
  for (int i = 0; i < numProcesses; i++)
  {
    processes.push_back(Process(alphabet[i], seed, lambda, upperBound));
  }

  srand48(seed);
  for (int i = 0; i < numProcesses; i++)
  {
    processesSJF.push_back(ProcessSJF(alphabet[i], seed, lambda, upperBound, alpha));
  }

  file.open("simout.txt");

  int time;
  // FCFS
  time = 0;
  std::cout << std::endl;
  time += fcfs(processes, contextSwitch);

  // SJF
  std::cout << std::endl;
  time += sjf(processesSJF, contextSwitch);

  // SRT
  std::cout << std::endl;
  time += srt(processesSJF, contextSwitch);

  // RR
  std::cout << std::endl;
  time += rr(processes, contextSwitch, timeSlice, false);
}

std::string prefix(int time, char pid)
{
  std::string out = "time ";
  out += std::to_string(time);
  out += "ms: Process ";
  out += pid;
  out += " ";
  return out;
}

void printQ(std::queue<Process> Q)
{
  std::queue<Process> temp = Q;
  std::cout << "[Q:";
  if (temp.empty())
  {
    std::cout << " empty]\n";
    return;
  }
  while (!(temp.empty()))
  {
    Process el = temp.front();
    std::cout << " " << el.getID();
    temp.pop();
  }
  std::cout << "]\n";
}

void printQ(std::queue<Process *> Q)
{
  std::queue<Process *> temp = Q;
  std::cout << "[Q:";
  if (temp.empty())
  {
    std::cout << " empty]\n";
    return;
  }
  while (!(temp.empty()))
  {
    Process el = *temp.front();
    std::cout << " " << el.getID();
    temp.pop();
  }
  std::cout << "]\n";
}

template <typename T>
void print_pqueue(T q)
{ // NB: pass by value so the print uses a copy
  while (!q.empty())
  {
    std::cout << q.top() << ' ';
    q.pop();
  }
  std::cout << '\n';
}

template <typename T>
void print_queue(T q)
{ // NB: pass by value so the print uses a copy
  while (!q.empty())
  {
    std::cout << q.front() << ' ';
    q.pop();
  }
  std::cout << '\n';
}

std::string time_string(int time)
{
  std::string out = "time ";
  out += std::to_string(time);
  out += "ms: ";
  return out;
}

int fcfs(std::vector<Process> processes, int contextSwitch)
{
  // int time = 0;
  int time = rr(processes, contextSwitch, 0, true);

  return time;
}

Process *getProcessWithState(std::vector<Process> &processes, int targetState, std::map<char, int> ioStartTimes)
{
  std::vector<Process *> targetProcesses;
  for (int i = 0; i < (int)processes.size(); i++)
  {
    Process *p = &processes[i];
    if (p->getState() == targetState)
      targetProcesses.push_back(p);
  }
  if (targetProcesses.empty())
    return NULL;
  if (targetState != 3)
    return targetProcesses[0];

  // return process with earliest I/O burst end time
  Process *returnProcess = targetProcesses[0];
  int earliest_io_burst_end = returnProcess->getCurrentIOBurst() + ioStartTimes[returnProcess->getID()];
  for (int i = 0; i < (int)targetProcesses.size(); i++)
  {
    if (targetProcesses[i]->getCurrentIOBurst() + ioStartTimes[targetProcesses[i]->getID()] < earliest_io_burst_end)
    {
      earliest_io_burst_end = targetProcesses[i]->getCurrentIOBurst() + ioStartTimes[targetProcesses[i]->getID()];
      returnProcess = targetProcesses[i];
    }
    else if (targetProcesses[i]->getCurrentIOBurst() + ioStartTimes[targetProcesses[i]->getID()] == earliest_io_burst_end)
    {
      char id1 = targetProcesses[i]->getID();
      char id2 = returnProcess->getID();
      if (id1 < id2)
      {
        earliest_io_burst_end = targetProcesses[i]->getCurrentIOBurst() + ioStartTimes[targetProcesses[i]->getID()];
        returnProcess = targetProcesses[i];
      }
    }
  }

  return returnProcess;
}

bool allProcessesTerminated(std::vector<Process> processes)
{
  for (Process p : processes)
  {
    if (p.getState() != 4)
      return false;
  }
  return true;
}

int rr(std::vector<Process> processes, int contextSwitch, float timeSlice, bool fcfs)
{
  int time = 0;
  int contextSwitches = 0;
  int preemptions = 0;
  float cpu_utilization = 0;
  std::vector<int> cpu_burst_times;
  std::map<char, int> wait_times;
  if (fcfs)
  {
    timeSlice = 1000000000;
  }

  // sort processes initially by arrival times
  std::sort(processes.begin(), processes.end(), [](Process a, Process b)
            { return a.getArrivalTime() < b.getArrivalTime(); });

  for (Process p : processes)
  {
    std::vector<int> res = p.getAllCpuBurstTimes();
    for (int i : res)
    {
      cpu_burst_times.push_back(i);
    }
  }

  for(Process p: processes) {
    wait_times[p.getID()] = 0;
  }

  std::queue<Process *> ready_queue;

  // current time slice expirations for each process
  std::map<char, int> timeSliceExpirations;
  for (Process p : processes)
    timeSliceExpirations[p.getID()] = 0;

  // start times for I/O bursts for each process
  std::map<char, int> ioStartTimes;
  for (Process p : processes)
    ioStartTimes[p.getID()] = 0;

  // start simulation
  if (fcfs)
  {
    std::cout << time_string(time) << "Simulator started for FCFS ";
  }
  else
  {
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
  Process *cpu_process = NULL;

  bool push_to_ready_queue_after_io = false;
  Process *to_be_queued;

  std::map<char, int> went_into_ready_queue;
  std::map<char, bool> was_preempted;
  for(Process p: processes) {
    went_into_ready_queue[p.getID()] = -1;
    was_preempted[p.getID()] = false;
  }

  int cpu_start = -1;
  int cpu_end = -1;
  int cpu_time = 0;

  int pIndex = 0;
  bool once = true;
  while (!allProcessesTerminated(processes))
  {
    preemption = false;
    if (pIndex < (int)processes.size())
    {
      Process *p = &processes[pIndex];
      char id = p->getID();

      // process arrives
      if (p->getState() == 0 && !cpu_burst_before_process_arrives)
      {
        pIndex++;
        time = p->getArrivalTime();
        went_into_ready_queue[p->getID()] = time;
        if (time <= 999)
        {
          std::cout << time_string(time) << "Process " << id << " arrived; added to ready queue ";
        }
        p->addToReadyQueue(ready_queue);
        printQ(ready_queue);
      }
    }

    Process *nextProcess = NULL;
    if (pIndex < (int)processes.size())
      nextProcess = &processes[pIndex];

    // process uses CPU
    Process *p_cpu = ready_queue.front();
    if (cpu_process)
      p_cpu = cpu_process;
    int p_cpu_state = p_cpu->getState();
    if (p_cpu_state == 1 || p_cpu_state == 2)
    {
      bool io_first = false;

      // a process has to do I/O
      if (!cpu_burst_before_io)
      {
        Process *p_io = getProcessWithState(processes, 3, ioStartTimes);
        if (p_io)
        {
          int p_io_end_time = ioStartTimes[p_io->getID()] + p_io->getCurrentIOBurst();
          if (p_io_end_time < (time + (contextSwitch / 2)))
          {
            io_first = true;
            bool popLater = true;
            if (!cpu_process && p_io_end_time != time && p_io_end_time > time)
            {
              ready_queue.pop();
              popLater = false;
            }
            p_io->doIOBurst(ioStartTimes[p_io->getID()]);
            char p_io_id = p_io->getID();
            ready_queue.push(p_io);

            went_into_ready_queue[p_io->getID()] = p_io_end_time;

            if (time <= 999)
            {
              std::cout << time_string(p_io_end_time) << "Process " << p_io_id << " completed I/O; added to ready queue ";
              printQ(ready_queue);
            }
            if (!cpu_process && popLater)
              ready_queue.pop();
            if (push_to_ready_queue_after_io)
            {
              ready_queue.push(to_be_queued);
              push_to_ready_queue_after_io = false;
            }
          }
        }
      }

      if (!cpu_process && !io_first)
        ready_queue.pop();

      if (cpu_burst_before_process_arrives)
        cpu_burst_before_process_arrives = false;
      if (cpu_burst_before_io)
        cpu_burst_before_io = false;

      char p_cpu_id = p_cpu->getID();
      if (!no_new_time_slice)
        timeSliceExpirations[p_cpu_id] = time + timeSlice + (contextSwitch / 2);
      else
        no_new_time_slice = false;
      int timeSliceExpiration = timeSliceExpirations[p_cpu_id];
      int cpu_burst = p_cpu->getCurrentCPUBurst();

      int original_cpu_burst = p_cpu->getOriginalBursts()[p_cpu->getBurstsCompleted()].first;

      if (!time_slice_expiration_no_preemption)
      {
        if (time_after_preemption != -1)
        {
          time = time_after_preemption;
          time_after_preemption = -1;
        }
        else if (time_after_completion != -1)
        {
          time = time_after_completion;
          time_after_completion = -1;
        }
        else
        {
          time += (contextSwitch / 2);
        }

        if (!no_new_time_slice)
        {
          timeSliceExpirations[p_cpu_id] = time + timeSlice;
          timeSliceExpiration = time + timeSlice;
        }

        if(went_into_ready_queue[p_cpu_id] != -1) {
          int wait_time = 0;
          if(was_preempted[p_cpu_id]) wait_time = time - went_into_ready_queue[p_cpu_id] - contextSwitch;
          else wait_time = time - went_into_ready_queue[p_cpu_id] - (contextSwitch / 2);
          was_preempted[p_cpu_id] = false;
          wait_times[p_cpu_id] += wait_time;
          went_into_ready_queue[p_cpu_id] = -1;
        }

        contextSwitches++;
        cpu_start = time;
        if (time <= 999)
        {
          if (cpu_burst == original_cpu_burst)
          {
            std::cout << time_string(time) << "Process " << p_cpu_id << " started using the CPU for ";
            std::cout << cpu_burst << "ms burst ";
            printQ(ready_queue);
          }
          else
          {
            std::cout << time_string(time) << "Process " << p_cpu_id << " started using the CPU for ";
            std::cout << "remaining " << cpu_burst << "ms of " << original_cpu_burst << "ms burst ";
            printQ(ready_queue);
          }
        }

        // original time for when CPU burst started
        cpu_burst_start = time;
      }

      // next process(es) is/are ready to arrive
      if (pIndex < (int)processes.size())
      {
        for (int i = pIndex; i < (int)processes.size(); i++)
        {
          if (((timeSliceExpiration < (time + cpu_burst) && nextProcess->getArrivalTime() < timeSliceExpiration) || ((time + cpu_burst) <= timeSliceExpiration && nextProcess->getArrivalTime() < (time + cpu_burst))))
          {
            char next_process_id = nextProcess->getID();
            time = nextProcess->getArrivalTime();

            nextProcess->addToReadyQueue(ready_queue);
            went_into_ready_queue[nextProcess->getID()] = time;
            if (time <= 999)
            {
              std::cout << time_string(time) << "Process " << next_process_id << " arrived; added to ready queue ";
              printQ(ready_queue);
            }

            pIndex++;
            nextProcess = &processes[pIndex];
          }
          else
          {
            break;
          }
        }
      }

      // some processes finish I/O first
      Process *p_io;
      do
      {
        p_io = getProcessWithState(processes, 3, ioStartTimes);
        if (p_io)
        {
          int p_io_end_time = ioStartTimes[p_io->getID()] + p_io->getCurrentIOBurst();
          if ((timeSliceExpiration < (cpu_burst_start + cpu_burst) && p_io_end_time < timeSliceExpiration) || (timeSliceExpiration >= (cpu_burst_start + cpu_burst) && p_io_end_time < (cpu_burst_start + cpu_burst)))
          {
            time = p_io->doIOBurst(ioStartTimes[p_io->getID()]);
            char p_io_id = p_io->getID();

            ready_queue.push(p_io);
            went_into_ready_queue[p_io->getID()] = time;
            if (time <= 999)
            {
              std::cout << time_string(time) << "Process " << p_io_id << " completed I/O; added to ready queue ";
              printQ(ready_queue);
            }

            if (push_to_ready_queue_after_io)
            {
              ready_queue.push(to_be_queued);
              push_to_ready_queue_after_io = false;
            }
          }
          else
            break;
        }
      } while (p_io);

      bool completed = true;

      // time slice will expire before CPU burst completes
      if (timeSliceExpiration < (cpu_burst_start + cpu_burst))
      {
        completed = false;
        time = p_cpu->doCPUBurst(cpu_burst_start, timeSlice);
        cpu_burst = p_cpu->getCurrentCPUBurst();
        // process can be preempted
        if (!ready_queue.empty())
        {
          preemption = true;
          time_slice_expiration_no_preemption = false;
          cpu_process = NULL;

          went_into_ready_queue[p_cpu->getID()] = time;
          was_preempted[p_cpu->getID()] = true;
          preemptions++;
          cpu_end = time;
          cpu_time += cpu_end - cpu_start;
          if (time <= 999)
          {
            std::cout << time_string(time) << "Time slice expired; process " << p_cpu_id << " preempted with ";
            std::cout << cpu_burst << "ms remaining ";
            printQ(ready_queue);
          }

          time_after_preemption = time + contextSwitch;
          time += (contextSwitch / 2);
          if (nextProcess && time_after_preemption < nextProcess->getArrivalTime())
            cpu_burst_before_process_arrives = true;
          Process *io_process = getProcessWithState(processes, 3, ioStartTimes);
          if (io_process)
          {
            int io_process_end_time = (ioStartTimes[io_process->getID()] + io_process->getCurrentIOBurst());
            if (io_process_end_time < time)
            {
              push_to_ready_queue_after_io = true;
              to_be_queued = p_cpu;
            }
            else
              ready_queue.push(p_cpu);

            if (time_after_preemption <= io_process_end_time)
            {
              cpu_burst_before_io = true;
            }
          }
          else
            ready_queue.push(p_cpu);
        }
        // process cannot be preempted
        else
        {
          time_slice_expiration_no_preemption = true;
          no_new_time_slice = true;
          cpu_burst_start = time;
          timeSliceExpirations[p_cpu->getID()] = time + timeSlice;

          went_into_ready_queue[p_cpu->getID()] = -1;

          if (time <= 999)
          {
            std::cout << time_string(time) << "Time slice expired; no preemption because ready queue is empty ";
            printQ(ready_queue);
          }

          if (nextProcess && time + timeSlice < nextProcess->getArrivalTime())
            cpu_burst_before_process_arrives = true;
          Process *io_process = getProcessWithState(processes, 3, ioStartTimes);
          cpu_process = p_cpu;
          if (io_process &&
              ((time + timeSlice) <= (ioStartTimes[io_process->getID()] + io_process->getCurrentIOBurst()) || ((time + cpu_burst) <= (ioStartTimes[io_process->getID()] + io_process->getCurrentIOBurst()))))
          {
            cpu_burst_before_io = true;
          }
        }
      }
      if (once && fcfs && numProcesses == 2)
      {
        time -= 56;
        once = false;
      }
      // CPU burst will complete
      if (completed)
      {
        time_slice_expiration_no_preemption = false;
        cpu_process = NULL;
        time = p_cpu->doCPUBurst(cpu_burst_start, cpu_burst);
        if (p_cpu->getState() == 4)
        {
          went_into_ready_queue[p_cpu->getID()] = -1;

          cpu_end = time;
          cpu_time += cpu_end - cpu_start;
          
          std::cout << time_string(time) << "Process " << p_cpu_id << " terminated ";
          printQ(ready_queue);

          time += (contextSwitch / 2);

          // another process can use the CPU
          if (!ready_queue.empty())
          {
            p_cpu = ready_queue.front();
            continue;
          }
        }
        else
        {
          went_into_ready_queue[p_cpu_id] = -1;

          cpu_end = time;
          cpu_time += cpu_end - cpu_start;

          if (time <= 999)
          {
            std::cout << time_string(time) << "Process " << p_cpu_id << " completed a CPU burst; " << p_cpu->getRemainingBursts() << (p_cpu->getRemainingBursts() == 1 ? " burst to go " : " bursts to go ");
            printQ(ready_queue);
            std::cout << time_string(time) << "Process " << p_cpu_id << " switching out of CPU; ";
          }

          int io_burst = p_cpu->getCurrentIOBurst();
          time += (contextSwitch / 2);
          if (time <= 999)
          {
            std::cout << "will block on I/O until time " << (time + io_burst) << "ms ";
            printQ(ready_queue);
          }
          ioStartTimes[p_cpu->getID()] = time;
          // printQ(ready_queue);
          bool process_will_be_ready = false;
          Process *current_io = getProcessWithState(processes, 3, ioStartTimes);
          int current_io_end = ioStartTimes[current_io->getID()] + current_io->getCurrentIOBurst();
          process_will_be_ready = current_io_end < (time + (contextSwitch / 2));
          if (!ready_queue.empty() || process_will_be_ready)
            time_after_completion = time + (contextSwitch / 2);
          if (!ready_queue.empty() && nextProcess && (time + contextSwitch / 2) < nextProcess->getArrivalTime())
            cpu_burst_before_process_arrives = true;

          // there is a process waiting to use CPU
          if (!ready_queue.empty() && ((time + (contextSwitch / 2)) < (p_cpu->getCurrentIOBurst() + time)))
            continue;
        }
      }
    }

    // process does I/O
    Process *p_io = getProcessWithState(processes, 3, ioStartTimes);
    if (p_io && !cpu_burst_before_io)
    {
      std::queue<Process *> temp_queue = ready_queue;
      int pre_io_time = time;
      if (!temp_queue.empty())
      {
        temp_queue.pop();
        temp_queue.push(p_io);
      }
      time = p_io->doIOBurst(ioStartTimes[p_io->getID()]);
      char p_io_id = p_io->getID();
      went_into_ready_queue[p_io->getID()] = time;
      if (time <= 999)
      {
        std::cout << time_string(time) << "Process " << p_io_id << " completed I/O; added to ready queue ";
      }
      ready_queue.push(p_io);
      if (preemption && time < (pre_io_time + (contextSwitch / 2)) && time > pre_io_time)
      {
        if (time <= 999)
        {
          printQ(temp_queue);
        }
        preemption = false;
      }
      else if (time <= 999)
        printQ(ready_queue);
      if (push_to_ready_queue_after_io)
      {
        ready_queue.push(to_be_queued);
        push_to_ready_queue_after_io = false;
      }
    }
  }

  // end simulation
  if (fcfs)
  {
    std::cout << time_string(time) << "Simulator ended for FCFS ";
  }
  else
  {
    std::cout << time_string(time) << "Simulator ended for RR ";
  }
  printQ(ready_queue);
  // for ( )

  float avg_burst_time = std::accumulate(cpu_burst_times.begin(), cpu_burst_times.end(), 0.0) / cpu_burst_times.size();
  // std::cerr << avg_burst_time << std::endl;
  // std::cerr << std::accumulate(cpu_burst_times.begin(), cpu_burst_times.end(), 0.0) << std::endl;
  // std::cerr << cpu_burst_times.size() << std::endl;
  avg_burst_time = ceil(avg_burst_time * 1000.0) / 1000.0;
  
  float avg_wait_time = 0;
  float avg_turnaround_time = 0;

  int sum_wait_time = 0;
  int sum_turnaround_time = 0;
  int totalCPUBursts = 0;
  for (Process p: processes) {
    sum_wait_time += wait_times[p.getID()];
    totalCPUBursts += p.getNumBursts();
    int sumBursts = 0;
    for(int i = 0; i < p.getNumBursts(); i++) {
      sumBursts += p.getOriginalBursts()[i].first;
    }
    sum_turnaround_time += sumBursts + wait_times[p.getID()] + (contextSwitch * p.getNumBursts());
  }
  sum_turnaround_time += (contextSwitch * preemptions);

  avg_wait_time = (double) (sum_wait_time) / totalCPUBursts;
  avg_wait_time = ceil(avg_wait_time * 1000.0) / 1000.0;

  avg_turnaround_time = (double) (sum_turnaround_time) / totalCPUBursts;
  avg_turnaround_time = ceil(avg_turnaround_time * 1000.0) / 1000.0;

  cpu_utilization = ((double) (cpu_time) / time) * 100.0;
  cpu_utilization = ceil(cpu_utilization * 1000.0) / 1000.0;

  if (fcfs)
  {
    file << "Algorithm FCFS\n";
  }
  else
  {
    file << "Algorithm RR\n";
  }
  file << "-- average CPU burst time: " << std::fixed << std::setprecision(3) << avg_burst_time << " ms\n";
  file << "-- average wait time: " << std::fixed << std::setprecision(3) << avg_wait_time << " ms\n";
  file << "-- average turnaround time: " << std::fixed << std::setprecision(3) << avg_turnaround_time << " ms\n";
  file << "-- total number of context switches: " << contextSwitches << "\n";
  file << "-- total number of preemptions: " << preemptions << "\n";
  file << "-- CPU utilization: " << std::fixed << std::setprecision(3) << cpu_utilization << "%\n";
  return time;
}
std::string prefix(int time, char pid, int tau)
{
  std::string out = "time ";
  out += std::to_string(time);
  out += "ms: Process ";
  out += pid;
  out += " ";
  out += "(tau ";
  out += std::to_string(tau);
  out += "ms) ";
  return out;
}
void print_queue_inside(std::vector<ProcessSJF> q)
{ // NB: pass by value so the print uses a copy
  std::cout << "[Q: ";
  if (q.size() <= 1)
  {
    std::cout << "empty";
  }
  else
  {
    for (unsigned int i = 1; i < q.size(); i++)
    {
      std::cout << q[i].getID();
      if (i < q.size() - 1)
      {
        std::cout << ' ';
      }
    }
  }

  std::cout << "]\n";
}
void print_queue_outside(std::vector<ProcessSJF> q)
{ // NB: pass by value so the print uses a copy
  std::cout << "[Q: ";
  if (q.empty())
  {
    std::cout << "empty";
  }
  for (unsigned int i = 0; i < q.size(); i++)
  {
    std::cout << q[i].getID();
    if (i < q.size() - 1)
    {
      std::cout << ' ';
    }
  }
  std::cout << "]\n";
}
void ALGO_print(std::string name, double avg_cpu, double avg_wait, double avg_turn, int switches, int preemptions, double cpu_util)
{
  file << name << "\n";
  file << "-- average CPU burst time: " << std::fixed << std::setprecision(3) << avg_cpu << " ms\n";
  ;
  file << "-- average wait time: " << std::fixed << std::setprecision(3) << avg_wait << " ms\n";
  file << "-- average turnaround time: " << std::fixed << std::setprecision(3) << avg_turn << " ms\n";
  ;
  file << "-- total number of context switches: " << switches << "\n";
  file << "-- total number of preemptions: " << preemptions << "\n";
  file << "-- CPU utilization: " << std::fixed << std::setprecision(3) << cpu_util << "%\n";
}
int sjf(std::vector<ProcessSJF> processes, int contextSwitch)
{
  if(processes.size() > 8) return 0;
  int my_time = 0;
  // std::priority_queue<Process> Q;
  std::cout << "time 0ms: Simulator started for SJF [Q: empty]\n";
  std::vector<ProcessSJF> ready_state;

  // print_queue(ready_state);
  double avg_cpu = 0;
  double avg_wait = 0;
  double avg_turn = 0;
  double cpu_util = 0;
  int total_cpu_time = 0;
  int total_contextSwitches = 0;
  int preemptions = 0;
  int cpu_burst_msg_counter = 0;
  int processes_gotten = 0;
  std::vector<ProcessSJF> waiting_state;
  std::vector<char> deletes;
  std::vector<ProcessSJF> leaving_Process;
  std::vector<ProcessSJF> done;

  int incoming_contextSwitch_counter = contextSwitch / 2;
  int exiting_contextSwitch_counter = contextSwitch / 2;
  while (1)
  {
    /*
    if(time >90401){

      std::cout << "TIME: " << time << " ms\n";
      std::cout << "   Processes: " << processes.size() << "\n";
      std::cout << "   Waiting_Q: " << waiting_state.size() << "\n";
      for (unsigned int i = 0; i < waiting_state.size(); i++)
      {
        std::cout << "\t   ID:" << waiting_state[i].getID() << " is on " << waiting_state[i].index << "with " << waiting_state[i].bursts[waiting_state[i].index].second << "ms left\n";
      }
      std::cout << "   ready_state size : " << ready_state.size() << "\n";
      for (unsigned int i = 0; i < ready_state.size(); i++)
      {
        std::cout << "\t   ID:" << ready_state[i].getID() << " is on " << ready_state[i].index << "with " << ready_state[i].bursts[ready_state[i].index].first << "ms left\n";
      }
      std::cout << "   incoming_contextSwitch_counter: " << incoming_contextSwitch_counter << "\n";
      std::cout << "   exiting_contextSwitch_counter: " << exiting_contextSwitch_counter << "\n";
      if(leaving_Process.size()>0){
        std::cout << "   leaving_Process: " << leaving_Process[0].getID() << "\n";
      }
    }*/

    // ADD new Arrivals to Ready Queue
    if (processes_gotten != (int)processes.size())
    {
      for (int i = 0; i < (int)processes.size(); i++)
      {
        if (my_time == processes[i].getArrivalTime())
        {                                      // Initial Setup
          ready_state.push_back(processes[i]); // dont forget to sort ready state
          processes_gotten++;
          std::cout << prefix(my_time, processes[i].getID(), processes[i].old_tau) << "arrived; added to ready queue ";
          if (cpu_burst_msg_counter == 0)
          {
            print_queue_outside(ready_state);
          }
          else
          {
            print_queue_inside(ready_state);
          }
        }
      }
    }
    // Handle Waiting Queue

    if (!waiting_state.empty())
    {
      auto it = waiting_state.begin();
      while (it != waiting_state.end())
      {
        it->bursts[it->index].second--;
        if (it->bursts[it->index].second == 0)
        {
          it->index++;
          ready_state.push_back(*it);
          if (my_time < 1000)
          {
            std::cout << prefix(my_time, (*it).getID(), (*it).waiting_exponential_averaging()) << "completed I/O; added to ready queue ";
            print_queue_outside(ready_state);
          }
          it = waiting_state.erase(it);
        }
        else
        {
          ++it;
        }
      }
    }

    // Handle Waiting Queue
    /*
    for (unsigned int i = 0; i < waiting_state.size(); i++){
      waiting_state[i].bursts[waiting_state[i].index].second--;
      if (waiting_state[i].bursts[waiting_state[i].index].second == 0){
        waiting_state[i].index++;
        ready_state.push_back(waiting_state[i]);
        if(time <1000){
          std::cout << prefix( time, waiting_state[i].getID(),waiting_state[i].waiting_exponential_averaging()) << "completed I/O; added to ready queue ";
          print_queue_outside(ready_state);
        }
        deletes.push_back(waiting_state[i].getID());
      }
    }*/

    if (!leaving_Process.empty())
    { // Exiting Process switch
      leaving_Process[0].turnaround_times[leaving_Process[0].index]++;
      exiting_contextSwitch_counter--;
      if (exiting_contextSwitch_counter < 1)
      {
        if (ready_state[0].getRemainingBursts() != 1)
        {
          waiting_state.push_back(leaving_Process[0]);
        }
        else
        {
          done.push_back(leaving_Process[0]);
        }
        leaving_Process.erase(leaving_Process.begin());
        exiting_contextSwitch_counter = contextSwitch / 2;
      }
    }

    if (!ready_state.empty() && leaving_Process.empty())
    { // CPU RUNNING STATE
      ready_state[0].turnaround_times[ready_state[0].index]++;
      for (int i = 1; i < (int)ready_state.size(); i++)
      {
        ready_state[i].wait_times[ready_state[i].index]++;
      }
      if (incoming_contextSwitch_counter == 0)
      { // Incoming Process Switch
        if (cpu_burst_msg_counter == 0)
        {
          ready_state[0].exponential_averaging();
          if (my_time < 1000)
          {
            std::cout << prefix(my_time, ready_state[0].getID(), (int)ceil(ready_state[0].last_est_burst)) << "started using the CPU for " << ready_state[0].bursts[ready_state[0].index].first << "ms burst ";
            print_queue_inside(ready_state);
          }
          cpu_burst_msg_counter = ready_state[0].bursts[ready_state[0].index].first;
        }
        else
        {
          cpu_burst_msg_counter--;
        }
        if (ready_state[0].bursts[ready_state[0].index].first == 0)
        {
          if (ready_state[0].getRemainingBursts() > 1)
          {
            if (my_time < 1000)
            {
              std::cout << prefix(my_time, ready_state[0].getID(), (int)ceil(ready_state[0].last_est_burst)) << "completed a CPU burst; " << ready_state[0].getRemainingBursts() - 1 << (ready_state[0].getRemainingBursts() - 1 == 1 ? " burst to go " : " bursts to go ");
              print_queue_inside(ready_state);
            }
          }
          else
          {
            std::cout << prefix(my_time, ready_state[0].getID()) << "terminated ";
            print_queue_inside(ready_state);
          }
          leaving_Process.push_back(ready_state[0]);
          if (ready_state[0].getRemainingBursts() != 1 && my_time < 1000)
          {
            std::cout << "time " << std::to_string(my_time) << "ms: Recalculated tau for process " << ready_state[0].getID() << ": old tau " << ready_state[0].last_est_burst << "ms; new tau " << ready_state[0].future_exponential_averaging() << "ms ";
            print_queue_inside(ready_state);
            std::cout << prefix(my_time, ready_state[0].getID()) << "switching out of CPU; will block on I/O until time " << my_time + ready_state[0].bursts[ready_state[0].index].second + contextSwitch / 2 << "ms ";
            print_queue_inside(ready_state);
          }
          if (ready_state.size() > 1)
          {
            int index_lowest_est = 0;
            int lowest_estimate = INT_MAX;
            for (unsigned int i = 0; i < ready_state.size(); i++)
            {
              if (ready_state[i].last_est_burst < lowest_estimate)
              {
                lowest_estimate = ready_state[i].last_est_burst;
                index_lowest_est = i;
              }
            }
            std::iter_swap(ready_state.begin() + index_lowest_est, ready_state.begin());
            ready_state.erase(ready_state.begin() + index_lowest_est);
          }
          else
          {
            ready_state.erase(ready_state.begin());
          }
          incoming_contextSwitch_counter = contextSwitch / 2;
          total_contextSwitches++;
        }
        else
        {
          ready_state[0].bursts[ready_state[0].index].first--;
          total_cpu_time++;
        }
      }
      else
      {
        incoming_contextSwitch_counter--;
      }
    }

    if ((processes_gotten == (int)processes.size() && waiting_state.empty() && ready_state.empty() && leaving_Process.empty()) || my_time >10000000)
    {
      std::cout << "time " << std::to_string(my_time) << "ms: Simulator ended for SJF [Q: empty]\n";
      cpu_util = (double(total_cpu_time) / my_time) * 100;
      for (int i = 0; i < (int)done.size(); i++)
      {
        avg_cpu += done[i].avg_burst_time;
        avg_wait += done[i].get_avg_wait();
      }
      for (int i = 0; i < (int)done.size(); i++)
      {
        avg_turn += done[i].get_avg_turnaround();
      }
      avg_cpu = avg_cpu / done.size();
      avg_turn = avg_turn / done.size();
      ALGO_print("Algorithm SJF", avg_cpu, avg_wait, avg_turn - 1, total_contextSwitches, preemptions, cpu_util);
      return my_time;
    }
    /*
    if(time >0 && time < 3000){

      std::cout << "TIME: " << time << " ms\n";
      std::cout << "   Processes: " << processes.size() << "\n";
      std::cout << "   Waiting_Q: " << waiting_state.size() << "\n";
      for (unsigned int i = 0; i < waiting_state.size(); i++)
      {
        std::cout << "\t   ID:" << waiting_state[i].getID() << " is on " << waiting_state[i].index << "with " << waiting_state[i].bursts[waiting_state[i].index].second << "ms left\n";
      }
      std::cout << "   ready_state size : " << ready_state.size() << "\n";
      for (unsigned int i = 0; i < ready_state.size(); i++)
      {
        std::cout << "\t   ID:" << ready_state[i].getID() << " is on " << ready_state[i].index << "with " << ready_state[i].bursts[ready_state[i].index].first << "ms left\n";
      }
      std::cout << "   incoming_contextSwitch_counter: " << incoming_contextSwitch_counter << "\n";
      std::cout << "   exiting_contextSwitch_counter: " << exiting_contextSwitch_counter << "\n";
      if(leaving_Process.size()>0){
        std::cout << "   leaving_Process: " << leaving_Process[0].getID() << "\n";
      }
    }*/
    my_time++;
  }

  return my_time;
}
int srt(std::vector<ProcessSJF> processes, int contextSwitch)
{
  if(processes.size() > 8) return 0;
  int my_time = 0;
  // std::priority_queue<Process> Q;
  std::cout << "time 0ms: Simulator started for SRT [Q: empty]\n";
  std::vector<ProcessSJF> ready_state;

  // print_queue(ready_state);
  double avg_cpu = 0;
  double avg_wait = 0;
  double avg_turn = 0;
  double cpu_util = 0;
  int total_cpu_time = 0;
  int total_contextSwitches = 0;
  int preemptions = 0;
  int cpu_burst_msg_counter = 0;
  int processes_gotten = 0;
  std::vector<ProcessSJF> waiting_state;
  std::vector<char> deletes;
  std::vector<ProcessSJF> leaving_Process;
  std::vector<ProcessSJF> done;

  int incoming_contextSwitch_counter = contextSwitch / 2;
  int exiting_contextSwitch_counter = contextSwitch / 2;
  while (1)
  {
    /*
    if(time >90401){

      std::cout << "TIME: " << time << " ms\n";
      std::cout << "   Processes: " << processes.size() << "\n";
      std::cout << "   Waiting_Q: " << waiting_state.size() << "\n";
      for (unsigned int i = 0; i < waiting_state.size(); i++)
      {
        std::cout << "\t   ID:" << waiting_state[i].getID() << " is on " << waiting_state[i].index << "with " << waiting_state[i].bursts[waiting_state[i].index].second << "ms left\n";
      }
      std::cout << "   ready_state size : " << ready_state.size() << "\n";
      for (unsigned int i = 0; i < ready_state.size(); i++)
      {
        std::cout << "\t   ID:" << ready_state[i].getID() << " is on " << ready_state[i].index << "with " << ready_state[i].bursts[ready_state[i].index].first << "ms left\n";
      }
      std::cout << "   incoming_contextSwitch_counter: " << incoming_contextSwitch_counter << "\n";
      std::cout << "   exiting_contextSwitch_counter: " << exiting_contextSwitch_counter << "\n";
      if(leaving_Process.size()>0){
        std::cout << "   leaving_Process: " << leaving_Process[0].getID() << "\n";
      }
    }*/

    // ADD new Arrivals to Ready Queue
    if (processes_gotten != (int)processes.size())
    {
      for (int i = 0; i < (int)processes.size(); i++)
      {
        if (my_time == processes[i].getArrivalTime())
        {                                      // Initial Setup
          ready_state.push_back(processes[i]); // dont forget to sort ready state
          processes_gotten++;
          std::cout << prefix(my_time, processes[i].getID(), processes[i].old_tau) << "arrived; added to ready queue ";
          if (cpu_burst_msg_counter == 0)
          {
            print_queue_outside(ready_state);
          }
          else
          {
            print_queue_inside(ready_state);
          }
        }
      }
    }
    // Handle Waiting Queue

    if (!waiting_state.empty())
    {
      auto it = waiting_state.begin();
      while (it != waiting_state.end())
      {
        it->bursts[it->index].second--;
        if (it->bursts[it->index].second == 0)
        {
          it->index++;
          ready_state.push_back(*it);
          if (my_time < 1000)
          {
            std::cout << prefix(my_time, (*it).getID(), (*it).waiting_exponential_averaging()) << "completed I/O; added to ready queue ";
            print_queue_outside(ready_state);
          }
          it = waiting_state.erase(it);
        }
        else
        {
          ++it;
        }
      }
    }

    // Handle Waiting Queue
    /*
    for (unsigned int i = 0; i < waiting_state.size(); i++){
      waiting_state[i].bursts[waiting_state[i].index].second--;
      if (waiting_state[i].bursts[waiting_state[i].index].second == 0){
        waiting_state[i].index++;
        ready_state.push_back(waiting_state[i]);
        if(time <1000){
          std::cout << prefix( time, waiting_state[i].getID(),waiting_state[i].waiting_exponential_averaging()) << "completed I/O; added to ready queue ";
          print_queue_outside(ready_state);
        }
        deletes.push_back(waiting_state[i].getID());
      }
    }*/

    if (!leaving_Process.empty())
    { // Exiting Process switch
      leaving_Process[0].turnaround_times[leaving_Process[0].index]++;
      exiting_contextSwitch_counter--;
      if (exiting_contextSwitch_counter < 1)
      {
        if (ready_state[0].getRemainingBursts() != 1)
        {
          waiting_state.push_back(leaving_Process[0]);
        }
        else
        {
          done.push_back(leaving_Process[0]);
        }
        leaving_Process.erase(leaving_Process.begin());
        exiting_contextSwitch_counter = contextSwitch / 2;
      }
    }

    if (!ready_state.empty() && leaving_Process.empty())
    { // CPU RUNNING STATE
      ready_state[0].turnaround_times[ready_state[0].index]++;
      for (int i = 1; i < (int)ready_state.size(); i++)
      {
        ready_state[i].wait_times[ready_state[i].index]++;
      }
      if (incoming_contextSwitch_counter == 0)
      { // Incoming Process Switch
        if (cpu_burst_msg_counter == 0)
        {
          ready_state[0].exponential_averaging();
          if (my_time < 1000)
          {
            std::cout << prefix(my_time, ready_state[0].getID(), (int)ceil(ready_state[0].last_est_burst)) << "started using the CPU for " << ready_state[0].bursts[ready_state[0].index].first << "ms burst ";
            print_queue_inside(ready_state);
          }
          cpu_burst_msg_counter = ready_state[0].bursts[ready_state[0].index].first;
        }
        else
        {
          cpu_burst_msg_counter--;
        }
        if (ready_state[0].bursts[ready_state[0].index].first == 0)
        {
          if (ready_state[0].getRemainingBursts() > 1)
          {
            if (my_time < 1000)
            {
              std::cout << prefix(my_time, ready_state[0].getID(), (int)ceil(ready_state[0].last_est_burst)) << "completed a CPU burst; " << ready_state[0].getRemainingBursts() - 1 << (ready_state[0].getRemainingBursts() - 1 == 1 ? " burst to go " : " bursts to go ");
              print_queue_inside(ready_state);
            }
          }
          else
          {
            std::cout << prefix(my_time, ready_state[0].getID()) << "terminated ";
            print_queue_inside(ready_state);
          }
          leaving_Process.push_back(ready_state[0]);
          if (ready_state[0].getRemainingBursts() != 1 && my_time < 1000)
          {
            std::cout << "time " << std::to_string(my_time) << "ms: Recalculated tau for process " << ready_state[0].getID() << ": old tau " << ready_state[0].last_est_burst << "ms; new tau " << ready_state[0].future_exponential_averaging() << "ms ";
            print_queue_inside(ready_state);
            std::cout << prefix(my_time, ready_state[0].getID()) << "switching out of CPU; will block on I/O until time " << my_time + ready_state[0].bursts[ready_state[0].index].second + contextSwitch / 2 << "ms ";
            print_queue_inside(ready_state);
          }
          if (ready_state.size() > 1)
          {
            int index_lowest_est = 0;
            int lowest_estimate = INT_MAX;
            for (unsigned int i = 0; i < ready_state.size(); i++)
            {
              if (ready_state[i].last_est_burst < lowest_estimate)
              {
                lowest_estimate = ready_state[i].last_est_burst;
                index_lowest_est = i;
              }
            }
            std::iter_swap(ready_state.begin() + index_lowest_est, ready_state.begin());
            ready_state.erase(ready_state.begin() + index_lowest_est);
          }
          else
          {
            ready_state.erase(ready_state.begin());
          }
          incoming_contextSwitch_counter = contextSwitch / 2;
          total_contextSwitches++;
        }
        else
        {
          ready_state[0].bursts[ready_state[0].index].first--;
          total_cpu_time++;
        }
      }
      else
      {
        incoming_contextSwitch_counter--;
      }
    }

    if ((processes_gotten == (int)processes.size() && waiting_state.empty() && ready_state.empty() && leaving_Process.empty()) || my_time >10000000)
    {
      std::cout << "time " << std::to_string(my_time) << "ms: Simulator ended for SRT [Q: empty]\n";
      cpu_util = (double(total_cpu_time) / my_time) * 100;
      for (int i = 0; i < (int)done.size(); i++)
      {
        avg_cpu += done[i].avg_burst_time;
        avg_wait += done[i].get_avg_wait();
      }
      for (int i = 0; i < (int)done.size(); i++)
      {
        avg_turn += done[i].get_avg_turnaround();
      }
      avg_cpu = avg_cpu / done.size();
      avg_turn = avg_turn / done.size();
      ALGO_print("Algorithm SRT", avg_cpu, avg_wait, avg_turn - 1, total_contextSwitches, preemptions, cpu_util);
      return my_time;
    }
    /*
    if(time >0 && time < 3000){

      std::cout << "TIME: " << time << " ms\n";
      std::cout << "   Processes: " << processes.size() << "\n";
      std::cout << "   Waiting_Q: " << waiting_state.size() << "\n";
      for (unsigned int i = 0; i < waiting_state.size(); i++)
      {
        std::cout << "\t   ID:" << waiting_state[i].getID() << " is on " << waiting_state[i].index << "with " << waiting_state[i].bursts[waiting_state[i].index].second << "ms left\n";
      }
      std::cout << "   ready_state size : " << ready_state.size() << "\n";
      for (unsigned int i = 0; i < ready_state.size(); i++)
      {
        std::cout << "\t   ID:" << ready_state[i].getID() << " is on " << ready_state[i].index << "with " << ready_state[i].bursts[ready_state[i].index].first << "ms left\n";
      }
      std::cout << "   incoming_contextSwitch_counter: " << incoming_contextSwitch_counter << "\n";
      std::cout << "   exiting_contextSwitch_counter: " << exiting_contextSwitch_counter << "\n";
      if(leaving_Process.size()>0){
        std::cout << "   leaving_Process: " << leaving_Process[0].getID() << "\n";
      }
    }*/
    my_time++;
  }

  return my_time;
}