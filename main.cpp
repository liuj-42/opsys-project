#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <vector>
#include <algorithm>

// #include "next_exp.cpp"
#include "process.h"
char alphabet[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'};

int fcfs(std::vector<Process> processes, int contextSwitch);
int sjf(std::vector<Process> processes, int contextSwitch);
void ALGO_print(std::string name, double avg_cpu, double avg_wait, double avg_turn, int switches, int preemptions, double cpu_util);
int main(int argc, char **argv)
{
  if (argc != 8)
  {
    std::cerr << "invalid arguments" << std::endl;
    return EXIT_FAILURE;
  }
  int numProcesses = atoi(*(argv + 1));
  int seed = atoi(*(argv + 2));
  double lambda = strtod(*(argv + 3), NULL);
  int upperBound = atoi(*(argv + 4));
  float contextSwitch = atof(*(argv + 5));
  float alpha = atof(*(argv + 6));
  float timeSlice = atof(*(argv + 7));

#if 1 // debugging
  std::string names[7] = {"processes\t\t\t", "seed\t\t\t\t", "lambda\t\t\t\t", "upper bound\t\t\t", "context switch time\t", "alpha\t\t\t\t", "time slice\t\t\t"};

  for (int i = 0; i < 7; i++)
  {
    std::cout << *(names + i) << "| " << *(argv + i + 1) << std::endl;
  }

  std::cout << "\n\n------------------------------\n";
#endif

  srand48(seed);

  // generate processes
  std::vector<Process> processes;
  for (int i = 0; i < numProcesses; i++)
  {
    processes.push_back(Process(alphabet[i], seed, lambda, upperBound, alpha));
  }

  int time = 0;
  // FCFS
  // time += fcfs( processes, contextSwitch );
  time += sjf(processes, contextSwitch);
  std::cout<<"TIME"<<time<<"\n";
  return 0;
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

template <typename T>
void print_queue(std::vector<Process> q)
{ // NB: pass by value so the print uses a copy
  for(unsigned int i=0; i< q.size(); i++)
  {
    std::cout << q[i] << ' ';
  }
  std::cout << '\n';
}

int sjf(std::vector<Process> processes, int contextSwitch)
{
  int time = 0;
  // std::priority_queue<Process> Q;
  std::cout << "time 0ms: Simulator started for SJF [Q: empty]\n";
  std::vector<Process> ready_state;

  // print_queue(ready_state);
  double avg_cpu = 0;
  double avg_wait = 0;
  double avg_turn = 0;
  double cpu_util = 0;
  int cpu_burst_msg_counter=0;
  std::vector<Process> waiting_state;
  std::vector<char> deletes;
  int running = 0;
  while (1)
  {   
    /*
      std::cout << "TIME: " << time << " ms\n";
      std::cout << "   Processes: " << processes.size() << "\n";
      std::cout << "   Waiting_Q: " << waiting_state.size() << "\n";
      for (unsigned int i = 0; i < waiting_state.size(); i++)
      {
        std::cout << "\t   ID:" << waiting_state[i].getID() << " is on " << waiting_state[i].index << "with " << waiting_state[i].bursts[waiting_state[i].index].second << "\n";
      }
      std::cout << "   ready_state: " << ready_state.size() << "\n";
    */
    // ADD new Arrivals to Ready Queue
    if (!processes.empty())
    {
      for (unsigned int i = 0; i < processes.size(); i++)
      {
        if (time == processes[i].getArrivalTime())
        { // Initial Setup
          ready_state.push_back(processes[i]);//dont forget to sort ready state
          deletes.push_back(processes[i].getID());
          std::cout << prefix( time, processes[i].getID() ) << "arrived; added to ready queue \n";
        }
      }
      // Delete From Processes
      for (unsigned int i = 0; i < deletes.size(); i++)
      {
        if (processes[i].getID() == deletes[0])
        {
          processes.erase(processes.begin() + i);
          deletes.erase(deletes.begin());
          i--;
        }
      }
    }
    //Calculate tau values for all processes in ready_state
    //find the lowest estimate
    int index_lowest_est =0;
    int estimate = INT_MAX;
    for(unsigned int i=0; i<ready_state.size();i++){
      int value = ready_state[i].exponential_averaging();
      if(estimate > value){
        estimate = value;
        index_lowest_est = i;
      }
    }
    // CPU RUNNING STATE
    if (!ready_state.empty())
    {     
      if(cpu_burst_msg_counter == 0){
        std::cout << prefix( time, ready_state[0].getID() ) << "started using the CPU for " << ready_state[0].bursts[ready_state[0].index].first << "ms burst \n";
        cpu_burst_msg_counter = ready_state[0].bursts[ready_state[0].index].first;
      }else{
        cpu_burst_msg_counter--;
      }
      if (ready_state[0].bursts[ready_state[0].index].first == 0)
      {
        if (ready_state[0].getRemainingBursts() != 1)
        {
          waiting_state.push_back(ready_state[0]); 
        }
        ready_state.erase(ready_state.begin()); 
        if (!ready_state.empty()){
          ready_state.insert(0,);//TODO: ADD in insert
        }
        
      }else{
        ready_state[0].bursts[ready_state[0].index].first--;
      }     
    }

    // Handle Waiting Queue
    for (unsigned int i = 0; i < waiting_state.size(); i++)
    {
      if (waiting_state[i].bursts[waiting_state[i].index].second == 0)
      {
        waiting_state[i].index++;
        ready_state.push_back(waiting_state[i]);
        waiting_state.erase(waiting_state.begin() + i);
      }else{
        waiting_state[i].bursts[waiting_state[i].index].second--;
      }      
    }

    time++;
    if (processes.empty() && waiting_state.empty() && ready_state.empty())
    {
      ALGO_print("Algorithm SJF", avg_cpu, avg_wait, avg_turn, 0, 0, cpu_util);
      return time;
    }
  }

  return time;
}
void ALGO_print(std::string name, double avg_cpu, double avg_wait, double avg_turn, int switches, int preemptions, double cpu_util)
{
  std::cout << name << "\n";
  std::cout << "-- average CPU burst time: " << avg_cpu << " ms\n";
  std::cout << "-- average wait time: " << avg_wait << " ms\n";
  std::cout << "-- average turnaround time: " << avg_turn << " ms\n";
  std::cout << "-- total number of context switches: " << switches << " ms\n";
  std::cout << "-- total number of preemptions: " << preemptions << "\n";
  std::cout << "-- CPU utilization: " << cpu_util << "%\n";
}