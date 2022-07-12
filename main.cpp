#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <vector>
<<<<<<< Updated upstream
#include <stdio.h>
#include <queue>
=======
#include <algorithm>

// #include "next_exp.cpp"
#include "process.h"
char alphabet[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'};

int fcfs( std::vector<Process> processes, int contextSwitch );
int sjf( std::vector<Process> processes, int contextSwitch);
void ALGO_print(std::string name, double avg_cpu, double avg_wait, double avg_turn, int switches, int preemptions, double cpu_util);
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
    processes.push_back( Process(alphabet[i], seed, lambda, upperBound,alpha) );
  }


  int time = 0;
  // FCFS
  //time += fcfs( processes, contextSwitch );
  time += sjf(processes,contextSwitch);
>>>>>>> Stashed changes

double exponential_avgeraging(float alpha, float actual_burst, float estimated_burst)
{
  double estimate = alpha * actual_burst + ((1 - alpha) * estimated_burst);
  return estimate;
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

struct CPUBurst
{
  int cpu_burst_time;
  int io_burst_time;
};

struct Process
{
  int id;
  int arrival_time;
  int cpu_bursts_num;
  int current_burst = 0;
  std::vector<CPUBurst> cpu_bursts;
};

double next_exp(int seed, double lambda, int upper_bound)
{
  double x = 0;
  while (1)
  {
    double r = drand48();
    x = -log(r) / lambda;
    if (x <= upper_bound)
      break;
  }
  return x;
}

double next_unif(int seed)
{
  double x = drand48() * 100;
  return x;
}

void generate_process(int id, int seed, double lambda, int upper_bound,
                      std::vector<Process> &processes)
{
  struct Process new_process;
  new_process.id = id;
  new_process.arrival_time = floor(next_exp(seed, lambda, upper_bound));
  new_process.cpu_bursts_num = ceil(next_unif(seed));

  std::vector<CPUBurst> cpu_bursts_process;
  for (int c = 0; c < new_process.cpu_bursts_num; c++)
  {
    struct CPUBurst cpu_burst;
    cpu_burst.cpu_burst_time = ceil(next_exp(seed, lambda, upper_bound));
    if (c == new_process.cpu_bursts_num - 1)
      cpu_burst.io_burst_time = 0;
    else
    {
      cpu_burst.io_burst_time = ceil(next_exp(seed, lambda, upper_bound));
      cpu_burst.io_burst_time *= 10;
    }
    cpu_bursts_process.push_back(cpu_burst);
  }
  new_process.cpu_bursts = cpu_bursts_process;
  processes.push_back(new_process);
}
struct compareProcess {
    bool operator()(Process const& p1, Process const& p2)
    {
        // return "true" if "p1" is ordered
        // before "p2", for example:
        return p1.id < p2.id;
    }
};
int Shortest_Job_First(float alpha, std::vector<Process> &processes)
{
  // Calculate all cpu time for all processes using exp avg
 
  // Sort based on processes cpu burts
  // add proper counters to algorithm
  // Remove process from arrival
  // Repeat additions to arrival based on cpu time with arrival time
  // Exit when process table is empty and return results
  int time =0;
  double avg_cpu = 0;
  double avg_wait = 0;
  double avg_turn = 0;
  double cpu_util = 0;
  std::priority_queue<Process, std::vector<Process>, compareProcess> ready_state;
  std::vector<Process> waiting_state;
  std::vector<int> deletes;
  //TODO:INCREMENT CORRECT COUNTERS AND ADD IN COMMENT PARTS
  while(1){ 
       std::cout << "TIME: " << time << " ms\n";
       std::cout << "   Processes: " << processes.size() << "\n";
       std::cout << "   Waiting_Q: " << waiting_state.size() << "\n";
       for(unsigned int i =0; i < waiting_state.size(); i++){
        std::cout<<"\t   ID:"<<waiting_state[i].id<<" is on "<<waiting_state[i].current_burst << "with "<<waiting_state[i].cpu_bursts[waiting_state[i].current_burst].io_burst_time<<"\n";
       }
       std::cout << "   ready_state: " << ready_state.size() << "\n";
      //ADD new Arrivals to Ready Queue
      if(!processes.empty()){
        for(unsigned int i = 0; i < processes.size(); i++){
          if(time==processes[i].arrival_time){//Initial Setup
            ready_state.push(processes[i]);
            deletes.push_back(processes[i].id);
            std::cout << "Process ID: " << processes[i].id << "\n";
          }
        }
        //Delete From Processes
        for(unsigned int i=0; i < processes.size(); i++){
          if(processes[i].id ==deletes[0]){
            processes.erase(processes.begin()+i);
            deletes.erase(deletes.begin());
            i--;
          }
        }
      }

      //CPU RUNNING STATE
      if(!ready_state.empty()){
        Process running = ready_state.top();        
        if(running.cpu_bursts[running.current_burst].cpu_burst_time ==0){
          if(running.current_burst != running.cpu_bursts_num){
            waiting_state.push_back(running);//Need to check
          }        
          ready_state.pop();//need to check
        }
        running.cpu_bursts[running.current_burst].cpu_burst_time--;
      }

      //Handle Waiting Queue
      for(unsigned int i=0; i< waiting_state.size(); i++){        
        if(waiting_state[i].cpu_bursts[waiting_state[i].current_burst].io_burst_time ==0){
          waiting_state[i].current_burst++;
          ready_state.push(waiting_state[i]);         
          waiting_state.erase(waiting_state.begin()+i);
        }
        waiting_state[i].cpu_bursts[waiting_state[i].current_burst].io_burst_time--;
      }

      time++;

    if(processes.empty() && waiting_state.empty() && ready_state.empty()){
      ALGO_print("Algorithm SJF", avg_cpu, avg_wait, avg_turn, 0, 0, cpu_util);
      return 0;
    }
  }  
  return 0;
}
/* argv[0] - next_exp.c
 * argv[1] - # of processes
 * argv[2] - seed
 * agrv[3] - lambda
 * argv[4] - upper bound
 */
int main(int argc, char **argv)
{
  if (argc != 6)
  {
    return EXIT_FAILURE;
  }
  int num_processes = atoi(*(argv + 1));
  int seed = atoi(*(argv + 2));
  double lambda = strtod(*(argv + 3), NULL);
  int upper_bound = atoi(*(argv + 4));
  float alpha = atof(*(argv + 4));
  srand48(seed);

  std::vector<Process> processes;

<<<<<<< Updated upstream
  for (int i = 0; i < num_processes; i++)
    generate_process(i, seed, lambda, upper_bound, processes);
  
  for (int i = 0; i < num_processes; i++)
  {
    std::cout << "Process ID: " << i << "\n";
    std::cout << "Arrival Time: " << processes[i].arrival_time << "ms\n";
    
    std::cout << "Num. of CPU Bursts: " << processes[i].cpu_bursts_num << "\n";
    std::cout << "CPU Bursts: \n\n";
    for (int c = 0; c < processes[i].cpu_bursts_num; c++)
    {
      std::cout << "CPU Burst #" << c << "\n";
      std::cout << "CPU Burst Time: " << processes[i].cpu_bursts[c].cpu_burst_time << "ms\n";
      std::cout << "I/O Burst Time: " << processes[i].cpu_bursts[c].io_burst_time << "ms\n";
      std::cout << "\n";
    }
  }
  std::cout << "Start of simulation\n";
  Shortest_Job_First(alpha, processes);
  return 0;
=======
int sjf( std::vector<Process> processes, int contextSwitch) {
  int time = 0;
  // std::priority_queue<Process> Q;
  std::cout << "time 0ms: Simulator started for SJF [Q: empty]\n";
  std::sort( processes.begin(), processes.end(), [](Process a, Process b) {
        return a.exponential_averaging() < b.exponential_averaging();
  });
  auto cmp = [](Process a, Process b) { return a.exponential_averaging() < b.exponential_averaging(); };
  std::priority_queue<Process, std::vector<Process>, decltype(cmp)> ready_state(cmp);
  

  //print_queue(ready_state);
  double avg_cpu = 0;
  double avg_wait = 0;
  double avg_turn = 0;
  double cpu_util = 0;
  std::vector<Process> waiting_state;
  std::vector<char> deletes;  

  while(1){
    if(!waiting_state.empty() || !ready_state.empty()){
      std::cout << "TIME: " << time << " ms\n";
      std::cout << "   Processes: " << processes.size() << "\n";
      std::cout << "   Waiting_Q: " << waiting_state.size() << "\n";
      for(unsigned int i =0; i < waiting_state.size(); i++){
        std::cout<<"\t   ID:"<<waiting_state[i].getID()<<" is on "<<waiting_state[i].index << "with "<<waiting_state[i].next().second<<"\n";
      }
      std::cout << "   ready_state: " << ready_state.size() << "\n";
    }
    
    //ADD new Arrivals to Ready Queue
    if(!processes.empty()){
      for(unsigned int i = 0; i < processes.size(); i++){
        if(time==processes[i].getArrivalTime()){//Initial Setup
          ready_state.push(processes[i]);
          deletes.push_back(processes[i].getID());
          std::cout << "Process ID: " << processes[i].getID() << "\n";
        }
      }
      //Delete From Processes
      for(unsigned int i=0; i < deletes.size(); i++){
        if(processes[i].getID() ==deletes[0]){
          processes.erase(processes.begin()+i);
          deletes.erase(deletes.begin());
          i--;
        }
      }
    }
    //CPU RUNNING STATE
    if(!ready_state.empty()){
      Process running = ready_state.top();    
      std::list<std::pair<int, int>>::iterator it = running.bursts.begin();
      for(int i =0; i<running.index;i++){it++;}
      std::pair<int, int> burst = *it;    
      std::cout << "Burst Left:"<<burst.first<< "\n";
      if(burst.first ==0){
        if(running.getRemainingBursts()==1){
          waiting_state.push_back(running);//Need to check
        }        

        ready_state.pop();//need to check
      }      
      it->first--;
    }

    //Handle Waiting Queue
    for(unsigned int i=0; i< waiting_state.size(); i++){   
      std::list<std::pair<int, int>>::iterator it = waiting_state[i].bursts.begin();
      for(int i =0; i<waiting_state[i].index;i++){it++;}
      std::pair<int, int> burst = *it;     
      if(burst.second ==0){
        waiting_state[i].index++;
        ready_state.push(waiting_state[i]);         
        waiting_state.erase(waiting_state.begin()+i);
      }     
      it->second--;
    }

    time++;
    if(processes.empty() && waiting_state.empty() && ready_state.empty()){
      ALGO_print("Algorithm SJF", avg_cpu, avg_wait, avg_turn, 0, 0, cpu_util);
      return time;
    }
  }
  
  return time;
}
void ALGO_print(std::string name, double avg_cpu, double avg_wait, double avg_turn, int switches, int preemptions, double cpu_util){
  std::cout << name << "\n";
  std::cout << "-- average CPU burst time: " << avg_cpu << " ms\n";
  std::cout << "-- average wait time: " << avg_wait << " ms\n";
  std::cout << "-- average turnaround time: " << avg_turn << " ms\n";
  std::cout << "-- total number of context switches: " << switches << " ms\n";
  std::cout << "-- total number of preemptions: " << preemptions << "\n";
  std::cout << "-- CPU utilization: " << cpu_util << "%\n";
>>>>>>> Stashed changes
}