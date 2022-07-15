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
#include <climits>
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

/*
#if 1 // debugging
  std::string names[7] = {"processes\t\t\t", "seed\t\t\t\t", "lambda\t\t\t\t", "upper bound\t\t\t", "context switch time\t", "alpha\t\t\t\t", "time slice\t\t\t"};

  for (int i = 0; i < 7; i++)
  {
    std::cout << *(names + i) << "| " << *(argv + i + 1) << std::endl;
  }

  std::cout << "\n\n------------------------------\n";*/


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
  std::cout<<"\n";
  time += sjf(processes, contextSwitch);
  return 0;
}

std::string prefix( int time, char pid ) {
  std::string out = "time ";
  out += std::to_string(time);
  out += "ms: Process ";
  out += pid;
  out += " ";
  return out;
}

std::string prefix(int time, char pid,int tau)
{
  std::string out = "time ";
  out += std::to_string(time);
  out += "ms: Process ";
  out += pid;
  out += " ";
  out +="(tau ";
  out += std::to_string(tau);
  out +="ms) ";
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

void print_queue_inside(std::vector<Process> q)
{ // NB: pass by value so the print uses a copy
  std::cout<< "[Q: ";
  if(q.size()<=1){
    std::cout<<"empty";
  }else{
    for(unsigned int i=1; i< q.size(); i++){
      std::cout << q[i].getID();
      if(i<q.size()-1){std::cout<<' ';}
    }
  }
  
  std::cout << "]\n";
}
void print_queue_outside(std::vector<Process> q)
{ // NB: pass by value so the print uses a copy
  std::cout<< "[Q: ";
  if(q.empty()){
    std::cout<<"empty";
  }
  for(unsigned int i=0; i< q.size(); i++)
  {
    std::cout << q[i].getID();
    if(i<q.size()-1){std::cout<<' ';}
  }
  std::cout << "]\n";
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
  std::vector<Process> leaving_Process;
  int value=0;
  int incoming_contextSwitch_counter = contextSwitch/2;
  int exiting_contextSwitch_counter = contextSwitch/2;
  while (1){   
    /*
    if(time >245 && time < 249){
      
      std::cout << "TIME: " << time << " ms\n";
      std::cout << "   Processes: " << processes.size() << "\n";
      std::cout << "   Waiting_Q: " << waiting_state.size() << "\n";
      for (unsigned int i = 0; i < waiting_state.size(); i++)
      {
        std::cout << "\t   ID:" << waiting_state[i].getID() << " is on " << waiting_state[i].index << "with " << waiting_state[i].bursts[waiting_state[i].index].second << "\n";
      }
      std::cout << "   ready_state: " << ready_state.size() << "\n";
      std::cout << "   incoming_contextSwitch_counter: " << incoming_contextSwitch_counter << "\n";
      std::cout << "   exiting_contextSwitch_counter: " << exiting_contextSwitch_counter << "\n";
      if(leaving_Process.size()>0){
        std::cout << "   leaving_Process: " << leaving_Process[0].getID() << "\n";
      }      
    }*/
    
    // ADD new Arrivals to Ready Queue
    if (!processes.empty()){
      for (unsigned int i = 0; i < processes.size(); i++){
        if (time == processes[i].getArrivalTime()){ // Initial Setup
          ready_state.push_back(processes[i]);//dont forget to sort ready state
          deletes.push_back(processes[i].getID());
          std::cout << prefix( time, processes[i].getID(),processes[i].old_tau ) << "arrived; added to ready queue ";
          if(cpu_burst_msg_counter == 0){
            print_queue_outside(ready_state);
          }else{
            print_queue_inside(ready_state);
          }
          
        }
      }
      // Delete From Processes
      for (unsigned int i = 0; i < deletes.size(); i++){
        if (processes[i].getID() == deletes[0]){
          processes.erase(processes.begin() + i);
          deletes.erase(deletes.begin());
          i--;
        }
      }
    }
    // Handle Waiting Queue
    for (unsigned int i = 0; i < waiting_state.size(); i++){
      waiting_state[i].bursts[waiting_state[i].index].second--; 
      if (waiting_state[i].bursts[waiting_state[i].index].second == 0){
        waiting_state[i].index++;
        ready_state.push_back(waiting_state[i]);
        std::cout << prefix( time, waiting_state[i].getID(),waiting_state[i].waiting_exponential_averaging()) << "completed I/O; added to ready queue ";
        print_queue_outside(ready_state);
        waiting_state.erase(waiting_state.begin() + i);
      }
    }

    if(!leaving_Process.empty()){//Exiting Process switch
      exiting_contextSwitch_counter--;
      if(exiting_contextSwitch_counter==0){
        if(ready_state[0].getRemainingBursts() !=1){
          waiting_state.push_back(leaving_Process[0]);
        }        
        leaving_Process.erase(leaving_Process.begin());
        exiting_contextSwitch_counter = contextSwitch/2;
      }
    }
    else if (!ready_state.empty()){     // CPU RUNNING STATE
      if(incoming_contextSwitch_counter ==0){//Incoming Process Switch
        if(cpu_burst_msg_counter == 0){
          ready_state[0].exponential_averaging();
          std::cout << prefix( time, ready_state[0].getID(),(int)ceil(ready_state[0].last_est_burst)) << "started using the CPU for " << ready_state[0].bursts[ready_state[0].index].first << "ms burst ";
          print_queue_inside(ready_state);
          cpu_burst_msg_counter = ready_state[0].bursts[ready_state[0].index].first;
        }else{
          cpu_burst_msg_counter--;
        }
        if (ready_state[0].bursts[ready_state[0].index].first == 0){
          if(ready_state[0].getRemainingBursts() >1){
            std::cout << prefix( time, ready_state[0].getID(),(int)ceil(ready_state[0].last_est_burst)) << "completed a CPU burst; " << ready_state[0].getRemainingBursts()-1 << (ready_state[0].getRemainingBursts()-1 == 1 ? " burst to go " : " bursts to go ");
            print_queue_inside(ready_state);
          }else{
            std::cout << prefix( time, ready_state[0].getID()) << "terminated ";
            print_queue_inside(ready_state);
          }
          leaving_Process.push_back(ready_state[0]); 
          if (ready_state[0].getRemainingBursts() != 1){            
            std::cout <<"time "<<std::to_string(time)<<"ms: Recalculated tau for process "<<ready_state[0].getID()<<": old tau "<<ready_state[0].last_est_burst<<"ms; new tau "<<ready_state[0].future_exponential_averaging()<<"ms ";
            print_queue_inside(ready_state);
            std::cout << prefix( time, ready_state[0].getID()) << "switching out of CPU; will block on I/O until time " << time + ready_state[0].bursts[ready_state[0].index].second + contextSwitch/2<< "ms ";
            print_queue_inside(ready_state);
          }
          if(ready_state.size()>1){
            int index_lowest_est=0;
            int lowest_estimate = INT_MAX;
            for(unsigned int i = 0; i < ready_state.size(); i++){
              if(ready_state[i].last_est_burst < lowest_estimate){lowest_estimate = ready_state[i].last_est_burst; index_lowest_est = i;}
            }
            std::iter_swap(ready_state.begin()+index_lowest_est,ready_state.begin());
            ready_state.erase(ready_state.begin()+index_lowest_est); 
          }else{
            ready_state.erase(ready_state.begin()); 
          }  
          incoming_contextSwitch_counter = contextSwitch/2;        
        }else{
          ready_state[0].bursts[ready_state[0].index].first--;
        }     
      }else{
        incoming_contextSwitch_counter--;
      }
    }

    

    if (processes.empty() && waiting_state.empty() && ready_state.empty() && leaving_Process.empty()){
      std::cout<<"time "<<std::to_string(time)<<"ms: Simulator ended for SJF [Q: empty]\n";
      //ALGO_print("Algorithm SJF", avg_cpu, avg_wait, avg_turn, 0, 0, cpu_util);
      return time;
    }
    time++;
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