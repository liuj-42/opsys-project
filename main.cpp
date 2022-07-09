#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <vector>
#include <stdio.h>

double exponential_avgeraging(float alpha, float t, float tau)
{
  double estimate = alpha * t + ((1 - alpha) * tau);
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
  int arrival_time;
  int cpu_bursts_num;
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
}
int Shortest_Job_First(int time, float alpha, std::vector<Process> &processes)
{
  // Calculate all cpu time for all processes using exp avg
  // Have an arrival queue
  // Add all 0 ms arrivals then remove process from process table
  // Sort based on processes cpu burts
  // add proper counters to algorithm
  // Remove process from arrival
  // Repeat additions to arrival based on cpu time with arrival time
  // Exit when process table is empty and return results

  double avg_cpu = 0;
  double avg_wait = 0;
  double avg_turn = 0;
  double cpu_util = 0;
  ALGO_print("Algorithm SJF", avg_cpu, avg_wait, avg_turn, 0, 0, cpu_util);
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

  for (int i = 0; i < num_processes; i++)
    generate_process(i, seed, lambda, upper_bound, processes);

  for (int i = 0; i < num_processes; i++)
  {
    std::cout << "Process ID: " << i << "\n";
    std::cout << "Arrival Time: " << processes[i].arrival_time << "ms\n";
    /*
    std::cout << "Num. of CPU Bursts: " << cpu_bursts_nums[i] << "\n";
    std::cout << "CPU Bursts: \n\n";
    for (int c = 0; c < cpu_bursts_nums[i]; c++)
    {
      std::cout << "CPU Burst #" << c << "\n";
      std::cout << "CPU Burst Time: " << cpu_bursts[i][c].cpu_burst_time << "ms\n";
      std::cout << "I/O Burst Time: " << cpu_bursts[i][c].io_burst_time << "ms\n";
      std::cout << "\n";
    }*/
  }
  std::cout << "Start of simulation\n";
  Shortest_Job_First(0, alpha, processes);
  return 0;
}