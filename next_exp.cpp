#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <vector>
#include <stdio.h>

struct CPUBurst {
  int cpu_burst_time;
  int io_burst_time;
};

double next_exp(int seed, double lambda, int upper_bound) {
  double x = 0;
  while(1) {
    double r = drand48();
    x = -log(r) / lambda;
    if(x <= upper_bound) break;
  }

  return x;
}

double next_unif(int seed) {
  double x = drand48() * 100;

  return x;
}

void generate_process(int id, int seed, double lambda, int upper_bound,
    std::vector<int> &arrival_times, std::vector<int> &cpu_bursts_nums,
    std::vector<std::vector<CPUBurst>> &cpu_bursts) {
  int arrival_time = floor(next_exp(seed, lambda, upper_bound));
  int cpu_bursts_num = ceil(next_unif(seed));

  std::vector<CPUBurst> cpu_bursts_process;
  for(int c = 0; c < cpu_bursts_num; c++) {
    struct CPUBurst cpu_burst;
    cpu_burst.cpu_burst_time = ceil(next_exp(seed, lambda, upper_bound));
    if(c == cpu_bursts_num - 1) cpu_burst.io_burst_time = 0;
    else {
      cpu_burst.io_burst_time = ceil(next_exp(seed, lambda, upper_bound));
      cpu_burst.io_burst_time *= 10;
    }
    cpu_bursts_process.push_back(cpu_burst);
  }

  cpu_bursts.push_back(cpu_bursts_process);
  arrival_times.push_back(arrival_time);
  cpu_bursts_nums.push_back(cpu_bursts_num);
}

/* argv[0] - next_exp.c
 * argv[1] - # of processes
 * argv[2] - seed
 * agrv[3] - lambda
 * argv[4] - upper bound
*/
// int main(int argc, char** argv) {
//   if(argc != 5) return EXIT_FAILURE;
//   int num_processes = atoi(*(argv + 1));
//   int seed = atoi(*(argv + 2));
//   double lambda = strtod(*(argv + 3), NULL);
//   int upper_bound = atoi(*(argv + 4));

//   srand48(seed);
  
//   std::vector<int> arrival_times;
//   std::vector<int> cpu_bursts_nums;
//   std::vector<std::vector<CPUBurst>> cpu_bursts;

//   for(int i = 0; i < num_processes; i++) generate_process(i, seed, lambda, upper_bound,
//     arrival_times, cpu_bursts_nums, cpu_bursts);

//   for(int i = 0; i < num_processes; i++) {
//     std::cout << "Process ID: " << i << "\n";
//     std::cout << "Arrival Time: " << arrival_times[i] << "ms\n";
//     std::cout << "Num. of CPU Bursts: " << cpu_bursts_nums[i] << "\n";
//     std::cout << "CPU Bursts: \n\n";
//     for(int c = 0; c < cpu_bursts_nums[i]; c++) {
//       std::cout << "CPU Burst #" << c << "\n";
//       std::cout << "CPU Burst Time: " << cpu_bursts[i][c].cpu_burst_time << "ms\n";
//       std::cout << "I/O Burst Time: " << cpu_bursts[i][c].io_burst_time << "ms\n";
//       std::cout << "\n";
//     }
//   }
// }

