#ifndef __process_h_
#define __process_h_

#include <iostream>
#include <math.h>
#include <time.h>
#include <vector>
#include <utility>
#include <list>
#include <queue>

// utility function prototypes (actual functions are at the bottom of the file)
double next_exp(int seed, double lambda, int upper_bound);
double next_unif(int seed);

class Process
{

public:
    Process() {
        pid = 0;
        bursts_completed = 0;
        state = 0;
        arrival_time = 0;
        num_bursts = 0;
    }
    
    Process(char id, int seed, double lambda, int upper_bound) : pid(id), tau(1/lambda)
    {
        // tau = 1/lambda;
        bursts_completed = 0;
        state = 0;
        arrival_time = floor(next_exp(seed, lambda, upper_bound));
        num_bursts = ceil(next_unif(seed));
        std::cout << "Process " << pid << ": arrival time " << arrival_time << "ms; tau " << tau << "ms; " << num_bursts << ( num_bursts == 1 ? "CPU burst: " : " CPU bursts:" ) << std::endl;
        for (int c = 0; c < num_bursts; c++)
        {   
            int cpuBurst = ceil(next_exp(seed, lambda, upper_bound));
            std::cout << "--> CPU burst " << cpuBurst << "ms";
            int ioBurst = 0;
            if (c != num_bursts - 1)
            {   
                ioBurst = ceil(next_exp(seed, lambda, upper_bound));
                ioBurst *= 10;
                std::cout << " --> I/O burst " << ioBurst << "ms";
            }
            std::cout << std::endl;
            
            std::pair<int, int> burst(cpuBurst, ioBurst);
            bursts.push_back(burst);
            originalBursts.push_back(burst);
        }
    }

    std::string toStr() {
        std::string out = "Process ";
        out += pid;
        out += ": arrival time ";
        out += std::to_string(arrival_time);
        out += "ms; tau";
        out += tau;
        out += "ms;";
        out += std::to_string(num_bursts);
        out += " CPU bursts\n";
        
        return out;
    }

    // getters
    char getID() { return pid; }
    int getArrivalTime() { return arrival_time; }
    int getNumBursts() { return num_bursts; }
    int getRemainingBursts() { return num_bursts - bursts_completed; }
    int getBurstsCompleted() { return bursts_completed; }
    int getState() { return state; }
    const std::vector<std::pair<int, int>> getOriginalBursts() { return originalBursts; }
    const std::vector<std::pair<int, int>> getBursts() { return bursts; }
    int getCurrentCPUBurst() { return bursts[bursts_completed].first; }
    int getCurrentIOBurst() { return bursts[bursts_completed - 1].second; }
    bool empty() { return bursts.empty(); }

    std::vector<int> getAllCpuBurstTimes() {
        std::vector<int> res;
        for ( std::pair<int, int> burst : bursts ) {
            res.push_back( burst.first );
        }
        return res;
    }

    // debug
    void printAllBursts() {
        auto burstItr = bursts.begin();
        while( burstItr != bursts.end() ) {
            std::cout << "CPU Burst: " << burstItr->first << "ms  \tIO Burst: " << burstItr++->second << "ms\n";
        }
    }

    // add process to ready queue
    void addToReadyQueue(std::queue<Process*> &queue) {
        queue.push(this);
        state = 1;
    }

    // return end time
    int doCPUBurst(int startTime, int burstTime) {
        // if process is not already using CPU, it's now using it
        if(state != 2) state = 2;
        std::pair<int, int> current_burst = bursts[bursts_completed];
        int cpu_burst = current_burst.first;
        // process will finish cpu burst
        if(burstTime == cpu_burst || cpu_burst == 0) {
            // cpu burst is completed
            cpu_burst = 0;
            current_burst.first = cpu_burst;
            bursts[bursts_completed] = current_burst;
            bursts_completed++;
            // if all bursts completed, then process can terminate
            if(bursts_completed == num_bursts) state = 4;
            // else, process will move to I/O
            else state = 3;
        }
        // process will not be able to finish cpu burst
        else {
            // update cpu burst with time remaining
            cpu_burst -= burstTime;
            current_burst.first = cpu_burst;
            bursts[bursts_completed] = current_burst;
        }

        return startTime + burstTime;
    }

    // return end time
    int doIOBurst(int startTime) {
        std::pair<int, int> current_burst = bursts[bursts_completed - 1];
        int io_burst = current_burst.second;
        int new_io_burst = 0;
        current_burst.second = new_io_burst;
        bursts[bursts_completed - 1] = current_burst;
        // return to ready queue
        state = 1;

        return startTime + io_burst;
    }

    friend std::ostream& operator<<(std::ostream& os, const Process& p);

private:
    char pid;           // Process name
    int bursts_completed; // number of bursts completed
    int arrival_time;   // Arrival time
    int num_bursts; // Number of bursts

    // 0 - ready to arrive
    // 1 - in ready queue
    // 2 - using CPU
    // 3 - using I/O
    // 4 - terminated
    int state; // process state

    // burst.first is CPU burst time
    // burst.second is IO burst time
    // std::list<std::pair<int, int>> bursts;
    std::vector<std::pair<int, int>> originalBursts;
    std::vector<std::pair<int, int>> bursts;
    int tau;
};

std::ostream& operator<<(std::ostream& os, const Process& p)
{
    os << p.pid << ':' << p.arrival_time;
    return os;
}


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

#endif