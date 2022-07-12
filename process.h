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
    Process(char id, int seed, double lambda, int upper_bound)
    {
        pid = id;
        arrival_time = floor(next_exp(seed, lambda, upper_bound));
        cpu_bursts_num = ceil(next_unif(seed));
        std::cout << "Process " << pid << ": arrival time " << arrival_time << "ms; tau " << "TBD" << "ms; " << cpu_bursts_num << " CPU bursts:" << std::endl;
        for (int c = 0; c < cpu_bursts_num; c++)
        {   
            int cpuBurst = ceil(next_exp(seed, lambda, upper_bound));
            std::cout << "--> CPU burst " << cpuBurst << "ms";
            int ioBurst = 0;
            if (c != cpu_bursts_num - 1)
            {   
                ioBurst = ceil(next_exp(seed, lambda, upper_bound));
                ioBurst *= 10;
                std::cout << " --> I/O burst " << ioBurst << "ms";
            }
            std::cout << std::endl;
            
            std::pair<int, int> burst(cpuBurst, ioBurst);
            bursts.push_back(burst);
            Q.push(burst);
        }
        std::cout << std::endl;
        out = toStr();
    }

    std::string toStr() {
        std::string out = "Process ";
        out += pid;
        out += ": arrival time ";
        out += std::to_string(arrival_time);
        out += "ms; tau ";
        out += "TBD";
        out += "ms; ";
        out += std::to_string(cpu_bursts_num);
        out += " CPU bursts";
        
        return out;
    }

    // getters
    char getID() { return pid; }
    int getArrivalTime() { return arrival_time; }
    int getBurstsNum() { return cpu_bursts_num; }
    int getRemainingBursts() { return cpu_bursts_num - index; }
    const std::list<std::pair<int, int>> getBursts() { return bursts; }
    bool empty() { return Q.empty(); }

    std::pair<int, int> next() { 
        std::pair<int, int> burst = Q.front();
        index++;
        Q.pop();
        return burst;
    }

    // debug
    void printAllBursts() {
        auto burstItr = bursts.begin();
        while( burstItr != bursts.end() ) {
            std::cout << "CPU Burst: " << burstItr->first << "ms  \tIO Burst: " << burstItr++->second << "ms\n";
        }
    }


    friend std::ostream& operator<<(std::ostream& os, const Process& p);
    friend bool operator< ( const Process& a, const Process& b);

private:
    char pid;           // Process name
    int index = 0;      
    int arrival_time;   // Arrival time
    int cpu_bursts_num; // Number of bursts
    // burst.first is CPU burst time
    // burst.second is IO burst time
    // std::list<std::pair<int, int>> bursts;
    std::list<std::pair<int, int>> bursts;
    std::queue<std::pair<int, int>> Q;

    std::string out;
};

std::ostream& operator<<(std::ostream& os, const Process& p)
{
    // os << p.pid << ': ' << p.arrival_time;
    os << p.out;
    return os;
}

bool operator< ( const Process& a, const Process& b) 
{
    return a.arrival_time > b.arrival_time;
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