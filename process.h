#ifndef __process_h_
#define __process_h_

#include <iostream>
#include <math.h>
#include <time.h>
#include <vector>
#include <utility>
#include <list>


class Process
{

public:
    Process(char id, int seed, double lambda, int upper_bound)
    {
        pid = id;
        arrival_time = floor(next_exp(seed, lambda, upper_bound));
        cpu_bursts_num = ceil(next_unif(seed));
        for (int c = 0; c < cpu_bursts_num; c++)
        {   
            int cpuBurst = ceil(next_exp(seed, lambda, upper_bound));
            int ioBurst = 0;
            if (c != cpu_bursts_num - 1)
            {   
                ioBurst = ceil(next_exp(seed, lambda, upper_bound));
                ioBurst *= 10;
            }
            bursts.push_back(std::pair(cpuBurst, ioBurst));
        }
        burstItr = bursts.begin();

        // cpu_bursts_process vector<CPUBurst>
        // arrival_time
        // cpu_bursts_num
    }

    std::pair<int, int> nextBurst() {
        return *(burstItr++);
    }

    // getters
    char getID() { return pid; }
    int getArrivalTime() { return arrival_time; }
    int getBurstsNum() { return cpu_bursts_num; }

private:
    char pid;
    int index = 0;
    int arrival_time;
    int cpu_bursts_num;
    // burst.first is CPU burst time
    // burst.second is IO burst time
    // std::vector<std::pair<int, int>> bursts;
    std::list<std::pair<int, int>> bursts;
    std::list<std::pair<int, int>>::iterator burstItr;
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

#endif