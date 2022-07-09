#include <iostream>
#include <fstream>
#include <string>

int Shortest_Job_First()
{
  // Calculate all cpu time for all processes using exp avg
  // Have an arrival queue
  // Add all 0 ms arrivals then remove process from process table
  // Sort based on processes cpu burts
  // add proper counters to algorithm
  // Remove process from arrival
  // Repeat additions to arrival based on cpu time with arrival time
  // Exit when process table is empty and return results
}

int main(int argc, char **argv)
{
  if (argc != 8)
  {
    std::cerr << "invalid arguments" << std::endl;
    return EXIT_FAILURE;
  }
  int n = atoi(argv[1]);
  int seed = atoi(argv[2]);
  float lambda = atof(argv[3]);
  float upperBound = atof(argv[4]);
  float contextSwitch = atof(argv[5]);
  float alpha = atof(argv[6]);
  float timeSlice = atof(argv[7]);

  std::cout << "processes|\tseed|\tlambda|\tupper bound|\tcontext switch time|\talpha|\ttime slice" << std::endl;
  std::cout << n << "\t\t\t\t\t\t" << seed << "\t\t" << lambda << "\t\t" << upperBound << "\t\t\t\t\t" << contextSwitch << "\t\t\t\t\t\t\t\t\t\t\t" << alpha << "\t\t\t" << timeSlice << std::endl;
}