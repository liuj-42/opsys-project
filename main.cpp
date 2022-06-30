#include <iostream>
#include <fstream>
#include <string>

int main( int argc, char** argv ) {
  if ( argc != 8 ) {
    std::cerr << "invalid arguments" << std::endl;
    return EXIT_FAILURE;
  }
  int n = atoi( argv[1] );
  int seed = atoi( argv[2] );
  float lambda = atof( argv[3] );
  float upperBound = atof( argv[4] );
  float contextSwitch = atof( argv[5] );
  float alpha = atof( argv[6] );
  float timeSlice = atof( argv[7] );

  std::cout << "processes|\tseed|\tlambda|\tupper bound|\tcontext switch time|\talpha|\ttime slice" << std::endl;
  std::cout << n << "\t\t\t\t\t\t" << seed << "\t\t" << lambda << "\t\t" << upperBound << "\t\t\t\t\t" << contextSwitch << "\t\t\t\t\t\t\t\t\t\t\t" << alpha << "\t\t\t" << timeSlice << std::endl;
}