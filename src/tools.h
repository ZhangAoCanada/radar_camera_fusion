#ifndef TOOLS_H_
#define TOOLS_H_

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <iomanip>
#include <vector>

#include <eigen3/Eigen/Dense>
// #include <opencv2/opencv.hpp>


void checkArgs(int argc, char* argv[]) {
  std::string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";
  bool has_valid_args = false;

  if(argc == 1) {
    std::cerr << usage_instructions << std::endl;
  }else if(argc == 2) {
    std::cerr << "Please include an output file.\n" << usage_instructions << std::endl;
  }else if(argc == 3) {
    has_valid_args = true;
  }else if(argc > 3) {
    std::cerr << "Too many arguments.\n" << usage_instructions << std::endl;
  }

  if(!has_valid_args){
    exit(EXIT_FAILURE);
  }
}


void checkFiles(std::ifstream& in_file, std::string& in_name, std::ofstream& out_file, std::string& out_name) {
  if(!in_file.is_open()){
    std::cerr << "Cannot open input file: " << in_name << std::endl;
    exit(EXIT_FAILURE);
  }
  if(!out_file.is_open()){
    std::cerr << "Cannot open output file: " << out_name << std::endl;
    exit(EXIT_FAILURE);
  }
}


double normalize(const double a){
  return (fabs(a) > M_PI) ? remainder(a, 2. * M_PI) : a;
}


Eigen::VectorXd calculate_RMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truths){
  Eigen::VectorXd rmse(4);
  rmse << 0.0, 0.0, 0.0, 0.0;

  for(int k = 0; k < estimations.size(); ++k){
    Eigen::VectorXd diff = estimations[k] - ground_truths[k];
    diff = diff.array() * diff.array();
    rmse += diff;
  }

  rmse /= (double)estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}


#endif // TOOLS_H_
