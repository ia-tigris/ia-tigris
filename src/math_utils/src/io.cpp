/*
* Copyright (c) 2016 Carnegie Mellon University, Author <sanjiban@cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/


/* Copyright 2015 Sanjiban Choudhury
 * io.cpp
 *
 *  Created on: Feb 21, 2016
 *      Author: Sanjiban Choudhury
 */


#include "math_utils/math_utils.h"
#include <iostream>
#include <fstream>
#include <iterator>

namespace ca {
namespace math_utils{
namespace io {

bool LoadTableFromNormalFile(const std::string &filename, Eigen::MatrixXd &table) {
  std::ifstream infile(filename);
  if(!infile)
    return false;
  std::string line;

  std::vector< std::vector<double> > data;
  while ( std::getline(infile, line) ) {
     std::istringstream is( line );
     data.push_back(
           std::vector<double>( std::istream_iterator<double>(is),
                             std::istream_iterator<double>() ) );
  }

  table = Eigen::MatrixXd(data.size(), data.front().size());

  for (int i = 0; i < table.rows(); i++) {
    for (int j = 0; j < table.cols(); j++) {
      table(i, j) = data[i][j];
    }
  }

  return true;
}

bool LoadTableFromCSVFile(const std::string &filename, Eigen::MatrixXd &table) {
  std::ifstream infile(filename);
  if(!infile)
    return false;
  std::string line;

  std::vector< std::vector<double> > data;
  while ( std::getline(infile, line) ) {
     std::istringstream is( line );

     std::string token;
     std::vector <double> data_line;
     while(std::getline(is, token, ',')) {
       data_line.push_back(std::stod(token));
     }

     data.push_back(data_line);
  }

  table = Eigen::MatrixXd(data.size(), data.front().size());

  for (int i = 0; i < table.rows(); i++) {
    for (int j = 0; j < table.cols(); j++) {
      table(i, j) = data[i][j];
    }
  }

  return true;
}

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

bool SaveTableToCSVFile(const std::string &filename, const Eigen::MatrixXd &table) {
  std::ofstream file(filename, std::ifstream::out);
  if(!file.good())
    return false;
  file << table.format(CSVFormat);
  file.close();
  return true;
}


bool SaveVecTableToCSVFile(const std::string &filename, const std::vector< std::vector<double> > &table) {
  std::ofstream file(filename, std::ifstream::out);
  if(!file.good())
    return false;
  for (auto it : table) {
    if (it.size() >= 2)
       std::copy(it.begin(), it.end()-1, std::ostream_iterator<double>(file, ","));
    if (it.size() >= 1)
       file << it.back();
    file << std::endl;
  }
  file.close();
  return true;
}


}
}
}

