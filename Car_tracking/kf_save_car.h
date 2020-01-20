#pragma once
#include <string>
#include <Eigen/Dense>
#include <fstream>

using namespace std;

class kf_save_car
{
public: 
	kf_save_car(string s);
	  void open();
	  void close();
	  void write(const Eigen::VectorXd& state);

private:
	string filename;
	ostream myfile;
};