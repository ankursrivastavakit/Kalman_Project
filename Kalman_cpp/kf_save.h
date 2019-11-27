#pragma once
#include <string>
#include <Eigen/Dense>
#include <fstream>
using namespace std;

class kf_save
{
public:
	kf_save(string s);
	void open();
	void close();
	void write(const Eigen::VectorXd& state);

private:
	//file name
	string filename;
	ofstream myfile;
};

