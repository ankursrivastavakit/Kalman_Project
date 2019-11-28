#pragma once
#include <kf_save.h>
#include <iostream>
using namespace std;

kf_save::kf_save(
	string s_in //name of file
) {
	filename = s_in;
}

void kf_save::open() {
	myfile.open(filename);
}

void kf_save::close() {
	myfile.close();
}

void kf_save::write(const Eigen::VectorXd& state){
	string x = to_string(state(0)); //taking x and y values
	string y = to_string(state(1));

	myfile << x << "," << y << "\n";

}