#pragma once
#include "kf_save_car.h"
#include <iostream>
using namespace std;

kf_save_car::kf_save_car(
	string s_in //name of file
) {
	filename = s_in;
}

void kf_save_car::open() {
	myfile.open(filename);
}

void kf_save_car::close() {
	myfile.close();
}

void kf_save_car::write(const Eigen::VectorXd& state) {
	string x = to_string(state(0)); //taking x and y values
	string y = to_string(state(1));
	string theta = to_string(state(3));

	myfile << x << "," << y << "," << theta << "\n";

}