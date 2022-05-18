#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <sstream>

#include "SimpleMath/SimpleMath.h"

using namespace std;

typedef VectorNd (*rhsFuncPtr) (const double, const VectorNd&);
typedef VectorNd (*integratorFuncPtr) (const double, const VectorNd&, const double, rhsFuncPtr);

/** \brief Delete all contents of an existing file, i.e. truncate it so
 * that it is empty. */
void reset_file (const char *filename) {
	ofstream output_file (filename, ios::trunc);

	if (!output_file) {
		cerr << "Error: could not reset file " << filename << "." << endl;
		abort();
	}

	output_file.close();
}

/** \brief Open and append data to the end of the file */
void append_data (const char *filename, double t, const VectorNd &values) {
	ofstream output_file (filename, ios::app);

	if (!output_file) {
		cerr << "Error: could not open file " << filename << "." << endl;
		abort();
	}

	output_file << scientific;
	output_file << t << ", ";
	for (unsigned int i = 0; i < values.size(); i++) {
		output_file << values[i];

		if (i != values.size() - 1)
			output_file << ", ";
	}
	output_file << endl;

	output_file.close();
}

VectorNd rhs_func (double t, const VectorNd &y) {
	assert (y.size() == 1);

	unsigned int dim = y.size();
	VectorNd res (VectorNd::Zero(dim));

    //insert rhs
//	res[0] = ___;

	return res;
}

VectorNd rk4_integrator (const double t, const VectorNd &y, const double h, rhsFuncPtr rhs) {
    //implement rk4_integrator
//	return ___;
}

VectorNd euler_integrator (const double t, const VectorNd &y, const double h, rhsFuncPtr rhs) {
    //implement euler_integrator
//	return ___;
}
double exact (double t) {
	return 1. / (1. + 100 * t * t);
}

VectorNd simulate (const double t0, const VectorNd &y0, const double tf, const double h, rhsFuncPtr rhs, integratorFuncPtr integrator, const string foutput) {
	double t = t0;
	VectorNd y = y0;
	VectorNd y_out = VectorNd::Zero(2);
	reset_file (foutput.c_str());

	while (t < tf) {

        // implement single step method
		y_out[0] = y;
		y_out[1] = exact(t);
		
        // save data
        append_data (foutput.c_str(), t, y_out);

	}

	return y;
}


int main(int argc, char* argv[]) {
	VectorNd y0 (VectorNd::Zero(1));
	VectorNd yf;

	double t0 = -3.;
	double tf = 1.0;
	y0[0] = 1. / 901.;

	for (int i = 2; i < 7; i++) {
	  stringstream outputfileStr;
	  outputfileStr << "output-2-" << i << ".csv";
		double h = pow (10, -i);
		yf = simulate (t0, y0, tf, h, rhs_func, rk4_integrator, outputfileStr.str());
		cout << "h = " << h << "\teta(tf) = " << yf[0] << "\ty(tf) = " << exact(tf) << "\terror = " << yf[0] - exact(tf) << endl;
	}

	return 0;
}
