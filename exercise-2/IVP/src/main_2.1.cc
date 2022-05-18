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

//VectorNd rhs_func (double t, const VectorNd &y) {
//    assert (y.size() == 1);
//
//    unsigned int dim = y.size();
//    VectorNd res (VectorNd::Zero(dim));
//
//    //insert rhs
//    res[0] = y[0];
//
//    return res;
//}

VectorNd rhs_func (double t, const VectorNd &y) {
	assert (y.size() == 1);

	unsigned int dim = y.size();
	VectorNd res (VectorNd::Zero(dim));

    //insert rhs
	res[0] = -200 * t * y[0] * y[0];

	return res;
}

VectorNd rk4_integrator (const double t, const VectorNd &y, const double h, rhsFuncPtr rhs) {
    //implement rk4_integrator
    //	return ___;
}

VectorNd euler_integrator (const double t, const VectorNd &y, const double h, rhsFuncPtr rhs) {
    //implement euler_integrator
    auto vector = VectorNd(1);
    vector[0] = y[0];
    return rhs(t, vector);
}

//double exact (double t) {
//    return exp(t);
//}

double exact (double t) {
	return 1. / (1. + 100 * t * t);
}

VectorNd simulate (const double t0, const VectorNd &y0, const double tf, const double h, rhsFuncPtr rhs, integratorFuncPtr integrator, const string foutput) {
	double t = t0;
	VectorNd y = y0;
	VectorNd y_out = VectorNd::Zero(2);
	reset_file (foutput.c_str());

    y_out[0] = y[0];
	while (t < tf) {

        // implement single step method
        if (t != t0) {
            y_out[0] = y_out[0] + h * integrator(t, y_out, h, rhs);
        }
		y_out[1] = exact(t);

//        std::cout << y_out << std::endl;

        // save data
        append_data (foutput.c_str(), t, y_out);

        t += h;
	}

	return y;
}


//int main(int argc, char* argv[]) {
//    VectorNd y0 (VectorNd::Zero(1));
//    VectorNd yf;
//
//    double t0 = 0.;
//    double tf = 1.0;
//    y0[0] = 1.;
//
//    for (int i = 2; i < 7; i++) {
//        stringstream outputfileStr;
//        outputfileStr << "output-1-" << i << ".csv";
//        double h = pow (10, -i);
//        yf = simulate (t0, y0, tf, h, rhs_func, euler_integrator, outputfileStr.str());
//        cout << "h = " << h << "\teta(tf) = " << yf[0] << "\ty(tf) = " << exact(tf) << "\terror = " << yf[0] - exact(tf) << endl;
//    }
//
//    return 0;
//}


int main(int argc, char* argv[]) {
	VectorNd y0 (VectorNd::Zero(1));
	VectorNd yf;

	double t0 = -3.;
	double tf = 1.0;
	y0[0] = 1. / 901.;

	for (int i = 2; i < 7; i++) {
	  stringstream outputfileStr;
	  outputfileStr << "output-1-" << i << ".csv";
		double h = pow (10, -i);
		yf = simulate (t0, y0, tf, h, rhs_func, euler_integrator, outputfileStr.str());
		cout << "h = " << h << "\teta(tf) = " << yf[0] << "\ty(tf) = " << exact(tf) << "\terror = " << yf[0] - exact(tf) << endl;
	}

	return 0;
}
