#define _SCL_SECURE_NO_WARNINGS
#include <iostream>
#include "KrParticleFilter.h"
#include <ctime>
#include <numeric>
#include <fstream>

using namespace std;
using namespace Eigen;

void stateTrans(MatrixXd & particles, const VectorXd & state) {
	particles.rowwise() += state.transpose();
}

VectorXd correct(MatrixXd & particles, const VectorXd & measurement) {
	return 2*VectorXd::Ones(particles.rows());
}

int main(void) {
	ParticleFilter pf;
	pf.initilize(1000, Vector2d{ 0, 0 }, Vector2d{ 0.1, 0.1}.asDiagonal());
	pf.stateTransitionFcn = stateTrans;
	pf.measurementLikelihoodFcn = correct; 

	VectorXd state;
	MatrixXd cov;
	
	double t = 0;

	double lastX = 0;
	double lastY = 0;
	
	ofstream of("./MATLAB/res.csv");
	while (t < 15) {
		t += 0.05;
		double r = t * 2;
		double x = r*cos(t);
		double y = r*sin(t);
		pf.predict(Vector2d{ x - lastX, y - lastY }, state, cov);
		if (((int)(t * 100)) % 10 == 0)
			pf.correct(Vector2d{ x, y }, state, cov);

		of << t << ", "<< state(0) << ", " << state(1) << endl;
		cout << t << endl;
	}

	cout << "finish";
	
}