#define _SCL_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>

#include "KrParticleFilter.h"
using namespace std;
using namespace Eigen;
using namespace pf;

void stateTrans(MatrixXd & particles, const VectorXd & state) {
	particles.rowwise() += state.transpose();
	particles = particles + randn((unsigned int)particles.rows(), (unsigned int)particles.cols()) * 0.1;
}

VectorXd correct(MatrixXd & particles, const VectorXd & measurement) {
	const double sigma = 0.5;
	MatrixXd err = particles.rowwise() - measurement.transpose();
	VectorXd distSquare = err.col(0).array().square() + err.col(1).array().square();
	VectorXd likelihood = distSquare.unaryExpr([&](const double x) {
		return normal_pdf(x, 0.0, sigma);
	});

	return likelihood;

}

int main(void) {

	ParticleFilter pf;
	pf.initilize(500, Vector2d{ 0, 0 }, Vector2d{ 0.02, 0.02 }.asDiagonal());
	pf.stateTransitionFcn = stateTrans;
	pf.measurementLikelihoodFcn = correct;

	VectorXd state;
	MatrixXd cov;

	double t = 0;

	double lastX = 0;
	double lastY = 0;

	ofstream of("./MATLAB/res.csv");
	IOFormat csvFmt(StreamPrecision, 0, ", ", "\n", "", "", "", "");

	while (t < 20) {

		t += 0.2;
		double x = t;
		double y = 0;
		pf.predict(Vector2d{ x - lastX, y - lastY }, state, cov);
		if (((int)(t * 100)) % 10 == 0)
			pf.correct(Vector2d{ x, y }, state, cov);

		lastX = x;
		lastY = y;

		// output particles
		MatrixXd pw(pf.particles.rows(), pf.particles.cols() + 1);
		pw << pf.particles, pf.weights;
		MatrixXd pws(pw.rows() + 1, pw.cols());
		pws << pw, 
			state.transpose(), MatrixXd::Ones(1, 1);
		//of << pws.format(csvFmt) << endl;


		// output state
		of  << state.transpose().format(csvFmt) << endl;
		cout << t << endl;
	}

	cout << "finish";
	cin.get();
}