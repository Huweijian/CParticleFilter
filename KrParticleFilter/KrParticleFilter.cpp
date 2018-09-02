#include "KrParticleFilter.h"
#include <cassert>
#include <random>
#include <functional>
#include <iostream>

using namespace Eigen;
using namespace std;

void ParticleFilter::initilize(int numParticles, Eigen::VectorXd mean, Eigen::MatrixXd covariance, std::vector<bool> circularVariables) {
	assert(numParticles > 0);
	assert(mean.size() != 0);

	// Assign basic particle filter properties
	this->numParticles = numParticles;
	numStateVariables = mean.size();
	particles = MatrixXd::Zero(numParticles, numStateVariables);
	weights = VectorXd::Zero(numParticles);
	isStateVariableCircular = circularVariables;

	// Sample from a normal distribution with mean and covariance
	assert(covariance.size() == numStateVariables);
	sampleGaussian(mean, covariance, circularVariables);

}

void ParticleFilter::sampleGaussian(Eigen::VectorXd initialMean, Eigen::MatrixXd initialCov, std::vector<bool> isCircVar) {
	assert(initialMean.size() == isCircVar.size());
	assert(initialMean.size() == initialCov.size());

	//TODO: circular variable initilization
	//TODO: random normalized distribution.
	randn(particles, 10, 1, initialMean, initialCov);

}


double ParticleFilter::normalSample(double dummy, double mean, double var) {
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::normal_distribution<> nd(mean, var);
	return nd(gen);
}

double test(double hi) {
	return 1.5;
}

void ParticleFilter::randn(Eigen::MatrixXd & mat, int row, int col, Eigen::VectorXd mean, Eigen::VectorXd cov) {
	using namespace std::placeholders;
	for (int i = 0; i < col; i++) {
		//auto sampler = bind(&ParticleFilter::normalSample, _1, mean(i), cov(i));
		//cout << sampler(1) << endl;
		mat.col(i) = MatrixXd::Zero(row, 1).unaryExpr(&test);
	}
	cout << mat(1);
}

