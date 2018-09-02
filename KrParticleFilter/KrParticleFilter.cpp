#include "KrParticleFilter.h"
#include <cassert>
#include <random>
#include <functional>
#include <iostream>
#include <Eigen/Eigenvalues>

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
	if (circularVariables.size() == 0)
		circularVariables = vector<bool>(numStateVariables, false);
	isStateVariableCircular = circularVariables;

	// Sample from a normal distribution with mean and covariance
	assert(covariance.rows() == numStateVariables);
	sampleGaussian(mean, covariance, isStateVariableCircular);

}

void ParticleFilter::sampleGaussian(Eigen::VectorXd &initialMean, Eigen::MatrixXd &initialCov, std::vector<bool> &isCircVar) {
	assert(initialMean.size() == isCircVar.size());
	assert(initialMean.size() == initialCov.rows());

	//TODO: circular variable initilization
	//TODO: random normalized distribution.
	cout << particles.size() << " " << initialMean.size() << " " << initialCov.size() << endl;
	randn(particles, 10, 1);

}

struct MultiNormalSampler {
	MultiNormalSampler(Eigen::MatrixXd const& covar)
		: MultiNormalSampler(Eigen::VectorXd::Zero(covar.rows()), covar) {
	}

	MultiNormalSampler(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
		: mean(mean) {
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
		transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
	}

	Eigen::VectorXd mean;
	Eigen::MatrixXd transform;
	Eigen::VectorXd operator()() const {
		static std::mt19937 gen{ std::random_device{}() };
		static std::normal_distribution<> dist;
		Eigen::VectorXd res = mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr(
			[&](double x) { return dist(gen); }
		);
		return res;
	}
};

double ParticleFilter::normalSample(double dummy, double mean, double var) {
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::normal_distribution<> nd(mean, var);
	return nd(gen);
}

void ParticleFilter::randn(Eigen::MatrixXd & mat, int row, int col) {
}

