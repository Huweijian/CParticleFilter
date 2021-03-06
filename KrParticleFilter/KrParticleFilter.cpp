/*
 References:

 [1] M.S.Arulampalam, S.Maskell, N.Gordon, T.Clapp, "A tutorial on
	   particle filters for online nonlinear / non - Gaussian Bayesian tracking, "
	   IEEE Transactions on Signal Processing, vol. 50, no. 2, pp. 174 - 188,
	   Feb 2002
 [2] Z.Chen, "Bayesian filtering: From Kalman filters to particle filters,
	   and beyond, " Statistics, vol. 182, no. 1, pp. 1-69, 2003
*/

// 禁止STL对Eigen矩阵的warning
#define _SCL_SECURE_NO_WARNINGS
#include <cassert>
#include <iostream>
#include <algorithm>
#include <numeric>

#include "KrParticleFilter.h"

using namespace Eigen;
using namespace std;

namespace pf {

	ParticleFilter::ParticleFilter() {
		// 初始化随机数种子
		std::srand((unsigned int)time(0));
	}

	void ParticleFilter::initilize(int numParticles, Eigen::VectorXd mean, Eigen::MatrixXd covariance, std::vector<bool> circularVariables) {
		assert(numParticles > 0);
		assert(mean.size() != 0);

		// Assign basic particle filter properties
		this->numParticles = numParticles;
		numStateVariables = (unsigned int)mean.size();
		particles = MatrixXd::Zero(numParticles, numStateVariables);
		weights = VectorXd::Ones(numParticles) / numParticles;
		if (circularVariables.size() == 0)
			circularVariables = vector<bool>(numStateVariables, false);

		isStateVariableCircular = circularVariables;

		// Sample from a normal distribution with mean and covariance
		assert(covariance.rows() == numStateVariables);
		sampleGaussian(mean, covariance, isStateVariableCircular);

	}

	// MultinomialResampler.m line 18
	Eigen::VectorXi ParticleFilter::multinominalResampler() {

		// Generate N random numbers and sort them
		VectorXd randSamples = VectorXd::Random(numParticles);
		randSamples = (randSamples.array() + 1) / 2;
		std::sort(randSamples.data(), randSamples.data() + randSamples.size());
		// Find indices in weights corresponding to random numbers in randSamples
		return findSortedSampleIndices(randSamples);
	}

	// Resampler.m line 24
	Eigen::VectorXi ParticleFilter::findSortedSampleIndices(const Eigen::VectorXd & randSamples) {
		int m = (int)weights.size();
		int n = (int)randSamples.size();
		// Compute the cumulative sum of all weights.
		VectorXd cumSumWeight = VectorXd::Zero(numParticles);
		std::partial_sum(weights.data(), weights.data() + weights.size(), cumSumWeight.data());
		assert(abs(cumSumWeight(numParticles - 1) - 1.0) < sqrt(epsilon));

		// Compute indices of resample choices. By default, use index of 1 (fallback in case that weights are not normalized).
		VectorXi sampleIndices = VectorXi::Ones(numParticles);

		//Find elements in cumulative sum that are greater or equal than the random values.Store and return the found indices.

		//Take advantage of the fact that randSamples is sorted, so we only have to iterate through cumSumWeights once.
		int i = 0, j = 0;
		while (i <= n-1 && j <= m-1) {
			while (j < m-1 && cumSumWeight(j) < randSamples(i)) {
				//% Find element in cumulative sum that is greater or equal to random number
				j++;
			}
			//% Random number falls within the interval defined by the weight at index j.Save the index.
			sampleIndices(i) = j;

			//% Look at next random number
			i = i + 1;
		}


		return sampleIndices;
	}

	// ResamplingPolicy.m line 222
	bool ParticleFilter::isResamplingTriggered() {
		bool trigger = false;

		double squardSum = 0;
		double neffRatio = 0;
		switch (triggeredMethod) {
		case RATIO:
			squardSum = weights.squaredNorm();
			assert(squardSum > 0);
			neffRatio = (1 / squardSum) / numParticles;
			trigger = neffRatio < minEffectiveParticleRatio;
			break;
		case INTERVAL:
			intervalCounter += 1;
			trigger = (intervalCounter % samplingInterval == 0);
			break;
		}

		return trigger;
	}

	// ParticleFilter.m line 864
	void ParticleFilter::resample() {
		// TODO: 多种重采样策略
		bool isTriggered = isResamplingTriggered();
		if (isTriggered) {
			MatrixXi sampleIndices = multinominalResampler();
			MatrixXd particlesNew(numParticles, numStateVariables);

			for (unsigned int i = 0; i < numParticles; i++) {
				particlesNew.row(i) = particles.row(sampleIndices(i));
			}

			//cout << "index:" << endl<< sampleIndices << endl;
			//cout << "Old" << endl << particles<<endl;
			//cout << "New" << endl << particlesNew<<endl;

			particles = particlesNew;
			weights = VectorXd::Ones(numParticles) / numParticles;
		}
	}

	// ParticleFilter.m line 837
	void ParticleFilter::sampleGaussian(Eigen::VectorXd &initialMean, Eigen::MatrixXd &initialCov, std::vector<bool> &isCircVar) {
		assert(initialMean.size() == isCircVar.size());
		assert(initialMean.size() == initialCov.rows());

		// Sample for non-circular state variables
		MatrixXd mat = (randn(numParticles, numStateVariables));
		initialCov = initialCov.array().sqrt();
		mat = mat * initialCov;
		mat.rowwise() += initialMean.transpose();
		particles = mat;

		//TODO: Sample for circular state variables
	}


	// Matlab randn
	Eigen::MatrixXd randn(unsigned int row, unsigned int col) {
        static std::mt19937 gen(std::random_device{}());
        static std::normal_distribution<> nd;
		MatrixXd mat = MatrixXd{ row, col }.unaryExpr(
			[&](double x) { return nd(gen); }
		);
		return mat;
	}

    Eigen::VectorXd randn(unsigned int len){
        return randn(len, 1);
    }

    double randn(){
        return randn(1,1)(0,0);
    }

	// ParticleFilter.m line 370
	void ParticleFilter::predict(const Eigen::VectorXd & trans) {
		//% Evolve state of each particle
		if (stateTransitionFcnP)
			(*stateTransitionFcnP)(particles, trans);
		else
			stateTransitionFcn(particles, trans);
	}

	// ParticleFilter.m line 370
	void ParticleFilter::predict(const Eigen::VectorXd & trans, Eigen::VectorXd & stateOut, Eigen::MatrixXd &covOut) {
		predict(trans);

		//TODO: % Wrap all circular variables
		getStateEstimate(stateOut, covOut);
	}

	// ParticleFilter.m line 434
	void ParticleFilter::correct(const Eigen::VectorXd & mes, Eigen::VectorXd & stateOut, Eigen::MatrixXd & covOut) {
		VectorXd likelihood;

		// Determine likelihood of measurement for each particle
		if (measurementLikelihoodFcnP)
			likelihood = (*measurementLikelihoodFcnP)(particles, mes);
		else
			likelihood = measurementLikelihoodFcn(particles, mes);

		// Always add some small fraction to the likelihoods to avoid a singularity when the weights are normalized.
		weights = (weights.array() * likelihood.array() + 1e-99).matrix();

		//  Normalize weights, so they sum up to 1
		weights = weights / weights.sum();

		// Extract state estimate.Doing this before any resampling is suggested by reference[2] 
		// on page 27 because resampling brings extra random variations to the current particles.

		// TODO: 循环变量的状态估计
		getStateEstimate(stateOut, covOut);

		resample();
	}

	// ParticleFilter.m line 504
	void ParticleFilter::getStateEstimate(Eigen::VectorXd & stateOut, Eigen::MatrixXd & covOut) {

		// 均值状态估计
		meanStateEstimator(stateOut, covOut);

		// TODO: 其他状态估计的方法 
		state = stateOut;
		stateCovariance = covOut;

	}

	// MeanStateEstimator.m line 38
	void ParticleFilter::meanStateEstimator(Eigen::VectorXd & stateOut, Eigen::MatrixXd & covOut) {

		// 非循环变量的状态估计
		nonCircleFitToSamples(stateOut, covOut);

		// TODO: 循环变量的状态估计
	}

	// NormalDistribution.m line 130
	void ParticleFilter::nonCircleFitToSamples(Eigen::VectorXd & sampleMean, Eigen::MatrixXd & sampleCov) {

		// Calculate weighted mean(assuming weights are normalized)
		auto weightedParticles = particles.array().colwise() * weights.array();
		sampleMean = weightedParticles.colwise().sum();

		// Calculate unbiased, weighted covariance matrix (assuming weights are normalized)
		auto weightSqr = weights.transpose() * weights;
		double factor = 1.0;
		if (abs(weightSqr - 1.0) > sqrt(epsilon)) {
			factor = 1 / (1 - weightSqr);
		}
		auto meanDiff = particles.rowwise() - sampleMean.transpose();
		sampleCov = factor * (meanDiff.array().colwise() * weights.array()).matrix().transpose() * meanDiff;
	}

}
