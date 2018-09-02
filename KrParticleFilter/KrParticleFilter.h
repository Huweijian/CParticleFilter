#include <functional>
#include <vector>
#include <array>
#include <string>
#include <Eigen\core>

typedef Eigen::VectorXd Particle;

class ParticleFilter {
public:
	ParticleFilter() {};
	~ParticleFilter() {};
	unsigned int numStateVariables;
	unsigned int numParticles;
	std::function<void(Eigen::MatrixXd, Eigen::VectorXd)> stateTransitionFcn;
	std::function<void(Eigen::MatrixXd, Eigen::VectorXd)> measurementLikelihoodFcn;
	std::vector<bool> isStateVariableCircular;
	std::string resamplingMethod = "multinominal";
	std::string stateEstimationMethod = "maxweight";
	Eigen::MatrixXd particles;
	Eigen::VectorXd weights;
	Particle state;
	Eigen::Matrix2d stateCovariance;

	// TODO:
	bool resamplingPolicy = false;

	void initilize(int numParticles, Eigen::VectorXd mean, Eigen::MatrixXd covariance, std::vector<bool> circularVariables = std::vector<bool>());




private:
	void sampleGaussian(Eigen::VectorXd & initialMean, Eigen::MatrixXd & initialCov, std::vector<bool>& isCircVar);
	double normalSample(double dummy, double mean, double var);
	void randn(Eigen::MatrixXd & mat, int row, int col);
};