#include <functional>
#include <vector>
#include <array>
#include <string>
#include <Eigen\core>
#include <ctime>
typedef Eigen::VectorXd Particle;

class ParticleFilter {
public:
	enum ResampleTriggeredMethod {
		RATIO,  // Trigger resampling if ratio of effective to total particles falls below threshold.
		INTERVAL // Trigger resampling if our interval counter is an integer multiple of the desired SamplingInterval
	};

	ParticleFilter() {
		std::srand((unsigned int)time(0));
	};
	~ParticleFilter() {};
	unsigned int numStateVariables;
	unsigned int numParticles;
	std::function<void(Eigen::MatrixXd&, const Eigen::VectorXd&)> stateTransitionFcn;
	std::function<Eigen::VectorXd (Eigen::MatrixXd, const Eigen::VectorXd&)> measurementLikelihoodFcn;
	std::vector<bool> isStateVariableCircular;
	std::string resamplingMethod = "multinominal";
	std::string stateEstimationMethod = "maxweight";
	Eigen::MatrixXd particles;
	Eigen::VectorXd weights;
	Particle state;
	Eigen::Matrix2d stateCovariance;
	void initilize(int numParticles, Eigen::VectorXd mean, Eigen::MatrixXd covariance, std::vector<bool> circularVariables = std::vector<bool>());
	void predict(const Eigen::VectorXd &trans);
	void predict(const Eigen::VectorXd &trans, Eigen::VectorXd &stateOut , Eigen::MatrixXd &covOut);
	void correct(const Eigen::VectorXd &measurement, Eigen::VectorXd &stateOut, Eigen::MatrixXd &covOut);

	// TODO: 重采样策略封装成类(Matlab robotics.ResamplingPolicy)
	bool resamplingPolicy = false;
	ResampleTriggeredMethod triggeredMethod = RATIO;
	unsigned int samplingInterval = 1;
	double minEffectiveParticleRatio = 0.5;
	
private:
	bool isResamplingTriggered();
	void resample();
	Eigen::VectorXi multinominalResampler();
	Eigen::VectorXi findSortedSampleIndices(const Eigen::VectorXd & randSamples);

	void sampleGaussian(Eigen::VectorXd & initialMean, Eigen::MatrixXd & initialCov, std::vector<bool>& isCircVar);
	Eigen::MatrixXd randn(int row, int col);
	void getStateEstimate(Eigen::VectorXd &stateOut, Eigen::MatrixXd &covOut);
	void meanStateEstimator(Eigen::VectorXd &stateOut, Eigen::MatrixXd &covOut);
	void nonCircleFitToSamples(Eigen::VectorXd &sampleMean, Eigen::MatrixXd & sampleCov);

	int intervalCounter = 0;

	const double epsilon = 1e-6;

};