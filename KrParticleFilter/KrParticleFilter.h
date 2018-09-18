#include <functional>
#include <vector>
#include <string>
#include <ctime>
#include <random>

#include <Eigen\core>
#include <Eigen/Eigenvalues>
namespace pf {

	Eigen::MatrixXd randn(unsigned int row, unsigned int col);

	// 多元正态分布随机采样器
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

	template <typename T = double>
	T normal_pdf(T x, T mean, T sigma) {
		static const T inv_sqrt_2pi = 0.3989422804014327;
		T a = (x - mean) / sigma;
		return inv_sqrt_2pi / sigma * std::exp(-T(0.5) * a * a);
	}

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
		std::function<Eigen::VectorXd(Eigen::MatrixXd, const Eigen::VectorXd&)> measurementLikelihoodFcn;
		std::vector<bool> isStateVariableCircular;
		std::string resamplingMethod = "multinominal";
		std::string stateEstimationMethod = "maxweight";
		Eigen::MatrixXd particles;
		Eigen::VectorXd weights;
		Eigen::VectorXd state;
		Eigen::Matrix2d stateCovariance;
		void initilize(int numParticles, Eigen::VectorXd mean, Eigen::MatrixXd covariance, std::vector<bool> circularVariables = std::vector<bool>());
		void predict(const Eigen::VectorXd &trans);
		void predict(const Eigen::VectorXd &trans, Eigen::VectorXd &stateOut, Eigen::MatrixXd &covOut);
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
		void getStateEstimate(Eigen::VectorXd &stateOut, Eigen::MatrixXd &covOut);
		void meanStateEstimator(Eigen::VectorXd &stateOut, Eigen::MatrixXd &covOut);
		void nonCircleFitToSamples(Eigen::VectorXd &sampleMean, Eigen::MatrixXd & sampleCov);

		int intervalCounter = 0;

		const double epsilon = 1e-6;

	};
}
