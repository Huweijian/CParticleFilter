#include <functional>
#include <vector>
#include <string>
#include <ctime>
#include <random>

#include <Eigen\core>
#include <Eigen/Eigenvalues>
namespace pf {

	// 生成row * col大小的独立的正态分布随机数矩阵，均值为0，方差为1
	Eigen::MatrixXd randn(unsigned int row, unsigned int col);

	// 多元正态分布随机数生成器
	struct MultiNormalSampler {
		Eigen::VectorXd mean;
		Eigen::MatrixXd transform;

		// 构造函数 covar 方差，均值为0
		MultiNormalSampler(Eigen::MatrixXd const& covar)
			: MultiNormalSampler(Eigen::VectorXd::Zero(covar.rows()), covar) {
		}

		// 构造函数 mean 均值，covar 方差
		MultiNormalSampler(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
			: mean(mean) {
			Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
			transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
		}
		
		// 生成一组多元正态分布随机数
		Eigen::VectorXd operator()() const {
			static std::mt19937 gen{ std::random_device{}() };
			static std::normal_distribution<> dist;
			Eigen::VectorXd res = mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr(
				[&](double x) { return dist(gen); }
			);
			return res;
		}
	};

	// 正态分布概率密度函数 T 数据类型，x输入，mean 正态分布均值，sigma 正态分布方差
	template <typename T = double>
	T normal_pdf(T x, T mean, T sigma) {
		static const T inv_sqrt_2pi = 0.3989422804014327;
		T a = (x - mean) / sigma;
		return inv_sqrt_2pi / sigma * std::exp(-T(0.5) * a * a);
	}

	// 粒子滤波器
	class ParticleFilter {
	public:
		// 重采样方法
		enum ResampleTriggeredMethod {
			RATIO,  // Trigger resampling if ratio of effective to total particles falls below threshold.
			INTERVAL // Trigger resampling if our interval counter is an integer multiple of the desired SamplingInterval
		};

		// 构造函数
		ParticleFilter();
		// 析构函数
		~ParticleFilter() {};

		// 状态变量数目
		unsigned int numStateVariables;
		// 粒子数目
		unsigned int numParticles;
		// 状态变换函数 输入为所有粒子particles (numParticles * numStateVariables大小的矩阵）和 状态更新量trans (向量) 输出为空
		std::function<void(Eigen::MatrixXd&, const Eigen::VectorXd&)> stateTransitionFcn;
		// 测量似然函数 输入为所有粒子particles (numParticles * numStateVariables大小的矩阵）和 测量值measurement (向量) 输出为每个粒子的似然（长度为numParticles的向量）
		std::function<Eigen::VectorXd(Eigen::MatrixXd, const Eigen::VectorXd&)> measurementLikelihoodFcn;
		// 粒子 (numParticles * numStateVariables大小的矩阵），每一行为一个粒子
		Eigen::MatrixXd particles;
		// 权重
		Eigen::VectorXd weights;
		// 当前状态估计
		Eigen::VectorXd state;
		// 当前协方差
		Eigen::Matrix2d stateCovariance;
		// 重采样策略
		ResampleTriggeredMethod triggeredMethod = RATIO;
		// 最小有效粒子数目阈值（当重采样策略为RATIO时有效）
		double minEffectiveParticleRatio = 0.5;
		// 重采样间隔（当重采样策略为INTERVAL时有效）
		unsigned int samplingInterval = 1;
		// 粒子初始化 numParticles 粒子数目， mean 初始化粒子均值，covariance 初始协方差，circularVariables 是否为循环变量（当前不可用）
		void initilize(int numParticles, Eigen::VectorXd mean, Eigen::MatrixXd covariance, std::vector<bool> circularVariables = std::vector<bool>());
		// 预测 trans 预测值
		void predict(const Eigen::VectorXd &trans);
		// 预测 trans 预测值, stateOut 预测后的平均值，covOut 预测后的协方差
		void predict(const Eigen::VectorXd &trans, Eigen::VectorXd &stateOut, Eigen::MatrixXd &covOut);
		// 更新 measurement 观测值, stateOut 预测后的平均值，covOut 预测后的协方差
		void correct(const Eigen::VectorXd &measurement, Eigen::VectorXd &stateOut, Eigen::MatrixXd &covOut);


		// TODO: 状态估计方法（目前只有加权平均）
		std::string stateEstimationMethod = "meanweight";
		// TODO: 重采样方法（目前只有Multinominal）
		std::string resamplingMethod = "multinominal";
		// TODO: 重采样策略封装成类(Matlab robotics.ResamplingPolicy)
		bool resamplingPolicy = false;
		// TODO: 循环变量标记
		std::vector<bool> isStateVariableCircular;

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
