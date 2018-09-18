#include <functional>
#include <vector>
#include <string>
#include <ctime>
#include <random>

#include <Eigen\core>
#include <Eigen/Eigenvalues>
namespace pf {

	// ����row * col��С�Ķ�������̬�ֲ���������󣬾�ֵΪ0������Ϊ1
	Eigen::MatrixXd randn(unsigned int row, unsigned int col);

	// ��Ԫ��̬�ֲ������������
	struct MultiNormalSampler {
		Eigen::VectorXd mean;
		Eigen::MatrixXd transform;

		// ���캯�� covar �����ֵΪ0
		MultiNormalSampler(Eigen::MatrixXd const& covar)
			: MultiNormalSampler(Eigen::VectorXd::Zero(covar.rows()), covar) {
		}

		// ���캯�� mean ��ֵ��covar ����
		MultiNormalSampler(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
			: mean(mean) {
			Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
			transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
		}
		
		// ����һ���Ԫ��̬�ֲ������
		Eigen::VectorXd operator()() const {
			static std::mt19937 gen{ std::random_device{}() };
			static std::normal_distribution<> dist;
			Eigen::VectorXd res = mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr(
				[&](double x) { return dist(gen); }
			);
			return res;
		}
	};

	// ��̬�ֲ������ܶȺ��� T �������ͣ�x���룬mean ��̬�ֲ���ֵ��sigma ��̬�ֲ�����
	template <typename T = double>
	T normal_pdf(T x, T mean, T sigma) {
		static const T inv_sqrt_2pi = 0.3989422804014327;
		T a = (x - mean) / sigma;
		return inv_sqrt_2pi / sigma * std::exp(-T(0.5) * a * a);
	}

	// �����˲���
	class ParticleFilter {
	public:
		// �ز�������
		enum ResampleTriggeredMethod {
			RATIO,  // Trigger resampling if ratio of effective to total particles falls below threshold.
			INTERVAL // Trigger resampling if our interval counter is an integer multiple of the desired SamplingInterval
		};

		// ���캯��
		ParticleFilter();
		// ��������
		~ParticleFilter() {};

		// ״̬������Ŀ
		unsigned int numStateVariables;
		// ������Ŀ
		unsigned int numParticles;
		// ״̬�任���� ����Ϊ��������particles (numParticles * numStateVariables��С�ľ��󣩺� ״̬������trans (����) ���Ϊ��
		std::function<void(Eigen::MatrixXd&, const Eigen::VectorXd&)> stateTransitionFcn;
		// ������Ȼ���� ����Ϊ��������particles (numParticles * numStateVariables��С�ľ��󣩺� ����ֵmeasurement (����) ���Ϊÿ�����ӵ���Ȼ������ΪnumParticles��������
		std::function<Eigen::VectorXd(Eigen::MatrixXd, const Eigen::VectorXd&)> measurementLikelihoodFcn;
		// ���� (numParticles * numStateVariables��С�ľ��󣩣�ÿһ��Ϊһ������
		Eigen::MatrixXd particles;
		// Ȩ��
		Eigen::VectorXd weights;
		// ��ǰ״̬����
		Eigen::VectorXd state;
		// ��ǰЭ����
		Eigen::Matrix2d stateCovariance;
		// �ز�������
		ResampleTriggeredMethod triggeredMethod = RATIO;
		// ��С��Ч������Ŀ��ֵ�����ز�������ΪRATIOʱ��Ч��
		double minEffectiveParticleRatio = 0.5;
		// �ز�����������ز�������ΪINTERVALʱ��Ч��
		unsigned int samplingInterval = 1;
		// ���ӳ�ʼ�� numParticles ������Ŀ�� mean ��ʼ�����Ӿ�ֵ��covariance ��ʼЭ���circularVariables �Ƿ�Ϊѭ����������ǰ�����ã�
		void initilize(int numParticles, Eigen::VectorXd mean, Eigen::MatrixXd covariance, std::vector<bool> circularVariables = std::vector<bool>());
		// Ԥ�� trans Ԥ��ֵ
		void predict(const Eigen::VectorXd &trans);
		// Ԥ�� trans Ԥ��ֵ, stateOut Ԥ����ƽ��ֵ��covOut Ԥ����Э����
		void predict(const Eigen::VectorXd &trans, Eigen::VectorXd &stateOut, Eigen::MatrixXd &covOut);
		// ���� measurement �۲�ֵ, stateOut Ԥ����ƽ��ֵ��covOut Ԥ����Э����
		void correct(const Eigen::VectorXd &measurement, Eigen::VectorXd &stateOut, Eigen::MatrixXd &covOut);


		// TODO: ״̬���Ʒ�����Ŀǰֻ�м�Ȩƽ����
		std::string stateEstimationMethod = "meanweight";
		// TODO: �ز���������Ŀǰֻ��Multinominal��
		std::string resamplingMethod = "multinominal";
		// TODO: �ز������Է�װ����(Matlab robotics.ResamplingPolicy)
		bool resamplingPolicy = false;
		// TODO: ѭ���������
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
