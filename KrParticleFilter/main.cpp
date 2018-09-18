#define _SCL_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>

#include "KrParticleFilter.h"

using namespace std;
using namespace Eigen;
using namespace pf;

// 状态变换函数
void stateTrans(MatrixXd & particles, const VectorXd & state) {
	particles.rowwise() += state.transpose();
	particles = particles + randn((unsigned int)particles.rows(), (unsigned int)particles.cols()) * 0.02;
}

// 更新函数
VectorXd correct(MatrixXd & particles, const VectorXd & measurement) {
	const double sigma = 0.1;
	MatrixXd err = particles.rowwise() - measurement.transpose();
	VectorXd distSquare = err.col(0).array().square() + err.col(1).array().square();
	VectorXd likelihood = distSquare.unaryExpr([&](const double x) {
		return normal_pdf(x, 0.0, sigma);
	});
	return likelihood;
}

int main(void) {

	// 1. 粒子初始化
	ParticleFilter pf;
	pf.initilize(5, Vector2d{ 0, 0 }, Vector2d{ 0.02, 0.02 }.asDiagonal());

	// 2. 设置粒子状态变化函数和更新函数
	pf.stateTransitionFcn = stateTrans;
	pf.measurementLikelihoodFcn = correct;

	// 输出路径和格式
	ofstream of("./MATLAB/res.csv");
	IOFormat csvFmt(StreamPrecision, 0, ", ", "\n", "", "", "", "");

	double lastX = 0;
	double lastY = 0;
	double t = 0;

	// 模拟20秒钟的运动
	while (t < 20) {
		t += 0.2;
		double x = 2 * t;
		double y = 0;

		// 平均状态和协方差
		VectorXd state;
		MatrixXd cov;
		// 3. 预测
		pf.predict(Vector2d{ x - lastX, y - lastY }, state, cov);
		if (((int)(t * 100)) % 10 == 0)
			// 4. 更新
			pf.correct(Vector2d{ x, y }, state, cov);

		// 输出全部粒子状态，平均状态和权重
		MatrixXd pw(pf.particles.rows(), pf.particles.cols() + 1);
		pw << pf.particles, pf.weights;
		MatrixXd pws(pw.rows() + 1, pw.cols());
		pws << pw,
			state.transpose(), MatrixXd::Ones(1, 1);
		of << pws.format(csvFmt) << endl;


		// 仅输出平均状态
		//of  << state.transpose().format(csvFmt) << endl;

		cout << t << endl;
		lastX = x;
		lastY = y;
	}

	cout << "finish";
	cin.get();
}