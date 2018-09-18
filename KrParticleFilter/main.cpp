#define _SCL_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>

#include "KrParticleFilter.h"

using namespace std;
using namespace Eigen;
using namespace pf;

// ״̬�任����
void stateTrans(MatrixXd & particles, const VectorXd & state) {
	particles.rowwise() += state.transpose();
	particles = particles + randn((unsigned int)particles.rows(), (unsigned int)particles.cols()) * 0.02;
}

// ���º���
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

	// 1. ���ӳ�ʼ��
	ParticleFilter pf;
	pf.initilize(5, Vector2d{ 0, 0 }, Vector2d{ 0.02, 0.02 }.asDiagonal());

	// 2. ��������״̬�仯�����͸��º���
	pf.stateTransitionFcn = stateTrans;
	pf.measurementLikelihoodFcn = correct;

	// ���·���͸�ʽ
	ofstream of("./MATLAB/res.csv");
	IOFormat csvFmt(StreamPrecision, 0, ", ", "\n", "", "", "", "");

	double lastX = 0;
	double lastY = 0;
	double t = 0;

	// ģ��20���ӵ��˶�
	while (t < 20) {
		t += 0.2;
		double x = 2 * t;
		double y = 0;

		// ƽ��״̬��Э����
		VectorXd state;
		MatrixXd cov;
		// 3. Ԥ��
		pf.predict(Vector2d{ x - lastX, y - lastY }, state, cov);
		if (((int)(t * 100)) % 10 == 0)
			// 4. ����
			pf.correct(Vector2d{ x, y }, state, cov);

		// ���ȫ������״̬��ƽ��״̬��Ȩ��
		MatrixXd pw(pf.particles.rows(), pf.particles.cols() + 1);
		pw << pf.particles, pf.weights;
		MatrixXd pws(pw.rows() + 1, pw.cols());
		pws << pw,
			state.transpose(), MatrixXd::Ones(1, 1);
		of << pws.format(csvFmt) << endl;


		// �����ƽ��״̬
		//of  << state.transpose().format(csvFmt) << endl;

		cout << t << endl;
		lastX = x;
		lastY = y;
	}

	cout << "finish";
	cin.get();
}