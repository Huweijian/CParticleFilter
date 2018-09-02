#include <iostream>
#include "KrParticleFilter.h"

using namespace std;
using namespace Eigen;
int main(void) {
	MatrixXd cov(1, 1);
	cov(0, 0) = 10;

	ParticleFilter pf;
	Vector3d v(1, 2, 5);
	pf.initilize(10, VectorXd::Zero(3), v.asDiagonal());
	cin.get();
}