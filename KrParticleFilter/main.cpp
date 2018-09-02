#include <iostream>
#include "KrParticleFilter.h"

using namespace std;
using namespace Eigen;
int main(void) {
	MatrixXd cov(1, 1);
	cov(0, 0) = 10;

	ParticleFilter pf;
	
	pf.initilize(10, VectorXd::Zero(1), MatrixXd::Zero(1, 1));
	cin.get();
}