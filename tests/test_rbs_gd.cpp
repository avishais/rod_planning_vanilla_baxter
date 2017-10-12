#include "../validity_checkers/StateValidityChecker.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

void interpolate(StateVector S, double r, StateVector &Si) {

	for (int i = 0; i < S.a.size(); i++) {
		Si.a[i] = S.a[i] + r*(Si.a[i]-S.a[i]);
	}
	for (int i = 0; i < S.q.size(); i++) {
		Si.q[i] = S.q[i] + r*(Si.q[i]-S.q[i]);
	}

}

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// KDL
	StateValidityChecker svc;

	State a(6), q(14);
	StateVector S1(14), S2(14);

	bool flag = true;
	while (flag) {

		cout << "Sampling ... \n";

		if (!svc.GDsample(a, q))
			continue;
		//a = {-0.727832, 4.47385, -2.02057, -28.5524, -22.7051, 19.6318};
		//q = {0.389213, -1.04944, -0.183361, -1.54181, -0.787979, -0.230507, 1.48241, 1.91819, 0.121899, -2.2043, 0.645127, -1.88113, -0.159913, 0.423325};
		S1.copy(a, q);
		S1.print();

		svc.log_q(a, q);
		//cin.ignore();

		while (1) {
			q = svc.rand_q(14);
			for (int i = 0; i < 6; i++)
				a[i] = fRand(-30, 30);
			//if (!svc.GDsample(a, q))
			//	continue;
			S2.copy(a, q);
			interpolate(S1, 0.1, S2); //(double)rand() / RAND_MAX

			if (svc.GDproject(S2.a, S2.q))
				break;
		}
		S2.print();

		svc.log_q(S2.a, S2.q);
		cout << "2...\n";

		cout << svc.normDistanceStateVector(S1, S2) << endl;
		//cin.ignore();

		flag = !svc.checkMotionRBS(S1, S2, 0);
	}
	cout << "Found RBS.\n";
	S1.print();
	S2.print();

	Matrix path, A;
	path.push_back(S1.q);
	path.push_back(S2.q);
	A.push_back(S1.a);
	A.push_back(S2.a);
	svc.reconstructRBS(S1, S2, path, A, 0, 1, 1);

	svc.log_q(A, path);

	return 0;
}

