/*
 * Checker.cpp
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

/*
myStateValidityCheckerClass::myStateValidityCheckerClass(const ob::SpaceInformationPtr &si) {

}*/

#include "StateValidityChecker.h"
#include <queue>

void StateValidityChecker::defaultSettings()
{
	stateSpace_ = mysi_->getStateSpace().get();
	if (!stateSpace_)
		OMPL_ERROR("No state space for motion validator");
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, State &a, State &q) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < na; i++)
		a[i] = A->values[i]; // Get state of rod
	for (unsigned i = 0; i < nq; i++)
		q[i] = Q->values[i]; // Get state of robots
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, State &a) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);

	for (unsigned i = 0; i < na; i++)
		a[i] = A->values[i]; // Get state of rod
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, State &a, State &q1, State &q2) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < na; i++)
		a[i] = A->values[i]; // Set state of rod
	for (unsigned i = 0; i < nq/2; i++) {
		q1[i] = Q->values[i]; // Set state of robot1
		q2[i] = Q->values[i+nq/2]; // Set state of robot1
	}
}

void StateValidityChecker::updateStateVector(const ob::State *state, State a, State q) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < na; i++)
		A->values[i] = a[i];

	for (unsigned i = 0; i < nq; i++)
		Q->values[i] = q[i];
}

void StateValidityChecker::updateStateVector(const ob::State *state, State a, State q1, State q2) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < na; i++)
		A->values[i] = a[i];
	for (unsigned i = 0; i < nq/2; i++) {
		Q->values[i] = q1[i];
		Q->values[i+nq/2]= q2[i];
	}
}

void StateValidityChecker::updateStateVector(const ob::State *state, State a) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);

	for (unsigned i = 0; i < na; i++)
		A->values[i] = a[i];
}

void StateValidityChecker::printStateVector(const ob::State *state) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	State a(na), q(nq);

	for (unsigned i = 0; i < na; i++)
		a[i] = A->values[i]; // Set state of rod
	for (unsigned i = 0; i < nq; i++) {
		q[i] = Q->values[i]; // Set state of robot1
	}

	cout << "a: "; kdl::printVector(a);
	cout << "q: "; kdl::printVector(q);
}

// ----------------------- v GD functions v ----------------------------

bool StateValidityChecker::GDsample(ob::State *st) {

	State a(na), q(nq), q1(nq/2), q2(nq/2);

	if (!GDsample(a, q))
		return false;

	updateStateVector(st, a, q);
	return true;
}

bool StateValidityChecker::GDsample(State &a, State &q) {

	bool flag = true;
	while (flag) {
		// Random rod configuration
		for (int i = 0; i < na; i++)
			a[i] = ((double) rand() / (RAND_MAX)) * 2 * 30 - 30;
		if (!isRodFeasible(a))
			continue;
		Matrix Q = getT(get_Points_on_Rod()-1);

		for (int k = 0; k < 10; k++) {

			// Random joint angles
			for (int i = 0; i < nq; i++)
				q[i] = ((double) rand() / (RAND_MAX)) * 2 * PI_ - PI_;

			if (!GD(q, Q)) // GD checks for joint limits
				continue;
			q = get_GD_result();

			if (withObs && collision_state(getPMatrix(), q))
				continue;

			flag = false;
			break;
		}
	}

	return true;
}

bool StateValidityChecker::GDproject(ob::State *st) {

	State a(na), q(nq);

	// Check that 'a' on the random state is feasible
	retrieveStateVector(st, a, q);

	bool valid = GDproject(a, q);

	if (valid) {
		updateStateVector(st, a, q);
		return true;
	}

	return false;
}

bool StateValidityChecker::GDproject(State a, State &q) {

	if (!isRodFeasible(a))
		return false;
	Matrix Q = getT(get_Points_on_Rod()-1);

	if (!GD(q, Q)) // GD checks for joint limits
		return false;

	q = get_GD_result();

	if (withObs && collision_state(getPMatrix(), q))
		return false;

	return true;
}

bool StateValidityChecker::GDproject(State &q, Matrix Q) {

	State q1(nq/2), q2(nq/2);

	if (!GD(q, Q)) // GD checks for joint limits
		return false;

	q = get_GD_result();

	if (withObs && collision_state(getPMatrix(), q))
		return false;

	return true;
}

// ----------------------- ^ GD functions ^ ----------------------------


// ---------------------------------------------------------------

void StateValidityChecker::log_q(const ob::State *st) {
	State a(na), q(nq);

	retrieveStateVector(st, a, q);
	log_q(a, q);
}

void StateValidityChecker::log_q(State a, State q) {
	std::ofstream qfile, afile, pfile, ai;
	qfile.open("./paths/path.txt");
	afile.open("./paths/afile.txt");
	pfile.open("./paths/rod_path.txt");

	qfile << 1 << endl;
	pfile << 501 << endl;

	for (int j = 0; j < na; j++)
		afile << a[j] << " ";
	for (int j = 0; j < nq; j++)
		qfile << q[j] << " ";

	rod_solve(a);
	State temp(3);
	// Log points on rod to file
	for (int k = 0; k < get_Points_on_Rod(); k++) {
		temp = getP(k);
		pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
	}
	pfile << endl;

	qfile.close();
	afile.close();
	pfile.close();
}

void StateValidityChecker::log_q(Matrix A, Matrix M) {
	std::ofstream qfile, afile, pfile, ai;
	qfile.open("./paths/path.txt");
	afile.open("./paths/afile.txt");
	pfile.open("./paths/rod_path.txt");

	qfile << M.size() << endl;
	pfile << M.size()*501 << endl;

	for (int i = 0; i < M.size(); i++) {
		for (int j = 0; j < nq; j++)
			qfile << M[i][j] << " ";
		for (int j = 0; j < na; j++)
			afile << A[i][j] << " ";
		qfile << endl;
		afile << endl;

		rod_solve(A[i]);
		State temp(3);
		// Log points on rod to file
		for (int k = 0; k < get_Points_on_Rod(); k++) {
			temp = getP(k);
			pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
		}
		pfile << endl;
	}

	qfile.close();
	afile.close();
	pfile.close();
}

// ---------------------------------------------------------------

double StateValidityChecker::normDistance(State a1, State a2) {
	double sum = 0;
	for (int i=0; i < a1.size(); i++)
		sum += pow(a1[i]-a2[i], 2);
	return sqrt(sum);
}

State StateValidityChecker::join_Vectors(State q1, State q2) {

	State q(q1.size()+q2.size());

	for (int i = 0; i < q1.size(); i++) {
		q[i] = q1[i];
		q[i+q1.size()] = q2[i];
	}

	return q;
}

void StateValidityChecker::seperate_Vector(State q, State &q1, State &q2) {

	for (int i = 0; i < q.size()/2; i++) {
		q1[i] = q[i];
		q2[i] = q[i+q.size()/2];
	}
}

double StateValidityChecker::StateDistance(const ob::State *s1, const ob::State *s2) {

	State aa(na), qa(nq);
	State ab(na), qb(nq);

	retrieveStateVector(s1, aa, qa);
	retrieveStateVector(s2, ab, qb);

	double sum = 0;
	for (int i=0; i < aa.size(); i++)
		sum += pow(aa[i]-ab[i], 2);
	for (int i=0; i < qa.size(); i++)
		sum += pow(qa[i]-qb[i], 2);
	return sqrt(sum);
}

double StateValidityChecker::AngleDistance(const ob::State *st1, const ob::State *st2) {
	State q1(nq), q2(nq), a_dummy(na);
	retrieveStateVector(st1, a_dummy, q1);
	retrieveStateVector(st2, a_dummy, q2);

	double sum = 0;
	for (int i = 0; i < nq; i++)
		sum += pow(q1[i]-q2[i], 2);
	return sqrt(sum);
}


// ====================== Check validity ===============================

// Validates a state by computing the passive chain based on a specific IK solution (input) and checking collision
bool StateValidityChecker::isValid(StateVector &S) {

	isValid_counter++;

	if (GDproject(S.a, S.q))
		return true;

	return false;
}

// Validates a state by computing the passive chain based on a specific IK solution (input) and checking collision
bool StateValidityChecker::isValid(ob::State *st) {

	isValid_counter++;

	return GDproject(st);
}

// ====================== v RBS - GD v ===============================

// Calls the Recursive Bi-Section algorithm (Hauser)
bool StateValidityChecker::checkMotionRBS(const ob::State *s1, const ob::State *s2)
{
	// We assume motion starts and ends in a valid configuration - due to projection
	bool result = true;

	State a1(na), a2(na), q1(nq), q2(nq);
	retrieveStateVector(s1, a1, q1);
	retrieveStateVector(s2, a2,  q2);

	StateVector S1(nq);
	S1.copy(a1, q1);
	StateVector S2(nq);
	S2.copy(a2, q2);

	result = checkMotionRBS(S1, S2, 0, 0);

	return result;
}

// Implements local-connection using Recursive Bi-Section Technique (Hauser)
bool StateValidityChecker::checkMotionRBS(StateVector S1, StateVector S2, int recursion_depth, int non_decrease_count) {

	// Check if reached the required resolution
	double d = normDistanceStateVector(S1, S2);

	if (d < RBS_tol)
		return true;

	if (recursion_depth > RBS_max_depth)// || non_decrease_count > 10)
		return false;

	StateVector S_mid(nq);
	midpoint(S1, S2, S_mid);

	// Check obstacles collisions and joint limits
	if (!isValid(S_mid)) // Also updates s_mid with the projected value
		return false;

	//if ( normDistanceStateVector(S1, S_mid) > d || normDistanceStateVector(S_mid, S2) > d )
	//		non_decrease_count++;

	if ( checkMotionRBS(S1, S_mid, recursion_depth+1, non_decrease_count) && checkMotionRBS(S_mid, S2, recursion_depth+1, non_decrease_count) )
		return true;
	else
		return false;
}

// *************** Reconstruct the RBS - for post-processing and validation

// Calls the Recursive Bi-Section algorithm (Hauser)
bool StateValidityChecker::reconstructRBS(const ob::State *s1, const ob::State *s2, Matrix &Confs, Matrix &A)
{
	// We assume motion starts and ends in a valid configuration - due to projection
	bool result = true;

	State a1(na), a2(na), q1(nq), q2(nq);
	retrieveStateVector(s1, a1, q1);
	retrieveStateVector(s2, a2,  q2);

	StateVector S1(nq);
	S1.copy(a1, q1);
	StateVector S2(nq);
	S2.copy(a2, q2);

	Confs.push_back(S1.q);
	Confs.push_back(S2.q);
	A.push_back(S1.a);
	A.push_back(S2.a);

	return reconstructRBS(S1, S2, Confs, A, 0, 1, 1);
}

bool StateValidityChecker::reconstructRBS(StateVector S1, StateVector S2, Matrix &M, Matrix &A, int iteration, int last_index, int firstORsecond) {
	// firstORsecond - tells if the iteration is from the first or second call for the recursion (in the previous iteration).
	// last_index - the last index that was added to M.

	iteration++;

	// Check if reached the required resolution
	double d = normDistanceStateVector(S1, S2);

	if (d < RBS_tol)
		return true;

	if (iteration > RBS_max_depth)// || non_decrease_count > 10)
		return false;

	StateVector S_mid(nq);
	midpoint(S1, S2, S_mid);

	// Check obstacles collisions and joint limits
	if (!isValid(S_mid)) // Also updates s_mid with the projected value
		return false; // Not suppose to happen since we run this function only when local connection feasibility is known

	if (firstORsecond==1) {
		M.insert(M.begin()+last_index, S_mid.q); // Inefficient operation, but this is only for post-processing and validation
		A.insert(A.begin()+last_index, S_mid.a);
	}
	else {
		M.insert(M.begin()+(++last_index), S_mid.q); // Inefficient operation, but this is only for post-processing and validation
		A.insert(A.begin()+last_index, S_mid.a);
	}

	int prev_size = M.size();
	if (!reconstructRBS(S1, S_mid, M, A, iteration, last_index, 1))
		return false;
	last_index += M.size()-prev_size;
	if (!reconstructRBS(S_mid, S2, M, A, iteration, last_index, 2))
		return false;

	return true;
}

double StateValidityChecker::normDistanceStateVector(StateVector S1, StateVector S2) {
	double sum = 0;
	for (int i=0; i < nq; i++)
		sum += pow(S1.q[i]-S2.q[i], 2);
	for (int i=0; i < na; i++)
		sum += pow(S1.a[i]-S2.a[i], 2);
	return sqrt(sum);
}

void StateValidityChecker::midpoint(StateVector S1, StateVector S2, StateVector &S_mid) {

	State a(na), q(nq);

	for (int i = 0; i < nq; i++)
		q[i] = (S1.q[i]+S2.q[i])/2;

	for (int i = 0; i < na; i++)
		a[i] = (S1.a[i]+S2.a[i])/2;

	S_mid.copy(a, q);
}

// =============================== LOG ====================================

void StateValidityChecker::LogPerf2file() {

	std::ofstream myfile;
	myfile.open("./paths/perf_log.txt");

	myfile << final_solved << endl;
	myfile << PlanDistance << endl; // Distance between nodes 1
	myfile << total_runtime << endl; // Overall planning runtime 2
	myfile << kdl::get_IK_counter() << endl; // How many IK checks? 5
	myfile << kdl::get_IK_time() << endl; // IK computation time 6
	myfile << get_collisionCheck_counter() << endl; // How many collision checks? 7
	myfile << get_collisionCheck_time() << endl; // Collision check computation time 8
	myfile << get_isValid_counter() << endl; // How many nodes checked 9
	myfile << nodes_in_path << endl; // Nodes in path 10
	myfile << nodes_in_trees << endl; // 11
	myfile << local_connection_time << endl;
	myfile << local_connection_count << endl;
	myfile << local_connection_success_count << endl;
	myfile << sampling_time << endl;
	myfile << sampling_counter[0] << endl;
	myfile << sampling_counter[1] << endl;
	myfile << get_odes_counter() << endl;
	myfile << get_valid_odes_counter() << endl;
	myfile << get_odes_time() << endl;

	myfile.close();
}
