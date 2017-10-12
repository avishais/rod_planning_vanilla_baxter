/*
 * Checker.h
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

#ifndef CHECKER_H_
#define CHECKER_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include "ompl/base/MotionValidator.h"
#include "ompl/base/State.h"
#include <ompl/config.h>

#include "../proj_classes/Rod_ODE_class.h"
#include "kdl_class.h"
#include "collisionDetection.h"

#include <iostream>

#define ARMS_DISTANCE 518.09

namespace ob = ompl::base;
using namespace std;

class StateVector {
public:
	StateVector(int num) {
		n = num;
		q.resize(n);
		a.resize(6);
	}

	void copy(State aa, State qq) {
		q = qq;
		a = aa;
	}

	void print() {
		cout << "a: [";
		for (int i = 0; i < a.size(); i++)
			cout << a[i] << ", ";
		cout << "]\n";
		cout << "q: [";
		for (int i = 0; i < q.size(); i++)
			cout << q[i] << ", ";
		cout << "]\n";
	}

	int n;
	State q;
	State a;
};

class StateValidityChecker : public rod_ode, public kdl, public collisionDetection
{
public:
	StateValidityChecker(const ob::SpaceInformationPtr &si) : mysi_(si.get()), kdl(ARMS_DISTANCE) {}; //Constructor
	StateValidityChecker() : kdl(ARMS_DISTANCE){};

	// ----------------------- v GD functions v ----------------------------

	/** Sample a random configuration */
	bool GDsample(ob::State *);
	bool GDsample(State&, State&);

	/** Project a random configuration */
	bool GDproject(State, State&);
	bool GDproject(State &, Matrix);
	bool GDproject(ob::State *st);

	// ----------------------- ^ GD functions ^ ----------------------------

	/** Log a configuration into path file for simulation */
	void log_q(const ob::State *st);
	void log_q(State a, State q);
	void log_q(Matrix A, Matrix M);

	/** Validity check of a configuration - update state after projection */
	bool isValid(StateVector &S, int active_chain, int IK_sol); // For APC
	bool isValid(StateVector &S); // For GD
	bool isValid(ob::State *st);

	bool checkMotionRBS(const ob::State *s1, const ob::State *s2);
	bool checkMotionRBS(StateVector S1, StateVector S2, int recursion_depth, int non_decrease_count = 0);
	bool checkMotionRBS(const ob::State *s1, const ob::State *s2, int active_chain, int ik_sol);
	bool checkMotionRBS(StateVector S1, StateVector S2, int active_chain, int ik_sol, int recursion_depth, int non_decrease_count);

	bool reconstructRBS(const ob::State *s1, const ob::State *s2, Matrix &Confs, Matrix &A);
	bool reconstructRBS(StateVector S1, StateVector S2, Matrix &M, Matrix &A, int iteration, int last_index, int firstORsecond);

	double normDistanceStateVector(StateVector S1, StateVector S2);
	void midpoint(StateVector S1, StateVector S2, StateVector &S_mid);

	// Retrieve and update
	void retrieveStateVector(const ob::State *state, State &a, State &q);
	void retrieveStateVector(const ob::State *state, State &a, State &q1, State &q2);
	void retrieveStateVector(const ob::State *state, State &a);
	void updateStateVector(const ob::State *state, State a, State q);
	void updateStateVector(const ob::State *state, State a, State q1, State q2);
	void updateStateVector(const ob::State *state, State a);
	void printStateVector(const ob::State *state);

	void defaultSettings();
	double normDistance(State, State);
	double StateDistance(const ob::State *, const ob::State *);
	double AngleDistance(const ob::State *, const ob::State *);

	/** Join the two robots joint vectors */
	State join_Vectors(State, State);

	/** Decouple the two robots joint vectors */
	void seperate_Vector(State, State &, State &);

	// Performance parameters
	int isValid_counter;
	int get_isValid_counter() {
		return isValid_counter;
	}

	// Performance parameters and handle
	double total_runtime; // Total planning time
	clock_t startTime; // Start clock
	clock_t endTime; // End clock
	int nodes_in_path; // Log nodes in path
	int nodes_in_trees; // Log nodes in both trees
	double PlanDistance; // Norm distance from start to goal configurations
	bool final_solved; // Planning query solved?
	double local_connection_time; // Log LC total time
	int local_connection_count; // Log number of LC attempts
	int local_connection_success_count; // Log number of LC success
	double sampling_time;
	State sampling_counter;

	/** Reset log parameters */
	void initiate_log_parameters() {
		kdl::IK_counter = 0;
		kdl::IK_time = 0;
		collisionCheck_counter = 0;
		collisionCheck_time = 0;
		isValid_counter = 0;
		nodes_in_path = 0;
		nodes_in_trees = 0;
		local_connection_time = 0;
		local_connection_count = 0;
		local_connection_success_count = 0;
		sampling_time = 0;
		sampling_counter.resize(2);
		sampling_counter[0] = sampling_counter[1] = 0; // [0/1] - successful/failed sampling
		odes_time = 0;
		valid_odes_counter = 0;
		odes_counter = 0;
	}

	void LogPerf2file();

	int na = 6; // Dimension of rod conf. space
	int nq = 14; // Dimension of robots joint space

private:
	ob::StateSpace *stateSpace_;
	ob::SpaceInformation    *mysi_;

	bool withObs = true; // Include obstacles?
	double RBS_tol = 0.1;
	double RBS_max_depth = 100;
};





#endif /* CHECKER_H_ */
