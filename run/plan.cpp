/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Avishai Sintov */

#include "plan.h"

bool isStateValid(const ob::State *state)
{
	return true;
}

ob::PlannerPtr plan_C::allocatePlanner(ob::SpaceInformationPtr si, plannerType p_type)
{
    switch (p_type)
    {
        case PLANNER_CBIRRT:
        {
            return std::make_shared<og::CBiRRT>(si, maxStep);
            break;
        }
        /*case PLANNER_RRT:
        {
            return std::make_shared<og::RRT>(si, maxStep);
            break;
        }*/
        /*case PLANNER_LAZYRRT:
        {
            return std::make_shared<og::LazyRRT>(si, maxStep);
            break;
        }*/
        /*case PLANNER_PRM:
        {
            return std::make_shared<og::PRM>(si);
            break;
        }
        case PLANNER_SBL:
        {
            return std::make_shared<og::SBL>(si, maxStep);
            break;
        }*/
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}

void plan_C::plan(State c_start, State c_goal, double runtime, plannerType ptype, double max_step) {

	// construct the state space we are planning inz
	ob::CompoundStateSpace *cs = new ob::CompoundStateSpace(); // Compound R^12 configuration space
	ob::StateSpacePtr A(new ob::RealVectorStateSpace(6)); // A-space - state space of the rod - R^6
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(14)); // Angles of Robot 1 & 2 - R^12
	cs->addSubspace(A, 1.0);
	cs->addSubspace(Q, 1.0);

	// set the bounds for the A=R^6
	ob::RealVectorBounds Abounds(6);
	Abounds.setLow(-30);
	Abounds.setHigh(30);

	// set the bounds for the Q=R^14 part of 'Cspace'
	ob::RealVectorBounds Qbounds(14);
	Qbounds.setLow(0, -2.461); // S0
	Qbounds.setHigh(0, 0.89);
	Qbounds.setLow(1, -2.147); // S1
	Qbounds.setHigh(1, +1.047);
	Qbounds.setLow(2, -3.028); // E0
	Qbounds.setHigh(2, +3.028);
	Qbounds.setLow(3, -1.6232); // E1
	Qbounds.setHigh(3, 1.0472);
	Qbounds.setLow(4, -3.059); // W0
	Qbounds.setHigh(4, 3.059);
	Qbounds.setLow(5, -1.571); // W1
	Qbounds.setHigh(5, 2.094);
	Qbounds.setLow(6, -3.059); // W2
	Qbounds.setHigh(6, 3.059);
	// Left arm
	Qbounds.setLow(7, -2.462); // S0
	Qbounds.setHigh(7, 0.89);
	Qbounds.setLow(8, -2.147); // S1
	Qbounds.setHigh(8, +1.047);
	Qbounds.setLow(9, -3.028); // E0
	Qbounds.setHigh(9, +3.028);
	Qbounds.setLow(10, -1.6232); // E1
	Qbounds.setHigh(10, 1.0472);
	Qbounds.setLow(11, -3.059); // W0
	Qbounds.setHigh(11, 3.059);
	Qbounds.setLow(12, -1.571); // W1
	Qbounds.setHigh(12, 2.094);
	Qbounds.setLow(13, -3.059); // W2
	Qbounds.setHigh(13, 3.059);
	// set the bound for the compound space
	cs->as<ob::RealVectorStateSpace>(0)->setBounds(Abounds);
	cs->as<ob::RealVectorStateSpace>(1)->setBounds(Qbounds);

	// construct a compound state space using the overloaded operator+
	ob::StateSpacePtr Cspace(cs);

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(Cspace));

	// set state validity checking for this space
	//si->setStateValidityChecker(ob::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
	si->setStateValidityCheckingResolution(0.1); // 3% ???

	// create start state
	ob::ScopedState<ob::CompoundStateSpace> start(Cspace);
	for (int i = 0; i < 6; i++)
			start->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = c_start[i]; // Access the first component of the start a-state
	for (int i = 0; i < c_start.size()-6; i++)
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[i] = c_start[i+6];

	// create goal state
	ob::ScopedState<ob::CompoundStateSpace> goal(Cspace);
	for (int i = 0; i < 6; i++)
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = c_goal[i]; // Access the first component of the goal a-state
	for (int i = 0; i < c_goal.size()-6; i++)
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[i] = c_goal[i+6];

	// create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	pdef->print();
/*
	// Register new projection evaluator
	if (ptype == PLANNER_SBL) {
		//Cspace->registerProjection("myProjection", ob::ProjectionEvaluatorPtr(new MyProjection(Cspace)));
		Cspace->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new MyProjection(Cspace)));
	}
*/
	maxStep = max_step;
	// create a planner for the defined space
	// To add a planner, the #include library must be added above
	ob::PlannerPtr planner = allocatePlanner(si, ptype);

	//planner->as<og::SBL>()->setProjectionEvaluator("myProjection");

	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

	// perform setup steps for the planner
	planner->setup();

	//planner->printSettings(std::cout); // Prints some parameters such as range
	//planner->printProperties(std::cout); // Prints some decisions such as multithreading, display approx solutions, and optimize?

	// print the settings for this space
	//si->printSettings(std::cout); // Prints state space settings such as check resolution, segmant count factor and bounds
	//si->printProperties(std::cout); // Prints state space properties, average length, dimension ...

	// print the problem settings
	//pdef->print(std::cout); // Prints problem definition such as start and goal states and optimization objective

	// attempt to solve the problem within one second of planning time
	clock_t begin = clock();
	ob::PlannerStatus solved = planner->solve(runtime);
	clock_t end = clock();
	cout << "Runtime: " << double(end - begin) / CLOCKS_PER_SEC << endl;

	if (solved) {
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		//ob::PathPtr path = pdef->getSolutionPath();

		// print the path to screen
		//path->print(std::cout);  // Print as vectors

		// Save path to file
		//std::ofstream myfile;
		//myfile.open("pathGD.txt");
		//og::PathGeometric& pog = static_cast<og::PathGeometric&>(*path); // Transform into geometric path class
		//pog.printAsMatrix(myfile); // Print as matrix to file
		//myfile.close();

		std::cout << "Found solution:" << std::endl;
		solved_bool = true;
	}
	else {
		std::cout << "No solutions found" << std::endl;
		solved_bool = false;
	}
}

int main(int argn, char ** args) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	double runtime; // Maximum allowed runtime
	plannerType ptype; // Planner type
	string plannerName;
	int env; // Tested environment index

	if (argn == 1) {
		runtime = 1; // sec
		ptype = PLANNER_CBIRRT;
		env = 1;
	}
	else if (argn == 2) {
		runtime = atof(args[1]);
		ptype = PLANNER_CBIRRT;
		env = 1;
	}
	else if (argn > 2) {
		runtime = atof(args[1]);
		switch (atoi(args[2])) {
		case 1 :
			ptype = PLANNER_CBIRRT;
			plannerName = "CBiRRT";
			break;
		case 2 :
			ptype = PLANNER_RRT;
			plannerName = "RRT";
			break;
		case 3 :
			ptype = PLANNER_LAZYRRT;
			plannerName = "LazyRRT";
			break;
		case 4 :
			ptype = PLANNER_PRM;
			plannerName = "PRM";
			break;
		case 5 :
			ptype = PLANNER_SBL;
			plannerName = "SBL";
			break;
		default :
			cout << "Error: Requested planner not defined.";
			exit(1);
		}
		if (argn == 4)
			env = atoi(args[3]);
		else
			env = 1;
	}

	plan_C Plan;

	srand (time(NULL));

	State c_start, c_goal;
	if (env == 1) {
		c_start = {-0.228674, -5.79388, 0.376973, -12.1839, 0.804729, 7.27214, -0.350352, -1.5366, 2.87881, -0.453977, -1.50315, 1.44179, -0.604059, -0.429175, -1.34472, 1.10585, -0.78678, 2.76589, -0.860165, -1.67489};
		c_goal = {1.13317, -4.08401, 2.74606, 6.78602, 11.6337, -5.10359, 0.862361, -0.234115, -2.1993, -0.763042, 2.05378, 0.149986, 2.24325, 0.615206, -0.584064, -1.06856, -1.15379, 1.68501, -0.847547, -1.82553};
	}
	else if (env == 2) {

	}

	int mode = 1;
	switch (mode) {
	case 1: {
		Plan.plan(c_start, c_goal, runtime, ptype, 0.5);

		break;
	}
	case 2 : { // Benchmark planning time with constant maximum step size
		ofstream GD;
		GD.open("./matlab/profile/profile_" + plannerName + "_env2.txt", ios::app);

		for (int k = 0; k < 100; k++) {
			//Plan.plan(c_start, c_goal, runtime, ptype, 2.6); // CBiRRT
			Plan.plan(c_start, c_goal, runtime, ptype, 0.6); // SBL

			// Extract from perf file
			ifstream FromFile;
			FromFile.open("./paths/perf_log.txt");
			string line;
			while (getline(FromFile, line))
				GD << line << "\t";
			FromFile.close();
			GD << endl;
		}
		GD.close();
		break;
	}
	case 3 : { // Benchmark maximum step size
		ofstream F;
		if (env == 1)
			F.open("./matlab/Benchmark_" + plannerName + "_env1_rB.txt", ios::app);
		else if (env == 2)
			F.open("./matlab/Benchmark_" + plannerName + "_env2_rB.txt", ios::app);

		for (int k = 0; k < 250; k++) {

			for (int j = 0; j < 11; j++) {
				double maxStep = 0.2 + 0.3*j;

				cout << "** Running Rod planning, iteration " << k+1 << " with maximum step: " << maxStep << " **" << endl;

				Plan.plan(c_start, c_goal, runtime, ptype, maxStep);

				F << maxStep << "\t";

				// Extract from perf file
				ifstream FromFile;
				FromFile.open("./paths/perf_log.txt");
				string line;
				while (getline(FromFile, line))
					F << line << "\t";
				FromFile.close();
				F << endl;
			}
		}
		F.close();
		break;
	}
	}

	std::cout << std::endl << std::endl;

	return 0;
}

