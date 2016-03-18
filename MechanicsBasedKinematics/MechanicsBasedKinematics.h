#pragma once

#include "CTR.h"
#include"LieGroup.h"

#include <Eigen/dense>

class MechanicsBasedKinematics
{
	CTR* robot;
	Eigen::VectorXd arcLengthGrid, normailizedArcLengthGrid;
	Eigen::MatrixXd BVPSolutionGrid, perturbedSolGrid;
	Eigen::MatrixXd jacobianBC;
	//SE3 baseFrameTransformation;
	std::vector<SE3> bishopFrames, perturbedBishopFrames;
	Eigen::VectorXd boundaryConditionTip;
	int maxIter;

public:
	MechanicsBasedKinematics(CTR* _robot, int numOfGridPoints = 100);
	~MechanicsBasedKinematics();
	bool ComputeKinematics(double* rotation, double* translation);
	void GetBishopFrame(double s, SE3& bishopFrame);
	void GetBishopFrame(std::vector<double> s, std::vector<SE3>& bishopFrame);
	bool GetControlJacobian(double s, Eigen::MatrixXd& controlJacobian);

	void printSolution(string filename = "../solution.txt") const;
	void printBishopFrame(string filename = "../frame.txt") const;

private:
	bool updateConfiguration (double* rotation, double* translation);
	bool solveBVP (Eigen::MatrixXd& solution);
	void solveIVP (Eigen::MatrixXd& solution, const Eigen::VectorXd& boundaryConditions);
	void updateBC (Eigen::VectorXd& errorBC);
	void computeBCJacobian (Eigen::MatrixXd& solution);
	bool hasBVPConverged (Eigen::MatrixXd& solution, Eigen::VectorXd& errorBC);
	void propagateBishopFrame (std::vector<SE3>& bishopFramesToPropagate, Eigen::MatrixXd& solution);
	void findNearestGridPoint(double s, int* beginIdx, double* fracFromBegin);
	void GetBishopFrame(double s, SE3& bishopFrame, std::vector<SE3>& frames);
	void ComputeBCJacobianNumerical(Eigen::MatrixXd& solution);
	void ComputeBCAtBase(const Eigen::VectorXd& solutionAtBase, const double* tubeTranslation, Eigen::VectorXd& estimatedBC);
	void Initialization(int numOfGridPoints);
};
