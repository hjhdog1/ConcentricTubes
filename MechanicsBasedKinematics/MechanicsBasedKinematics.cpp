#include "MechanicsBasedKinematics.h"
#include <fstream>

#define CTR_EPSILON 0.0001

MechanicsBasedKinematics::MechanicsBasedKinematics(CTR* _robot, int numOfGridPoints) : maxIter(100)
{
	this->robot = _robot;
	Initialization(numOfGridPoints);
}

MechanicsBasedKinematics::~MechanicsBasedKinematics()
{
	// delete this->robot;
}

bool MechanicsBasedKinematics::ComputeKinematics(double* rotation, double* translation)
{
	if(!this->robot->UpdateConfiguration(rotation, translation))
		return false;

	if(!this->solveBVP(this->BVPSolutionGrid))
		return false;

	this->propagateBishopFrame(this->bishopFrames, this->BVPSolutionGrid);
	
	return true;
}

void MechanicsBasedKinematics::GetBishopFrame(double s, SE3& bishopFrame)
{
	GetBishopFrame(s, bishopFrame, this->bishopFrames);
}

void MechanicsBasedKinematics::GetBishopFrame(std::vector<double> s, std::vector<SE3>& bishopFrame)
{
	int sizeS = s.size();
	if(bishopFrame.size() != sizeS)
		bishopFrame.resize(sizeS);

	for(int i = 0; i < sizeS; ++i)
		this->GetBishopFrame(s[i], bishopFrame[i]);
}

bool MechanicsBasedKinematics::GetControlJacobian(double s, Eigen::MatrixXd& controlJacobian)
{
	int numTubes = this->robot->GetNumOfTubes();

	// Check the size of controlJacobian, and resize if necessary.
	if(controlJacobian.cols() != 2*numTubes || controlJacobian.rows() != 6)
		controlJacobian.resize(6,2*numTubes);
	
	//double* rotation, * translation;
	//this->robot->GetConfiguration(rotation, translation);
	double* rotation = this->robot->GetRotation();
	double* translation = this->robot->GetTranslation();

	double* perturbedRot = new double[numTubes];
	double* perturbedTrans = new double[numTubes];

	memcpy(perturbedRot, rotation, sizeof(double)*numTubes);
	memcpy(perturbedTrans, translation, sizeof(double)*numTubes);

	
	SE3 frameBeforePerturb, frameAfterPerturb;
	
	this->GetBishopFrame(s, frameBeforePerturb, this->bishopFrames);

	double invEps = (1.0/CTR_EPSILON);
	for(int i = 0; i < numTubes; ++i)
	{
		perturbedRot[i] += CTR_EPSILON;
		perturbedTrans[i] += CTR_EPSILON;
		
		// Jacobian w.r.t. rotation
		this->robot->UpdateConfiguration(perturbedRot, translation);
		if(!this->solveBVP(perturbedSolGrid))
			return false;
		this->propagateBishopFrame(this->perturbedBishopFrames, perturbedSolGrid);
		this->GetBishopFrame(s, frameAfterPerturb, this->perturbedBishopFrames);

		se3 TmpColumn = Log(Inv(frameBeforePerturb)*frameAfterPerturb) * invEps;
		controlJacobian.col(i) = Eigen::Map<Eigen::VectorXd>(&TmpColumn[0],6);	// This couldn't work.

		// Jacobin w.r.t. translation
		this->robot->UpdateConfiguration(rotation, perturbedTrans);
		if(!this->solveBVP(perturbedSolGrid))
			return false;
		this->propagateBishopFrame(this->perturbedBishopFrames, perturbedSolGrid);
		this->GetBishopFrame(s, frameAfterPerturb, this->perturbedBishopFrames);

		TmpColumn = Log(Inv(frameBeforePerturb)*frameAfterPerturb) * invEps;
		controlJacobian.col(i + numTubes) = Eigen::Map<Eigen::VectorXd>(&TmpColumn[0],6);	// This couldn't work.

		perturbedRot[i] = rotation[i];
		perturbedTrans[i] = translation[i];
	}
	this->robot->UpdateConfiguration(rotation, translation);

	delete perturbedRot, perturbedTrans;

	return true;
}


bool MechanicsBasedKinematics::solveBVP (Eigen::MatrixXd& solution)
{
	Eigen::VectorXd errorBC;
	for(int i = 0; i < this->maxIter; ++i)
	{
		this->solveIVP(solution, this->boundaryConditionTip);
		
		if(this->hasBVPConverged(solution, errorBC))
			return true;

		this->computeBCJacobian(solution);
		this->updateBC(errorBC);
		
	}


	return false;
}

void MechanicsBasedKinematics::solveIVP(Eigen::MatrixXd& solution, const Eigen::VectorXd& boundaryConditions)
{
	// scale arcLengthGrid to robot length
	this->arcLengthGrid = this->normailizedArcLengthGrid * this->robot->GetLength();
	
	int numTubes = this->robot->GetNumOfTubes();
	int numGridPoints = this->arcLengthGrid.size();

	solution.col(numGridPoints-1).setZero();
	solution.block(0,numGridPoints-1, boundaryConditions.size(),1) = boundaryConditions;

	//std::cout << "solution at Tip = " << solution.col(numGridPoints-1).transpose() << std::endl;
	//std::cout << "boundaryConditions = " << boundaryConditions.transpose() << std::endl;

	
	for(int i = numGridPoints-1; i > 0; --i)
	{
		double s = this->arcLengthGrid[i];
		
		// uz, theta
		vector<int> existingTubeIDs;
		this->robot->GetExistingTubes(s, existingTubeIDs);

		double momentSum = 0;
		double firstKz = 0;
		double first_u_hat_z = 0;

		double ds = s - this->arcLengthGrid[i-1];

		// integrate theta
		solution.block(0,i-1, numTubes,1) = solution.block(0,i, numTubes,1) - solution.block(numTubes,i, numTubes,1) * ds;

		// compute uxy
		double sumkxy = 0, sumkz = 0;
		Vec3 sumRKu(0);
		vector<SO3> Rz(existingTubeIDs.size());
		vector<Vec3> u_hat(existingTubeIDs.size());

		std::vector<double> nu(existingTubeIDs.size());
		for (int j = 0; j < existingTubeIDs.size(); ++j)
		{
			double precurvature[3];
			this->robot->ComputePrecurvature(s, existingTubeIDs[j], precurvature);
			u_hat[j] = Vec3(precurvature[0], precurvature[1], precurvature[2]);

			double kxy = this->robot->GetStiffness(existingTubeIDs[j]);
			nu[j] = this->robot->GetPoissonsRatio(existingTubeIDs[j]);
			double kz = kxy / (1 + nu[j]);
			sumkxy += kxy;
			sumkz += kz;

			double theta = solution(existingTubeIDs[j], i);
			Rz[j] = Exp(0, 0, theta);
			sumRKu += Rz[j] * Vec3(kxy * u_hat[j][0], kxy * u_hat[j][1], kz * u_hat[j][2]);
		}
		Eigen::VectorXd m0 = solution.block(2*numTubes, i, 3,1);
		sumRKu += Vec3(m0[0], m0[1], m0[2]);

		Vec3 inv_sumK(1 / sumkxy, 1 / sumkxy, 1 / sumkz);
		Vec3 u(inv_sumK[0] * sumRKu[0], inv_sumK[1] * sumRKu[1], inv_sumK[2] * sumRKu[2]);	// z components doesn't seem to be used any where.

		solution(solution.rows()-2,i) = u[0];
		solution(solution.rows()-1,i) = u[1];
		
		
		// integrate uz
		for(int j = 0; j < existingTubeIDs.size(); j++)
		{
			Vec3 u_curTube = Inv(Rz[j]) * u;

			double duzds = (1+nu[j]) * (u_curTube[0]*u_hat[j][1] - u_curTube[1]*u_hat[j][0]);

			solution(numTubes + existingTubeIDs[j], i-1) = solution(numTubes + existingTubeIDs[j], i) - duzds * ds;
		}

		// TODO: integrate m (moment) and n (force)

	}

	// uxy at s = 0
	double sumkxy = 0, sumkz = 0;
	Vec3 sumRKu(0);
	vector<SO3> Rz(numTubes);
	vector<Vec3> u_hat(numTubes);

	std::vector<double> nu(numTubes);
	for (int j = 0; j < numTubes; ++j)
	{
		double precurvature[3];
		this->robot->ComputePrecurvature(this->arcLengthGrid[0], j, precurvature);
		u_hat[j] = Vec3(precurvature[0], precurvature[1], precurvature[2]);

		double kxy = this->robot->GetStiffness(j);
		nu[j] = this->robot->GetPoissonsRatio(j);
		double kz = kxy / (1 + nu[j]);
		sumkxy += kxy;
		sumkz += kz;

		double theta = solution(j, 0);
		Rz[j] = Exp(0, 0, theta);
		sumRKu += Rz[j] * Vec3(kxy * u_hat[j][0], kxy * u_hat[j][1], kz * u_hat[j][2]);
	}
	Eigen::VectorXd m0 = solution.block(2*numTubes, 0, 3,1);
	sumRKu += Vec3(m0[0], m0[1], m0[2]);

	Vec3 inv_sumK(1 / sumkxy, 1 / sumkxy, 1 / sumkz);
	Vec3 u(inv_sumK[0] * sumRKu[0], inv_sumK[1] * sumRKu[1], inv_sumK[2] * sumRKu[2]);

	solution(solution.rows()-2,0) = u[0];
	solution(solution.rows()-1,0) = u[1];
		


	//std::cout<< "sol column(0) = " << solution.col(0).transpose() << std::endl;
	//std::cout<< "sol column(1) = " << solution.col(1).transpose() << std::endl;
	//std::cout<< "sol = " << solution << std::endl;
}

void MechanicsBasedKinematics::updateBC(Eigen::VectorXd& errorBC)
{
	boundaryConditionTip += this->jacobianBC.inverse() * errorBC;
	//std::cout << "BC at Tip = [" << boundaryConditionTip.transpose() << "]" << std::endl;
}

void MechanicsBasedKinematics::computeBCJacobian(Eigen::MatrixXd& solution)
{
	this->ComputeBCJacobianNumerical(solution);

	// TODO this->computeBCJacobianAnalytical()
}


void MechanicsBasedKinematics::printSolution(string filename) const
{
	std::ofstream file(filename);
	file << this->arcLengthGrid.transpose() << std::endl;
	file << this->BVPSolutionGrid;
	file.close();
}

void MechanicsBasedKinematics::printBishopFrame(string filename) const
{
	std::ofstream file(filename);
	for(int i = 0; i < bishopFrames.size(); ++i)
	{
		for(int j = 0; j < 12 ; ++j)
			file << this->bishopFrames[i][j] << "\t";
		file << std::endl;
	}
	file.close();
}


bool MechanicsBasedKinematics::hasBVPConverged(Eigen::MatrixXd& solution, Eigen::VectorXd& errorBC)
{
	int numTubes = this->robot->GetNumOfTubes();

	//double* robotRotation, *robotTranslation;
	//this->robot->GetConfiguration(robotRotation, robotTranslation);
	double* robotRotation = this->robot->GetRotation();
	double* robotTranslation = this->robot->GetTranslation();


	Eigen::VectorXd solutionAtBase = solution.block(0,0, 2 * numTubes , 1);
	//Eigen::VectorXd solutionAtBase = solution.col(0);
	//std::cout << "solution at base = " << solutionAtBase.transpose() << std::endl;
	Eigen::VectorXd estimatedBC;
	estimatedBC.resize(numTubes);

	this->ComputeBCAtBase(solutionAtBase, robotTranslation, estimatedBC);

	Eigen::VectorXd desiredBC = Eigen::Map<Eigen::VectorXd>(robotRotation, numTubes);
	errorBC = desiredBC - estimatedBC;
	double errorNorm = errorBC.norm();
	
	//std::cout << "desired BC at Base = [" << desiredBC.transpose() << "], estimated BC at base = [" << estimatedBC.transpose() << "]"<< std::endl;
	//std::cout << "estimated BC at base = [" << estimatedBC.transpose() << "]"<< std::endl;
	//std::cout<< "Convergence error = " << errorNorm << std::endl;
	
	if (errorNorm < 0.001)
		return true;

	return false;
}

void MechanicsBasedKinematics::propagateBishopFrame (std::vector<SE3>& bishopFramesToPropagate, Eigen::MatrixXd& solution)
{
	int numGridPoints = this->arcLengthGrid.size();
	if(bishopFramesToPropagate.size() != numGridPoints)
		bishopFramesToPropagate.resize(numGridPoints);

	//double* robotRotation, *robotTranslation;
	//this->robot->GetConfiguration(robotRotation, robotTranslation);
	double* robotRotation = this->robot->GetRotation();
	double* robotTranslation = this->robot->GetTranslation();

	bishopFramesToPropagate[0].SetEye();
	bishopFramesToPropagate[0].SetPosition(Vec3(0, 0, robotTranslation[0]));
	Eigen::Vector2d uxy;
	for(int i = 1; i < numGridPoints; ++i)
	{
		uxy = solution.block(solution.rows()-2,i,2,1);
		bishopFramesToPropagate[i] = bishopFramesToPropagate[i-1] * Exp(se3(uxy[0],uxy[1],0, 0,0,1) * (this->arcLengthGrid[i] - this->arcLengthGrid[i-1]) );
	}

}

void MechanicsBasedKinematics::findNearestGridPoint(double s, int* beginIdx, double* fracFromBegin)
{
	// TODO: change it to be binary search
	for(int i = 0; i < this->arcLengthGrid.size(); ++i)
		if(s < arcLengthGrid[i])
		{
			*beginIdx = i-1;
			*fracFromBegin = (s - arcLengthGrid[i-1]) / (arcLengthGrid[i] - arcLengthGrid[i-1]);
			return;
		}

}

void MechanicsBasedKinematics::GetBishopFrame(double s, SE3& bishopFrame, std::vector<SE3>& frames)
{
	int beginIdx;
	double frac;
	this->findNearestGridPoint(s, &beginIdx, &frac);

	se3 w1 = Log(Inv(frames[beginIdx])*frames[beginIdx+1]);
	bishopFrame = frames[beginIdx] * Exp(frac*w1);
}

void MechanicsBasedKinematics::ComputeBCJacobianNumerical(Eigen::MatrixXd& solution)
{
	Eigen::VectorXd perturbedBCTip(this->boundaryConditionTip);
		
	//double* translation;
	//this->robot->GetTranslation(translation);
	double* translation = this->robot->GetTranslation();

	Eigen::VectorXd estimatedBCBase, perturbedBCBase;
	estimatedBCBase.resize(this->robot->GetNumOfTubes());
	
	this->ComputeBCAtBase(solution.col(0), translation, estimatedBCBase);
	perturbedBCBase = estimatedBCBase;

	for (int i = 0; i < perturbedBCTip.size(); ++i)
	{
		perturbedBCTip[i] += 0.001;

		this->solveIVP(solution, perturbedBCTip);

		this->ComputeBCAtBase(solution.col(0), translation, perturbedBCBase);
				
		this->jacobianBC.col(i) = (perturbedBCBase - estimatedBCBase) / 0.001;

		perturbedBCTip[i] = this->boundaryConditionTip[i];
	}

	//std::cout << "Jacobian = [" << this->jacobianBC << "]" << std::endl;

}

void MechanicsBasedKinematics::ComputeBCAtBase(const Eigen::VectorXd& solutionAtBase, const double* tubeTranslation, Eigen::VectorXd& estimatedBC)
{
	int numTubes = this->robot->GetNumOfTubes();
	for(int i = 0; i < numTubes; ++i)
		estimatedBC[i] = solutionAtBase[i] - solutionAtBase[i + numTubes] * (tubeTranslation[0] - tubeTranslation[i]);

}

void MechanicsBasedKinematics::Initialization(int numOfGridPoints)
{
	int numTubes = this->robot->GetNumOfTubes();

	this->arcLengthGrid.resize(numOfGridPoints);
	this->normailizedArcLengthGrid.resize(numOfGridPoints);
	this->BVPSolutionGrid.resize(2*numTubes+8, numOfGridPoints);
	this->perturbedSolGrid.resize(2*numTubes+8, numOfGridPoints);
	this->jacobianBC.resize(numTubes, numTubes);

	this->bishopFrames.resize(numOfGridPoints);
	this->perturbedBishopFrames.resize(numOfGridPoints);

	this->boundaryConditionTip.resize(numTubes);

	// set zero
	this->arcLengthGrid.setZero();
	this->BVPSolutionGrid.setZero();
	this->perturbedSolGrid.setZero();
	this->jacobianBC.setZero();
	//this->boundaryConditionTip.setZero();
	this->boundaryConditionTip.setOnes();

	// normalizedArcLengthGrid
	for(int i = 0; i < numOfGridPoints; ++i)
		normailizedArcLengthGrid[i] = 1.0/((double)numOfGridPoints-1.0)*(double)i;
}