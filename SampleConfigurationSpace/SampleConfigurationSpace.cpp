// SampleConfigurationSpace.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <fstream>
#include <ctime>

#include "LieGroup.h"
#include "Utilities.h"
#include "HTransform.h"
#include "MechanicsBasedKinematics.h"
#include "CTRFactory.h"

void GenerateTrainingData();
void GenerateRandomConfigurations(int num = 100);

void GenerateSamples(CTR* robot, int numOfPointsPerDim, ::std::vector<double*>& configurations);
void RecordTrainingSample(MechanicsBasedKinematics* kinematics, CTR* robot, double* configuration, ::std::ofstream& os);
void TestJointAnglesConversion();
void ExampleCameraRotationComputation();

int _tmain(int argc, _TCHAR* argv[])
{
	//GenerateTrainingData();
	//GenerateRandomConfigurations();
	//TestJointAnglesConversion();
	ExampleCameraRotationComputation();

	return 0;
}


void ExampleCameraRotationComputation()
{
	// You need a robot pointer (the parameters are updated to match the robot to be used for Benoit's surgery on May 5th 2016.
	CTR* robot = CTRFactory::buildCTR("");

	// You also need a pointer to the kinematics class
	MechanicsBasedKinematics* kinematics = new MechanicsBasedKinematics(robot, 100); // the integration grid consists of 100 points (increase if you have convergence problems)

	// Receive joint values from network...
	// a : relative tube rotation [rad]
	// d : relative tube translation [mm]
	// configuration[0] = a21 = theta2 - theta1
	// configuration[1] = a21 = theta3 - theta1
	// configuration[2] = d31 = d3 - d1
	// configuration[3/4] = rigid body rotation/translation
	double configuration[5] = {M_PI,  M_PI, 35, 0*M_PI, 30};

	// Convert the received the configuration to comply with the definition of the mechanics based kinematics implementation
	double rotation[3] = {0};
	double translation[3] = {0};
	MechanicsBasedKinematics::RelativeToAbsolute(robot, configuration, rotation, translation);

	// Solve kinematic solver
	kinematics->ComputeKinematics(rotation, translation);
	
	// Get the transformation defining the tip's pose (CAREFUL!! You still need to compensate for the rigid body motion of the robot)
	// (if you want you can query many points on the robot using the overloaded function. In this case you can visualize the shape next to the camera view)
	SE3 HTip;
	kinematics->GetBishopFrame(HTip);

	// Define transformation to be used for rigid body motion
	SE3 HRigidBody = RotZ(configuration[3]);
	Vec3 PBase = Vec3(0, 0, configuration[4]);
	HRigidBody.SetPosition(PBase);

	// Apply transformation
	HTip = HRigidBody * HTip;

	// Print out translation for testing
	for (int i = 0; i < 3; ++i)
		::std::cout << HTip.GetPosition()[i] << " ";
	::std::cout << ::std::endl;
}


void TestJointAnglesConversion()
{
	CTR* robot = CTRFactory::buildCTR("");

	double configuration[3] = {M_PI, M_PI, 20};

	double rotation[3] = {0}; double translation[3] = {0};
	
	MechanicsBasedKinematics::RelativeToAbsolute(robot, configuration, rotation, translation);

	::std::cout << "Relative Configuration:" ;
	PrintCArray(configuration, 3);
	::std::cout << ::std::endl;

	::std::cout << "Conversion to Absolute" << ::std::endl;
	PrintCArray(rotation, 3);
	PrintCArray(translation, 3);

}


void GenerateRandomConfigurations(int num)
{
	CTR* robot = CTRFactory::buildCTR("");

	double epsilon = 0.001;
	double collarLength = robot->GetTubes()[0].GetCollarLength();
	double translationLowerLimit = robot->GetLowerTubeJointLimits()[2] + epsilon;
	double translationUpperLimit = robot->GetUpperTubeJointLimits()[2] - epsilon;
	double translationRange = (translationUpperLimit - translationLowerLimit);

	MechanicsBasedKinematics* kinematics = new MechanicsBasedKinematics(robot, 100);
	//kinematics->ActivateIVPJacobian();
	
	::std::string filename = GetDateString() + "_random.txt";
	::std::ofstream os(filename.c_str());

	clock_t startTime = clock(); //Start timer

	for (int i = 0; i < num; ++i)
	{
		Eigen::VectorXd randVec = Eigen::VectorXd::Random(3, 1);
		randVec[2] += 1.0;
		randVec[2] *= 0.5;

		randVec[0] *= M_PI;
		randVec[1] *= M_PI;
		randVec[2] *= translationRange;

		//::std::cout << randVec.transpose() << ::std::endl;
		double configuration[6] = {0, randVec[0], randVec[1], 0, -collarLength, randVec[2] + translationLowerLimit};
		RecordTrainingSample(kinematics, robot, configuration, os);
	}

	clock_t endTime = clock(); 

	clock_t timePassed =  endTime - startTime;
	double secondsPassed = timePassed / (double) CLOCKS_PER_SEC;

	::std::cout << "time passed = " << secondsPassed << " [sec]" << ::std::endl;
	
	os.close();

}


void GenerateTrainingData()
{
	CTR* robot = CTRFactory::buildCTR("");
	
	MechanicsBasedKinematics* kinematics = new MechanicsBasedKinematics(robot, 100);
	kinematics->ActivateIVPJacobian();

	::std::string filename = GetDateString() + "_training.txt";
	::std::ofstream os(filename.c_str());

	clock_t startTime = clock(); //Start timer

	int numOfPointPerDim = 51;
	::std::vector<double* > configurations;
	GenerateSamples(robot, numOfPointPerDim, configurations);

	::std::cout << "Generated " << configurations.size() << " configurations"  << ::std::endl;

	int counter = 0;
	for(::std::vector<double*>::iterator it = configurations.begin(); it != configurations.end(); ++it)
	{
		RecordTrainingSample(kinematics, robot, *it, os);
		if (counter % 80000)
			::std::cout << static_cast<double> (counter)/configurations.size();
	}

	clock_t endTime = clock(); 

	clock_t timePassed =  endTime - startTime;
	double secondsPassed = timePassed / (double) CLOCKS_PER_SEC;

	::std::cout << "time passed = " << secondsPassed << " [sec]" << ::std::endl;
	
	os.close();

}


void GenerateSamples(CTR* robot, int numOfPointsPerDim, ::std::vector<double*>& configurations)
{
	double stepRotation = 2 * M_PI/(numOfPointsPerDim - 1);
	
	double epsilon = 0.0001;
	double collarLength = robot->GetTubes()[0].GetCollarLength();
	double translationLowerLimit = robot->GetLowerTubeJointLimits()[2] + epsilon;
	double translationUpperLimit = robot->GetUpperTubeJointLimits()[2] - epsilon;
	double stepTranslation = (translationUpperLimit - translationLowerLimit)/(numOfPointsPerDim - 1);
	
	double initialConfiguration[6] = {0, -1.0 * M_PI, -1.0 * M_PI, 0, -collarLength, translationLowerLimit};

	double* configuration = new double[6];

	for (int i = 0; i < numOfPointsPerDim; ++i)
	{
		for(int j = 0; j < numOfPointsPerDim; ++j)
		{
			for(int k = 0; k < numOfPointsPerDim; ++k)
			{
				double* configurationTmp = new double[6];
				memcpy(configurationTmp, initialConfiguration, sizeof(double) * 6);
				
				configurations.push_back(configurationTmp);

				initialConfiguration[5] += stepTranslation;
			}
			initialConfiguration[5] = translationLowerLimit;
			initialConfiguration[2] += stepRotation;
		}
		initialConfiguration[2] = -M_PI;
		initialConfiguration[1] += stepRotation;
	}
}

void RecordTrainingSample(MechanicsBasedKinematics* kinematics ,CTR* robot, double* configuration, ::std::ofstream& os)
{
	static int failedBVP = 0;
	static int counter = 0;

	double rotation[3] = {0};
	double translation[3] = {0};
	memcpy(rotation, configuration, sizeof(double) * 3);
	memcpy(translation, &configuration[3], sizeof(double) * 3);
	
	double sBP = robot->GetTubes()[0].GetTubeLength(); // arclength corresponding to balanced pair
	double translationOffset = robot->GetLowerTubeJointLimits()[2];

	counter++;
	if(!kinematics->ComputeKinematics(rotation, translation))
	{
		::std::cout << "Number of BVP Failures:" << double(++failedBVP)/counter << ::std::endl;;

		PrintCArray(rotation, 3);
		PrintCArray(translation, 3);

		::std::cout << "-----------------" << ::std::endl;
		return;
	}

	SE3 bishopFrame;
	kinematics->GetBishopFrame(bishopFrame);

	Vec3 tipPosition = bishopFrame.GetPosition();
	Vec3 tipTangent = bishopFrame.GetOrientation().GetZ();

	double configurationRec[3];
	configurationRec[0] = rotation[1];
	configurationRec[1] = rotation[2];
	configurationRec[2] = translation[2];

	for (int i = 0; i < 3; ++i)
	{
		if (i == 2)
			configurationRec[i] -= translationOffset;
		os << configurationRec[i] << "\t";
	}

	for (int i = 0; i < 3; ++i)
		os << tipPosition[i] << "\t";

	for (int i = 0; i < 3; ++i)
		os << tipTangent[i] << "\t";
	
	os << ::std::endl;
}


