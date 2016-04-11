// SampleConfigurationSpace.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <fstream>
#include <ctime>

#include "LieGroup.h"
#include "Utilities.h"
#include "MechanicsBasedKinematics.h"
#include "CTRFactory.h"


void GenerateSamples(CTR* robot, int numOfPointsPerDim, ::std::vector<double*>& configurations);
void RecordTrainingSample(MechanicsBasedKinematics* kinematics, double* configuration, ::std::ofstream& os);

int _tmain(int argc, _TCHAR* argv[])
{
	// Load the robot
	CTR* robot = CTRFactory::buildCTR("");
	
	// Initialize kinematics
	MechanicsBasedKinematics* kinematics = new MechanicsBasedKinematics(robot, 100);
	kinematics->ActivateIVPJacobian();

	// Open the file to store training data
	::std::string filename = GetDateString() + "_training.txt";
	::std::ofstream os(filename.c_str());

	clock_t startTime = clock(); //Start timer

	// Generate the grid points
	int numOfPointPerDim = 50;
	::std::vector<double* > configurations;
	GenerateSamples(robot, numOfPointPerDim, configurations);

	::std::cout << "Generated " << configurations.size() << " configurations"  << ::std::endl;

	// Compute the forward kinematics for each configuration and write the results to a file
	int counter = 0;
	for(::std::vector<double*>::iterator it = configurations.begin(); it != configurations.end(); ++it)
	{
		RecordTrainingSample(kinematics, *it, os);
		if (counter % 80000)
			::std::cout << static_cast<double> (counter)/configurations.size();
	}

	clock_t endTime = clock(); 

	clock_t timePassed =  endTime - startTime;
	double secondsPassed = timePassed / (double) CLOCKS_PER_SEC;

	::std::cout << "time passed = " << secondsPassed << " [sec]" << ::std::endl;
	
	os.close();

	return 0;
}

void GenerateSamples(CTR* robot, int numOfPointsPerDim, ::std::vector<double*>& configurations)
{
	double stepRotation = 3 * M_PI/(numOfPointsPerDim - 1);

	double collarLength = robot->GetTubes()[0].GetCollarLength();
	double translationLimit = robot->GetLowerTubeJointLimits()[2];

	double stepTranslation = translationLimit/(numOfPointsPerDim - 1);
	double initialTranslation3 = - 2 * collarLength - translationLimit;
	double initialConfiguration[6] = {0, -1.5 * M_PI, -1.5 * M_PI, 0, -collarLength, initialTranslation3};

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
			initialConfiguration[5] = initialTranslation3;
			initialConfiguration[2] += stepRotation;
		}
		initialConfiguration[2] = -M_PI;
		initialConfiguration[1] += stepRotation;
	}
}

void RecordTrainingSample(MechanicsBasedKinematics* kinematics ,double* configuration, ::std::ofstream& os)
{
	double rotation[3] = {0};
	double translation[3] = {0};
	memcpy(rotation, configuration, sizeof(double) * 3);
	memcpy(translation, &configuration[3], sizeof(double) * 3);

	kinematics->ComputeKinematics(rotation, translation);

	SE3 bishopFrame;
	kinematics->GetBishopFrame(bishopFrame);

	Vec3 tipPosition = bishopFrame.GetPosition();
	Vec3 tipTangent = bishopFrame.GetOrientation().GetZ();

	double configurationRec[3];
	configurationRec[0] = rotation[1];
	configurationRec[1] = rotation[2];
	configurationRec[2] = translation[2];

	for (int i = 0; i < 3; ++i)
		os << configurationRec[i] << "\t";

	for (int i = 0; i < 3; ++i)
		os << tipPosition[i] << "\t";

	for (int i = 0; i < 3; ++i)
		os << tipTangent[i] << "\t";

	os << ::std::endl;
}

