#include <iostream>
#include <iomanip>
#include "CTRFactory.h"
#include "MechanicsBasedKinematics.h"
#include "UKF.h"
#include "SyntheticDataGenerator.h"

// timer
#include <ctime>

void testSimulator();
void testUKF();
void testFreeParameters();
void testCTR();
void testKinematics();
void testKinematicsSingleConfiguration();

int main()
{
	//testKinematicsSingleConfiguration();
	//testKinematics();
	//testFreeParameters();

	testUKF();

	//testSimulator();
	
	
	
	std::cout << "Finished. Press enter to close." << std::endl;
	std::cin.ignore();
	return 0;
}

void testSimulator()
{
	CTR* robot = CTRFactory::buildCTR("");
	
	//vector<double> nominalValues;
	//vector<double*> freeParameters = robot->GetFreeParameters();
	//for(int i = 0; i < freeParameters.size(); ++i)
	//{
	//	nominalValues.push_back(*freeParameters[i]);
	//	*freeParameters[i] *= 1.4;
	//}

	SyntheticDataGenerator syntheticData(robot, "../jointTipTrajectory_measured.txt");
	syntheticData.GenerateTipTrajectory();
	syntheticData.PrintTipTrajectory("../jointTipTrajectory_theoretical.txt");

	
	_sleep(10000);
}

void testUKF()
{
	CTR* robot = CTRFactory::buildCTR("");

	//SyntheticDataGenerator simulator(robot, "../jointTrajectoryFull.txt");
	//simulator.GenerateTipTrajectory();

	SyntheticDataGenerator simulator(robot);
	//simulator.ReadJointAndTipTrajectory("../jointTipTrajectory_measured.txt");
	simulator.ReadJointAndTipTrajectory("../jointTipTrajectory_theoretical.txt", false);

	std::vector<double> nominalValues;
	std::vector<double*> freeParameters = robot->GetFreeParameters();
	for(int i = 0; i < freeParameters.size(); ++i)
	{
		nominalValues.push_back(*freeParameters[i]);
		*freeParameters[i] *= 1.25;
	}

	double measVar[6] = {1, 1, 1, 0.1, 0.1, 0.1};
	std:vector<double> measVarSTL(measVar, measVar+6);
	UKF ukf(robot, robot->GetFreeParameterVariances(), measVarSTL);
	ukf.Initialize();
	

	double pos[3], ori[9], rotation[3], translation[3];
	while(simulator.LoadOneMeasurement(pos, ori, rotation, translation))
	{
		robot->UpdateConfiguration(rotation, translation);
		ukf.StepFilter(ori, pos);
		for(int i = 0; i < freeParameters.size(); ++i)
			std::cout << std::fixed << std::setprecision(1) << (*freeParameters[i] - nominalValues[i])/nominalValues[i] * 100 << "%\t";

		//std::cout << std::setprecision(5) << "\t" << *freeParameters[0] << "\t" << 1 / *freeParameters[1] << "\t" << *freeParameters[2] << "\t"  << 1 / *freeParameters[3];
		//std::cout << std::setprecision(5) << 1 / *freeParameters[0] << "\t" << *freeParameters[1] << "\t" << 1 / *freeParameters[2] << "\t" << *freeParameters[3] << "\t"  << 1 / *freeParameters[4];
		//std::cout << std::setprecision(5) << 1 / *freeParameters[0] << "\t" << 1 / *freeParameters[1] << "\t" << 1 / *freeParameters[2];

		std::cout << std::endl;
	}

	//_sleep(10000);

}

void testFreeParameters()
{
	CTR* robot = CTRFactory::buildCTR("");

	std::vector<double*> freeParameters = robot->GetFreeParameters();
	
	std::cout << "Initial value = " << robot->GetTubes()[0].GetBendingStiffness() << std::endl;
	*freeParameters[0] = 2;
	std::cout << "Changed value = " << robot->GetTubes()[0].GetBendingStiffness() << std::endl;
	
	_sleep(10000);
}

void testCTR()
{
	CTR* const robot = CTRFactory::buildCTR("");

	double rotation[3] = {0,0,0};
	double translation[3] = {0, -17, -54};
	if(robot->UpdateConfiguration(rotation, translation))
		std::cout << "Configuration 1 OK! Length = " << robot->GetLength() <<  std::endl;

	//const double precurvature[3] = {0,0,0};
	const double* precurvature;
	double s[4] = {129.9,130.1, 149.9, 150.1};
	for(int j = 0; j < 4; ++j)
	{
		for(int i = 0; i < 3; ++i)
			if(robot->ComputePrecurvature(s[j],i,&precurvature))
				std::cout << "Precurvature of tube " << i << " is [ " << precurvature[0] << ", "<< precurvature[1] << ", "<< precurvature[2] << "]." << std::endl;
			else
				std::cout << "Tube " << i << " doesn't exist at s = " << s[j] << std::endl;
	}
}


void testKinematics()
{
	CTR* const robot = CTRFactory::buildCTR("");

	MechanicsBasedKinematics kinematics(robot,100);
	kinematics.ActivateIVPJacobian();

	int numBVP = 1000;
	double rotation[3] = {0,0,0};
	double translation[3] = {0,0,0};

	//double rotation[3] = {1,-1,0.5};
	//double translation[3] = {0, -17, -74};

	//double rotation[3] = {5.5292, 1.25664, 0.0628319};
	//double translation[3] = {83, 66, -35.6659};

	clock_t startTime = clock(); //Start timer

	for(int i = 0; i < numBVP; ++i)
	{
		for(int j = 0 ; j < 3; ++j)
			rotation[j] = (double)(rand()%100)/100.0*2*M_PI;

		translation[0] = (double)(rand()%100);
		translation[1] = translation[0] - 17;
		translation[2] = translation[0] - 34 - (double)(rand()%100)/100.0 * 86.3938;

		if(!kinematics.ComputeKinematics(rotation, translation))
		{
			std::cout << "rot. = " << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << ", \t \t";
			std::cout << "trans. = " << translation[0] << ", " << translation[1] << ", " << translation[2] << std::endl;

			std::cout << "FAILED!!" << std::endl;
		}
	}

	clock_t testTime = clock(); // end timer
	clock_t timePassed =  testTime - startTime;
	double secondsPassed = timePassed / (double)CLOCKS_PER_SEC;

	std::cout << "Freq. = " << numBVP/secondsPassed << "Hz" << std::endl;

	_sleep(10000);
}


void testKinematicsSingleConfiguration()
{
	CTR* const robot = CTRFactory::buildCTR("");

	MechanicsBasedKinematics kinematics(robot,100);
	kinematics.ActivateIVPJacobian();

	int numBVP = 1;
	
	double rotation[3] = {0, 6.28318531, 6.28318531};
	double translation[3] = {0, -17.00000000, -104.00000000 };


	if(kinematics.ComputeKinematics(rotation, translation))
		std::cout << "Solved!!" << std::endl;
	else
		std::cout << "Failed!!" << std::endl;


	kinematics.printSolution();
	kinematics.printBishopFrame();

	_sleep(10000);
}
