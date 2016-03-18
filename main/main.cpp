#include <iostream>
#include "CTRFactory.h"
#include "MechanicsBasedKinematics.h"

// timer
#include <ctime>

void testCTR();

int main()
{
	CTR* const robot = CTRFactory::buildCTR("");

	MechanicsBasedKinematics kinematics(robot,100);

	double rotation[3] = {1,-1,0.5};
	double translation[3] = {0, -17, -74};
	
	clock_t startTime = clock(); //Start timer
	if(kinematics.ComputeKinematics(rotation, translation))
		std::cout << "SOLVED!!" << std::endl;
	else
		std::cout << "FAILED!!" << std::endl;
	clock_t testTime = clock(); // end timer
	clock_t timePassed =  testTime - startTime;
	double secondsPassed = timePassed / (double)CLOCKS_PER_SEC;

	std::cout << "time = " << 1/secondsPassed<< std::endl;

	kinematics.printSolution();
	kinematics.printBishopFrame();

	//_sleep(10000);

	return 0;
}

void testCTR()
{
	CTR* const robot = CTRFactory::buildCTR("");

	double rotation[3] = {0,0,0};
	double translation[3] = {0, -17, -54};
	if(robot->UpdateConfiguration(rotation, translation))
		std::cout << "Configuration 1 OK! Length = " << robot->GetLength() <<  std::endl;

	double precurvature[3] = {0,0,0};
	double s[4] = {129.9,130.1, 149.9, 150.1};
	for(int j = 0; j < 4; ++j)
	{
		for(int i = 0; i < 3; ++i)
			if(robot->ComputePrecurvature(s[j],i,precurvature))
				std::cout << "Precurvature of tube " << i << " is [ " << precurvature[0] << ", "<< precurvature[1] << ", "<< precurvature[2] << "]." << std::endl;
			else
				std::cout << "Tube " << i << " doesn't exist at s = " << s[j] << std::endl;
	}
}