#pragma once

#include <string>
#include <vector>
#include "MechanicsBasedKinematics.h"

class SyntheticDataGenerator
{
	std::vector<std::vector<double>> jointTrajectory;
	std::vector<std::vector<double>> tipTrajectory;
	MechanicsBasedKinematics* kinematics;
	CTR* robot;

public:
	SyntheticDataGenerator(CTR* _robot, std::string _jointTrajectoryFile);
	~SyntheticDataGenerator();
	void GenerateTipTrajectory();
	void PrintTipTrajectory(std::string _tipTrajectoryfile);

};
