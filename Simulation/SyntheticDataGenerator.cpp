#include "SyntheticDataGenerator.h"
#include <fstream>

::std::vector<::std::string> ReadLinesFromFile(const ::std::string& pathToFile);
::std::vector< double> DoubleVectorFromString(const ::std::string& inputString);

SyntheticDataGenerator::SyntheticDataGenerator(CTR* _robot, std::string _jointTrajectoryFile)
{
	this->robot = _robot;
	
	this->kinematics = new MechanicsBasedKinematics(_robot);
	this->kinematics->ActivateIVPJacobian();

	std::vector<::std::string> lineVector = ReadLinesFromFile(_jointTrajectoryFile);
	for(int i = 0; i < lineVector.size() ; ++i)
		this->jointTrajectory.push_back(DoubleVectorFromString(lineVector[i]));
}

SyntheticDataGenerator::~SyntheticDataGenerator()
{
	delete this->kinematics;
}

void SyntheticDataGenerator::GenerateTipTrajectory()
{
	this->tipTrajectory.resize(this->jointTrajectory.size());
	
	SE3 tipFrame;
	for(int i = 0 ; i < this->jointTrajectory.size(); ++i)
	{
		double* rotation = this->jointTrajectory[i].data() + 1;
		double* translation = this->jointTrajectory[i].data() + 1 + this->robot->GetNumOfTubes();
		if(kinematics->ComputeKinematics(rotation, translation))
			std::cout << "Solved!" << std::endl;
		kinematics->GetBishopFrame(tipFrame);

		this->tipTrajectory[i].resize(6);
		for(int j = 0; j < 3; ++j)
		{
			this->tipTrajectory[i][j] = tipFrame.GetPosition()[j];
			this->tipTrajectory[i][j+3] = tipFrame.GetOrientation().GetZ()[j];
		}
	}
}

void SyntheticDataGenerator::PrintTipTrajectory(std::string _tipTrajectoryFile)
{
	std::ofstream stream(_tipTrajectoryFile);

	for(int i = 0 ; i < this->tipTrajectory.size() ; ++i)
		for(int j = 0 ; j < this->tipTrajectory[i].size() ; ++j)
		{
			stream << this->tipTrajectory[i][j];
			if(j < this->tipTrajectory[i].size()-1)
				stream << "\t";
			else
				stream << "\n";
		}

	stream.close();
}


::std::vector<::std::string> ReadLinesFromFile(const ::std::string& pathToFile)
{
	::std::vector< ::std::string> linesVector;

	::std::ifstream inputFile(pathToFile.c_str());
	
	::std::string tempLine;
	while(::std::getline(inputFile, tempLine))
		linesVector.push_back(tempLine);

	return linesVector;
}


::std::vector< double> DoubleVectorFromString(const ::std::string& inputString)
{
	::std::istringstream ss(inputString);

	::std::vector<double> result;
	while(!ss.eof())
	{
		double tmp;
		ss >> tmp;
		result.push_back(tmp);
	}
	result.pop_back();

	return result;
}