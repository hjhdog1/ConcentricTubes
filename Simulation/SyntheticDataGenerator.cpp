#include "SyntheticDataGenerator.h"
#include <fstream>
#include "Utilities.h"

//::std::vector<::std::string> ReadLinesFromFile(const ::std::string& pathToFile);
//::std::vector< double> DoubleVectorFromString(const ::std::string& inputString);

SyntheticDataGenerator::SyntheticDataGenerator(CTR* _robot)
	: lineCounter(0)
{
	this->robot = _robot;
	
	this->kinematics = new MechanicsBasedKinematics(_robot);
	this->kinematics->ActivateIVPJacobian();
}

SyntheticDataGenerator::SyntheticDataGenerator(CTR* _robot, std::string _jointTrajectoryFile)
	: lineCounter(0)
{
	this->robot = _robot;
	
	this->kinematics = new MechanicsBasedKinematics(_robot);
	this->kinematics->ActivateIVPJacobian();

	std::vector<::std::string> lineVector = ReadLinesFromFile(_jointTrajectoryFile);
	for(int i = 0; i < lineVector.size() ; ++i)
	{
		std::vector<double> doubleLineVector = DoubleVectorFromString(lineVector[i]);
		doubleLineVector.erase(doubleLineVector.begin()+1+2*this->robot->GetNumOfTubes(), doubleLineVector.end());
		this->jointTrajectory.push_back(doubleLineVector);
	}
}

SyntheticDataGenerator::~SyntheticDataGenerator()
{
	delete this->kinematics;
}

void SyntheticDataGenerator::ReadJointAndTipTrajectory(std::string _jointTipTrajectoryfile,  bool isEMmeasurement)
{
	int numTubes = this->robot->GetNumOfTubes();
	this->jointTrajectory.clear();
	this->tipTrajectory.clear();

	std::vector<::std::string> lineVector = ReadLinesFromFile(_jointTipTrajectoryfile);
	for(int i = 0; i < lineVector.size() ; ++i)
	{
		std::vector<double> doubleLineVector = DoubleVectorFromString(lineVector[i]);

		std::vector<double> joint(doubleLineVector.begin(), doubleLineVector.begin() + 2*numTubes+1),
							tip(doubleLineVector.begin()+2*numTubes+1, doubleLineVector.end());

		// For now, tip has EM tracker frame. EM tracker has been attached on the robot tip
		// with its x-axis colinearly aligned to z-axis of robot tip in opposite direction.
		// So, the following lines swaps the x- and z-axes with proper signs.
		// Note that it doesn't mean 'tip' is the correct tip orientation. It is correct with z-axis,
		// but not correct with x- and y- axis.
		if(isEMmeasurement)
			for(int j = 0 ; j < 3; j++)
			{
				double temp = tip[3+j];
				tip[3+j] = tip[9+j];
				tip[9+j] = -temp;
			}

		this->jointTrajectory.push_back(joint);
		this->tipTrajectory.push_back(tip);
	}
}

void SyntheticDataGenerator::GenerateTipTrajectory()
{
	this->tipTrajectory.resize(this->jointTrajectory.size());
	
	SE3 tipFrame;
	for(int i = 0 ; i < this->jointTrajectory.size(); ++i)
	{
		double* rotation = this->jointTrajectory[i].data() + 1;
		double* translation = this->jointTrajectory[i].data() + 1 + this->robot->GetNumOfTubes();
		//if(kinematics->ComputeKinematics(rotation, translation))
			//std::cout << "Solved!" << std::endl;
		if(!kinematics->ComputeKinematics(rotation, translation))
			std::cout << rotation[0] << "\t" << rotation[1] << "\t" << rotation[2] << "\t" << translation[0] << "\t" << translation[1] << "\t" << translation[2] << std::endl;
		kinematics->GetBishopFrame(tipFrame);

		this->tipTrajectory[i].resize(12);
		for(int j = 0; j < 3; ++j)
		{
			this->tipTrajectory[i][j] = tipFrame.GetPosition()[j];
			this->tipTrajectory[i][j+3] = tipFrame.GetOrientation().GetX()[j];
			this->tipTrajectory[i][j+6] = tipFrame.GetOrientation().GetY()[j];
			this->tipTrajectory[i][j+9] = tipFrame.GetOrientation().GetZ()[j];
		}
	}
}

void SyntheticDataGenerator::PrintTipTrajectory(std::string _tipTrajectoryFile)
{
	std::ofstream stream(_tipTrajectoryFile);

	for(int i = 0 ; i < this->tipTrajectory.size() ; ++i)
	{
		for(int j = 0 ; j < this->jointTrajectory[i].size() ; ++j)
			stream << this->jointTrajectory[i][j] << "\t";

		for(int j = 0 ; j < this->tipTrajectory[i].size() ; ++j)
		{
			stream << this->tipTrajectory[i][j];
			if(j < this->tipTrajectory[i].size()-1)
				stream << "\t";
			else
				stream << "\n";
		}
	}

	stream.close();
}

//
//::std::vector<::std::string> ReadLinesFromFile(const ::std::string& pathToFile)
//{
//	::std::vector< ::std::string> linesVector;
//
//	::std::ifstream inputFile(pathToFile.c_str());
//	
//	::std::string tempLine;
//	while(::std::getline(inputFile, tempLine))
//		linesVector.push_back(tempLine);
//
//	return linesVector;
//}
//
//
//::std::vector< double> DoubleVectorFromString(const ::std::string& inputString)
//{
//	::std::istringstream ss(inputString);
//
//	::std::vector<double> result;
//	while(!ss.eof())
//	{
//		if( ss != "" )
//		{
//			double tmp;
//			ss >> tmp;
//			result.push_back(tmp);
//		}
//	}
//
//	return result;
//}

bool SyntheticDataGenerator::LoadOneMeasurement(double* pos, double* ori, double* rotation, double* translation)
{
	if(this->lineCounter >= this->tipTrajectory.size())
		return false;

	memcpy(pos, this->tipTrajectory[this->lineCounter].data(), sizeof(double)*3);
	memcpy(ori, this->tipTrajectory[this->lineCounter].data() + 3, sizeof(double)*9);

	memcpy(rotation, this->jointTrajectory[this->lineCounter].data() + 1, sizeof(double) * this->robot->GetNumOfTubes());
	memcpy(translation, this->jointTrajectory[this->lineCounter].data() + 1 + this->robot->GetNumOfTubes(), sizeof(double) * this->robot->GetNumOfTubes());
	this->lineCounter++;

	return true;

}