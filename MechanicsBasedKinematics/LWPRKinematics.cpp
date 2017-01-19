#include "LWPRKinematics.h"

#include "Utilities.h"
#include "HTransform.h"

#include <iostream>




LWPRKinematics::LWPRKinematics(const ::std::string& pathToForwardModel):
	modelPath(pathToForwardModel)
{
	this->forwardModel = new LWPR_Object(pathToForwardModel.c_str());
	this->forwardModel->updateD(true);

}


LWPRKinematics::~LWPRKinematics()
{
	delete this->forwardModel;
}

bool
LWPRKinematics::ComputeKinematics(const double* jAng, double* posOrt)
{
	double scalingFactors[5] = {M_PI, M_PI, 35, M_PI, 100};

	::std::vector< double> inputData(jAng, jAng + this->forwardModel->nIn());

	for (int i = 0; i < inputData.size(); ++i)
		inputData[i] *= scalingFactors[i];

	this->CheckJointLimits(inputData);

#ifdef _SCALED_
	inputData[0] /= M_PI;
	inputData[1] /= M_PI;
	inputData[2] = inputData[2]/35.0 ;
#endif
	
	::std::vector<double> outputData = this->forwardModel->predict(inputData, 0.001);

	::std::vector<double> orientation = ::std::vector<double> (outputData.begin() + 3, outputData.end());
	
	this->CompensateForRigidBodyMotion(jAng, outputData.data(), posOrt);
		
	return true;
}

bool
LWPRKinematics::ComputeKinematics(const ::Eigen::VectorXd& jAng, ::Eigen::VectorXd& posOrt)
{
	double scalingFactors[5] = {M_PI, M_PI, 35, M_PI, 100};

	::std::vector< double> inputData(jAng.data(), jAng.data() + this->forwardModel->nIn());

	for (int i = 0; i < inputData.size(); ++i)
		inputData[i] *= scalingFactors[i];

	this->CheckJointLimits(inputData);

#ifdef _SCALED_
	inputData[0] /= M_PI;
	inputData[1] /= M_PI;
	inputData[2] = inputData[2]/35.0 ;
#endif
	
	::std::vector<double> outputData = this->forwardModel->predict(inputData, 0.001);

	::std::vector<double> orientation = ::std::vector<double> (outputData.begin() + 3, outputData.end());
	
	double posOrtTemp[6] = {0};
	this->CompensateForRigidBodyMotion(jAng.data(), outputData.data(), posOrtTemp);
	
	posOrt.resize(6);
	memcpy(posOrt.data(), posOrtTemp, 6 * sizeof(double));
	return true;
}

void 
LWPRKinematics::CompensateForRigidBodyMotion(const double* jAng, const double* posOrt, double* posOrtFinal)
{

	::Eigen::Matrix3d rotation = RotateZ(jAng[3]);
	::Eigen::Vector3d translation;
	
	translation << 0, 0, jAng[4];
	
	HTransform baseTransform(rotation, translation);

	::std::vector<double> finalPosition;
	HTransform::applyHTransform(baseTransform, ::std::vector<double> (posOrt, posOrt + 3), finalPosition);

	memcpy(posOrtFinal, finalPosition.data(), 3 * sizeof(double));

	::std::vector<double> finalOrientation;
	HTransform::applyRotation(baseTransform, ::std::vector<double> (posOrt + 3, posOrt + 6), finalOrientation);

	finalOrientation /= Norm2(finalOrientation);
	double a = Norm2(finalOrientation);

	memcpy(posOrtFinal + 3, finalOrientation.data(), 3 * sizeof(double));

}

bool 
LWPRKinematics::ComputeJacobian(const ::Eigen::VectorXd& configuration, ::Eigen::MatrixXd& J)
{
	J.resize(6, configuration.size());

	double epsilon = 0.00001;
	double invEpsilon = 1.0/epsilon;

	::Eigen::VectorXd currentX, perturbedX;
	this->ComputeKinematics(configuration, currentX);

	::Eigen::VectorXd perturbedConfiguration = configuration;
	for (int i = 0; i < configuration.size(); ++i)
	{
		perturbedConfiguration(i) += epsilon;
		this->ComputeKinematics(perturbedConfiguration, perturbedX);
		J.col(i) = (perturbedX - currentX) * invEpsilon;
		perturbedConfiguration(i) = configuration(i);
	}

	return true;
}

