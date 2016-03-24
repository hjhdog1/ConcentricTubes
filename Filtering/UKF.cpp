#include "UKF.h"

UKF::UKF(CTR* _robot, const std::vector<double>& _initialStateCov, const std::vector<double>& measurementCov)
	: robot(_robot), alpha(0.001), kappa(0), beta(2)
{
	this->SetMeasurementCovariance(measurementCov);
	this->initialStateCov = _initialStateCov;
}

UKF::~UKF()
{
	delete this->kinematics;
}

void UKF::Initialize()
{
	this->kinematics = new MechanicsBasedKinematics(this->robot);
	this->kinematics->ActivateIVPJacobian();
	
	this->parameters = this->robot->GetFreeParameters();
	
	this->variances = this->robot->GetFreeParameterVariances();

	this->stateDim = this->parameters.size();
	
	this->augDim = 2*this->stateDim + 6;
	
	this->meanAState.resize(this->augDim,1);
	this->meanAState.setZero();
	for(int i = 0 ; i < this->stateDim ; ++i)
		this->meanAState(i) = *this->parameters[i];

	this->measurementEig.resize(6);
	this->measurementEig.setZero();

	this->Wm.resize(2*this->augDim + 1);
	this->Wm.setZero();

	this->Wc.resize(2*this->augDim + 1, 2*this->augDim + 1);
	this->Wc.setZero();

	this->updateLambda();	// This will update the weights (Wc, Wm) as well as lambda.

	this->covAState.resize(this->augDim, this->augDim);
	this->covAState.setZero();
	for(int i = 0 ; i < this->stateDim ; ++i)
	{
		this->covAState(i,i) = this->initialStateCov[i];
		this->covAState(this->stateDim + i, this->stateDim + i) = this->variances[i];
	}
	for(int i = 0 ; i < 6 ; ++i)
		this->covAState(2*this->stateDim + i, 2*this->stateDim + i) = this->measCovariance(i);
	
	this->sigmaPoints.resize(this->augDim, 2*this->augDim + 1);
	this->sigmaPoints.setZero();

	this->dX.resize(this->stateDim, 2*this->augDim + 1);
	this->dX.setZero();
}

void UKF::SetMeasurementCovariance(const std::vector<double> cov)
{
	if(!this->measCovariance.size())
	{
		this->measCovariance.resize(6);
		this->measCovariance.setZero();
	}

	for(int i = 0 ; i < 6 ; ++i)
		this->measCovariance(i) = cov[i];
}

void UKF::SetAlpha(double _alpha)
{
	this->alpha = _alpha;
	this->updateLambda();
}

void UKF::SetBeta(double _beta)
{
	this->beta = _beta;
	this->updateWeights();
}

void UKF::SetKappa(double _kappa)
{
	this->kappa = _kappa;
	this->updateLambda();
}

void UKF::predict()
{
	this->updateSigmaPoints();

	int numOfSigmaPoints = 2*this->augDim + 1;
	
	this->sigmaPoints.block(0,0,this->stateDim, numOfSigmaPoints) += this->sigmaPoints.block(this->stateDim,0,this->stateDim, numOfSigmaPoints);
	
	this->meanAState.segment(0,this->stateDim) = this->sigmaPoints.block(0,0,this->stateDim, numOfSigmaPoints) * this->Wm;
	
	this->dX = this->sigmaPoints.block(0,0,this->stateDim, numOfSigmaPoints);
	for(int i = 0; i < numOfSigmaPoints; ++i)
		this->dX.col(i) -= this->meanAState.segment(0,this->stateDim);

	this->covAState.block(0,0,this->stateDim, this->stateDim) = this->dX * this->Wc * this->dX.transpose();
}

bool UKF::update()
{	
	int numOfSigmaPoints = 2*this->augDim + 1;

	double* rotation = this->robot->GetRotation();
	double* translation = this->robot->GetTranslation();

	SE3 tipFrame;
	Eigen::MatrixXd Y(6, 2*this->augDim + 1);
	Eigen::VectorXd yFromKinematics(6);
	for(int i = 0; i < numOfSigmaPoints ; ++i)
	{
		this->updateModelParameters(this->sigmaPoints.col(i));
		if(!this->kinematics->ComputeKinematics(rotation, translation))
			return false;
		this->kinematics->GetBishopFrame(tipFrame);

		for(int j = 0; j < 3; ++j)
		{
			yFromKinematics(j) = tipFrame.GetPosition()[j];
			yFromKinematics(j+3) = tipFrame.GetOrientation().GetZ()[j];
		}

		Y.col(i) = yFromKinematics + this->sigmaPoints.block(2*this->stateDim, i, 6, 1);
	}

	Eigen::VectorXd meanY = Y * this->Wm;
	Eigen::MatrixXd dY = Y;
	for(int i = 0; i < numOfSigmaPoints ; ++i)
		dY.col(i) -= meanY;

	Eigen::MatrixXd Pyy = dY * this->Wc * dY.transpose();
	Eigen::MatrixXd Pxy = this->dX * this->Wc * dY.transpose();
	Eigen::MatrixXd KalmanGain = Pxy * Pyy.inverse();

	this->meanAState.segment(0,this->stateDim) += KalmanGain * (this->measurementEig - meanY);

	this->covAState.block(0,0,this->stateDim,this->stateDim) -= KalmanGain * Pyy * KalmanGain.transpose();

	this->updateModelParameters();

	return true;
}


void UKF::updateMeasurement(double _rot[9], double _pos[3])
{
	Filter::updateMeasurement(_rot, _pos);

	double temp[6];
	memcpy(temp, _pos, sizeof(double)*3);
	memcpy(&temp[3], &_rot[6], sizeof(double)*3);

	this->measurementEig = Eigen::Map<Eigen::VectorXd>(temp,6);
}

void UKF::updateModelParameters()
{
	for(int i = 0 ; i < this->stateDim ; ++i)
		*this->parameters[i] = this->meanAState(i);
}

void UKF::updateModelParameters(const Eigen::VectorXd& _parameters)
{
	for(int i = 0 ; i < this->stateDim ; ++i)
		*this->parameters[i] = _parameters(i);
}


void UKF::updateSigmaPoints()
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(this->covAState, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXd S;
	S.resize(this->augDim, this->augDim);
	S.setZero();
	for(int i = 0; i < this->augDim ; ++i)
		S(i,i) = sqrt(svd.singularValues()(i));
	Eigen::MatrixXd sqrtCovAState = svd.matrixU() * S * svd.matrixV().transpose();
	this->sigmaPoints.col(0) = this->meanAState;
	double sqrtLambdaL = sqrt(this->lambda + this->augDim);
	for(int i = 0; i < sqrtCovAState.cols(); ++i)
	{
		this->sigmaPoints.col(2*i+1) = this->meanAState + sqrtLambdaL*sqrtCovAState.col(i);
		this->sigmaPoints.col(2*i+2) = this->meanAState - sqrtLambdaL*sqrtCovAState.col(i);
	}
}

void UKF::updateLambda()
{
	this->lambda = this->alpha * this->alpha * (this->augDim + this->kappa) - this->augDim;
	this->updateWeights();
}

void UKF::updateWeights()
{
	this->Wm(0) = this->lambda / (this->augDim + this->lambda);
	this->Wc(0,0) = this->Wm(0) + (1 - this->alpha * this->alpha + this->beta);

	for(int i = 1 ; i <= 2*this->augDim; ++i)
	{
		this->Wm(i) = 0.5/(this->augDim + this->lambda);
		this->Wc(i,i) = this->Wm(i);
	}
}
