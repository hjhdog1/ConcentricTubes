#pragma once

#include "Filter.h"
#include "MechanicsBasedKinematics.h"

class UKF: public Filter
{
	std::vector<double*> parameters;
	std::vector<double> variances;
	Eigen::VectorXd meanAState, measurementEig, measCovariance, Wm; 
	Eigen::MatrixXd covAState, sigmaPoints, dX, Wc;
	double lambda, alpha, beta, kappa;
	MechanicsBasedKinematics* kinematics;
	CTR* robot;
	int augDim, stateDim;
	std::vector<double> initialStateCov;

public:
	UKF(CTR* _robot, const std::vector<double>& initialStateCov, const std::vector<double>& measurementCov);
	~UKF();
	virtual void Initialize();
	void SetAlpha(double _alpha);
	void SetBeta(double _beta);
	void SetKappa(double _kappa);
	
protected:
	virtual void predict();
	virtual bool update();
	
	void SetMeasurementCovariance(const std::vector<double> cov);

	virtual void updateMeasurement(double _rot[9], double _pos[3]);
	void updateModelParameters();
	void updateModelParameters(const Eigen::VectorXd& _parameters);
	void updateSigmaPoints();

	void updateLambda();
	void updateWeights();

};
