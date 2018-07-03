#pragma once
#include <vector>
#include <ctime>
#include <fstream>
#include <iostream>
#include "Eigen/Dense"
#include "CTR.h"
#include "MechanicsBasedKinematics.h"
#include "ShapeMeasurement.h"

class CTRCalibration
{
public:
	CTRCalibration();
	~CTRCalibration();
	void			SetInitParameters(::std::vector<double> initParam);
	void			SetScaleFactor(::std::vector<double> scale);
	void			SetTrainingData(char* trainingXML, int nPointsAtEachConf = 1);
	void			SetValidationData(char* validationXML);
	void			SetOutputFile(char* paramsTXT);
	void			SetNumGridPoints(int numGridPoints)	{m_numOfGridPoints = numGridPoints;};
	void			SetMaxIter(int maxIter)	{m_maxIter = maxIter;};
	void			Calibrate();
	double			Validate(char* outputFile);
	CTR*			CalibratedCTR();

private:
	void			Initialize();
	void			BuildRobots();
	void			Clear();

	void			SetParamsToCTR(CTR* robot, const ::Eigen::VectorXd& params);
	void			UpdateParamsInAllCTR();
	double			ComputeSingleShapeError(CTR* robot, MechanicsBasedKinematics* kinematics, const Measurement& meas, double& max_error_current, bool solveKin = true);
	double			ComputeErrorOnTrainingSet(double& max_error);
	double			ComputeErrorOnValidationSet(double& max_error, int& max_ID);
	void			ComputeErrorJacobian(::Eigen::MatrixXd& error_jacobian);
	double			UpdateParams(double stepSize, double& max_error);
	
	void			PrintOnConsole(double error, double max_error, int iter, double step, double error_val = -1);
	void			PrintInFile(double error, double max_error, double error_val = -1);
	
	void			Tic();
	double			Toc();		// in seconds


private:
	ShapeDataset								m_traingSet, m_validationSet;
	::std::vector<CTR*>							m_robots;
	::std::vector<MechanicsBasedKinematics>		m_kinematics;
	::std::vector<::std::vector<double*>>		m_pRobotParams;
	CTR*										m_calibratedCTR;

	::Eigen::VectorXd							m_params, m_scaleFactor;

	int											m_nPointsAtEachConf;	// number of measurement at each coniguration (normally 1, but sometimes more than 1)
	double										m_tol;
	int											m_maxIter;
	int											m_numOfGridPoints;

	clock_t										m_startTime;

	::std::ofstream								m_stream, m_stream_val;

	::std::string								m_trainingXML, m_validationXML;
};

