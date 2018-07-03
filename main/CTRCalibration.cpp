#include "CTRCalibration.h"
#include "CTRFactory.h"


CTRCalibration::CTRCalibration()
{
	Initialize();
}

CTRCalibration::~CTRCalibration()
{
	Clear();
}

void CTRCalibration::SetInitParameters(::std::vector<double> initParam)
{
	m_params.resize(initParam.size());
	for(int i = 0 ; i < m_params.size() ; i++)
		m_params(i) = initParam[i];

	UpdateParamsInAllCTR();
}

void CTRCalibration::SetScaleFactor(::std::vector<double> scale)
{
	m_scaleFactor.resize(scale.size());
	for(int i = 0 ; i < m_params.size() ; i++)
		m_scaleFactor(i) = scale[i];
}

void CTRCalibration::SetTrainingData(char* trainingXML, int nPointsAtEachConf)
{
	BuildShapeDatasetFromString(trainingXML, m_traingSet);
	m_nPointsAtEachConf = nPointsAtEachConf;
	BuildRobots();

	m_trainingXML = std::string(trainingXML);

	::std::cout << "Number of loaded measurements:" << m_traingSet.size() << ::std::endl;	
}

void CTRCalibration::SetValidationData(char* validationXML)
{
	BuildShapeDatasetFromString(validationXML, m_validationSet);

	m_validationXML = std::string(validationXML);
}

void CTRCalibration::SetOutputFile(char* paramsTXT)
{
	m_stream.open(paramsTXT);
}

void CTRCalibration::Calibrate()
{
	double error_prev = 1e10;
	double max_error = 0.0;
	double error = ComputeErrorOnTrainingSet(max_error);
	int iter = 0;
	double init_Step = 0.0001;
	double step = init_Step;

	double error_val = 0;
	double error_val_max = 0;
	int id;

	// start timer
	Tic();
	::std::cout << "Calibration started" << ::std::endl;
	while( ::std::abs(error - error_prev) > m_tol && iter < m_maxIter )
	{
		error_prev = error;

		// update parameters
		max_error = 0.0;
		error = UpdateParams(step, max_error);

		error_val = this->ComputeErrorOnValidationSet(error_val_max, id);
		// update stepsize
		if (error > error_prev)
			step = ::std::max(0.95*step, 0.5*init_Step);

		// update iter
		iter++;

		// print intermadiate results
		if (iter % 10 == 0)
			PrintOnConsole(error, max_error, iter, step, error_val);

		if (iter % 50 == 0)
			PrintInFile(error, max_error, error_val);
	}
	m_stream.close();

	// print result
	::std::cout << "Calibrated model parameters:" << m_params.transpose() << ::std::endl;  

	// builed calibrated ctr
	m_calibratedCTR = CTRFactory::buildCTR("");
	SetParamsToCTR(m_calibratedCTR, m_params);
}

double CTRCalibration::Validate(char* outputFile)
{
	m_stream_val.open(outputFile);

	double max_error = 0.0;
	int max_ID = 0;
	double error = ComputeErrorOnValidationSet(max_error, max_ID);

	::std::cout << "========= training set: " << m_trainingXML << " =========" << ::std::endl;
	::std::cout << "========= validation set: " << m_validationXML << " =========" << ::std::endl;
	::std::cout << "Calibrated model parameters:" << m_params.transpose() << ::std::endl; 
	::std::cout << "mean error: " << error << ", \t max error(" << max_ID <<  "-th conf) : " << max_error << ::std::endl;

	m_stream_val << "mean error: " << error << ", \t max error: " << max_error << ::std::endl;
	m_stream_val.flush();
	m_stream_val.close();

	return 0;
}

CTR* CTRCalibration::CalibratedCTR()
{
	return m_calibratedCTR;
}

void CTRCalibration::Initialize()
{
	// set variables
	m_calibratedCTR = NULL;
	m_nPointsAtEachConf = 1;
	m_tol = 0.000000000000001;
	m_maxIter = 1000;
	m_numOfGridPoints = 100;

	// set init params
	CTR* tempRobot = CTRFactory::buildCTR("");
	int num_of_free_params = tempRobot->GetFreeParameters().size();
	m_params.resize(num_of_free_params);
	for(int i = 0 ; i < m_params.size(); i++)
		m_params(i) = *(tempRobot->GetFreeParameters()[i]);
	delete tempRobot;

	// set scale factors
	m_scaleFactor.resize(num_of_free_params);
	double temp_scale_factor[9] = {100, 100, 1, 100, 100, 1, 100, 100, 1};
	//double temp_scale_factor[8] = {100, 100, 1, 100, 100, 1, 100, 100};
	for(int i = 0 ; i < m_scaleFactor.size(); i++)
		m_scaleFactor(i) = temp_scale_factor[i];
	

	// init timer
	Tic();
}

void CTRCalibration::BuildRobots()
{
	Clear();
	
	int nMeasurement = m_traingSet.size();
	int nRobots = nMeasurement/m_nPointsAtEachConf;		// equal to num of configs

	m_robots.resize(nRobots);
	m_pRobotParams.resize(nRobots);
	for(int i = 0 ; i < nRobots; i++)
	{
		m_robots[i] = CTRFactory::buildCTR("");
		m_kinematics.push_back(MechanicsBasedKinematics(m_robots[i], m_numOfGridPoints));
		m_kinematics[i].ActivateIVPJacobian();
		m_pRobotParams[i] = m_robots[i]->GetFreeParameters();
	}
}

void CTRCalibration::Clear()
{
	int nRobots = m_robots.size();
	for(int i = 0 ; i < nRobots; i++)
		delete m_robots[i];

	m_robots.clear();
	m_kinematics.clear();
	m_pRobotParams.clear();
}

void CTRCalibration::SetParamsToCTR(CTR* robot, const ::Eigen::VectorXd& params)
{
	::std::vector<double*> freeParams = robot->GetFreeParameters();
	for(int j = 0 ; j < params.size() ; j++)
		*freeParams[j] = params(j);
}

void CTRCalibration::UpdateParamsInAllCTR()
{
	int nRobots = m_robots.size();
	for(int i = 0 ; i < nRobots; i++)
		for(int j = 0 ; j < m_params.size() ; j++)
			*m_pRobotParams[i][j] = m_params(j);
}

double CTRCalibration::ComputeSingleShapeError(CTR* robot, MechanicsBasedKinematics* kinematics, const Measurement& meas, double& max_error_current, bool solveKin)
{
	double rotation[3] = {0};
	double translation[3] = {0};
	double relativeConf[5];

	memcpy(relativeConf, meas.GetConfiguration().data(), sizeof(double) * 5);
	relativeConf[0] *= M_PI/180.0;
	relativeConf[1] *= M_PI/180.0;
	relativeConf[3] *= M_PI/180.0;

	::std::vector<::Eigen::Vector3d> positionsAlongRobotExp = meas.GetShapeEig();
	::std::vector<::Eigen::Vector3d> positionsAlongRobotModel;
	::Eigen::Vector3d error;

	MechanicsBasedKinematics::RelativeToAbsolute(robot, relativeConf, rotation, translation);

	::std::vector<double> robot_length_parameter = meas.GetArcLength();
	
	if(solveKin)
		if(!kinematics->ComputeKinematics(rotation, translation))
			return -1.0;

	kinematics->GetRobotShape(robot_length_parameter, positionsAlongRobotModel);

	double sum = 0;

	for(int i = 0; i < positionsAlongRobotModel.size(); ++i)
	{
		error = positionsAlongRobotExp[i] - positionsAlongRobotModel[i];
		if (error.norm() > max_error_current)
			max_error_current = error.norm();
		sum += error.norm() * error.norm();
	}

	return sum/positionsAlongRobotModel.size();
	//error = positionsAlongRobotExp[positionsAlongRobotExp.size() - 1] - positionsAlongRobotModel[positionsAlongRobotModel.size() - 1];
	//max_error_current = error.norm();
	//return max_error_current;
}

double CTRCalibration::ComputeErrorOnTrainingSet(double& max_error)
{
	double error = 0.0;
	double tmp = 0.0;
	double max_error_current = 0.0;
	int counter = 0;
	for (int i = 0; i < m_traingSet.size(); i += m_nPointsAtEachConf)
	{
		int robotId = i/m_nPointsAtEachConf;
		CTR* robot = m_robots[robotId];
		MechanicsBasedKinematics* kinematics = &m_kinematics[robotId];

		tmp = ComputeSingleShapeError(robot, kinematics, m_traingSet[i], max_error_current);
		if (tmp < 0)
			continue;
		else
		{
			error += tmp;
			counter += m_nPointsAtEachConf;

			for(int j = 1 ; j < m_nPointsAtEachConf ; j++)
			{
				double temp_max = 0.0;
				tmp = ComputeSingleShapeError(robot, kinematics, m_traingSet[i+j], temp_max, false);
				error += tmp;

				if (temp_max > max_error_current)
					max_error_current = temp_max;
			}
		}

		if (max_error_current > max_error)
		{
			max_error = max_error_current;
			//::std::cout << "id: " << i << ", conf: ";
			//for (int k = 0 ; k < 5; k++)
			//	::std::cout << dataset[i].GetConfiguration()[k] << "	";
			//::std::cout << ::std::endl;
		}
	}

	return ::std::sqrt(error/counter);
}

double CTRCalibration::ComputeErrorOnValidationSet(double& max_error, int& max_ID)
{
	CTR* robot = CTRFactory::buildCTR("");
	SetParamsToCTR(robot, m_params);
	MechanicsBasedKinematics kinematics(robot, m_numOfGridPoints);
	kinematics.ActivateIVPJacobian();

	double error = 0.0;
	double tmp = 0.0;
	double max_error_current = 0.0;
	int counter = 0;
	//for (int i = 0; i < m_validationSet.size(); i++)
	for (int i = m_validationSet.size() -1; i >=0 ; i--)
	{
		tmp = ComputeSingleShapeError(robot, &kinematics, m_validationSet[i], max_error_current);
		if (tmp < 0)
			continue;
		else
		{
			error += tmp;
			counter++;

			//::std::cout << i << "-th conf: " << tmp << ::std::endl;
			
			::Eigen::Vector3d position;
			kinematics.GetTipPosition(position);
			m_stream_val << i << "\t" << tmp << "\t" << position.transpose() << ::std::endl;
			
		}

		if (max_error_current > max_error)
		{
			max_error = max_error_current;
			max_ID = i;
			//::std::cout << "id: " << i << ", conf: ";
			//for (int k = 0 ; k < 5; k++)
			//	::std::cout << dataset[i].GetConfiguration()[k] << "	";
			//::std::cout << ::std::endl;
		}
	}

	delete robot;

	return ::std::sqrt(error/counter);
}

void CTRCalibration::ComputeErrorJacobian(::Eigen::MatrixXd& error_jacobian)
{
	error_jacobian.resize(m_params.size(),1);
	::Eigen::VectorXd params_original(m_params);

	double epsilon = 0.00001;
	double invEpsilon = 1.0/epsilon;
	double error_perturbed = 0;
	double error_perturbed2 = 0;
	double dummy = 0;
	for(int i = 0; i < m_params.size(); ++i)
	{
		double cur_epsilon = epsilon/m_scaleFactor[i];
		m_params[i] += cur_epsilon;
		//*(robot->GetFreeParameters()[i]) = params[i];
		UpdateParamsInAllCTR();
		error_perturbed = ComputeErrorOnTrainingSet(dummy);
		//error_perturbed = ComputeErrorOnDatasetShape(robot, kinematics, dataset, dummy);
		m_params[i] = params_original[i];
		
		m_params[i] -= cur_epsilon;
		//*(robot->GetFreeParameters()[i]) = params[i];
		UpdateParamsInAllCTR();
		error_perturbed2 = ComputeErrorOnTrainingSet(dummy);
		//error_perturbed2 = ComputeErrorOnDatasetShape(robot, kinematics, dataset, dummy);
		error_jacobian(i) = (error_perturbed - error_perturbed2) / (2*cur_epsilon);
		//jacobian(i) = (error_perturbed - error_original) / cur_epsilon;
		m_params[i] = params_original[i];
		//*(robot->GetFreeParameters()[i]) = m_params[i];
	}

}

double CTRCalibration::UpdateParams(double stepSize, double& max_error)
{
	// compute jacobian
	::Eigen::MatrixXd error_jacobian;
	ComputeErrorJacobian(error_jacobian);
	
	// update parameters
	for(int i = 0; i < m_params.size(); i++)
		error_jacobian(i)/= m_scaleFactor[i] * m_scaleFactor[i];
	error_jacobian /= error_jacobian.norm();	// JHa - normalize error_jacobian
	m_params -= stepSize * error_jacobian;

	// compute error
	double error = ComputeErrorOnTrainingSet(max_error);

	return error;
}

void CTRCalibration::PrintOnConsole(double error, double max_error, int iter, double step, double error_val)
{
	double time_taken = Toc();
	double time_left = (m_maxIter - iter) * time_taken/60.0/(double)iter;
	::std::cout << "iter:" << iter << "  " << "mean_error (train):" <<  error << ", mean_error (val):" <<  error_val << "   step:" << step <<  "   Estimated Time left:" << time_left << ::std::endl; 
}

void CTRCalibration::PrintInFile(double error, double max_error, double error_val)
{
	::std::cout << "intermediate parameter values:" << m_params.transpose() << ::std::endl;
	m_stream << m_params.transpose() << "   " << error << "	" << error_val << " " << max_error << ::std::endl;
}

void CTRCalibration::Tic()
{
	m_startTime = clock();
}

double CTRCalibration::Toc()
{
	clock_t end  = clock();
	double duration = (end - m_startTime)/(double) CLOCKS_PER_SEC;
	return duration;
}

