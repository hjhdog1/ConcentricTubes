#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>
#include "CTRFactory.h"
#include "MechanicsBasedKinematics.h"
#include "LWPRKinematics.h"
#include "UKF.h"
#include "SyntheticDataGenerator.h"
#include "Utilities.h"
#include <Eigen/Dense>
#include <ctime>
#include <tinyxml.h>
#include "ShapeMeasurement.h"
//#include <opencv2/core.hpp>
//#include <opencv2/imgproc.hpp>
//#include "opencv2/imgcodecs.hpp"
//#include <opencv2/highgui.hpp>
//#include <opencv2/ml.hpp>
#define pi M_PI
// timer
#include <ctime>

//using namespace cv;
//using namespace cv::ml;

typedef ::std::vector<::std::vector<double>> DoubleVec;

void testSimulator();
void testUKF();
void testFreeParameters();
void testCTR();
void testKinematics();
//void testSCurve();
void testKinematicsSingleConfiguration();
void testSVMClassifier();
void fitMechanicsBasedKinematics();
void fitMechanicsBasedKinematicsShape();
double ComputeErrorOnDataset(CTR* robot, MechanicsBasedKinematics* kinematics, DoubleVec& data_in, DoubleVec& data_out);
double ComputeErrorOnDatasetShape(CTR* robot, MechanicsBasedKinematics* kinematics, const ShapeDataset& dataset);
void ComputeErrorJacobian(::Eigen::VectorXd& params, CTR* robot, MechanicsBasedKinematics* kinematics, DoubleVec& data_in, DoubleVec& data_out,  double error_original,::Eigen::MatrixXd& jacobian);
void ComputeErrorJacobianShape(::Eigen::VectorXd& params, CTR* robot, MechanicsBasedKinematics* kinematics, const ShapeDataset& dataset, double error_original, ::Eigen::MatrixXd& jacobian);
void preprocessData(CTR* robot,::std::vector<::std::string>& dataStr, DoubleVec& data_in, DoubleVec& data_out);
void evaluateModel();
double ComputeSingleShapeError(CTR* robot, MechanicsBasedKinematics* kinematics, const Measurement& meas);

// controller tests
void testOptimizationController();
void testOptimizationControllerOnData();
void runOptimizationController(LWPRKinematics* kinematics, double initialConfiguration[], double goalInTaskSapce[6], double outputConfiguration[]);
void checkJointLimits(::Eigen::VectorXd& configuration);
bool computeObjectiveFunction(LWPRKinematics* kinematics, const ::Eigen::VectorXd& targetX, ::Eigen::VectorXd& x, double t, double& funVal, double& realError);
void computeObjectiveFunctionJacobian(LWPRKinematics* kinematics, const ::Eigen::VectorXd& targetX, ::Eigen::VectorXd& x, double t, ::Eigen::MatrixXd& J);
void solveFirstObjective(LWPRKinematics* kinematics, const ::Eigen::VectorXd& targetX, ::Eigen::VectorXd& x, double t, double eps, double mu);
void unscaleVector(::Eigen::VectorXd& x, double scalingFactors[]);
void unscaleVector(double x[], int x_size, double scalingFactors[]);
void scaleVector(::Eigen::VectorXd& x, double scalingFactors[]);
void scaleVector(double x[], int x_size, double scalingFactors[]);

// tinyXML test
void testTinyXML();
void testShapeDataset();

int main()
{
	//testOptimizationController();
	//testOptimizationControllerOnData();
	//fitMechanicsBasedKinematics();
	//testTinyXML();
	//testShapeDataset();
	fitMechanicsBasedKinematicsShape();

	return 0;
}

void fitMechanicsBasedKinematicsShape()
{
	::std::ofstream os("C:/Users/RC/Dropbox/parameters.txt");
	
	// load training data
	ShapeDataset dataset;
	BuildShapeDatasetFromString("shape_training2.xml", dataset);
	::std::cout << "Number of loaded measurements:" << dataset.size() << ::std::endl;	

	// build the robot
	CTR* robot = CTRFactory::buildCTR("");
	::std::vector<double*> parameters = robot->GetFreeParameters();
	int numParameters = parameters.size();

	::Eigen::VectorXd params(numParameters);
	for(int i = 0; i < numParameters; ++i)
		params(i) = *parameters[i];

	// initialize mechanics-based kinematics
	MechanicsBasedKinematics* kinematics = new MechanicsBasedKinematics(robot,100);
	kinematics->ActivateIVPJacobian();

	double error_prev = 0.0;

	double tolerance = 0.00001;
	int max_iterations = 1000;

	int iter = 0;
	::Eigen::MatrixXd error_jacobian(parameters.size(),1);
	
	double step = 0.00001;
	
	double mean_error = ComputeErrorOnDatasetShape(robot, kinematics, dataset);

	::std::cout << mean_error << ::std::endl;

	double scale_factor = 100;

	// loop until convergence
	clock_t start = clock();
	while ( ::std::abs(mean_error - error_prev) > tolerance && iter < max_iterations)
	{
		
		error_prev = mean_error;

		// compute error jacobian with respect to parameters		
		ComputeErrorJacobianShape(params, robot, kinematics, dataset, mean_error, error_jacobian);

		// update parameters
		for(int i = 0; i < 5; i+=2)
			error_jacobian(i)/= scale_factor * scale_factor;
		params -= step * error_jacobian;
		
		// update robot based on updated parameters
		for(int i = 0 ; i < parameters.size() ; i++)
			*parameters[i] = params(i);

		// update error
		mean_error = ComputeErrorOnDatasetShape(robot, kinematics, dataset);


		if (mean_error > error_prev)
			step *= 0.90;

		clock_t end  = clock();
		double duration = (end - start)/(double) CLOCKS_PER_SEC/(double) ++iter;

		if (iter % 10 == 0)
			::std::cout << "iter:" << iter << "  " << "mean_error:" <<  mean_error << "   step:" << step <<  "   Estimated Time left:" << (max_iterations - iter) * duration/60.0 << ::std::endl; 

		if (iter % 50 == 0)
		{
			::std::cout << "intermediate parameter values:" << params.transpose() << ::std::endl;
			os << params.transpose() << "   " << mean_error << ::std::endl;
		}
		
	}
	os.close();
	::std::cout << "Calibrated model parameters:" << params.transpose() << ::std::endl; 
}

double ComputeErrorOnDatasetShape(CTR* robot, MechanicsBasedKinematics* kinematics, const ShapeDataset& dataset)
{
	double error = 0.0;
	for (int i = 0; i < dataset.size(); ++i)
		error += ComputeSingleShapeError(robot, kinematics, dataset[i]);

	return error/dataset.size();
}


double ComputeSingleShapeError(CTR* robot, MechanicsBasedKinematics* kinematics, const Measurement& meas)
{
	double rotation[3] = {0};
	double translation[3] = {0};
	double* relativeConf = new double[5];

	memcpy(relativeConf, meas.GetConfiguration().data(), sizeof(double) * 5);
	relativeConf[0] *= M_PI/180.0;
	relativeConf[1] *= M_PI/180.0;
	relativeConf[3] *= M_PI/180.0;

	::std::vector<::Eigen::Vector3d> positionsAlongRobotExp = meas.GetShapeEig();
	::std::vector<::Eigen::Vector3d> positionsAlongRobotModel;
	::Eigen::Vector3d error;

	MechanicsBasedKinematics::RelativeToAbsolute(robot, relativeConf, rotation, translation);

	::std::vector<double> robot_length_parameter = meas.GetArcLength();

	kinematics->ComputeKinematics(rotation, translation);
	kinematics->GetRobotShape(robot_length_parameter, positionsAlongRobotModel);

	double sum = 0;

	for(int i = 0; i < positionsAlongRobotModel.size(); ++i)
	{
		error = positionsAlongRobotExp[i] - positionsAlongRobotModel[i];
		sum += error.norm();
	}

	return sum/positionsAlongRobotModel.size();
}

void testShapeDataset()
{
	ShapeDataset dataset;
	BuildShapeDatasetFromString("measurements.xml", dataset);
	::std::cout << "Number of loaded measurements:" << dataset.size() << ::std::endl;

	for(int i = 0; i < dataset.size(); ++i)
	{
		::std::cout << "Measurement:" << i << ::std::endl;
		::std::cout << dataset[i] << ::std::endl;
	}
}

void composeXML()
{
	// create and store an XML file
	TiXmlDocument doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	TiXmlElement* measurement = new TiXmlElement( "Measurement" );

	TiXmlElement* configuration = new TiXmlElement( "Configuration" );
	configuration->SetAttribute("q", "180 180 32 0 12");
	measurement->LinkEndChild(configuration);

	TiXmlElement* shape = new TiXmlElement( "Shape" );
	TiXmlElement* point = new TiXmlElement( "Point" );
	shape->LinkEndChild(point);
	point->SetAttribute("coords", "0 0 0");
	point->SetAttribute("s", "0");

	point = new TiXmlElement( "Point" );
	shape->LinkEndChild(point);
	point->SetAttribute("coords", "0 0 0");
	point->SetAttribute("s", "0");	measurement->LinkEndChild(shape);

	doc.LinkEndChild(measurement);
	doc.SaveFile( "measurements.xml" );
}

void parseXML()
{
	TiXmlDocument doc( "measurements.xml" );
	if (!doc.LoadFile()) return;

	TiXmlHandle hDoc(&doc);
	TiXmlElement* pElem, *pElemChild;
	TiXmlHandle hRoot(0);

	// parse all measurements
	TiXmlElement* pElemRoot ;
	pElemRoot = hDoc.FirstChildElement().Element();
	hRoot = TiXmlHandle(pElem);

	pElem = hRoot.FirstChildElement("Measurement").Element();
	char buffer[100];
	double s, x, y, z;
	int measurmentCounter = 0; 
	int pointCounter;

	::std::string configuration;
	::std::vector<double> configurationDVector;

	for (pElem; pElem; pElem=pElem->NextSiblingElement())
	{
		::std::cout << "Measurement :" << measurmentCounter++ << ::std::endl;

		pElemChild = pElem->FirstChildElement("Configuration");
		configuration = pElemChild->GetText();
		configurationDVector = DoubleVectorFromString(configuration);
		::std::cout << "Configuration:";
		PrintCArray(configurationDVector.data(), configurationDVector.size());

		pElemChild = pElem->FirstChildElement("Shape");
		pElemChild = pElemChild->FirstChildElement("Point");

		pointCounter = 0;
		for (pElemChild; pElemChild; pElemChild=pElemChild->NextSiblingElement())
		{
			pElemChild->QueryDoubleAttribute("s", &s);
			pElemChild->QueryDoubleAttribute("x", &x);
			pElemChild->QueryDoubleAttribute("y", &y);
			pElemChild->QueryDoubleAttribute("z", &z);
			::std::cout << "Point " << pointCounter++ << ": x:" << x << ", y:" << y << ", z:" << z << ", s:" << s << ::std::endl;
		}
		::std::cout << ::std::endl;
	}
	

	}

void testTinyXML()
{
	//composeXML();
	parseXML();
}


double scalingFactors[5] = {M_PI/180.0, M_PI/180.0, 35, M_PI/180.0, 100};

void testOptimizationControllerOnData()
{
	// Initialize robot and kinematics solver
	//::std::string pathToModel("models/lwpr_forward_2016_4_28_14_32_16_D80.bin");	
	::std::string pathToModel("models/lwpr_forward_2016_11_17_16_14_55.bin");	
	LWPRKinematics* kinematics = new LWPRKinematics(pathToModel);

	//read data
	::std::vector<::std::string> dataStr = ReadLinesFromFile("./testControl.txt");
	//::std::vector<::std::string> dataStr = ReadLinesFromFile("./debug_control2.txt");
	::std::vector<double> dataVec;
	double configuration[5] = {0};
	double taskGoal[6] = {0};
	double outputConfiguration[5] = {0};

	// Initial robot configuration
	double initialConfiguration[5] = {pi, pi, 15, 0, 0};
	double posOrt[6] = {0};

	double maxPositionError = 0;
	double currentPositionError = 0;

	::std::ofstream os("results.txt");
	int numSample = 0;
	for (::std::vector<::std::string>::iterator it = dataStr.begin(); it != dataStr.end(); ++it)
	{
		dataVec.clear();
		dataVec = DoubleVectorFromString(*it);
		memcpy(configuration, &dataVec.data()[6], 5 * sizeof(double));	
		memcpy(taskGoal, dataVec.data(), 6 * sizeof(double));	

		clock_t start = clock();	
		runOptimizationController(kinematics, configuration, taskGoal, outputConfiguration);
		clock_t end = clock();

		// validate
		::std::cout << "time required to solve kinematics: " << (double) (end - start)/CLOCKS_PER_SEC << " [sec]" << ", test sample: " << numSample << ::std::endl;

		kinematics->ComputeKinematics(outputConfiguration, posOrt);

		//::std::cout << "Goal Position:";
		//PrintCArray(taskGoal, 3);
		//::std::cout << "Actual Position:";
		//PrintCArray(posOrt, 3);
		//::std::cout << ::std::endl;
		//::std::cout << "Goal Tangent:";
		//PrintCArray(&taskGoal[3], 3);
		//::std::cout << "Actual Tangent:";
		//PrintCArray(&posOrt[3], 3);
		//::std::cout << ::std::endl;
		//::std::cout << ::std::endl;

		::Eigen::VectorXd actTang = ::Eigen::Map< ::Eigen::VectorXd> (&posOrt[3],3);
		::Eigen::VectorXd goalTang = ::Eigen::Map< ::Eigen::VectorXd> (&taskGoal[3],3);

		::Eigen::VectorXd actPos = ::Eigen::Map< ::Eigen::VectorXd> (posOrt,3);
		::Eigen::VectorXd goalPos = ::Eigen::Map< ::Eigen::VectorXd> (taskGoal,3);
		currentPositionError = (actPos - goalPos).norm();

		//if (currentPositionError > 0.5)
		//	os  << ::Eigen::Map<::Eigen::VectorXd> (configuration, 5).transpose() << " " << goalPos.transpose() << " " << ::Eigen::Map<::Eigen::VectorXd> (&posOrt[3], 3).transpose() << ::std::endl;

		maxPositionError = (currentPositionError > maxPositionError ? currentPositionError : maxPositionError);
		::std::cout << "Position error: " << (actPos - goalPos).norm() << " [mm]" << ::std::endl;
		::std::cout << "Orientation Error:" << RAD2DEG(::std::acos(goalTang.dot(actTang)/(goalTang.norm() * actTang.norm()))) << " [deg]" << ::std::endl; 
	
		//::Eigen::VectorXd tmpConf = ::Eigen::Map< ::Eigen::VectorXd> (outputConfiguration, 5);
		//unscaleVector(tmpConf, scalingFactors);
		//::std::cout << "Computed configuration: ";
		//PrintCArray(tmpConf.data(), 5);
		//::std::cout << endl;

		// update initial configuration
		memcpy(configuration, outputConfiguration, 5 * sizeof(double));
		unscaleVector(configuration, 5, scalingFactors);
		numSample++;
	}
	::std::cout << "max error across the whole dataset" << maxPositionError << ::std::endl;
	//os << "max error across the whole dataset" << maxPositionError << ::std::endl;
	os.close();
	
}


void testOptimizationController()
{
	// Initialize robot and kinematics solver
	//::std::string pathToModel("models/lwpr_forward_2016_4_28_14_32_16_D80.bin");	
	::std::string pathToModel("models/lwpr_forward_2016_11_17_16_14_55.bin");	
	LWPRKinematics* kinematics = new LWPRKinematics(pathToModel);

	// Initial robot configuration
	double initialConfiguration[5] = {pi/4, -0.5*pi, 15, -1.3 * pi, 50};

	// Compute a feasible goal in the task space
	double goalInTaskSpace[6] = {0};
	double goalConfiguration[5] = {pi/3, -0.2*pi, 25, -0.3 * pi, 20};

	scaleVector(goalConfiguration, 5, scalingFactors);
	kinematics->ComputeKinematics(goalConfiguration, goalInTaskSpace);

	//goalInTaskSpace[3] = 0;
	//goalInTaskSpace[4] = 0;
	//goalInTaskSpace[5] = 1;

	double outputConfiguration[5] = {0};
	clock_t start = clock();	
	runOptimizationController(kinematics, initialConfiguration, goalInTaskSpace, outputConfiguration);
	clock_t end = clock();

	::std::cout << "time required to solve kinematics: " << (double) (end - start)/CLOCKS_PER_SEC << " [sec]" << ::std::endl;

	double posOrt[6] = {0};
	kinematics->ComputeKinematics(outputConfiguration, posOrt);


	::std::cout << "Goal Position:";
	PrintCArray(goalInTaskSpace, 3);
	::std::cout << "Actual Position:";
	PrintCArray(posOrt, 3);
	::std::cout << "Goal Tangent:";
	PrintCArray(&goalInTaskSpace[3], 3);
	::std::cout << "Actual Tangent:";
	PrintCArray(&posOrt[3], 3);
	::std::cout << ::std::endl;

	::Eigen::VectorXd actTang = ::Eigen::Map< ::Eigen::VectorXd> (&posOrt[3],3);
	::Eigen::VectorXd goalTang = ::Eigen::Map< ::Eigen::VectorXd> (&goalInTaskSpace[3],3);

	::Eigen::VectorXd actPos = ::Eigen::Map< ::Eigen::VectorXd> (posOrt,3);
	::Eigen::VectorXd goalPos = ::Eigen::Map< ::Eigen::VectorXd> (goalInTaskSpace,3);
	::std::cout << "Position error: " << (actPos - goalPos).norm() << " [mm]" << ::std::endl;
	::std::cout << "Orientation Error:" << RAD2DEG(::std::acos(goalTang.dot(actTang)/(goalTang.norm() * actTang.norm()))) << " [deg]" << ::std::endl; 
	
	::Eigen::VectorXd tmpConf = ::Eigen::Map< ::Eigen::VectorXd> (outputConfiguration, 5);
	unscaleVector(tmpConf, scalingFactors);
	::std::cout << "Computed configuration: ";
	PrintCArray(tmpConf.data(), 5);
	::std::cout << endl;
}

void runOptimizationController(LWPRKinematics* kinematics, double initialConfiguration[], double goalInTaskSapce[6], double outputConfiguration[])
{
	::Eigen::VectorXd targetX = ::Eigen::Map<::Eigen::VectorXd> (goalInTaskSapce, 6);
	::Eigen::VectorXd configuration = ::Eigen::Map<::Eigen::VectorXd> (initialConfiguration, 5);
	::Eigen::VectorXd currentX;
	::Eigen::MatrixXd J, Jp;

	int iterations = 0;
	int maxIterations = 100;
	double step = 1.0;

	::Eigen::VectorXd error(6), errorPrev(6);

	//double scalingFactors[5] = {M_PI, M_PI, 35, M_PI, 100};

	scaleVector(configuration, scalingFactors);

	kinematics->ComputeKinematics(configuration, currentX);	

	error = targetX - currentX;

	::Eigen::VectorXd confPrev;

	while (error.segment(0, 3).norm() > 1.0 && iterations < maxIterations)
	{
		kinematics->ComputeJacobian(configuration, J);
		Jp = J.block(0,0,3,5);

		if (step < 1.e-10)
			break;

		confPrev = configuration;
		configuration += step * Jp.transpose() * (Jp * Jp.transpose()).inverse() * error.segment(0, 3);

		unscaleVector(configuration, scalingFactors);
		checkJointLimits(configuration);
		scaleVector(configuration, scalingFactors);

		kinematics->ComputeKinematics(configuration, currentX);
		
		errorPrev = error;
		error = targetX - currentX;

		while (error.norm() > errorPrev.norm() && step > 0.001)
		{
			step *= 0.8;
			configuration = confPrev;
			error = errorPrev;
			configuration += step * Jp.transpose() * (Jp * Jp.transpose()).inverse() * error.segment(0, 3);

			unscaleVector(configuration, scalingFactors);
			checkJointLimits(configuration);
			scaleVector(configuration, scalingFactors);

			kinematics->ComputeKinematics(configuration, currentX);

			errorPrev = error;
			error = targetX - currentX;
		}
		//::std::cout << "iterations: " << iterations << ", position error [mm]: " << error.segment(0, 3).norm() << " , step size: " << step << ::std::endl;
		step = 1.0;
		iterations++;
	}

	// check if solution is in the feasible set: if not return --- TODO
	if (error.segment(0, 3).norm() > 0.5)
	{
		memcpy(outputConfiguration, configuration.data(), configuration.size() * sizeof(double));
		return;
	}

	double t = 10;
	double mu = 12.0;
	double eps = 0.0001;
	double Jcost = 0.0;


	for (int k = 0; k < maxIterations; ++k)
	{

		solveFirstObjective(kinematics, targetX, configuration, t, eps, mu);
		
		//::std::cout << "k-iterations: " << k << ", position error: " << error.segment(0, 3).norm() <<", fVal: " << error.segment(3, 3).norm() << ::std::endl;
		if (1.0/t < eps) break;

		t *= mu;
	}
	memcpy(outputConfiguration, configuration.data(), configuration.size() * sizeof(double));
}

void unscaleVector(::Eigen::VectorXd& x, double scalingFactors[])
{
	for (int i = 0; i < x.size(); ++i)
		x(i) *= scalingFactors[i];
}

void scaleVector(::Eigen::VectorXd& x, double scalingFactors[])
{
	for (int i = 0; i < x.size(); ++i)
		x(i) /= scalingFactors[i];

}

void scaleVector(double x[], int x_size, double scalingFactors[])
{
	for (int i = 0; i < x_size; ++i)
		x[i] /= scalingFactors[i];
}

void unscaleVector(double x[], int x_size, double scalingFactors[])
{
	for (int i = 0; i < x_size; ++i)
		x[i] *= scalingFactors[i];
}

void solveFirstObjective(LWPRKinematics* kinematics, const ::Eigen::VectorXd& targetX, ::Eigen::VectorXd& x, double t, double eps, double mu)
{
	double Jcost = 0.0;
	double JcostPrev = 1000.0;
	int iterations = 0;
	int maxIterations = 50;
	double step = 0.80;
	//double scalingFactors[5] = {M_PI, M_PI, 35, M_PI, 100};
	::Eigen::VectorXd xPrev(x);
	::Eigen::MatrixXd J;
	double realCost = 0;
	computeObjectiveFunction(kinematics, targetX, x, t, Jcost, realCost);
	while ( ::std::abs(Jcost - JcostPrev) > 1.e-03  && iterations < maxIterations)
	{
		computeObjectiveFunctionJacobian(kinematics, targetX, x, t, J);
		xPrev = x;
		JcostPrev = Jcost;
		x -= step/t * J.transpose() * (J * J.transpose()).inverse() * Jcost;

		unscaleVector(x, scalingFactors);
		checkJointLimits(x);
		scaleVector(x, scalingFactors);

		bool respectConstraints = computeObjectiveFunction(kinematics, targetX, x, t, Jcost, realCost);
		step = 0.8;
		int iterationsInner = 0;
		while ( (Jcost > JcostPrev && step > 0.0000001 && iterationsInner < maxIterations) || !respectConstraints)
		{
				step *= 0.8;
				Jcost = JcostPrev;
				x = xPrev;
				x -= step * J.transpose() * (J * J.transpose()).inverse() * Jcost;
				unscaleVector(x, scalingFactors);
				checkJointLimits(x);
				scaleVector(x, scalingFactors);

				respectConstraints = computeObjectiveFunction(kinematics, targetX, x, t, Jcost, realCost);
				iterationsInner++;
		}
	//	::std::cout << "iterations: " << iterations << ", fVal: " << Jcost << ::std::endl;
		iterations++;
	}
}

void computeObjectiveFunctionJacobian(LWPRKinematics* kinematics, const ::Eigen::VectorXd& targetX, ::Eigen::VectorXd& x, double t, ::Eigen::MatrixXd& J)
{
	double epsilon = 0.00001;
	double invEpsilon = 1.0/epsilon;
	
	double f0, fNew, JJ;
	computeObjectiveFunction(kinematics, targetX, x, t, f0, JJ);
	::Eigen::VectorXd perturbedX = x;

	J.resize(1, x.size());
	for(int i = 0; i < x.size(); ++i)
	{
		perturbedX(i) += epsilon;
		computeObjectiveFunction(kinematics, targetX, perturbedX, t, fNew, JJ);
		J(0, i) = (fNew - f0) * invEpsilon;
		perturbedX(i) = x(i);
	}
}

bool computeObjectiveFunction(LWPRKinematics* kinematics, const ::Eigen::VectorXd& targetX, ::Eigen::VectorXd& x, double t, double& funVal, double& realError)
{
	::Eigen::VectorXd outX;
	kinematics->ComputeKinematics(x, outX);

	double constraintSlack = max(0.001, 1.0 - (outX.segment(0, 3) - targetX.segment(0, 3)).norm());
	
	double phi = -::std::log(constraintSlack);
	realError = (outX.segment(3, 3) - targetX.segment(3, 3)).norm();
	funVal = t * realError + phi;

	if (1.0 - (outX.segment(0, 3) - targetX.segment(0, 3)).norm() < 0) return false;
	
	return true;
}

void checkJointLimits(::Eigen::VectorXd& configuration)
{
	configuration(0) = ::std::fmod(configuration(0),  M_PI);
	configuration(1) = ::std::fmod(configuration(1),  M_PI);
	configuration(3) = ::std::fmod(configuration(3),  M_PI);

	if (configuration(2) < 0) configuration(2) = 0;
	if (configuration(2) > 34.5) configuration(2) = 34.5;

	if (configuration(4) < -100) configuration(2) = -100;
	if (configuration(4) >  100) configuration(2) = 100;
}

// ---------------------------------------------------------------------------------------------------------------------//
void evaluateModel()
{
	// load training data
	::std::vector< ::std::string> dataStr = ReadLinesFromFile("./2016-10-04-09-05-36_record_joint.txt");
	::std::vector< ::std::vector< double>> data_in, data_out;

	// load model with initial parameters
	CTR* robot = CTRFactory::buildCTR("");
	::std::vector<double*> parameters = robot->GetFreeParameters();

	double calibratedParameters[5] = {0.00358469, 0.998491, 0.00355983, 0.274077, 0.0180651};
	for(int i = 0; i < 5; ++i)
		*parameters[i] = calibratedParameters[i];


	preprocessData(robot, dataStr, data_in, data_out);

	MechanicsBasedKinematics* kinematics = new MechanicsBasedKinematics(robot,100);
	kinematics->ActivateIVPJacobian();
	
	double mean_error = ComputeErrorOnDataset(robot, kinematics, data_in, data_out);
	::std::cout << mean_error << ::std::endl;

}

void fitMechanicsBasedKinematics()
{
	::std::ofstream os("C:/Users/RC/Dropbox/parameters.txt");
	// load training data
	//::std::vector< ::std::string> dataStr = ReadLinesFromFile("./2016-10-06-09-46-43_record_joint.txt");
	::std::vector< ::std::string> dataStr = ReadLinesFromFile("./test_fitting.txt");
	::std::vector< ::std::vector< double>> data_in, data_out;

	// load model with initial parameters
	CTR* robot = CTRFactory::buildCTR("");
	::std::vector<double*> parameters = robot->GetFreeParameters();
	::Eigen::VectorXd params(5);
	for(int i = 0; i < 5; ++i)
		params(i) = *parameters[i];

	//::std::cout << params << ::std::endl;

	preprocessData(robot, dataStr, data_in, data_out);

	//::std::cout << data_in.size() << " " << data_out.size() << ::std::endl;
	//PrintCArray(data_in[0].data(), 6);

	MechanicsBasedKinematics* kinematics = new MechanicsBasedKinematics(robot,100);
	kinematics->ActivateIVPJacobian();

	//double mean_error = 1000000.0;
	double error_prev = 0.0;

	double tolerance = 0.00001;
	int max_iterations = 1000;

	int iter = 0;
	::Eigen::MatrixXd error_jacobian(5,1);
	
	double step = 0.00001;
	
	double mean_error = ComputeErrorOnDataset(robot, kinematics, data_in, data_out);
	::std::cout << mean_error << ::std::endl;

	double scale_factor = 100;
	// loop until convergence
	clock_t start = clock();
	while ( ::std::abs(mean_error - error_prev) > tolerance && iter < max_iterations)
	{
		
		error_prev = mean_error;

		// compute error jacobian with respect to parameters		
		ComputeErrorJacobian(params, robot, kinematics, data_in, data_out, mean_error, error_jacobian);

		// update parameters
		for(int i = 0; i < 5; i+=2)
			error_jacobian(i)/= scale_factor * scale_factor;
		params -= step * error_jacobian;
		
		// update robot based on updated parameters
		for(int i = 0 ; i < parameters.size() ; i++)
			*parameters[i] = params(i);

		// update error
		mean_error = ComputeErrorOnDataset(robot, kinematics, data_in, data_out);


		if (mean_error > error_prev)
			step *= 0.90;

		clock_t end  = clock();
		double duration = (end - start)/(double) CLOCKS_PER_SEC/(double) ++iter;

		if (iter % 10 == 0)
			::std::cout << "iter:" << iter << "  " << "mean_error:" <<  mean_error << "   step:" << step <<  "   Estimated Time left:" << (max_iterations - iter) * duration/60.0 << ::std::endl; 

		if (iter % 50 == 0)
		{
			::std::cout << "intermediate parameter values:" << params.transpose() << ::std::endl;
			os << params.transpose() << "   " << mean_error << ::std::endl;
		}
		
	}
	os.close();
	::std::cout << "Calibrated model parameters:" << params.transpose() << ::std::endl; 
}

void preprocessData(CTR* robot,::std::vector<::std::string>& dataStr, DoubleVec& data_in, DoubleVec& data_out)
{
	double relative_configuration[5] = {0};
	double rotation[3] = {0};
	double translation[3] = {0};

	::std::vector<::std::string>::iterator it = dataStr.begin();
	::std::vector<double> tmp;
	::std::vector<double> tmpAbsConf(6);
	::std::vector<double> tmpPosition(3);
	for(it; it < dataStr.end(); ++it)
	{
		tmp = DoubleVectorFromString(*it);
		memcpy(relative_configuration, &tmp.data()[6], sizeof(double) * 5);

		MechanicsBasedKinematics::RelativeToAbsolute(robot, relative_configuration, rotation, translation);

		memcpy(tmpAbsConf.data(), rotation, sizeof(double) * 3);
		memcpy(&tmpAbsConf.data()[3], translation, sizeof(double) * 3);
		memcpy(tmpPosition.data(), &tmp.data()[0], sizeof(double) * 3);

		data_in.push_back(tmpAbsConf);
		data_out.push_back(tmpPosition);
	}
}



double ComputeErrorOnDataset(CTR* robot, MechanicsBasedKinematics* kinematics, DoubleVec& data_in, DoubleVec& data_out)
{
	double rotation[3] = {0};
	double translation[3] = {0};

	::Eigen::Vector3d position;
	::Eigen::Vector3d actual_position;

	double error = 0;
	::Eigen::Vector3d instant_error;

	for (int i = 0; i < data_in.size(); ++i)
	{
		memcpy(rotation, data_in[i].data(), sizeof(double) * 3);
		memcpy(translation, &data_in[i].data()[3], sizeof(double) * 3);
		
		kinematics->ComputeKinematics(rotation, translation);
		kinematics->GetTipPosition(position);

		actual_position = ::Eigen::Map<::Eigen::Vector3d> (data_out[i].data(), 3);	

		instant_error = actual_position - position;

		error += instant_error.norm();
	}

	error /= data_in.size();

	return error;
}

void ComputeErrorJacobian(::Eigen::VectorXd& params, CTR* robot, MechanicsBasedKinematics* kinematics, DoubleVec& data_in, DoubleVec& data_out, double error_original, ::Eigen::MatrixXd& jacobian)
{
	::Eigen::VectorXd params_original(params);

	double epsilon = 0.000001;
	double invEpsilon = 1.0/epsilon;
	double error_perturbed = 0;
	
	for(int i = 0; i < params.size(); ++i)
	{
		params[i] += epsilon;
		*(robot->GetFreeParameters()[i]) = params[i];
		error_perturbed = ComputeErrorOnDataset(robot, kinematics, data_in, data_out);
		jacobian(i) = (error_perturbed - error_original) * invEpsilon;
		params[i] = params_original[i];
		*(robot->GetFreeParameters()[i]) = params[i];
	}

}

void ComputeErrorJacobianShape(::Eigen::VectorXd& params, CTR* robot, MechanicsBasedKinematics* kinematics, const ShapeDataset& dataset, double error_original, ::Eigen::MatrixXd& jacobian)
{
	::Eigen::VectorXd params_original(params);

	double epsilon = 0.000001;
	double invEpsilon = 1.0/epsilon;
	double error_perturbed = 0;
	
	for(int i = 0; i < params.size(); ++i)
	{
		params[i] += epsilon;
		*(robot->GetFreeParameters()[i]) = params[i];
		error_perturbed = ComputeErrorOnDatasetShape(robot, kinematics, dataset);
		jacobian(i) = (error_perturbed - error_original) * invEpsilon;
		params[i] = params_original[i];
		*(robot->GetFreeParameters()[i]) = params[i];
	}

}

//void testSVMClassifier()
//{
//
//    // Data for visual representation
//    int width = 512, height = 512;
//    Mat image = Mat::zeros(height, width, CV_8UC3);
//    // Set up training data
//    int labels[4] = {1, -1, -1, -1};
//    float trainingData[4][2] = { {501, 10}, {255, 10}, {501, 255}, {10, 501} };
//    Mat trainingDataMat(4, 2, CV_32FC1, trainingData);
//    Mat labelsMat(4, 1, CV_32SC1, labels);
//    // Train the SVM
//    Ptr<SVM> svm = SVM::create();
//    svm->setType(SVM::C_SVC);
//    svm->setKernel(SVM::LINEAR);
//    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));
//    svm->train(trainingDataMat, ROW_SAMPLE, labelsMat);
//    // Show the decision regions given by the SVM
//    Vec3b green(0,255,0), blue (255,0,0);
//    for (int i = 0; i < image.rows; ++i)
//        for (int j = 0; j < image.cols; ++j)
//        {
//            Mat sampleMat = (Mat_<float>(1,2) << j,i);
//            float response = svm->predict(sampleMat);
//            if (response == 1)
//                image.at<Vec3b>(i,j)  = green;
//            else if (response == -1)
//                image.at<Vec3b>(i,j)  = blue;
//        }
//    // Show the training data
//    int thickness = -1;
//    int lineType = 8;
//    circle( image, Point(501,  10), 5, Scalar(  0,   0,   0), thickness, lineType );
//    circle( image, Point(255,  10), 5, Scalar(255, 255, 255), thickness, lineType );
//    circle( image, Point(501, 255), 5, Scalar(255, 255, 255), thickness, lineType );
//    circle( image, Point( 10, 501), 5, Scalar(255, 255, 255), thickness, lineType );
//    // Show support vectors
//    thickness = 2;
//    lineType  = 8;
//    Mat sv = svm->getUncompressedSupportVectors();
//    for (int i = 0; i < sv.rows; ++i)
//    {
//        const float* v = sv.ptr<float>(i);
//        circle( image,  Point( (int) v[0], (int) v[1]),   6,  Scalar(128, 128, 128), thickness, lineType);
//    }
//    imwrite("result.png", image);        // save the image
//    imshow("SVM Simple Example", image); // show it to the user
//    waitKey(0);
//
//}


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
	simulator.ReadJointAndTipTrajectory("../jointTipTrajectory_measured.txt");
	//simulator.ReadJointAndTipTrajectory("../jointTipTrajectory_measured_short.txt");
	//simulator.ReadJointAndTipTrajectory("../jointTipTrajectory_measured_permuted.txt");
	//simulator.ReadJointAndTipTrajectory("../jointTipTrajectory_theoretical.txt", false);

	std::vector<double> nominalValues;
	std::vector<double*> freeParameters = robot->GetFreeParameters();
	for(int i = 0; i < freeParameters.size(); ++i)
	{
		nominalValues.push_back(*freeParameters[i]);
		*freeParameters[i] *= 1.10;
	}

	double measVar[6] = {1, 1, 1, 0.1, 0.1, 0.1};
	std:vector<double> measVarSTL(measVar, measVar+6);
	UKF ukf(robot, robot->GetFreeParameterVariances(), measVarSTL);
	ukf.Initialize();

	MechanicsBasedKinematics kin(robot);
	
	::std::ofstream paramOs("../parameterHistory.txt"), errorOs("../errorHistory.txt");

	double pos[3], ori[9], rotation[3], translation[3];
	while(simulator.LoadOneMeasurement(pos, ori, rotation, translation))
	{
		robot->UpdateConfiguration(rotation, translation);
		if(!ukf.StepFilter(ori, pos))
			::std::cout << "StepFilter Faild!!" << ::std::endl;
		
		if(simulator.GetCounter() % 100 == 0)
			::std::cout << ::std::endl << (double)(simulator.GetCounter())/(double)(simulator.GetTrajectoryLength()) * 100 << "%" << ::std::endl << ::std::endl;


		for(int i = 0; i < freeParameters.size(); ++i)
		{
			std::cout << std::fixed << std::setprecision(1) << (*freeParameters[i] - nominalValues[i])/nominalValues[i] * 100 << "%\t";
			paramOs << *freeParameters[i] << "\t";
		}
		paramOs << std::endl;

		kin.ComputeKinematics(rotation, translation);
		SE3 tipFrame;
		kin.GetBishopFrame(tipFrame);

		Vec3 measuredPos(pos[0], pos[1], pos[2]);
		Vec3 posError = measuredPos - tipFrame.GetPosition();

		//::std::cout << "Tip position error: " << posError[0] << ", " << posError[1] << ", " << posError[2] << ::std::endl;
		errorOs << posError[0] << "\t" << posError[1] << "\t" << posError[2] << ::std::endl;
		::std::cout << "Tip position error: " << posError.Normalize() << ::std::endl;
		
		//std::cout << std::setprecision(5) << "\t" << *freeParameters[0] << "\t" << 1 / *freeParameters[1] << "\t" << *freeParameters[2] << "\t"  << 1 / *freeParameters[3];
		//std::cout << std::setprecision(5) << 1 / *freeParameters[0] << "\t" << *freeParameters[1] << "\t" << 1 / *freeParameters[2] << "\t" << *freeParameters[3] << "\t"  << 1 / *freeParameters[4];
		//std::cout << std::setprecision(5) << 1 / *freeParameters[0] << "\t" << 1 / *freeParameters[1] << "\t" << 1 / *freeParameters[2];

		
	}

	//_sleep(10000);

	paramOs.close();
	errorOs.close();

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
//
//void testSCurve()
//{
//	CTR* const robot = CTRFactory::buildCTR("");
//
//	MechanicsBasedKinematics kinematics(robot,100);
//	kinematics.ActivateIVPJacobian();
//
//	double rotation[3] = {0,0,0};
//	double translation[3] = {0,-17,-34-86.3938};
//
//	int scurveRes = 51;
//	Eigen::MatrixXd scurve(2,scurveRes);
//	for(int i = 0; i < scurveRes; ++i)
//	{
//		rotation[2] = 2*M_PI*(double)i/(double)(scurveRes-1);
//
//		if(!kinematics.ComputeKinematics(rotation, translation))
//		{
//			std::cout << "rot. = " << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << ", \t \t";
//			std::cout << "trans. = " << translation[0] << ", " << translation[1] << ", " << translation[2] << std::endl;
//
//			std::cout << "FAILED!!" << std::endl;
//		}
//
//		scurve(0, i) = rotation[2];
//		scurve(1, i) = kinematics.GetInnerTubeRotation();
//
//	}
//
//	std::ofstream stm("scurve.txt");;
//	for(int i = 0; i < scurve.rows(); ++i)
//		for(int j = 0; j < scurve.cols(); ++j)
//		{
//			stm << scurve(i,j);
//			if (j == scurve.cols()-1)
//				stm << std::endl;
//			else
//				stm << "\t";
//
//		}
//
//
//	_sleep(10000);
//}

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

