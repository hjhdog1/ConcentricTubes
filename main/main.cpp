#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>
#include "CTRFactory.h"
#include "MechanicsBasedKinematics.h"
#include "UKF.h"
#include "SyntheticDataGenerator.h"
#include "Utilities.h"
#include <Eigen/Dense>
#include <ctime>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>

// timer
#include <ctime>

using namespace cv;
using namespace cv::ml;

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
double ComputeErrorOnDataset(CTR* robot, MechanicsBasedKinematics* kinematics, DoubleVec& data_in, DoubleVec& data_out);
void ComputeErrorJacobian(::Eigen::VectorXd& params, CTR* robot, MechanicsBasedKinematics* kinematics, DoubleVec& data_in, DoubleVec& data_out,  double error_original,::Eigen::MatrixXd& jacobian);
void preprocessData(CTR* robot,::std::vector<::std::string>& dataStr, DoubleVec& data_in, DoubleVec& data_out);
void evaluateModel();

int main()
{
	//testKinematicsSingleConfiguration();
	//testKinematics();
	//testFreeParameters();

	//testUKF();

	//testSimulator();
	
	//testSCurve();
	//testSVMClassifier();
	//fitMechanicsBasedKinematics();
	evaluateModel();

	std::cout << "Finished. Press enter to close." << std::endl;
	std::cin.ignore();
	return 0;
}


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
	::std::ofstream os("parameters.txt");
	// load training data
	::std::vector< ::std::string> dataStr = ReadLinesFromFile("./2016-10-03-16-12-39_record_joint.txt");
	//::std::vector< ::std::string> dataStr = ReadLinesFromFile("./test_fitting.txt");
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


void testSVMClassifier()
{

    // Data for visual representation
    int width = 512, height = 512;
    Mat image = Mat::zeros(height, width, CV_8UC3);
    // Set up training data
    int labels[4] = {1, -1, -1, -1};
    float trainingData[4][2] = { {501, 10}, {255, 10}, {501, 255}, {10, 501} };
    Mat trainingDataMat(4, 2, CV_32FC1, trainingData);
    Mat labelsMat(4, 1, CV_32SC1, labels);
    // Train the SVM
    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::LINEAR);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));
    svm->train(trainingDataMat, ROW_SAMPLE, labelsMat);
    // Show the decision regions given by the SVM
    Vec3b green(0,255,0), blue (255,0,0);
    for (int i = 0; i < image.rows; ++i)
        for (int j = 0; j < image.cols; ++j)
        {
            Mat sampleMat = (Mat_<float>(1,2) << j,i);
            float response = svm->predict(sampleMat);
            if (response == 1)
                image.at<Vec3b>(i,j)  = green;
            else if (response == -1)
                image.at<Vec3b>(i,j)  = blue;
        }
    // Show the training data
    int thickness = -1;
    int lineType = 8;
    circle( image, Point(501,  10), 5, Scalar(  0,   0,   0), thickness, lineType );
    circle( image, Point(255,  10), 5, Scalar(255, 255, 255), thickness, lineType );
    circle( image, Point(501, 255), 5, Scalar(255, 255, 255), thickness, lineType );
    circle( image, Point( 10, 501), 5, Scalar(255, 255, 255), thickness, lineType );
    // Show support vectors
    thickness = 2;
    lineType  = 8;
    Mat sv = svm->getUncompressedSupportVectors();
    for (int i = 0; i < sv.rows; ++i)
    {
        const float* v = sv.ptr<float>(i);
        circle( image,  Point( (int) v[0], (int) v[1]),   6,  Scalar(128, 128, 128), thickness, lineType);
    }
    imwrite("result.png", image);        // save the image
    imshow("SVM Simple Example", image); // show it to the user
    waitKey(0);

}


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
