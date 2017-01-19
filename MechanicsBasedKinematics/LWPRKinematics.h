/**
  *@brief Class for kinematic modeling using LWPR
  *
  * This class is used to implement a kinematic model for the Concentric Tube Robot. The model is based on
  * Locally Weighted Projection Regression (LWPR) as described in Vijaykumar et al (2005)
  * http://homepages.inf.ed.ac.uk/svijayak/publications/vijayakumar-TRUoE2005.pdf
  *
  *@author Georgios Fagogenis
  *@bug no known bugs at the moment
  */

#pragma once

#include <Eigen/Dense>

#include "lwpr.hh"

#define _SCALED_
#define L31_MAX 34.5
# define L31_MIN 3

class LWPRKinematics 
{
	LWPR_Object* forwardModel;
	::std::string modelPath;

public:
	
	/**
	 *@brief - Constructor of LWPR kinematic model for the Contentric Tube Robot
	 */
	LWPRKinematics(const ::std::string& pathToForwardModel = "default_path_here");

	/**
	 *@brief - Destructor of LWPR kinematic model for the Contentric Tube Robot
	 */
	virtual ~LWPRKinematics();

	/**
	  *@brief - compute forward kinematics using LWPR
	  *@param - joint angles
	  *@param - tip position and orientation
	  */
	bool ComputeKinematics(const double* jAng, double* posOrt);	
	bool ComputeKinematics(const ::Eigen::VectorXd& jAng, ::Eigen::VectorXd& posOrt);	

	/**
	  *@brief - compute model Kacobian using LWPR
	  *@param - joint angles
	  *@param - Jacobian
	  */
	bool ComputeJacobian(const ::Eigen::VectorXd& configuration, ::Eigen::MatrixXd& J);

protected:

	// Copy operation is not allowed at this point
    LWPRKinematics(LWPRKinematics* lwprKinematics);
	LWPRKinematics(LWPRKinematics& lwprKinematics);
	
	// Neither copy through assignment
	//LWPRKinematics& operator = (const LWPRKinematics& rhs);
	

	/**
	  *@brief base rotation and translation in forward kinematics computation
	  *@param[in] joint angles
	  *@param[in] relative tip position and orientation
	  *@param[out] tip position and orientation in the robot frame
	  */
	void CompensateForRigidBodyMotion(const double* jAng, const double* posOrt, double* posFinal);

	/**
	  *@brief Check if the joint values (input to the forward kinematics) are in limit and if not cap their values accordingly
	  *@param values to be checked and (if necessary) adjusted to fall within the joint limits
	  */
	template<class T>
	void CheckJointLimits(T& inputData)
	{
		inputData[0] = 1 * atan2(sin(inputData[0]), cos(inputData[0]));
		inputData[1] = 1 * atan2(sin(inputData[1]), cos(inputData[1]));

		if (inputData[2] < 0) {inputData[2] = L31_MIN;}
		if (abs(inputData[2]) > L31_MAX) {inputData[2] = L31_MAX;}
	}


};