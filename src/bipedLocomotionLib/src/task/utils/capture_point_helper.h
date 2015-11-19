/*
 * capture_point_helper.h
 *
 *  Created on: Apr 27, 2013
 *      Author: righetti
 */

#ifndef CAPTURE_POINT_HELPER_H_
#define CAPTURE_POINT_HELPER_H_

#include <Eigen/Eigen>

#include <SL.h>

class CapturePointHelper {
public:
	CapturePointHelper();
	virtual ~CapturePointHelper();

	bool initialize(double z_com, double z_floor);

	//things are in world frame
	bool getInstantaneousCapturePoint(const Eigen::Vector3d& com, const Eigen::Vector3d& dcom,
	                                  Eigen::Vector3d& icp);

	//things are in world frame
	bool getPredictedCapturePoint(double deltaT, const Eigen::Vector3d& icp,
	                              const Eigen::Vector3d& cop, Eigen::Vector3d& pred_cp);

	double getOmega0(){return omega0_;}

private:
	bool initialized_;

	double omega0_, z_com_, z_floor_;
};

#endif /* CAPTURE_POINT_HELPER_H_ */
