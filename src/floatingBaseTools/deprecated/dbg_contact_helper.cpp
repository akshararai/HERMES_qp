/*
 * dbg_contact_helper.cpp
 *
 *  Created on: Dec 06, 2013
 *      Author: herzog
 */

#include <iostream>
#include <sstream>
#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
//#define ros_EXISTS

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>


#include "GeometryUtils.h"

#include "FootContactHandlerHermes.h"

#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"
#include "SL_objects.h"
#include "utility_macros.h"


extern "C"
{
/* global functions */
void add_dbg_contact_helper(void);

/* local functions */
static int init_dbg_contact_helper(void);
static int run_dbg_contact_helper(void);
static int change_dbg_contact_helper(void);
}
void add_dbg_contact_helper(void)
{
  addTask("DBG Contact Helper", init_dbg_contact_helper, run_dbg_contact_helper, change_dbg_contact_helper);
}

int change_dbg_contact_helper()
{
  return TRUE;
}

namespace dbg_contact_helper
{
boost::shared_ptr<floating_base_utilities::KinematicsEigen> kinematics_;
boost::shared_ptr<floating_base_utilities::KinematicsEigen> init_kinematics_;
boost::shared_ptr<momentum_balance_control::FootContactHandlerHermes> contact_helper_;

unsigned int next_marker_id_ = 0;

std::map<std::string, ros::Publisher> publisher_;

}


visualization_msgs::Marker setupMarker()
{
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/BASE";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "ns";
	marker.id = dbg_contact_helper::next_marker_id_++;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::POINTS;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	marker.scale.z = 1.0;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();

	return marker;
}

geometry_msgs::PoseStamped setupPoseStamped()
{
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "/BASE";
  return pose;
}

void setMarkerRed(visualization_msgs::Marker& marker)
{
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
}
void setMarkerGreen(visualization_msgs::Marker& marker)
{
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
}
void setMarkerBlue(visualization_msgs::Marker& marker)
{
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
}

template<typename Vec>
void eigenToGeometryMsgsPoint(const Eigen::MatrixBase<Vec>& eigen_vec, geometry_msgs::Point& point)
{
        point.x = eigen_vec[0];
        point.y = eigen_vec[1];
        point.z = eigen_vec[2];
}

template<typename Mat>
void eigenToGeometryMsgsPoseStamped(const Eigen::MatrixBase<Mat>& eigen_trans, geometry_msgs::PoseStamped& pose)
{
  using namespace floating_base_utilities;
  Eigen::Quaterniond eig_quat;
  GeometryUtils::rotationMatrixToQuaternion(eigen_trans.template topLeftCorner<3,3>(), eig_quat);
  pose.pose.orientation.w = eig_quat.w();
  pose.pose.orientation.x = eig_quat.x();
  pose.pose.orientation.y = eig_quat.y();
  pose.pose.orientation.z = eig_quat.z();
  eigenToGeometryMsgsPoint(eigen_trans.template topRightCorner<3,1>(), pose.pose.position);
}

void generateRandomCoPs(int endeff_id, visualization_msgs::Marker& inside, visualization_msgs::Marker& outside)
{
	using namespace dbg_contact_helper;
	using namespace floating_base_utilities;
	RtMatrixX<4, 6>::d cop_ineq_mat;
	contact_helper_->getCoPInequality(endeff_id, cop_ineq_mat);
	inside.points.clear();
	outside.points.clear();
	for(int i=0; i<100; ++i)
	{
		Eigen::Matrix<double, 6, 1> gen_frc = Eigen::Matrix<double, 6, 1>::Random();
		gen_frc[2] = 5.0;
//		std::cout << "gen_frc: " << gen_frc.transpose() << std::endl;
		Eigen::Vector3d cop;
		contact_helper_->computeEndeffCoP(endeff_id, gen_frc,
				contact_helper_->projected_foot_frames_[endeff_id], cop);
		cop = contact_helper_->projected_foot_frames_[endeff_id].topLeftCorner<3,3>()*cop +
				contact_helper_->projected_foot_frames_[endeff_id].topRightCorner<3,1>();
		geometry_msgs::Point gm_cop;
		eigenToGeometryMsgsPoint(cop, gm_cop);
//		RtVectorX<4>::d sup_inter_slack = cop_ineq_mat *gen_frc;
		if(( (cop_ineq_mat *gen_frc).array() <= 0.0).all() )
		{
			inside.points.push_back(gm_cop);
		}
		else
		{
			outside.points.push_back(gm_cop);
		}
	}
}



int init_dbg_contact_helper()
{
  using namespace dbg_contact_helper;
  using namespace floating_base_utilities;
  using namespace momentum_balance_control;

//  for(int i=0; i<proj_left_orient_.size(); ++i)
//	  addVarToCollect((char *)&proj_left_orient_.data()[i], (std::string("proj_left_")+boost::lexical_cast<std::string>(i)).c_str(),"-",DOUBLE,TRUE);
  std::cout << "Running Contact Helper Debug" << std::endl;
  kinematics_.reset(new KinematicsEigen());
  init_kinematics_.reset(new KinematicsEigen());
  contact_helper_.reset(new FootContactHandlerHermes());

  init_kinematics_->initialize(joint_state, base_state, base_orient, endeff);
  kinematics_->initialize(joint_state, base_state, base_orient, endeff);
  contact_helper_->initialize(kinematics_.get());


  ROS_INFO_STREAM("in rviz please subscribe to:");
  //TODO: list all publisher
  ros::init(std::vector<std::pair<std::string, std::string> >(),
      "dbg_contact_helper");
  ros::NodeHandle n;

  for(int e=1; e<=N_ENDEFFS; ++e)
  {
	std::string pub_name = std::string("scaled_sup_pol_")+boost::lexical_cast<std::string>(cart_names[e]);
	publisher_.insert(std::make_pair<std::string, ros::Publisher> (pub_name,
		n.advertise < visualization_msgs::Marker> (pub_name, 10)));
	pub_name = std::string("random_CoPs_")+boost::lexical_cast<std::string>(cart_names[e]);
	publisher_.insert(std::make_pair<std::string, ros::Publisher> (pub_name,
		n.advertise < visualization_msgs::Marker> (pub_name, 10)));
	pub_name = std::string("projected_sup_pol_")+boost::lexical_cast<std::string>(cart_names[e]);
	publisher_.insert(std::make_pair<std::string, ros::Publisher> (pub_name,
		n.advertise < visualization_msgs::Marker> (pub_name, 10)));
    pub_name = std::string("eff_pub_")+boost::lexical_cast<std::string>(cart_names[e]);
    publisher_.insert(std::make_pair<std::string, ros::Publisher> (pub_name,
        n.advertise < geometry_msgs::PoseStamped> (pub_name, 10) ));
    pub_name = std::string("proj_eff_pub_")+boost::lexical_cast<std::string>(cart_names[e]);
        publisher_.insert(std::make_pair<std::string, ros::Publisher> (pub_name,
            n.advertise < geometry_msgs::PoseStamped> (pub_name, 10) ));
  }
  ros::Rate loop_rate(30);


  visualization_msgs::Marker marker_sup_pol_corners = setupMarker();
  setMarkerGreen(marker_sup_pol_corners);
  marker_sup_pol_corners.type = visualization_msgs::Marker::LINE_STRIP;
  visualization_msgs::Marker marker_proj_sup_pol_corners = setupMarker();
  setMarkerBlue(marker_proj_sup_pol_corners);
  marker_proj_sup_pol_corners.type = visualization_msgs::Marker::LINE_STRIP;
  visualization_msgs::Marker marker_rand_CoPs_inside = setupMarker();
  setMarkerGreen(marker_rand_CoPs_inside);
  visualization_msgs::Marker marker_rand_CoPs_outside = setupMarker();
  setMarkerRed(marker_rand_CoPs_outside);

  int count = 0;
  double run_time=0.0;
  scd();
  while (ros::ok())
  {
    run_time += loop_rate.expectedCycleTime().toSec();
    std::cout << "run time: " << run_time << std::endl;

    //update kinematics
    Eigen::Matrix<double, N_DOFS+6, 1> joint_positions = init_kinematics_->generalizedJointPositions();
    Eigen::Matrix4d base_to_rfoot = init_kinematics_->endeffPose(LEFT_FOOT).inverse()*init_kinematics_->getLinkPose(BASE);
    Eigen::Matrix4d rot_action = Eigen::Matrix4d::Identity();
    rot_action.topLeftCorner<3,3>() = (Eigen::AngleAxisd(1.2*std::sin(run_time), Eigen::Vector3d::UnitX())*
    		Eigen::AngleAxisd(1.2 *std::sin(run_time), Eigen::Vector3d::UnitY())*
    		Eigen::AngleAxisd(1.2 *std::sin(run_time), Eigen::Vector3d::UnitZ())).matrix();
    Eigen::Matrix4d new_base = base_to_rfoot.inverse() * rot_action*base_to_rfoot;
    joint_positions.segment(N_DOFS, 3) = new_base.topRightCorner<3,1>();
    joint_positions.tail<3>() = new_base.topLeftCorner<3,3>().eulerAngles(0, 1, 2);
    kinematics_->update(joint_positions, kinematics_->generalizedJointVelocities(),
        kinematics_->generalizedJointAccelerations(), kinematics_->endeffectors());
    contact_helper_->update();


    for(int e=1; e<= N_ENDEFFS; ++e)
    {
      marker_sup_pol_corners.points.clear();
      marker_proj_sup_pol_corners.points.clear();
      geometry_msgs::Point corner;
      Eigen::Matrix4d std_eff_pose;
      kinematics_->standardizedEndeffPose(e, std_eff_pose);


      for (int i = 0; i <= 4; ++i)
      {

        // support polygon corners
        eigenToGeometryMsgsPoint(
        		std_eff_pose.topLeftCorner<3,3>()*contact_helper_->
        		support_polygon_corners_[e].col(i%4) +
        		std_eff_pose.topRightCorner<3,1>(), corner);
        marker_sup_pol_corners.points.push_back(corner);

        // projected support polygon corners
        eigenToGeometryMsgsPoint(
        		contact_helper_->projected_foot_frames_[e].topLeftCorner<3,3>()*contact_helper_->projected_sup_pol_corners_[e].col(i%4)+
        		contact_helper_->projected_foot_frames_[e].topRightCorner<3,1>(), corner);
        marker_proj_sup_pol_corners.points.push_back(corner);
      }

      // add projected ankle
      eigenToGeometryMsgsPoint(
    		  std_eff_pose.topLeftCorner<3,3>()*contact_helper_->endeff_pos_to_sup_polyg_projection_[e]+
      		std_eff_pose.topRightCorner<3,1>(), corner);
      marker_sup_pol_corners.points.push_back(corner);

      //publish frames
      geometry_msgs::PoseStamped std_endeff_frame = setupPoseStamped();
      eigenToGeometryMsgsPoseStamped(contact_helper_->projected_foot_frames_[e], std_endeff_frame);
      std::string pub_name = std::string("proj_eff_pub_")+boost::lexical_cast<std::string>(cart_names[e]);
      publisher_.find(pub_name)->second.publish(std_endeff_frame);


      std_endeff_frame = setupPoseStamped();
      eigenToGeometryMsgsPoseStamped(std_eff_pose, std_endeff_frame);
      pub_name = std::string("eff_pub_")+boost::lexical_cast<std::string>(cart_names[e]);
      publisher_.find(pub_name)->second.publish(std_endeff_frame);

      // Publish foot geometry marker
      pub_name = std::string("scaled_sup_pol_")+boost::lexical_cast<std::string>(cart_names[e]);
      publisher_.find(pub_name)->second.publish(marker_sup_pol_corners);
      pub_name = std::string("projected_sup_pol_")+boost::lexical_cast<std::string>(cart_names[e]);
      publisher_.find(pub_name)->second.publish(marker_proj_sup_pol_corners);

      // publis randomly generated CoPs
      pub_name = std::string("random_CoPs_")+boost::lexical_cast<std::string>(cart_names[e]);
      generateRandomCoPs(e, marker_rand_CoPs_inside, marker_rand_CoPs_outside);
      publisher_.find(pub_name)->second.publish(marker_rand_CoPs_inside);
      publisher_.find(pub_name)->second.publish(marker_rand_CoPs_outside);
    }

    loop_rate.sleep();
  }


  std::cout << "ERROR: This task needs to be compiled with ROS" << std::endl;


  std::cout << "This task will not enter run(). done..." << std::endl;
  return FALSE;
}


int run_dbg_contact_helper()
{
	std::cout << "This task is not supposed to enter run(). freeze..." << std::endl;
	freeze();
  return 0;
}
