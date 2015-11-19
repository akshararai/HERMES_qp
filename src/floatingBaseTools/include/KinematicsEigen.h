/*
 * KinematicsEigen.h
 *
 *  Created on: Apr 1, 2013
 *      Author: herzog
 */

#ifndef KINEMATICSEIGEN_H_
#define KINEMATICSEIGEN_H_

#include <vector>
#include <Eigen/Eigen>

#include <SL.h>
#include <utility.h>
#include <utility_macros.h>
#include <SL_kinematics.h>
#include <SL_dynamics.h>
#include <SL_task_servo.h>
#include <SL_user.h>

namespace floating_base_utilities
{

/*!
* @class KinematicsEigen
* @author Alexander Herzog
* @brief Eigen Wrapper around SL kinematics
* Maintains some kinematics structure in Eigen datatypes. functions starting
* with 'compute' are state free, where functions starting with 'update' maintain
* computed kinematics structure inside of this class. just calling update() will
* compute things in the right order.
*/
class KinematicsEigen {
public:
  KinematicsEigen();
  KinematicsEigen(const KinematicsEigen& other);
	virtual ~KinematicsEigen();
	/** this is deprecated! it will lead to bugs! */ void initialize();
	void initialize(const Eigen::Matrix<double, N_DOFS + 6, 1>& joint_positions,
            const Eigen::Matrix<double, N_DOFS + 6, 1>& joint_velocities,
            const Eigen::Matrix<double, N_DOFS + 6, 1>& joint_accelerations,
            SL_endeff* eff);
        void initialize(SL_Jstate* state, SL_Cstate& basec, SL_quat& baseo, SL_endeff* eff);

    /*!
    * @name update internal state.
    * This should be done once per control cycle
    * @{
    */
    //! update using eigen constructs
		void update(const Eigen::Matrix<double, N_DOFS + 6, 1>& joint_positions,
		                const Eigen::Matrix<double, N_DOFS + 6, 1>& joint_velocities,
		                const Eigen::Matrix<double, N_DOFS + 6, 1>& joint_accelerations,
		                const SL_endeff* eff);
		//! update using SL constructs
		void update(SL_Jstate* state, SL_Cstate& basec, SL_quat& baseo, const SL_endeff* eff);

		/*
		 * @}
		 */

		/** compute the Link Poses from joint and end effector positions
		 *
		 */
		void computeCartesianLinkInformation(SL_Jstate* state, SL_Cstate& basec, SL_quat& baseo, SL_endeff* eff,
				Eigen::Vector3d& center_of_gravity, Eigen::Matrix<double, 3, (N_DOFS+1)>& joint_axes,
				Eigen::Matrix<double, 3, (N_DOFS+1)>& joint_cart_poitions,
				Eigen::Matrix<double, 4*(N_LINKS+1), 4>& link_poses,
				Eigen::Matrix<double, 3, (N_LINKS+1)>& link_poitions,
				Eigen::Matrix<double, 3, (N_DOFS+1)>& link_com_poitions,
				Eigen::Matrix<double, 4*(N_DOFS+1), 4>& joint_poses);
		void computeCartesianLinkInformation(const Eigen::Matrix<double, N_DOFS + 6, 1>& joint_positions,
				SL_endeff* eff, Eigen::Vector3d& center_of_gravity, Eigen::Matrix<double, 3, (N_DOFS+1)>& joint_axes,
				Eigen::Matrix<double, 3, (N_DOFS+1)>& joint_cart_poitions,
				Eigen::Matrix<double, 4*(N_LINKS+1), 4>& link_poses,
				Eigen::Matrix<double, 3, (N_LINKS+1)>& link_poitions,
				Eigen::Matrix<double, 3, (N_DOFS+1)>& link_com_poitions,
				Eigen::Matrix<double, 4*(N_DOFS+1), 4>& joint_poses);

		/** compute full body jacobian from joint positions
		 *
		 */
		void computeFullBodyJointsJacobian(const Eigen::Matrix<double, 3, (N_DOFS+1)>& joint_axes,
				const Eigen::Matrix<double, 3, (N_DOFS+1)>& joint_cart_poitions,
				Eigen::Matrix<double, 2*N_CART*N_DOFS+2*N_CART, N_DOFS+2*N_CART>& jac) const;

		/** some accessors
		 *
		 */
		const Eigen::Matrix<double, 2*N_CART, N_DOFS+2*N_CART>& linkJacobian(int link_id);
		const Eigen::Matrix<double, 2*N_CART, N_DOFS+2*N_CART>& linkJacobianDerivative(int link_id);
		const Eigen::Matrix<double, N_DOFS+6, 1>& generalizedJointPositions() const{return joint_positions_;};
	        const Eigen::Matrix<double, N_DOFS+6, 1>& generalizedJointVelocities() const{return joint_velocities_;};
	        const Eigen::Matrix<double, N_DOFS+6, 1>& generalizedJointAccelerations() const{return joint_accelerations_;};
		const Eigen::Vector3d cog() const{return cog_;};
		const Eigen::Matrix<double, 4*(N_LINKS+1), 4>& linkPoses() const
				{return link_poses_;};
		const Eigen::Matrix<double, 4*(N_DOFS+1), 4>& jointPoses() const{return joint_poses_;};
		const Eigen::Matrix<double, 3, (N_DOFS+1)>& cartesianJointPositions() const
				{return joint_cart_poitions_;};
		const Eigen::Matrix<double, 3, (N_DOFS+1)>& jointAxes() const
				{return joint_axes_;};
		const Eigen::Matrix<double, 2*N_CART*N_DOFS+2*N_CART, N_DOFS+2*N_CART>& fullBodyJointJacobian() const
				{return full_body_joint_jacobian_;};
		void getLinkPosition(unsigned int link_id, Eigen::Vector3d& pos) const {pos = link_positions_.block(0, link_id,3,1);};
                Eigen::Block<const Eigen::Matrix<double, 3, (N_LINKS+1)> > linkPosition(unsigned int link_id)  const {return link_positions_.block(0, link_id,3,1);};

		void getLinkPose(unsigned int link_id, Eigen::Matrix<double, 4, 4>& pose) const  // x_b = pose * x_l; x_l position in link frame, x_b position in base frame
			{pose = link_poses_.block(4*link_id, 0,4,4);};
		Eigen::Block<const Eigen::Matrix<double, 4*(N_LINKS+1), 4> > getLinkPose(unsigned int link_id)  const {return link_poses_.block(4*link_id, 0,4,4);};
                Eigen::Block<const Eigen::Matrix<double, 4*(N_LINKS+1), 4> > endeffPose(unsigned int endeff_id)  const {return getLinkPose(link2endeffmap[endeff_id]);};
                Eigen::Block<const Eigen::Matrix<double, 3, (N_LINKS+1)> > endeffPosition(unsigned int endeff_id)  const {return linkPosition(link2endeffmap[endeff_id]);};

		void getJointPose(unsigned int joint_id, Eigen::Matrix<double, 4, 4>& pose) const
			{pose = joint_poses_.block(4*joint_id, 0,4,4);};
    Eigen::Vector3d cartesianJointPosition(unsigned int joint_id)  const {return joint_cart_poitions_.col(joint_id);};
		const Eigen::Matrix<double, 3, (N_DOFS+1)>& linkComPositions() const {return link_com_poitions_;};
		double robotMass() const {return robot_mass_;};

		/** some helper functions
		 *
		 */
		void eigenGeneralizedJointPositionsToSlPose(const Eigen::Matrix<double, 6, 1>& joint_positions,
				SL_quat& rot, SL_Cstate& pos) const;
		void slPoseToEigenGeneraliedJointPositions(SL_quat& rot, SL_Cstate& pos,
				Eigen::Matrix<double, 6, 1>& joint_positions) const;
		void eigenGeneralizedJointVelocitiesToSlPose(const Eigen::Matrix<double, 6, 1>& joint_velocities,
				SL_quat& rot, SL_Cstate& lin) const;
		void getSlState(SL_Jstate* state, SL_Cstate& basec, SL_quat& baseo) const;
//		void computeJointPosesBIPED(SL_Jstate *state, SL_Cstate *basec, SL_quat *baseo, SL_endeff *eff,
//				Eigen::Matrix<double, 4*(N_DOFS+1), 4>& joint_poses) const;
                const Eigen::Matrix4d& standardizedToRealEndeffTransform(int id) const{return standardized_endeff_frames_[id];};

    void standardizedEndeffPose(int id, Eigen::Matrix4d& pose) const;
		Eigen::Matrix4d standardizedEndeffPose(int id) const;
		const SL_endeff* endeffectors() const{return endeffectors_;};
		const SL_endeff endeffector(int endeff) const{return endeffectors_[endeff];};

//                template <typename Derived>
//                void getEndeffPosition(int endeff, Eigen::MatrixBase<Derived>& position) const{for(int i=1;i<=3;++i) position[i]=endeffector(endeff).x[i];};
//                template <typename Derived>
//                void getEndeffOrientation(int endeff, Eigen::MatrixBase<Derived>& orientation) const{for(int i=1;i<=3;++i) orientation[i]=endeffector(endeff).a[i];};
//                template <typename Derived>
//                void getEndeffPose(int endeff, Eigen::MatrixBase<Derived>& pose) const{getEndeffPosition(endeff, pose.block<3,1>(0,0)); getEndeffOrientation(endeff,pose.block<3,1>(3,0));};

	private:
                Eigen::Matrix<bool, N_LINKS+1, 1> is_link_jacobian_computed_;
                Eigen::Matrix<bool, N_LINKS+1, 1> is_prev_link_jacobian_computed_;
		Eigen::Matrix<double, N_DOFS+6, 1> joint_positions_;
		Eigen::Matrix<double, N_DOFS+6, 1> joint_velocities_;
		Eigen::Matrix<double, N_DOFS+6, 1> joint_accelerations_;

		Eigen::Matrix<double, 4*(N_LINKS+1), 4> link_poses_;
		Eigen::Matrix<double, 4*(N_DOFS+1), 4> joint_poses_;
		Eigen::Matrix<double, 3, (N_LINKS+1)> link_positions_;
		Eigen::Matrix<double, 3, (N_DOFS+1)> joint_cart_poitions_;
		Eigen::Matrix<double, 3, (N_DOFS+1)> link_com_poitions_;
		Eigen::Matrix<double, 3, (N_DOFS+1)> joint_axes_;

                Matrix xmcog;
                Matrix xaxis;
                Matrix xorigin;
                Matrix xlink;
                Matrix ahmat[N_LINKS + 1];
                Matrix ahmat_dof[N_DOFS + 1];
                SL_endeff endeffectors_[N_ENDEFFS+1];

                Eigen::Matrix<double, 2*N_CART*N_DOFS+2*N_CART, N_DOFS+2*N_CART> full_body_joint_jacobian_;
                Eigen::Matrix<double, 2*N_CART, N_DOFS+2*N_CART> link_jacobians_[N_LINKS+1];
                Eigen::Matrix<double, 2*N_CART, N_DOFS+2*N_CART> link_jacobian_derivs_[N_LINKS+1];
                Eigen::Matrix<double, 4, 4> standardized_endeff_frames_[N_ENDEFFS+1];

		Eigen::Vector3d cog_;
		double robot_mass_;

		void computeFullBodyLinksJacobian();
		inline void computeLinkJacobian(int link_id);

	};

} /* namespace floating_base_utilities */
#endif /* KINEMATICSEIGEN_H_ */
