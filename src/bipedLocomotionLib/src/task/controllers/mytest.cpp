/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal

 *********************************************************************
 \remarks      I modify from Alexander Herzog's example code

 \file         mytest.h

 \author       Zhibin Li
 \date         Dec 1, 2015

 *********************************************************************/

#include <mytest.h>

namespace wholebody_demo
{
mytest::mytest() :
    config_file_("mytestConfig.cf"),
    task_start_time(0.0),
    real_time(0.0),
    push_time(5.0), // let push time start later, let robot be steady first
    Num_loop(0),
    stability_margin(0.10),
    cog_kp(500.0),
    cog_kd(50.0),
    hinvdyn_solver_(kinematics_, momentum_helper_, contact_helper_, endeff_kinematics_)
{
    // stop data collection to avoid crashes
    //stopcd();
    freezeBase(false);

    for (int i = 1; i < N_DOFS; ++i)
    {
//        cout << "joint_names " << joint_names[i] << " " << i << endl;
    }

    //set the endeffector constraints for double support
    cout << "N_ENDEFFS :  " << N_ENDEFFS << endl;

    for(int i=1; i<=N_ENDEFFS; ++i)
    {
        endeff_constraints_[i] = endeff[i];
        for(int j=1; j<=6; ++j)
            endeff_constraints_[i].c[j] = 1;
    }


    // initialize our helpers
    kinematics_.initialize(joint_state, base_state, base_orient, endeff_constraints_);
    momentum_helper_.initialize();
    hinvdyn_solver_.initialize();


    // read simulation parameters
    if(!read_parameter_pool_double(config_file_.c_str(),"push_force",&push_force))
        assert(false && "reading parameter push_force failed");
    if(!read_parameter_pool_double(config_file_.c_str(),"push_duration",&push_duration))
        assert(false && "reading parameter push_duration failed");

    if(!read_parameter_pool_double(config_file_.c_str(),"cog_kd",&cog_kp))
        assert(false && "reading parameter cog_kp failed");
    if(!read_parameter_pool_double(config_file_.c_str(),"cog_kd",&cog_kd))
        assert(false && "reading parameter cog_kd failed");

    if(!read_parameter_pool_double(config_file_.c_str(),"reflex_mag_sp",&reflex_mag_sp))
        assert(false && "reading parameter reflex_mag_sp failed");
    if(!read_parameter_pool_double(config_file_.c_str(),"reflex_mag_sr",&reflex_mag_sr))
        assert(false && "reading parameter reflex_mag_sr failed");
    if(!read_parameter_pool_double(config_file_.c_str(),"reflex_mag_e",&reflex_mag_e))
        assert(false && "reading parameter reflex_mag_e failed");
    if(!read_parameter_pool_double(config_file_.c_str(),"reflex_time",&reflex_time))
        assert(false && "reading parameter reflex_time failed");

    if(!read_parameter_pool_double(config_file_.c_str(),"stability_margin",&stability_margin))
        assert(false && "reading parameter stability_margin failed");    

    joint_init_state.resize(N_DOFS);


    for(int i=0; i<=N_DOFS-1; i++)  {
        joint_init_state(i) = joint_des_state[i+1].th;
    }


    Arm.armLeft.set_A_retract(1.0);
    Arm.armLeft.set_T_retract(reflex_time);

    rcom_init = kinematics_.cog();

    // update SL data collection and start collecting data
    updateDataCollectScript();
    scd();


    cout << "Initialization done." << endl;

    cout << "N_DOFS: " << N_DOFS << endl;
    cout << "n_dofs: " << n_dofs << endl;

    cout << "COM: " << kinematics_.cog() << endl;;

    addVarToCollect((char *)&(rcom[0]), "cog_x","m",DOUBLE,TRUE);
    addVarToCollect((char *)&(rcom[1]), "cog_y","m",DOUBLE,TRUE);
    addVarToCollect((char *)&(rcom[2]), "cog_z","m",DOUBLE,TRUE);
    addVarToCollect((char *)&(drcom[0]), "cog_dx","m/s",DOUBLE,TRUE);
    addVarToCollect((char *)&(drcom[1]), "cog_dy","m/s",DOUBLE,TRUE);
    addVarToCollect((char *)&(drcom[2]), "cog_dz","m/s",DOUBLE,TRUE);
    addVarToCollect((char *)&(CapturePoint), "Capture_point","m",DOUBLE,TRUE);
    addVarToCollect((char *)&(output), "output","xx",DOUBLE,TRUE);


    task_start_time = task_servo_time;  // you should put this assignment in the end, coz servo time is running in paralell thread
}

mytest::~mytest()
{
}

int mytest::run()
{
    double sim_time, roundup;

    sim_time = task_servo_time - task_start_time;
//    cout << "rt time " << real_time << "\tsim_time "<< sim_time << "\t"; // << endl;

//    cout << "delta time "<< sim_time-real_time << endl;

    roundup = ceil(real_time);
    if ( abs(real_time-roundup)<= 1.0/double(task_servo_rate) ) // task_servo_rate=1000
    {
        //cout << "time: "<<real_time; // <<endl;
        //cout << "\tfall? " << isFall << "\t";
        //cout << "reflex "<< output << endl;
    }

    // simulate a push
    if(!real_robot_flag)
    {
        if( real_time >= push_time &&real_time <= push_time + 1.0/double(task_servo_rate))
        {
//            cout << "push: " << push_force << "N\tDuration: "<< push_duration <<" s" << endl;
        }

        if( real_time >= push_time &&real_time <= push_time + push_duration)
        {
//            cout << "push : " << push_force << endl;
            uext_sim[L_HAA].f[_Y_] = .5*push_force;
            uext_sim[R_HAA].f[_Y_] = .5*push_force;
            sendUextSim();
        }
    }


    /* feedback */
    // update our helpers
    kinematics_.update(joint_state, base_state, base_orient, endeff_constraints_);

    int dummy_int;  // end effector is not used yet
    endeff_kinematics_.computeJacobians(joint_state, base_state, base_orient,
                                        endeff_constraints_, dummy_int);

    momentum_helper_.update(kinematics_);

    rcom = kinematics_.cog();
    drcom = momentum_helper_.getdCog();
    CapturePoint = rcom(1)+sqrt(1.0/9.81)*drcom(1) - rcom_init(1);  // sqrt(1.0/9.81)=0.319
//    cout << "dx "<< drcom(1) <<"\tTc*dx "<< sqrt(1.0/9.81)*drcom(1) << endl;
//    cout << "COM: " << rcom(1) <<"\t CP: "<< CapturePoint << endl;
//    cout << "CP: "<< CapturePoint << endl;

    /* falling detection */

    if (CapturePoint>stability_margin && real_time >= push_time)
    {
        isFall=true;
    }
    else
    {
        isFall=false;
    }
//    cout << "fall? "<< isFall << endl;

    Arm.reflex(isFall);
    output = Arm.armLeft.m_retraction(0);
//    cout << "fall? " << isFall << "\t";
//    cout << "reflex "<< output << endl;

    // send optimum torques to robot
    // do some transition later when needed
    // here we transition from the previous controller
//    double transition = min(1., task_servo_time - task_start_time_);
    //cout << "transition " << transition << endl;


    /* servo control */
    for(int i=1; i<=N_DOFS; ++i)
    {
        /*if(i>=1 && i<=14)
            continue;
        if(i>=32)
            continue;*/

        //    joint_des_state[i].uff =  hinvdyn_solver_.admis_torques_[i-1];
        //    joint_des_state[i].thdd = (1.-transition)*init_joint_state_thdd_[i-1];
        // SL provides a joint PD controller. The following cancels it out,
        // because we would like to do pure feed-forward control
        //joint_des_state[i].th = joint_default_state[i].th;
        //joint_des_state[B_TFE].th = (1.-transition)*init_joint_state_th_[B_TFE-1] + transition*joint_state[B_TFE].th;
        //joint_des_state[i].thd = joint_default_state[i].thd;
        //cout << "DOF " << joint_names[i] << " hinv t " << hinvdyn_solver_.admis_torques_[i-1] << endl;

    }


    //joint_des_state[L_AFE].uff = -cog_kp*rcom(1)-cog_kd*drcom(1);
    //joint_des_state[R_AFE].uff = -cog_kp*rcom(1)-cog_kd*drcom(1);

    // shoudler pitch
    joint_des_state[L_SFE].th = joint_init_state[L_SFE-1] + reflex_mag_sp*Arm.armLeft.m_retraction(0);
    joint_des_state[R_SFE].th = joint_init_state[R_SFE-1] + reflex_mag_sp* Arm.armLeft.m_retraction(0);

    // shoudler roll
    joint_des_state[L_SAA].th = joint_init_state[L_SAA-1]-reflex_mag_sr*Arm.armLeft.m_retraction(0);
    joint_des_state[R_SAA].th = joint_init_state[R_SAA-1]-reflex_mag_sr*Arm.armLeft.m_retraction(0);

    // elbow
    joint_des_state[L_EB].th = joint_init_state[L_EB-1]+reflex_mag_e*Arm.armLeft.m_retraction(0);
    joint_des_state[R_EB].th = joint_init_state[R_EB-1]+reflex_mag_e*Arm.armLeft.m_retraction(0);

    real_time = Num_loop*1.0/double(task_servo_rate);
    Num_loop++;

    return TRUE;
}

}  // Namespace
 
