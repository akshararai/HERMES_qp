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

using namespace floating_base_utilities;

namespace wholebody_demo
{
mytest::mytest() :
    config_file_("mytestConfig.cf"),
    task_start_time(0.0),
    real_time(0.0),
    push_time(2.0),
    Num_loop(0)
{
    // stop data collection to avoid crashes
    stopcd();
    freezeBase(false);

    for (int i = 1; i < N_DOFS; ++i)
    {
        cout << "joint_names " << joint_names[i] << " " << i << endl;
    }


    // read simulation parameters
    if(!read_parameter_pool_double(config_file_.c_str(),"push_force",&push_force))
        assert(false && "reading parameter push_force failed");
    if(!read_parameter_pool_double(config_file_.c_str(),"push_duration",&push_duration))
        assert(false && "reading parameter push_duration failed");

    // update SL data collection and start collecting data
    updateDataCollectScript();
    scd();

    cout << "Initialization done." << endl;

    cout << "N_DOFS: " << N_DOFS << endl;
    cout << "n_dofs: " << n_dofs << endl;


//  task_start_time = task_servo_time;
}

mytest::~mytest()
{
}

int mytest::run()
{
//    real_time = task_servo_time - task_start_time;
    double roundup = ceil(real_time);
    if ( abs(real_time-roundup)<= 1.0/double(task_servo_rate) ) // task_servo_rate=1000
    {
        cout << "time: "<<real_time<<endl;
    }

    // simulate a push
    if(!real_robot_flag)
    {
        if( real_time >= push_time &&real_time <= push_time + 1.0/double(task_servo_rate))
        {
            cout << "push: " << push_force << "N\tDuration: "<< push_duration <<" s" << endl;
        }

        if( real_time >= push_time &&real_time <= push_time + push_duration)
        {
//            cout << "push : " << push_force << endl;
            uext_sim[L_HAA].f[_Y_] = .5*push_force;
            uext_sim[R_HAA].f[_Y_] = .5*push_force;
            sendUextSim();
        }
    }

    // send optimum torques to robot
    // do some transition later when needed
    // here we transition from the previous controller
//    double transition = min(1., task_servo_time - task_start_time_);
    //cout << "transition " << transition << endl;


    /* servo control */
    for(int i=1; i<=N_DOFS; ++i)
    {
        if(i>=1 && i<=14)
            continue;
        if(i>=32)
            continue;

        //    joint_des_state[i].uff =  hinvdyn_solver_.admis_torques_[i-1];
        //    joint_des_state[i].thdd = (1.-transition)*init_joint_state_thdd_[i-1];
        // SL provides a joint PD controller. The following cancels it out,
        // because we would like to do pure feed-forward control
        joint_des_state[i].th = joint_default_state[i].th;
        //joint_des_state[B_TFE].th = (1.-transition)*init_joint_state_th_[B_TFE-1] + transition*joint_state[B_TFE].th;
        joint_des_state[i].thd = joint_default_state[i].thd;
        //cout << "DOF " << joint_names[i] << " hinv t " << hinvdyn_solver_.admis_torques_[i-1] << endl;

    }

    real_time = Num_loop*1.0/double(task_servo_rate);
    Num_loop++;

    return TRUE;
}

}  // Namespace
 
