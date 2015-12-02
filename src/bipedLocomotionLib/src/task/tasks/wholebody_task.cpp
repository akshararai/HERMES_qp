/*You need to add your headers here before the SL headers*/
//#include "mytest.h"

#include "wholebody_task.h"


// SL system headers
#include "SL_system_headers.h"

// SL includes
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"


/* functions */
extern "C"{

void add_wholebody_task();

static int init_wholebody_task();
static int run_wholebody_task();
static int change_wholebody_task();
}

// function that adds tasks to SL, associated with SL architecture
void add_wholebody_task()
{
    // this is defined in SL/include/SL_tasks.h
    addTask("Whole body task", init_wholebody_task,
          run_wholebody_task, change_wholebody_task);
}


static int init_wholebody_task()
{
//    task_start_time = task_servo_time;
//    for(int i=1; i<=n_dofs; ++i){
//      init_joint_state_uff[i-1] = joint_default_state[i].uff;
//      init_joint_state_th[i-1] = 0.0;
//      init_joint_state_thd[i-1] = 0.0;
//      init_joint_state_thdd[i-1] = 0.0;
//    }

    config_file_="mytestConfig.cf";
//    contact_helper_(config_file_);
//    // read simulation parameters
//    if(!read_parameter_pool_double(config_file_.c_str(),"push_force",&push_force_))
//      assert(false && "reading parameter push_force failed");
//    if(!read_parameter_pool_double(config_file_.c_str(),"push_dur",&push_duration_))
//      assert(false && "reading parameter push_dur failed");

    for(int i=1; i<=n_dofs; ++i)
    {
//        printf("initial joint angle [%d]: %.3f \n", i, joint_default_state[i].th);
    }

    // start save data, in SL/src/SL_collect_data.c
    scd();

    return TRUE;
}

// main function that runs
static int run_wholebody_task()
{
    double roundup = ceil(real_time);
    if ( abs(real_time-roundup)<= 0.5*1.0/double(task_servo_rate) )
    {
        cout << "time: "<<real_time<<endl;
    }
//    cout << "controller time " << real_time << endl;

//    cout << "real_robot_flag : " << real_robot_flag << endl;
    // simulate a push
    if(!real_robot_flag)
    {
        if( real_time >= push_time &&real_time <= push_time + push_duration_)
        {
//            cout << "push : " << push_force_ << " N" << endl;
            uext_sim[L_HAA].f[_Y_] = .5*push_force_;
            uext_sim[R_HAA].f[_Y_] = .5*push_force_;
            sendUextSim();
        }
    }

    /* servo control */
    for (int i=0; i<n_dofs; i++)
    {
//        joint_des_state[i].th   =   init_joint_state[i].th;
//        joint_des_state[i].thd  =   init_joint_state[i].thd;
//        joint_des_state[i].uff =0.0;    // desired_torque;
        joint_des_state[i].th   =  joint_default_state[i].th;
    }

    no_loop++;
    real_time = no_loop*1.0/double(task_servo_rate);

    return TRUE;
}

static int change_wholebody_task()
{
    return TRUE;
}


