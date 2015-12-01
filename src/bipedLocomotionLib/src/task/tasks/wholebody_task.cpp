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
//#include <Eigen/Eigen>
//#include "wholebody_task.h"


/* local variables */
static double real_time = 0.0;
static double transition_time = 2.0;

// I will resuse these parameter inputs later
static double freq = 0.1;
static double amp = 1.0;
static double offset = 0.0;
static int jid = 0;
static int step_tests = 0;

/* newly added */
double task_start_time;
//Eigen::Matrix<double, N_DOFS,1> init_joint_state_uff, init_joint_state_th, init_joint_state_thd, init_joint_state_thdd;


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
    int j, i;
    int ans;
//    static int firsttime = TRUE;

//    if (firsttime){
//    firsttime = FALSE;
//    //    freq = 0.1; // frequency
//    //    amp  = 0.5; // amplitude
//    }

    // defined in SL/include/SL_tasks.h
    // get_int(const char *comment, int defaultvalue, int *value)
//    get_int("Do step tests?", step_tests, &step_tests);
//    if(step_tests)
//    {
//        get_int("Torque step which joint?",jid,&jid);
//        get_double("Step amplitude [Nm]", amp,&amp);
//        get_double("Step offset [Nm]", offset, &offset);
//        get_double("Step frequency [Hz]",freq,&freq);


//        if (jid < 1 || jid > n_dofs || amp > 10.0 || amp < -10.0 || freq < 0.1 || freq > 1.0)
//            return FALSE;
//        printf("running a %f Nm step test with offset %f for joint %s\n", amp, offset, freq, joint_names[jid]);
//    }
//    else
//    {
//        get_int("Torque sine which joint?",jid,&jid);
//        get_double("Sine amplitude [Nm]",amp,&amp);
//        get_double("Sine frequency [Hz]",freq,&freq);
//        get_double("Sine offset [Nm]", offset, &offset);

//        real_time = 0.0;

//        if (jid < 1 || jid > n_dofs || amp > 10.0 || amp < -10.0 || freq < 0.1 || freq > 40.0)
//            return FALSE;

//        printf("running a %f Nm sine offset at %f at %f Hz for joint %s\n", amp, offset, freq, joint_names[jid]);
//    }

    // ready to go
    ans = 1;
//    while (ans == 999)
//    {
//        if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
//            return FALSE;
//    }

    // only go when user really types the right thing
    if (ans != 1)
        return FALSE;


//    task_start_time = task_servo_time;
    for(int i=1; i<=n_dofs; ++i){
//      init_joint_state_uff[i-1] = joint_default_state[i].uff;
//      init_joint_state_th[i-1] = 0.0;
//      init_joint_state_thd[i-1] = 0.0;
//      init_joint_state_thdd[i-1] = 0.0;
    }

    for(int i=1; i<=n_dofs; ++i)
    {
        printf("initial joint angle [%d]: %.3f \n", i, joint_default_state[i].th);
    }

    // start save data, in SL/src/SL_collect_data.c
    scd();

    return TRUE;
}

// main function that runs
static int run_wholebody_task()
{
//    static double desired_torque = offset;
//    if(step_tests)
//    {
//        static double running_time = 0.0;
//        static double sign = 1.0;
//    if(running_time >1/freq)
//    {
//        desired_torque  = offset + sign*amp;
//        sign *=-1.0;
//        running_time = 0.0;
//    }
//    running_time += 1.0/double(task_servo_rate);
//    }
//    else
//    {
//        //transition for 2 seconds
//        double mult = 0.0;
//        if(real_time < transition_time)
//            mult = real_time/transition_time;
//        else
//            mult = 1.0;
//        desired_torque = mult*(amp * sin(real_time * freq * 2 * M_PI) + offset);
//    }


    for (int i=0; i<n_dofs; i++)
    {
//        joint_des_state[i].th   =   init_joint_state[i].th;
//        joint_des_state[i].thd  =   init_joint_state[i].thd;
//        joint_des_state[i].uff =0.0;    // desired_torque;
    }


    real_time += 1.0/double(task_servo_rate);

    return TRUE;
}

static int change_wholebody_task()
{
    return TRUE;
}


