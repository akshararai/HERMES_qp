/*!
 * @file
 * this file contains the implementation of the functions defined in raw_data_access.h
 *
 * @authors Ludovic Righetti
*/


#include "raw_data_access.h"
#include <utility.h>

RawDataAccess::RawDataAccess()
{
}

RawDataAccess::~RawDataAccess()
{
}

bool RawDataAccess::init()
{
    char memory_name[] = "RawDataAccess";

    char sem_name[100];
    sprintf(sem_name, "%s.%s_sem",robot_name, memory_name);
    sem_ = semBSmCreate(sem_name, parent_process_id, SEM_Q_FIFO, SEM_FULL);
    if(sem_ == (SEM_ID)(-1))
    {
        printf("ERROR >> RawDataAccess: Could not create semaphore\n");
        return false;
    }

    char mem_name[100];
    sprintf(mem_name, "%s.%s", robot_name, memory_name);
    data_ = (smRawData *)smMemCalloc(mem_name, parent_process_id, 1, sizeof(smRawData));
    if(data_ == NULL)
    {
        printf("ERROR >> RawDataAccess: Could not create shared memory\n");
        return false;
    }

    return true;
}


bool RawDataAccess::set_to_sm(SL_Jstate *raw_state, double *raw_misc)
{
    if(semTake(sem_, ns2ticks(NO_WAIT)) == ERROR)
    {
        return false;
    }
    for(int i=1; i<=N_DOFS; ++i)
        data_->joint_state[i] = raw_state[i];

    for(int i=1; i<=N_MISC_SENSORS; ++i)
        data_->misc_sensor[i] = raw_misc[i];

    semGive(sem_);
    return true;
}

bool RawDataAccess::get_from_sm(SL_Jstate *raw_state, double *raw_misc)
{
    if(semTake(sem_, ns2ticks(NO_WAIT)) == ERROR)
    {
        return false;
    }
    for(int i=1; i<=N_DOFS; ++i)
        raw_state[i] = data_->joint_state[i];

    for(int i=1; i<=N_MISC_SENSORS; ++i)
        raw_misc[i] = data_->misc_sensor[i];

    semGive(sem_);
    return true;
}

