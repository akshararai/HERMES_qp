/*!
 * @file
 * this file contain definitions that allow to share raw data between
 * the motor servo and the task servo
 *
 * @authors Ludovic Righetti
*/

#ifndef RAW_DATA_ACCESS_H
#define RAW_DATA_ACCESS_H

#include <SL.h>
#include <SL_user.h>
#include <SL_system_headers.h>


class RawDataAccess
{
public:
    RawDataAccess();
    virtual ~RawDataAccess();

    bool init();
    bool set_to_sm(SL_Jstate *raw_state, double *raw_misc);
    bool get_from_sm(SL_Jstate *raw_state, double *raw_misc);

private:

    struct smRawData
    {
        SL_Jstate joint_state[N_DOFS+1];
        double misc_sensor[N_MISC_SENSORS+1];
    };

    smRawData *data_;
    SEM_ID sem_;
};

#endif // RAW_DATA_ACCESS_H

