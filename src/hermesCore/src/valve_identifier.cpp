/*!
 * @file
 *
 *  @authors Ludovic Righetti
 */

#include <SL_user_sensor_proc_xeno.h>
#include <valve_identifier.h>
#include <SL_common.h>

ValveIdentifier::ValveIdentifier()
{
  experiment_running_ = false;
}

ValveIdentifier::~ValveIdentifier()
{
}

bool ValveIdentifier::initializeExperiment()
{
  int test_length;
  std::vector<double> test_signal;
  double magnitude;
  double sampling_time;
  int rc;

  //first check that this experiment is not running
  if ( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
  {
    printf("ValveIdentifier ERROR >> Error cannot acquire gdc mutex, error code %d\n", rc);
    return false;
  }
  if(experiment_running_)
  {
    printf("ValveIdentifier ERROR >> Valve identification experiment already running!!\n");
    rt_mutex_release(&gdc_mutex);
    return false;
  }
  rt_mutex_release(&gdc_mutex);

  dof_ = 0;

  if (!get_int("Which joint?",dof_,&dof_))
    return false;

  if(dof_ < 1 || dof_ > N_DOFS)
  {
    printf("ERROR: %d is not a valid joint number.\n",dof_);
    return false;
  }

  char filename[200];
  while(!get_string("which file to load experiment from?", filename, filename))
  {
  }


  //read the length of the data
  if(!read_parameter_pool_int(filename, "LENGTH", &test_length))
  {
    printf("ValveIdentifier ERROR >> cannot read LENGTH in %s\n", filename);
    return false;
  }

  //allocate signal
  test_signal.resize(test_length+1);

  if(!read_parameter_pool_double_array(filename, "SIGNAL", test_length, &(test_signal[0])))
  {
    printf("ValveIdentifier ERROR >> cannot read SIGNAL in %s\n", filename);
    return false;
  }

  if(!read_parameter_pool_double(filename, "MAGNITUDE", &magnitude))
  {
    printf("ValveIdentifier ERROR >> cannot read MAGNITUDE in %s\n", filename);
    return false;
  }

  if(!read_parameter_pool_double(filename, "SAMPLINGTIME", &sampling_time))
  {
    printf("ValveIdentifier ERROR >> cannot read SAMPLINGTIME in %s\n", filename);
    return false;
  }


  printf("valve identification experiment\n\n");
  printf("sampling time: %f - magnitude %f - length %d\n", sampling_time, magnitude, test_length);
  int check = 1;
  get_int("Start experiment (1) or abort (0).", check, &check);

  if(check)
  {
    //first disable this object in RT thread
    int rc;
    if ( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
    {
      printf("ValveIdentifier ERROR >> Error cannot acquire gdc mutex, error code %d\n", rc);
      return false;
    }
    experiment_running_ = false;
    rt_mutex_release(&gdc_mutex);

    test_length_ = test_length;
    magnitude_ = magnitude;
    sampling_time_ = sampling_time;
    test_signal_ = test_signal;
    index_ = 1;

    //now we can turn on everything again - previous allocations are done without mutex to prevent
    //locking the motor task when allocating memory for the test_signal
    if ( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
    {
      printf("ValveIdentifier ERROR >> Error cannot acquire gdc mutex, error code %d\n", rc);
      return false;
    }
    experiment_running_ = true;
    rt_mutex_release(&gdc_mutex);

  }
  return experiment_running_;

}

bool ValveIdentifier::computeValveCommands(std::vector<hermes_communication_tools::GDCState>& gdc_states)
{
  if(experiment_running_)
  {
    int card_num = -1;
    //find GDC associated with dof
    for(int i=0; i<(int)gdc_states.size(); ++i)
    {
      if(gdc_states[i].dof_number_ == dof_)
      {
        card_num = i;
        break;
      }
    }
    if(card_num >= 0)
    {
      gdc_states[card_num].desired_valve_command_ = int16_t(test_signal_[index_] * magnitude_);
      ++index_;
      if(index_ > test_length_)
      {
        experiment_running_ = false;
      }
    }
  }
  return true;
}
