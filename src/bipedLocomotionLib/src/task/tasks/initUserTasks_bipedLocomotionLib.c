
void add_biped_locomotion_lib_tasks()
{
  extern void add_test_ik_task();
  extern void add_walking_task();
  extern void add_balance_task();
  extern void add_zero_torque_task();
  extern void add_torque_sine_task();                                                                                                    
  extern void add_gravity_compensation_task();                                                                                           
  extern void add_mass_estimation();                                                                                                     
  extern void add_cog_force_utils();                                                                                                     
  extern void add_fb_sine_task();                                                                                                        
  extern void add_joint_calibration_task();
  extern void add_hinvdyn_example_task();



  add_test_ik_task();
  add_walking_task();
  add_balance_task();
  add_zero_torque_task();
  add_torque_sine_task();                                                                                                    
  add_gravity_compensation_task();                                                                                           
  add_mass_estimation();                                                                                                     
  add_cog_force_utils();                                                                                                     
  add_fb_sine_task();                                                                                                        
  add_joint_calibration_task();
  add_hinvdyn_example_task();
}
