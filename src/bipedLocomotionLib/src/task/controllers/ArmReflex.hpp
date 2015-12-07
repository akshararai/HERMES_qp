/**
@file ArmReflex.h
@author Zhibin Li <zhibin.li[at]iit.it>
@version 0.0 27/11/15

@section License
Copyright (C) 2015 Department of Advanced Robotics, Italian Institute of Technology, all rights reserved.

@section Description
Header of arm reflex control
*/

#ifndef ArmReflex_H
#define ArmReflex_H

#include <iostream>
#include <Eigen/Dense>
#include<vector>
using namespace Eigen;
using namespace std;

class ArmReflex
{
public:
    ArmReflex();
    ~ArmReflex();

    double dT;				// sampling time
    double A_retract;	  // Amplitude of retraction reflex, vertical modification
    double T_retract;	  // duration of retraction reflex

    double A_ext;	  // Amplitude of extension reflex, vertical modification
    double T_ext;	  // duration of extension reflex

    string hand_contact;

    bool reflexMode; // perhaps I can use bool

    // member variables for retraction
    double a_max_retract; // max acceleration
    double v_max_retract; // max velocity

    // member variables for extension
    double a_max_ext; // max acceleration
    double v_max_ext; // max velocity
    double p_max_ext; // max displacement

    // member variables for detection
    double Fz_th;	// threshold for Fz detection

    Vector3d m_remainPos;
    double m_remainTime;

    Vector3d m_remainPos_left;
    double m_remainTime_left;
    Vector3d m_remainPos_right;
    double m_remainTime_right;

    double ds_Time;  // double support time
    double swingTime;

    double Tcutoff;

    double N;  // clear time for leg extension

    void reflex(bool fall_trigger); // main function

    void updateswingArmTraj(Vector3d remainP_left,double remainTime_left,Vector3d remainP_right,double remainTime_right, Vector3d remainP, double remainTime);

    void retractionReflex(); // leg retraction reflex in case of early contact
    void extensionReflex();	// leg extension reflex in case of late contact


    void setArmContactPhase(string leftarm, string rightarm);

    // below are testing functions
    double reflexRetractionOutput(double input, double a_max_retract);

    // update swing arm trajectory data, runs all the time
    void updateswingArmData(Vector3d remainP,double remainTime);

    // state machine
    void stateMachine(bool fall_trigger);

    double getarmTrajMod();

    // setting flags of controller
    bool isControllerOn;
    void enable();
    void disable();
    bool isEnabled();

    // set reflex mode
    void setReflexMode(bool);

public:
    Vector3d m_angle;   // virtual arm angle, now use the same for left and right arm

private:

    VectorXd m_FTl; // filtered FT
    VectorXd m_FTr;

    // member functions
    bool contactPhase(const VectorXd &FTl,const VectorXd &FTr);

    void reflexActivation(double threshold, double window, double dT);	// trigger function
    bool reflexSelection(); // input ideal and real gait phases, output what type of reflex shall be activated
    void emergencyLevel(double realtime, double idealtime, double gain);	// level of emergency depending of the timing during the phase
    void virtualMomentum(); // virtual force projection/mapping

    // clearance of the modification computed from reflex,
    // the ideal time tells you how fast you should you clear it completely
    void reflexDeactivation(double realtime, double idealtime);

    // you still need some state machine mechanism to coordinate different reflexes



public:
    // definition of nested class, each arm should have such a functional module.
    class virtualModel
    {
    public:
        virtualModel();
        ~virtualModel();

        bool whicharm; // 1 for left, 0 for right;
        bool isActivation;
        int max_count_activation;
        int counterActivation;

        bool isAerial;
        int max_count_aerial;
        int counterAerial;
        vector<string> arm_phase;

        double m0, m1, m2, n0, n1, n2, K, D;
        Matrix2d A, At;
        Vector2d B, Bt;
        //Vector3d output; // output state of unit response, should change to parameter input

        Vector3d m_retraction, m_extension; // note: default value is zero in eigen already, this is the state vector

        double A_retract;	  // Amplitude of retraction reflex, vertical modification
        double T_retract;	  // duration of retraction reflex

        double A_ext;	  // Amplitude of extension reflex, vertical modification
        double T_ext;	  // duration of extension reflex

        // structure of bool type flags instead of single bool variables
        struct _flags
        {
            bool enable;    // default init is false anyway
            bool done;
        };
        _flags flg_retract, flg_ext;

        // member variables for retraction
        double a_max_retract; // max acceleration
        double v_max_retract; // max velocity

        // member variables for extension
        double a_max_ext; // max acceleration
        double v_max_ext; // max velocity
        double p_max_ext; // max displacement

        void Initialization(double dT, bool);
        void reflexConfig(double dT, double Tn, double zeta);
        void reflexRetractionActivation(double dT, double Tn, double zeta);
        double unitResponseConstrained(double input, double a_max_retract, Vector3d &output);
        double reflexRetractionOutput(double input, double a_max_retract);
        double reflexRetractionClear(double input, double a_max_retract);
        double getRetractionOutput(); // decide later what variables you want to get
        bool swingArmActivation(bool fall_trigger);

        // extension
        void updateHandContact(string arm_phase);
        bool stanceArmAerial(VectorXd & FT, double Fz_th);
        double getExtensionOutput(); // decide later what variables you want to get
        void set_A_retract(double mag);
        void set_T_retract(double time);

    };
    virtualModel armLeft, armRight;
};



#endif // ArmReflex_H
