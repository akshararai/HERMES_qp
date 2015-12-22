/**
    @file ArmReflex.cpp
	@author Zhibin Li <zhibin.li[at]iit.it>
	@version 0.0 02/04/15

	@section License
	Copyright (C) 2015 Department of Advanced Robotics, Italian Institute of Technology, all rights reserved.

	@section Description
	Reflex behavior realized by an engineering way. Work in progress, all parameteric interfaces are provisional.
	Comments: best to finish retraction control completely with state machine coordination, then add extension reflex.
*/

#include "ArmReflex.hpp"

ArmReflex::ArmReflex():
dT(0.002),	// defualt value, later in config it will be assigned again
Tcutoff(50.0),  // hard coded
Fz_th(5.0),
t_wait(2.0),
reflexMode(false),
isControllerOn(false),
isDebugMode(false)
{
    // NOTE: _flg_retract and _flg_ext are initialized as false by default
    shoulder.Initialization(dT, true);
    elbow.Initialization(dT, false);

    shoulder.max_count_contact=shoulder.setPushbacktime(t_wait, dT);
    elbow.max_count_contact=elbow.setPushbacktime(t_wait, dT);

    // you must initialize m_FTl here, otherwise it crashes
    m_FTl = VectorXd::Zero(6);
    m_FTr = VectorXd::Zero(6);

    hand_contact="00";

    // for lateral push to the right
    shoulder.A_retract=1.0;
    elbow.A_retract=100.0/57.3;

    if (isDebugMode)
    {
        cout<<"Fz_th: \t"<<Fz_th<<endl;
        cout<<"Tcutoff: \t"<<Tcutoff<<endl;
        cout<<"shoulder.flg_retract.enable: \t"<<shoulder.flg_retract.enable<<endl;
        cout<<"shoulder.flg_retract.done: \t"<<shoulder.flg_retract.done<<endl;
        cout<<"elbow.flg_retract.enable: \t"<<elbow.flg_retract.enable<<endl;
        cout<<"elbow.flg_retract.done: \t"<<elbow.flg_retract.done<<endl;

        cout<<"shoulder.m_retraction: \t"<<shoulder.m_retraction<<endl;
        cout<<"elbow.m_retraction: \t"<<elbow.m_retraction<<endl;
    }


}

ArmReflex::~ArmReflex()
{
}




void ArmReflex::reflex(bool fall_trigger, double realtime, VectorXd &leftHandFT, VectorXd &rightHandFT)
{
    // apply filter
    m_FTl=(Tcutoff*m_FTl+dT*leftHandFT)/(Tcutoff+dT);
    m_FTr=(Tcutoff*m_FTr+dT*rightHandFT)/(Tcutoff+dT);

//    cout<<"Fz left: "<<leftHandFT.norm()<<"\t"<<"Fz right: "<<rightHandFT.norm()<<endl;

//    cout<<"Fz left:\t"<<m_FTl.norm()<<"\t"<<"Fz right:"<<m_FTr.norm()<<endl;

//    cout<<"left:\t"<<leftHandFT<<endl;

    shoulder.armContact(leftHandFT, Fz_th);   // temperarily, I use shoulder for one arm, elbow for another
    elbow.armContact(rightHandFT, Fz_th);

    setArmContactPhase(shoulder.isContact, elbow.isContact);

//    cout<<"hand_contact: "<<hand_contact<<endl;

    stateMachine(fall_trigger, realtime);
}


bool ArmReflex::contactPhase(const VectorXd &FTl,const VectorXd &FTr)
{
    return 0;
}


bool ArmReflex::reflexSelection()
{
    return 0;
}


void ArmReflex::reflexActivation(double threshold, double window, double dT)
{

}

/**
 * @brief ArmReflex::emergencyLevel
 * @param realtime
 * @param idealtime
 * @param gain
 * @section the level of emergency will determine the value of gain
 */
void ArmReflex::emergencyLevel(double realtime, double idealtime, double gain)
{

}


void ArmReflex::extensionReflex()
{

}

void ArmReflex::reflexDeactivation(double realtime, double idealtime)
{

}


void ArmReflex::enable()
{
    isControllerOn = true;
}

void ArmReflex::disable()
{
    isControllerOn = false;
}

bool ArmReflex::isEnabled()
{
    return isControllerOn;
}

// get remaining position vector and time of swing arm
void ArmReflex::updateswingArmData(Vector3d remainP,double remainTime)
{
//	m_remainPos=remainP;
//	m_remainTime=remainTime;
}

// get remaining position vector and time of left right arm
void ArmReflex::updateswingArmTraj(Vector3d remainP_left,double remainTime_left,Vector3d remainP_right,double remainTime_right, Vector3d remainP,double remainTime)
{
//    m_remainPos_left=remainP_left;
//    m_remainTime_left=remainTime_left;

//    m_remainPos_right=remainP_right;
//    m_remainTime_right=remainTime_right;

//    m_remainTime=remainTime;

//    if (hand_contact=="10")
//    {
//        m_remainPos=remainP_left;
//    }
//    else if (hand_contact=="01")
//    {
//        m_remainPos=remainP_right;
//    }
//    else
//    {
//        m_remainPos<<0,0,0;
//    }
}

void ArmReflex::setArmContactPhase(bool leftarm, bool rightarm)
{
    // pass in value correct
    shoulder.updateHandContact(leftarm);
    elbow.updateHandContact(rightarm);

    if (leftarm==0 && rightarm==1)
    {
        hand_contact="01";
    }
    else if (leftarm==1 && rightarm==1)
    {
        hand_contact="11";
    }
    else if (leftarm==1 && rightarm==0)
    {
        hand_contact="10";
    }
    else
    {
        hand_contact="00";
        // both hand aerial phase
    }
}

void ArmReflex::stateMachine(bool fall_trigger, double realtime)
{
	/*-------  early landing detection ------*/
    // determine whether or not it is going to fall, fall activation
    bool armActivation=shoulder.swingArmActivation(fall_trigger);

    // if fall activated, and state is 00, set to 10 state
    if (armActivation&&!shoulder.flg_retract.enable&&!shoulder.flg_retract.done)
	{        
        shoulder.flg_retract.enable=true;
        elbow.flg_retract.enable=true;

        shoulder.flg_retract.time=realtime;

        shoulder.Tn=1.0*shoulder.T_retract;   // response time Tn 0.5 is half of the remain time
        shoulder.zeta=1.0;    // damping ratio
        shoulder.reflexConfig(dT);

        elbow.Tn=1.0*shoulder.T_retract;
        elbow.zeta=1.0;    // damping ratio
        elbow.reflexConfig(dT);
        if (isDebugMode)
        {
//            cout<<"m_remainTime"<<"\t"<<m_remainTime<<"\t";
            //cout<<"left retraction triggered"<<"\t"<<armLeft.A_retract<<"\t"<<Tn<<endl;
        }
	}

    /*-------- first part, set flags, state machine --------*/
    // if 10 state, the falling is deactivated, then set flags to 00
    if (shoulder.flg_retract.enable && !armActivation)//
	{
        shoulder.flg_retract.enable=false;
        elbow.flg_retract.enable=false;

		// this switch happens only once
        shoulder.Tn=0.5;   // hard coded clearance time
        shoulder.zeta=1.0;    // damping ratio
        shoulder.reflexConfig(dT);

        // resolve potential place for bug seen in exp
        // this switch happens only once
        elbow.Tn=0.5;   // hard coded clearance time
        elbow.zeta=1.0;    // damping ratio
        elbow.reflexConfig(dT);
        if (isDebugMode)
        {
            //cout<<"left clearance triggered"<<"\t"<<endl;
        }
	}
    // 01 state, deactivate, 01 --> 00
    // falling flag is disabled, then automatically jumps back to 00 state
    else if (!shoulder.flg_retract.enable&&shoulder.flg_retract.done)
	{
        shoulder.flg_retract.done=false;
        elbow.flg_retract.done=false;
        if (isDebugMode)
        {
            // auto transit 01 state to 00 state
            //cout<<"left flags all deactivated"<<"\t"<<endl;
        }
	}

    // add hand contact state, if contact force > threshold, set done 1
    // so below you configure only once the flags, for push back reflex
    if (shoulder.flg_retract.enable && !shoulder.flg_retract.done && shoulder.isContact)
    {
        shoulder.flg_retract.done=true;
        shoulder.Tn=0.8;   // response time Tn 0.5 is half of the remain time
        shoulder.zeta=1.0;    // damping ratio
        shoulder.reflexConfig(dT);
    }
    if (elbow.flg_retract.enable && !elbow.flg_retract.done && elbow.isContact)
    {
        // potential place for bug seen in exp
        elbow.flg_retract.done=true;
        elbow.Tn=0.8;
        elbow.zeta=1.0;    // damping ratio
        elbow.reflexConfig(dT);
    }

	/*------------- apply the control actions based on the flags -----------------*/
    // logic flow:
    // 00: rest condition, or clearance
    // 10: activation of reaching, push forward, reflex in action
    // 11: if new hand contact is activated for a certain time, then .done=1
    // 11: in 11 state, activate push back reflex
    // 01: if falling flag is disabled, then automatically jumps back to 00 state

    // 10 state
    if (shoulder.flg_retract.enable&&!shoulder.flg_retract.done)
	{
        // apply control modification, shoulder.m_retraction
        shoulder.reflexRetractionOutput(shoulder.A_retract, shoulder.a_max_retract);
        elbow.reflexRetractionOutput(elbow.A_retract, elbow.a_max_retract);
        if (0)
        {
            cout<<"Left retracting:"<<"\t"<<shoulder.A_retract<<"\t"<<shoulder.m_retraction(0)<<endl;
        }
	}
    // 00 state: deactivation, clearance
    else if (!shoulder.flg_retract.enable&&!shoulder.flg_retract.done)
	{
		// clear control modification
        shoulder.reflexRetractionClear(0.0, shoulder.a_max_retract);	// always clear to zero
        elbow.reflexRetractionClear(0.0, elbow.a_max_retract);	// always clear to zero
        if (isDebugMode)
        {
            cout<<"Left clearing"<<"\t"<<shoulder.m_retraction(0)<<endl;
        }
        if ( abs(shoulder.m_retraction(0))<1e-6)
        {
            shoulder.m_retraction(0)=0.0;
            shoulder.flg_retract.done=false;
        }
        if ( abs(elbow.m_retraction(0))<1e-6)
        {
            elbow.m_retraction(0)=0.0;
            elbow.flg_retract.done=false;
        }
	}
    // new state: 11, activate pushing back
    else if (shoulder.flg_retract.enable&&shoulder.flg_retract.done)
	{
        double mag_elbow = 0.8*elbow.A_retract;
        double mag_shoulder = 0.5*mag_elbow;
        shoulder.reflexRetractionOutput(shoulder.A_retract+mag_shoulder, shoulder.a_max_retract);
        elbow.reflexRetractionOutput(elbow.A_retract-mag_elbow, elbow.a_max_retract);
        if (0)
        {
            cout<<"push back!"<<endl; //
        }
	}

}


double ArmReflex::getarmTrajMod()
{

}


// NOT debugged!
void ArmReflex::setReflexMode(bool flag)
{
    reflexMode=flag;
}

void ArmReflex::setSamplingTime(double T)
{
    dT=T;
    shoulder.Initialization(dT, true);
    elbow.Initialization(dT, false);
}


/*-------  testing code, using nested classes to better wrap up for left and right arm -----------*/

ArmReflex::virtualModel::virtualModel():
    A_retract(0.7),
    T_retract(0.3), // here is the place you tune the response settling time
    A_ext(0.0),
    T_ext(10.0),
    Tn(T_retract),
    a_max_retract(20.0),
    v_max_retract(0.5),
    a_max_ext(3.0),
    v_max_ext(0.5),
    p_max_ext(0.02),
    zeta(1.0),
    isActivation(false),
    isContact(false),
    max_count_activation(5),
    counterActivation(0),
    max_count_contact(5),
    counterContact(0),
    arm_phase(2,0)
{
    m_retraction<<0,0,0;
    m_extension<<0,0,0;
}

ArmReflex::virtualModel::~virtualModel()
{
}


void ArmReflex::virtualModel::Initialization(double dT, bool arm)
{
	A<<0,1,
	   0,0;
	B<<0,1;

	At=MatrixXd::Identity(2, 2) + A*dT;
	Bt=B*dT;

    whicharm=arm;
}

//void ArmReflex::virtualModel::reflexRetractionActivation(double dT, double Tn, double zeta)
//{
//	reflexConfig(dT, Tn, zeta);
//}

void ArmReflex::virtualModel::reflexConfig(double T)
{
    if (Tn<=0.1)
    {
        Tn=0.1; // prevent wrong pass in data, Tn cannot be negative or zero.
    }
	double fn=1.0/Tn;
	m0=M_PI*M_PI*T*T*fn*fn;
	m1=2.0*M_PI*M_PI*T*T*fn*fn;
	m2= M_PI*M_PI*T*T*fn*fn;
	n0=(M_PI*M_PI*T*T*fn*fn + 2.0*M_PI*zeta*T*fn + 1.0);
	n1=(2.0*M_PI*M_PI*T*T*fn*fn - 2.0);
	n2=(M_PI*M_PI*T*T*fn*fn - 2.0*M_PI*zeta*T*fn + 1.0);
	K=4.0*M_PI*M_PI*fn*fn;
	D=4.0*M_PI*fn*zeta;
}

// note that unitResponseConstrained is reused, therefore output is the parameter input
double ArmReflex::virtualModel::unitResponseConstrained(double input, double a_max_retract, Vector3d &output)
{
	output(2)=K*(input-output(0))-D*output(1);

	if ( abs(output(2)) > a_max_retract )
	{
		output(2) = output(2)/abs(output(2))*a_max_retract;
	}

	Vector2d x( output(0),output(1) );

	x=At*x+Bt*output(2);

	output<<x,output(2);

	return output(0);
}


double ArmReflex::virtualModel::reflexRetractionOutput(double input, double a_max_retract)
{
	return unitResponseConstrained(input, a_max_retract,m_retraction);
}

double ArmReflex::virtualModel::reflexRetractionClear(double input, double a_max_retract)
{
    return unitResponseConstrained(input, a_max_retract, m_retraction);
}

double ArmReflex::virtualModel::getRetractionOutput()
{
	return m_retraction(0);
}


double ArmReflex::virtualModel::getExtensionOutput()
{
    return m_extension(0);
}

// trigger based on boolean judgement
bool ArmReflex::virtualModel::swingArmActivation(bool fall_trigger)
{
    if (fall_trigger == true && !isActivation)
    {
        counterActivation++;
//        cout<<"count: "<<counter<<"\t Fz: "<<FT(2)<<"\t whicharm: "<<whicharm<<endl;
    }
    else if (counterActivation>=0 && !fall_trigger)
    {
        counterActivation--;
    }

    if(counterActivation>max_count_activation)
    {
        isActivation=true;
//        cout<<"Collision=true"<<"\t whicharm: "<<whicharm<<endl;
    }
    else if (counterActivation<0)
    {
        isActivation=false;
    }

//    cout<<"count: "<<counterActivation<<"\t isActivation \t"<<isActivation<<endl;
//    cout<<"isAct?\t"<<isActivation<<endl;
    return isActivation;
}


bool ArmReflex::virtualModel::armContact(VectorXd & FT, double Fz_th)
{
    if ( FT.norm()>=Fz_th && !isContact )
    {
        counterContact++;
    }
    else if (counterContact>=0 && FT.norm()<Fz_th)
    {
        counterContact--;
    }

    if(counterContact>max_count_contact)
    {
        isContact=true;
    }
    else if (counterContact<0)
    {
        isContact=false;
    }

    return isContact;
}



bool ArmReflex::virtualModel::contactArmPushBack(double realtime, double trigger_time)
{
    if ( realtime>trigger_time && !isPushBack )
    {
        counterPush++;
    }
    else if (counterPush>=0 && realtime>trigger_time)
    {
        counterPush--;
    }

    if(counterPush>max_count_push)
    {
        isPushBack=true;
    }
    else if (counterPush<0)
    {
        isPushBack=false;
    }

    return isPushBack;
}


void ArmReflex::virtualModel::updateHandContact(bool phase)
{
    if(arm_phase.at(1)!=phase)
    {
        arm_phase.at(0)=arm_phase.at(1);
        arm_phase.at(1)=phase;
    }
}


void ArmReflex::virtualModel::set_A_retract(double mag)
{
    A_retract=mag;
}


void ArmReflex::virtualModel::set_T_retract(double time)
{
    T_retract=time;
}

int ArmReflex::virtualModel::setPushbacktime(double time, double T)
{
    return floor(abs(time)/abs(T));
}
