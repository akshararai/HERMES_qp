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
dT(0.001),	// defualt value, later in config it will be assigned again
a_max_retract(6.0),
v_max_retract(1.0),
A_retract(0.0),	// value will be computed before, not predetermined
T_retract(0.0), // value will be computed before, not predetermined
a_max_ext(5.0),
v_max_ext(0.3),	// max velocity allowed to extend leg, coz the collision force is realted with velocity before collision
p_max_ext(0.02),
A_ext(0.0), // value will be computed before, not predetermined
T_ext(0.0), // value will be computed before, not predetermined
Fz_th(8.0),	// tuned to be good in simulation
hand_contact("11"),
m_remainTime(0.0),
swingTime(0.0),
m_remainTime_left(0.0),
m_remainTime_right(0.0),
Tcutoff(5.0),  // hard coded
N(1.2),  // hard coded temp to change later
reflexMode(false),
isControllerOn(false)
{
    // NOTE: _flg_retract and _flg_ext are initialized as false by default
    armLeft.Initialization(dT, true);
    armRight.Initialization(dT, false);
    m_FTl = VectorXd::Zero(6);
    m_FTr = VectorXd::Zero(6);
    Fz_th=10.0;
    ds_Time=0.2;
    swingTime=0.5*(1.0-0.2);  // hard coded adaptation

    m_remainPos<<0.0,0.0,0.0;
    m_remainPos_left<<0.0,0.0,0.0;
    m_remainPos_right<<0.0,0.0,0.0;

    armRight.a_max_retract=a_max_retract;
    armLeft.a_max_retract=a_max_retract;

    if (1)
    {
        cout<<"swingTime: \t"<<swingTime<<endl;
        cout<<"Fz_th: \t"<<Fz_th<<endl;
        cout<<"Tcutoff: \t"<<Tcutoff<<endl;
        cout<<"armLeft.flg_retract.enable: \t"<<armLeft.flg_retract.enable<<endl;
        cout<<"armLeft.flg_retract.done: \t"<<armLeft.flg_retract.done<<endl;
        cout<<"armRight.flg_retract.enable: \t"<<armRight.flg_retract.enable<<endl;
        cout<<"armRight.flg_retract.done: \t"<<armRight.flg_retract.done<<endl;

        cout<<"armLeft.m_retraction: \t"<<armLeft.m_retraction<<endl;
        cout<<"armRight.m_retraction: \t"<<armRight.m_retraction<<endl;
    }


}

ArmReflex::~ArmReflex()
{
}




void ArmReflex::reflex(bool fall_trigger)
{
	// without filter at the moment
//    m_FTl=(Tcutoff*m_FTl+dT*FTl)/(Tcutoff+dT);
//    m_FTr=(Tcutoff*m_FTr+dT*FTr)/(Tcutoff+dT);
//    cout<<"Fz left:\t"<<m_FTl(2)<<"\t"<<"Fz right:"<<m_FTr(2)<<endl;

    stateMachine(fall_trigger);
}


bool ArmReflex::contactPhase(const VectorXd &FTl,const VectorXd &FTr)
{

}


bool ArmReflex::reflexSelection()
{

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
	m_remainPos=remainP;
	m_remainTime=remainTime;
}

// get remaining position vector and time of left right arm
void ArmReflex::updateswingArmTraj(Vector3d remainP_left,double remainTime_left,Vector3d remainP_right,double remainTime_right, Vector3d remainP,double remainTime)
{
    m_remainPos_left=remainP_left;
    m_remainTime_left=remainTime_left;

    m_remainPos_right=remainP_right;
    m_remainTime_right=remainTime_right;

    m_remainTime=remainTime;

    if (hand_contact=="10")
    {
        m_remainPos=remainP_left;
    }
    else if (hand_contact=="01")
    {
        m_remainPos=remainP_right;
    }
    else
    {
        m_remainPos<<0,0,0;
    }
}

void ArmReflex::setArmContactPhase(string leftarm, string rightarm)
{
    // pass in value correct
    armLeft.updateHandContact(leftarm);
    armRight.updateHandContact(rightarm);

    if (leftarm=="aerial"&& rightarm=="contact")
    {
        hand_contact="01";
    }
    else if (leftarm=="contact"&& rightarm=="contact")
    {
        hand_contact="11";
    }
    else if (leftarm=="contact"&& rightarm=="aerial")
    {
        hand_contact="10";
    }
    else
    {
        hand_contact="00";
        // both aerial phase
    }

}

void ArmReflex::stateMachine(bool fall_trigger)
{
	/*-------  early landing detection ------*/
    // left arm during swing phase, activate once
    bool leftActivation=armLeft.swingArmActivation(fall_trigger);
    if (leftActivation&&!armLeft.flg_retract.enable&&!armLeft.flg_retract.done)
	{
        armLeft.flg_retract.enable=true;
//        armLeft.A_retract=1.0; // pass in position is with respect to the moving arm frame
//        armLeft.T_retract=0.4;
        double Tn=1.0*armLeft.T_retract;   // response time Tn 0.5 is half of the remain time
        double zeta=1.5;    // damping ratio
        armLeft.reflexConfig(dT, Tn, zeta);
        if (1)
        {
//            cout<<"m_remainTime"<<"\t"<<m_remainTime<<"\t";
            cout<<"left retraction triggered"<<"\t"<<armLeft.A_retract<<"\t"<<Tn<<endl;
        }
	}

//    // right arm during swing phase, activate once
//    bool rightCollision=armRight.swingArmActivation(fall_trigger);
//    if (rightCollision)
//	{
//        armRight.flg_retract.enable=true;
//        armRight.A_retract=-m_remainPos(2);
//        armRight.T_retract=m_remainTime;
//        double Tn=0.5*armRight.T_retract;   // response time Tn 0.5 is half of the remain time
//        double zeta=0.8;    // damping ratio
//        armRight.reflexConfig(dT, Tn, zeta);
//        if (1)
//        {
//            cout<<"m_remainTime"<<"\t"<<m_remainTime<<"\t";
//            cout<<"right retraction triggered"<<"\t"<<armRight.A_retract<<"\t"<<Tn<<endl;
//        }
//	}


	/*-------- first part, set flags --------*/
    // for the left arm, Single Support (SS) phase, first time enter SS after activation in previous Swing Phase
	// changed to first time enter DS after activation in previous Swing Phase
    if (armLeft.flg_retract.enable && !armLeft.flg_retract.done && !leftActivation)// (armLeft.flg_retract.enable&&armLeft.flg_retract.done)
	{
		// here I need to get the duration (Step time) of the current SS phase!
        armLeft.flg_retract.enable=false;
        armLeft.flg_retract.done=true;
		// this switch happens only once
        double Tn=0.5;   // hard coded DS time for the test
        double zeta=1.0;    // damping ratio
        armLeft.reflexConfig(dT, Tn, zeta);
        if (1)
        {
            cout<<"left clearance triggered"<<"\t"<<endl;
        }
	}
    // for the left arm, during Double Support (DS) phase, change flags, deactivate
    // changed to: first time enter Left Single Support (SS) phase, deactivate
    else if (!armLeft.flg_retract.enable&&armLeft.flg_retract.done)
	{
        armLeft.flg_retract.enable=false;
        armLeft.flg_retract.done=false;
        if (1)
        {
            // auto transit 10 state to 00 state
            cout<<"left flags all deactivated"<<"\t"<<endl;
        }
	}


//    // for the right arm, Single Support (SS) phase, first time enter SS after activation in previous Swing Phase
//	// changed to first time enter DS after activation in previous Swing Phase
//    if (armRight.flg_retract.enable&&!armRight.flg_retract.done)
//	{
//        armRight.flg_retract.enable=false;
//        armRight.flg_retract.done=true;
//		// this switch happens only once
//        double Tn=0.8*0.2;   // hard coded DS time for the test
//		double zeta=0.9;    // damping ratio
//        armRight.reflexConfig(dT, Tn, zeta);
//        if (1)
//        {
//            cout<<"right clearance triggered"<<"\t"<<endl;
//        }
//	}
//    // for the right arm, during Double Support (DS) phase, change flags, deactivate
//    // changed to: first time enter Right Single Support (SS) phase, deactivate
//    else if (!armRight.flg_retract.enable&&armRight.flg_retract.done)
//	{
//        armRight.flg_retract.enable=false;
//        armRight.flg_retract.done=false;
//        if (1)
//        {
//            cout<<"right flags all deactivated"<<"\t"<<endl;
//        }
//	}

	/*------------- apply the control actions based on the flags -----------------*/
    // left arm
    if (armLeft.flg_retract.enable&&!armLeft.flg_retract.done)
	{
        // apply control modification, armLeft.m_retraction
        double output=0;
        output=armLeft.reflexRetractionOutput(armLeft.A_retract, a_max_retract);
        if (0)
        {
            cout<<"Left retracting:"<<"\t"<<armLeft.A_retract<<"\t"<<armLeft.m_retraction(0)<<endl;
        }
	}
    else if (!armLeft.flg_retract.enable&&!armLeft.flg_retract.done)
	{
		// clear control modification
        double output=0;
        output=armLeft.reflexRetractionClear(0.0, a_max_retract);	// always clear to zero
        if (0)
        {
            cout<<"Left clearing"<<"\t"<<output<<endl;
        }
        if ( abs(armLeft.m_retraction(0))<1e-6)
        {
            armLeft.m_retraction(0)=0.0;
            armLeft.flg_retract.done=false;
        }
	}
    else if (armLeft.flg_retract.enable&&armLeft.flg_retract.done)
	{
        if (1)
        {
            cout<<"unexpected state"<<endl; // warning for possible bug
        }
	}

//    // right arm
//    if (armRight.flg_retract.enable&&!armRight.flg_retract.done)
//	{
//		// apply control modification
//        double output=0;
//        output=armRight.reflexRetractionOutput(armRight.A_retract, armRight.a_max_retract);
//        if (1)
//        {
//            cout<<"Right retracting"<<"\t"<<armRight.A_retract<<"\t"<<armRight.m_retraction(0)<<endl;
//        }
//	}
//    else if (!armRight.flg_retract.enable&&armRight.flg_retract.done)
//	{
//		// clear control modification
//        double output=0;
//        output=armRight.reflexRetractionClear(0.0, armRight.a_max_retract);	// always clear to zero
//        if ( abs(armRight.m_retraction(0))<1e-6)
//        {
//            armRight.m_retraction(0)=0.0;
//            armRight.flg_retract.done=false;
//        }
//        if (1)
//        {
//            cout<<"Right clearing"<<"\t"<<output<<endl;
//        }
//	}
//    else if (armRight.flg_retract.enable&&armRight.flg_retract.done)
//	{
//        if (1)
//        {
//            cout<<"unexpected state"<<endl; // warning for possible bug
//        }
//	}


//    /*------------ below is the state machine for the leg extension ---------------*/
//    bool leftAerial=armLeft.stanceArmAerial(m_FTl, Fz_th);

//    if (leftAerial) // exclude double support phase:  && armLeft.arm_phase.at(1)!=armRight.arm_phase.at(1)
//    {
//        double temp=armLeft.m_extension(1);
//        armLeft.m_extension(1)+=-a_max_ext*dT;
//        if(abs(armLeft.m_extension(1))>v_max_ext)
//        {
//            armLeft.m_extension(1)=armLeft.m_extension(1)/abs(armLeft.m_extension(1))*v_max_ext;
//        }
//        armLeft.m_extension(0)+=0.5*(armLeft.m_extension(1)+temp)*dT;
////        cout<<"armLeft extension: "<<armLeft.m_extension(0)<<endl;
//    }
//    else
//    {
//        armLeft.m_extension(1)=-1.0/N*armLeft.m_extension(0);
//        armLeft.m_extension(0)+=armLeft.m_extension(1)*dT;
////        cout<<"armLeft clear: "<<armLeft.m_extension(0)<<endl;
//    }

//    if(abs(armLeft.m_extension(0))>p_max_ext)
//    {
//        armLeft.m_extension(0)=armLeft.m_extension(0)/abs(armLeft.m_extension(0))*p_max_ext;
//    }

//    bool rightAerial=armRight.stanceArmAerial(m_FTr, Fz_th);

//    if (rightAerial) // exclude double support phase:  && armLeft.arm_phase.at(1)!=armRight.arm_phase.at(1)
//    {
//        double temp=armRight.m_extension(1);
//        armRight.m_extension(1)+=-a_max_ext*dT;
//        if(abs(armRight.m_extension(1))>v_max_ext)
//        {
//            armRight.m_extension(1)=armRight.m_extension(1)/abs(armRight.m_extension(1))*v_max_ext;
//        }
//        armRight.m_extension(0)+=0.5*(armRight.m_extension(1)+temp)*dT;
//    }
//    else
//    {
//        armRight.m_extension(1)=-1.0/N*armRight.m_extension(0);
//        armRight.m_extension(0)+=armRight.m_extension(1)*dT;
//    }

//    if(abs(armRight.m_extension(0))>p_max_ext)
//    {
//        armRight.m_extension(0)=armRight.m_extension(0)/abs(armRight.m_extension(0))*p_max_ext;
//    }
////    cout<<"Right phase: "<<armRight.arm_phase.at(1)<<" m_remainTime "<<m_remainTime<<" ds_Time "<<ds_Time<<" extension: "<<armRight.m_extension(0)<<endl;

}


double ArmReflex::getarmTrajMod()
{

}


// NOT debugged!
void ArmReflex::setReflexMode(bool flag)
{
    reflexMode=flag;
}


/*-------  testing code, using nested classes to better wrap up for left and right arm -----------*/

ArmReflex::virtualModel::virtualModel():
    A_retract(0.7),
    T_retract(0.3), // here is the place you tune the response settling time
    A_ext(0.0),
    T_ext(10.0),
    a_max_retract(3.0),
    v_max_retract(0.5),
    a_max_ext(3.0),
    v_max_ext(0.5),
    p_max_ext(0.02),
    isActivation(false),
    isAerial(false),
    max_count_activation(5),
    counterActivation(0),
    max_count_aerial(3),
    counterAerial(0),
    arm_phase(2,"stance")
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

void ArmReflex::virtualModel::reflexConfig(double T, double Tn, double zeta)
{
    if (Tn<=0.05)
    {
        Tn=0.05; // prevent wrong pass in data, Tn cannot be negative or zero.
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
	return unitResponseConstrained(input, a_max_retract,m_retraction);
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


bool ArmReflex::virtualModel::stanceArmAerial(VectorXd & FT, double Fz_th)
{
    if ( FT(2)<Fz_th && (arm_phase.at(0)=="swing" && arm_phase.at(1)=="stance") )
    {
        counterAerial++;
//        cout<<"counterAerial: "<<counterAerial<<"\t Fz: "<<FT(2)<<"\t whicharm: "<<whicharm<<endl;
    }
    else
    {
        counterAerial=0;
    }

    if(counterAerial>max_count_aerial)
    {
        isAerial=true;
//        cout<<"Aerial=true"<<"\t whicharm: "<<whicharm<<endl;
    }
    else
    {
        isAerial=false;
    }

    return isAerial;
}


void ArmReflex::virtualModel::updateHandContact(string phase)
{
    if(arm_phase.at(1)!=phase)
    {
        arm_phase.at(0)=arm_phase.at(1);
        arm_phase.at(1)=phase;
    }
}
