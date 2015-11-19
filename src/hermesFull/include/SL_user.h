/*!=============================================================================
  ==============================================================================

  \file    SL_user.h

  \author  Stefan Schaal
  \date    Jan 2004

  ==============================================================================
  \remarks
  
  robot specific definitions
  
  ============================================================================*/
  
#ifndef _SL_user_
#define _SL_user_

/*! the robot name */
#define ROBOT_NAME "hermes"

#define HAS_UPPER_BODY
#define HAS_LOWER_BODY

/* dimensions of the robot */
#define PELVISOFFSET  0.0042 //!< 0.165in
#define PELVIS2THORAX 0.1761 //!< 6.933in
#define THORAX2HEAD   0.3 
#define XHIP          0.0889 //!< 3.5in
#define YHIP          0.0417 //!< 1.63in 
#define YKNEE         0.0647 //!< 2.548in
#define UPPERLEG      0.3950 //!< 15.43in
#define UPPERLEGMOD   0.3919 //!< 15.309in
#define LOWERLEG      0.3840 //!< 15.000in
#define THORAX2SHOULDER 0.111 //!< 4.38in
#define SHOULDERX     0.2768 //!< 10.899in
#define SHOULDERY     0.024  //!< 0.95 in
#define UPPERARM      0.2577 //!< 10.146 in
#define LOWERARM      0.2408 //!< 9.481 in
#define WRISTY        0.00376 //!< 0.148 in

#define THORAX2NECK   0.385
#define CERVICAL      0.0001
#define HEAD          0.1356
#define EYEXOFF       0.0366
#define EYEYOFF       0.0674
#define EYE           0.03
#define TOPofHEAD     0.28

#define THIGHANGLEOFF (15.5/180.*PI)

#define FOOT             0.22
#define XTOE_ORG         0.025
#define YTOE_ORG         0.18
#define ZTOE             0.0512
#define XHEEL_ORG        0.05
#define YHEEL_ORG        0.065
#define ZHEEL            0.0512
#define XMETATARSAL_ORG  0.05
#define YMETATARSAL_ORG  0.14
#define XFOOTASYM        0.01

#define IMU_X_OFFSET  0.129
#define IMU_Y_OFFSET  0.1855

// hands length
#define XHAND		0.075

// finger dimensions
#define HANDXROT        0.9
#define THUMBZROT       0.1
#define THUMBANGLEOFF (0.3)
#define XTHUMB          0.067
#define YTHUMB          0.01
#define ZTHUMB          0.005
#define XTHUMBFLEX      0.02
#define YTHUMBFLEX      0.04
#define TH1SEG          0.030
#define TH2SEG          0.023
#define XIF             0.10
#define YIF           (-0.01)
#define ZIF             0.03
#define XMF             0.1
#define YMF           (-0.02)
#define ZMF             0.015
#define XRF             0.095
#define YRF           (-0.03)
#define ZRF            (-0.00)
#define XLF             0.09
#define YLF           (-0.04)
#define ZLF            (-0.015)
#define F1SEG           0.030
#define F2SEG           0.022
#define F3SEG           0.020


// world coordinate offset of foot force/torque/acc sensor relative to AAA
// note that the signs of the offsets need to chosen correctly for the left and right foot in the C-code
#define FTA_X_OFF 0.01024    //!< 0.4in
#define FTA_Y_OFF 0.014848   //!< 0.58in
#define FTA_Z_OFF 0.0433152  //!< 1.692in

/* special dimensions of neck arrangement */                                                   
#define NECK_A 0.0508 //Ludo 0.0606   /*! the distance between the base hinges of the linear neck actuators */
#define NECK_B 0.0314 //Ludo 0.0329   /*! the maximal moment arm for the head-nod movement */
#define NECK_C 0.1589 //Ludo 0.4649   /*! vertical length from base hinges to universal joint */

/*! links of the robot */
enum RobotLinks {
  B_SACRAL=1,

  L_SHOULDER,
  L_SHOULDER_AA,
  L_SHOULDER_HR,
  L_ELBOW,
  L_WRIST,
  L_HAND,

  L_THUMB_R,
  L_THUMB_F,
  L_THUMB_S1,
  L_THUMB_S2,
  
  L_INDEX_FINGER,
  L_INDEX_FINGER_S1,
  L_INDEX_FINGER_S2,
  L_INDEX_FINGER_S3,
  
  L_MIDDLE_FINGER,
  L_MIDDLE_FINGER_S1,
  L_MIDDLE_FINGER_S2,
  L_MIDDLE_FINGER_S3,
  
  L_RING_FINGER,
  L_RING_FINGER_S1,
  L_RING_FINGER_S2,
  L_RING_FINGER_S3,

  L_LITTLE_FINGER,
  L_LITTLE_FINGER_S1,
  L_LITTLE_FINGER_S2,
  L_LITTLE_FINGER_S3,
  
  R_SHOULDER,
  R_SHOULDER_AA,
  R_SHOULDER_HR,
  R_ELBOW,
  R_WRIST,
  R_HAND,

  R_THUMB_R,
  R_THUMB_F,
  R_THUMB_S1,
  R_THUMB_S2,
  
  R_INDEX_FINGER,
  R_INDEX_FINGER_S1,
  R_INDEX_FINGER_S2,
  R_INDEX_FINGER_S3,
  
  R_MIDDLE_FINGER,
  R_MIDDLE_FINGER_S1,
  R_MIDDLE_FINGER_S2,
  R_MIDDLE_FINGER_S3,
  
  R_RING_FINGER,
  R_RING_FINGER_S1,
  R_RING_FINGER_S2,
  R_RING_FINGER_S3,

  R_LITTLE_FINGER,
  R_LITTLE_FINGER_S1,
  R_LITTLE_FINGER_S2,
  R_LITTLE_FINGER_S3,
  
  B_CERVICAL1,
  B_CERVICAL2,
  R_EYE_AXIS,
  R_EYE,
  L_EYE_AXIS,
  L_EYE,
  B_HEAD,

  R_HIP,
  R_HIP_R,
  R_KNEE,
  R_ANKLE,
  R_OUT_TOE,
  R_IN_TOE,
  R_OUT_METATARSAL,
  R_IN_METATARSAL,
  R_OUT_HEEL,
  R_IN_HEEL,
  R_FOOT,

  L_HIP,
  L_HIP_R,
  L_KNEE,
  L_ANKLE,
  L_OUT_TOE,
  L_IN_TOE,
  L_OUT_METATARSAL,
  L_IN_METATARSAL,
  L_OUT_HEEL,
  L_IN_HEEL,
  L_FOOT,

  N_ROBOT_LINKS
};

/*! endeffector information */
enum RobotEndeffectors {
  RIGHT_HAND=1,
  LEFT_HAND,
  RIGHT_FOOT,
  LEFT_FOOT,

  N_ROBOT_ENDEFFECTORS
};

/*! vision variables */
enum VisionCameras {
  CAMERA_1=1,
  CAMERA_2,

  N_VISION_CAMERAS
};

enum ColorBlobs {
  BLOB1=1,
  BLOB2,
  BLOB3,
  BLOB4,
  BLOB5,
  BLOB6,

  N_COLOR_BLOBS
};

#define FOVEA 1
#define WIDE  2

#define FOVEA_RES_HORIZONTAL  640
#define FOVEA_RES_VERTICAL    480
#define WIDE_RES_HORIZONTAL   640
#define WIDE_RES_VERTICAL     480
#define FOVEA_ANGLE            24
#define WIDE_ANGLE            100

/*! define the DOFs of this robot */
enum RobotDOFs {
  BASE=0,

  L_SFE, // 01
  L_SAA, // 02
  L_HR,  // 03
  L_EB,  // 04 
  L_WR,  // 05
  L_WFE, // 06 
  L_WAA, // 07
  
  R_SFE, // 08
  R_SAA, // 09 
  R_HR,  // 10
  R_EB,  // 11
  R_WR,  // 12
  R_WFE, // 13
  R_WAA, // 14

  L_HFE, // 15
  L_HAA, // 16
  L_HFR, // 17
  L_KFE, // 18
  L_AR,  // 19
  L_AFE, // 20
  L_AAA, // 21

  R_HFE, // 22
  R_HAA, // 23
  R_HFR, // 24
  R_KFE, // 25
  R_AR,  // 26
  R_AFE, // 27
  R_AAA, // 28

  B_TR,  // 29
  B_TAA, // 30
  B_TFE, // 31

  B_HN,  // 32
  B_HT,  // 33
  B_HR,  // 34

  R_EP,  // 35
  R_ET,  // 36
  L_EP,  // 37
  L_ET,  // 38

  L_THR, // 39
  L_THF, // 40
  L_IF,  // 41
  L_MF,  // 42
  L_RF,  // 43
  L_LF,  // 44

  R_THR, // 45
  R_THF, // 46
  R_IF,  // 47
  R_MF,  // 48
  R_RF,  // 49
  R_LF,  // 50

  N_ROBOT_DOFS
};

/*! define miscellaneous sensors of this robot */
enum MiscSensors {
  B_Q0_IMU=1,
  B_Q1_IMU,
  B_Q2_IMU,
  B_Q3_IMU,

	B_TSTAMP_IMU,

  B_AD_A_IMU,
  B_AD_B_IMU,
  B_AD_G_IMU,

  B_XACC_IMU,
  B_YACC_IMU,
  B_ZACC_IMU,

  B_XACC_UNSTAB_IMU,
  B_YACC_UNSTAB_IMU,
  B_ZACC_UNSTAB_IMU,

  L_CFx,
  L_CFy,
  L_CFz,
  L_CTa,
  L_CTb,
  L_CTg,

  R_CFx,
  R_CFy,
  R_CFz,
  R_CTa,
  R_CTb,
  R_CTg,

  R_FOOT_XACC,
  R_FOOT_YACC,
  R_FOOT_ZACC,

  L_FOOT_XACC,
  L_FOOT_YACC,
  L_FOOT_ZACC,

  N_ROBOT_MISC_SENSORS
};

/*! number of degrees-of-freedom of robot */
#define N_DOFS (N_ROBOT_DOFS-1)

/*! N_DOFS + fake DOFS, needed for parameter estimation; 
   fake DOFS come from creating endeffector information */
#define N_DOFS_EST (N_DOFS+45)

/*! N_DOFS to be excluded from parameter estimation (e.g., eye joints);
   these DOFS must be the last DOFS in the arrays */
#define N_DOFS_EST_SKIP 14

/*! number of links of the robot */
#define N_LINKS    (N_ROBOT_LINKS-1)

/*! number of miscellaneous sensors */
#define N_MISC_SENSORS   (N_ROBOT_MISC_SENSORS-1)

/*! number of endeffectors */
#define N_ENDEFFS  (N_ROBOT_ENDEFFECTORS-1)

/*! number of cameras used */
#define N_CAMERAS (N_VISION_CAMERAS-1)

/*! number of blobs that can be tracked by vision system */
#define MAX_BLOBS (N_COLOR_BLOBS-1)

/*! vision default post processing */
#define VISION_DEFAULT_PP "vision_default.pp"

/* the servo rate used by the I/O with robot: this limits the
   servo rates of all other servos */
#define  SERVO_BASE_RATE 1000

/* divisor to obtain task servo rate (task servo can run slower than
   base rate, but only in integer fractions */
#define  TASK_SERVO_RATIO   R1TO1

/* settings for D/A debugging information -- see SL_oscilloscope.c */
#define   D2A_CM      1
#define   D2A_CT      2
#define   D2A_CV      3
#define   D2A_CR      4

/* function prototype */
#ifdef __cplusplus
extern "C" {
#endif

  extern double MAG;
  extern double XMETATARSALINNER;
  extern double XMETATARSALOUTER;
  extern double XTOEINNER;
  extern double XTOEOUTER;
  extern double XHEELINNER;
  extern double XHEELOUTER;
  extern double YHEEL;
  extern double YMETATARSAL;
  extern double YTOE;



  extern void drawPhantomFromCurrentState(void);
  extern void drawPhantomFromDesiredState(void);


#ifdef __cplusplus
}
#endif


#endif  /* _SL_user_ */

