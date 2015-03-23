/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                          (c) Copyright 2009-2010; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                            EXAMPLE CODE
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : JJL
*                 EHS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <includes.h>


/*
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*/
#if 0
typedef  enum {                                                 /* States for the motor drive task.                     */
    MOTOR_DRIVE_OFF_FORW_NEXT = 0,
    MOTOR_DRIVE_OFF_REV_NEXT,
    MOTOR_DRIVE_FORW,
    MOTOR_DRIVE_REV
} tState;

#endif

#define  FLAG_PUSH_BUTTON_PRESSED         (OS_FLAGS)0x0001u     /* Flag Definitions                                     */
#define  FLAG_BUMP_SENSOR_PRESSED         (OS_FLAGS)0x0002u
#define  FLAG_BUMP_SENSOR_RELEASED        (OS_FLAGS)0x0004u


typedef enum {
	FORWARD_STATE,
	CONVERSE_STATE,
	DONE_STATE,
	PATH_STATE_END
}Path_State;

typedef enum {
	INFINITE_STATE,
	BARRIER_STATE,
	END_STATE,
	TR_PRM_TOP
}Traversal_Primary_State;

typedef enum  {
	MOVE_REVERSE,
	BARR_CHANGE_ANGLE,
	MOVE_FORWARD,
	INF_CHANGE_ANGLE,
	TR_SEC_TOP
}Traversal_Secondry_State;

#define ANGLE_CHANGE_BLOCKED_STATE OS_OPT_PEND_FLAG_SET_ANY + \
									OS_OPT_PEND_FLAG_CONSUME
									
#define	ANGLE_CHANGE_NON_BLOCKED_STATE OS_OPT_PEND_FLAG_SET_ANY + \
										OS_OPT_PEND_FLAG_CONSUME + \
										OS_OPT_PEND_NON_BLOCKING
										

/*
*********************************************************************************************************
*                                            LOCAL VARIABLES
*********************************************************************************************************
*/

static  OS_TCB        AppTaskStartTCB;

/* motor movement task*/
static  OS_TCB        AppTaskMotorDriveTCB;
static	tDirection		AppRobotDirection;


static  OS_TCB        AppTaskControlTCB;
static 	OS_TCB		  AppTaskAngleChangeTCB;

/* Input sensor and push button monitor*/
static  OS_TCB        AppTaskInputMonitorTCB;

static  CPU_STK       AppTaskStartStk[APP_TASK_START_STK_SIZE];
static  CPU_STK       AppTaskLeftMotorDriveStk[APP_TASK_LEFT_MOTOR_DRIVE_STK_SIZE];
static  CPU_STK       AppTaskRightMotorDriveStk[APP_TASK_LEFT_MOTOR_DRIVE_STK_SIZE];
static  CPU_STK       AppTaskControlStk[APP_TASK_CONTROL_STK_SIZE];
static  CPU_STK       AppTaskInputMonitorStk[APP_TASK_INPUT_MONITOR_STK_SIZE];


//static  OS_FLAG_GRP   AppMotorGrp; 
static  OS_FLAG_GRP   AppAngleChangeGroup;


//static  const  tSide  AppRobotLeftSide  = LEFT_SIDE;
//static  const  tSide  AppRobotRightSide = RIGHT_SIDE;

                                                                /* Variables used by uC/Probe for monitoring            */
static  CPU_BOOLEAN   bLED[2] = {DEF_TRUE, DEF_FALSE};
//static  CPU_BOOLEAN   bRunForward[2];
//static  CPU_BOOLEAN   bRunReverse[2];
//static  CPU_BOOLEAN   bStop[2];

CPU_INT32U distCount = 0;
static  CPU_BOOLEAN enComputeDist = DEF_ENABLED;


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void      AppTaskStart         (void   *p_arg);
static  void      AppTaskMotorDrive    (void   *p_arg);
static  void      AppTaskControl       (void   *p_arg);
static  void      AppTaskInputMonitor  (void   *p_arg);

static  OS_FLAGS  AppMotorDriveFlagPend(tSide   eSide);
static  void      AppTasksInit         (void);

static void ComputeDistance(Traversal_Primary_State traversal_state, Traversal_Secondry_State traversal_Secondry_state);
	

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none.
*
* Returns     : none.
*********************************************************************************************************
*/

int  main (void)
{
    OS_ERR  err;


    BSP_IntDisAll();                                            /* Disable all interrupts.                              */

    OSInit(&err);                                               /* Init uC/OS-III.                                      */

    OSTaskCreate((OS_TCB     *)&AppTaskStartTCB,                /* Create the start task                                */
                 (CPU_CHAR   *)"App Task Start",
                 (OS_TASK_PTR ) AppTaskStart,
                 (void       *) 0,
                 (OS_PRIO     ) APP_TASK_START_PRIO,
                 (CPU_STK    *)&AppTaskStartStk[0],
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */
}


/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppTaskStart (void  *p_arg)
{
    CPU_INT32U  clk_freq;
    CPU_INT32U  cnts;
    OS_ERR      err;


   (void)&p_arg;

    BSP_Init();                                                 /* Initialize BSP functions                             */
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */

    clk_freq = BSP_CPUClkFreq();                                /* Determine SysTick reference freq.                    */
    cnts     = clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;        /* Determine nbr SysTick increments                     */
    OS_CPU_SysTickInit(cnts);                                   /* Init uC/OS periodic time src (SysTick).              */
    CPU_TS_TmrFreqSet(clk_freq);

#if (OS_CFG_STAT_TASK_EN > 0u)
    OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

    CPU_IntDisMeasMaxCurReset();

    OSTimeDlyHMSM(0u, 0u, 0u, 50u,                              /* Wait 50ms                                            */
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
    
    BSP_WheelSensorEnable();
    BSP_WheelSensorEnable();
    BSP_WheelSensorEnable();
    BSP_WheelSensorEnable();

    AppTasksInit();

    BSP_LED_On(1u);
    BSP_LED_Off(2u);

    while (DEF_ON) {                                            /* Task body, always written as an infinite loop.       */
        OSTimeDlyHMSM(0u, 0u, 1u, 0u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

        BSP_LED_Toggle(0u);

        bLED[0] ^= DEF_TRUE;
        bLED[1] ^= DEF_TRUE;
        
    }
}


static  void  AppTaskControl (void  *p_arg)
{
    OS_MSG_SIZE   msg_size;
    CPU_INT08U    ucSwitches;
    CPU_INT08U    ucDelta;
    CPU_INT08U   *pucMsg;
    OS_ERR        err;
    CPU_TS        ts;


    while (DEF_ON) {
        pucMsg = (CPU_INT08U  *)OSTaskQPend(0u,                 /* Pend until msg is received from switch monitor task. */
                                            OS_OPT_PEND_BLOCKING,
                                            &msg_size,
                                            &ts,
                                            &err);

                                                                /* Get data from message.                               */
        ucSwitches = pucMsg[0];
        ucDelta    = pucMsg[1];

        if ((ucDelta & 0x01u) && !(ucSwitches & 0x01u)) {       /* Check if right push button was just pressed.         */
            OSFlagPost(&AppAngleChangeGroup,            /* Post flag to right motor drive task that button ...  */
                        FLAG_PUSH_BUTTON_PRESSED,               /* ... has been pressed.                                */
                        OS_OPT_POST_FLAG_SET,
                       &err);
        }

        if ((ucDelta & 0x02u) && !(ucSwitches & 0x02u)) {       /* Check if left push button was just pressed.          */
            OSFlagPost(&AppAngleChangeGroup,             /* Post flag to left motor drive task that button ...   */
                        FLAG_PUSH_BUTTON_PRESSED,               /* ... has been pressed.                                */
                        OS_OPT_POST_FLAG_SET,
                       &err);
        }

#if 1
		 if (((ucDelta & 0x04u) && !(ucSwitches & 0x04u)) ||
		 	((ucDelta & 0x08u) && !(ucSwitches & 0x08u))) {
		 	
			
				OSTimeDlyResume(&AppTaskAngleChangeTCB, &err);
				
		
		#if 0
				OSTimeDlyHMSM(0u, 0u, 0u,  10u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
		#endif
		 	}
#endif
        if ((ucDelta & 0x04u) && !(ucSwitches & 0x04u)) {       /* Check if right bump sensor was just pressed.         */
            OSFlagPost(&AppAngleChangeGroup,            /* Post flag to right motor drive task that sensor ...  */
                        FLAG_BUMP_SENSOR_PRESSED,               /* ... has been pressed.                                */
                        OS_OPT_POST_FLAG_SET,
                       &err);
        }
		if ((ucDelta & 0x08u) && !(ucSwitches & 0x08u)) {		/* Check if left bump sensor was just pressed.			*/
			OSFlagPost(&AppAngleChangeGroup,			 /* Post flag to left motor drive task that sensor ...	 */
						FLAG_BUMP_SENSOR_PRESSED,				/* ... has been pressed.								*/
						OS_OPT_POST_FLAG_SET,
					   &err);
		}
		
        if ((ucDelta & 0x04u) && (ucSwitches & 0x04u)) {        /* Check if right bump sensor was just released.        */
            OSFlagPost(&AppAngleChangeGroup,            /* Post flag to right motor drive task that sensor ...  */
                        FLAG_BUMP_SENSOR_RELEASED,              /* ... has been released.                               */
                        OS_OPT_POST_FLAG_SET,
                       &err);
        }

		if ((ucDelta & 0x08u) && (ucSwitches & 0x08u)) {        /* Check if left bump sensor was just released.         */
            OSFlagPost(&AppAngleChangeGroup,             /* Post flag to left motor drive task that sensor ...   */
                        FLAG_BUMP_SENSOR_RELEASED,              /* ... has been released.                               */
                        OS_OPT_POST_FLAG_SET,
                       &err);
        }
		
    }
}

static  void  AppTaskMotorDrive (void  *p_arg)
{

    
	tDirection *dir;
	OS_ERR err;
	OS_MSG_SIZE msg_size;
	CPU_TS ts;

    BSP_MotorStop(LEFT_SIDE);
	BSP_MotorStop(RIGHT_SIDE);
	
    BSP_MotorSpeed(LEFT_SIDE, 52u << 8u);
	BSP_MotorSpeed(RIGHT_SIDE, 50u << 8u);

	BSP_MotorDir(LEFT_SIDE, FORWARD);
	BSP_MotorDir(RIGHT_SIDE,FORWARD);
	
	BSP_MotorRun(LEFT_SIDE);
	BSP_MotorRun(RIGHT_SIDE);
	
    while (DEF_ON) {
		
		dir = (tDirection *)OSTaskQPend(0u,
                    OS_OPT_PEND_BLOCKING,
                    &msg_size,
                    &ts,
                    &err);
		
		BSP_MotorStop(RIGHT_SIDE);
		BSP_MotorStop(LEFT_SIDE);
		
		switch(*dir) {
			
			case TURN_RIGHT: 
				BSP_MotorDir(LEFT_SIDE,FORWARD);
				BSP_MotorRun(LEFT_SIDE);
				BSP_MotorDir(RIGHT_SIDE,REVERSE);
				BSP_MotorRun(RIGHT_SIDE);
			break;

			case TURN_LEFT:
				BSP_MotorDir(RIGHT_SIDE,FORWARD);
				BSP_MotorRun(RIGHT_SIDE);
				BSP_MotorDir(LEFT_SIDE,REVERSE);
				BSP_MotorRun(LEFT_SIDE);
			break;

			case TURN_ABOUT:
				BSP_MotorDir(RIGHT_SIDE,FORWARD);
				BSP_MotorRun(RIGHT_SIDE);
				BSP_MotorDir(LEFT_SIDE,REVERSE);
				BSP_MotorRun(LEFT_SIDE);
				
			break;
			
			case FORWARD:
			case REVERSE:

				BSP_MotorDir(LEFT_SIDE,*dir);
				BSP_MotorDir(RIGHT_SIDE,*dir);
				BSP_MotorRun(LEFT_SIDE);
				BSP_MotorRun(RIGHT_SIDE);
				
			break;
			default:
				BSP_MotorStop(LEFT_SIDE);
				BSP_MotorStop(RIGHT_SIDE);
				BSP_DisplayStringDraw("DEFAULT",29u, 0u);
			break;
		}
		
   	}
    
}


//tDirection motor_dir;

static void act_on_traversal_sec_state( 
										CPU_INT16U time_in_ms,
										tDirection _dir) {

	
	OS_ERR err;
	//tDirection *dir;
	
	if(_dir > TURN_ABOUT)
		return;

	// = (tDirection *)malloc(sizeof(*dir));

	//motor_dir = _dir;
	
#if 0
		BSP_MotorStop(RIGHT_SIDE);
		BSP_MotorStop(LEFT_SIDE);
		
		switch(_dir) {
			
			case TURN_RIGHT: 
				BSP_MotorDir(LEFT_SIDE,FORWARD);
				BSP_MotorRun(LEFT_SIDE);
			break;

			case TURN_LEFT:
				BSP_MotorDir(RIGHT_SIDE,FORWARD);
				BSP_MotorRun(LEFT_SIDE);
			break;

			case TURN_ABOUT:
				BSP_MotorStop(RIGHT_SIDE);
				BSP_MotorStop(LEFT_SIDE);
				//BSP_MotorDir(RIGHT_SIDE,FORWARD);
				//BSP_MotorRun(LEFT_SIDE);
			break;
			
			default:
				BSP_MotorDir(LEFT_SIDE,_dir);
				BSP_MotorDir(RIGHT_SIDE,_dir);
				BSP_MotorRun(LEFT_SIDE);
				BSP_MotorRun(RIGHT_SIDE);
			break;
			
		}

#endif
	
	OSTaskQPost(&AppTaskMotorDriveTCB,
					(void*)&_dir,
					sizeof(_dir),OS_OPT_POST_FIFO,
					&err);

	if(err != OS_ERR_NONE) {
		BSP_DisplayStringDraw("ERROR",29u, 0u);
	}
	
	if(time_in_ms)	
		OSTimeDlyHMSM(0u, 0u, (time_in_ms/1000),  (time_in_ms%1000),
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

}

struct action_coordinator {
	tDirection dir;
	CPU_INT16U time_in_ms;
};

/*
	1. PATH_STATE
	2. TRAVERSAL_STATE
	3. ACTION_STATE
*/

static struct action_coordinator coordinator[PATH_STATE_END][TR_PRM_TOP][TR_SEC_TOP];

static void AppTaskAngleChange(void *p_arg) {

	
	CPU_TS ts;
	//CPU_TS prev_ts = 0;
	OS_ERR err;

	Path_State path_direction = FORWARD_STATE;
	Traversal_Primary_State traversal_state = INFINITE_STATE;
	Traversal_Secondry_State traversal_Secondry_state = MOVE_REVERSE;
	OS_OPT task_state = ANGLE_CHANGE_BLOCKED_STATE;
	
	OS_FLAGS flag;

	tDirection barr_dir;
	//int toggle = 0;
	
	OSTaskQFlush (&AppTaskMotorDriveTCB,
							  &err);

	while(DEF_ON) {
		
		flag = OSFlagPend(&AppAngleChangeGroup,
                    FLAG_BUMP_SENSOR_PRESSED,
                    0,
                    task_state,
                    &ts,
                    &err);
#if 1		
		if((ANGLE_CHANGE_NON_BLOCKED_STATE == task_state) && (OS_ERR_PEND_WOULD_BLOCK != err)) {

			if(OS_ERR_NONE != err) {
				BSP_DisplayStringDraw("ERROR",29u, 0u);
				break;
			}
			
			BSP_MotorStop(LEFT_SIDE);
			BSP_MotorStop(RIGHT_SIDE);
			
			path_direction++;

			if(DONE_STATE == path_direction) {
				BSP_MotorStop(LEFT_SIDE);
				BSP_MotorStop(RIGHT_SIDE);
				BSP_DisplayStringDraw("DONE",29u, 0u);
				ComputeDistance(traversal_state, traversal_Secondry_state);
				break;
			}
			
			traversal_state = END_STATE;
			traversal_Secondry_state = MOVE_REVERSE;			
		}
		else if(ANGLE_CHANGE_BLOCKED_STATE == task_state) {

			BSP_MotorStop(LEFT_SIDE);
			BSP_MotorStop(RIGHT_SIDE);

			flag = OSFlagPend(&AppAngleChangeGroup,
                    FLAG_BUMP_SENSOR_PRESSED,
                    50,
                    task_state,
                    &ts,
                    &err);


			BSP_MotorStop(LEFT_SIDE);
			BSP_MotorStop(RIGHT_SIDE);
			
			task_state = ANGLE_CHANGE_NON_BLOCKED_STATE;	
			traversal_state = BARRIER_STATE;
			traversal_Secondry_state = MOVE_REVERSE;
		}
		
/*
		switch(path_direction)
		{
			case 	FORWARD_STATE	: BSP_DisplayStringDraw("FORWARD",29u, 1u);
										break;
			case 	CONVERSE_STATE	: BSP_DisplayStringDraw("CONVERSE",29u, 1u);
										break;
			case	DONE_STATE		: BSP_DisplayStringDraw("DONE",29u, 1u);
										break;
			default					: BSP_DisplayStringDraw("ERROR 0",29u, 1u);
										break;
		}

	*/	
		switch(traversal_state) {

			case INFINITE_STATE:
			//	BSP_DisplayStringDraw("INFINITE",29u, 0u);
				traversal_Secondry_state = MOVE_FORWARD;
				task_state = ANGLE_CHANGE_BLOCKED_STATE;
				act_on_traversal_sec_state(0,FORWARD);
			break;

			case BARRIER_STATE:
			{
			//	BSP_DisplayStringDraw("BARIER",29u, 0u);
				if(MOVE_REVERSE == traversal_Secondry_state)
					ComputeDistance(traversal_state, traversal_Secondry_state);

				if(BARR_CHANGE_ANGLE == traversal_Secondry_state || 
					INF_CHANGE_ANGLE == traversal_Secondry_state) {
					
					 if( TURN_LEFT == coordinator[path_direction][traversal_state][traversal_Secondry_state].dir) 
					 	coordinator[path_direction][traversal_state][traversal_Secondry_state].dir = TURN_RIGHT;
					else if(TURN_RIGHT == coordinator[path_direction][traversal_state][traversal_Secondry_state].dir)
						 coordinator[path_direction][traversal_state][traversal_Secondry_state].dir = TURN_LEFT;
					
					if(FORWARD_STATE == path_direction)
						barr_dir =  coordinator[path_direction][traversal_state][traversal_Secondry_state].dir;
				}

				if (MOVE_FORWARD == traversal_Secondry_state)
					enComputeDist == DEF_ENABLED;
					
				act_on_traversal_sec_state(
					coordinator[path_direction][traversal_state][traversal_Secondry_state].time_in_ms,
					  coordinator[path_direction][traversal_state][traversal_Secondry_state].dir);
			
				if(INF_CHANGE_ANGLE == traversal_Secondry_state) {	 	
						traversal_state = INFINITE_STATE;
				}
				else {
						traversal_Secondry_state++;
				}
			}
			break;

			case END_STATE:
			//	BSP_DisplayStringDraw("END",29u, 0u);
			//BSP_MotorStop(RIGHT_SIDE);
			//BSP_MotorStop(LEFT_SIDE);
				if(MOVE_REVERSE == traversal_Secondry_state)
					ComputeDistance(traversal_state, traversal_Secondry_state);
			
				if(BARR_CHANGE_ANGLE == traversal_Secondry_state) {
					coordinator[path_direction][traversal_state][traversal_Secondry_state].dir = barr_dir;

					coordinator[path_direction][BARRIER_STATE][BARR_CHANGE_ANGLE].dir= (TURN_RIGHT == barr_dir)? TURN_LEFT : TURN_RIGHT;
					coordinator[path_direction][BARRIER_STATE][INF_CHANGE_ANGLE].dir = (TURN_RIGHT == barr_dir)? TURN_LEFT : TURN_RIGHT;
				}
				
				act_on_traversal_sec_state( 
 						coordinator[path_direction][traversal_state][traversal_Secondry_state].time_in_ms,
						coordinator[path_direction][traversal_state][traversal_Secondry_state].dir);

				if(INF_CHANGE_ANGLE == traversal_Secondry_state) {
					traversal_state = INFINITE_STATE;
					ComputeDistance(traversal_state, traversal_Secondry_state);
				}
				else{
						traversal_Secondry_state++;
				}
			break;

			default:
			break;
		}
#else

		BSP_MotorStop(LEFT_SIDE);
		BSP_MotorStop(RIGHT_SIDE);

		dir = REVERSE;
		OSTaskQPost(&AppTaskMotorDriveTCB,
					(void*)&dir,
					sizeof(dir),OS_OPT_POST_FIFO,
					&err);

		OSTimeDlyHMSM(0u, 0u, 2u, 0u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

		BSP_MotorStop(LEFT_SIDE);
		BSP_MotorStop(RIGHT_SIDE);

		dir = TURN_LEFT;
		OSTaskQPost(&AppTaskMotorDriveTCB,
							(void*)&dir,
							sizeof(dir),OS_OPT_POST_FIFO,
							&err);
		
		OSTimeDlyHMSM(0u, 0u, 2u, 0u,
					OS_OPT_TIME_HMSM_STRICT,
					&err);

		dir = FORWARD;
		OSTaskQPost(&AppTaskMotorDriveTCB,
					(void*)&dir,
					sizeof(dir),OS_OPT_POST_FIFO,
					&err);
		
		OSTimeDlyHMSM(0u, 0u, 2u, 0u,
							OS_OPT_TIME_HMSM_STRICT,
							&err);

		dir = TURN_RIGHT;
		OSTaskQPost(&AppTaskMotorDriveTCB,
							(void*)&dir,
							sizeof(dir),OS_OPT_POST_FIFO,
							&err);
		
		OSTimeDlyHMSM(0u, 0u, 2u, 0u,
					OS_OPT_TIME_HMSM_STRICT,
					&err);
		
		dir = FORWARD;
#endif
		
	}

}

static  void  AppTaskInputMonitor (void  *p_arg)
{
    CPU_INT08U pucMsg[2];
    CPU_INT08U ucData;
    OS_ERR     err;

    // The debounced state of the 4 switches. The bit positions
    // correspond to:
    //
    //     0 - Right Push Button
    //     1 - Left  Push Button
    //     2 - Right Bump Sensor
    //     3 - Left  Bump Sensor
    CPU_INT08U  ucSwitches;
    
    // The swithches that just changed state. The bit positions
    // are the same as g_ucPushButtons.
    CPU_INT08U  ucDelta;
    
    // The vertical counter used to debounce the switches.  The
    // bit positions are the same as g_ucPushButtons.
    CPU_INT08U  ucSwitchesClockA;
    CPU_INT08U  ucSwitchesClockB;


   (void)&p_arg;

                                                                /* Initialize the variables                             */
 //   ucSwitches       = 0x0Fu;
	ucSwitches       = 0x07u;
    ucSwitchesClockA =    0u;
    ucSwitchesClockB =    0u;
    
    while (DEF_ON) {
        OSTimeDlyHMSM(0u, 0u, 0u, 5u,                           /* Delay for 5 milliseconds.                            */
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);	
#if 0		
                                                            /* Read the state of the switches.                      */
        ucData =  BSP_PushButtonGetStatus(1u)        |          /*    Right Push Button                                 */
                 (BSP_PushButtonGetStatus(2u) << 1u) |          /*    Left  Push Button                                 */
                 (BSP_BumpSensorGetStatus(1u) << 2u) |          /*    Right Bump Sensor                                 */
                 (BSP_BumpSensorGetStatus(2u) << 3u);           /*    Left  Bump Sensor                                 */
#endif
		if(!BSP_BumpSensorGetStatus(1u) & !BSP_BumpSensorGetStatus(2u))
		{		
		ucData =  BSP_PushButtonGetStatus(1u)        |          /*    Right Push Button                                 */
                 (BSP_PushButtonGetStatus(2u) << 1u) |          /*    Left  Push Button                                 */
                 (BSP_BumpSensorGetStatus(1u) << 2u) |          /*    Right Bump Sensor                                 */
                 (BSP_BumpSensorGetStatus(2u) << 2u);           /*    Left  Bump Sensor  */
		}
		else if (!BSP_BumpSensorGetStatus(1u))
		{
		 ucData =  BSP_PushButtonGetStatus(1u)        |          /*    Right Push Button                                 */
                 (BSP_PushButtonGetStatus(2u) << 1u) |          /*    Left  Push Button                                 */
                 (BSP_BumpSensorGetStatus(1u) << 2u) |          /*    Right Bump Sensor                                 */
                 (!BSP_BumpSensorGetStatus(2u) << 2u);           /*    Left  Bump Sensor  */
		}
		else if(!BSP_BumpSensorGetStatus(2u))
		{		 
		ucData =  BSP_PushButtonGetStatus(1u)        |          /*    Right Push Button                                 */
                 (BSP_PushButtonGetStatus(2u) << 1u) |          /*    Left  Push Button                                 */
                 (!BSP_BumpSensorGetStatus(1u) << 2u) |          /*    Right Bump Sensor                                 */
                 (BSP_BumpSensorGetStatus(2u) << 2u);           /*    Left  Bump Sensor  */
		}
		else
		{		
		ucData =  BSP_PushButtonGetStatus(1u)        |          /*    Right Push Button                                 */
                 (BSP_PushButtonGetStatus(2u) << 1u) |          /*    Left  Push Button                                 */
                 (BSP_BumpSensorGetStatus(1u) << 2u) |          /*    Right Bump Sensor                                 */
                 (BSP_BumpSensorGetStatus(2u) << 2u);           /*    Left  Bump Sensor  */
		}
                 
																/* Determine the switches at a different state than ... */
        ucDelta = ucData ^ ucSwitches;                          /* ... the debounced state.                             */

        ucSwitchesClockA ^=  ucSwitchesClockB;                  /* Increment the clocks by one.                         */
        ucSwitchesClockB  = ~ucSwitchesClockB;

        ucSwitchesClockA &= ucDelta;                            /* Reset the clocks corresponding to switches that ...  */
        ucSwitchesClockB &= ucDelta;                            /* ... have not changed state.                          */

                                                                /* Get the new debounced switch state.                  */
        ucSwitches &=    ucSwitchesClockA | ucSwitchesClockB;
        ucSwitches |= (~(ucSwitchesClockA | ucSwitchesClockB)) & ucData;

        ucDelta ^= ucSwitchesClockA | ucSwitchesClockB;         /* Determine switches that changed debounced state.     */

		if(!(GPIOPinRead(RIGHT_IR_SENSOR_A_PORT, RIGHT_IR_SENSOR_A_PIN) || 
			GPIOPinRead(RIGHT_IR_SENSOR_B_PORT, RIGHT_IR_SENSOR_B_PIN)) && enComputeDist)
		{
			distCount++;
		}

#if 0
		if (ucDelta & 0x0Fu) {                                  /* Check if any button or bump sensor switched state.   */
            pucMsg[0] = ucSwitches;
            pucMsg[1] = ucDelta;

            OSTaskQPost((OS_TCB    *)&AppTaskControlTCB,
                        (void      *)&pucMsg[0],
                        (OS_MSG_SIZE) 2u,
                        (OS_OPT     ) OS_OPT_POST_FIFO,
                        (OS_ERR    *)&err);
        }
#endif
	if (ucDelta & 0x07u) {									/* Check if any button or bump sensor switched state.	*/
		pucMsg[0] = ucSwitches;
		pucMsg[1] = ucDelta;
	
		OSTaskQPost((OS_TCB    *)&AppTaskControlTCB,
					(void	   *)&pucMsg[0],
					(OS_MSG_SIZE) 2u,
					(OS_OPT 	) OS_OPT_POST_FIFO,
					(OS_ERR    *)&err);
	}

		
    }
}

static void ComputeDistance(Traversal_Primary_State traversal_state, Traversal_Secondry_State traversal_Secondry_state)
{
	CPU_INT32U ActualDist;
	char buff[10];
	if (enComputeDist)
	{
		ActualDist = (distCount/8)*72;
		//C = 2*pi*r = pi * d = 3.14 * 23 = 72.22 mm = 72 approx
        BSP_DisplayStringDraw("DISTANCE",29u, 0u);
		sprintf(buff, "= %d", ActualDist);
		BSP_DisplayStringDraw(buff,29u, 1u);
		enComputeDist = DEF_DISABLED;
		
	}
	return;
}

static void AppTasksInit(void)
{
    OS_ERR  err;

	memset(coordinator, TURN_ABOUT+1 , sizeof(coordinator));
	//static tDirection prevDir = TURN_LEFT;

	
	coordinator[FORWARD_STATE][BARRIER_STATE][MOVE_REVERSE].dir = REVERSE;
	coordinator[FORWARD_STATE][BARRIER_STATE][MOVE_REVERSE].time_in_ms = 300;

	coordinator[FORWARD_STATE][BARRIER_STATE][BARR_CHANGE_ANGLE].dir = TURN_LEFT;
	coordinator[FORWARD_STATE][BARRIER_STATE][BARR_CHANGE_ANGLE].time_in_ms = 1100;
	
	coordinator[FORWARD_STATE][BARRIER_STATE][MOVE_FORWARD].dir = FORWARD;
	coordinator[FORWARD_STATE][BARRIER_STATE][MOVE_FORWARD].time_in_ms = 2000;

	coordinator[FORWARD_STATE][BARRIER_STATE][INF_CHANGE_ANGLE].dir = TURN_LEFT;
	coordinator[FORWARD_STATE][BARRIER_STATE][INF_CHANGE_ANGLE].time_in_ms = 1100;



	coordinator[CONVERSE_STATE][END_STATE][MOVE_REVERSE].dir = REVERSE;
	coordinator[CONVERSE_STATE][END_STATE][MOVE_REVERSE].time_in_ms = 300;
	
	coordinator[CONVERSE_STATE][END_STATE][BARR_CHANGE_ANGLE].dir = TURN_RIGHT;
	coordinator[CONVERSE_STATE][END_STATE][BARR_CHANGE_ANGLE].time_in_ms = 1100;


/*
	coordinator[FORWARD_STATE][END_STATE][MOVE_REVERSE].dir = REVERSE;
	coordinator[FORWARD_STATE][END_STATE][MOVE_REVERSE].time_in_ms = 600;

	coordinator[FORWARD_STATE][END_STATE][BARR_CHANGE_ANGLE].dir = TURN_ABOUT;
	coordinator[FORWARD_STATE][END_STATE][BARR_CHANGE_ANGLE].time_in_ms = 1500;
*/
	/*coordinator[FORWARD_STATE][END_STATE][MOVE_FORWARD].dir = -1;
	coordinator[FORWARD_STATE][END_STATE][MOVE_FORWARD].time_in_ms = ;

	coordinator[FORWARD_STATE][END_STATE][INF_CHANGE_ANGLE].dir = 0;
	coordinator[FORWARD_STATE][END_STATE][INF_CHANGE_ANGLE].time_in_ms = 0;
	*/


	coordinator[CONVERSE_STATE][BARRIER_STATE][MOVE_REVERSE].dir = REVERSE;
	coordinator[CONVERSE_STATE][BARRIER_STATE][MOVE_REVERSE].time_in_ms = 300;

	coordinator[CONVERSE_STATE][BARRIER_STATE][BARR_CHANGE_ANGLE].dir = TURN_LEFT;
	coordinator[CONVERSE_STATE][BARRIER_STATE][BARR_CHANGE_ANGLE].time_in_ms = 1100;

	coordinator[CONVERSE_STATE][BARRIER_STATE][MOVE_FORWARD].dir = FORWARD;
	coordinator[CONVERSE_STATE][BARRIER_STATE][MOVE_FORWARD].time_in_ms = 2000;

	coordinator[CONVERSE_STATE][BARRIER_STATE][INF_CHANGE_ANGLE].dir = TURN_LEFT;
	coordinator[CONVERSE_STATE][BARRIER_STATE][INF_CHANGE_ANGLE].time_in_ms = 1100;


	// control flag between angle change and motor move
	OSFlagCreate(&AppAngleChangeGroup,
                 "The input Flag Group",
                 (OS_FLAGS)0,
                 &err);
	
    OSTaskCreate((OS_TCB     *)&AppTaskMotorDriveTCB,
                 (CPU_CHAR   *)"Motor Drive Task",
                 (OS_TASK_PTR ) AppTaskMotorDrive,
                 (void       *)&AppRobotDirection,
                 (OS_PRIO     ) APP_TASK_MOTOR_DRIVE_PRIO,
                 (CPU_STK    *)&AppTaskRightMotorDriveStk[0],
                 (CPU_STK_SIZE) APP_TASK_RIGHT_MOTOR_DRIVE_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_RIGHT_MOTOR_DRIVE_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);


		/* interrupt based tasks*/
	 OSTaskCreate((OS_TCB     *)&AppTaskInputMonitorTCB,
                 (CPU_CHAR   *)"Switch/Sensor Monitor Task",
                 (OS_TASK_PTR ) AppTaskInputMonitor,
                 (void       *) 0,
                 (OS_PRIO     ) APP_TASK_INPUT_MONITOR_PRIO,
                 (CPU_STK    *)&AppTaskInputMonitorStk[0],
                 (CPU_STK_SIZE) APP_TASK_INPUT_MONITOR_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_INPUT_MONITOR_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

	 
	 OSTaskCreate((OS_TCB	  *)&AppTaskControlTCB,
					  (CPU_CHAR   *)"Control Task",
					  (OS_TASK_PTR ) AppTaskControl,
					  (void 	  *) 0,
					  (OS_PRIO	   ) APP_TASK_CONTROL_PRIO,
					  (CPU_STK	  *)&AppTaskControlStk[0],
					  (CPU_STK_SIZE) APP_TASK_CONTROL_STK_SIZE / 10u,
					  (CPU_STK_SIZE) APP_TASK_CONTROL_STK_SIZE,
					  (OS_MSG_QTY  ) 6u,
					  (OS_TICK	   ) 0u,
					  (void 	  *) 0,
					  (OS_OPT	   )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
					  (OS_ERR	  *)&err);
	 OSTaskCreate((OS_TCB	  *)&AppTaskAngleChangeTCB,
					  (CPU_CHAR   *)"Angle Change Task",
					  (OS_TASK_PTR ) AppTaskAngleChange,
					  (void 	  *) 0,
					  (OS_PRIO	   ) APP_ANGLE_CHANGE_PRIO,
					  (CPU_STK	  *)&AppTaskLeftMotorDriveStk[0],
					  (CPU_STK_SIZE) APP_TASK_CONTROL_STK_SIZE / 10u,
					  (CPU_STK_SIZE) APP_TASK_CONTROL_STK_SIZE,
					  (OS_MSG_QTY  ) 6u,
					  (OS_TICK	   ) 0u,
					  (void 	  *) 0,
					  (OS_OPT	   )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
					  (OS_ERR	  *)&err);

}
