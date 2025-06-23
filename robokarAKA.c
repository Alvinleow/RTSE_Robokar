#include "..\inc\kernel.h"                  /* Always include these to use uCOS-II      */
#include "..\inc\hal_robo.h"                /*   and RoboKar HAL                        */
#include <util/delay.h>
#include <stdbool.h>

#define TASK_STK_SZ            128          /* Size of each task's stacks (# of bytes)  */
#define TASK_START_PRIO          1          /* Highest priority                         */
#define TASK_SWITCH_ROUTE_PRIO   2
#define TASK_CHKCOLLIDE_PRIO     3
#define TASK_CTRLMOTOR_PRIO      4
#define TASK_NAVIG_PRIO          5 
#define TASK_LED_PRIO            6

#define ROUTE_OTHERS             0
#define ROUTE_AB                 1
#define ROUTE_EF                 2

char ROUTE = ROUTE_OTHERS;                  /* Current route                            */

int reverse_color_route(int pattern) {
    switch (pattern) {
        case 0: return 7;  // 000 → 111
        case 1: return 6;  // 001 → 110
        case 2: return 5;  // 010 → 101
        case 3: return 4;  // 011 → 100
        case 4: return 3;  // 100 → 011
        case 5: return 2;  // 101 → 010
        case 6: return 1;  // 110 → 001
        case 7: return 0;  // 111 → 000
        default: return 0; // fallback for invalid input
    }
}

OS_STK TaskStartStk[TASK_STK_SZ];           /* TaskStartTask stack                      */
OS_STK ChkCollideStk[TASK_STK_SZ];          /* Task StopOnCollide stack                 */
OS_STK CtrlmotorStk[TASK_STK_SZ];           /* Task CtrlMotors stack                    */
OS_STK NavigStk[TASK_STK_SZ];               /* Task NavigRobot stack                    */
OS_STK LEDStk[TASK_STK_SZ];                 /* Task LED stack                           */
OS_STK ToggleRouteStk[TASK_STK_SZ];         /* Task ToggleRoute stack                   */

/* ------ Global shared variable -------*/
/* Ideally, this should be protected by a semaphore etc */
struct robostate {
    int rspeed;                             /* right motor speed  (-100 -- +100)        */
    int lspeed;                             /* leftt motor speed  (-100 -- +100)        */
    char obstacle;                          /* obstacle? 1 = yes, 0 = no                */
    int collision;                          /* collision? 1 = yes, 0 = no               */
} myrobot;

/*------High pririority task----------*/
void CheckCollision (void *data) {
    for(;;)
    {
        if ( (robo_proxSensor() == 1) )             /* obstacle?                         */
            myrobot.obstacle = 1;                   /* signal obstacle present           */
        else
            myrobot.obstacle = 0;                   /* signal no obstacle                */

		OSTimeDlyHMSM(0, 0, 0, 100);                /* Task period ~ 100 ms              */
    }
}

/* Control robot Motors TASK */
void CntrlMotors (void *data) {
    int speed_r, speed_l;

    for(;;)
    {
        speed_r = myrobot.rspeed;
        speed_l = myrobot.lspeed;
        robo_motorSpeed(speed_l, speed_r);
        OSTimeDlyHMSM(0, 0, 0, 10);                /* Task period ~ 10 ms              */
    }
}

/* --- Task for navigating robot ----
 * Write you own navigation task here
 */

void Navig (void *data) {
    int line_value;

    for (;;) {
        line_value = (ROUTE == ROUTE_AB) ? reverse_color_route(robo_lineSensor()) : robo_lineSensor();

        if (ROUTE == ROUTE_EF) {
            // ROUTE_EF
            switch (line_value) {
                case 0:  // All white � REVERSE
                    myrobot.rspeed = -LOW_SPEED;
                    myrobot.lspeed = -LOW_SPEED;
                    cputs("000 - All white � REVERSE\r\n");
                    break;
                    
                case 1:  // Veering left � Turn RIGHT
                    myrobot.rspeed = STOP_SPEED;
                    myrobot.lspeed = MEDIUM_SPEED;
                    cputs("001 - Veering LEFT � Turn RIGHT\r\n");
                    break;
                    
                case 2:  // Centered � Move FORWARD
                    myrobot.rspeed = LOW_SPEED;
                    myrobot.lspeed = LOW_SPEED;
                    cputs("010 - Centered � Move FORWARD\r\n");
                    break;

                case 3:  // Veering left � Turn RIGHT
                    myrobot.rspeed = STOP_SPEED;
                    myrobot.lspeed = MEDIUM_SPEED;
                    cputs("001 - Veering LEFT � Turn RIGHT\r\n");
                    break;
                    
                case 4:  // Veering right � Turn LEFT
                    myrobot.rspeed = MEDIUM_SPEED;
                    myrobot.lspeed = STOP_SPEED;
                    cputs("100 - Veering RIGHT � Turn LEFT\r\n");
                    break;
                    
                case 6:  // Veering right � Turn LEFT
                    myrobot.rspeed = MEDIUM_SPEED;
                    myrobot.lspeed = STOP_SPEED;
                    cputs("100 - Veering RIGHT � Turn LEFT\r\n");
                    break;

                case 7:  // All black � Turn RIGHT
                    myrobot.rspeed = STOP_SPEED;
                    myrobot.lspeed = MEDIUM_SPEED;
                    cputs("111 - All black � Turn RIGHT\r\n");
							
					section++;
					robo_Honk();
                    break;

                case 5:
                default:
                    myrobot.rspeed = LOW_SPEED;
                    myrobot.lspeed = LOW_SPEED;
                    break;
            }
            robo_motorSpeed(myrobot.lspeed, myrobot.rspeed);
            OSTimeDlyHMSM(0, 0, 0, 30);
        } else {
            // ROUTE_OTHERS
            if(myrobot.obstacle == 0) {
                switch (line_value) {
                    case 0:  // All white � REVERSE
                        myrobot.rspeed = -LOW_SPEED;
                        myrobot.lspeed = -LOW_SPEED;
                        cputs("000 - All white � REVERSE\r\n");
                        break;
                    
                    case 1:  // Veering left � Turn RIGHT
                        myrobot.rspeed = STOP_SPEED;
                        myrobot.lspeed = MEDIUM_SPEED;
                        cputs("001 - Veering LEFT � Turn RIGHT\r\n");
                        break;
                    
                    case 2:  // Centered � Move FORWARD
                        myrobot.rspeed = LOW_SPEED;
                        myrobot.lspeed = LOW_SPEED;
                        cputs("010 - Centered � Move FORWARD\r\n");
                        break;

                    case 3:  // Veering left � Turn RIGHT
                        myrobot.rspeed = STOP_SPEED;
                        myrobot.lspeed = MEDIUM_SPEED;
                        cputs("001 - Veering LEFT � Turn RIGHT\r\n");
                        break;
                    
                    case 4:  // Veering right � Turn LEFT
                        myrobot.rspeed = MEDIUM_SPEED;
                        myrobot.lspeed = STOP_SPEED;
                        cputs("100 - Veering RIGHT � Turn LEFT\r\n");
                        break;
                    
                    case 6:  // Veering right � Turn LEFT
                        myrobot.rspeed = MEDIUM_SPEED;
                        myrobot.lspeed = STOP_SPEED;
                        cputs("100 - Veering RIGHT � Turn LEFT\r\n");
                        break;

                    case 7:  // All black � STOP
                        myrobot.rspeed = STOP_SPEED;
                        myrobot.lspeed = STOP_SPEED;
                        cputs("111 - All black � STOP\r\n");
                        break;

                    case 5:
                    default:
                        myrobot.rspeed = LOW_SPEED;
                        myrobot.lspeed = LOW_SPEED;
                        break;
                }
                robo_motorSpeed(myrobot.lspeed, myrobot.rspeed);
                OSTimeDlyHMSM(0, 0, 0, 5);
            } else if (myrobot.obstacle == 1) {
                // REVERSE
                myrobot.rspeed = -LOW_SPEED;
                myrobot.lspeed = -LOW_SPEED;
                robo_motorSpeed(myrobot.lspeed, myrobot.rspeed);
                OSTimeDlyHMSM(0, 0, 0 , 300);
   
                // Turn RIGHT
                myrobot.rspeed = STOP_SPEED;
                myrobot.lspeed = MEDIUM_SPEED;
                robo_motorSpeed(myrobot.lspeed, myrobot.rspeed);
                OSTimeDlyHMSM(0, 0, 0, 400);
   
                // Move FORWARD
                myrobot.rspeed = LOW_SPEED;
                myrobot.lspeed = LOW_SPEED;
                robo_motorSpeed(myrobot.lspeed, myrobot.rspeed);
                OSTimeDlyHMSM(0, 0, 0, 600);
           }
        }
        OSTimeDlyHMSM(0, 0, 0, 5);
    }
}

void ToggleLED(void *data) {

    for (;;)
    {
        int light_val = robo_lightSensor();

        if (light_val > 75)
        {
            robo_Honk();
            robo_LED_toggle();

        }

        OSTimeDlyHMSM(0, 0, 0, 50);   /* Task period ~ 10 ms */
    }
}

/*------Highest pririority task----------*/
/* Create all other tasks here           */
void TaskStart( void *data ) {
    OS_ticks_init();                                        /* enable RTOS timer tick        */

    OSTaskCreate(CheckCollision,                            /* Task function                 */
                (void *)0,                                  /* nothing passed to task        */
                (void *)&ChkCollideStk[TASK_STK_SZ - 1],    /* stack allocated to task       */
                TASK_CHKCOLLIDE_PRIO);                      /* priority of task              */

    OSTaskCreate(CntrlMotors,                               /* Task function                 */
                (void *)0,                                  /* nothing passed to task        */
                (void *)&CtrlmotorStk[TASK_STK_SZ - 1],     /* stack allocated to task       */
                TASK_CTRLMOTOR_PRIO);                       /* priority of task              */

    OSTaskCreate(Navig,                                     /* Task function                 */
                (void *)0,                                  /* nothing passed to task        */
                (void *)&NavigStk[TASK_STK_SZ - 1],         /* stack allocated to task       */
                TASK_NAVIG_PRIO);                           /* priority of task              */

    OSTaskCreate(ToggleLED,                                 /* Task function                 */
                (void *)0,                                  /* nothing passed to task        */
                (void *)&LEDStk[TASK_STK_SZ - 1],           /* stack allocated to task       */
                TASK_LED_PRIO);                             /* priority of task              */

    while(1)
    {
        OSTimeDlyHMSM(0, 0, 5, 0);                          /* Task period ~ 5 secs          */
        robo_LED_toggle();                                  /* Show that we are alive        */
    }
}

void SwitchRouteTask(void *data) {
    char isPressed_before = 0;
    char isPressed_now;
    char number_pressed = 0;

    while (robo_goPressed()) {
        OSTimeDlyHMSM(0, 0, 0, 50);
    }

    for (;;) {
        isPressed_now = robo_goPressed();

        if (isPressed_now && !isPressed_before) {
            number_pressed++;
            if (number_pressed == 1) {
                ROUTE = ROUTE_AB;
                robo_Honk();
            } else if (number_pressed == 2) {
                ROUTE = ROUTE_OTHERS;
                robo_Honk();
				robo_Honk();
            } else if (number_pressed == 3) {
                ROUTE = ROUTE_EF;
                robo_Honk(); 
				robo_Honk(); 
				robo_Honk();
                number_pressed = 0;
            }
            OSTimeDlyHMSM(0, 0, 0, 200);
        }

        isPressed_before = isPressed_now;
        OSTimeDlyHMSM(0, 0, 0, 50);
    }
}

int main( void ) {
    robo_Setup();                                          /* initialize HAL for RoboKar     */                    

	OSInit();                                              /* initialize UCOS-II kernel      */

    robo_motorSpeed(STOP_SPEED, STOP_SPEED);               /* Stop the robot                 */
    myrobot.rspeed   = STOP_SPEED;                         /* Initialize myrobot states      */
    myrobot.lspeed   = STOP_SPEED;
    myrobot.obstacle = 0;                                  /*  No collisioin                 */
    myrobot.collision = 0;                                 /*  No collisioin                 */

    OSTaskCreate(TaskStart,                                /* create TaskStart Task          */
                (void *)0,
                (void *)&TaskStartStk[TASK_STK_SZ - 1],
                TASK_START_PRIO);
	
    OSTaskCreate(SwitchRouteTask,                          /* create SwitchRoute Task        */
                (void *)0,
                (void *)&TaskStartStk[TASK_STK_SZ - 1],
                TASK_SWITCH_ROUTE_PRIO);

	cputs("System booted!\r\n");
	robo_Honk(); 
	robo_wait4goPress();                                   /* Wait for to GO                 */

    OSStart();                                             /* Start multitasking             */
    while (1);                                             /* die here                       */
}

