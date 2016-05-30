//Skeleton code to be used when implementing firmware for the subject IIS301T
//The code controls the ArduMoto motor controller and reads data from
//  encapsulated motor wheel encoders in order to determine rotation rates.
//Rotation rates are used to control the individual wheel speeds in order to move
//  to specific locations

//BOARD: Arduino UNO
//
//                  ----------------------
//                 | arduino UNO          |
//                 |                      |
//                -| 3V3              D13 | - left_motor_direction
//                -| 5V               D12 | - right_motor_direction
//                 | pwm          pwm	D11 | - left_motor_pwm
//                -| RST          pwm D10 | -
//                -| AREF         pwm  D9 | -
//                 |                   D8 | -
//                -| A0                D7 | - chn_r_a
//                -| A1           pwm  D6 | - chn_r_b
//                -| A2           pwm  D5 | - chn_l_b
//                -| A3                D4 | - chn_l_a
//		            -| A4           pwm  D3 | - right_motor_pwm
//	              -| A5                D2 | -
//                 |              TX   D1 | -
//                 |              RX   D0 | -
//                 |                      |
//                  ----------------------

// v1.1
//    Cleaned up the code by creating functions
// v1.2 - 29/10/14
//    Added seperate direction control constants for the wheels: 
//          left_fwd, left_rev, right_fwd, right_rev
//    Added seperate physical parameters for the left and right wheels: 
//          Seperate encoder tick variables for each wheel
//          Seperate wheel circumference variables




//DO NOT CHANGE THESE PIN ASSIGNMENTS
//These pins are used for the ArduMoto motor controller
const int right_motor_pwm = 3;  //PWM control for motor on output A
const int left_motor_pwm = 11;  //PWM control for motor on output B
const int right_motor_direction = 12;
const int left_motor_direction = 13;
//The following pins are used for the encoder inputs
const int chn_l_a = 4;  //Left wheel encoder channel A
const int chn_l_b = 5;  //Left wheel encoder channel B
const int chn_r_a = 7;  //Right wheel encoder channel A
const int chn_r_b = 6;  //Right wheel encoder channel B

//DO NOT CHANGE THESE ASSIGNMENTS
const int encoderArray[]={0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
const float pi = 3.142;
float v = 0.0;      //Velocity of center point of robot measured in mm/second
float w = 0.0;    //Vehicle's rotational speed measured in radians per second
unsigned long int old_time = 0;
unsigned long int time = 0;

//DO NOT CHANGE THE FOLLOWING VARIABLES
//These variable are used in the operation of the speed controller
int currLeft = 0;    //Used to calculate change delta ticks on left wheel
int prevLeft = 0;    //Used to calculate change delta ticks on left wheel
int currRight = 0;    //Used to calculate change delta ticks on right wheel
int prevRight = 0;    //Used to calculate change delta ticks on right wheel
int encCntLeft = 0;    //Actual measured amount of ticks on left wheel
int encCntRight = 0;   //Actual measured amount of ticks on right wheel
int encIn = 0;        //Variable used to hold the combination of prev and current encoder values
float error_l = 0.0;
float error_r = 0.0;
float e_old = 0.0;
float e_old2 = 0.0;
float e_old_r = 0.0;
float e_old2_r = 0.0;
float ticks_l = 0.0;
float ticks_r = 0.0;
float ticks_desired_l = 0.0;
float ticks_desired_r = 0.0;
float l_mot = 0.0;    //left motor reference speed
float l_control = 0.0;
float r_mot = 0.0;    //right motor reference speed
float r_control = 0.0;
float error_dist = 0.0;
float error_dist_old = 0.0;
float error_mot;

//ONLY CHANGE THIS UNDER SPECIAL CIRCUMSTANCES
//Constants for timing loop updating PID and control values
int delta_t = 99;    //time used by PID controller to update control data in milli-seconds
const float PID_dc = 1000 / (delta_t + 1);    //Calcualte how many times this loop fires per second ie converts delta_t to Hz
//Constants for the robot's physical parameters
const int left_ticks_per_rev = 3592; //2256; //1632;  //Wheel encoder ticks per revolution for left wheel
const int right_ticks_per_rev = 3592; //2256; //1632;  //Wheel encoder ticks per revolution for right wheel
const float wheelbase = 173.0;      //Wheelbase of chassis in milli-meters
const float left_wheel_radius = 35.0;  //Wheel radii in milli-meters
const float right_wheel_radius = 35.0;  //Wheel radii in milli-meters
const float left_wheel_circ = 2*pi*left_wheel_radius;
const float right_wheel_circ = 2*pi*right_wheel_radius;
const float left_seg_lin_distance = left_wheel_circ / left_ticks_per_rev;
//Constants controlling the individual wheel rotation direction
const boolean left_fwd = 1;      //If left motor is not rotating the right way change this between 1 and 0 to determine direction
const boolean right_fwd = 1;      //If right motor is not rotating the right way change this between 1 and 0 to determine direction
const boolean left_rev = !left_fwd;
const boolean right_rev = !right_fwd;
//Gain parameters for the speed controller PID controller
float Kp = 0.01;      // Proportional gain for PID controller
float Ki = 0.01;
float Kd = 0.01;
//Gain parameters for the speed control between the two motors, helping the robot travel in a straight line
float Kp_speed = 0.01;
float Ki_speed = 0;



//USER DEFINED VARIABLES
//Add any of your global variables from this point


//End of user defined global variables
/*   _____ ______ _______ _    _ _____  
  / ____|  ____|__   __| |  | |  __ \ 
 | (___ | |__     | |  | |  | | |__) |
  \___ \|  __|    | |  | |  | |  ___/ 
  ____) | |____   | |  | |__| | |     
 |_____/|______|  |_|   \____/|_|     
*/                                    
void setup()
{
//////////////////////////////////////////////////////////
//DO NOT CHANGE ANY CODE OUTSIDE OF THE CUSTOM CODE AREA
//////////////////////////////////////////////////////////
  setupMotorControl();
  Serial.begin(9600);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//CUSTOM CODE START
//Put your custom code here


//CUSTOM CODE END
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++    
}

/*
  _      ____   ____  _____  
 | |    / __ \ / __ \|  __ \ 
 | |   | |  | | |  | | |__) |
 | |   | |  | | |  | |  ___/ 
 | |___| |__| | |__| | |     
 |______\____/ \____/|_|     
*/                           
void loop()
{
//////////////////////////////////////////////////////////
//DO NOT CHANGE ANY CODE OUTSIDE OF THE CUSTOM CODE AREA
//////////////////////////////////////////////////////////
  
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//CUSTOM CODE START

  v=100;

//CUSTOM CODE END
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  


// This routine is executed every delta_t and is used to update the motor control values
//    
//Add code here which need to be executed at delta_t times
  time = millis();    //Get the current millis() value from the Arduino
  int interval = time-old_time; 
  if (interval > delta_t)    //Enter IF after delay of delta_t seconds
  {

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//CUSTOM CODE START


//CUSTOM CODE END
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++    
    velocityControl(v,w);      //Sends new v and w values to the motor controller      
    old_time = time;
  }  //end of PID Interval
} //end of Loop()

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//User defined functions and procedures START here





//User defined functions and procedures END here
//--------------------------------------------------------------------------
/*                                                                               
   SSSSSSSSSSSSSSS TTTTTTTTTTTTTTTTTTTTTTT     OOOOOOOOO     PPPPPPPPPPPPPPPPP   
 SS:::::::::::::::ST:::::::::::::::::::::T   OO:::::::::OO   P::::::::::::::::P  
S:::::SSSSSS::::::ST:::::::::::::::::::::T OO:::::::::::::OO P::::::PPPPPP:::::P 
S:::::S     SSSSSSST:::::TT:::::::TT:::::TO:::::::OOO:::::::OPP:::::P     P:::::P
S:::::S            TTTTTT  T:::::T  TTTTTTO::::::O   O::::::O  P::::P     P:::::P
S:::::S                    T:::::T        O:::::O     O:::::O  P::::P     P:::::P
 S::::SSSS                 T:::::T        O:::::O     O:::::O  P::::PPPPPP:::::P 
  SS::::::SSSSS            T:::::T        O:::::O     O:::::O  P:::::::::::::PP  
    SSS::::::::SS          T:::::T        O:::::O     O:::::O  P::::PPPPPPPPP    
       SSSSSS::::S         T:::::T        O:::::O     O:::::O  P::::P            
            S:::::S        T:::::T        O:::::O     O:::::O  P::::P            
            S:::::S        T:::::T        O::::::O   O::::::O  P::::P            
SSSSSSS     S:::::S      TT:::::::TT      O:::::::OOO:::::::OPP::::::PP          
S::::::SSSSSS:::::S      T:::::::::T       OO:::::::::::::OO P::::::::P          
S:::::::::::::::SS       T:::::::::T         OO:::::::::OO   P::::::::P          
 SSSSSSSSSSSSSSS         TTTTTTTTTTT           OOOOOOOOO     PPPPPPPPPP         
*/
//--------------------------------------------------------------------------
//DO NOT CHANGE THE FUNCTIONS AND PROCEDURES BENEATH

void setupMotorControl()
{
  pinMode(right_motor_pwm, OUTPUT);  //Set control pins to be outputs
  pinMode(left_motor_pwm, OUTPUT);
  pinMode(right_motor_direction, OUTPUT);
  pinMode(left_motor_direction, OUTPUT);
  digitalWrite(right_motor_pwm,LOW);
  digitalWrite(left_motor_pwm,LOW);
  digitalWrite(right_motor_direction,right_fwd);
  digitalWrite(left_motor_direction,left_fwd);        
  analogWrite(right_motor_pwm, 0);    
  analogWrite(left_motor_pwm, 0);    
  
  //Change default PWM frequency to 30Hz
  TCCR2B = (TCCR2B & 0xF8) | 7;
  
  PCICR |= (1<<PCIE2);  //Enable interupts in PORTD
  PCMSK2 |= (1<<PCINT23);  //enable int on PD7
  PCMSK2 |= (1<<PCINT22);  //enable int on PD6
  PCMSK2 |= (1<<PCINT21);  //enable int on PD5
  PCMSK2 |= (1<<PCINT20);  // enable int on PD4
  MCUCR=(1<<ISC01)|(1<<ISC00);
  
  pinMode(chn_l_a,INPUT);
  digitalWrite(chn_l_a,HIGH);
  pinMode(chn_l_b,INPUT);
  digitalWrite(chn_l_b,HIGH);  
  pinMode(chn_r_a,INPUT);
  digitalWrite(chn_r_a,HIGH);  
  pinMode(chn_r_b,INPUT);  
  digitalWrite(chn_r_b,HIGH);      
}

//This PID controller is based on the vw PID controller from Prof Braunl
//    Embedded Robotics - Mobile Robot Design and Applications with Embedded Systems, 3rd Edition, p89
void velocityControl(float v1, float w1)
{
    ticks_l = encCntLeft * PID_dc;    //Calculates the ticks per second for left wheel
    ticks_r = encCntRight * PID_dc;  //Calcualtes the ticks per second for the right wheel
    
    ticks_desired_l = (v1 - (wheelbase*w1)/2) * (left_ticks_per_rev / (2*pi*left_wheel_radius));
    ticks_desired_r = (v1 + (wheelbase*w1)/2) * (right_ticks_per_rev / (2*pi*right_wheel_radius));
    
    error_l = (ticks_desired_l - ticks_l);
    error_r = (ticks_desired_r - ticks_r);
    
    //Left motor PID Controller
    l_mot = Kp * (error_l - e_old) + Ki*(error_l + e_old)/2 + Kd*(error_l-2*e_old+e_old2); 
    //Right motor PID Controller
    r_mot = Kp * (error_r - e_old_r) + Ki*(error_r + e_old_r)/2 + Kd*(error_r-2*e_old_r+e_old2_r);    
    //Distance travelled between wheels PID controller
    error_dist = ticks_l - ticks_r + w1;
    error_mot = Kp_speed * (error_dist - error_dist_old) + Ki_speed * (error_dist + error_dist_old)/2;
    //Control signals for left and right motor
    l_control += l_mot - error_mot;   
    r_control += r_mot + error_mot; 
    
    //Clamps control signals to accepted values
    if (l_control > 250) l_control = 250;
    if (l_control < -250) l_control = -250;
  
    if (r_control > 250) r_control = 250;
    if (r_control < -250) r_control = -250;    
    
    //Sends control signals to pwm outputs for the motor control
    //If control signal is negative then the motor is reversed    
    //The ABS() of the control signal is used in order to get rid of a negative control value
    digitalWrite(right_motor_direction,right_fwd);    
    if (r_control < 0)     digitalWrite(right_motor_direction,right_rev);
    analogWrite(right_motor_pwm,abs(r_control));
    
    digitalWrite(left_motor_direction,left_fwd);            
    if (l_control < 0)     digitalWrite(left_motor_direction,left_rev);
    analogWrite(left_motor_pwm,abs(l_control));

    encCntLeft = 0;
    e_old2 = e_old;
    e_old = error_l;
    
    encCntRight = 0;
    e_old2_r = e_old_r;
    e_old_r = error_r; 
 
   error_dist_old = error_dist;       
}

void readencoder()
{  
  //Use this section if using quad input encoders  
  currLeft = B00110000 & PIND;    //Mask for data from PD4 and PD5 (left wheel encoder)
  currRight = B11000000 & PIND;    //Mask for inputs PD7 and PD6 (right wheel encoder)
  
  currLeft = currLeft >> 4;
  prevLeft = prevLeft << 2;  
  encIn = currLeft | prevLeft;
  encCntLeft += encoderArray[encIn];
  prevLeft = currLeft;

  currRight = currRight >> 6;
  prevRight = prevRight << 2;
  encIn = currRight | prevRight;
  encCntRight += encoderArray[encIn];
  prevRight = currRight;
}

/*********************************************************
//Int vector for when state change interupt fires on PORTD
/*********************************************************/
ISR(PCINT2_vect)
{
  readencoder();  
}
