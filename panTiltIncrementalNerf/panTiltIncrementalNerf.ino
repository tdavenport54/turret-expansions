/***********************************************************************************
 *     ______            RobotGeek Pan and Tilt Demo              ______
 *      |  |            Incremental  Analog Control                |  | 
 *      |__|_   _             & Nerf Launcher                _    _|__|
 *      |____|__|                                            |___|____|
 *  The following sketch will allow you to control a Desktop RobotTurret v3 using
 *  the included RobotGeek Joysticl and RobotGeek Pushbutton
 *
 *  Products
 *    http://www.robotgeek.com/robotgeek-pantilt.aspx
 *    http://www.robotgeek.com/robotgeek-geekduino-sensor-kit
 *    http://www.robotgeek.com/robotgeek-joystick
 *    http://www.robotgeek.com/p/power-supply-6vdc-2a.aspx
 *    
 *    
 *  Wiring
 *    Pan Servo - Digital Pin 9 
 *    Tilt Servo - Digital Pin 10 
 *    Fire / Trigger Servo - Digital Pin 11
 *
 *    Joystick1(Vertica)l   - Analog Pin 0
 *    Joystick2(Horizontal) - Analog Pin 1 
 *    Jumper for pins 9/10/11 should be set to 'VIN'
 *  
 *    NOTE: It is possible to control both pan and tilt with one joystick - this
 *          demo uses 2 for ease of use
 *
 *  Control Behavior:
 *    Moving the joysticks will incrementally move the servo. This means that when you
 *    release the joystick to its 'home' position, the servo will stay in the last place
 *    it was. This allows for fine control over the servo's position.
 *    The Horizontal Joystick will incrementaly move the Pan Servo
 *    The Vertical Joystick will incrementaly move the Tilt Servo
 *
 *  External Resources
 *
 ***********************************************************************************/
//Includes
#include <Servo.h>   //servo library to help control the servos

//Defines
#define PAN 9         //Pan Servo Digital Pin
#define TILT 10       //Tilt Servo Digital Pin
#define TRIGGER 11       //Tilt Servo Digital Pin
#define LASER_PIN 2       //Laser Digital Pin

#define H_JOY_PIN 1              //Horizontal Joystick Analog Pin
#define V_JOY_PIN 0              //Vertical Joystick Analog Pin 

//generic deadband limits - not all joystics will center at 512, so these limits remove 'drift' from joysticks that are off-center.
#define DEADBANDLOW 482   //decrease this value if drift occurs, increase it to increase sensitivity around the center position
#define DEADBANDHIGH 542  //increase this value if drift occurs, decrease it to increase sensitivity around the center position

//max/min puse values in microseconds to send to the servo
#define PAN_MIN      600  //full counterclockwise for RobotGeek 180 degree servo
#define PAN_MAX      2400 //full clockwise for RobotGeek 180 degree servo
#define TILT_MIN  600     //full counterclockwise for RobotGeek 180 degree servo
#define TILT_MAX  2400    //full clockwise for RobotGeek 180 degree servo
#define TRIGGER_MIN  1300     //full counterclockwise for RobotGeek 180 degree servo
#define TRIGGER_MAX  1900    //full clockwise for RobotGeek 180 degree servo

Servo panServo, tiltServo, triggerServo;        // create servo objects to control the pan and tilt servos
int horizontalValue, verticalValue;             //variables to hold the last reading from the analog pins for the horizontal and vertical joystick. The value will be between 0 and 1023
int horizontalValueMapped, verticalValueMapped; //the joystick values will be changed (or 'mapped') to new values to be sent to the servos.

int panValue = 1500;   //current positional value being sent to the pan servo. Start at the 'centered' position
int tiltValue = 1500;  //current positional value being sent to the tilt servo. Start at the 'centered' position
int triggerValue = 1300;  //current positional value being sent to the tilt servo. Start at the 'centered' position
int laserState = HIGH;
int speed = 20;        //alter this value to change the speed of the system. Higher values mean higher spe

void setup() 
{ 
   //initialize servos
  panServo.attach(PAN, PAN_MIN, PAN_MAX);  // attaches/activates the pan servo on pin PAN 
  tiltServo.attach(TILT, TILT_MIN, TILT_MAX);  // attaches/activates the tilt servo 
  triggerServo.attach(TRIGGER, TRIGGER_MIN, TRIGGER_MAX);  // attaches/activates the tilt servo 
  
  pinMode(2, OUTPUT);
 digitalWrite(2,HIGH);

  //Analog pins do not need to be initialized
  
  //use the writeMicroseconds to set the servos to their default positions
  panServo.writeMicroseconds(panValue); 
  tiltServo.writeMicroseconds(tiltValue); 
  triggerServo.writeMicroseconds(triggerValue);
  
  pinMode(4,INPUT);
  
} 

void loop() 
{ 

  /**************Servo Positions *******************************/
  //read the values from the analog sensors/joysticks
   horizontalValue = analogRead(H_JOY_PIN);
   verticalValue = analogRead(V_JOY_PIN);
   
   
 
   
   //only update the shoulder joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(horizontalValue > DEADBANDHIGH || horizontalValue < DEADBANDLOW)
   {
     horizontalValueMapped = map(horizontalValue, 0, 1023, speed, -speed); //Map analog value from native joystick value (0 to 1023) to incremental change (speed to -speed). Note that the program is mapping inversely - this is to allow the joystick movement to match the servo movement
     panValue = panValue - horizontalValueMapped; //add mapped shoulder joystick value to present Shoulder Value (positive values of joyShoulderMapped will increase the position, negative values will decrease the position)  Note that the program is mapping inversely - this is to allow the joystick movement to match the servo movement
   
     //even though the servos have min/max value built in when servo.attach() was called, the program must still keep the
     //panValue variable within the min/max bounds, or the turret may become unresponsive
     panValue = max(panValue, PAN_MIN);  //use the max() function to make sure the value never falls below PAN_MIN (0 degrees)
     panValue = min(panValue, PAN_MAX);  //use the min() function to make sute the value never goes above PAN_MAX (180 degrees)
   }
      
   //only update the shoulder joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(verticalValue > DEADBANDHIGH || verticalValue < DEADBANDLOW)
   {
     verticalValueMapped = map(verticalValue, 0, 1023, speed, -speed); //Map analog value from native joystick value (0 to 1023) to incremental change (speed to -speed)
     tiltValue = tiltValue - verticalValueMapped; //add mapped shoulder joystick value to present Shoulder Value (positive values of joyShoulderMapped will increase the position, negative values will decrease the position)
  
     //even though the servos have min/max value built in when servo.attach() was called, the program must still keep the
     //tiltValue variable within the min/max bounds, or the turret may become unresponsive
     tiltValue = max(tiltValue, TILT_MIN);//use the max() function to make sure the value never falls below 0
     tiltValue = min(tiltValue, TILT_MAX);//use the min() function to make sute the value never goes above 180  
   }

  //use the writeMicroseconds to set the servos to their new positions
  panServo.writeMicroseconds(panValue); 
  tiltServo.writeMicroseconds(tiltValue);
  
  delay(10); // waits for the servo to get to they're position before continuing 
  
  
  if(digitalRead(4) == HIGH)
  {
    triggerServo.writeMicroseconds(1375); 
    delay(500);
    triggerServo.writeMicroseconds(1300); 
    delay(500);
    
    
  }
  
} 

