#include <Servo.h> 
#include <ArloRobot.h>                      
#include <SoftwareSerial.h>  
#include <math.h>

/*-----------------------------------------NOTES-----------------------------------------------------

1280-1480 1480-1520 1520-1720 pre servo
*/

/*-------------------------------------MACROS---------------------------------------------------------*/
#define DEBUG false;

/*--------------------------Useful Values--------------------------------------------------------------*/
#define POWER1 1400
#define POWER2 1600
#define ARLOPOWER 127

#define QTI_TRESHOLD 100
#define JOYTOLERANCE 100

#define OTACKA 213
#define STUPEN 0.59167
#define CENTER_X 506
#define CENTER_Y 503

/*--------------------------Pins-----------------------------------------------------------------------*/
#define CHANNEL1 13 
#define CHANNEL2 12

#define CLK_ENC 3
#define DT_ENC 7

#define QTI 2

#define SERVO_PIN 9

#define JOY_SWITCH 4

/*-------------------------Shorcuts for commands-------------------------------------------------------*/
#define SERVO_CLKWS turnServo.writeMicroseconds(POWER2);
#define SERVO_CCLKWS turnServo.writeMicroseconds(POWER1);
#define SERVO_STOP turnServo.writeMicroseconds(1500);

#define ARLOSTOP  Arlo.writeMotorPower(0,0);
#define ARLOGO    Arlo.writeMotorPower(0,ARLOPOWER);

#define READJOY Xjoy = analogRead(A0);Yjoy = analogRead(A1);
#define CALCJOY Jval=atan2((Yjoy-CENTER_Y),(Xjoy-CENTER_X))*57.295779513082320876798154814105;Jval=(int)Jval; 

/*---------------------------------------VARIABLES---------------------------------------------------*/
volatile long int feedback; // premenna do ktorej sa ukladaju z encodera

int Xjoy;
int Yjoy; 
double Jval;

volatile int counter = 0; 
volatile int aState;
volatile int aLastState;  

/*-----------------------------------------OBJECTS---------------------------------------------------*/
ArloRobot Arlo;
SoftwareSerial ArloSerial(CHANNEL2,CHANNEL1); 
Servo turnServo; //servo na obvode


void setup() {
//---------------PIN setup------
 pinMode (CLK_ENC,INPUT);
 pinMode (DT_ENC,INPUT); 
pinMode(JOY_SWITCH,INPUT_PULLUP); 

//--------------Setup of operations-----------------------------------
turnServo.attach(SERVO_PIN);

Serial.begin(9600); //PC
ArloSerial.begin(19200); //DHB

Arlo.begin(ArloSerial);

SERVO_CLKWS
while(qtiRead() != 0){
 
 debug('Q',qtiRead());
 
  delay(20);
  }
SERVO_STOP

//----------Encoder startup-----------------------------------------
aLastState = digitalRead(CLK_ENC);  
attachInterrupt(digitalPinToInterrupt(CLK_ENC),interruptFunction,CHANGE);

}

void loop() {
 READJOY
 //debug('X',Xjoy);
  //debug('Y',Yjoy);
 debug ('W',walkedAngle());

if ((abs(Yjoy-CENTER_Y)<JOYTOLERANCE)&&(abs(Xjoy-CENTER_X)<JOYTOLERANCE))
{
   
  //debug('C',1);
if (digitalRead(JOY_SWITCH)==0)
  {
   SERVO_STOP
    ARLOGO
       while (digitalRead(JOY_SWITCH)==0){
       SERVO_STOP
      debug ('G',1);
   delay(20);
   }
    ARLOSTOP
    debug('S',1);
  }
  }
else{

delay (200);
READJOY
CALCJOY
debug ('J',(int)(Jval));
 directServoAngle(Jval); 

}

delay(20);

}


/*---------------------------SERVO MOVEMENT FUNCTIONS----------------------------------------------------*/
int directServoAngle (signed int angle)
{
if (abs(angle - walkedAngle()) < 2)
return (1);

 debug ('N',angle);
   debug('M',walkedAngle());
    
    if (walkedAngle()>=angle) {
    
  
         while(walkedAngle()>=(angle))
        {
        debug ('W',walkedAngle());
        debug('A',angle);
        
         SERVO_CCLKWS
         delay(20);         
        } 
         SERVO_STOP
        
        delay(20);
       
    }    
    else {

  
    while (walkedAngle()<=(angle)) 
        {
         debug ('W',walkedAngle());
        debug('A',angle);

          
         SERVO_CLKWS
         
          delay(20);
        }
       SERVO_STOP
      
        delay(20);

    }   
     
return(0);   
  }


     

/*---------------------------------------Driver-----------------------------------------------------------*/
void interruptFunction(){
   aState = digitalRead(CLK_ENC); 
   if (aState != aLastState){     
     if (digitalRead(DT_ENC) != aState) { 
       counter ++;
     } else {
       counter --;
     }
   } 
   aLastState = aState; 
  }


int qtiRead(){
  long duration = 0;
   pinMode(QTI, OUTPUT);     // Make pin OUTPUT
   digitalWrite(QTI, HIGH);  // Pin HIGH (discharge capacitor)
   delay(1);                      // Wait 1ms
   pinMode(QTI, INPUT);      // Make pin INPUT
   digitalWrite(QTI, LOW);   // Turn off internal pullups
   while(digitalRead(QTI)){  // Wait for pin to go LOW
      duration++;
   }
   if (duration<QTI_TRESHOLD)
   return 0;
   else
   return 1;
  }

/*-----------------------------------Other------------------------------------------------------------------*/
int walkedAngle(){
 int angle;
  angle = counter/STUPEN;
  return angle; 
  }


void debug (char m1, int m2)
{
  bool d = DEBUG;
  if (d==true){
Serial.print(m1);
Serial.println(m2, DEC);
  }
  }
