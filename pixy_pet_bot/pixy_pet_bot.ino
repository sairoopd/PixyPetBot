#include "Arduino.h"
#include <SPI.h>  
#include <Pixy.h>

Pixy pixy;

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)
#define pwmA 3
#define dirA 2

#define pwmB 5
#define dirB 9

#define pwmC 6
#define dirC 10
int cal =0;
class ServoLoop
{
public:
  ServoLoop(int32_t pgain, int32_t dgain);

  void update(int32_t error);
   
  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_pgain;
  int32_t m_dgain;
};


ServoLoop panLoop(300, 500);
ServoLoop tiltLoop(500, 700);

ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = PIXY_RCS_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

void ServoLoop::update(int32_t error)
{
  long int vel;
  char buf[32];
  if (m_prevError!=0x80000000)
  {  
    vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>10;
    //sprintf(buf, "%ld\n", vel);
    //Serial.print(buf);
    m_pos += vel;
    if (m_pos>PIXY_RCS_MAX_POS) 
      m_pos = PIXY_RCS_MAX_POS; 
    else if (m_pos<PIXY_RCS_MIN_POS) 
      m_pos = PIXY_RCS_MIN_POS;
  }
  m_prevError = error;
}

typedef struct {
  int pulse;
  bool direction;
} MotorValues;

MotorValues motorA;
MotorValues motorB;
MotorValues motorC;

// Globals
double sideStep = 0.60; // Limiting factor to ensure direct side to side movement
double side =0.50;


void setup()
{
  Serial.begin(115200); 
  Serial.print("Starting...\n");
  
  pixy.init();
    

  // Set motor controller communication pins as outputs
  pinMode(dirA, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(dirB, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(dirC, OUTPUT);
  pinMode(pwmC, OUTPUT);
  
  // Command all motors to stop
  allStop();
}
uint32_t lastblocktime = 0;
void loop()
{ 
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  int32_t panError, tiltError;
  
  blocks = pixy.getBlocks();
  
  if (blocks)
  {
    Serial.println("Tracking");
    panError = X_CENTER-pixy.blocks[0].x;
    tiltError = pixy.blocks[0].y-Y_CENTER;
    
    panLoop.update(panError);
    tiltLoop.update(tiltError);
    
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
    int trackedblocks = 0;
    j++;

    if(j >= 4)
    {
    FollowBlock(trackedblocks);
    lastblocktime = millis();
    }
  }
  else if (millis() - lastblocktime > 100)
  {
   
    scanforblocks();
  }  
}
int scanincrement = (PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS)/150;
uint32_t lastmove = 0; 
void scanforblocks()
{
  allStop();
  if (millis()- lastmove > 20)
  {
    
    
    lastmove=millis();
    panLoop.m_pos += scanincrement;
    if ((panLoop.m_pos>=PIXY_RCS_MAX_POS)||(panLoop.m_pos<=PIXY_RCS_MIN_POS))
    { 
      tiltLoop.m_pos = random(PIXY_RCS_MAX_POS*0.6,PIXY_RCS_MAX_POS);
      scanincrement = -scanincrement;
      cal = cal+1;
      Serial.println("cal =");
      Serial.print(cal);
      if ( cal > 2) 
      {
         spinCounterClockwise(5);
        cal =0;
        }
          
        }
        delay (random(50,100));
      }
      
      pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
           
    }
int oldX,oldY,oldSignature;
int Trackblock(int Blockcount)
{ int trackedblocks = 0;
  long maxsize = 0;
  Serial.print("blocks=");
  Serial.print(Blockcount);
  for (int i =0; i<Blockcount; i++)
  { 
    if((oldSignature ==0) || (pixy.blocks[i].signature == oldSignature))
    { long newsize = pixy.blocks[i].height*pixy.blocks[i].width;
        if (newsize>maxsize)
          {
            trackedblocks = i;
            maxsize = newsize;
          }
     }
   }  

    int32_t panError = X_CENTER-pixy.blocks[0].x;
    int32_t tiltError = pixy.blocks[0].y-Y_CENTER;
    
    panLoop.update(panError);
    tiltLoop.update(tiltError);
    oldX = pixy.blocks[trackedblocks].x;
    oldY = pixy.blocks[trackedblocks].y;
    oldSignature = pixy.blocks[trackedblocks].signature;
    return trackedblocks;
  }

  int32_t size = 0;
void FollowBlock(int trackedBlock)
{
size = pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height;

if(panLoop.m_pos > 200 && panLoop.m_pos < 700 && size > 1600 )
{
 
  Serial.println("botstop ok");
   allStop();
   if(size > 3600)
   {
    backwardMovement(1);
    Serial.println("bot reverse");
    }
  }

 else if(panLoop.m_pos > 700 && size > 1600)
 {
   leftMovement(1);
   Serial.println("move left");
  }

  else if(panLoop.m_pos < 200 && size > 1600)
 {
   rightMovement(1);
   Serial.println("move right");
  }
  else if( size < 1600)
 {
   forwardMovement(1);
   Serial.println("bot forward");
  }
 
Serial.println("size = ");
Serial.print(size);
size -= size >> 3;
Serial.println("size new =");
Serial.print(size);
}

void allStop() {
 analogWrite(pwmA, 0);
 analogWrite(pwmB, 0);
 analogWrite(pwmC, 0);
}

// ****************************************************
// Sets the PWM motor values
// RETURNS: none
// ****************************************************
void commandMotors() {
 analogWrite(pwmA, motorA.pulse);
 analogWrite(pwmB, motorB.pulse);
 analogWrite(pwmC, motorC.pulse);
}

// ****************************************************
// Forward motor movement
// RETURNS: none
// ****************************************************
void forwardMovement(int a) {  
  // Set motor directions
  digitalWrite(dirA, HIGH); digitalWrite(dirB, HIGH); digitalWrite(dirC, LOW);
  
  // Ramp up the appropriate motors
  for (int i = 0; i < a; i++)
  { motorA.pulse = 0; motorB.pulse = 100; motorC.pulse = 100; commandMotors(); delay(25); }  
  //allStop();
}

// ****************************************************
// Backward motor movement
// RETURNS: none
// ****************************************************
void backwardMovement(int a) {  
  // Set motor directions
  digitalWrite(dirA, HIGH); digitalWrite(dirB, LOW); digitalWrite(dirC, HIGH);
  
  // Ramp up the appropriate motors
  for (int i = 0; i < a; i++)
  { motorA.pulse = 0; motorB.pulse =100 ; motorC.pulse = 100; commandMotors(); delay(25); }
  //allStop();
}

// ****************************************************
// Right motor movement
// RETURNS: none
// ****************************************************
void rightMovement (int a) {  
  // Set motor directions
  digitalWrite(dirA, LOW); digitalWrite(dirB, HIGH); digitalWrite(dirC, HIGH);
  
  // Ramp up the appropriate motors
  for (int i = 0; i < a; i++)
  { motorA.pulse = 50; motorB.pulse = 30  ; motorC.pulse =  30; commandMotors(); delay(25); }
  //allStop();
}

// ****************************************************
// Left motor movement
// RETURNS: none
// ****************************************************
void leftMovement (int a) {  
  // Set motor directions
  digitalWrite(dirA, HIGH); digitalWrite(dirB, LOW); digitalWrite(dirC, LOW);
  
  // Ramp up the appropriate motors
  for (int i = 0; i < a; i++)
  { motorA.pulse = 50; motorB.pulse = 30 ; motorC.pulse = 30; commandMotors(); delay(25); }
  //allStop();
}

// ****************************************************
// Spin Clowise motor movement
// RETURNS: none
// ****************************************************
void spinClockwise(int a) {  
  // Set motor directions
  digitalWrite(dirA, HIGH); digitalWrite(dirB, HIGH); digitalWrite(dirC, HIGH);
  
  // Ramp up the appropriate motors
  for (int i = 0; i < a; i++)
  { motorA.pulse = 100; motorB.pulse = 100; motorC.pulse = 100; commandMotors(); delay(25); }
  //allStop();
}

// ****************************************************
// Spin Counter Clockwise motor movement
// RETURNS: none
// ****************************************************
void spinCounterClockwise (int a) {
  // Set motor directions
  digitalWrite(dirA, LOW); digitalWrite(dirB, LOW); digitalWrite(dirC, LOW);
  
  // Ramp up the appropriate motors
  for (int i = 0; i < a; i++)
  { motorA.pulse = 100; motorB.pulse = 100; motorC.pulse = 100; commandMotors(); delay(25); }
  //allStop();
}
