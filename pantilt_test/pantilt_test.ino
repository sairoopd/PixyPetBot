//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a simple tracking demo that uses the pan/tilt unit.  For
// more information, go here:
//
// http://cmucam.org/projects/cmucam5/wiki/Run_the_Pantilt_Demo
//

#include <SPI.h>  
#include <Pixy.h>

Pixy pixy;

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)
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



void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");
  
  pixy.init();
}
uint32_t lastblocktime = 0;
void loop()
{ 
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  //int32_t panError, tiltError;
  //Serial.print("i am here \n");
  blocks = pixy.getBlocks();
  //Serial.print("i am here 1 \n");
  Serial.println("Block=");
  Serial.print(blocks);
  
  if (blocks)
  { 
    int trackedblocks = Trackblock(blocks);
    Serial.print("follow \n");
    FollowBlock(trackedblocks);
    lastblocktime = millis();
  }
  else if (millis() - lastblocktime > 100)
  {
    Serial.print("stop bot \n");
    scanforblocks();
    Serial.print("stop \n");
    }  
}
int oldX,oldY,oldSignature;
int Trackblock(int Blockcount)
{ int trackedblocks = 0;
  long maxsize = 0;
  Serial.print("blocks=");
  Serial.print(Blockcount,"\n");
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
    
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
    oldX = pixy.blocks[trackedblocks].x;
    oldY = pixy.blocks[trackedblocks].y;
    Serial.println("panLoop = ");
    Serial.print(panLoop.m_pos);
    oldSignature = pixy.blocks[trackedblocks].signature;
    Serial.print("\n trackedblocks");
    Serial.print(trackedblocks);
    return trackedblocks;
  

  }
int scanincrement = (PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS)/150;
uint32_t lastmove = 0; 
void scanforblocks()
{
  if (millis()- lastmove > 20)
  {
    
    
    lastmove=millis();
    panLoop.m_pos += scanincrement;
    if ((panLoop.m_pos>=PIXY_RCS_MAX_POS)||(panLoop.m_pos<=PIXY_RCS_MIN_POS))
    { 
      tiltLoop.m_pos = random(PIXY_RCS_MAX_POS*0.6,PIXY_RCS_MAX_POS);
      scanincrement = -scanincrement;
     cal = cal+1;
      if (scanincrement <0 && cal > 4) 
      {
        Serial.print("rotate 180 \n");
        cal =0;
        }
        else 
        {
          Serial.print("antirotate \n");
          
        }
        delay (random(50,100));
      }
      Serial.println("panLoop =");
      Serial.println(panLoop.m_pos);
      Serial.println("tiltLoop = ");
      Serial.println(tiltLoop.m_pos);
      pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
      
      
    }
  }
int32_t size = 400;
void FollowBlock(int trackedBlock)
{
int32_t followError = PIXY_RCS_CENTER_POS - panLoop.m_pos; // How far off-center are we looking now?
// Size is the area of the object.
// We keep a running average of the last 8.
size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height;
Serial.println("size = ");
Serial.print(size);
size -= size >> 3;
Serial.println("size new =");
Serial.print(size);
// Forward speed decreases as we approach the object (size is larger)
int forwardSpeed = constrain(400 - (size/256), -100, 400);
// Steering differential is proportional to the error times the forward speed
int32_t differential = (followError + (followError * forwardSpeed))>>8;
// Adjust the left and right speeds by the steering differential.
int leftSpeed = constrain(forwardSpeed + differential, -400, 400);
int rightSpeed = constrain(forwardSpeed - differential, -400, 400);
// And set the motor speeds
//motors.setLeftSpeed(leftSpeed);
//motors.setRightSpeed(rightSpeed);
}

