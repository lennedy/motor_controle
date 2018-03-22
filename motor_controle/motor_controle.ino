/******************************************************************
 * PID Simple Example (Augmented with Processing.org Communication)
 * Version 0.2
 * by Marcelo Moraes
 * License: Free
 * April 2013
 * Sorocaba - São Paulo - Brazil
 ******************************************************************/

#include "contr_motor.h"

#define PRINT_MOTOR1

// Library definition
//#include <PID_v1.h>
#include <math.h>

#define PINO_ENCODER1_1 2 //interrupcao
#define PINO_ENCODER1_2 5 //
#define PINO_PWM_SETPOINT_1 A0
#define PINO_DIRECAO_SETPOINT_1 12
#define PINO_PWM_OUTPUT_1 9
#define PINO_DIRECAO_OUTPUT_1 8

#define PINO_ENCODER2_1 3 //interrupcao
#define PINO_ENCODER2_2 4 //
#define PINO_PWM_SETPOINT_2 A1
#define PINO_DIRECAO_SETPOINT_2 11
#define PINO_PWM_OUTPUT_2 6
#define PINO_DIRECAO_OUTPUT_2 7


Encoder encoder1(PINO_ENCODER1_1,PINO_ENCODER1_2);
contr_motor Motor1(encoder1);

Encoder encoder2(PINO_ENCODER2_1,PINO_ENCODER2_2);
contr_motor Motor2(encoder2);

// TimestampLED_PIDcontroller_2.ino:156:32: error: sufixo "Output" inválido em constante flutuanteLED_PIDcontroller_2.ino:156:32: error: sufixo "Output" inválido em constante flutuante
unsigned long serialTime; //this will help us know when to talk with processing
unsigned long now = 0; //This variable is used to keep track of time
unsigned long lastMessage = 0; //This keeps track of when our loop last spoke to serial

void encoder1_interrupt(){
  Motor1.calculapulso();
}

void encoder2_interrupt(){
  Motor2.calculapulso();
}

void setup()
{
  Serial.begin(9600); //Start a serial session
  lastMessage = millis(); // timestamp
  Motor1.setPinos(PINO_PWM_SETPOINT_1, PINO_DIRECAO_SETPOINT_1, PINO_PWM_OUTPUT_1, PINO_DIRECAO_OUTPUT_1);
  Motor1.config();

  Motor2.setPinos(PINO_PWM_SETPOINT_2, PINO_DIRECAO_SETPOINT_2, PINO_PWM_OUTPUT_2, PINO_DIRECAO_OUTPUT_2);
  Motor2.config();

  Motor2.setTuningsPID(2,0,0);
  Motor1.setTuningsPID(2,0,0);  
  attachInterrupt(0, encoder1_interrupt, CHANGE);
  attachInterrupt(1, encoder2_interrupt, CHANGE);
}


void loop(){

  Motor1.executa();
  Motor2.executa();
  
  now = millis(); // Keep track of time
  
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }  
}




/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output  
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while(Serial.available()&&index<26)
  {
    if(index==0) Auto_Man = Serial.read();
    else if(index==1) Direct_Reverse = Serial.read();
    else foo.asBytes[index-2] = Serial.read();
    index++;
  } 
  
  // if the information we got was in the correct format, 
  // read it into the system
  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
  {
    #ifdef PRINT_MOTOR1
    Motor1.setSetPoint(double(foo.asFloat[0]));
    #else
    Motor2.setSetPoint(double(foo.asFloat[0]));
    #endif
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
                                          //   value of "Input"  in most cases (as 
                                          //   in this one) this is not needed.
    if(Auto_Man==0)                       // * only change the output if we are in 
    {                                     //   manual mode.  otherwise we'll get an
      #ifdef PRINT_MOTOR1
      Motor1.setOutput(double(foo.asFloat[2]));      //   output blip, then the controller will 
      #else
      Motor2.setOutput(double(foo.asFloat[2]));      //   output blip, then the controller will
      #endif
    }                                     //   overwrite.
    
    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    #ifdef PRINT_MOTOR1
    Motor1.setTuningsPID(p, i, d);            //
    #else
    Motor2.setTuningsPID(p, i, d);            //
    #endif
    
    if(Auto_Man==0) {
      #ifdef PRINT_MOTOR1
      Motor1.setModePID(MANUAL);// * set the controller mode
      #else
      Motor1.setModePID(MANUAL);// * set the controller mode
      #endif
    }
    else {
      #ifdef PRINT_MOTOR1
      Motor1.setModePID(AUTOMATIC);             //
      #else
      Motor2.setModePID(AUTOMATIC);             //
      #endif
    }
    
    if(Direct_Reverse==0) {
      #ifdef PRINT_MOTOR1
      Motor1.setControllerDirectionPID(DIRECT);// * set the controller Direction
      #else
      Motor2.setControllerDirectionPID(DIRECT);// * set the controller Direction
      #endif
    }
    else{ 
      #ifdef PRINT_MOTOR1
      Motor1.setControllerDirectionPID(REVERSE);          //
      #else
      Motor2.setControllerDirectionPID(REVERSE);          //
      #endif
    }
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  #ifdef PRINT_MOTOR1
  Serial.print("PID ");
  Serial.print(1024*(Motor1.getSetPoint()/100.0));   
  Serial.print(" ");
  Serial.print(1024*(Motor1.getInput()/100.0));   
 //Serial.print((Motor1.getInput()));
  Serial.print(" ");
  Serial.print(((Motor1.getOutput()/100.0)*127.0+127));   
//  Serial.print(Output);   
  Serial.print(" ");
  Serial.print(Motor1.getKpPID());   
  Serial.print(" ");
  Serial.print(Motor1.getKiPID());   
  Serial.print(" ");
  Serial.print(Motor1.getKdPID());   
  Serial.print(" ");
  if(Motor1.getModePID()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(Motor1.getDirectionPID()==DIRECT) Serial.println("Direct");
  else Serial.println("Reverse");
  #else
  Serial.print("PID ");
  Serial.print(1024*(Motor2.getSetPoint()/100.0));   
  Serial.print(" ");
  Serial.print(1024*(Motor2.getInput()/100.0));   
 //Serial.print((Motor2.getInput()));
  Serial.print(" ");
  Serial.print(((Motor2.getOutput()/100.0)*127.0+127));   
//  Serial.print(Output);   
  Serial.print(" ");
  Serial.print(Motor2.getKpPID());   
  Serial.print(" ");
  Serial.print(Motor2.getKiPID());   
  Serial.print(" ");
  Serial.print(Motor2.getKdPID());   
  Serial.print(" ");
  if(Motor2.getModePID()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(Motor2.getDirectionPID()==DIRECT) Serial.println("Direct");
  else Serial.println("Reverse");  
  #endif
}
