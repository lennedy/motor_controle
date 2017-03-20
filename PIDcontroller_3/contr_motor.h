

#include <Arduino.h>
#include <PID_v1.h>

class contr_motor{

  double Setpoint, Input, Output; // these are just variables for storing values
  
  int inputPin = 0; // Photo resistor input
  int outputPin = 3; // LED output
  int signalPin = 4; // Pin to inform the way of moviment
  int setPointPin = 1; // Potentiometer input
  
  
  // Tuning parameters
  float Kp=3.0; //Initial Proportional Gain 
  float Ki=2.9; //Initial Integral Gain 
  float Kd=0; //Initial Differential Gain 
  
  // Specify the links and initial tuning parameters
  PID myPID;
  
  const int sampleRate = 1; // Variable that determines how fast our PID loop runs
  
  public:
  contr_motor();
  void config();
  void executa();
  
  inline double ler_entrada(){return ler_analogico(inputPin);}
  inline double ler_setPoint(){return ler_analogico(setPointPin);}
  inline double ler_analogico(const int pino){return 100*(analogRead(pino)/1023.0);}
  void escr_analogico(int pino, double valor);
  
  inline void setModePID(int mode){myPID.SetMode(mode);}
  inline void setControllerDirectionPID(int direcao){myPID.SetControllerDirection(direcao);}
  inline void setTuningsPID(float p, float i, float d){myPID.SetTunings(p, i, d);}
  inline float getKpPID(){return myPID.GetKp();}
  inline float getKiPID(){return myPID.GetKi();}
  inline float getKdPID(){return myPID.GetKd();}
  inline int getModePID(){myPID.GetMode();}
  inline int getDirectionPID(){myPID.GetDirection();}
  
  inline void setSetPoint(double sp){Setpoint = sp;}
  inline void setOutput(double op){Output = op;}  
  inline double getSetPoint(){return Setpoint;}
  inline double getOutput(){return Output;}
  inline double getInput(){return Input;}
};
