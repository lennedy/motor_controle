#include <Arduino.h>

#define VEL_MAX_ENCODER_POR_MILI_SEGUNDOS 1.9

class Encoder{
  
  const byte Encoder_C1;
  const byte Encoder_C2;
  byte Encoder_C1Last;
  long duracao;
  boolean Direcao=true;
  double velocidade=0; //velocidade em pusos/segundo
  unsigned long tempo_ant=0;
  long duracao_ant=0;
  const int amostragem_vel=100; //tempo de amostragem em milesegundos 
  
  public:
  inline void config(){pinMode(Encoder_C2, INPUT);}
  Encoder(byte pino_c1, byte pino_c2);
  void calculapulso();
  
  inline void resetDuracao(){duracao=0;}
  inline long getDuracao(){return duracao;}
  inline boolean getDirecao(){return Direcao;}
  
  inline double getVelocidade(){return velocidade;}
  
  void calculaVelocidade();

};
