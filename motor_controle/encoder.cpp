#include "encoder.h"

Encoder::Encoder(byte pino_c1, byte pino_c2):
Encoder_C1(pino_c1), 
Encoder_C2(pino_c2)
{
  duracao=0;
}

void Encoder::calculapulso(){
  int Lstate = digitalRead(Encoder_C1);
  if ((Encoder_C1Last == LOW) && Lstate == HIGH){
    
    int val = digitalRead(Encoder_C2);
//    Serial.println(val);
    if (val == LOW && Direcao)    {
      Direcao = false; //Reverse
      
    }
    else if (val == HIGH && !Direcao)    {
           
      Direcao = true;  //Forward
    }
  }
  Encoder_C1Last = Lstate;
 
  if (!Direcao)  duracao--;
  else  duracao++;
}

void Encoder::calculaVelocidade(){
  
  unsigned long tempo_atual = millis();
  unsigned long timeChange = (tempo_atual - tempo_ant);
  if(timeChange>=amostragem_vel){
    velocidade = (duracao - duracao_ant)/((double)(timeChange));
    velocidade = 100*velocidade/VEL_MAX_ENCODER_POR_MILI_SEGUNDOS;
    //Serial.println(duracao);// - duracao_ant);
    duracao_ant = duracao;
    tempo_ant = tempo_atual;

  }
  
}
