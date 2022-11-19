#define led 7
#define RX 2
#define TX 3

//code pin 1:0000 


word octet_recu=30;

#include <SoftwareSerial.h>
#include <DualMAX14870MotorShield.h>

DualMAX14870MotorShield motors;
SoftwareSerial bluetooth(RX, TX); // RX | TX
// RX arduino <--- TX cible
// TX arduino ---> RX cible

int initialSpeed=0;
int spdleft=0;
int spdright=0;

void recevoir()
{
  if (bluetooth.available())
  {
    octet_recu = bluetooth.read();
    
  }
  
}

void setup()

{
  Serial.begin(9600); // vitesse serial monitor

  bluetooth.begin(9600);  // vitesse software serial
  pinMode(led, OUTPUT);
}

void loop()
{
  
  recevoir();
  // appel de la procédure recevoir

  if (octet_recu==100) {
    initialSpeed=90;
    }
    
  
  octet_recu=max(0,octet_recu);
  octet_recu=min(octet_recu,60);
  spdleft=initialSpeed+((30-octet_recu)*2);
  spdright=initialSpeed+((octet_recu-30)*2);
  

  
  motors.enableDrivers();

   //run M1 motor with positive speed

 
 motors.setM1Speed(spdleft);
 motors.setM2Speed(spdright);
 
 
}
