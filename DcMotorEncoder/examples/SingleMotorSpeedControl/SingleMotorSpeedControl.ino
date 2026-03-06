#include "Encoder.h"
#include "DC_Motor_Encoder.h"

#include "pinout.h"

#define SERIAL_TO_PC_BAUDRATE 115200

int tops_codeur = 440;
float consigne_vitesse = 0.0;
float kp_factor = 30.0;
float ki_factor = 50.0;
float kd_factor = 0.0;


Encoder objet_codeur4 = Encoder(encoderA_mot4, encoderB_mot4, tops_codeur, encoder_mot4_inversion);
Encoder* Encoder4 = &objet_codeur4;

DC_Motor_Encoder moteur4 = DC_Motor_Encoder(mot4_plus, mot4_moins, mot4_pwm, mot4_enable, Encoder4, kp_factor, ki_factor, kd_factor);

long compteur = 0;
int index = 0;
float consignes[8] = {-0.5, -0.8, 3.0, 2.2, 0.0, -0.7, -1.0, 0.5};

void setup ()
{
  //Initialize Serial Monitor
  Serial.begin(SERIAL_TO_PC_BAUDRATE);

  // On initialise le Encoder
  Serial.println("Init Encoder...");

  Encoder4->init_codeur([]{Encoder4->tic_detector();});

  // On initialise les valeurs limites du moteur (à définir sur chaque moteur!):

  moteur4.setMotorMaxSpeed(5.5);
  moteur4.setMotorMaxAcceleration(2.5);
}


void loop()
{
  // Cette boucle permet de choisir une nouvelle consigne de vitesse toutes les 3 secondes
  if (millis() - compteur > 6000)
  {
    compteur = millis();
    index = (index + 1) % 8;
    consigne_vitesse = consignes[index];
    Serial.println(consigne_vitesse);
  }
  
  // La consigne de vitesse est appliquée

  moteur4.controlMotorSpeed(consigne_vitesse);
  // moteur4.moveMotor(35);
  moteur4.displayStatus();

  delay(20);

}
