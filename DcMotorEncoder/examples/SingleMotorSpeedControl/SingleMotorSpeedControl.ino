#include "Encoder.h"
#include "DC_Motor_Encoder.h"

#include "pinout.h"

#define SERIAL_TO_PC_BAUDRATE 115200

int tops_codeur = 634;
float consigne_vitesse = 0.0;
float kp_factor = 15.0;
float ki_factor = 85.0;
float kd_factor = 0.0;

Encoder objet_codeur1 = Encoder(encoderA_mot_PAMI_1, encoderB_mot_PAMI_1, tops_codeur, encoder_mot_PAMI_1_inversion);
Encoder* Encoder1 = &objet_codeur1;

DC_Motor_Encoder moteur1 = DC_Motor_Encoder(mot_PAMI_1_plus, mot_PAMI_1_moins, mot_PAMI_1_pwm, mot_PAMI_1_enable, Encoder1, kp_factor, ki_factor, kd_factor);

long compteur = 0;
int index = 0;
float consignes[8] = {0.5, 0.8, 3.0, 2.2, 0.0, -0.7, -1.0, 0.5};

void setup ()
{
  //Initialize Serial Monitor
  Serial.begin(SERIAL_TO_PC_BAUDRATE);

  // On initialise le Encoder
  Serial.println("Init Encoder...");
  Encoder1->init_codeur([]{Encoder1->tic_detector();});

  // On initialise les valeurs limites du moteur (à définir sur chaque moteur!):
  moteur1.setMotorMaxSpeed(5.5);
  moteur1.setMotorMaxAcceleration(2.5);
}


void loop()
{
  // Cette boucle permet de choisir une nouvelle consigne de vitesse toutes les 3 secondes
  if (millis() - compteur > 3000)
  {
    compteur = millis();
    index = (index + 1) % 8;
    consigne_vitesse = consignes[index];
    Serial.println(consigne_vitesse);
  }
  
  La consigne de vitesse est appliquée
  moteur.controlMotorSpeed(consigne_vitesse);

  delay(20);

}
