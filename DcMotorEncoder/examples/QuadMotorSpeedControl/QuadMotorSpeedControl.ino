#include "Encoder.h"
#include "DC_Motor_Encoder.h"

#include "pinout.h"

#define SERIAL_TO_PC_BAUDRATE 115200

int tops_codeur = 440;
float consigne_vitesse = 0.0;
float kp_factor = 25.0;
float ki_factor = 50.0;
float kd_factor = 0.0;

Encoder objet_codeur1 = Encoder(encoderA_mot1, encoderB_mot1, tops_codeur, encoder_mot1_inversion);
Encoder* Encoder1 = &objet_codeur1;
Encoder objet_codeur2 = Encoder(encoderA_mot2, encoderB_mot2, tops_codeur, encoder_mot2_inversion);
Encoder* Encoder2 = &objet_codeur2;
Encoder objet_codeur3 = Encoder(encoderA_mot3, encoderB_mot3, tops_codeur, encoder_mot3_inversion);
Encoder* Encoder3 = &objet_codeur3;
Encoder objet_codeur4 = Encoder(encoderA_mot4, encoderB_mot4, tops_codeur, encoder_mot4_inversion);
Encoder* Encoder4 = &objet_codeur4;

DC_Motor_Encoder moteur1 = DC_Motor_Encoder(mot1_plus, mot1_moins, mot1_pwm, mot1_enable, Encoder1, kp_factor, ki_factor, kd_factor);
DC_Motor_Encoder moteur2 = DC_Motor_Encoder(mot2_plus, mot2_moins, mot2_pwm, mot2_enable, Encoder2, kp_factor, ki_factor, kd_factor);
DC_Motor_Encoder moteur3 = DC_Motor_Encoder(mot3_plus, mot3_moins, mot3_pwm, mot3_enable, Encoder3, kp_factor, ki_factor, kd_factor);
DC_Motor_Encoder moteur4 = DC_Motor_Encoder(mot4_plus, mot4_moins, mot4_pwm, mot4_enable, Encoder4, kp_factor, ki_factor, kd_factor);

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
  Encoder2->init_codeur([]{Encoder2->tic_detector();});
  Encoder3->init_codeur([]{Encoder3->tic_detector();});
  Encoder4->init_codeur([]{Encoder4->tic_detector();});

  // On initialise les valeurs limites du moteur (à définir sur chaque moteur!):
  moteur1.setMotorMaxSpeed(5.5);
  moteur1.setMotorMaxAcceleration(2.5);
  moteur2.setMotorMaxSpeed(5.5);
  moteur2.setMotorMaxAcceleration(2.5);
  moteur3.setMotorMaxSpeed(5.5);
  moteur3.setMotorMaxAcceleration(2.5);
  moteur4.setMotorMaxSpeed(5.5);
  moteur4.setMotorMaxAcceleration(2.5);
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
  
  // La consigne de vitesse est appliquée
  moteur1.controlMotorSpeed(consigne_vitesse);
  moteur2.controlMotorSpeed(consigne_vitesse);
  moteur3.controlMotorSpeed(consigne_vitesse);
  moteur4.controlMotorSpeed(consigne_vitesse);
  // moteur4.moveMotor(45);
  moteur4.displayStatus();

  delay(20);

}
