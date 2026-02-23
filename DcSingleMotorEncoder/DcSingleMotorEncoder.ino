#include "Encoder.h"
#include "DC_Motor_Encoder.h"

#include "pinout.h"

#define SERIAL_TO_PC_BAUDRATE 115200

int tops_codeur = 440;
float consigne_vitesse = 0.0;
float kp_factor = 15.0;
float ki_factor = 5.0;
float kd_factor = 0.0;

Encoder objet_codeur = Encoder(encoderA_mot1, encoderB_mot1, tops_codeur, true);
Encoder* Encoder = &objet_codeur;

DC_Motor_Encoder moteur = DC_Motor_Encoder(mot1_plus, mot1_moins, mot1_pwm, mot1_enable, Encoder, kp_factor, ki_factor, kd_factor);

long compteur = 0;
int index = 0;
float consignes[8] = {0.5, 0.8, 3.0, 4.2, 0.0, -0.7, -4.0, 0.5};

void setup ()
{
  //Initialize Serial Monitor
  Serial.begin(SERIAL_TO_PC_BAUDRATE);

  // On initialise le Encoder
  Serial.println("Init Encoder...");
  Encoder->init_codeur([]{Encoder->tic_detector();});
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
  moteur.controlMotorSpeed(consigne_vitesse);

  // En alternative, on peut commander le moteur avec un PWM fixé
  //  moteur.moveMotor(150);

  // Pour afficher les informations du moteur
  moteur.displayStatus();

  delay(20);

  // En alternative, utiliser une commande qui permet de faire tourner le moteur pendant un nombre de révolutions fixé
  // while (!moteur.hasFinishedRevolutionsNonBlocking(2))
  // {
	//   moteur.controlMotorSpeed(1.5);
  //   moteur.displayStatus();
  // }
  // moteur.stopMotor();
  
  // delay(2000);

}
