#include "Encoder.h"
#include "DC_Motor_Encoder.h"

#include "pinout.h"

#define SERIAL_TO_PC_BAUDRATE 115200

int tops_codeur = 635;
float consigne_vitesse = 0.0;
float kp_factor = 25.0;
float ki_factor = 0.0;
float kd_factor = 0.0;


Encoder objet_codeur1 = Encoder(encoderA_mot_PAMI_1, encoderB_mot_PAMI_1, tops_codeur, encoder_mot_PAMI_1_inversion);
Encoder* Encoder1 = &objet_codeur1;
Encoder objet_codeur2 = Encoder(encoderA_mot_PAMI_2, encoderB_mot_PAMI_2, tops_codeur, encoder_mot_PAMI_2_inversion);
Encoder* Encoder2 = &objet_codeur2;

DC_Motor_Encoder moteur1 = DC_Motor_Encoder(mot_PAMI_1_plus, mot_PAMI_1_moins, mot_PAMI_1_pwm, mot_PAMI_1_enable, Encoder1, kp_factor, ki_factor, kd_factor);
DC_Motor_Encoder moteur2 = DC_Motor_Encoder(mot_PAMI_2_plus, mot_PAMI_2_moins, mot_PAMI_2_pwm, mot_PAMI_2_enable, Encoder2, kp_factor, ki_factor, kd_factor);

long compteur = 0;
int index = 0;
float consignes[8] = {-0.5, -0.8, 3.0, 2.2, 0.0, -0.7, -1.0, 0.5};

void setup ()
{
  //Initialize Serial Monitor
  Serial.begin(SERIAL_TO_PC_BAUDRATE);

  // On initialise le Encoder
  Serial.println("Init Encoder...");

  Encoder1->init_codeur([]{Encoder1->tic_detector();});
  Encoder2->init_codeur([]{Encoder2->tic_detector();});

  // On initialise les valeurs limites du moteur (à définir sur chaque moteur!):

  moteur1.setMotorMaxSpeed(5.5);
  moteur1.setMotorMaxAcceleration(2.5);
  moteur2.setMotorMaxSpeed(5.5);
  moteur2.setMotorMaxAcceleration(2.5);
}


void loop()
{
  // Cet exemple permet de faire tourner deux moteurs d'un certain nombre de tours, avec une vitesse fixée
  // La fonction est non-bloquante, c'est à dire qu'on peut faire d'autres actions pendant que les moteurs tournent
  // (à l'intérieur de la boucle while)

  float nbRevs = -5;
  float speed_setpoint = 2.0;

  moteur1.startMovementNonBlocking(nbRevs, speed_setpoint);
  moteur2.startMovementNonBlocking(nbRevs, speed_setpoint);

  while (!moteur1.isMovementNonBlockingFinished() && !moteur2.isMovementNonBlockingFinished())
  {
    moteur1.updateMovementNonBlocking();
    moteur2.updateMovementNonBlocking();
    // moteur1.displayStatus();
    delay(20);
  }

  moteur1.stopMotor();
  moteur2.stopMotor();

  delay(3000);
  
  nbRevs = 5;
  speed_setpoint = 2.0;

  moteur1.startMovementNonBlocking(nbRevs, speed_setpoint);
  moteur2.startMovementNonBlocking(nbRevs, speed_setpoint);

  while (!moteur1.isMovementNonBlockingFinished() && !moteur2.isMovementNonBlockingFinished())
  {
    moteur1.updateMovementNonBlocking();
    moteur2.updateMovementNonBlocking();
    // moteur1.displayStatus();
    delay(20);
  }

  moteur1.stopMotor();
  moteur2.stopMotor();

  delay(3000);  

}
