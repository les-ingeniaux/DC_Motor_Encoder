#ifndef pinout_h
#define pinout_h

#include "Arduino.h" 

constexpr int  mot4_plus = 35;
constexpr int  mot4_moins = 34;
constexpr int  mot4_pwm = 11;
constexpr int  mot4_enable = 38;
constexpr int  encoderA_mot4 = 19;
constexpr int  encoderB_mot4 = 51;

//moteur3_
constexpr int  mot3_plus = 36;
constexpr int  mot3_moins = 37;
constexpr int  mot3_pwm = 12;
constexpr int  mot3_enable = 38;
constexpr int  encoderA_mot3 = 3;
constexpr int  encoderB_mot3 = 49;

//moteur1_
constexpr int  mot1_plus = 32;
constexpr int  mot1_moins = 33;
constexpr int  mot1_pwm = 9;
constexpr int  mot1_enable = 23;
constexpr int  encoderA_mot1 = 2;
constexpr int  encoderB_mot1 = 48;

//moteur2_
constexpr int  mot2_plus = 31;
constexpr int  mot2_moins = 30;
constexpr int  mot2_pwm = 8;
constexpr int  mot2_enable = 23;
constexpr int  encoderA_mot2 = 18;
constexpr int  encoderB_mot2 = 50;
//STBY
//constexpr int  stby = 25;

//moteur PAMI 2_
constexpr int  mot_PAMI_2_plus = 8;
constexpr int  mot_PAMI_2_moins = 12;
constexpr int  mot_PAMI_2_pwm = 11;
constexpr int  mot_PAMI_2_enable = false;
constexpr int  encoderA_mot_PAMI_2 = 3;
constexpr int  encoderB_mot_PAMI_2 = 5;
//STBY

//moteur PAMI 1_
constexpr int  mot_PAMI_1_plus = 4;
constexpr int  mot_PAMI_1_moins = 7;
constexpr int  mot_PAMI_1_pwm = 9;
constexpr int  mot_PAMI_1_enable = false;
constexpr int  encoderA_mot_PAMI_1 = 2;
constexpr int  encoderB_mot_PAMI_1 = 6;

#endif