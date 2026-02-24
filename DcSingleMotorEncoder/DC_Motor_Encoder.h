#ifndef DC_Motor_Encoder_h
#define DC_Motor_Encoder_h

#include "Arduino.h" 
#include "Encoder.h"

class DC_Motor_Encoder 
{
public:
	DC_Motor_Encoder(int pinA, int pinB, int pinPWM, int pinEnable, Encoder* codeur_moteur, float kp_factor = 70.0, float ki_factor = 10.0, float kd_factor = 15.0);

	void stopMotor();
	void releaseMotor();
	void moveMotor(int power);
	void setMotorMaxSpeed(float vitesse);
	void setMotorMaxAcceleration(float acceleration);
	int getMotorTops();
	void reset_encoder();
	int getMotorPWM();
	float getMotorSpeed();
	float getMotorSpeedSetpoint();
	float getMotorSpeedError();
	float filterSpeedSetpoint(float speed_setpoint);
	void controlMotorSpeed(float speed_setpoint);
	void doRevolutionsBlocking(int number_of_revs, int PWM=100);
	void doRevolutionsSpeedBlocking(int number_of_revs, float speed_setpoint=1.0);
	bool hasFinishedRevolutionsNonBlocking(int number_of_revs);
	void displayStatus();
	
private:
	int _pinA;
	int _pinB;
	int _pinPWM;
	int _pinEnable;
	float _min_vitesse = 0.1; // rps
	float _max_vitesse = 6.5; // rps - Valeur par défaut à remplacer par une valeur spécifique à votre moteur!
	float _max_acceleration = 2.5; // rps² - Valeur par défaut à remplacer par une valeur spécifique à votre application!

	Encoder* _codeur_moteur;

	int _commande_PWM;
	float _raw_speed_setpoint;
	float _speed_setpoint;
	float _prev_speed_setpoint;
	float _max_new_speed_setpoint;
	float _min_new_speed_setpoint;
	float _integral_error;
	float _error;
	float _prev_error;
	long int _mot_timer_ms;
	long int _prev_mot_timer_ms;
	long int _mot_timer_diff_ms;

	int _max_PWM = 255;
	float _kp_factor;
	float _ki_factor;
	float _kd_factor;

	bool _isFinished;
};

#endif