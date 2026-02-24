#include "Arduino.h"
#include "DC_Motor_Encoder.h"

DC_Motor_Encoder::DC_Motor_Encoder(int pinA, int pinB, int pinPWM, int pinEnable, Encoder* codeur_moteur, float kp_factor, float ki_factor, float kd_factor) 
{
	pinMode(pinA, OUTPUT);
	pinMode(pinB, OUTPUT);
	pinMode(pinPWM, OUTPUT);

	if (pinEnable != false)
		pinMode(pinEnable, OUTPUT);

	digitalWrite(pinA, LOW);
	digitalWrite(pinB, LOW);
	analogWrite(pinPWM, 0);
	if (pinEnable != false)
		digitalWrite(pinEnable, LOW);
	
	_pinA = pinA;
	_pinB = pinB;
	_pinPWM = pinPWM;
	_pinEnable = pinEnable;
	_codeur_moteur = codeur_moteur;

	_kp_factor = kp_factor;
	_ki_factor = ki_factor;
	_kd_factor = kd_factor;

	_speed_setpoint = 0.0;
	_prev_speed_setpoint = 0.0;
	_commande_PWM = 0;
	_error = 0.0;
	_prev_error = 0.0;
	_integral_error = 0.0;
	_prev_mot_timer_ms = 0;	
	_isFinished = true;
}

void DC_Motor_Encoder::setMotorMaxSpeed(float vitesse)
{
	_max_vitesse = vitesse;
}

void DC_Motor_Encoder::setMotorMaxAcceleration(float acceleration)
{
	_max_acceleration = acceleration;
}

int DC_Motor_Encoder::getMotorTops()
{
	return _codeur_moteur->get_encoder_tics();
}

void DC_Motor_Encoder::reset_encoder()
{
  _codeur_moteur->reset_codeur();
}

float DC_Motor_Encoder::getMotorSpeed()
{
	return _codeur_moteur->get_encoder_filtered_speed();
}

float DC_Motor_Encoder::getMotorSpeedSetpoint()
{
	return _speed_setpoint;
}


float DC_Motor_Encoder::getMotorSpeedError()
{
	return _error;
}


void DC_Motor_Encoder::stopMotor()
{
	digitalWrite(_pinA, HIGH);
	digitalWrite(_pinB, HIGH);

	// _speed_setpoint = 0.0;
	_integral_error = 0.0;
}

void DC_Motor_Encoder::releaseMotor()
{
	if (_pinEnable != false)
		digitalWrite(_pinEnable, LOW);
	digitalWrite(_pinA, LOW);
	digitalWrite(_pinB, LOW);

	// _speed_setpoint = 0.0;
	_integral_error = 0.0;
}

void DC_Motor_Encoder::moveMotor(int power)
{
	_commande_PWM = power;
	
	if (_pinEnable != false)
		digitalWrite(_pinEnable, HIGH);

	if (_commande_PWM > 0)
	{
		digitalWrite(_pinA, HIGH);
		digitalWrite(_pinB, LOW);
		analogWrite(_pinPWM, _commande_PWM);   
	}
	else if(_commande_PWM < 0)
	{
		digitalWrite(_pinA, LOW);
		digitalWrite(_pinB, HIGH);
		analogWrite(_pinPWM, -_commande_PWM);
	}
	else 
	{
		stopMotor();
	}

}

float DC_Motor_Encoder::filterSpeedSetpoint(float raw_speed_setpoint)
{
	_max_new_speed_setpoint = _prev_speed_setpoint + _max_acceleration / 1000.0 * _mot_timer_diff_ms;
	_min_new_speed_setpoint = _prev_speed_setpoint - _max_acceleration / 1000.0 * _mot_timer_diff_ms;
	float filtered_speed_setpoint = max(min(raw_speed_setpoint, _max_new_speed_setpoint ), _min_new_speed_setpoint);

	return filtered_speed_setpoint;
}

void DC_Motor_Encoder::controlMotorSpeed(float raw_speed_setpoint)
{
	_raw_speed_setpoint = raw_speed_setpoint;
	_mot_timer_ms = millis();
	_mot_timer_diff_ms = (_mot_timer_ms - _prev_mot_timer_ms);

	if (_mot_timer_diff_ms == 0)
		_mot_timer_diff_ms = 1; // Protection contre la division par zéro

	_speed_setpoint = filterSpeedSetpoint(_raw_speed_setpoint);

	int ffwd_PWM = (_speed_setpoint / _max_vitesse) * _max_PWM;

	if (abs(_speed_setpoint) < _min_vitesse)
	{
		_error = 0;
		_integral_error = 0;
		_prev_error = 0;

		_commande_PWM = 0;
	}
	else
	{
		if (abs(getMotorSpeed()) < _min_vitesse) // if speed is zero and the setpoint higher (when power is cutoff), to avoid saturating the integral 
		{
			_integral_error = 0;
		}

		_speed_setpoint = max(-_max_vitesse, min(_max_vitesse, _speed_setpoint));

		_error = (_speed_setpoint - getMotorSpeed());
		_integral_error = _integral_error + (_speed_setpoint - getMotorSpeed()) * _mot_timer_diff_ms / 1000.0;
		_commande_PWM = min(_max_PWM, max(-_max_PWM, ffwd_PWM + _error * _kp_factor + _integral_error * _ki_factor + (_error - _prev_error) / _mot_timer_diff_ms * _kd_factor * 1000.0 ));  
		_prev_error = _error;
	}
	
	moveMotor(_commande_PWM);
	_prev_mot_timer_ms = _mot_timer_ms;
	_prev_speed_setpoint = _speed_setpoint;

}

int DC_Motor_Encoder::getMotorPWM()
{
	return _commande_PWM;
}

void DC_Motor_Encoder::doRevolutionsBlocking(int number_of_revs, int PWM)
{
	bool isFinished = false;
	_integral_error = 0;
	reset_encoder();
	int avance = _codeur_moteur->get_tops_per_tour() * PWM / 1500; // paramètre pour corriger l'inertie à l'arrêt
	
	while (!isFinished)
	{
		moveMotor(PWM);
		if (abs(getMotorTops()) >= (number_of_revs * _codeur_moteur->get_tops_per_tour() - avance))
			isFinished = true;		
		displayStatus();
	}

	stopMotor();

}

void DC_Motor_Encoder::doRevolutionsSpeedBlocking(int number_of_revs, float speed_setpoint)
{
	bool isFinished = false;
	_integral_error = 0;
	reset_encoder();
	int avance = _codeur_moteur->get_tops_per_tour() * speed_setpoint / 100; // paramètre pour corriger l'inertie à l'arrêt
	
	while (!isFinished)
	{
		controlMotorSpeed(speed_setpoint);
		if (abs(getMotorTops()) >= (number_of_revs * _codeur_moteur->get_tops_per_tour() - avance))
			isFinished = true;		
		displayStatus();
	}

	stopMotor();
}

bool DC_Motor_Encoder::hasFinishedRevolutionsNonBlocking(int number_of_revs)
{
	// bool isFinished = false;

	if(_isFinished)
	{
		reset_encoder();
		_isFinished = false;
		_integral_error = 0;
	}

	if (abs(getMotorTops())  >= (number_of_revs * _codeur_moteur->get_tops_per_tour()))
	{
		_isFinished = true;
	}
	
	return _isFinished;
}

void DC_Motor_Encoder::displayStatus()
{
  String affichage_valeurs;

  affichage_valeurs.concat("Setpoint:");
  affichage_valeurs.concat(getMotorSpeedSetpoint()); 
  affichage_valeurs.concat("\tRaw Setpoint:");
  affichage_valeurs.concat(_raw_speed_setpoint); 
  affichage_valeurs.concat("\tMax new Setpoint:");
  affichage_valeurs.concat(_max_new_speed_setpoint); 
	affichage_valeurs.concat("\tMin new Setpoint:");
  affichage_valeurs.concat(_min_new_speed_setpoint); 
	affichage_valeurs.concat("\tTimer diff:");
  affichage_valeurs.concat(_mot_timer_diff_ms); 
  affichage_valeurs.concat("\tTops:");
  affichage_valeurs.concat(getMotorTops());  
  affichage_valeurs.concat("\tSpeed:");
  affichage_valeurs.concat(getMotorSpeed());  
  affichage_valeurs.concat("\tPWM:");
  affichage_valeurs.concat(getMotorPWM());

  Serial.println(affichage_valeurs);

}
