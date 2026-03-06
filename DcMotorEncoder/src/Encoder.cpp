#include "Encoder.h"

Encoder::Encoder(unsigned int pinA, unsigned int pinB, int tops_per_tour, bool inv_sign, int edgeType)
{
  _total_pulses = 0;
  _pinA = pinA;
  _pinB = pinB;
  _inv_sign = inv_sign;
  _edgeType = edgeType;
  _tops_per_tour = tops_per_tour;

  _total_pulses_for_speed = 0;
  _prev_total_pulses_for_speed = 0;
}

void Encoder::init_codeur(void (*ISR_callback)(void))
{
  pinMode(_pinA, INPUT);
  pinMode(_pinB, INPUT);

  reset_codeur();

  if (_edgeType == RISING)
    attachInterrupt(digitalPinToInterrupt(_pinA), ISR_callback, RISING);
  if (_edgeType == FALLING)
    attachInterrupt(digitalPinToInterrupt(_pinA), ISR_callback, FALLING);  
}

void Encoder::reset_codeur()
{
  _total_pulses = 0;
}

void Encoder::tic_detector()
{
  if (!digitalRead(_pinB))
  {
    if (_inv_sign)
      _total_pulses--;
    else
      _total_pulses++;
  }
  else
  {
    if (_inv_sign)
      _total_pulses++;
    else
      _total_pulses--;    
  }
  
    
}

long Encoder::get_encoder_tics()
{
    noInterrupts();            // Disable interrupts
    long tics = _total_pulses; // Copy safely
    interrupts();              // Re-enable interrupts
    return tics;
}

int Encoder::get_tops_per_tour()
{
  return _tops_per_tour;
}

float Encoder::get_raw_encoder_speed()
{
  unsigned long now = micros();

  if (now - _encoder_timer < 5000) // 5 ms
    return _raw_encoder_speed;
      
  _prev_encoder_timer = _encoder_timer;
  _encoder_timer = micros();
  _encoder_timer_diff = _encoder_timer - _prev_encoder_timer;  
  _prev_raw_encoder_speed = _raw_encoder_speed;

  _total_pulses_for_speed = get_encoder_tics();

  if (_encoder_timer_diff > 0)
  {
    _raw_encoder_speed = seconds_to_micro * (_total_pulses_for_speed - _prev_total_pulses_for_speed) / _encoder_timer_diff / _tops_per_tour;
  }

  _prev_total_pulses_for_speed = _total_pulses_for_speed;

  return _raw_encoder_speed;
}

float Encoder::get_encoder_filtered_speed()
{
  if (micros() - _encoder_timer > 2.0f / (min_vitesse * _tops_per_tour) * seconds_to_micro )
  {
    _raw_encoder_speed = 0;
  }

  // lance le calcul de la vitesse et met à jour _raw_encoder_speed et _prev_raw_encoder_speed
  get_raw_encoder_speed();

  if ( abs(_raw_encoder_speed - _prev_raw_encoder_speed) > _speed_change_threshold )
  {
    _raw_encoder_speed = max(_prev_raw_encoder_speed - _speed_change_threshold, min(_raw_encoder_speed, _prev_raw_encoder_speed + _speed_change_threshold));
  }
  
  _encoder_filtered_speed = _alpha_filter * _raw_encoder_speed + (1 - _alpha_filter) * _encoder_filtered_speed;
  
  return _encoder_filtered_speed;  
}
