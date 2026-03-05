#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>

class Encoder
{
  public:
    Encoder(unsigned int pinA, unsigned int pinB, int tops_per_tour, bool inv_sign);

    void tic_detector();
    void init_codeur(void (*ISR_callback)(void));
    long get_encoder_tics();
    float get_raw_encoder_speed();
    float get_encoder_filtered_speed();
    void reset_codeur();
    int get_tops_per_tour();

  private:
  
    volatile long _total_pulses;
    volatile long _prev_total_pulses;
    long int _encoder_timer = 0;
    long int _prev_encoder_timer = 0;
    long int _encoder_timer_diff = 0;
    float _raw_encoder_speed = 0.0;    
    float _prev_raw_encoder_speed = 0.0;   
    float _encoder_filtered_speed = 0.0;  
    
    unsigned int _pinA;
    unsigned int _pinB;
    bool _inv_sign = false;

    const float min_vitesse = 0.1; // rps

    int _tops_per_tour = 400;
    float seconds_to_micro = 1000000.0;

    float _alpha_filter = 0.2;
    float _speed_change_threshold = 0.1;
};

#endif