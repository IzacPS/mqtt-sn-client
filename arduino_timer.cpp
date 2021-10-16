

#include "arduino_timer.h"
#if ARDUINO >= 100
	#include <Arduino.h>
#endif

arduino_timer_t::arduino_timer_t(){
}

arduino_timer_t::arduino_timer_t(uint32_t ms){
    _end_time = millis() + ms;

}

void arduino_timer_t::stop(){
    _end_time = 0;
}

void arduino_timer_t::start(uint32_t msec){
    _end_time = millis() + msec;
}

void arduino_timer_t::random_start(uint32_t msec)
{
	srand(millis());
	uint32_t tm = rand() % (msec);
	start(tm);
}


bool arduino_timer_t::is_time_up(){
	return (millis() >= _end_time);
}

