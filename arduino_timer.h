/*
 * mqUtil.h
 *
 *                      The BSD License
 *
 *           Copyright (c) 2014, tomoaki@tomy-tech.com
 *                    All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: 2014/06/01
 *    Modified: 2014/09/05
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.0.0
 */

#ifndef ARDUION_TIMER_H
#define ARDUION_TIMER_H
#include <stdint.h>
#include "mqttsn_debug.h"

class arduino_timer_t{
public:
    arduino_timer_t();
    arduino_timer_t(uint32_t msec);
    ~arduino_timer_t() = default;
    void start(uint32_t msec = 0);
    void random_start(uint32_t msec);
    //bool is_time_up(uint32_t msec);
    bool is_time_up(void);
    void stop();
    //static uint32_t get_unix_time();
    //static void set_unix_time(uint32_t utc);
	//static void set_stop_time_duration(uint32_t msec);
    //static void initialize();
    //static void change_UTC();

private:
    long _end_time = 0;
    //uint32_t _start_time;
    //uint32_t _current_time;
    //uint32_t _millis;
	//uint32_t _timeup_unix_time;
	///static uint32_t _unix_time;
	//static uint32_t _epoch_time;
	//static uint32_t _timer_stop_time_accum;
	//static bool _utc_flag;

};

// typedef struct
// {
//     uint32_t prev_time;
//     uint32_t interval;
//     int (*callback)(void);
// }mq_timer_tbl_t;

// typedef struct arduino_timer_t
// {
//     uint32_t start_time;
//     uint32_t current_time;
//     uint32_t millis;
//     uint32_t timeup_unix_time;
//     static uint32_t unix_time;
//     static uint32_t epoch_time;
//     static uint32_t timer_stop_time_accum;
//     static bool utc_flag;
//     mq_timer_tbl_t *timer_tlbs;
//     uint8_t timer_cnt;
//     bool init_flag;
// }arduino_timer_t;

// void arduino_timer_init(arduino_timer_t &timer);

// void arduino_timer_start(arduino_timer_t &timer, uint32_t ms);

// bool arduino_timer_is_time_up(arduino_timer_t &timer, uint32_t ms);

// static inline bool arduino_timer_is_time_up(arduino_timer_t &timer)
// {
//     return arduino_timer_is_time_up(timer, timer.millis);
// }

// void arduino_timer_stop(arduino_timer_t &timer);

// void arduino_timer_set_unix_time(uint32_t utc);

// uint32_t arduino_timer_get_unix_time();

// void arduino_timer_initialize();

// void arduino_timer_set_stop_time_duration(uint32_t ms);

// void arduino_timer_change_utc();

// void arduino_timer_start(void (*start_callback)(void));

// void arduino_timer_stop(void (*stop_callback)(void));

// //bool arduino_timer_wakeup(arduino_timer_t &timer, void (*required_reboot_or_invalid_topic_callback)(void));

// //int arduino_timer_register_callback(arduino_timer_t &timer, uint32_t sec, int (*callback)(void));

// void arduino_timer_refresh_register_table(arduino_timer_t &timer);

#endif //ARDUION_TIMER_H