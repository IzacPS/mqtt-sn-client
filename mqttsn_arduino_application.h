#ifndef MQTTSN_ARDUINO_CLIENT_H
#define MQTTSN_ARDUINO_CLIENT_H
#include "MQTTSNClient.h"
#include "arduino_timer.h"

#define TOPIC_REGISTRATION_DELIMITER {0, 0, 0}

namespace mqttsn
{
//#define APPLICATION_DEBUG
#if defined(APPLICATION_DEBUG)
    typedef sensor_network_t<SoftwareSerial, 19200, 9,10> app_sensor_network_t;
#else
    typedef sensor_network_t<Stream, 19200, 0> app_sensor_network_t;
#endif

    //code from mqttsnClientAppFw4Arduino.h
    //put reference later
    enum mqtt_int_status
    {
        WAIT, 
        INT0_LL,
        INT0_HL,
        INT_WDT
    };


    struct topic_registrations_t
    {
        const char *topicname;
        void* data;
        uint8_t data_size;
    };

    struct topic_subscription_map_t
    {
        const char *topicname;
        mqttsn::message_handler_pfn handler;
    };

    class arduino_application_t
    {
    public:
        arduino_application_t() = default;
        ~arduino_application_t() = default;
        void init();
        void connect(
            const char *device_name,
            uint32_t keep_alive_time,
            uint8_t clean_section,
            uint8_t will_flag);
        void ping_request();
        void register_topics();
        void subscribe_for_topics();
        void publish_topics();
        void disconnect(uint16_t time = 0);
        void flush(){ get_network_instance()->flush(); }
        void sleep() {get_client_instance()->sleep(); }
        void wakeup() {get_client_instance()->wakeup(); }

        app_sensor_network_t  *get_network_instance();
        client_t<app_sensor_network_t, arduino_timer_t> *get_client_instance();
    };
}

#endif //MQTTSN_ARDUINO_CLIENT_H