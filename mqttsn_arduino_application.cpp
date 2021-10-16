#include "mqttsn_arduino_application.h"
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

extern mqttsn::topic_registrations_t reg[];
extern mqttsn::topic_subscription_map_t tmap[];

void mqttsn::arduino_application_t::init()
{
    get_network_instance()->init(2);
    get_client_instance()->init(get_network_instance());
}

void mqttsn::arduino_application_t::connect(
    const char *device_name,
    uint32_t keep_alive_time,
    uint8_t clean_section,
    uint8_t will_flag)
{

    uint8_t rc = FAILURE;
    while((rc = get_client_instance()->searchgw(0)) != SUCCESS);

    MQTTSNPacket_connectData default_options = MQTTSNPacket_connectData_initializer;
    default_options.clientID.cstring = (char*)device_name;
    default_options.duration = keep_alive_time;
    default_options.cleansession = clean_section;
    default_options.willFlag = will_flag;
    
    while((rc = get_client_instance()->connect(default_options)) != SUCCESS);
    // {
    //     Serial.print("result = ");
    //     Serial.println(rc);
    // }
}

void  mqttsn::arduino_application_t::ping_request()
{
    get_client_instance()->ping_request();
}

void mqttsn::arduino_application_t::register_topics()
{
    int rc = FAILURE;
    for(uint8_t i = 0; reg[i].topicname; i++)
    {
        rc = get_client_instance()->register_topic_name(reg[i].topicname, reg[i].data, reg[i].data_size);
    }
}

void mqttsn::arduino_application_t::publish_topics()
{
    get_client_instance()->publish_topics();
}

void mqttsn::arduino_application_t::subscribe_for_topics()
{
    for(uint8_t i = 0; tmap[i].topicname; i++)
    {
        MQTTSN_topicid pid = {};
        pid.data.long_.name = (char*)tmap[i].topicname;
        pid.data.long_.len = strlen(tmap[i].topicname);
        get_client_instance()->subscribe(pid, mqttsn::QOS0, tmap[i].handler);
    }
}

void mqttsn::arduino_application_t::disconnect(uint16_t time = 0)
{
    get_client_instance()->disconnect(time);
    
}




// ISR(WDT_vector)
// {
//     setup_wdt();
//     wdt_reset();
//     if(sleep_count == SLEEP_COUNT_TO_WAKEUP)
//     {
//     //     wdt_disable();
//          device_wokeup = true;
//          sleep_count = 0;
//     }
//     sleep_count++;
// }

// void mqttsn::arduino_application_t::enterSleep()
// {

//     //get_client_instance()->sleep();
//     set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//     sleep_enable();
//     sleep_cpu();
//     sleep_disable();
//     delay(200);
//     was_sleeping = true;
//     //get_client_instance()->wakeup();
// }

mqttsn::app_sensor_network_t  *mqttsn::arduino_application_t::get_network_instance()
{
    static app_sensor_network_t _network;
    return &_network;
}

mqttsn::client_t<mqttsn::app_sensor_network_t , arduino_timer_t> *mqttsn::arduino_application_t::get_client_instance()
{
    static mqttsn::client_t<app_sensor_network_t , arduino_timer_t>  _client;
    return &_client;
}
