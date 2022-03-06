#include <SoftwareSerial.h>
#include <mqttsn_arduino_application.h>
#define PIN_A0_WIND_DIRECTON 0
#define PIN_A1_SOIL 1

#define SLEEP_TIME_IN_MIN 1

#define WAKEUP_CLOCK_INT 3


#define ANEM_WIND_SPEED_INT_PIN 2
#define ANEM_WIND_SPEED_COLLECT_DATA_TIME_MS 5000
#define ANEM_RADIUS_MM 147

#define MATH_PI 3.14159265

#include <DHT.h>; //INCLUSÃO DE BIBLIOTECA
#define DHTPIN 4 //PINO DIGITAL UTILIZADO PELO DHT22
#define DHTTYPE DHT22 //DEFINE O MODELO DO SENSOR (DHT22 / AM2302)

DHT dht(DHTPIN, DHTTYPE); //PASSA OS PARÂMETROS PARA A FUNÇÃO


struct anem_wind_props_t
{
    uint16_t rpm = 0;
    float wind_speed = 0;
};

// void turn_on_led(mqttsn::message_data_t &data)
// {
//     char led_state = *(char*)data.message.payload;

//     if(led_state == '1')
//         digitalWrite(6, HIGH);
//     else
//         digitalWrite(6, LOW);
// }

// void print_byte(mqttsn::message_data_t &data)
// {

//     // if(led_state)
//     //     digitalWrite(6, HIGH);
//     // else
//     //     digitalWrite(6, LOW);
// }


    uint16_t vals = 0;


struct data_to_send_t
{
    float air_humidity = 0;
    float temperature = 0;
}data_to_send;

mqttsn::topic_registrations_t reg[] = {
    {"sta1/air/rh", &data_to_send.air_humidity, sizeof(float)},
    {"sta1/temp", &data_to_send.temperature, sizeof(float)},
    TOPIC_REGISTRATION_DELIMITER
};

 //mqttsn::topic_subscription_map_t tmap [] = {
//     {" ", turn_on_led},
//     {"nw/zb/ard1/print", print_byte},
//     TOPIC_REGISTRATION_DELIMITER
// };



void set_data_to_send()
{
    data_to_send.air_humidity = dht.readHumidity();
    data_to_send.temperature = dht.readTemperature();
}

mqttsn::arduino_application_t application;

#include <avr/sleep.h>
#include <avr/wdt.h>
#define SLEEP_COUNT_TO_WAKEUP 112
volatile uint32_t sleep_count = SLEEP_COUNT_TO_WAKEUP;
volatile bool wdt_interrupt;

ISR(WDT_vect)
{
    wdt_interrupt = true;
}

void enter_sleep()
{
    application.sleep();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        wdt_interrupt = false;
        sleep_enable();
    //}
    sleep_cpu();                   //go to sleep
    sleep_disable();               //wake up here
    delay(100);
    application.wakeup();
}

void setup()
{
    dht.begin();
    application.init();

    pinMode(REED_PIN, INPUT_PULLUP);
    
    MCUSR &= ~_BV(WDRF);                            //clear WDRF
    WDTCSR |= _BV(WDCE) | _BV(WDE);                 //enable WDTCSR change
    WDTCSR =  _BV(WDIE) | _BV(WDP3) | _BV(WDP0);    //~8 sec
}

volatile bool was_connected = false;
volatile bool was_sleeping = false;
void loop()
{    
    if(wdt_interrupt)
    {
        if(++sleep_count >= SLEEP_COUNT_TO_WAKEUP){
            sleep_count = 0;
            wdt_disable();
            delay(2000);
            application.connect("ard0", 600, false, false);
            
            if(!was_connected)
            {
                application.register_topics();
                was_connected = true;
            }
            
            if(was_sleeping)
            {
                application.ping_request();
                was_sleeping = false;
            }

            set_data_to_send();
            application.publish_topics();
            
            application.disconnect(600);
            delay(200);

            wdt_reset();
            MCUSR &= ~_BV(WDRF);                            //clear WDRF
            WDTCSR |= _BV(WDCE) | _BV(WDE);                 //enable WDTCSR change
            WDTCSR =  _BV(WDIE) | _BV(WDP3) | _BV(WDP0);    //~8 sec
        }
    }
    application.flush();
    enter_sleep();
}
