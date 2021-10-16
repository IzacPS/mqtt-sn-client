#include <SoftwareSerial.h>
#include <mqttsn_arduino_application.h>
#define PIN_A0_WIND_DIRECTON 0
#define PIN_A1_SOIL 1

#define SLEEP_TIME_IN_MIN 1

#define WAKEUP_CLOCK_INT 3

#define REED_PIN 5

#define ANEM_WIND_SPEED_INT_PIN 2
#define ANEM_WIND_SPEED_COLLECT_DATA_TIME_MS 5000
#define ANEM_RADIUS_MM 147

#define MATH_PI 3.14159265

#include <DHT.h>; //INCLUSÃO DE BIBLIOTECA
#define DHTPIN 4 //PINO DIGITAL UTILIZADO PELO DHT22
#define DHTTYPE DHT22 //DEFINE O MODELO DO SENSOR (DHT22 / AM2302)

DHT dht(DHTPIN, DHTTYPE); //PASSA OS PARÂMETROS PARA A FUNÇÃO

enum
{
    DIRECTION_N   = 0,
    DIRECTION_NE  = 45,
    DIRECTION_E   = 90,
    DIRECTION_SE  = 135,
    DIRECTION_S   = 180,
    DIRECTION_SO  = 225,
    DIRECTION_O   = 270,
    DIRECTION_NO  = 315
};

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
    float wind_speed = 0;
    float air_humidity = 0;
    float temperature = 0;
    float rain_mm;
    uint16_t wind_rpm = 0;
    uint16_t wind_direction = 0;
    uint16_t soil_moisture = 0;
}data_to_send;

mqttsn::topic_registrations_t reg[] = {
    {"sta1/wind/speed", &data_to_send.wind_speed, sizeof(float)},
    //{"sta1/wind/rpm", inform_wind_rpm},
    {"sta1/wind/dir", &data_to_send.wind_direction, sizeof(uint16_t)},
    {"sta1/soil", &data_to_send.soil_moisture, sizeof(uint16_t)},
    {"sta1/air/rh", &data_to_send.air_humidity, sizeof(float)},
    {"sta1/temp", &data_to_send.temperature, sizeof(float)},
    {"sta1/rain", &data_to_send.rain_mm, sizeof(float)},
    TOPIC_REGISTRATION_DELIMITER
};

 //mqttsn::topic_subscription_map_t tmap [] = {
//     {" ", turn_on_led},
//     {"nw/zb/ard1/print", print_byte},
//     TOPIC_REGISTRATION_DELIMITER
// };

uint16_t get_soil_moisture()
{
    return analogRead(PIN_A1_SOIL);
}

uint16_t spin_counter = 0;

void add_spin_counter(){ 
    spin_counter++; 
}

void anem_wind_get_props(data_to_send_t &props)
{
    spin_counter = 0;
    attachInterrupt(digitalPinToInterrupt(ANEM_WIND_SPEED_INT_PIN), add_spin_counter, RISING);
    //long start_time = millis();
    long end_time = millis() + ANEM_WIND_SPEED_COLLECT_DATA_TIME_MS;
    while(millis() < end_time);
    detachInterrupt(digitalPinToInterrupt(ANEM_WIND_SPEED_INT_PIN));
    props.wind_rpm = (spin_counter * 60) / (ANEM_WIND_SPEED_COLLECT_DATA_TIME_MS / 1000);
    props.wind_speed = (((4 * MATH_PI * ANEM_RADIUS_MM * props.wind_rpm) / 60) / 1000) * 3.6; //km/h
}

uint16_t wind_direction()
{
    float val = analogRead(PIN_A0_WIND_DIRECTON) * (5.0 / 1023.0);
    
    if(val <= 0.21)
        return DIRECTION_NO;
    else if(val <= 0.42)
        return DIRECTION_O;
    else if(val <= 0.63)
        return DIRECTION_SO;
    else if(val <= 0.84)
        return DIRECTION_S;
    else if(val <= 1.05)
        return DIRECTION_SE;
    else if(val <= 1.26)
        return DIRECTION_E;
    else if(val <= 1.47)
        return DIRECTION_NE;
    else
        return DIRECTION_N;

}

uint16_t old_val = 0;
uint16_t reed_count = 0;
float res = 0;
float get_rain_measure()
{
    //long start_time = millis();
    long end_time = millis() + ANEM_WIND_SPEED_COLLECT_DATA_TIME_MS;
    while(millis() < end_time)
    {
        uint16_t val = digitalRead(REED_PIN);
        if(val == LOW && old_val == HIGH)
        {
            delay(10);
            res = (float)reed_count  * 0.25;
            reed_count = reed_count + 1;
            old_val = val;
        }
        else
            old_val = val;
    }
    return res;
}


void set_data_to_send()
{
    data_to_send.soil_moisture = get_soil_moisture();
    anem_wind_get_props(data_to_send);
    data_to_send.wind_direction = wind_direction();
    data_to_send.air_humidity = dht.readHumidity();
    data_to_send.temperature = dht.readTemperature();
    data_to_send.rain_mm = get_rain_measure();
}

mqttsn::arduino_application_t application;

#include <Rtc_Pcf8563.h>
Rtc_Pcf8563 rtc;

#include <avr/sleep.h>
#include <avr/wdt.h>
#define SLEEP_COUNT_TO_WAKEUP 112
volatile uint32_t sleep_count = SLEEP_COUNT_TO_WAKEUP;
volatile bool wdt_interrupt;

//handles the Watchdog Time-out Interrupt
ISR(WDT_vect)
{
    wdt_interrupt = true;
}

void enter_sleep()
{
    application.sleep();
    //byte adcsra = ADCSRA;          //save the ADC Control and Status Register A
    //ADCSRA = 0;                    //disable the ADC
    //EICRA = _BV(ISC01);            //configure INT0 to trigger on falling edge
    //EIFR = _BV(INTF0);             //ensure interrupt flag cleared
    //EIMSK = _BV(INT0);             //enable INT0
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    //ATOMIC_BLOCK(ATOMIC_FORCEON) {
        wdt_interrupt = false;
       // extInterrupt = false;
        sleep_enable();
    //    byte mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE); //turn off the brown-out detector while sleeping
    //    byte mcucr2 = mcucr1 & ~_BV(BODSE);
    //    MCUCR = mcucr1; //timed sequence
    //    MCUCR = mcucr2; //BODS stays active for 3 cycles, sleep instruction must be executed while it's active
    //}
    sleep_cpu();                   //go to sleep
    sleep_disable();               //wake up here
    delay(100);
    application.wakeup();
    //ADCSRA = adcsra;               //restore ADCSRA
}

void setup()
{
    //Serial.begin(19200);
    dht.begin();
    application.init();

    pinMode(REED_PIN, INPUT_PULLUP);
    

    //pinMode(WAKEUP_CLOCK_INT, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(WAKEUP_CLOCK_INT), wakeup, FALLING);
        //wdt_reset();
    //enable wdt
    MCUSR &= ~_BV(WDRF);                            //clear WDRF
    WDTCSR |= _BV(WDCE) | _BV(WDE);                 //enable WDTCSR change
    WDTCSR =  _BV(WDIE) | _BV(WDP3) | _BV(WDP0);    //~8 sec
    //Serial.println("start");
}

//byte ss[] = {0x7E, 0x00, 0x7D, 0x31, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE, 0x00, 0x00, 0x03, 0x01, 0x00, 0xED};


volatile bool was_connected = false;
volatile bool was_sleeping = false;
void loop()
{    
    if(wdt_interrupt)
    {
        //Serial.println(".");
        if(++sleep_count >= SLEEP_COUNT_TO_WAKEUP){
            sleep_count = 0;
            wdt_disable();
            delay(2000);
            //Serial.println("wokeup");
            application.connect("ard0", 600, false, false);
            //Serial.println("Connected");
            
            if(!was_connected)
            {
                application.register_topics();
                was_connected = true;
                //Serial.println("topics registered");
            }
            
            if(was_sleeping)
            {
                application.ping_request();
                was_sleeping = false;
                //Serial.println("ping requested");
            }

            set_data_to_send();
            application.publish_topics();
            //Serial.println("topics published");
            
            application.disconnect(600);
            delay(200);
            //Serial.println("going to sleep");

            wdt_reset();
            MCUSR &= ~_BV(WDRF);                            //clear WDRF
            WDTCSR |= _BV(WDCE) | _BV(WDE);                 //enable WDTCSR change
            WDTCSR =  _BV(WDIE) | _BV(WDP3) | _BV(WDP0);    //~8 sec
        }
    }
    application.flush();
    //delay(10000);
    enter_sleep();
}
