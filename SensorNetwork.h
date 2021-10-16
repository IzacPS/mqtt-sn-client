/**************************************************************************************
 * Copyright (c) 2016, Tomoaki Yamaguchi
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Tomoaki Yamaguchi - initial API and implementation 
 **************************************************************************************/
#ifndef SENSORNETWORKX_H_
#define SENSORNETWORKX_H_
#if ARDUINO >= 100
	#include <Arduino.h>
	#include <ArxTypeTraits.h>
	#include <SoftwareSerial.h>
	#include "mqttsn_debug.h"
	#if defined(__AVR_ATmega328P__)
		#define RECV_DELAY_TYPE_US 800
	#elif defined(__AVR_ATmega2560__)
		#define RECV_DELAY_TYPE_US 800
	#endif
#endif


using namespace std;

namespace mqttsn
{
	namespace
	{
		//uint8_t zigbee_data = 


		template<typename T, int rx, int tx = -1>
		struct serialn_t;

		template<>
		struct serialn_t<Stream, 0, -1>{
			static  void begin(int baudrate) { Serial.begin(baudrate);}

			static Stream* get_instance() { return (Stream*)&Serial; }
		};

		template<int rx, int tx>
		struct serialn_t<SoftwareSerial, rx, tx>{
			static inline void begin(int baudrate) { serialn_t<SoftwareSerial, rx, tx>::get_instance()->begin(baudrate);}

			static inline SoftwareSerial* get_instance() { 
				static SoftwareSerial ss(rx, tx);
				return &ss;	
			}
		};

		
	#if defined(UBRR1H)
		template<>
		struct serialn_t<Stream, 1, -1>{
			static void begin(int baudrate) { Serial1.begin(baudrate);}
			static Stream* get_instance() { return (Stream*)&Serial1; }
		};
	#endif

	#if defined(UBRR2H)
		template<>
		struct serialn_t<Stream, 2, -1>{
			static void begin(int baudrate) { Serial2.begin(baudrate);}

			static Stream* get_instance() { return (Stream*)&Serial2; };
		};
	#endif
	#if defined(UBRR2H)
		template<>
		struct serialn_t<Stream, 3, -1>{
			static void begin(int baudrate) { Serial3.begin(baudrate);}
			static Stream* get_instance() { return (Stream*)&Serial3; };
		};
	#endif
	};

//#define DEBUG_NWSTACK

#ifdef  DEBUG_NWSTACK
  #define D_NWSTACK(...) printf(__VA_ARGS__)
#else
  #define D_NWSTACK(...)
#endif

#define API_XMITREQUEST          0x10
#define API_RESPONSE             0x90
#define API_MODEMSTATUS          0x8A
#define API_XMITSTATUS           0x8B

#define XMIT_STATUS_TIME_OVER    5000

#define START_BYTE               0x7e
#define ESCAPE                   0x7d
#define XON                      0x11
#define XOFF                     0x13

#define XBEE_CTS_PIN 7
#define XBEE_SLEEP_PIN 8

// /*===========================================
//   Class  serial_port_t
//  ============================================*/
template<typename serial_t, int baudrate, int rxPIN, int txPIN = -1>
class serial_port_t{
public:
	~serial_port_t() = default;
	void init();
	bool write(unsigned char b) 
	{
		while(true){
			if(digitalRead(XBEE_CTS_PIN) == LOW)
				break;
		} 
		return (bool)_serial->write(b); 
	}

	int read() { return _serial->read(); }
	uint8_t available() { return _serial->available(); }
	void flush() { _serial->flush(); }
	static serial_port_t<serial_t, baudrate, rxPIN, txPIN> *get_instance();

private:
	serial_port_t();
	serial_t *_serial;
};

// /*=========================================
//  Class serial_port_t
//  =========================================*/
template<typename serial_t, int baudrate, int rxPIN, int txPIN>
serial_port_t<serial_t, baudrate, rxPIN, txPIN>::serial_port_t()
{
	pinMode(XBEE_CTS_PIN, INPUT_PULLUP); 
	_serial = serialn_t<serial_t, rxPIN, txPIN>::get_instance();
	serialn_t<serial_t, rxPIN, txPIN>::begin(baudrate);
}

template<typename serial_t, int baudrate, int rxPIN, int txPIN>
void serial_port_t<serial_t, baudrate, rxPIN, txPIN>::init()
{
	_serial = serialn_t<serial_t, rxPIN, txPIN>::get_instance();
	serialn_t<serial_t, rxPIN, txPIN>::begin(baudrate);
}


template<typename serial_t, int baudrate, int rxPIN, int txPIN>
serial_port_t<serial_t, baudrate, rxPIN, txPIN>* serial_port_t<serial_t, baudrate, rxPIN, txPIN>::get_instance()
{
	static serial_port_t<serial_t, baudrate, rxPIN, txPIN> serial_port_instance = serial_port_t<serial_t, baudrate, rxPIN, txPIN>();
	return &serial_port_instance;
}

// /*===========================================
//  Class  SensorNetAddreess
//  ============================================*/


struct sensor_net_address_t
{
	sensor_net_address_t() = default;
	~sensor_net_address_t() = default;
	void set_address(uint8_t* address64, uint8_t* address16);
	int  set_address(const char* data);
	void set_broadcast_address(void);
	bool is_match(sensor_net_address_t* addr);
	sensor_net_address_t& operator =(sensor_net_address_t& addr);
	char* sprint(char*);

	uint8_t _address64[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff};
	uint8_t _address16[2] = {0xff, 0xfe};
};

union xbee_api_frame_t
{
	struct
	{
		uint8_t start_byte;
		uint8_t msb;
		uint8_t lsb;
		uint8_t cmd_data[]; 
	};
	uint8_t data[128];
};

// /*========================================
//  Class xbee_t
//  =======================================*/
template<typename serial_t, int baudrate, int rxPIN, int txPIN = -1>
class xbee_t
{
public:
	xbee_t() = default;
	xbee_t(uint8_t api_mode);
	~xbee_t() = default;
	void init(uint8_t api_mode);
	int available() { return _serial_port->available(); }
	int unicast(const uint8_t* buf, uint16_t length, sensor_net_address_t* send_to_addr);
	int broadcast(const uint8_t* buf, uint16_t length);
	int recv(uint8_t* buf, sensor_net_address_t* addr);
	void set_api_mode(uint8_t mode);
	void sleep();
	void wakeup();
	void close(void);
	void flush(){ _serial_port->flush(); }

private:
	int read_api_frame(xbee_api_frame_t &frame_data);
	bool recv(uint8_t* buf);
	int send(const uint8_t* payload, uint8_t plen, sensor_net_address_t* addr);
	void send(uint8_t b);

	serial_port_t<serial_t, baudrate, rxPIN, txPIN>* _serial_port;
	uint8_t _frameid;
	uint8_t _respcd;
	uint8_t _respid;
	uint8_t _datalen;
	uint8_t _apimode;
};

// // /*===========================================
// //               Class  xbee_t
// //  ============================================*/
template<typename serial_t, int baudrate, int rxPIN, int txPIN>
xbee_t<serial_t, baudrate, rxPIN, txPIN>::xbee_t(uint8_t api_mode){
	
    _serial_port = serial_port_t<serial_t, baudrate, rxPIN, txPIN>::get_instance();
    _respcd = 0;
    _respid = 0;
    _datalen = 0;
    _frameid = 0;
    _apimode = api_mode;
	pinMode(XBEE_SLEEP_PIN, OUTPUT);
	digitalWrite(XBEE_SLEEP_PIN, LOW);
}

template<typename serial_t, int baudrate, int rxPIN, int txPIN>
void xbee_t<serial_t, baudrate, rxPIN, txPIN>::init(uint8_t api_mode){
	
    _serial_port = serial_port_t<serial_t, baudrate, rxPIN, txPIN>::get_instance();
    _respcd = 0;
    _respid = 0;
    _datalen = 0;
    _frameid = 0;
    _apimode = api_mode;

	pinMode(XBEE_SLEEP_PIN, OUTPUT);
	digitalWrite(XBEE_SLEEP_PIN, LOW);
}

template<typename serial_t, int baudrate, int rxPIN, int txPIN>
int xbee_t<serial_t, baudrate, rxPIN, txPIN>::broadcast(const uint8_t* payload, uint16_t payloadlen){
	sensor_net_address_t addr;
	return send(payload, (uint8_t)payloadlen, &addr);
}

template<typename serial_t, int baudrate, int rxPIN, int txPIN>
int xbee_t<serial_t, baudrate, rxPIN, txPIN>::unicast(const uint8_t* payload, uint16_t payloadlen, sensor_net_address_t* addr){
	return send(payload, (uint8_t) payloadlen, addr);
}

template<typename serial_t, int baudrate, int rxPIN, int txPIN>
int xbee_t<serial_t, baudrate, rxPIN, txPIN>::recv(uint8_t* buf, sensor_net_address_t* addr)
{
	xbee_api_frame_t api_frame;
	int len;
	
	while ( true )
	{ 	
		//memset(&api_frame, 0, sizeof(xbee_api_frame_t));

		if ( (len = read_api_frame(api_frame)) > 0 )
		{
			
			if ( api_frame.cmd_data[0] == API_RESPONSE )
			{
				memcpy(addr->_address64, api_frame.data + 4, 8);
				memcpy(addr->_address16, api_frame.data + 12, 2);
				len -= 12;
				memcpy( buf, api_frame.data + 15, len);
				return len;
			}
			else if ( api_frame.cmd_data[0] == API_XMITSTATUS )
			{
				_respid = api_frame.data[1];
				_respcd = api_frame.data[5];
			}
		}
		else
		{
			//Serial.println("error!");
		    return 0;
		}
	}
}

//TODO: remove all this printing
template<typename serial_t, int baudrate, int rxPIN, int txPIN>
int xbee_t<serial_t, baudrate, rxPIN, txPIN>::read_api_frame(xbee_api_frame_t &frame_data){
	uint8_t buf;
	uint8_t checksum = 0;
	bool found_start = false;
	uint8_t count = 0;

	delayMicroseconds(RECV_DELAY_TYPE_US);
	while (recv(&buf))
	{
		//arduino mega
		delayMicroseconds(RECV_DELAY_TYPE_US);

		if(buf == START_BYTE)
		{	
			//Serial.print(buf, HEX);
			//Serial.print(" ");
			frame_data.data[count++] = START_BYTE;
			found_start = true;
			break;
		}
	}
	
	if(!found_start)
		goto exit;
	
	if(!recv(&buf)) //msb
		goto exit;
	frame_data.data[count++] = buf;
	delayMicroseconds(RECV_DELAY_TYPE_US);
	//Serial.print(buf, HEX);
	//Serial.print(" ");
	
	if(!recv(&buf)) //lsb
		goto exit;
	frame_data.data[count++] = buf;
	delayMicroseconds(RECV_DELAY_TYPE_US);
	//Serial.print(buf, HEX);
	//Serial.print(" ");

	for(uint8_t i = 0; i < frame_data.lsb; i++)
	{
		if(!recv(&buf))
			goto exit;
		frame_data.data[count++] = buf;
		checksum += buf;
		delayMicroseconds(RECV_DELAY_TYPE_US);
		//Serial.print(buf, HEX);
		//Serial.print(" ");
	}

	if(!recv(&buf))
		goto exit;
	frame_data.data[count++] = buf;
	delayMicroseconds(RECV_DELAY_TYPE_US);
	//Serial.print(buf, HEX);

	//Serial.println();
	//Serial.println();
	
    if ((0xff - checksum ) == frame_data.cmd_data[frame_data.lsb])
	{
		_serial_port->flush();
		return frame_data.lsb;
	}
	//else
	// {
	// 	PRINTLN("check sum not match");
	// 	//PRINTLN(checksum);
	// 	//PRINTLN(frame_data.lsb);
	// 	//PRINTLN(frame_data.cmd_data[frame_data.lsb]);
	// 	//PRINTLN(count);
	// }

exit:
	_serial_port->flush();
	return -1;
}

template<typename serial_t, int baudrate, int rxPIN, int txPIN>
int xbee_t<serial_t, baudrate, rxPIN, txPIN>::send(const uint8_t* payload, uint8_t plen, sensor_net_address_t* addr)
{	
    uint8_t checksum = 0;
    //_respcd = -1;

    _serial_port->write(START_BYTE);


    send(0x00);              // Message Length
    send(14 + plen);         // Message Length


    _serial_port->write(API_XMITREQUEST); // Transmit Request API
    checksum += API_XMITREQUEST;

    if (_frameid++ == 0x00 ) // Frame ID
	{
    	_frameid = 1;
	}
    send(_frameid);
    checksum += _frameid;

	for ( int i = 0; i < 8; i++)    // Address64
	{
		send(addr->_address64[i]);
		checksum += addr->_address64[i];
	}

	for ( int i = 0; i < 2; i++)    // Address16
	{
		send(addr->_address16[i]);
		checksum += addr->_address16[i];
	}

    send(0x00);   // Broadcast Radius
    checksum += 0x00;

    send(0x00);   // Option: Use the extended transmission timeout 0x40
    checksum += 0x00;

    for ( uint8_t i = 0; i < plen; i++ ){
        send(payload[i]);     // Payload
        checksum += payload[i];
    }

    checksum = 0xff - checksum;
    send(checksum);

    return (int)plen;
}

template<typename serial_t, int baudrate, int rxPIN, int txPIN>
void xbee_t<serial_t, baudrate, rxPIN, txPIN>::send(uint8_t c)
{
  if(_apimode == 2 && (c == START_BYTE || c == ESCAPE || c == XON || c == XOFF)){
	  _serial_port->write(ESCAPE);
	  _serial_port->write(c ^ 0x20);
  }else{
	  _serial_port->write(c);
  }
}

template<typename serial_t, int baudrate, int rxPIN, int txPIN>
bool xbee_t<serial_t, baudrate, rxPIN, txPIN>::recv(uint8_t* buf)
{
	if (_serial_port->available() > 0)
	{
		*buf = _serial_port->read();
		if ( *buf == ESCAPE && _apimode == 2)
		{
			*buf = _serial_port->read();
			*buf = 0x20 ^ *buf;
		}
		return true;
	}
	return false;
}

template<typename serial_t, int baudrate, int rxPIN, int txPIN>
void xbee_t<serial_t, baudrate, rxPIN, txPIN>::set_api_mode(uint8_t mode)
{
	_apimode = mode;
}

template<typename serial_t, int baudrate, int rxPIN, int txPIN>
void xbee_t<serial_t, baudrate, rxPIN, txPIN>::sleep()
{
	pinMode(XBEE_SLEEP_PIN, INPUT);
	digitalWrite(XBEE_SLEEP_PIN, HIGH);
}

template<typename serial_t, int baudrate, int rxPIN, int txPIN>
void xbee_t<serial_t, baudrate, rxPIN, txPIN>::wakeup()
{
	pinMode(XBEE_SLEEP_PIN, OUTPUT);
	digitalWrite(XBEE_SLEEP_PIN, LOW);
}

// /*===========================================
//  Class  sensor_network_t
//  ============================================*/
template<typename serial_t, int baudrate, int rxPIN, int txPIN = -1>
class sensor_network_t: public xbee_t<serial_t, baudrate, rxPIN, txPIN>
{
public:
	sensor_network_t() = default;
	~sensor_network_t() = default;

	int unicast(const uint8_t* payload, uint16_t payload_length, sensor_net_address_t* sendto);
	int broadcast(const uint8_t* payload, uint16_t payload_length);
	int read(uint8_t* buf, sensor_net_address_t *addr);
//	int initialize(Stream &serial, uint32_t baud_rate);
	const char* get_description(void);
	//sensor_net_address_t* get_sender_address(void);

private:
	//sensor_net_address_t _client_addr;   // Sender's address. not gateway's one.
};

/*===========================================
 Class  sensor_network_t
 ============================================*/
 template<typename serial_t, int baudrate, int rxPIN, int txPIN>
int sensor_network_t<serial_t, baudrate, rxPIN, txPIN>::unicast(const uint8_t* payload, uint16_t payload_length, sensor_net_address_t* send_to_addr)
{
	return xbee_t<serial_t, baudrate, rxPIN, txPIN>::unicast(payload, payload_length, send_to_addr);
}

template<typename serial_t, int baudrate, int rxPIN, int txPIN>
int sensor_network_t<serial_t, baudrate, rxPIN, txPIN>::broadcast(const uint8_t* payload, uint16_t payload_length)
{
	return xbee_t<serial_t, baudrate, rxPIN, txPIN>::broadcast(payload, payload_length);
}

template<typename serial_t, int baudrate, int rxPIN, int txPIN>
int sensor_network_t<serial_t, baudrate, rxPIN, txPIN>::read(uint8_t* buf, sensor_net_address_t *addr)
{
	return xbee_t<serial_t, baudrate, rxPIN, txPIN>::recv(buf, addr);
}

// template<typename serial_t, int baudrate, int rxPIN, int txPIN>
// sensor_net_address_t* sensor_network_t<serial_t, baudrate, rxPIN, txPIN>::get_sender_address(void)
// {
// 	return &_client_addr;
// }

}



#endif /* SENSORNETWORKX_H_ */
