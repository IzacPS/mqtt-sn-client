/*******************************************************************************
 * Copyright (c) 2014, 2015 IBM Corp.
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
 *    Ian Craggs - initial API and implementation and/or initial documentation
 *******************************************************************************/

#if !defined(MQTTSNCLIENT_H)
#define MQTTSNCLIENT_H

#include "FP.h"
#include "MQTTSNPacket.h"
#include "stdio.h"
#include "MQTTLogging.h"
#include "SensorNetwork.h"
#include <inttypes.h>

// Data limits
#if !defined(MAX_REGISTRATIONS)
    #define MAX_REGISTRATIONS 7
#endif
#if !defined(MAX_REGISTRATION_TOPIC_NAME_LENGTH)
    #define MAX_REGISTRATION_TOPIC_NAME_LENGTH 30
#endif
#if !defined(MAX_INCOMING_QOS2_MESSAGES)
    #define MAX_INCOMING_QOS2_MESSAGES 10
#endif

#if !defined(MQTTSNCLIENT_QOS1)
    #define MQTTSNCLIENT_QOS1 1
#endif
#if !defined(MQTTSNCLIENT_QOS2)
    #define MQTTSNCLIENT_QOS2 0
#endif

#define MQTTSN_MAX_ACTIVE_GATEWAYS  1
#define MQTTSN_ADVERTISE_TIME       (1000 * 60 * 15) // 15 minutes
#define MQTTSN_ADVERTISE_COUNT      3
#define MQTTSN_SEARCHGW_TIME        (1000 * 5)
#define MQTTSN_GWINFO_TIME          MQTTSN_SEARCHGW_TIME
#define MQTTSN_WAIT_TIME            (1000 * 5)
#define MQTTSN_RETRY_TIME           (1000 * 15)
#define MQTTSN_TIMEOUT_TIME         (30000)
#define MQTTSN_RETRY_COUNT          5

#include "mqttsn_debug.h"

namespace mqttsn
{
    enum send_request_type
    {
        NO_REQUEST,
        SEND_REQUEST_UNICAST,
        SEND_REQUEST_BROADCAST
    };

    enum QoS { QOS0, QOS1, QOS2 };

    // all failure return codes must be negative
    enum return_code { MAX_SUBSCRIPTIONS_EXCEEDED = -3, BUFFER_OVERFLOW = -2, FAILURE = -1, SUCCESS = 0 };


    struct message_t
    {
        enum QoS qos;
        bool retained;
        bool dup;
        unsigned short id;
        void *payload;
        size_t payloadlen;
    };


    struct message_data_t
    {
        message_data_t(MQTTSN_topicid &aTopic, struct message_t &aMessage)  : message(aMessage), topic(aTopic)
        { }

        struct message_t &message;
        MQTTSN_topicid &topic;
    };


    class packetid_t
    {
    public:
        packetid_t()
        {
            next = 0;
        }

        int get_next()
        {
            return next = (next == MAX_PACKET_ID) ? 1 : ++next;
        }

    private:
        static const int MAX_PACKET_ID = 65535;
        int next;
    };

    typedef void (*message_handler_pfn)(message_data_t&);

    /**
     * @class MQTTSNClient
     * @brief blocking, non-threaded MQTTSN client API
     *
     * This version of the API blocks on all method calls, until they are complete.  This means that only one
     * MQTT request can be in process at any one time.
     * @param Network a network class which supports send, receive
     * @param Timer a timer class with the methods:
     */
    template<class network_type, class timer_type, int MAX_PACKET_SIZE = 100, int MAX_MESSAGE_HANDLERS = 5>
    class client_t
    {

    public:

        //static message_handler_pfn subscriptions[MAX_MESSAGE_HANDLERS];
        /** Construct the client
         *  @param network - pointer to an instance of the Network class - must be connected to the endpoint
         *      before calling MQTT connect
         *  @param limits an instance of the Limit class - to alter limits as required
         */
        client_t(network_type *network);

        client_t() = default;
        ~client_t() = default;


        /** Set the default message handling callback - used for any message which does not match a subscription message handler
         *  @param mh - pointer to the callback function
         */
        void set_default_message_handler(message_handler_pfn mh)
        {
            default_message_handler.attach(mh);
        }

        void init(network_type* network);
        
        int searchgw(uint8_t radius);

        int ping_request();

        int register_topic_name(const char* topicname, void* data_ptr, uint8_t data_size);

        /** MQTT Connect - send an MQTT connect packet down the network and wait for a Connack
         *  The nework object must be connected to the network endpoint before calling this
         *  Default connect options are used
         *  @return success code -
         */
        int connect();
        
        /** MQTT Connect - send an MQTT connect packet down the network and wait for a Connack
         *  The nework object must be connected to the network endpoint before calling this
         *  @param options - connect options
         *  @return success code -
         */
        int connect(MQTTSNPacket_connectData& options);

        /** MQTT Publish - send an MQTT publish packet and wait for all acks to complete for all QoSs
         *  @param topic - the topic to publish to
         *  @param message - the message to send
         *  @return success code -
         */
        int publish(MQTTSN_topicid& topic, message_t& message);
        
        /** MQTT Publish - send an MQTT publish packet and wait for all acks to complete for all QoSs
         *  @param topic - the topic to publish to
         *  @param payload - the data to send
         *  @param payloadlen - the length of the data
         *  @param qos - the QoS to send the publish at
         *  @param retained - whether the message should be retained
         *  @return success code -
         */
        int publish(MQTTSN_topicid &topic, void* payload, size_t payloadlen, enum QoS qos = QOS0, bool retained = false);
        
        /** MQTT Publish - send an MQTT publish packet and wait for all acks to complete for all QoSs
         *  @param topic - the topic to publish to
         *  @param payload - the data to send
         *  @param payloadlen - the length of the data
         *  @param id - the packet id used - returned 
         *  @param qos - the QoS to send the publish at
         *  @param retained - whether the message should be retained
         *  @return success code -
         */
        int publish(MQTTSN_topicid& topic, void* payload, size_t payloadlen, unsigned short& id, enum QoS qos = QOS1, bool retained = false);
        
        /** MQTT Subscribe - send an MQTT subscribe packet and wait for the suback
         *  @param topic_filter - a topic pattern which can include wildcards
         *  @param qos - the MQTT QoS to subscribe at
         *  @param mh - the callback function to be invoked when a message is received for this subscription
         *  @return success code -
         */

        void publish_topics();

        int subscribe(MQTTSN_topicid& topic_filter, enum QoS qos, message_handler_pfn mh);

        /** MQTT Unsubscribe - send an MQTT unsubscribe packet and wait for the unsuback
         *  @param topic_filter - a topic pattern which can include wildcards
         *  @return success code -
         */
        int unsubscribe(MQTTSN_topicid& topic_filter);

        /** MQTT Disconnect - send an MQTT disconnect packet, and clean up any state
         *  @param duration - used for sleeping clients, 0 means no duration
         *  @return success code -
         */
        int disconnect(unsigned short duration = 0);

        /** A call to this API must be made within the keepAlive interval to keep the MQTT connection alive
         *  yield can be called if no other MQTT operation is needed.  This will also allow messages to be
         *  received.
         *  @param timeout_ms the time to wait, in milliseconds
         *  @return success code - on failure, this means the client has disconnected
         */
        int yield(unsigned long timeout_ms = 1000L);

        /** Is the client connected?
         *  @return flag - is the client connected or not?
         */
        bool is_connected()
        {
            return isconnected;
        }

        void sleep() { _network->sleep(); }
        void wakeup() { _network->wakeup(); }
        
    protected:

        int cycle(timer_type& timer);
        int waitfor(int packet_type, timer_type& timer);

    private:

        uint16_t getUint16(uint8_t* pos){
            uint16_t val = ((uint16_t)*pos++ << 8);
            return val += *pos;
        }

        void setUint16(uint8_t* pos, uint16_t val){
            *pos++ = (val >> 8) & 0xff;
            *pos   = val & 0xff;
        }

        int keepalive();
        int publish(int len, timer_type& timer, enum QoS qos);

        int decode_packet(int* value, int timeout);
        int read_packet(timer_type& timer, sensor_net_address_t *addr, bool wait_for_buffer_data = true);
        int send_packet(int length, timer_type& timer, int8_t send_request_type);
        int deliver_message(MQTTSN_topicid& topic, message_t& message);
        bool is_topic_matched(char* topic_filter, MQTTSNString& topic_name);

        network_type* _network;
        //unsigned long command_timeout_ms;

        unsigned char sendbuf[MAX_PACKET_SIZE];
        unsigned char readbuf[MAX_PACKET_SIZE];

        timer_type last_sent, last_received;
        uint32_t _duration;
        bool ping_outstanding;
        //bool _cleansession;
        MQTTSNString _clientid;

        packetid_t _packetid;
        //packetid_t _reg_topic_id;

        struct message_handlers_t
        {
            MQTTSN_topicid topic_filter = {};
            FP<void, message_data_t&> fp;
        } message_handlers[MAX_MESSAGE_HANDLERS];      // message_t handlers are indexed by subscription topic
        uint8_t message_handlers_count = 0;

        FP<void, message_data_t&> default_message_handler;

        bool isconnected;
        
        struct registrations_t
        {
            //char name[MAX_REGISTRATION_TOPIC_NAME_LENGTH]; might be usefull someday
            void* data_to_send = 0;
            uint16_t id;
            uint8_t data_size;
        } registrations[MAX_REGISTRATIONS];
        uint8_t current_reg = 0;

        struct gateways_t
        {
            uint8_t id;
            union
            {
                uint8_t addr_data[10];
                struct
                {
                    uint8_t address64[8];
                    uint8_t address16[2];
                };
            };
            uint16_t keep_alive_duration;
        }active_gateways[MQTTSN_MAX_ACTIVE_GATEWAYS];
        int8_t active_gateways_count = 0;

        int8_t current_active_gateway = 0;


    #if MQTTCLIENT_QOS1 || MQTTCLIENT_QOS2
        unsigned char pubbuf[MAX_PACKET_SIZE];  // store the last publish for sending on reconnect
        int inflight_len;
        unsigned short inflight_msgid;
        enum QoS inflight_QoS;
    #endif

    #if MQTTCLIENT_QOS2
        bool pubrel;
        unsigned short incoming_QoS2_messages[MAX_INCOMING_QOS2_MESSAGES];
        bool is_QoS2_msgid_free(unsigned short id);
        bool use_QoS2_msgid(unsigned short id);
    #endif

    };

}

template<class network_type, class timer_type, int a, int MAX_MESSAGE_HANDLERS>
mqttsn::client_t<network_type, timer_type, a, MAX_MESSAGE_HANDLERS>::client_t(network_type* network)  : _network(network), _packetid()
{
    //timer_type::initialize();
    last_sent = timer_type();
    last_received = timer_type();

    ping_outstanding = false;
    isconnected = false;
    
#if MQTTCLIENT_QOS1 || MQTTCLIENT_QOS2
    inflight_msgid = 0;
    inflight_QoS = QOS0;
#endif

     
#if MQTTCLIENT_QOS2
    pubrel = false;
    for (int i = 0; i < MAX_INCOMING_QOS2_MESSAGES; ++i)
        incoming_QoS2_messages[i] = 0;
#endif
}

template<class network_type, class timer_type, int a, int MAX_MESSAGE_HANDLERS>
void mqttsn::client_t<network_type, timer_type, a, MAX_MESSAGE_HANDLERS>::init(network_type* network)//, unsigned int command_timeout_ms)
{
    _network = network;
    _packetid = packetid_t();

    //timer_type::initialize();
    last_sent = timer_type();
    last_received = timer_type();

    ping_outstanding = false;
    isconnected = false;
    
#if MQTTCLIENT_QOS1 || MQTTCLIENT_QOS2
    inflight_msgid = 0;
    inflight_QoS = QOS0;
#endif

     
#if MQTTCLIENT_QOS2
    pubrel = false;
    for (int i = 0; i < MAX_INCOMING_QOS2_MESSAGES; ++i)
        incoming_QoS2_messages[i] = 0;
#endif
}

#if MQTTCLIENT_QOS2
template<class network_type, class timer_type, int a, int b>
bool mqttsn::client_t<network_type, timer_type, a, b>::is_QoS2_msgid_free(unsigned short id)
{
    for (int i = 0; i < MAX_INCOMING_QOS2_MESSAGES; ++i)
    {
        if (incoming_QoS2_messages[i] == id)
            return false;
    }
    return true;
}


template<class network_type, class timer_type, int a, int b>
bool mqttsn::client_t<network_type, timer_type, a, b>::use_QoS2_msgid(unsigned short id)
{
    for (int i = 0; i < MAX_INCOMING_QOS2_MESSAGES; ++i)
    {
        if (incoming_QoS2_messages[i] == 0)
        {
            incoming_QoS2_messages[i] = id;
            return true;
        }
    }
    return false;
}
#endif

template<class network_type, class timer_type, int a, int b>
int mqttsn::client_t<network_type, timer_type, a, b>::send_packet(int length, timer_type& timer, int8_t send_request_type)
{
    int rc = FAILURE,
        sent = 0;

    if(send_request_type == SEND_REQUEST_BROADCAST)
    {
        while(sent != length && !timer.is_time_up())
        {
            sent = _network->broadcast(sendbuf,length);
        }
    }else if(send_request_type == SEND_REQUEST_UNICAST)
    {
        while(sent != length && !timer.is_time_up())
        {
            sent = _network->unicast(sendbuf, length, (sensor_net_address_t*)active_gateways[current_active_gateway].addr_data);
        }
    }

    if (sent == length)
    {
        last_sent.start(_duration);
        rc = SUCCESS;
    }

    return rc;
}

template<class network_type, class timer_type, int a, int b>
int mqttsn::client_t<network_type, timer_type, a, b>::decode_packet(int* value, int timeout)
{
    unsigned char c;
    int multiplier = 1;
    int len = 0;
    const int MAX_NO_OF_REMAINING_LENGTH_BYTES = 4;

    *value = 0;
    do
    {
        int rc = MQTTSNPACKET_READ_ERROR;

        if (++len > MAX_NO_OF_REMAINING_LENGTH_BYTES)
        {
            rc = MQTTSNPACKET_READ_ERROR; /* bad data */
            goto exit;
        }
        rc = _network->read(&c, 1);
        if (rc != 1)
            goto exit;
        *value += (c & 127) * multiplier;
        multiplier *= 128;
    } while ((c & 128) != 0);
exit:
    return len;
}

/**
 * If any read fails in this method, then we should disconnect from the network, as on reconnect
 * the packets can be retried.
 * @param timeout the max time to wait for the packet read to complete, in milliseconds
 * @return the MQTT packet type, or -1 if none
 */
template<class network_type, class timer_type, int MAX_PACKET_SIZE, int b>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, b>::read_packet(timer_type& timer, sensor_net_address_t *addr,  bool wait_for_buffer_data)
{
    int rc = FAILURE;
    int len = 0;  // the length of the whole packet including length field 
    int lenlen = 0;
    int datalen = 0;

    #define MQTTSN_MIN_PACKET_LENGTH 2
    
    if(timer.is_time_up())
        goto exit;

    if(wait_for_buffer_data)
    {
        while(!_network->available())
        {
            if(timer.is_time_up())
                goto exit;
        }
        //delay(100);
    }
    if ((len = _network->read(readbuf, addr)) < MQTTSN_MIN_PACKET_LENGTH)
        goto exit;
    
    // 2. read the length.  This is variable in itself 
    lenlen = MQTTSNPacket_decode(readbuf, len, &datalen);
    if (datalen != len)
        goto exit; // there was an error 

    rc = readbuf[lenlen];
    last_received.start(_duration);

exit:   
#if defined(MQTT_DEBUG)
    char printbuf[50];
    DEBUG("Rc %d from receiving packet %s\n", rc, MQTTPacket_toString(printbuf, sizeof(printbuf), readbuf, len));
#endif
    return rc;
}

// assume topic filter and name is in correct format
// # can only be at end
// + and # can only be next to separator
template<class network_type, class timer_type, int a, int b>
bool mqttsn::client_t<network_type, timer_type, a, b>::is_topic_matched(char* topic_filter, MQTTSNString& topic_name)
{
    char* curf = topic_filter;
    char* curn = topic_name.lenstring.data;
    char* curn_end = curn + topic_name.lenstring.len;

    while (*curf && curn < curn_end)
    {
        if (*curn == '/' && *curf != '/')
            break;
        if (*curf != '+' && *curf != '#' && *curf != *curn)
            break;
        if (*curf == '+')
        {   // skip until we meet the next separator, or end of string
            char* nextpos = curn + 1;
            while (nextpos < curn_end && *nextpos != '/')
                nextpos = ++curn + 1;
        }
        else if (*curf == '#')
            curn = curn_end - 1;    // skip until end of string
        curf++;
        curn++;
    };

    return (curn == curn_end) && (*curf == '\0');
}

template<class network_type, class timer_type, int a, int MAX_MESSAGE_HANDLERS>
int mqttsn::client_t<network_type, timer_type, a, MAX_MESSAGE_HANDLERS>::deliver_message(MQTTSN_topicid& topic, message_t& message)
{
    int rc = FAILURE;

    // we have to find the right message handler - indexed by topic
    for (int i = 0; i < MAX_MESSAGE_HANDLERS; ++i)
    {
        if (topic.data.id == message_handlers[i].topic_filter.data.id)//(MQTTSNtopic_equals(&topic, message_handlers[i].topic_filter) ||
        {
            if (message_handlers[i].fp.attached())
            {
                message_data_t md(topic, message);
                message_handlers[i].fp(md);
                rc = SUCCESS;
            }
        }
    }

    if (rc == FAILURE && default_message_handler.attached())
    {
        message_data_t md(topic, message);
        default_message_handler(md);
        rc = SUCCESS;
    }

    return rc;
}

template<class network_type, class timer_type, int a, int b>
int mqttsn::client_t<network_type, timer_type, a, b>::yield(unsigned long timeout_ms)
{
    int rc = SUCCESS;
    timer_type timer = timer_type();

    timer.start(timeout_ms);
    while (!timer.is_time_up())
    {
        if (cycle(timer) == FAILURE)
        {
            rc = FAILURE;
            break;
        }
    }

    return rc;
}

template<class network_type, class timer_type, int MAX_PACKET_SIZE, int b>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, b>::cycle(timer_type& timer)
{
    /* get one piece of work off the wire and one pass through */

    // read the socket, see what work is due
    sensor_net_address_t addr;
    unsigned short packet_type = read_packet(timer, &addr);
    int len = 0;
    unsigned char rc = SUCCESS;
    switch (packet_type)
    {
        case MQTTSN_GWINFO:
        {
            if(readbuf[0] == 3)
            {
                active_gateways[active_gateways_count].id = readbuf[2];
                memcpy(active_gateways[active_gateways_count].addr_data, addr._address64, 10);
                goto exit;
            }
            else if(readbuf[0] > 3)
            {
                if(MQTTSNDeserialize_gwinfo(&active_gateways[active_gateways_count].id, 0, (unsigned char**)&active_gateways[active_gateways_count].addr_data, readbuf, MAX_PACKET_SIZE) == 1)
                    goto exit;
            }
            rc = FAILURE;
        }break;
        case MQTTSN_CONNACK:
        case MQTTSN_PUBACK:
        case MQTTSN_SUBACK:
        case MQTTSN_REGACK:
        break;
        case MQTTSN_REGISTER:
        {
            unsigned short topicid, packetid;
            MQTTSNString topic_name;
            rc = MQTTSN_RC_ACCEPTED;
            if (MQTTSNDeserialize_register(&topicid, &packetid, &topic_name, readbuf, MAX_PACKET_SIZE) != 1)
                goto exit;
            len = MQTTSNSerialize_regack(sendbuf, MAX_PACKET_SIZE, topicid, packetid, rc);
            if (len <= 0)
                rc = FAILURE;
            else
                rc = send_packet(len, timer, SEND_REQUEST_UNICAST);
            break;
        }
        case MQTTSN_PUBLISH:
            MQTTSN_topicid topicid;
            message_t msg;
            if (MQTTSNDeserialize_publish((unsigned char*)&msg.dup, (int*)&msg.qos, (unsigned char*)&msg.retained, (unsigned short*)&msg.id, &topicid,
                                 (unsigned char**)&msg.payload, (int*)&msg.payloadlen, readbuf, MAX_PACKET_SIZE) != 1)
                goto exit;
#if MQTTCLIENT_QOS2
            if (msg.qos != QOS2)
#endif
                deliver_message(topicid, msg);
#if MQTTCLIENT_QOS2
            else if (is_QoS2_msgid_free(msg.id))
            {
                if (use_QoS2_msgid(msg.id))
                    deliver_message(topic_name, msg);
                else
                    WARN("Maximum number of incoming QoS2 messages exceeded");
            }   
#endif
#if MQTTCLIENT_QOS1 || MQTTCLIENT_QOS2
            if (msg.qos != QOS0)
            {
                if (msg.qos == QOS1)
                    len = MQTTSNSerialize_puback(sendbuf, MAX_PACKET_SIZE, topicid.data.id, msg.id, 0);
                else if (msg.qos == QOS2)
                    len = MQTTSNSerialize_pubrec(sendbuf, MAX_PACKET_SIZE, msg.id);
                if (len <= 0)
                    rc = FAILURE;
                else
                    rc = send_packet(len, timer, SEND_REQUEST_UNICAST);
                if (rc == FAILURE)
                    goto exit; // there was a problem
            }
            break;
#endif
#if MQTTCLIENT_QOS2
        case PUBREC:
            unsigned short mypacketid;
            unsigned char dup, type;
            if (MQTTDeserialize_ack(&type, &dup, &mypacketid, readbuf, MAX_PACKET_SIZE) != 1)
                rc = FAILURE;
            else if ((len = MQTTSerialize_ack(sendbuf, MAX_PACKET_SIZE, PUBREL, 0, mypacketid)) <= 0)
                rc = FAILURE;
            else if ((rc = send_packet(len, timer, SEND_REQUEST_UNICAST)) != SUCCESS) // send the PUBREL packet
                rc = FAILURE; // there was a problem
            if (rc == FAILURE)
                goto exit; // there was a problem
            break;
        case PUBCOMP:
            break;
#endif
            break;
        case MQTTSN_PINGRESP:
            ping_outstanding = false;
            break;
    }

exit:
    keepalive();
    if (rc == SUCCESS)
        rc = packet_type;
    return rc;
}

template<class network_type, class timer_type, int MAX_PACKET_SIZE, int b>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, b>::ping_request()
{ //TODO: ADD TIME RETRY 
    timer_type timer = timer_type();
    int rc = FAILURE;

    int len = MQTTSNSerialize_pingreq(sendbuf, MAX_PACKET_SIZE, _clientid);
    if (len <= 0 || (rc = send_packet(len, timer, SEND_REQUEST_UNICAST)) != SUCCESS)
        goto exit;
    rc = SUCCESS;
    ping_outstanding = true;
    timer.start(MQTTSN_WAIT_TIME);
    if(waitfor(MQTTSN_PINGRESP, timer) != MQTTSN_PINGRESP)
        rc = FAILURE;
exit:
    return rc;
}


template<class network_type, class timer_type, int MAX_PACKET_SIZE, int b>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, b>::keepalive()
{
    int rc = FAILURE;   

    if (_duration == 0)
    {
        rc = SUCCESS;
        goto exit;
    }

    if (last_sent.is_time_up() || last_received.is_time_up())
    {
        if (!ping_outstanding)
        {
            timer_type timer = timer_type(MQTTSN_WAIT_TIME);
            int len = MQTTSNSerialize_pingreq(sendbuf, MAX_PACKET_SIZE, _clientid);
            if (len > 0 && (rc = send_packet(len, timer, SEND_REQUEST_UNICAST)) == SUCCESS) // send the ping packet
                ping_outstanding = true;
        }
    }

exit:
    return rc;
}

// only used in single-threaded mode where one command at a time is in process
template<class network_type, class timer_type, int a, int b>
int mqttsn::client_t<network_type, timer_type, a, b>::waitfor(int packet_type, timer_type& timer)
{
    int rc = FAILURE;
    //++times;
    do
    {
        if (timer.is_time_up())
            break; // we timed out
    }while ((rc = cycle(timer)) != packet_type);

    return rc;
}

template<class network_type, class timer_type, int MAX_PACKET_SIZE, int b>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, b>::connect(MQTTSNPacket_connectData& options)
{
    timer_type connect_timer = timer_type();
    int rc = FAILURE;
    int len = 0;

    // don't send connect packet again if we are already connected
    if (isconnected)
        goto exit;
    
    _duration = (uint32_t)options.duration  * (uint32_t)1000; //cast needed for multiplication - bug found without casting
    //_cleansession = options.cleansession;
    _clientid = options.clientID;
    if ((len = MQTTSNSerialize_connect(sendbuf, MAX_PACKET_SIZE, &options)) <= 0)
        goto exit;

    connect_timer.start(MQTTSN_WAIT_TIME);
    if ((rc = send_packet(len, connect_timer, SEND_REQUEST_UNICAST)) != SUCCESS)  // send the connect packet
        goto exit; // there was a problem

    last_received.start(_duration); // this is to sincronize the received and send timers;
    // this will be a blocking call, wait for the connack
    //consider the case where the cliente want to send will
    if (waitfor(MQTTSN_CONNACK, connect_timer) == MQTTSN_CONNACK)
    {
        //Serial.println("passed waitfor");
        int connack_rc = 255;
        if (MQTTSNDeserialize_connack(&connack_rc, readbuf, MAX_PACKET_SIZE) == 1)
            rc = connack_rc;
        else
            rc = FAILURE;
    }
    else
        rc = FAILURE;
        
#if MQTTCLIENT_QOS2
    // resend an inflight publish
    if (inflight_msgid >0 && inflight_QoS == QOS2 && pubrel)
    {
        if ((len = MQTTSerialize_ack(sendbuf, MAX_PACKET_SIZE, PUBREL, 0, inflight_msgid)) <= 0)
            rc = FAILURE;
        else
            rc = publish(len, connect_timer, inflight_QoS);
    }
    else
#endif
#if MQTTCLIENT_QOS1 || MQTTCLIENT_QOS2
    if (inflight_msgid > 0)
    {
        memcpy(sendbuf, pubbuf, MAX_PACKET_SIZE);
        rc = publish(inflight_len, connect_timer, inflight_QoS);
    }
#endif

exit:
    if (rc == SUCCESS)
        isconnected = true;
    return rc;
}

template<class network_type, class timer_type, int MAX_PACKET_SIZE, int b>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, b>::connect()
{
    MQTTSNPacket_connectData default_options = MQTTSNPacket_connectData_initializer;
    //default_options.duration = 800;
    //default_options.clientID.cstring = "ard1";
    return connect(default_options);
}

template<class network_type, class timer_type, int MAX_PACKET_SIZE, int b>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, b>::searchgw(uint8_t radius)
{
    timer_type searchgw_delay = timer_type();
    int rc = FAILURE;
    int len = 0;
    bool is_same_request = false;

    if ((len = MQTTSNSerialize_searchgw(sendbuf, MAX_PACKET_SIZE, radius)) <= 0)
        goto exit;
    searchgw_delay.random_start(MQTTSN_SEARCHGW_TIME);
    // random delay time before sending searchgw, if a searchgw request is found then 
    // cancel the searchgw request, sinse another client just send the resquest.
    //
    if (waitfor(MQTTSN_SEARCHGW, searchgw_delay) == MQTTSN_SEARCHGW)
    {
        unsigned char r = -1;
        if(MQTTSNDeserialize_searchgw(&r, readbuf, MAX_PACKET_SIZE) == 1)
        {
            if(r == radius)
                is_same_request = true;
        }
    }

    if(is_same_request)
    {
        searchgw_delay.start(MQTTSN_WAIT_TIME);
        if(waitfor(MQTTSN_GWINFO, searchgw_delay) == MQTTSN_GWINFO)
        {
            if(MQTTSNDeserialize_gwinfo(&active_gateways[active_gateways_count].id, 0, (unsigned char**)&active_gateways[active_gateways_count].addr_data, readbuf, MAX_PACKET_SIZE) == 1)
            {
                rc = SUCCESS;
                goto exit;
            }
        }
    }

    uint16_t count = 0;
    rc = FAILURE;
    for(uint8_t i = 0; i < MQTTSN_RETRY_COUNT; i++)
    {
        searchgw_delay.start(MQTTSN_WAIT_TIME);
        if ((rc = send_packet(len, searchgw_delay, SEND_REQUEST_BROADCAST)) != SUCCESS)  // send the searchgw packet
            goto exit; // there was a problem
        if(waitfor(MQTTSN_GWINFO, searchgw_delay) == MQTTSN_GWINFO)
        {
            rc = SUCCESS;
            goto exit;
        }
        else
        {
            goto exit;
        }
    }


#if MQTTCLIENT_QOS2
    // resend an inflight publish
    if (inflight_msgid >0 && inflight_QoS == QOS2 && pubrel)
    {
        if ((len = MQTTSerialize_ack(sendbuf, MAX_PACKET_SIZE, PUBREL, 0, inflight_msgid)) <= 0)
            rc = FAILURE;
        else
            rc = publish(len, connect_timer, inflight_QoS);
    }
    else
#endif
#if MQTTCLIENT_QOS1 || MQTTCLIENT_QOS2
    if (inflight_msgid > 0)
    {
        memcpy(sendbuf, pubbuf, MAX_PACKET_SIZE);
        rc = publish(inflight_len, connect_timer, inflight_QoS);
    }
#endif

exit:
    //memset(sendbuf, 0, MAX_PACKET_SIZE - 1);
    // if (rc == SUCCESS)
    // {
    //     PRINT("gateway id = ");
    //     for(uint8_t i = 0; i < 10; i++)
    //     {
    //         PRINT(active_gateways[current_active_gateway].addr_data[i], HEX);
    //         PRINT(" ");
    //     }
    //     PRINTLN("");
        
    //     //isconnected = true;
    // }
    return rc;
}

template<class network_type, class timer_type, int MAX_PACKET_SIZE, int MAX_MESSAGE_HANDLERS>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, MAX_MESSAGE_HANDLERS>::register_topic_name(const char* topicname, void* data_ptr, uint8_t data_size)
{
    timer_type timer = timer_type();
    int rc = FAILURE;
    int len = 0;

    if(!is_connected())
        return;

    MQTTSNString t = {(char*)topicname, {0, 0}};
    unsigned short pid = _packetid.get_next();
    if((len = MQTTSNSerialize_register(sendbuf, MAX_PACKET_SIZE, 0, pid, &t)) <= 0)
        goto exit;
    
    timer.start(MQTTSN_WAIT_TIME);
    
    if((rc = send_packet(len, timer, SEND_REQUEST_UNICAST)) != SUCCESS)
        goto exit;
    
    timer.start(MQTTSN_WAIT_TIME);
    if(waitfor(MQTTSN_REGACK, timer) == MQTTSN_REGACK)
    {
        unsigned short rec_pid = 0;
        uint8_t res = 0;
        if(MQTTSNDeserialize_regack(&registrations[current_reg].id, &rec_pid, &res, readbuf, MAX_PACKET_SIZE) == 1)
        {
            if(rec_pid != pid || res != 0)
                goto exit;
            registrations[current_reg].data_to_send = data_ptr;
            registrations[current_reg].data_size = data_size;
            current_reg++;
            rc = SUCCESS;
            goto exit;
        }
    }

exit:
    return rc;
}

template<class network_type, class timer_type, int MAX_PACKET_SIZE, int MAX_MESSAGE_HANDLERS>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, MAX_MESSAGE_HANDLERS>::subscribe(MQTTSN_topicid& topic_filter, enum QoS qos, message_handler_pfn message_handler_pfn)
{
    int rc = FAILURE;
    timer_type timer = timer_type();
    int len = 0;
    bool free_handler = false;

    if (!isconnected)
        goto exit;

    if (message_handlers_count == MAX_MESSAGE_HANDLERS)
    {                                 // No message handler free
        rc = MAX_SUBSCRIPTIONS_EXCEEDED;
        goto exit; 
    }

    len = MQTTSNSerialize_subscribe(sendbuf, MAX_PACKET_SIZE, 0, qos, _packetid.get_next(), &topic_filter);
    if (len <= 0)
        goto exit;
    
    timer.start(MQTTSN_WAIT_TIME);
    if ((rc = send_packet(len, timer, SEND_REQUEST_UNICAST)) != SUCCESS) // send the subscribe packet
        goto exit;             // there was a problem

    if (waitfor(MQTTSN_SUBACK, timer) == MQTTSN_SUBACK)      // wait for suback
    {
        int grantedQoS = -1;
        unsigned short mypacketid;
        unsigned char rc;
        if (MQTTSNDeserialize_suback(&grantedQoS, &message_handlers[message_handlers_count].topic_filter.data.id, &mypacketid, &rc, readbuf, MAX_PACKET_SIZE) == 1)
            rc = grantedQoS;
        if (rc == MQTTSN_RC_ACCEPTED)
        {
            if(message_handlers_count < MAX_MESSAGE_HANDLERS)
            {
                message_handlers[message_handlers_count].fp.attach(message_handler_pfn);
                rc = 0;
                message_handlers_count++;
            }
            // for (int i = 0; i < MAX_MESSAGE_HANDLERS; ++i)
            // {
            //     // if (message_handlers[i].topic_filter == 0)
            //     // {
            //     //     message_handlers[i].topic_filter = &topic_filter;
            //     //     message_handlers[i].fp.attach(message_handler_pfn);
            //     //     rc = 0;
            //     //     break;
            //     // }
            // }
        }
    }
    else
        rc = FAILURE;

exit:
    if (rc != SUCCESS)
        isconnected = false;
    return rc;
}

template<class network_type, class timer_type, int MAX_PACKET_SIZE, int MAX_MESSAGE_HANDLERS>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, MAX_MESSAGE_HANDLERS>::unsubscribe(MQTTSN_topicid& topic_filter)
{
    int rc = FAILURE;
    timer_type timer = timer_type();
    int len = 0;

    if (!isconnected)
        goto exit;

    if ((len = MQTTSNSerialize_unsubscribe(sendbuf, MAX_PACKET_SIZE, _packetid.get_next(), &topic_filter)) <= 0)
        goto exit;
    timer.start(MQTTSN_WAIT_TIME);
    if ((rc = send_packet(len, timer, SEND_REQUEST_UNICAST)) != SUCCESS) // send the unsubscribe packet
        goto exit; // there was a problem

    if (waitfor(MQTTSN_UNSUBACK, timer) == MQTTSN_UNSUBACK)
    {
        unsigned short mypacketid;  // should be the same as the packetid above
        if (MQTTSNDeserialize_unsuback(&mypacketid, readbuf, MAX_PACKET_SIZE) == 1)
            rc = 0;
    }
    else
        rc = FAILURE;

exit:
    if (rc != SUCCESS)
        isconnected = false;
    return rc;
}

template<class network_type, class timer_type, int MAX_PACKET_SIZE, int b>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, b>::publish(int len, timer_type& timer, enum QoS qos)
{
    int rc;
    
    if ((rc = send_packet(len, timer, SEND_REQUEST_UNICAST)) != SUCCESS) // send the publish packet
        goto exit; // there was a problem

#if MQTTCLIENT_QOS1 
    if (qos == QOS1)
    {
        if (waitfor(MQTTSN_PUBACK, timer) == MQTTSN_PUBACK)
        {
            unsigned short mypacketid;
            unsigned char type;
            if (MQTTSNDeserialize_ack(&type, &mypacketid, readbuf, MAX_PACKET_SIZE) != 1)
                rc = FAILURE;
            else if (inflight_msgid == mypacketid)
                inflight_msgid = 0;
        }
        else
            rc = FAILURE;
    }
#elif MQTTCLIENT_QOS2
    else if (qos == QOS2)
    {
        if (waitfor(PUBCOMP, timer) == PUBCOMP)
        {
            unsigned short mypacketid;
            unsigned char type;
            if (MQTTDeserialize_ack(&type, &mypacketid, readbuf, MAX_PACKET_SIZE) != 1)
                rc = FAILURE;
            else if (inflight_msgid == mypacketid)
                inflight_msgid = 0;
        }
        else
            rc = FAILURE;
    }
#endif

exit:
    if (rc != SUCCESS)
        isconnected = false;
    return rc;
}

template<class network_type, class timer_type, int MAX_PACKET_SIZE, int b>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, b>::publish(MQTTSN_topicid& topic, void* payload, size_t payloadlen, unsigned short& id, enum QoS qos, bool retained)
{
    int rc = FAILURE;
    timer_type timer = timer_type();
    int len = 0;

    if (!isconnected)
        goto exit;

#if MQTTCLIENT_QOS1 || MQTTCLIENT_QOS2
    if (qos == QOS1 || qos == QOS2)
        id = _packetid.get_next();
#endif

    len = MQTTSNSerialize_publish(sendbuf, MAX_PACKET_SIZE, 0, qos, retained, id,
              topic, (unsigned char*)payload, payloadlen);
    if (len <= 0)
        goto exit;
        
#if MQTTCLIENT_QOS1 || MQTTCLIENT_QOS2
    if (!cleansession)
    {
        memcpy(pubbuf, sendbuf, len);
        inflight_msgid = id;
        inflight_len = len;
        inflight_QoS = qos;
#if MQTTCLIENT_QOS2
        pubrel = false;
#endif
    }
#endif
    timer.start(MQTTSN_WAIT_TIME);
    rc = publish(len, timer, qos);
exit:
    return rc;
}

template<class network_type, class timer_type, int MAX_PACKET_SIZE, int b>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, b>::publish(MQTTSN_topicid& topic_name, void* payload, size_t payloadlen, enum QoS qos, bool retained)
{
    unsigned short id = 0;  // dummy - not used for anything
    return publish(topic_name, payload, payloadlen, id, qos, retained);
}

template<class network_type, class timer_type, int MAX_PACKET_SIZE, int b>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, b>::publish(MQTTSN_topicid& topic_name, message_t& message)
{
    return publish(topic_name, message.payload, message.payloadlen, message.qos, message.retained);
}

template<class network_type, class timer_type, int MAX_PACKET_SIZE, int b>
void mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, b>::publish_topics()
{
    for(uint8_t i = 0; i < current_reg; i++)
    {
        MQTTSN_topicid topicid;
        topicid.type = MQTTSN_TOPIC_TYPE_NORMAL;
        topicid.data.id = registrations[i].id;
        publish(topicid, registrations[i].data_to_send, registrations[i].data_size, QOS0, false);
    }
}

template<class network_type, class timer_type, int MAX_PACKET_SIZE, int b>
int mqttsn::client_t<network_type, timer_type, MAX_PACKET_SIZE, b>::disconnect(unsigned short duration)
{
    int rc = FAILURE;
    timer_type timer = timer_type(MQTTSN_WAIT_TIME);     // we might wait for incomplete incoming publishes to complete
    int int_duration = (duration == 0) ? -1 : (int)duration;
    int len = MQTTSNSerialize_disconnect(sendbuf, MAX_PACKET_SIZE, int_duration);
    if (len > 0)
        rc = send_packet(len, timer, SEND_REQUEST_UNICAST);            // send the disconnect packet

    isconnected = false;
    return rc;
}


#endif

