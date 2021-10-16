// /**************************************************************************************
//  * Copyright (c) 2016, Tomoaki Yamaguchi
//  *
//  * All rights reserved. This program and the accompanying materials
//  * are made available under the terms of the Eclipse Public License v1.0
//  * and Eclipse Distribution License v1.0 which accompany this distribution.
//  *
//  * The Eclipse Public License is available at
//  *    http://www.eclipse.org/legal/epl-v10.html
//  * and the Eclipse Distribution License is available at
//  *   http://www.eclipse.org/org/documents/edl-v10.php.
//  *
//  * Contributors:
//  *    Tomoaki Yamaguchi - initial API and implementation 
//  **************************************************************************************/

#include "SensorNetwork.h"

// /*===========================================
//  Class  SensorNetAddreess
//  ============================================*/

void mqttsn::sensor_net_address_t::set_address(uint8_t* address64, uint8_t* address16)
{
	memcpy(_address64, address64, 8);
	memcpy(_address16, address16, 2);
}


int mqttsn::sensor_net_address_t::set_address(const char* address64)
{
	memcpy(_address64, address64, 8);
	memset(_address16, 0, sizeof(_address16));
	return 0;
}

void mqttsn::sensor_net_address_t::set_broadcast_address(void)
{
	memset(_address64, 0, 6);
	_address64[6] = 0xff;
	_address64[7] = 0xff;
	_address16[0] = 0xff;
	_address16[1] = 0xfe;
}

bool mqttsn::sensor_net_address_t::is_match(sensor_net_address_t* addr)
{

	return (memcmp(this->_address64, addr->_address64, 8 ) == 0 &&  memcmp(this->_address16, addr->_address16, 2) == 0);
}

mqttsn::sensor_net_address_t& mqttsn::sensor_net_address_t::operator =(sensor_net_address_t& addr)
{
	memcpy(_address64, addr._address64, 8);
	memcpy(_address16, addr._address16, 2);
	return *this;
}



