/*
 * GenericDriver.cpp
 *
 *  Created on: May 3, 2022
 *      Author: noahr
 */

#include "stm32l4xx_hal.h"
#include "GenericDriver.h"
#include <stdlib.h>

GenericDriver::GenericDriver()
    :
    _mode(ModeInitialising),
    _thisAddress(BROADCAST_ADDRESS),
    _txHeaderTo(BROADCAST_ADDRESS),
    _txHeaderFrom(BROADCAST_ADDRESS),
    _txHeaderId(0),
    _txHeaderFlags(0),
    _rxBad(0),
    _rxGood(0),
    _txGood(0),
    _cad_timeout(0)
{
}

bool GenericDriver::init()
{
    return true;
}

// Blocks until a valid message is received
void GenericDriver::waitAvailable(uint16_t polldelay)
{
    while (!available())
      {
	YIELD;
	if (polldelay)
		HAL_Delay(polldelay);
      }
}

// Blocks until a valid message is received or timeout expires
// Return true if there is a message available
// Works correctly even on millis() rollover
bool GenericDriver::waitAvailableTimeout(uint16_t timeout, uint16_t polldelay)
{
    unsigned long starttime = HAL_GetTick();
    while ((HAL_GetTick() - starttime) < timeout)
    {
        if (available())
	{
           return true;
	}
	YIELD;
	if (polldelay)
		HAL_Delay(polldelay);
    }
    return false;
}

bool GenericDriver::waitPacketSent()
{
    while (_mode == ModeTx)
	YIELD; // Wait for any previous transmit to finish
    return true;
}

bool GenericDriver::waitPacketSent(uint16_t timeout)
{
    unsigned long starttime = HAL_GetTick();
    while ((HAL_GetTick() - starttime) < timeout)
    {
        if (_mode != ModeTx) // Any previous transmit finished?
           return true;
	YIELD;
    }
    return false;
}

// Wait until no channel activity detected or timeout
bool GenericDriver::waitCAD()
{
    if (!_cad_timeout)
	return true;

    // Wait for any channel activity to finish or timeout
    // Sophisticated DCF function...
    // DCF : BackoffTime = random() x aSlotTime
    // 100 - 1000 ms
    // 10 sec timeout
    unsigned long t = HAL_GetTick();
    while (isChannelActive())
    {
         if (HAL_GetTick() - t > _cad_timeout)
	     return false;
         HAL_Delay((rand() % 10 + 1) * 100);
    }

    return true;
}

// subclasses are expected to override if CAD is available for that radio
bool GenericDriver::isChannelActive()
{
    return false;
}

void GenericDriver::setPromiscuous(bool promiscuous)
{
    _promiscuous = promiscuous;
}

void GenericDriver::setThisAddress(uint8_t address)
{
    _thisAddress = address;
}

void GenericDriver::setHeaderTo(uint8_t to)
{
    _txHeaderTo = to;
}

void GenericDriver::setHeaderFrom(uint8_t from)
{
    _txHeaderFrom = from;
}

void GenericDriver::setHeaderId(uint8_t id)
{
    _txHeaderId = id;
}

void GenericDriver::setHeaderFlags(uint8_t set, uint8_t clear)
{
    _txHeaderFlags &= ~clear;
    _txHeaderFlags |= set;
}

uint8_t GenericDriver::headerTo()
{
    return _rxHeaderTo;
}

uint8_t GenericDriver::headerFrom()
{
    return _rxHeaderFrom;
}

uint8_t GenericDriver::headerId()
{
    return _rxHeaderId;
}

uint8_t GenericDriver::headerFlags()
{
    return _rxHeaderFlags;
}

int16_t GenericDriver::lastRssi()
{
    return _lastRssi;
}

GenericDriver::Mode  GenericDriver::mode()
{
    return _mode;
}

void  GenericDriver::setMode(Mode mode)
{
    _mode = mode;
}

bool  GenericDriver::sleep()
{
    return false;
}

// Diagnostic help
void GenericDriver::printBuffer(const char* prompt, const uint8_t* buf, uint8_t len)
{
	// use usart to print this
//#ifdef RH_HAVE_SERIAL
//    Serial.println(prompt);
//    uint8_t i;
//    for (i = 0; i < len; i++)
//    {
//	if (i % 16 == 15)
//	    Serial.println(buf[i], HEX);
//	else
//	{
//	    Serial.print(buf[i], HEX);
//	    Serial.print(' ');
//	}
//    }
//    Serial.println("");
//#endif
}

uint16_t GenericDriver::rxBad()
{
    return _rxBad;
}

uint16_t GenericDriver::rxGood()
{
    return _rxGood;
}

uint16_t GenericDriver::txGood()
{
    return _txGood;
}

void GenericDriver::setCADTimeout(unsigned long cad_timeout)
{
    _cad_timeout = cad_timeout;
}
