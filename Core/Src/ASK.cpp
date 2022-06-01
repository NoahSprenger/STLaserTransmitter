/*
 * ASK.cpp
 *
 *  Created on: May 3, 2022
 *      Author: noahr
 */
#include "stm32l4xx_hal.h"
#include "ASK.h"
#include <CRC.h>

// use timer from main.c

static ASK* thisASKDriver;

#define DRAM_ATTR

DRAM_ATTR static uint8_t symbols[] =
{
    0xd,  0xe,  0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
    0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

// This is the value of the start symbol after 6-bit conversion and nybble swapping
#define ASK_START_SYMBOL 0xb38

ASK::ASK(uint16_t speed, uint8_t rxPin, uint8_t txPin, uint8_t pttPin, bool pttInverted)
    :
    _speed(speed),
    _rxPin(rxPin),
    _txPin(txPin),
    _pttPin(pttPin),
    _rxInverted(false),
    _pttInverted(pttInverted)
{
    // Initialise the first 8 nibbles of the tx buffer to be the standard
    // preamble. We will append messages after that. 0x38, 0x2c is the start symbol before
    // 6-bit conversion to RH_ASK_START_SYMBOL
    uint8_t preamble[ASK_PREAMBLE_LEN] = {0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x38, 0x2c};
    memcpy(_txBuf, preamble, sizeof(preamble));
}

bool ASK::init()
{
    if (!GenericDriver::init())
	return false;
    thisASKDriver = this;

    // Set up digital IO pins for arduino -> this should be done in main.cpp
//    pinMode(_txPin, OUTPUT);
//    pinMode(_rxPin, INPUT);
//    pinMode(_pttPin, OUTPUT);

    // Ready to go
    setModeIdle();
//    timerSetup(); // should be handled by main.c

    return true;
}

//// Common function for setting timer ticks @ prescaler values for speed
//// Returns prescaler index into {0, 1, 8, 64, 256, 1024} array
//// and sets nticks to compare-match value if lower than max_ticks
//// returns 0 & nticks = 0 on fault
//uint8_t ASK::timerCalc(uint16_t speed, uint16_t max_ticks, uint16_t *nticks)
//{
//    return 0; // not implemented or needed on other platforms
//}

//// Handled in Main.c
//// The idea here is to get 8 timer interrupts per bit period
//void ASK::timerSetup()
//{
//    HAL_TIM_BASE_Stop(&htim5); // pause timer
//    uint16_t us=(HAL_RCC_GETHCLKFreq()/8)/_speed;
//    htim5.Instance->ARR = us;
//    htim5.Instance->CCR1 = us - 1;
//    //    timer.setMode(1, TIMER_OUTPUT_COMPARE); // should be done in main.c setup
////    timer.setOverflow(us, MICROSEC_FORMAT);
////    timer.setCaptureCompare(1, us - 1, MICROSEC_COMPARE_FORMAT);
////    timer.attachInterrupt(interrupt); // should be attached in main.c
//}

void INTERRUPT_ATTR ASK::setModeIdle()
{
    if (_mode != ModeIdle)
    {
	// Disable the transmitter hardware
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		_mode = ModeIdle;
    }
}

void ASK::setModeTx()
{
    if (_mode != ModeTx)
    {
	// PRepare state varibles for a new transmission
	_txIndex = 0;
	_txBit = 0;
	_txSample = 0;

	// Enable the transmitter hardware
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

	_mode = ModeTx;
    }
}

void ASK::setModeRx() {
	if (_mode != ModeRx) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		_mode = ModeIdle;
	}
}

// Call this often
bool ASK::available()
{
    if (_mode == ModeTx)
	return false;
    setModeRx();
    if (_rxBufFull)
    {
	validateRxBuf();
	_rxBufFull= false;
    }
    return _rxBufValid;
}

bool INTERRUPT_ATTR ASK::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;

    if (buf && len)
    {
	// Skip the length and 4 headers that are at the beginning of the rxBuf
	// and drop the trailing 2 bytes of FCS
	uint8_t message_len = _rxBufLen-ASK_HEADER_LEN - 3;
	if (*len > message_len)
	    *len = message_len;
	memcpy(buf, _rxBuf+ASK_HEADER_LEN+1, *len);
    }
    _rxBufValid = false; // Got the most recent message, delete it
//    printBuffer("recv:", buf, *len);
    return true;
}

// Caution: this may block
bool ASK::send(const uint8_t* data, uint8_t len)
{
    uint8_t i;
    uint16_t index = 0;
    uint16_t crc = 0xffff;
    uint8_t *p = _txBuf + ASK_PREAMBLE_LEN; // start of the message area
    uint8_t count = len + 3 + ASK_HEADER_LEN; // Added byte count and FCS and headers to get total number of bytes

    if (len > ASK_MAX_MESSAGE_LEN)
	return false;

    // Wait for transmitter to become available
    waitPacketSent();

    if (!waitCAD())
	return false;  // Check channel activity

    // Encode the message length
    crc = crc_ccitt_update(crc, count);
    p[index++] = symbols[count >> 4];
    p[index++] = symbols[count & 0xf];

    // Encode the headers
    crc = crc_ccitt_update(crc, _txHeaderTo);
    p[index++] = symbols[_txHeaderTo >> 4];
    p[index++] = symbols[_txHeaderTo & 0xf];
    crc = crc_ccitt_update(crc, _txHeaderFrom);
    p[index++] = symbols[_txHeaderFrom >> 4];
    p[index++] = symbols[_txHeaderFrom & 0xf];
    crc = crc_ccitt_update(crc, _txHeaderId);
    p[index++] = symbols[_txHeaderId >> 4];
    p[index++] = symbols[_txHeaderId & 0xf];
    crc = crc_ccitt_update(crc, _txHeaderFlags);
    p[index++] = symbols[_txHeaderFlags >> 4];
    p[index++] = symbols[_txHeaderFlags & 0xf];

    // Encode the message into 6 bit symbols. Each byte is converted into
    // 2 6-bit symbols, high nybble first, low nybble second
    for (i = 0; i < len; i++)
    {
	crc = crc_ccitt_update(crc, data[i]);
	p[index++] = symbols[data[i] >> 4];
	p[index++] = symbols[data[i] & 0xf];
    }

    // Append the fcs, 16 bits before encoding (4 6-bit symbols after encoding)
    // Caution: VW expects the _ones_complement_ of the CCITT CRC-16 as the FCS
    // VW sends FCS as low byte then hi byte
    crc = ~crc;
    p[index++] = symbols[(crc >> 4)  & 0xf];
    p[index++] = symbols[crc & 0xf];
    p[index++] = symbols[(crc >> 12) & 0xf];
    p[index++] = symbols[(crc >> 8)  & 0xf];

    // Total number of 6-bit symbols to send
    _txBufLen = index + ASK_PREAMBLE_LEN;

    // Start the low level interrupt handler sending symbols
    setModeTx();

    return true;
}

// Read the RX data input pin, taking into account platform type and inversion.
bool INTERRUPT_ATTR ASK::readRx()
{
    bool value;
    value = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
    return value ^ _rxInverted;
}

// Write the TX output pin, taking into account platform type.
void INTERRUPT_ATTR ASK::writeTx(bool value)
{
    if (value == true) {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    }
    else {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    }
}

void INTERRUPT_ATTR ASK::writePtt(bool value) {
	if (value == true) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	}
}

uint8_t ASK::maxMessageLength()
{
    return ASK_MAX_MESSAGE_LEN;
}

// Convert a 6 bit encoded symbol into its 4 bit decoded equivalent
uint8_t INTERRUPT_ATTR ASK::symbol_6to4(uint8_t symbol)
{
    uint8_t i;
    uint8_t count;

    // Linear search :-( Could have a 64 byte reverse lookup table?
    // There is a little speedup here courtesy Ralph Doncaster:
    // The shortcut works because bit 5 of the symbol is 1 for the last 8
    // symbols, and it is 0 for the first 8.
    // So we only have to search half the table
    for (i = (symbol>>2) & 8, count=8; count-- ; i++)
	if (symbol == symbols[i]) return i;

    return 0; // Not found
}

// Check whether the latest received message is complete and uncorrupted
// We should always check the FCS at user level, not interrupt level
// since it is slow
void ASK::validateRxBuf()
{
    uint16_t crc = 0xffff;
    // The CRC covers the byte count, headers and user data
    for (uint8_t i = 0; i < _rxBufLen; i++)
	crc = crc_ccitt_update(crc, _rxBuf[i]);
    if (crc != 0xf0b8) // CRC when buffer and expected CRC are CRC'd
    {
	// Reject and drop the message
	_rxBad++;
	_rxBufValid = false;
	return;
    }

    // Extract the 4 headers that follow the message length
    _rxHeaderTo    = _rxBuf[1];
    _rxHeaderFrom  = _rxBuf[2];
    _rxHeaderId    = _rxBuf[3];
    _rxHeaderFlags = _rxBuf[4];
    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == BROADCAST_ADDRESS)
    {
	_rxGood++;
	_rxBufValid = true;
    }
}


void INTERRUPT_ATTR ASK::receiveTimer()
{
    bool rxSample = readRx();

    // Integrate each sample
    if (rxSample)
	_rxIntegrator++;

    if (rxSample != _rxLastSample)
    {
	// Transition, advance if ramp > 80, retard if < 80
	_rxPllRamp += ((_rxPllRamp < ASK_RAMP_TRANSITION)
			   ? ASK_RAMP_INC_RETARD
			   : ASK_RAMP_INC_ADVANCE);
	_rxLastSample = rxSample;
    }
    else
    {
	// No transition
	// Advance ramp by standard 20 (== 160/8 samples)
	_rxPllRamp += ASK_RAMP_INC;
    }
    if (_rxPllRamp >= ASK_RX_RAMP_LEN)
    {
	// Add this to the 12th bit of _rxBits, LSB first
	// The last 12 bits are kept
	_rxBits >>= 1;

	// Check the integrator to see how many samples in this cycle were high.
	// If < 5 out of 8, then its declared a 0 bit, else a 1;
	if (_rxIntegrator >= 5)
	    _rxBits |= 0x800;

	_rxPllRamp -= ASK_RX_RAMP_LEN;
	_rxIntegrator = 0; // Clear the integral for the next cycle

	if (_rxActive)
	{
	    // We have the start symbol and now we are collecting message bits,
	    // 6 per symbol, each which has to be decoded to 4 bits
	    if (++_rxBitCount >= 12)
	    {
		// Have 12 bits of encoded message == 1 byte encoded
		// Decode as 2 lots of 6 bits into 2 lots of 4 bits
		// The 6 lsbits are the high nybble
		uint8_t this_byte =
		    (symbol_6to4(_rxBits & 0x3f)) << 4
		    | symbol_6to4(_rxBits >> 6);

		// The first decoded byte is the byte count of the following message
		// the count includes the byte count and the 2 trailing FCS bytes
		// REVISIT: may also include the ACK flag at 0x40
		if (_rxBufLen == 0)
		{
		    // The first byte is the byte count
		    // Check it for sensibility. It cant be less than 7, since it
		    // includes the byte count itself, the 4 byte header and the 2 byte FCS
		    _rxCount = this_byte;
		    if (_rxCount < 7 || _rxCount > ASK_MAX_PAYLOAD_LEN)
		    {
			// Stupid message length, drop the whole thing
			_rxActive = false;
			_rxBad++;
                        return;
		    }
		}
		_rxBuf[_rxBufLen++] = this_byte;

		if (_rxBufLen >= _rxCount)
		{
		    // Got all the bytes now
		    _rxActive = false;
		    _rxBufFull = true;
		    setModeIdle();
		}
		_rxBitCount = 0;
	    }
	}
	// Not in a message, see if we have a start symbol
	else if (_rxBits == ASK_START_SYMBOL)
	{
	    // Have start symbol, start collecting message
	    _rxActive = true;
	    _rxBitCount = 0;
	    _rxBufLen = 0;
	}
    }
}

void INTERRUPT_ATTR ASK::transmitTimer()
{
    if (_txSample++ == 0)
    {
	// Send next bit
	// Symbols are sent LSB first
	// Finished sending the whole message? (after waiting one bit period
	// since the last bit)
	if (_txIndex >= _txBufLen)
	{
	    setModeIdle();
	    _txGood++;
	}
	else
	{
	    writeTx(_txBuf[_txIndex] & (1 << _txBit++));
	    if (_txBit >= 6)
	    {
		_txBit = 0;
		_txIndex++;
	    }
	}
    }

    if (_txSample > 7)
	_txSample = 0;
}

void ASK::handleTimerInterrupt() {
	if (_mode == ModeRx)
		receiveTimer();
	else if (_mode == ModeTx)
		transmitTimer();
}




