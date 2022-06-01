/*
 * ASK.h
 *
 *  Created on: May 3, 2022
 *      Author: noahr
 */

#ifndef INC_ASK_H_
#define INC_ASK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "GenericDriver.h"

#define ASK_MAX_PAYLOAD_LEN 67
#define ASK_HEADER_LEN 4
#ifndef ASK_MAX_MESSAGE_LEN
	#define ASK_MAX_MESSAGE_LEN (ASK_MAX_PAYLOAD_LEN - ASK_HEADER_LEN - 3)
#endif

#ifndef ASK_RX_SAMPLES_PER_BIT
	#define ASK_RX_SAMPLES_PER_BIT 8
#endif

#define ASK_RX_RAMP_LEN 160

// Ramp adjustment parameters
// Standard is if a transition occurs before RH_ASK_RAMP_TRANSITION (80) in the ramp,
// the ramp is retarded by adding RH_ASK_RAMP_INC_RETARD (11)
// else by adding RH_ASK_RAMP_INC_ADVANCE (29)
// If there is no transition it is adjusted by RH_ASK_RAMP_INC (20)
/// Internal ramp adjustment parameter
#define ASK_RAMP_INC (ASK_RX_RAMP_LEN/ASK_RX_SAMPLES_PER_BIT)
/// Internal ramp adjustment parameter
#define ASK_RAMP_TRANSITION ASK_RX_RAMP_LEN/2
/// Internal ramp adjustment parameter
#define ASK_RAMP_ADJUST 9
/// Internal ramp adjustment parameter
#define ASK_RAMP_INC_RETARD (ASK_RAMP_INC-ASK_RAMP_ADJUST)
/// Internal ramp adjustment parameter
#define ASK_RAMP_INC_ADVANCE (ASK_RAMP_INC+ASK_RAMP_ADJUST)

#define ASK_PREAMBLE_LEN 8

class ASK : public GenericDriver
{
public:
	ASK(uint16_t speed=2000, uint8_t rxPin = 11, uint8_t txPin = 12, uint8_t pttPin = 10, bool pttInverted = false);
    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool    init();

    /// Tests whether a new message is available
    /// from the Driver.
    /// On most drivers, this will also put the Driver into RHModeRx mode until
    /// a message is actually received bythe transport, when it wil be returned to RHModeIdle.
    /// This can be called multiple times in a timeout loop
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    virtual bool    available();

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    INTERRUPT_ATTR virtual bool    recv(uint8_t* buf, uint8_t* len);

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is NOT permitted.
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send (> 0)
    /// \return true if the message length was valid and it was correctly queued for transmit
    virtual bool    send(const uint8_t* data, uint8_t len);

    /// Returns the maximum message length
    /// available in this Driver.
    /// \return The maximum legal message length
    virtual uint8_t maxMessageLength();

    /// If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running,
    /// disables them.
    INTERRUPT_ATTR void           setModeIdle();

    /// If current mode is Tx or Idle, changes it to Rx.
    /// Starts the receiver in the RF69.
    INTERRUPT_ATTR void           setModeRx();

    void handleTimerInterrupt();

    /// If current mode is Rx or Idle, changes it to Rx. F
    /// Starts the transmitter in the RF69.
    void           setModeTx();

    /// Returns the current speed in bits per second
    /// \return The current speed in bits per second
    uint16_t        speed() { return _speed;}


protected:

    /// Read the rxPin in a platform dependent way, taking into account whether it is inverted or not
    INTERRUPT_ATTR bool            readRx();

    /// Write the txPin in a platform dependent way
    void            writeTx(bool value);

    /// Write the txPin in a platform dependent way, taking into account whether it is inverted or not
    void            writePtt(bool value);

    /// Translates a 6 bit symbol to its 4 bit plaintext equivalent
    INTERRUPT_ATTR uint8_t         symbol_6to4(uint8_t symbol);

    /// The receiver handler function, called a 8 times the bit rate
    void            receiveTimer();

    /// The transmitter handler function, called a 8 times the bit rate
    void            transmitTimer();

    /// Check whether the latest received message is complete and uncorrupted
    /// We should always check the FCS at user level, not interrupt level
    /// since it is slow
    void            validateRxBuf();

    /// Configure bit rate in bits per second
    uint16_t        _speed;

    /// The configure receiver pin
    uint8_t _rxPin;

    /// The configure transmitter pin
    uint8_t         _txPin;

    /// The configured transmitter enable pin
    uint8_t         _pttPin;

    /// True of the sense of the rxPin is to be inverted
    bool            _rxInverted;

    /// True of the sense of the pttPin is to be inverted
    bool            _pttInverted;

    // Used in the interrupt handlers
    /// Buf is filled but not validated
    volatile bool   _rxBufFull;

    /// Buf is full and valid
    volatile bool   _rxBufValid;

    /// Last digital input from the rx data pin
    volatile bool   _rxLastSample;

    /// This is the integrate and dump integral. If there are <5 0 samples in the PLL cycle
    /// the bit is declared a 0, else a 1
    volatile uint8_t _rxIntegrator;

    /// PLL ramp, varies between 0 and RH_ASK_RX_RAMP_LEN-1 (159) over
    /// RH_ASK_RX_SAMPLES_PER_BIT (8) samples per nominal bit time.
    /// When the PLL is synchronised, bit transitions happen at about the
    /// 0 mark.
    volatile uint8_t _rxPllRamp;

    /// Flag indicates if we have seen the start symbol of a new message and are
    /// in the processes of reading and decoding it
    volatile uint8_t _rxActive;

    /// Last 12 bits received, so we can look for the start symbol
    volatile uint16_t _rxBits;

    /// How many bits of message we have received. Ranges from 0 to 12
    volatile uint8_t _rxBitCount;

    /// The incoming message buffer
    uint8_t _rxBuf[ASK_MAX_PAYLOAD_LEN];

    /// The incoming message expected length
    volatile uint8_t _rxCount;

    /// The incoming message buffer length received so far
    volatile uint8_t _rxBufLen;

    /// Index of the next symbol to send. Ranges from 0 to vw_tx_len
    uint8_t _txIndex;

    /// Bit number of next bit to send
    uint8_t _txBit;

    /// Sample number for the transmitter. Runs 0 to 7 during one bit interval
    uint8_t _txSample;

    /// The transmitter buffer in _symbols_ not data octets
    uint8_t _txBuf[(ASK_MAX_PAYLOAD_LEN * 2) + ASK_PREAMBLE_LEN];

    /// Number of symbols in _txBuf to be sent;
    uint8_t _txBufLen;
};

#ifdef __cplusplus
}
#endif

#endif /* INC_ASK_H_ */
