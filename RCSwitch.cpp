/*
  RCSwitch - Arduino libary for remote control outlet switches
  Copyright (c) 2011 Suat Özgür.  All right reserved.
  
  Contributors:
  - Andre Koehler / info(at)tomate-online(dot)de
  - Gordeev Andrey Vladimirovich / gordeev(at)openpyro(dot)com
  - Skineffect / http://forum.ardumote.com/viewtopic.php?f=2&t=46
  - Dominik Fischer / dom_fischer(at)web(dot)de
  - Frank Oltmanns / <first name>.<last name>(at)gmail(dot)com
  - Andreas Steinel / A.<lastname>(at)gmail(dot)com
  - Max Horn / max(at)quendi(dot)de
  - Robert ter Vehn / <first name>.<last name>(at)gmail(dot)com
  - Johann Richard / <first name>.<last name>(at)gmail(dot)com
  - Vlad Gheorghe / <first name>.<last name>(at)gmail(dot)com https://github.com/vgheo
  - Martin Laclaustra / <first name>.<last name>(at)gmail(dot)com
  - Matias Cuenca-Acuna
  - Anthony Casagrande

  Project home: https://github.com/sui77/rc-switch/

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <functional>
#include "RCSwitch.h"

#ifdef RaspberryPi
    // PROGMEM and _P functions are for AVR based microprocessors,
    // so we must normalize these for the ARM processor:
    #define PROGMEM
    #define memcpy_P(dest, src, num) memcpy((dest), (src), (num))
#endif


/* Format for protocol definitions:
 * {pulselength, Sync bit, "0" bit, "1" bit, invertedSignal}
 * 
 * pulselength: pulse length in microseconds, e.g. 350
 * Sync bit: {1, 31} means 1 high pulse and 31 low pulses
 *     (perceived as a 31*pulselength long pulse, total length of sync bit is
 *     32*pulselength microseconds), i.e:
 *      _
 *     | |_______________________________ (don't count the vertical bars)
 * "0" bit: waveform for a data bit of value "0", {1, 3} means 1 high pulse
 *     and 3 low pulses, total length (1+3)*pulselength, i.e:
 *      _
 *     | |___
 * "1" bit: waveform for a data bit of value "1", e.g. {3,1}:
 *      ___
 *     |   |_
 *
 * These are combined to form Tri-State bits when sending or receiving codes.
 */
#if defined(ESP8266) || defined(ESP32)
static const VAR_ISR_ATTR RCSwitch::Protocol proto[] = {
#else
static const RCSwitch::Protocol PROGMEM proto[] = {
#endif
  { 350, {  1, 31 }, {  1,  3 }, {  3,  1 }, false },    // protocol 1
  { 650, {  1, 10 }, {  1,  2 }, {  2,  1 }, false },    // protocol 2
  { 100, { 30, 71 }, {  4, 11 }, {  9,  6 }, false },    // protocol 3
  { 380, {  1,  6 }, {  1,  3 }, {  3,  1 }, false },    // protocol 4
  { 500, {  6, 14 }, {  1,  2 }, {  2,  1 }, false },    // protocol 5
  { 450, { 23,  1 }, {  1,  2 }, {  2,  1 }, true },     // protocol 6 (HT6P20B)
  { 150, {  2, 62 }, {  1,  6 }, {  6,  1 }, false },    // protocol 7 (HS2303-PT, i. e. used in AUKEY Remote)
  { 200, {  3, 130}, {  7, 16 }, {  3,  16}, false},     // protocol 8 Conrad RS-200 RX
  { 200, { 130, 7 }, {  16, 7 }, { 16,  3 }, true},      // protocol 9 Conrad RS-200 TX
  { 365, { 18,  1 }, {  3,  1 }, {  1,  3 }, true },     // protocol 10 (1ByOne Doorbell)
  { 270, { 36,  1 }, {  1,  2 }, {  2,  1 }, true },     // protocol 11 (HT12E)
  { 320, { 36,  1 }, {  1,  2 }, {  2,  1 }, true },      // protocol 12 (SM5212)
  { 130, { 1,  80 }, {  2,  7 }, {  6,  3 }, false },      // protocol 13 (S72-RC11U, Fosmon)
  { 189, { 1,  31 }, {  1,  3 }, {  3,  1 }, false }      // protocol 14 (Q92-BH-V, dewenwils)
};

enum {
   numProto = sizeof(proto) / sizeof(proto[0])
};

RCSwitch::RCSwitch() {
  this->nTransmitterPin = -1;
  this->nRepeatTransmit = 10;
  this->setProtocol(1);
  #if not defined( RCSwitchDisableReceiving )
    this->nReceiverInterrupt = -1;
    this->nReceiveTolerance = 60;
    this->nReceivedValue = 0;
  #endif
}

/**
  * Sets the protocol to send.
  */
void RCSwitch::setProtocol(Protocol _protocol) {
  this->protocol = _protocol;
}

/**
  * Sets the protocol to send, from a list of predefined protocols
  */
void RCSwitch::setProtocol(int nProtocol) {
  if (nProtocol < 1 || nProtocol > numProto) {
    nProtocol = 1;  // TODO: trigger an error, e.g. "bad protocol" ???
  }
#if defined(ESP8266) || defined(ESP32)
  memcpy_P(&this->protocol, &proto[nProtocol-1], sizeof(Protocol));
#else
  memcpy_P(&this->protocol, &proto[nProtocol-1], sizeof(Protocol));
#endif
}

/**
  * Sets the protocol to send with pulse length in microseconds.
  */
void RCSwitch::setProtocol(int nProtocol, int nPulseLength) {
  setProtocol(nProtocol);
  this->setPulseLength(nPulseLength);
}


/**
  * Sets pulse length in microseconds
  */
void RCSwitch::setPulseLength(int nPulseLength) {
  this->protocol.pulseLength = nPulseLength;
}

/**
  * Sets sync factor
  */
void RCSwitch::setSyncFactor(uint8_t nSyncFactorHigh, uint8_t nSyncFactorLow) {
  if (nSyncFactorHigh > 0) {
    this->protocol.syncFactor.high = nSyncFactorHigh;
  }
  if (nSyncFactorLow > 0) {
    this->protocol.syncFactor.low = nSyncFactorLow;
  }
}

/**
 * Sets Repeat Transmits
 */
void RCSwitch::setRepeatTransmit(int _nRepeatTransmit) {
  this->nRepeatTransmit = _nRepeatTransmit;
}

/**
 * Set Receiving Tolerance
 */
#if not defined( RCSwitchDisableReceiving )
void RCSwitch::setReceiveTolerance(int nPercent) {
  this->nReceiveTolerance = nPercent;
}
#endif
  

/**
 * Enable transmissions
 *
 * @param nTransmitterPin    Arduino Pin to which the sender is connected to
 */
void RCSwitch::enableTransmit(int _nTransmitterPin) {
  this->nTransmitterPin = _nTransmitterPin;
  pinMode(this->nTransmitterPin, OUTPUT);
}

/**
  * Disable transmissions
  */
void RCSwitch::disableTransmit() {
  this->nTransmitterPin = -1;
}

/**
 * Switch a remote switch on (Type D REV)
 *
 * @param sGroup        Code of the switch group (A,B,C,D)
 * @param nDevice       Number of the switch itself (1..3)
 */
void RCSwitch::switchOn(char sGroup, int nDevice) {
  this->sendTriState( this->getCodeWordD(sGroup, nDevice, true) );
}

/**
 * Switch a remote switch off (Type D REV)
 *
 * @param sGroup        Code of the switch group (A,B,C,D)
 * @param nDevice       Number of the switch itself (1..3)
 */
void RCSwitch::switchOff(char sGroup, int nDevice) {
  this->sendTriState( this->getCodeWordD(sGroup, nDevice, false) );
}

/**
 * Switch a remote switch on (Type C Intertechno)
 *
 * @param sFamily  Familycode (a..f)
 * @param nGroup   Number of group (1..4)
 * @param nDevice  Number of device (1..4)
  */
void RCSwitch::switchOn(char sFamily, int nGroup, int nDevice) {
  this->sendTriState( this->getCodeWordC(sFamily, nGroup, nDevice, true) );
}

/**
 * Switch a remote switch off (Type C Intertechno)
 *
 * @param sFamily  Familycode (a..f)
 * @param nGroup   Number of group (1..4)
 * @param nDevice  Number of device (1..4)
 */
void RCSwitch::switchOff(char sFamily, int nGroup, int nDevice) {
  this->sendTriState( this->getCodeWordC(sFamily, nGroup, nDevice, false) );
}

/**
 * Switch a remote switch on (Type B with two rotary/sliding switches)
 *
 * @param nAddressCode  Number of the switch group (1..4)
 * @param nChannelCode  Number of the switch itself (1..4)
 */
void RCSwitch::switchOn(int nAddressCode, int nChannelCode) {
  this->sendTriState( this->getCodeWordB(nAddressCode, nChannelCode, true) );
}

/**
 * Switch a remote switch off (Type B with two rotary/sliding switches)
 *
 * @param nAddressCode  Number of the switch group (1..4)
 * @param nChannelCode  Number of the switch itself (1..4)
 */
void RCSwitch::switchOff(int nAddressCode, int nChannelCode) {
  this->sendTriState( this->getCodeWordB(nAddressCode, nChannelCode, false) );
}

/**
 * Deprecated, use switchOn(const char* sGroup, const char* sDevice) instead!
 * Switch a remote switch on (Type A with 10 pole DIP switches)
 *
 * @param sGroup        Code of the switch group (refers to DIP switches 1..5 where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 * @param nChannelCode  Number of the switch itself (1..5)
 */
void RCSwitch::switchOn(const char* sGroup, int nChannel) {
  const char* code[6] = { "00000", "10000", "01000", "00100", "00010", "00001" };
  this->switchOn(sGroup, code[nChannel]);
}

/**
 * Deprecated, use switchOff(const char* sGroup, const char* sDevice) instead!
 * Switch a remote switch off (Type A with 10 pole DIP switches)
 *
 * @param sGroup        Code of the switch group (refers to DIP switches 1..5 where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 * @param nChannelCode  Number of the switch itself (1..5)
 */
void RCSwitch::switchOff(const char* sGroup, int nChannel) {
  const char* code[6] = { "00000", "10000", "01000", "00100", "00010", "00001" };
  this->switchOff(sGroup, code[nChannel]);
}

/**
 * Switch a remote switch on (Type A with 10 pole DIP switches)
 *
 * @param sGroup        Code of the switch group (refers to DIP switches 1..5 where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 * @param sDevice       Code of the switch device (refers to DIP switches 6..10 (A..E) where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 */
void RCSwitch::switchOn(const char* sGroup, const char* sDevice) {
  this->sendTriState( this->getCodeWordA(sGroup, sDevice, true) );
}

/**
 * Switch a remote switch off (Type A with 10 pole DIP switches)
 *
 * @param sGroup        Code of the switch group (refers to DIP switches 1..5 where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 * @param sDevice       Code of the switch device (refers to DIP switches 6..10 (A..E) where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 */
void RCSwitch::switchOff(const char* sGroup, const char* sDevice) {
  this->sendTriState( this->getCodeWordA(sGroup, sDevice, false) );
}


/**
 * Returns a char[13], representing the code word to be send.
 *
 */
char* RCSwitch::getCodeWordA(const char* sGroup, const char* sDevice, bool bStatus) {
  static char sReturn[13];
  int nReturnPos = 0;

  for (int i = 0; i < 5; i++) {
    sReturn[nReturnPos++] = (sGroup[i] == '0') ? 'F' : '0';
  }

  for (int i = 0; i < 5; i++) {
    sReturn[nReturnPos++] = (sDevice[i] == '0') ? 'F' : '0';
  }

  sReturn[nReturnPos++] = bStatus ? '0' : 'F';
  sReturn[nReturnPos++] = bStatus ? 'F' : '0';

  sReturn[nReturnPos] = '\0';
  return sReturn;
}

/**
 * Encoding for type B switches with two rotary/sliding switches.
 *
 * The code word is a tristate word and with following bit pattern:
 *
 * +-----------------------------+-----------------------------+----------+------------+
 * | 4 bits address              | 4 bits address              | 3 bits   | 1 bit      |
 * | switch group                | switch number               | not used | on / off   |
 * | 1=0FFF 2=F0FF 3=FF0F 4=FFF0 | 1=0FFF 2=F0FF 3=FF0F 4=FFF0 | FFF      | on=F off=0 |
 * +-----------------------------+-----------------------------+----------+------------+
 *
 * @param nAddressCode  Number of the switch group (1..4)
 * @param nChannelCode  Number of the switch itself (1..4)
 * @param bStatus       Whether to switch on (true) or off (false)
 *
 * @return char[13], representing a tristate code word of length 12
 */
char* RCSwitch::getCodeWordB(int nAddressCode, int nChannelCode, bool bStatus) {
  static char sReturn[13];
  int nReturnPos = 0;

  if (nAddressCode < 1 || nAddressCode > 4 || nChannelCode < 1 || nChannelCode > 4) {
    return 0;
  }

  for (int i = 1; i <= 4; i++) {
    sReturn[nReturnPos++] = (nAddressCode == i) ? '0' : 'F';
  }

  for (int i = 1; i <= 4; i++) {
    sReturn[nReturnPos++] = (nChannelCode == i) ? '0' : 'F';
  }

  sReturn[nReturnPos++] = 'F';
  sReturn[nReturnPos++] = 'F';
  sReturn[nReturnPos++] = 'F';

  sReturn[nReturnPos++] = bStatus ? 'F' : '0';

  sReturn[nReturnPos] = '\0';
  return sReturn;
}

/**
 * Like getCodeWord (Type C = Intertechno)
 */
char* RCSwitch::getCodeWordC(char sFamily, int nGroup, int nDevice, bool bStatus) {
  static char sReturn[13];
  int nReturnPos = 0;

  int nFamily = (int)sFamily - 'a';
  if ( nFamily < 0 || nFamily > 15 || nGroup < 1 || nGroup > 4 || nDevice < 1 || nDevice > 4) {
    return 0;
  }
  
  // encode the family into four bits
  sReturn[nReturnPos++] = (nFamily & 1) ? 'F' : '0';
  sReturn[nReturnPos++] = (nFamily & 2) ? 'F' : '0';
  sReturn[nReturnPos++] = (nFamily & 4) ? 'F' : '0';
  sReturn[nReturnPos++] = (nFamily & 8) ? 'F' : '0';

  // encode the device and group
  sReturn[nReturnPos++] = ((nDevice-1) & 1) ? 'F' : '0';
  sReturn[nReturnPos++] = ((nDevice-1) & 2) ? 'F' : '0';
  sReturn[nReturnPos++] = ((nGroup-1) & 1) ? 'F' : '0';
  sReturn[nReturnPos++] = ((nGroup-1) & 2) ? 'F' : '0';

  // encode the status code
  sReturn[nReturnPos++] = '0';
  sReturn[nReturnPos++] = 'F';
  sReturn[nReturnPos++] = 'F';
  sReturn[nReturnPos++] = bStatus ? 'F' : '0';

  sReturn[nReturnPos] = '\0';
  return sReturn;
}

/**
 * Encoding for the REV Switch Type
 *
 * The code word is a tristate word and with following bit pattern:
 *
 * +-----------------------------+-------------------+----------+--------------+
 * | 4 bits address              | 3 bits address    | 3 bits   | 2 bits       |
 * | switch group                | device number     | not used | on / off     |
 * | A=1FFF B=F1FF C=FF1F D=FFF1 | 1=0FF 2=F0F 3=FF0 | 000      | on=10 off=01 |
 * +-----------------------------+-------------------+----------+--------------+
 *
 * Source: http://www.the-intruder.net/funksteckdosen-von-rev-uber-arduino-ansteuern/
 *
 * @param sGroup        Name of the switch group (A..D, resp. a..d) 
 * @param nDevice       Number of the switch itself (1..3)
 * @param bStatus       Whether to switch on (true) or off (false)
 *
 * @return char[13], representing a tristate code word of length 12
 */
char* RCSwitch::getCodeWordD(char sGroup, int nDevice, bool bStatus) {
  static char sReturn[13];
  int nReturnPos = 0;

  // sGroup must be one of the letters in "abcdABCD"
  int nGroup = (sGroup >= 'a') ? (int)sGroup - 'a' : (int)sGroup - 'A';
  if ( nGroup < 0 || nGroup > 3 || nDevice < 1 || nDevice > 3) {
    return 0;
  }

  for (int i = 0; i < 4; i++) {
    sReturn[nReturnPos++] = (nGroup == i) ? '1' : 'F';
  }

  for (int i = 1; i <= 3; i++) {
    sReturn[nReturnPos++] = (nDevice == i) ? '1' : 'F';
  }

  sReturn[nReturnPos++] = '0';
  sReturn[nReturnPos++] = '0';
  sReturn[nReturnPos++] = '0';

  sReturn[nReturnPos++] = bStatus ? '1' : '0';
  sReturn[nReturnPos++] = bStatus ? '0' : '1';

  sReturn[nReturnPos] = '\0';
  return sReturn;
}

/**
 * @param sCodeWord   a tristate code word consisting of the letter 0, 1, F
 */
void RCSwitch::sendTriState(const char* sCodeWord) {
  // turn the tristate code word into the corresponding bit pattern, then send it
  unsigned long code = 0;
  unsigned int length = 0;
  for (const char* p = sCodeWord; *p; p++) {
    code <<= 2L;
    switch (*p) {
      case '0':
        // bit pattern 00
        break;
      case 'F':
        // bit pattern 01
        code |= 1L;
        break;
      case '1':
        // bit pattern 11
        code |= 3L;
        break;
    }
    length += 2;
  }
  this->send(code, length);
}

/**
 * @param sCodeWord   a binary code word consisting of the letter 0, 1
 */
void RCSwitch::send(const char* sCodeWord) {
  // turn the tristate code word into the corresponding bit pattern, then send it
  unsigned long code = 0;
  unsigned int length = 0;
  for (const char* p = sCodeWord; *p; p++) {
    code <<= 1L;
    if (*p != '0')
      code |= 1L;
    length++;
  }
  this->send(code, length);
}

/**
 * Transmit the first 'length' bits of the integer 'code'. The
 * bits are sent from MSB to LSB, i.e., first the bit at position length-1,
 * then the bit at position length-2, and so on, till finally the bit at position 0.
 */
void RCSwitch::send(uint64_t code, unsigned int length) {
  if (this->nTransmitterPin == -1)
    return;

#if not defined( RCSwitchDisableReceiving )
  // make sure the receiver is disabled while we transmit
  int nReceiverInterrupt_backup = nReceiverInterrupt;
  if (nReceiverInterrupt_backup != -1) {
    this->disableReceive();
  }
#endif

  for (int nRepeat = 0; nRepeat < nRepeatTransmit; nRepeat++) {
    for (int i = length-1; i >= 0; i--) {
      if (code & (1L << i))
        this->transmit(protocol.one);
      else
        this->transmit(protocol.zero);
    }
    this->transmit(protocol.syncFactor);
  }

  // Disable transmit after sending (i.e., for inverted protocols)
  digitalWrite(this->nTransmitterPin, LOW);

#if not defined( RCSwitchDisableReceiving )
  // enable receiver again if we just disabled it
  if (nReceiverInterrupt_backup != -1) {
    this->enableReceive(nReceiverInterrupt_backup);
  }
#endif
}

/**
 * Transmit a single high-low pulse.
 */
void RCSwitch::transmit(HighLow pulses) {
  uint8_t firstLogicLevel = (this->protocol.invertedSignal) ? LOW : HIGH;
  uint8_t secondLogicLevel = (this->protocol.invertedSignal) ? HIGH : LOW;
  
  digitalWrite(this->nTransmitterPin, firstLogicLevel);
  delayMicroseconds( this->protocol.pulseLength * pulses.high);
  digitalWrite(this->nTransmitterPin, secondLogicLevel);
  delayMicroseconds( this->protocol.pulseLength * pulses.low);
}


#if not defined( RCSwitchDisableReceiving )
/**
 * Enable receiving data
 */
void RCSwitch::enableReceive(int interrupt) {
#ifdef RaspberryPi
  int receiverpin = interrupt;
#else
  // learn which digital pin corresponds to that interrupt
  int receiverpin = -1;
  for(int i = 0; i < 40; i++) {
    if (digitalPinToInterrupt(i) == interrupt) {
      receiverpin = i;
      break;
    }
  }
#endif
  this->nReceiverInterrupt = interrupt;
  this->nStaticReceiverPin = receiverpin;
  this->enableReceive();
}

void RCSwitch::enableReceive() {
  if (this->nReceiverInterrupt != -1) {
    this->nReceivedValue = 0;
    this->nReceivedBitlength = 0;
#if defined(RaspberryPi) // Raspberry Pi
    wiringPiISR(this->nReceiverInterrupt, INT_EDGE_BOTH, &handleInterrupt);
#else // Arduino
    attachInterrupt(this->nReceiverInterrupt, std::bind(&RCSwitch::handleInterrupt, this), CHANGE);
#endif
  }
}

/**
 * Disable receiving data
 */
void RCSwitch::disableReceive() {
#if not defined(RaspberryPi) // Arduino
  detachInterrupt(this->nReceiverInterrupt);
#endif // For Raspberry Pi (wiringPi) you can't unregister the ISR
  this->nReceiverInterrupt = -1;
}

bool RCSwitch::available() const {
  return this->nReceivedValue != 0;
}

void RCSwitch::resetAvailable() {
  this->nReceivedValue = 0;
}

unsigned long RCSwitch::getReceivedValue() const {
  return this->nReceivedValue;
}

unsigned int RCSwitch::getReceivedBitlength() const {
  return this->nReceivedBitlength;
}

unsigned int RCSwitch::getReceivedDelay() const {
  return this->nReceivedDelay;
}

unsigned int RCSwitch::getReceivedProtocol() const {
  return this->nReceivedProtocol;
}

unsigned int* RCSwitch::getReceivedRawdata() {
  return this->timings;
}

bool RCSwitch::getReceivedInverted() const {
  return this->nReceivedInverted;
}

unsigned int RCSwitch::getReceivedLevelInFirstTiming() const {
  return this->nReceivedLevelInFirstTiming;
}

/* helper function for the receiveProtocol method */
static inline unsigned int diff(unsigned int A, unsigned int B) {
  return (A > B) ? A - B : B - A;
}

/**
 *
 */
bool RECEIVE_ATTR RCSwitch::receiveProtocol(const int p) {

    int finalp = 0; // no protocol recognized
    unsigned int tmpfirstperiodlevel = this->firstperiodlevel; // store it before it is overwritten

    // ignore very short transmissions: no device sends them, so this must be noise
    if (changeCount < RCSWITCH_MIN_CHANGE_COUNT) { return false; } // also ensure avoiding 0 division later

    // changeCount is the number of stored durations
    // timings positions span from 0 ... (changeCount - 1)
    //
    // non-inverted protocols with recorded...
    // signals starting low: data bits timings from positions 1 ... (changeCount - 2)
    //                       sync bit timings[changeCount - 1], timings[0]
    // non-inverted protocols with recorded...
    // signals starting high: data bits timings from positions 2 ... (changeCount - 1)
    //                        sync bit timings[0], timings[1]
    // This version takes into account both options by advancing 1 position for the later
    //
    // inverted protocols with recorded...
    // signals starting low: data bits timings from positions 2 ... (changeCount - 1)
    //                       sync bit timings[changeCount - 1], timings[0]
    // inverted protocols with recorded...
    // signals starting high: data bits timings from positions 1 ... (changeCount - 2)
    //                        sync bit timings[0], timings[1]
    // This version stores an alternate phase for decoding to cope with inverted protocols

    unsigned int numberofdatabits = (changeCount - 2) / 2;

    unsigned int dataduration = 0;
    unsigned long squareddataduration = 0; // preparation for variance calculation
    unsigned long code = 0;
    unsigned int delay = 0; // all appearances of delay can be removed if, in future versions, it is decided to drop backwards compatibility

    unsigned int alternatedataduration = 0;
    unsigned long alternatesquareddataduration = 0; // preparation for variance calculation
    unsigned long alternatecode = 0;
    unsigned int alternatedelay = 0; // all appearances of delay can be removed if, in future versions, it is decided to drop backwards compatibility

    // calculate average of data bits duration
    // calculate variance of data bits duration
    // decode bit sequence,
    // get delay as average of the shorter level timings (for backwards compatibility)

    // calculate for alternate positions (shifted one timing) as well

    for (unsigned int i = 1; i < changeCount - 2; i += 2) {

        unsigned int thisbitDuration = this->timings[i]+this->timings[i + 1];
        dataduration += thisbitDuration;
        squareddataduration += (unsigned long)thisbitDuration*(unsigned long)thisbitDuration;

        code <<= 1;
        if (this->timings[i] < this->timings[i + 1]) {
            // zero
            // sum accumulated duration of shorter level timings
            delay += this->timings[i];
            // all appearances of delay can be removed if dropping backwards compatibility
        } else {
            // one
            code |= 1;
            // sum accumulated duration of shorter level timings
            delay += this->timings[i + 1];
            // all appearances of delay can be removed if dropping backwards compatibility
        }

        // for inverted protocols - timings are shifted

        unsigned int alternatebitDuration = this->timings[i + 1]+this->timings[i + 2];
        alternatedataduration += alternatebitDuration;
        alternatesquareddataduration += (unsigned long)alternatebitDuration*(unsigned long)alternatebitDuration;

        alternatecode <<= 1;
        if (this->timings[i + 1] < this->timings[i + 2]) {
            // zero
            // sum accumulated duration of shorter level timings
            alternatedelay += this->timings[i + 1];
            // all appearances of delay can be removed if dropping backwards compatibility
        } else {
            // one
            alternatecode |= 1;
            // sum accumulated duration of shorter level timings
            alternatedelay += this->timings[i + 2];
            // all appearances of delay can be removed if dropping backwards compatibility
        }
    }

    unsigned long variancebitduration = (squareddataduration - (unsigned long)dataduration*(unsigned long)dataduration/numberofdatabits)/(numberofdatabits-1);
    unsigned long alternatevariancebitduration = (alternatesquareddataduration - (unsigned long)alternatedataduration*(unsigned long)alternatedataduration/numberofdatabits)/(numberofdatabits-1);

    // decide whether databits are represented by timings 1+2 or 2+3
    bool databitsstartinone = variancebitduration < alternatevariancebitduration;
    // Value true when NOT INVERTED
    // PITFALL: occasionally (depending on the combination of bits) an inverted signal could be identified as direct signal

    unsigned int averagebitduration = 0;
    unsigned long squaredaveragebitduration = 0;

    if(databitsstartinone) {
        averagebitduration = (int)(0.5 + ((double)dataduration)/numberofdatabits);
        squaredaveragebitduration = (unsigned long)averagebitduration * (unsigned long)averagebitduration;
    } else {
        averagebitduration = (int)(0.5 + ((double)alternatedataduration)/numberofdatabits);
        squaredaveragebitduration = (unsigned long)averagebitduration * (unsigned long)averagebitduration;
        variancebitduration = alternatevariancebitduration;
        code = alternatecode;
        delay = alternatedelay;
    }

    // check whether all bits durations are similar, discard otherwise
    // a coefficient of variation (standard deviation/average) threshold of 5% should be adequate
    // that means rejecting if standard deviation > average * 5 / 100
    // in the squared scale is variance > squared average * 25 / 10000

    if (variancebitduration * 10000 > squaredaveragebitduration * 25 ) {
        return false;
    }

    bool invertedprotocoldecoded = !((!databitsstartinone) ^ (tmpfirstperiodlevel == 0));

    // get delay as average of the shorter level timings
    delay = (int)(0.5 + ((double)delay)/numberofdatabits);

    // ratio between long and short timing
    auto protocolratio = (unsigned int)(0.5 + ((double)(averagebitduration - delay)) / (double)delay);

    // improved pulselenght (delay) calculation
    int normalizedpulselength = (int)(0.5 + (double)averagebitduration/(double)(protocolratio+1));

    // store results

    this->nReceivedValue = code;
    this->nReceivedBitlength = numberofdatabits;
    this->nReceivedDelay = normalizedpulselength;

    this->nReceivedInverted = invertedprotocoldecoded;
    this->nReceivedLevelInFirstTiming = tmpfirstperiodlevel;


    // for compatibility: check which protocol fits the data

    // this can be completely removed from the receiver part of the library
    // and let the programer check if the received code is from the protocol
    // that was expected
    const unsigned int delayTolerance = delay * this->nReceiveTolerance / 100;

    for(unsigned int i = 1; i <= numProto; i++) {

#if defined(ESP8266) || defined(ESP32)
        const Protocol &pro = proto[i-1];
#else
        Protocol pro;
        memcpy_P(&pro, &proto[i-1], sizeof(Protocol));
#endif

        if (invertedprotocoldecoded == pro.invertedSignal && // protocol inversion is correct AND
            diff(delay, pro.pulseLength) < delayTolerance && // pulse length is correct AND
            protocolratio == (int)(0.5 + (double)pro.one.high/(double)pro.one.low) && // long vs short ratio is correct AND
            ( (databitsstartinone) ?
            diff(this->timings[0], pro.syncFactor.low * delay) < (pro.syncFactor.low * delayTolerance) // the sync timing is correct
            :
            diff(this->timings[1], pro.syncFactor.low * delay) < (pro.syncFactor.low * delayTolerance) // the sync timing is correct
            )  &&
            ( (databitsstartinone) ?
            diff(this->timings[changeCount-1], pro.syncFactor.high * delay) < (pro.syncFactor.high * delayTolerance ) // the sync timing is correct
            :
            diff(this->timings[0], pro.syncFactor.high * delay) < (pro.syncFactor.high * delayTolerance ) // the sync timing is correct
            )
            )
            { // the sync timing is correct

                finalp = i;
		break;
	}
    }
    
    /* For protocols that start low, the sync period looks like
     *               _________
     * _____________|         |XXXXXXXXXXXX|
     *
     * |--1st dur--|-2nd dur-|-Start data-|
     *
     * The 3rd saved duration starts the data.
     *
     * For protocols that start high, the sync period looks like
     *
     *  ______________
     * |              |____________|XXXXXXXXXXXXX|
     *
     * |-filtered out-|--1st dur--|--Start data--|
     *
     * The 2nd saved duration starts the data
     */

    this->nReceivedProtocol = finalp; // will be 0 if the code is recognized but it is of an unknown protocol
    return true;

}

void RECEIVE_ATTR RCSwitch::handleInterrupt() {
  const unsigned long time = micros();
  const unsigned int duration = time - lastTime;

  if (duration > nSeparationLimit && changeCount != 1 ) {
    // A long stretch without signal level change occurred. This could
    // be the gap between two transmission.
    if ((repeatCount==0) || (diff(duration, this->timings[0]) < 200)) {
      // This long signal is close in length to the long signal which
      // started the previously recorded timings; this suggests that
      // it may indeed by a a gap between two transmissions (we assume
      // here that a sender will send the signal multiple times,
      // with roughly the same gap between them).
      repeatCount++;
      if (repeatCount == 2) {
        receiveProtocol(1);
        repeatCount = 0;
      }
    }
    changeCount = 0;
    // store the opposite level, because the time recorded is the one of the previous level
    firstperiodlevel = ! digitalRead(nStaticReceiverPin);
  }
 
  // detect overflow
  if (changeCount >= RCSWITCH_MAX_CHANGES) {
    changeCount = 0;
    // store the opposite level, because the time recorded is the one of the previous level
    firstperiodlevel = ! digitalRead(nStaticReceiverPin);
    repeatCount = 0;
  }

  timings[changeCount++] = duration;
  lastTime = time;  
}
#endif
