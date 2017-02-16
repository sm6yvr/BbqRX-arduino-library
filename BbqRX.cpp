/*
*
*  BbqRX arduino library is a library to receive and decode temperature data from maverick et-73x
*
*   Copyright (C) 2017 Roberth Andersson
*   Copyright (C) 2014 B. Tod Cox, John Cox
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
v   the Free Software Foundation; either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
v   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software Foundation,
*   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
v
*   This library is based on BBQDuino and all logic to read and parse temperature data is from
*   that lib. It has been repackaged into a more generic lib so it's easier to use for everyone
*   in the arduino community. See https://github.com/btodcox/BBQduino for more info.
*
*/
#include "arduino.h"
#include "BbqRX.h"
//#include "EEPROM.h"


#define INPUT_CAPTURE_IS_RISING_EDGE()    ((TCCR1B & _BV(ICES1)) != 0)
#define INPUT_CAPTURE_IS_FALLING_EDGE()   ((TCCR1B & _BV(ICES1)) == 0)
#define SET_INPUT_CAPTURE_RISING_EDGE()   (TCCR1B |=  _BV(ICES1))
#define SET_INPUT_CAPTURE_FALLING_EDGE()  (TCCR1B &= ~_BV(ICES1))

#define BBQ_RESET() { short_count = packet_bit_pointer = 0; long_count = 0; BBQ_rx_state = RX_STATE_IDLE; current_bit = BIT_ZERO; }

#define TIMER_PERIOD_US             4   // Timer resolution is 4 micro seconds for 16MHz Uno
#define BBQ_PACKET_BIT_LENGTH   13*8    // 13 bytes with 2 quaternary encoded nibbles each

// pulse widths. short pulses ~500us, long pulses ~1000us. 50us tolerance
#define SHORT_PULSE_MIN_WIDTH       170/TIMER_PERIOD_US
#define SHORT_PULSE_MAX_WIDTH       300/TIMER_PERIOD_US
#define START_LONG_MIN_WIDTH        3700/TIMER_PERIOD_US  // Long low with short 5.2ms
#define START_LONG_MAX_WIDTH        5800/TIMER_PERIOD_US
#define LONG_PULSE_MIN_WIDTH        340/TIMER_PERIOD_US
#define LONG_PULSE_MAX_WIDTH        600/TIMER_PERIOD_US

// number of long/shorts in a row before the stream is treated as valid
#define SHORT_COUNT_SYNC_MIN        8

// states the receiver can be
#define RX_STATE_IDLE               0 // waiting for incoming stream
#define RX_STATE_READY              1 // "preamble" almost complete, next rising edge is start of data
#define RX_STATE_RECEIVING          2 // receiving valid stream
#define RX_STATE_PACKET_RECEIVED    3 // valid stream received

#define BIT_ZERO                    0
#define BIT_ONE                     1

#define DEBUG                       1

// Type aliases for brevity in the actual code
typedef unsigned int       uint; //16bit
typedef signed int         sint; //16bit

volatile int ste=LOW;
volatile int new_data=0;
int pin13led = 13;

uint probe1, probe2, tmp_probe1, tmp_probe2;
uint32_t check_data;
uint16_t chksum_data, chksum_sent, chk_xor, chk_xor_expected=0;
boolean chk_xor_once;
uint captured_time;
uint previous_captured_time;
uint captured_period;
uint current_bit;
uint packet_bit_pointer;
uint short_count;
uint long_count;
uint BBQ_rx_state;

boolean previous_period_was_short = false;

// byte arrays used to store incoming temperature data
byte BBQ_packet[(BBQ_PACKET_BIT_LENGTH/8)];
volatile byte BBQ_packet_process[(BBQ_PACKET_BIT_LENGTH/8)];
byte probe1_array[6], probe2_array[6]; //only need the 6 nibbles with actual probe temp data
byte last_BBQ_packet[(BBQ_PACKET_BIT_LENGTH/8)];

ProbeTempCallback BbqRX::_callback;
static boolean _inCallback;
static boolean _useChecksum;

ISR(TIMER1_OVF_vect) { }

ISR(TIMER1_CAPT_vect)
{
    BbqRX::OCR1A_ISR();
}

void BbqRX::OCR1A_ISR()
{
    int i; //"temp" variable
    // Immediately grab the current capture time in case it triggers again and
    // overwrites ICR1 with an unexpected new value
    captured_time = ICR1;

    //immediately grab the current capture polarity and reverse it to catch all the subsequent high and low periods coming in
    if (INPUT_CAPTURE_IS_RISING_EDGE()) {
    SET_INPUT_CAPTURE_FALLING_EDGE();      //previous period was low and just transitioned high
    } else {
    SET_INPUT_CAPTURE_RISING_EDGE();       //previous period was high and transitioned low
    }

    // calculate the current period just measured, to accompany the polarity now stored
    captured_period = (captured_time - previous_captured_time);

    // Analyse the incoming data stream. If idle, we need to detect the start of an incoming weather packet.
    // Incoming packet starts with several short pulses (over 100 short pulses) before a long pulse to signify
    // the start of the data.

    if (BBQ_rx_state == RX_STATE_IDLE) {
        if (( (captured_period >= SHORT_PULSE_MIN_WIDTH) && (captured_period <= SHORT_PULSE_MAX_WIDTH)  )) {
            // short pulse, continue counting short pulses
            short_count++;
            if ( short_count != long_count) { //should only have long (~5 ms) low followed by (~250 usec) high; if not, clear count
                short_count = 0;
                long_count = 0;
            }
            if(short_count == SHORT_COUNT_SYNC_MIN) { //8 long/short start pulses in row; packet with info is about to start!
                BBQ_rx_state = RX_STATE_READY;
            }
    } else if (((captured_period >= START_LONG_MIN_WIDTH) && (captured_period <= START_LONG_MAX_WIDTH) )) {
        // long pulse. if there has been enough short pulses beforehand, we have a valid bit stream, else reset and start again
        long_count++;
        //ste = !ste;
    } else {
      BBQ_RESET();
    }
} else if (BBQ_rx_state == RX_STATE_READY) {

    // this rising edge (assuming a ~4.8msec low) is start of data
    if (((captured_period >= START_LONG_MIN_WIDTH) && (captured_period <= START_LONG_MAX_WIDTH))) {
        // start of data packet, long pulse, swap the currrent_bit
        current_bit = !current_bit;
        BBQ_packet[packet_bit_pointer >> 3] |=  (0x80 >> (packet_bit_pointer&0x07)); //store the bit
        packet_bit_pointer++;
        previous_period_was_short = false;
        BBQ_rx_state = RX_STATE_RECEIVING;
    } else {
        BBQ_RESET();
    }

} else if (BBQ_rx_state == RX_STATE_RECEIVING) {

    // incoming pulses are a valid bit stream, manchester encoded. starting with a zero bit, the next bit will be the same as the
    // previous bit if there are two short pulses, or the bit will swap if the pulse is long
    if (((captured_period >= SHORT_PULSE_MIN_WIDTH) && (captured_period <= SHORT_PULSE_MAX_WIDTH))) {
        // short pulse
        if (previous_period_was_short) {
            // previous bit was short, add the current_bit value to the stream and continue to next incoming bit
            if (current_bit == BIT_ONE) {
                BBQ_packet[packet_bit_pointer >> 3] |=  (0x80 >> (packet_bit_pointer&0x07));
            }
            else if (current_bit == BIT_ZERO) {
                BBQ_packet[packet_bit_pointer >> 3] &= ~(0x80 >> (packet_bit_pointer&0x07));
            }
            packet_bit_pointer++;
            previous_period_was_short = false;
        } else {
            // previous bit was long, remember that and continue to next incoming bit
            previous_period_was_short = true;
        }
    } else if (((captured_period >= LONG_PULSE_MIN_WIDTH) && (captured_period <= LONG_PULSE_MAX_WIDTH))) {
        // long pulse, swap the current_bit
        current_bit = !current_bit;

        // add current_bit value to the stream and continue to next incoming bit
        if(current_bit == BIT_ONE) {
            BBQ_packet[packet_bit_pointer >> 3] |=  (0x80 >> (packet_bit_pointer&0x07));
        } else if (current_bit == BIT_ZERO) {
            BBQ_packet[packet_bit_pointer >> 3] &= ~(0x80 >> (packet_bit_pointer&0x07));
        }
        packet_bit_pointer++;
    }


}
    // check to see if a full packet has been received
    if(packet_bit_pointer >= BBQ_PACKET_BIT_LENGTH) {
        // full packet received, switch state to RX_STATE_PACKET_RECEIVED
        for (i=0; i<=13; i++){
          BBQ_packet_process[i]=BBQ_packet[i];
        }

        //BBQ_rx_state = RX_STATE_PACKET_RECEIVED;
        new_data = 1; //signal to main loop that new packet is available
        digitalWrite(7,HIGH);
        digitalWrite(13,HIGH);
        BBQ_RESET();
        BbqRX::setBbqDataPacket();
    }
    // save the current capture data as previous so it can be used for period calculation again next time around
    previous_captured_time = captured_time;
}

void BbqRX::interruptHandler(ProbeTemp probeTemp) {

    if (!_inCallback) {
        _inCallback = true;

       // Call callback funktion to return temperature to main class
       (_callback)(probeTemp);
       _inCallback = false;
    }
    //Reset after callback.
    return;
}

void BbqRX::setBbqDataPacket()
{

    ProbeTemp probeTemp = BbqRX::getTemperatureFromHex();

    if (probeTemp.isValid == true) {
        BbqRX::interruptHandler(probeTemp);
    }

}

ProbeTemp BbqRX::getTemperatureFromHex() {

    int i; //looping dummy

 #ifdef DEBUG
    for( i = 0; i < ((BBQ_PACKET_BIT_LENGTH/8)); i++) {
        Serial.print(BBQ_packet_process[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
 #endif

    if ((BBQ_packet_process[0] == 0xAA) &&
        (BBQ_packet_process[1] == 0x99) &&
        (BBQ_packet_process[2] == 0x95)  &&
        ((BBQ_packet_process[3] == 0x59) || //regular data packet
        (BBQ_packet_process[3] == 0x6A)))  //update transmitter chk_xor_expected--still contains temp info!!!
    {

        tmp_probe2=tmp_probe1=0;

        // convert temp packet from quaternary encoding
        probe2_array[0]= BbqRX::quart(BBQ_packet_process[8] & 0x0F);
        probe2_array[1]= BbqRX::quart(BBQ_packet_process[8] >> 4);
        probe2_array[2]= BbqRX::quart(BBQ_packet_process[7] & 0x0F);
        probe2_array[3]= BbqRX::quart(BBQ_packet_process[7] >> 4);
        probe2_array[4]= BbqRX::quart(BBQ_packet_process[6] & 0x0F);

        probe1_array[0]= BbqRX::quart(BBQ_packet_process[6] >> 4);
        probe1_array[1]= BbqRX::quart(BBQ_packet_process[5] & 0x0F);
        probe1_array[2]= BbqRX::quart(BBQ_packet_process[5] >> 4);
        probe1_array[3]= BbqRX::quart(BBQ_packet_process[4] & 0x0F);
        probe1_array[4]= BbqRX::quart(BBQ_packet_process[4] >> 4);

        for (i=0;i<=4;i++){
            tmp_probe2 += probe2_array[i] * (1<<(2*i));
        }

        for (i=0;i<=4;i++){
            tmp_probe1 += probe1_array[i] * (1<<(2*i));
        }

        //calc checksum and XOR with sent checksum to see if we got good data from correct transmitter
        //checksum calculation needs nibbles 6-17; see adafruit link for info.
        check_data = (uint32_t) BbqRX::quart(BBQ_packet_process[3] >> 4) << 22;
        check_data |= (uint32_t) BbqRX::quart(BBQ_packet_process[3]  & 0x0F) << 20;
        check_data |= (uint32_t) tmp_probe1 << 10;
        check_data |= (uint32_t) tmp_probe2;

        chksum_data = BbqRX::calculate_checksum(check_data);

        // nibbles 18-21 have checksum info from sender
        // convert sent checksum nibbles from quaternary encoding
        chksum_sent =  (uint16_t) BbqRX::quart(BBQ_packet_process[9] >> 4)     << 14;
        chksum_sent |= (uint16_t) BbqRX::quart(BBQ_packet_process[9]  & 0x0F)  << 12;
        chksum_sent |= (uint16_t) BbqRX::quart(BBQ_packet_process[10] >> 4)    << 10;
        chksum_sent |= (uint16_t) BbqRX::quart(BBQ_packet_process[10]  & 0x0F) << 8;
        chksum_sent |= (uint16_t) BbqRX::quart(BBQ_packet_process[11] >> 4)    << 6;
        chksum_sent |= (uint16_t) BbqRX::quart(BBQ_packet_process[11]  & 0x0F) << 4;
        chksum_sent |= (uint16_t) BbqRX::quart(BBQ_packet_process[12] >> 4)    << 2;
        chksum_sent |= (uint16_t) BbqRX::quart(BBQ_packet_process[12]  & 0x0F);

        // if packet is valid and from correct transmitter, chk_xor is constant
        // chk_xor will be different for each transmitter and will only change when
        //    a) transmitter is powered off/on
        //    b) sync/reset button is pressed on transmitter
        // Maverick wireless BBQ thermometers only allow the receiver to update the
        // transmitter chk_xor ONCE.  any new 6A packets are ignored by receiver until
        // receiver is power cylced or reset.

        chk_xor = chksum_data ^ chksum_sent;
     #ifdef DEBUG
        Serial.print("chx: ");
        Serial.println(chk_xor, HEX);
     #endif
        //check if we need to update chk_xor_expected
        if ( (BBQ_packet_process[3] == 0x6A) &&
            (chk_xor_once == false) ) // only allow the chk_xor_expected to be updated ONCE
        {
            chk_xor_expected = chk_xor;
            chk_xor_once = true;
            //store new value in EEPROM so can survive reset or powercycle without resyncing with transmitter
            // TODO: enable this later!
            //EEPROM.write(0, (byte) chk_xor_expected);
            //EEPROM.write(1, ( (byte) (chk_xor_expected >> 8)) );
        }

        // finish up probe temp calculations to yield celcius temps
        // if the chk_xor is good for current packet
        // and update temps/display if all is good
        if ((_useChecksum == true && chk_xor == chk_xor_expected) || (_useChecksum == false)) {
            chk_xor_once = true;  // could have a valid chk_xor_expected stored in EEPROM, if so, prevent resync without reset/powercyle

        if (tmp_probe1 != 0){ //check for unplugged temp probe
            probe1 = tmp_probe1-532;
            //probe1 = (probe1 * 18 + 5)/10 + 32; //convert to fahrenheit using fast integer math
        } else {
            probe1 = 0;  //probe temp of 0 indicates unplugged temp probe on transmitter
        }
        if (tmp_probe2 != 0){ //check for unplugged temp probe
            probe2 = tmp_probe2-532;
            //probe2 = (probe2 * 18 + 5)/10 + 32; //convert to fahrenheit using fast integer math
        } else {
            probe2 = 0; //probe temp of 0 indicates unplugged temp probe on transmitter
        }
    } else {
        probe1 = 0;
        probe2 = 0;
    #ifdef DEBUG
        Serial.println("Checksum did not match and useChecksum is set to true. Discarding data and waiting for new.");
    #endif
    }
    ProbeTemp probeTemp { probe1, probe2, true, chk_xor};
    return probeTemp;
}
}

void BbqRX::begin(bool useChecksum, ProbeTempCallback callback)
{
  _callback = callback;
  _useChecksum = useChecksum;

  Serial.println("begin");

    // set the pins we will to outputs
    pinMode(7,OUTPUT);
    digitalWrite(7,HIGH);

    // setup Input Capture Unit interrupt (ICP1) that allows precise timing of pulses
    // which, for this program, allows us to decode the 2000baud temp data from the BBQ transmitter
    DDRB = 0x2F;   // B00101111
    DDRB  &= ~(1<<DDB0);    // PBO(ICP1) input
    PORTB &= ~(1<<PORTB0);  // ensure pullup resistor is also disabled
    DDRD  |=  B11000000;    // (1<<PORTD6);   //DDRD  |=  (1<<PORTD7); (example of B prefix)

    //---------------------------------------------------------------------------------------------
    //ICNC1: Input Capture Noise Canceler         On, 4 successive equal ICP1 samples required for trigger (4*4uS = 16uS delayed)
    //ICES1: Input Capture Edge Select            1 = rising edge to begin with, input capture will change as required
    //CS12,CS11,CS10   TCNT1 Prescaler set to 0,1,1 see table and notes above
    TCCR1A = B00000000;   //Normal mode of operation, TOP = 0xFFFF, TOV1 Flag Set on MAX
    //This is supposed to come out of reset as 0x00, but something changed it, I had to zero it again here to make the TOP truly 0xFFFF
    TCCR1B = ( _BV(ICNC1) | _BV(CS11) | _BV(CS10) );
    SET_INPUT_CAPTURE_RISING_EDGE();
    //Timer1 Input Capture Interrupt Enable, Overflow Interrupt Enable
    TIMSK1 = ( _BV(ICIE1) | _BV(TOIE1) );

    BBQ_RESET();
    Serial.println(F("BBQ RX v0.1"));
    Serial.println(F("Ready to receive probe data"));
}

// make the quarternary convertion
byte BbqRX::quart(byte param)
{
    param &= 0x0F;
    if (param==0x05) return(0);
    if (param==0x06) return(1);
    if (param==0x09) return(2);
    if (param==0x0A) return(3);
}

double BbqRX::get_temp(byte select)
{
    if (select==1) return((double)probe1);
    if (select==2) return((double)probe2);
    //	if (select==3) return((double)DS1820_temp);
    return(0);	// just for safety
}

uint16_t BbqRX::shiftreg(uint16_t currentValue) {
    uint8_t msb = (currentValue >> 15) & 1;
    currentValue <<= 1;
    if (msb == 1) {
        // Toggle pattern for feedback bits
        // Toggle, if MSB is 1
        currentValue ^= 0x1021;
    }
    return currentValue;
}

//data = binary representation of nibbles 6 - 17
//e.g. xxxx:xxxx:xxxx:0010:1000:1010:0110:0101:0101:xxxx:xxxx:xxxx:xxxx

//  -> uint32_t data = 0x28a655
uint16_t BbqRX::calculate_checksum(uint32_t data) {
    uint16_t mask = 0x3331; //initial value of linear feedback shift register
    uint16_t csum = 0x0;
    int i = 0;
    for(i = 0; i < 24; ++i) {
        if((data >> i) & 0x01) {
            //data bit at current position is "1"
            //do XOR with mask
            csum ^= mask;
        }
        mask = BbqRX::shiftreg(mask);
    }
    return csum;
}


// sets every element of str to 0 (clears array)
void BbqRX::StrClear(char *str, char length)
{
    for (int i = 0; i < length; i++) {
        str[i] = 0;
    }
}
