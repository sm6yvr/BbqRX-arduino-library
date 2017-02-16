/*
 *
 *  BbqRX arduino library is a library to receive and decode temperature data from maverick et-73x
 *
 *   Copyright (C) 2017 Roberth Andersson
 *   Copyright (C) 2014 B. Tod Cox, John Cox
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software Foundation,
 *   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
 *
 *   This library is based on BBQDuino and all logic to read and parse temperature data is from
 *   that lib. It has been repackaged into a more generic lib so it's easier to use for everyone
 *   in the arduino community. See https://github.com/btodcox/BBQduino for more info.
 *
 */

#include "Arduino.h"

typedef void (*ProbeTempCallback)(struct ProbeTemp);

struct ProbeTemp
{
    double probe1;
    double probe2;
    bool isValid;
    char chksum;
};

class BbqRX
{
    public:
        BbqRX();
        static void begin(bool useChecksum, ProbeTempCallback callback);
        static void OCR1A_ISR();

    protected:
        static void setBbqDataPacket();
        static void interruptHandler(ProbeTemp probeTemp);
        static ProbeTemp getTemperatureFromHex();

    private:
        static ProbeTempCallback _callback;
        static byte quart(byte param);
        double get_temp(byte select);
        static uint16_t shiftreg(uint16_t currentValue);
        static uint16_t calculate_checksum(uint32_t data);
        void StrClear(char *str, char length);
};
