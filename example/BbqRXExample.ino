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
*  This is an example sketch of how to read temperature data from a maverick et-73x
*  The data from 433,92MHz receiver must be connected to pin 8 on your arduino uno.
*/

#include <BbqRX.h>

void setup()
{
    Serial.begin(9600);
    Serial.println("Setup BbqRXExample");
    BbqRX::begin(false, receiveTemp);
}

void loop()
{
}

void receiveTemp(ProbeTemp probeTemp)
{
    Serial.print("Probe1 = ");
    Serial.println(probeTemp.probe1, DEC);
    Serial.print("Probe2 = ");
    Serial.println(probeTemp.probe2, DEC);
}
