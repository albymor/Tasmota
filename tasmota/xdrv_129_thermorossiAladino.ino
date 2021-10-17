/*
  xdrv_129_thermorossiAladino.ino - Protocol to emulate Thermorossi Aladino RF remote for Tasmota

  Copyright (C) 2021  Alberto Morato

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//#ifdef USE_ALADINO

#define XDRV_129 129

#define D_JSON_THERMOROSSI_POWER "Power"
#define D_JSON_THERMOROSSI_FAN "Fan"

#define D_CMND_THERMOROSSISEND "ThermorossiSend"

const char kRfCommands[] PROGMEM = "|" // No prefix
    D_CMND_THERMOROSSISEND;

void (*const RfCommands[])(void) PROGMEM = {
    &ThermosossiTransmitt};

const uint8_t PowerLevels[6] = {
    0x3c, // OFF
    0x01, // Level 1
    0x02, // Level 2
    0x03, // Level 3
    0x04, // Level 4
    0x05  // Level 5
};

const uint8_t FanLevels[6] = {
    0x0b, // Level 1
    0x0c, // Level 2
    0x0d, // Level 3
    0x0e, // Level 4
    0x0f, // Level 5
    0x10  // Level 6
};

#include <TasmotaSerial.h>
TasmotaSerial *ThermorossiPort = nullptr;


void ThermorossiInit(void)
{
  ThermorossiPort = new TasmotaSerial(-1, Pin(GPIO_ALADINO_RF));
  ThermorossiPort->begin(2400, SERIAL_8N2);
  pinMode(Pin(GPIO_ALADINO_RF), OUTPUT);
  digitalWrite(Pin(GPIO_ALADINO_RF), LOW);
}

void ThermosossiRFSend(uint8_t *data)
{
  pinMode(Pin(GPIO_ALADINO_RF), OUTPUT);
  digitalWrite(Pin(GPIO_ALADINO_RF), HIGH);
  delay(4);
  for (size_t i = 0; i < 20; i++)
  {
    for (size_t b = 0; b < 3; b++)
    {
      ThermorossiPort->write(data[b]);
    }
  }
  ThermorossiPort->write(0xff);
  for (size_t i = 0; i < 20; i++)
  {
    for (size_t b = 0; b < 3; b++)
    {
      ThermorossiPort->write(data[b]);
    }
  }
  ThermorossiPort->write(0x01);
  pinMode(Pin(GPIO_ALADINO_RF), OUTPUT);
  digitalWrite(Pin(GPIO_ALADINO_RF), LOW);
}


/*********************************************************************************************\
 * Commands
\*********************************************************************************************/




void ThermosossiTransmitt(void)
{
  if (!PinUsed(GPIO_ALADINO_RF))
  {
    return;
  }

  bool error = false;
  int32_t fan = -1;
  int32_t power = -1;

  JsonParser parser(XdrvMailbox.data);
  JsonParserObject root = parser.getRootObject();

  if (root)
  {
    // ThermorossiSend {"power":2,"fan":5}
    char parm_uc[10];
    power = root.getInt(PSTR(D_JSON_THERMOROSSI_POWER), power); 
    fan = root.getInt(PSTR(D_JSON_THERMOROSSI_FAN), fan);
  }
  else
  {
    // TODO
  }


  uint8_t data[] = {0xe5, 0xc3, 0x00};

  if (power>=0 && power<=5)
  {
    data[2] = PowerLevels[power];
    ThermosossiRFSend(data);
  }
  else if (fan>=1 && fan<=6)
  {
    data[2] = FanLevels[fan-1];
    ThermosossiRFSend(data);
  }
  else
  {
    error = true;
  }  

  if(!error)
  {
    ResponseCmndDone();
  }
  else
  {
    ResponseCmndError();
  }
  
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv129(uint8_t function)
{
  bool result = false;

  if (PinUsed(GPIO_ALADINO_RF))
  {
    switch (function)
    {
    case FUNC_COMMAND:
      result = DecodeCommand(kRfCommands, RfCommands);
      break;
    case FUNC_INIT:
      ThermorossiInit();
      break;
    }
  }
  return result;
}

//#endif  // USE_ALADINO
