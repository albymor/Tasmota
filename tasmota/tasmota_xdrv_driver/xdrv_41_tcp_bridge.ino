/*
  xdrv_41_tcp_bridge.ino - TCP to serial bridge

  Copyright (C) 2021  Theo Arends and Stephan Hadinger

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

#ifdef USE_TCP_BRIDGE

#define XDRV_41                    41

#ifndef TCP_BRIDGE_CONNECTIONS
#define TCP_BRIDGE_CONNECTIONS 2    // number of maximum parallel connections
#endif

#ifndef TCP_BRIDGE_BUF_SIZE
#define TCP_BRIDGE_BUF_SIZE    255  // size of the buffer, above 132 required for efficient XMODEM
#endif

//const uint16_t tcp_port = 8880;
WiFiServer   *server_tcp = nullptr;
//WiFiClient   client_tcp1, client_tcp2;
WiFiClient   client_tcp[TCP_BRIDGE_CONNECTIONS];
uint8_t      client_next = 0;
uint8_t     *tcp_buf = nullptr;     // data transfer buffer
bool         ip_filter_enabled = false;
IPAddress    ip_filter;




#include <TasmotaSerial.h>

typedef struct
{
	uint32_t header;
	uint32_t batteryVoltage;
	uint32_t panelVoltage;
	uint8_t chargerState;
	uint16_t pwm;
	uint32_t vdd;
	uint16_t batteryTargetVoltage;
	uint8_t loadState;
	int32_t current;
	uint8_t commandCounter;
	uint8_t lastError;
	uint16_t tail;
} CommData;

CommData chargerData, lastValidChargerData;

#define COB_BUFFER_LEN 64
uint8_t cobBuffer[COB_BUFFER_LEN], decodeData[COB_BUFFER_LEN];
uint8_t datalen, writeindex, decoded_len;

void unpackData(const uint8_t* dataArray, uint8_t dataArrayLength, CommData* chargerData);

TasmotaSerial *TCPSerial = nullptr;

const char kTCPCommands[] PROGMEM = "TCP" "|"    // prefix
  "Start" "|" "Baudrate" "|" "Config" "|" "Connect"
  ;

void (* const TCPCommand[])(void) PROGMEM = {
  &CmndTCPStart, &CmndTCPBaudrate, &CmndTCPConfig, &CmndTCPConnect
  };


void unpackData(const uint8_t* dataArray, uint8_t dataArrayLength, CommData* chargerData)
{
    // Check if dataArray has enough bytes to fill the structure
    if (dataArray == NULL || sizeof(dataArrayLength) > sizeof(dataArray)) {
        // Handle the error appropriately (e.g., return an error code)
        return;
    }

    // Copy the data from the dataArray into chargerData while handling endianness
    chargerData->header = (uint32_t)dataArray[0] |
                         ((uint32_t)dataArray[1] << 8) |
                         ((uint32_t)dataArray[2] << 16) |
                         ((uint32_t)dataArray[3] << 24);

    chargerData->batteryVoltage = (uint32_t)dataArray[4] |
                                 ((uint32_t)dataArray[5] << 8) |
                                 ((uint32_t)dataArray[6] << 16) |
                                 ((uint32_t)dataArray[7] << 24);

    chargerData->panelVoltage = (uint32_t)dataArray[8] |
                               ((uint32_t)dataArray[9] << 8) |
                               ((uint32_t)dataArray[10] << 16) |
                               ((uint32_t)dataArray[11] << 24);

    chargerData->chargerState = dataArray[12];
    chargerData->pwm = (uint16_t)((uint16_t)dataArray[13] |
                                ((uint16_t)dataArray[14] << 8));

    chargerData->vdd = (uint32_t)dataArray[15] |
                     ((uint32_t)dataArray[16] << 8) |
                     ((uint32_t)dataArray[17] << 16) |
                     ((uint32_t)dataArray[18] << 24);

    chargerData->batteryTargetVoltage = (uint16_t)((uint16_t)dataArray[19] |
                                               ((uint16_t)dataArray[20] << 8));

    chargerData->loadState = dataArray[21];

    chargerData->current = (int32_t)((int32_t)dataArray[22] |
                                   ((int32_t)dataArray[23] << 8) |
                                   ((int32_t)dataArray[24] << 16) |
                                   ((int32_t)dataArray[25] << 24));

    chargerData->commandCounter = dataArray[26];
    chargerData->lastError = dataArray[27];

    chargerData->tail = (uint16_t)((uint16_t)dataArray[28] |
                               ((uint16_t)dataArray[29] << 8));
}

void cobs_decode(const uint8_t* encodedData, uint8_t encodedLength, uint8_t* decodedData, uint8_t* decodedLength) {
    int readIndex = 0;
    int writeIndex = 0;

    while (readIndex < encodedLength) {
        uint8_t code = encodedData[readIndex];

        if (readIndex + code > encodedLength && code != 1) {
            // If code is too large, it's an error.
            // This can happen if there is missing data or a corrupted frame.
            *decodedLength = 0; // Indicate an error
            return;
        }

        readIndex++;

        for (int i = 1; i < code; i++) {
            decodedData[writeIndex++] = encodedData[readIndex++];
        }

        if (code < 0xFF && readIndex < encodedLength) {
            decodedData[writeIndex++] = 0;
        }
    }

    *decodedLength = writeIndex;
}

void cobsDecode(const uint8_t *ptr, int length, uint8_t *dst)
{
  const uint8_t *end = ptr + length;
  int i, code;

  while (ptr < end)
  {
    code = *ptr++;

    for (i = 1; i < code; i++)
      *dst++ = *ptr++;

    if (code < 0xFF)
      *dst++ = 0;
  }
}

//
// Called at event loop, checks for incoming data from the CC2530
//
void TCPLoop(void)
{
  uint8_t c;
  bool busy;    // did we transfer some data?
  int32_t buf_len;

  if (!TCPSerial) return;

  // check for a new client connection
  if ((server_tcp) && (server_tcp->hasClient())) {
    WiFiClient new_client = server_tcp->available();

    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_TCP "Got connection from %s"), new_client.remoteIP().toString().c_str());
    // Check for IP filtering if it's enabled.
    if (ip_filter_enabled) {
      if (ip_filter != new_client.remoteIP()) {
        AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_TCP "Rejected due to filtering"));
        new_client.stop();
      } else {
        AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_TCP "Allowed through filter"));
      }
    }

    // find an empty slot
    uint32_t i;
    for (i=0; i<nitems(client_tcp); i++) {
      WiFiClient &client = client_tcp[i];
      if (!client) {
        client = new_client;
        break;
      }
    }
    if (i >= nitems(client_tcp)) {
      i = client_next++ % nitems(client_tcp);
      WiFiClient &client = client_tcp[i];
      client.stop();
      client = new_client;
    }
  }

  do {
    busy = false;       // exit loop if no data was transferred

    // start reading the UART, this buffer can quickly overflow
    buf_len = 0;
    while ((buf_len < TCP_BRIDGE_BUF_SIZE) && (TCPSerial->available())) {
      c = TCPSerial->read();
      if (c >= 0) {
        tcp_buf[buf_len++] = c;
        busy = true;
      }
    }
    if (buf_len > 0) {
      AddLog(LOG_LEVEL_DEBUG_MORE, PSTR(D_LOG_TCP "from MCU: %*_H"), buf_len, tcp_buf);
      
      for (uint32_t i=0; i<nitems(client_tcp); i++) {
        WiFiClient &client = client_tcp[i];
        if (client) { client.write(tcp_buf, buf_len); }
      }

      for(uint8_t i = 0; i<buf_len; i++)
      {
        cobBuffer[writeindex] = tcp_buf[i];
        writeindex = (writeindex+1)%COB_BUFFER_LEN;
        if(tcp_buf[i] == 0)
        {
          datalen=writeindex;
          writeindex=0;
          cobs_decode(cobBuffer, datalen-1, decodeData, &decoded_len);
          CommData chargerData;
          chargerData.header = 0;
          unpackData(decodeData, decoded_len, &chargerData);
          if (chargerData.header == 0xdeadbeef)
          {
            AddLog(LOG_LEVEL_DEBUG_MORE, PSTR(D_LOG_TCP "Charger data %i %i %i %i %i %i %i %i %d %i %i %i"), chargerData.header, chargerData.batteryVoltage, chargerData.panelVoltage, chargerData.chargerState, chargerData.pwm, chargerData.vdd, chargerData.batteryTargetVoltage, chargerData.loadState, chargerData.current, chargerData.commandCounter, chargerData.lastError, chargerData.tail);
            memcpy(&lastValidChargerData, &chargerData, sizeof(CommData));
          }
        }
      }

      
    }

    // handle data received from TCP
    for (uint32_t i=0; i<nitems(client_tcp); i++) {
      WiFiClient &client = client_tcp[i];
      buf_len = 0;
      while (client && (buf_len < TCP_BRIDGE_BUF_SIZE) && (client.available())) {
        c = client.read();
        if (c >= 0) {
          tcp_buf[buf_len++] = c;
          busy = true;
        }
      }
      if (buf_len > 0) {
        AddLog(LOG_LEVEL_DEBUG_MORE, PSTR(D_LOG_TCP "to MCU/%d: %*_H"), i+1, buf_len, tcp_buf);
        TCPSerial->write(tcp_buf, buf_len);
      }
    }

    yield();    // avoid WDT if heavy traffic
  } while (busy);
}

/********************************************************************************************/

void TCPInit(void) {
  if (PinUsed(GPIO_TCP_RX) && PinUsed(GPIO_TCP_TX)) {
    if (0 == (0x80 & Settings->tcp_config)) // !0x80 means unitialized
      Settings->tcp_config = 0x80 | ParseSerialConfig("8N1"); // default as 8N1 for backward compatibility
    tcp_buf = (uint8_t*) malloc(TCP_BRIDGE_BUF_SIZE);
    if (!tcp_buf) { AddLog(LOG_LEVEL_ERROR, PSTR(D_LOG_TCP "could not allocate buffer")); return; }

    if (!Settings->tcp_baudrate)  { Settings->tcp_baudrate = 115200 / 1200; }
    TCPSerial = new TasmotaSerial(Pin(GPIO_TCP_RX), Pin(GPIO_TCP_TX), TasmotaGlobal.seriallog_level ? 1 : 2, 0, TCP_BRIDGE_BUF_SIZE);   // set a receive buffer of 256 bytes
    if (TCPSerial->begin(Settings->tcp_baudrate * 1200, ConvertSerialConfig(0x7F & Settings->tcp_config))) {
      if (TCPSerial->hardwareSerial()) {
        ClaimSerial();
      }
#ifdef ESP32
      AddLog(LOG_LEVEL_DEBUG, PSTR(D_LOG_TCP "using hardserial %d"), TCPSerial->getUart());
#endif
    } else {
      AddLog(LOG_LEVEL_DEBUG, PSTR(D_LOG_TCP "failed init serial"));
    }
  }
  memset(&lastValidChargerData, 0, sizeof(CommData));
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

//
// Command `TCPStart`
// Params: port,<IPv4 allow>
//
void CmndTCPStart(void) {

  if (!TCPSerial) { return; }

  int32_t tcp_port = XdrvMailbox.payload;
  if (ArgC() == 2) {
    char sub_string[XdrvMailbox.data_len];
    ip_filter.fromString(ArgV(sub_string, 2));
    ip_filter_enabled = true;
  } else {
    // Disable whitelist if previously set
    ip_filter_enabled = false;
  }

  if (server_tcp) {
    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_TCP "Stopping TCP server"));
    server_tcp->stop();
    delete server_tcp;
    server_tcp = nullptr;

    for (uint32_t i=0; i<nitems(client_tcp); i++) {
      WiFiClient &client = client_tcp[i];
      client.stop();
    }
  }
  if (tcp_port > 0) {
    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_TCP "Starting TCP server on port %d"), tcp_port);
    if (ip_filter_enabled) {
      AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_TCP "Filtering %s"), ip_filter.toString().c_str());
    }
    server_tcp = new WiFiServer(tcp_port);
    server_tcp->begin(); // start TCP server
    server_tcp->setNoDelay(true);
  }

  ResponseCmndDone();
}

void CmndTCPBaudrate(void) {
  if (!TCPSerial) { return; }

  if ((XdrvMailbox.payload >= 1200) && (XdrvMailbox.payload <= 115200)) {
    XdrvMailbox.payload /= 1200;  // Make it a valid baudrate
    if (Settings->tcp_baudrate != XdrvMailbox.payload) {
      Settings->tcp_baudrate = XdrvMailbox.payload;
      if (!TCPSerial->begin(Settings->tcp_baudrate * 1200, ConvertSerialConfig(0x7F & Settings->tcp_config))) {
        AddLog(LOG_LEVEL_DEBUG, PSTR(D_LOG_TCP "failed reinit serial"));
      }  // Reinitialize serial port with new baud rate
    }
  }
  ResponseCmndNumber(Settings->tcp_baudrate * 1200);
}

void CmndTCPConfig(void) {
  if (!TCPSerial) { return; }

  if (XdrvMailbox.data_len > 0) {
    uint8_t serial_config = ParseSerialConfig(XdrvMailbox.data);
    if ((serial_config >= 0) && (Settings->tcp_config != (0x80 | serial_config))) {
      Settings->tcp_config = 0x80 | serial_config; // default 0x00 should be 8N1
      if (!TCPSerial->begin(Settings->tcp_baudrate * 1200, ConvertSerialConfig(0x7F & Settings->tcp_config))) {
        AddLog(LOG_LEVEL_DEBUG, PSTR(D_LOG_TCP "failed reinit serial"));
      }  // Reinitialize serial port with new config
    }
  }
  ResponseCmndChar_P(GetSerialConfig(0x7F & Settings->tcp_config).c_str());
}

//
// Command `Connect`
// Params: port,<IPv4>
//
void CmndTCPConnect(void) {
  int32_t tcp_port = XdrvMailbox.payload;

  if (!TCPSerial) { return; }

  if (ArgC() == 2) {
    char sub_string[XdrvMailbox.data_len];
    WiFiClient new_client;
    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_TCP "Connecting to %s on port %d"), ArgV(sub_string, 2),tcp_port);
    if (new_client.connect(ArgV(sub_string, 2),tcp_port)) {
      AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_TCP "connected!"));
    } else {
      AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_TCP "error connecting!"));
    }

    // find an empty slot
    uint32_t i;
    for (i=0; i<nitems(client_tcp); i++) {
      WiFiClient &client = client_tcp[i];
      if (!client) {
        client = new_client;
        break;
      }
    }
    if (i >= nitems(client_tcp)) {
      i = client_next++ % nitems(client_tcp);
      WiFiClient &client = client_tcp[i];
      client.stop();
      client = new_client;
    }
  } else {
    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_TCP "Usage: port,ip_address"));
  }

  ResponseCmndDone();
}

void TCPJsonAppend(void)
{
  //AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_TCP "TCP server aa"));
  ResponseAppend_P(PSTR(",\"Charge Controller\":{\"Battery Voltage\":\"%i\",\"Panel Voltage\":%i,\"Charger State\":%i,\"PWM\":%i,\"Vdd\":%i,\"Target Voltage\":%i,\"Load State\":%i,\"Current\":%i,\"Comm. Counter\":%i,\"lastError\":%i,}"),
          lastValidChargerData.batteryVoltage, lastValidChargerData.panelVoltage, lastValidChargerData.chargerState, lastValidChargerData.pwm, lastValidChargerData.vdd, lastValidChargerData.batteryTargetVoltage, lastValidChargerData.loadState, lastValidChargerData.current, lastValidChargerData.commandCounter, lastValidChargerData.lastError);
  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR(D_LOG_TCP "TCP Sending tele"));
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv41(uint32_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_JSON_APPEND:
      TCPJsonAppend();
      break;
    case FUNC_LOOP:
      TCPLoop();
      break;
    case FUNC_PRE_INIT:
      TCPInit();
      break;
    case FUNC_COMMAND:
      result = DecodeCommand(kTCPCommands, TCPCommand);
      break;
  }
  return result;
}

#endif // USE_TCP_BRIDGE
