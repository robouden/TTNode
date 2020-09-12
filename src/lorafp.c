// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// LoRa / LoRaWAN Frequency Plan Support
//
// This is a really "elegant" (yeah right) hack that adapts TTN's Arduino Library
// so that we can use it almost verbatim for frequency plan configuration.
//
// See below.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "debug.h"
#include "misc.h"
#include "lorafp.h"

#ifdef LORA

// My statics, and everything needed to enable TTN code to remain largely intact

#define PROGMEM
#define debugPrint(x)
#define debugPrintHex(x,y)
#define debugPrintLn(x)
#define debugPrintMessage(x,y)
#define F(x) x
#define strcpy_P(a, b) strlcpy(a, b, sizeof(a))
#define pgm_read_word(x) (*(x))
#define pgmstrcmp(x, y) strcmp((char *)(x), (char *)(y))
#define waitForOk() true
char set_buffer[40];

#define HEX 0
typedef void (*modem_stream_print_t) (uint8_t, uint8_t);
typedef void (*modem_stream_write_t) (const char *str);
typedef bool (*modem_stream_available_t) (void);
typedef void (*modem_stream_read_t) (void);
typedef uint8_t port_t;
struct modem_stream_s {
    modem_stream_available_t available;
    modem_stream_read_t read;
    modem_stream_write_t write;
    modem_stream_print_t print;
};
typedef struct modem_stream_s modem_stream_t;

static modem_stream_t modem;
static modem_stream_t *modemStream;
static uint16_t thiscmdno;
static uint16_t targetcmdno;
static bool targetcmdfound;
static char *pBuffer;
static uint16_t BufferLeft;
static uint16_t BufferNeeded;

// Forwards
void sendCommand(uint8_t table, uint8_t index, bool appendSpace);
bool sendMacSet(uint8_t index, const char *value);
bool sendChSet(uint8_t index, uint8_t channel, const char *value);

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//
//  Everything after here is almost exactly a copy/paste of the code within:
//  https://github.com/TheThingsNetwork/arduino-device-lib
//  src/TheThingsNetwork.h
//  src/TheThingsNetwork.cpp
//
// The only substantive changes involve the function prototypes, because what's defined
// here must be straight ansi C because of compiler options that I've chosen to use.
//
// 1) Replace "TheThingsNetwork::" with ""
// 2) add fp as the first arg to configureChannels and setSf
// 3) remove print argument from sendcommand and delete code within where it is processed
// 4) change modemstream->print()'s with a single arg to ->write()'s
// 5) change references to 2-arg debugprint to debugprinthex
// 6) change "uint8_t dr;" to "uint8_t dr = 0;" in setSf because of initialization warning
// 7) delete functions pmgstrcmp, waitforok, TheThingsNetwork, getAppEui, getHardwareEui
//    reset, join, provision, readresponse, personalize, savestate, debugprintindex,
//    sendbytes, poll, showstatus, autobaud, readline, onmessage, debugprintmessage, sleep
//
// Yes, it is kind-of crazy that I do the kind of "emulation" that I do above, just
// so that I don't need to modify the code below.  However, I feel that it's very
// important that we be able to readily copy over any changes that TTN makes to
// frequency plan configuration methods, and so I think it's worth the hackiness.
//
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// From TheThingsNetwork.h

#define TTN_DEFAULT_SF 7
#define TTN_DEFAULT_FSB 2
#define TTN_RETX "7"

#define TTN_PWRIDX_EU868 "1"
#define TTN_PWRIDX_US915 "5"
#define TTN_PWRIDX_AS920_923 "1" // TODO: should be 0, but the current RN2903AS firmware doesn't accept that value (probably still using EU868: 1=14dBm)

#define TTN_BUFFER_SIZE 300

typedef uint8_t port_t;

enum ttn_response_t
{
  TTN_ERROR_SEND_COMMAND_FAILED = (-1),
  TTN_ERROR_UNEXPECTED_RESPONSE = (-10),
  TTN_SUCCESSFUL_TRANSMISSION = 1,
  TTN_SUCCESSFUL_RECEIVE = 2
};

enum ttn_fp_t
{
  TTN_FP_EU868,
  TTN_FP_US915,
  TTN_FP_AS920_923
};


// From TheThingsNetwork.cpp

#define TTN_HEX_CHAR_TO_NIBBLE(c) ((c >= 'A') ? (c - 'A' + 0x0A) : (c - '0'))
#define TTN_HEX_PAIR_TO_BYTE(h, l) ((TTN_HEX_CHAR_TO_NIBBLE(h) << 4) + TTN_HEX_CHAR_TO_NIBBLE(l))

const char ok[] PROGMEM = "ok";
const char on[] PROGMEM = "on";
const char off[] PROGMEM = "off";
const char accepted[] PROGMEM = "accepted";
const char mac_tx_ok[] PROGMEM = "mac_tx_ok";
const char mac_rx[] PROGMEM = "mac_rx";
const char rn2483[] PROGMEM = "RN2483";

const char *const compare_table[] PROGMEM = {ok, on, off, accepted, mac_tx_ok, mac_rx, rn2483};

#define CMP_OK 0
#define CMP_ON 1
#define CMP_OFF 2
#define CMP_ACCEPTED 3
#define CMP_MAC_TX_OK 4
#define CMP_MAC_RX 5
#define CMP_RN2483 6

#define SENDING "Sending: "
#define SEND_MSG "\r\n"

const char eui[] PROGMEM = "EUI: ";
const char battery[] PROGMEM = "Battery: ";
const char appEui[] PROGMEM = "AppEUI: ";
const char devEui[] PROGMEM = "DevEUI: ";
const char band[] PROGMEM = "Band: ";
const char data_rate[] PROGMEM = "Data Rate: ";
const char rx_delay_1[] PROGMEM = "RX Delay 1: ";
const char rx_delay_2[] PROGMEM = "RX Delay 2: ";
const char version[] PROGMEM = "Version: ";
const char model[] PROGMEM = "Model: ";
const char devaddr[] PROGMEM = "DevAddr: ";

const char *const show_table[] PROGMEM = {eui, battery, appEui, devEui, band, data_rate, rx_delay_1, rx_delay_2, version, model, devaddr};

#define SHOW_EUI 0
#define SHOW_BATTERY 1
#define SHOW_APPEUI 2
#define SHOW_DEVEUI 3
#define SHOW_BAND 4
#define SHOW_DATA_RATE 5
#define SHOW_RX_DELAY_1 6
#define SHOW_RX_DELAY_2 7
#define SHOW_VERSION 8
#define SHOW_MODEL 9
#define SHOW_DEVADDR 10

const char invalid_sf[] PROGMEM = "Invalid SF";
const char invalid_fp[] PROGMEM = "Invalid frequency plan";
const char unexpected_response[] PROGMEM = "Unexpected response: ";
const char send_command_failed[] PROGMEM = "Send command failed";
const char join_failed[] PROGMEM = "Send join command failed";
const char join_not_accepted[] PROGMEM = "Join not accepted: ";
const char personalize_not_accepted[] PROGMEM = "Personalize not accepted";
const char response_is_not_ok[] PROGMEM = "Response is not OK: ";
const char error_key_length[] PROGMEM = "One or more keys are of invalid length.";
const char check_configuration[] PROGMEM = "Check your coverage, keys and backend status.";

const char *const error_msg[] PROGMEM = {invalid_sf, invalid_fp, unexpected_response, send_command_failed, join_failed, join_not_accepted, personalize_not_accepted, response_is_not_ok, error_key_length, check_configuration};

#define ERR_INVALID_SF 0
#define ERR_INVALID_FP 1
#define ERR_UNEXPECTED_RESPONSE 2
#define ERR_SEND_COMMAND_FAILED 3
#define ERR_JOIN_FAILED 4
#define ERR_JOIN_NOT_ACCEPTED 5
#define ERR_PERSONALIZE_NOT_ACCEPTED 6
#define ERR_RESPONSE_IS_NOT_OK 7
#define ERR_KEY_LENGTH 8
#define ERR_CHECK_CONFIGURATION 9

const char personalize_accepted[] PROGMEM = "Personalize accepted. Status: ";
const char join_accepted[] PROGMEM = "Join accepted. Status: ";
const char successful_transmission[] PROGMEM = "Successful transmission";
const char successful_transmission_received[] PROGMEM = "Successful transmission. Received ";

const char *const success_msg[] PROGMEM = {personalize_accepted, join_accepted, successful_transmission, successful_transmission_received};

#define SCS_PERSONALIZE_ACCEPTED 0
#define SCS_JOIN_ACCEPTED 1
#define SCS_SUCCESSFUL_TRANSMISSION 2
#define SCS_SUCCESSFUL_TRANSMISSION_RECEIVED 3

const char radio_prefix[] PROGMEM = "radio";
const char radio_set[] PROGMEM = "set";
const char radio_get[] PROGMEM = "get";
const char radio_get_bw[] PROGMEM = "bw";
const char radio_get_prlen[] PROGMEM = "prlen";
const char radio_get_crc[] PROGMEM = "crc";
const char radio_get_cr[] PROGMEM = "cr";
const char radio_get_sf[] PROGMEM = "sf";

const char *const radio_table[] PROGMEM = {radio_prefix, radio_set, radio_get, radio_get_bw, radio_get_prlen, radio_get_crc, radio_get_cr, radio_get_sf};

#define RADIO_PREFIX 0
#define RADIO_SET 1
#define RADIO_GET 2
#define RADIO_GET_BW 3
#define RADIO_GET_PRLEN 4
#define RADIO_GET_CRC 5
#define RADIO_GET_CR 6
#define RADIO_GET_SF 7

const char sys_prefix[] PROGMEM = "sys";
const char sys_sleep[] PROGMEM = "sleep";
const char sys_reset[] PROGMEM = "reset";
const char sys_erase_fw[] PROGMEM = "eraseFW";
const char sys_factory_rst[] PROGMEM = "factoryRESET";
const char sys_set[] PROGMEM = "set";
const char sys_get[] PROGMEM = "get";
const char sys_get_ver[] PROGMEM = "ver";
const char sys_get_vdd[] PROGMEM = "vdd";
const char sys_get_hweui[] PROGMEM = "hweui";
const char sys_set_get_nvm[] PROGMEM = "nvm";
const char sys_set_pindig[] PROGMEM = "pindig";

const char *const sys_table[] PROGMEM = {sys_prefix, sys_sleep, sys_reset, sys_erase_fw, sys_factory_rst, sys_set, sys_get, sys_get_ver, sys_get_vdd, sys_get_hweui, sys_set_get_nvm, sys_set_pindig};

#define SYS_PREFIX 0
#define SYS_SLEEP 1
#define SYS_RESET 2
#define SYS_ERASE_FW 3
#define SYS_FACTORY_RST 4
#define SYS_SET 5
#define SYS_GET 6
#define SYS_GET_VER 7
#define SYS_GET_VDD 8
#define SYS_GET_HWEUI 9
#define SYS_SET_GET_NVM 10
#define SYS_SET_PINDIG 11

const char mac_prefix[] PROGMEM = "mac";
const char mac_reset[] PROGMEM = "reset";
const char mac_tx[] PROGMEM = "tx";
const char mac_join[] PROGMEM = "join";
const char mac_save[] PROGMEM = "save";
const char mac_force_enable[] PROGMEM = "forceENABLE";
const char mac_pause[] PROGMEM = "pause";
const char mac_resume[] PROGMEM = "resume";
const char mac_set[] PROGMEM = "set";
const char mac_get[] PROGMEM = "get";

const char *const mac_table[] PROGMEM = {mac_prefix, mac_reset, mac_tx, mac_join, mac_save, mac_force_enable, mac_pause, mac_resume, mac_set, mac_get};

#define MAC_PREFIX 0
#define MAC_RESET 1
#define MAC_TX 2
#define MAC_JOIN 3
#define MAC_SAVE 4
#define MAC_FORCE_ENABLE 5
#define MAC_PAUSE 6
#define MAC_RESUME 7
#define MAC_SET 8
#define MAC_GET 9

const char mac_devaddr[] PROGMEM = "devaddr";
const char mac_deveui[] PROGMEM = "deveui";
const char mac_appeui[] PROGMEM = "appeui";
const char mac_nwkskey[] PROGMEM = "nwkskey";
const char mac_appskey[] PROGMEM = "appskey";
const char mac_appkey[] PROGMEM = "appkey";
const char mac_pwridx[] PROGMEM = "pwridx";
const char mac_dr[] PROGMEM = "dr";
const char mac_adr[] PROGMEM = "adr";
const char mac_bat[] PROGMEM = "bat";
const char mac_retx[] PROGMEM = "retx";
const char mac_linkchk[] PROGMEM = "linkchk";
const char mac_rxdelay1[] PROGMEM = "rxdelay1";
const char mac_rxdelay2[] PROGMEM = "rxdelay2";
const char mac_band[] PROGMEM = "band";
const char mac_ar[] PROGMEM = "ar";
const char mac_rx2[] PROGMEM = "rx2";
const char mac_ch[] PROGMEM = "ch";

const char *const mac_options[] PROGMEM = {mac_devaddr, mac_deveui, mac_appeui, mac_nwkskey, mac_appskey, mac_appkey, mac_pwridx, mac_dr, mac_adr, mac_bat, mac_retx, mac_linkchk, mac_rxdelay1, mac_rxdelay2, mac_band, mac_ar, mac_rx2, mac_ch};

#define MAC_DEVADDR 0
#define MAC_DEVEUI 1
#define MAC_APPEUI 2
#define MAC_NWKSKEY 3
#define MAC_APPSKEY 4
#define MAC_APPKEY 5
#define MAC_PWRIDX 6
#define MAC_DR 7
#define MAC_ADR 8
#define MAC_BAT 9
#define MAC_RETX 10
#define MAC_LINKCHK 11
#define MAC_RXDELAY1 12
#define MAC_RXDELAY2 13
#define MAC_BAND 14
#define MAC_AR 15
#define MAC_RX2 16
#define MAC_CH 17

const char mac_join_mode_otaa[] PROGMEM = "otaa";
const char mac_join_mode_abp[] PROGMEM = "abp";

const char *const mac_join_mode[] PROGMEM = {mac_join_mode_otaa, mac_join_mode_abp};

#define MAC_JOIN_MODE_OTAA 0
#define MAC_JOIN_MODE_ABP 1

const char channel_dcycle[] PROGMEM = "dcycle";
const char channel_drrange[] PROGMEM = "drrange";
const char channel_freq[] PROGMEM = "freq";
const char channel_status[] PROGMEM = "status";

const char *const mac_ch_options[] PROGMEM = {channel_dcycle, channel_drrange, channel_freq, channel_status};

#define MAC_CHANNEL_DCYCLE 0
#define MAC_CHANNEL_DRRANGE 1
#define MAC_CHANNEL_FREQ 2
#define MAC_CHANNEL_STATUS 3

const char mac_tx_type_cnf[] PROGMEM = "cnf";
const char mac_tx_type_ucnf[] PROGMEM = "uncnf";

const char *const mac_tx_table[] PROGMEM = {mac_tx_type_cnf, mac_tx_type_ucnf};

#define MAC_TX_TYPE_CNF 0
#define MAC_TX_TYPE_UCNF 1

#define MAC_TABLE 0
#define MAC_GET_SET_TABLE 1
#define MAC_JOIN_TABLE 2
#define MAC_CH_TABLE 3
#define MAC_TX_TABLE 4
#define SYS_TABLE 5
#define RADIO_TABLE 6
#define ERR_MESSAGE 7
#define SUCCESS_MESSAGE 8

uint8_t digits(uint8_t port)
{
  if (port >= 100)
  {
    return 3;
  }
  else if (port >= 10)
  {
    return 2;
  }
  return 1;
}

uint8_t receivedPort(const char *s)
{
  uint8_t port = 0;
  uint8_t i = 0;
  while (s[i] != ' ' && s[i] != '\0')
  {
    port *= 10;
    port += s[i] - 48;
    i++;
  }
  return port;
}

void clearReadBuffer()
{
  while (modemStream->available())
  {
    modemStream->read();
  }
}


void configureEU868()
{
  sendMacSet(MAC_RX2, "3 869525000");
  sendChSet(MAC_CHANNEL_DRRANGE, 1, "0 6");

  char buf[10];
  uint32_t freq = 867100000;
  uint8_t ch;
  for (ch = 0; ch < 8; ch++)
  {
    sendChSet(MAC_CHANNEL_DCYCLE, ch, "799");
    if (ch > 2)
    {
      sprintf(buf, "%lu", freq);
      sendChSet(MAC_CHANNEL_FREQ, ch, buf);
      sendChSet(MAC_CHANNEL_DRRANGE, ch, "0 5");
      sendChSet(MAC_CHANNEL_STATUS, ch, "on");
      freq = freq + 200000;
    }
  }
  sendMacSet(MAC_PWRIDX, TTN_PWRIDX_EU868);
}

void configureUS915(uint8_t fsb)
{
  uint8_t ch;
  uint8_t chLow = fsb > 0 ? (fsb - 1) * 8 : 0;
  uint8_t chHigh = fsb > 0 ? chLow + 7 : 71;
  uint8_t ch500 = fsb + 63;
  for (ch = 0; ch < 72; ch++)
  {
    if (ch == ch500 || (ch <= chHigh && ch >= chLow))
    {
      sendChSet(MAC_CHANNEL_STATUS, ch, "on");
      if (ch < 63)
      {
        sendChSet(MAC_CHANNEL_DRRANGE, ch, "0 3");
      }
    }
    else
    {
      sendChSet(MAC_CHANNEL_STATUS, ch, "off");
    }
  }
  sendMacSet(MAC_PWRIDX, TTN_PWRIDX_US915);
}

void configureAS920_923()
{
  sendMacSet(MAC_ADR, "off"); // TODO: remove when ADR is implemented for this plan
  sendMacSet(MAC_RX2, "2 923200000");

  char buf[10];
  uint32_t freq = 922000000;
  uint8_t ch;
  for (ch = 0; ch < 8; ch++)
  {
    sendChSet(MAC_CHANNEL_DCYCLE, ch, "799");
    if (ch > 1)
    {
      sprintf(buf, "%lu", freq);
      sendChSet(MAC_CHANNEL_FREQ, ch, buf);
      sendChSet(MAC_CHANNEL_DRRANGE, ch, "0 5");
      sendChSet(MAC_CHANNEL_STATUS, ch, "on");
      freq = freq + 200000;
    }
  }
  // TODO: SF7BW250/DR6 channel, not properly supported by RN2903AS yet
  //sendChSet(MAC_CHANNEL_DCYCLE, 8, "799");
  //sendChSet(MAC_CHANNEL_FREQ, 8, "922100000");
  //sendChSet(MAC_CHANNEL_DRRANGE, 8, "6 6");
  //sendChSet(MAC_CHANNEL_STATUS, 8, "on");
  // TODO: Add FSK channel
  sendMacSet(MAC_PWRIDX, TTN_PWRIDX_AS920_923);
}

void configureChannels(enum ttn_fp_t fp, uint8_t fsb)
{
  switch (fp)
  {
  case TTN_FP_EU868:
    configureEU868();
    break;
  case TTN_FP_US915:
    configureUS915(fsb);
    break;
  case TTN_FP_AS920_923:
    configureAS920_923();
    break;
  default:
    debugPrintMessage(ERR_MESSAGE, ERR_INVALID_FP);
    break;
  }
  sendMacSet(MAC_RETX, TTN_RETX);
}

bool setSF(enum ttn_fp_t fp, uint8_t sf)
{
  uint8_t dr = 0;
  switch (fp)
  {
  case TTN_FP_EU868:
  case TTN_FP_AS920_923:
    dr = 12 - sf;
    break;
  case TTN_FP_US915:
    dr = 10 - sf;
    break;
  }
  char s[2];
  s[0] = '0' + dr;
  s[1] = '\0';
  return sendMacSet(MAC_DR, s);
}

void sendCommand(uint8_t table, uint8_t index, bool appendSpace)
{
  char command[100];
  switch (table)
  {
  case MAC_TABLE:
    strcpy_P(command, (char *)pgm_read_word(&(mac_table[index])));
    break;
  case MAC_GET_SET_TABLE:
    strcpy_P(command, (char *)pgm_read_word(&(mac_options[index])));
    break;
  case MAC_JOIN_TABLE:
    strcpy_P(command, (char *)pgm_read_word(&(mac_join_mode[index])));
    break;
  case MAC_CH_TABLE:
    strcpy_P(command, (char *)pgm_read_word(&(mac_ch_options[index])));
    break;
  case MAC_TX_TABLE:
    strcpy_P(command, (char *)pgm_read_word(&(mac_tx_table[index])));
    break;
  case SYS_TABLE:
    strcpy_P(command, (char *)pgm_read_word(&(sys_table[index])));
    break;
  case RADIO_TABLE:
    strcpy_P(command, (char *)pgm_read_word(&(radio_table[index])));
    break;
  default:
    return;
  }
  modemStream->write(command);
  if (appendSpace)
  {
    modemStream->write(" ");
  }
}

bool sendMacSet(uint8_t index, const char *value)
{
  clearReadBuffer();
  debugPrint(SENDING);
  sendCommand(MAC_TABLE, MAC_PREFIX, true);
  sendCommand(MAC_TABLE, MAC_SET, true);
  sendCommand(MAC_GET_SET_TABLE, index, true);
  modemStream->write(value);
  modemStream->write(SEND_MSG);
  debugPrintLn(value);
  return waitForOk();
}

bool sendChSet(uint8_t index, uint8_t channel, const char *value)
{
  clearReadBuffer();
  char ch[5];
  if (channel > 9)
  {
    ch[0] = ((channel - (channel % 10)) / 10) + 48;
    ch[1] = (channel % 10) + 48;
    ch[2] = '\0';
  }
  else
  {
    ch[0] = channel + 48;
    ch[1] = '\0';
  }
  debugPrint(F(SENDING));
  sendCommand(MAC_TABLE, MAC_PREFIX, true);
  sendCommand(MAC_TABLE, MAC_SET, true);
  sendCommand(MAC_GET_SET_TABLE, MAC_CH, true);
  sendCommand(MAC_CH_TABLE, index, true);
  modemStream->write(ch);
  modemStream->write(" ");
  modemStream->write(value);
  modemStream->write(SEND_MSG);
  debugPrint(channel);
  debugPrint(F(" "));
  debugPrintLn(value);
  return waitForOk();
}

bool sendJoinSet(uint8_t type)
{
  clearReadBuffer();
  debugPrint(F(SENDING));
  sendCommand(MAC_TABLE, MAC_PREFIX, true);
  sendCommand(MAC_TABLE, MAC_JOIN, true);
  sendCommand(MAC_JOIN_TABLE, type, false);
  modemStream->write(SEND_MSG);
  debugPrintLn();
  return waitForOk();
}

bool sendPayload(uint8_t mode, uint8_t port, uint8_t *payload, size_t length)
{
  clearReadBuffer();
  debugPrint(F(SENDING));
  sendCommand(MAC_TABLE, MAC_PREFIX, true);
  sendCommand(MAC_TABLE, MAC_TX, true);
  sendCommand(MAC_TX_TABLE, mode, true);
  char sport[4];
  if (port > 99)
  {
    sport[0] = ((port - (port % 100)) / 100) + 48;
    sport[1] = (((port % 100) - (port % 10)) / 10) + 48;
    sport[2] = (port % 10) + 48;
    sport[3] = '\0';
  }
  else if (port > 9)
  {
    sport[0] = ((port - (port % 10)) / 10) + 48;
    sport[1] = (port % 10) + 48;
    sport[2] = '\0';
  }
  else
  {
    sport[0] = port + 48;
    sport[1] = '\0';
  }
  modemStream->write(sport);
  modemStream->write(" ");
  debugPrint(sport);
  debugPrint(F(" "));
  uint8_t i = 0;
  for (i = 0; i < length; i++)
  {
    if (payload[i] < 16)
    {
      modemStream->write("0");
      modemStream->print(payload[i], HEX);
      debugPrint(F("0"));
      debugPrintHex(payload[i], HEX);
    }
    else
    {
      modemStream->print(payload[i], HEX);
      debugPrintHex(payload[i], HEX);
    }
  }
  modemStream->write(SEND_MSG);
  debugPrintLn();
  return waitForOk();
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//
// Everything below here is how I expose this function to the rest of my code
//
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// My modem stream emulation
bool modem_stream_available() { return false; }
void modem_stream_read() {}
void modem_stream_write(const char *str) {
    uint16_t len;
    // We'll process this write if it's the right target
    bool processcmd = (thiscmdno == targetcmdno);
    // See if this is SEND_MSG in the most efficient way possible, knowing what SEND_MSG actually is
    if (str[0] == SEND_MSG[0] && str[1] == SEND_MSG[1] && str[2] == SEND_MSG[2]) {
        thiscmdno++;
        if (processcmd) {
            // Exit WITHOUT appending the SEND_MSG terminator, because the caller doesn't want it
            targetcmdfound = true;
            return;
        }
    }
    // Exit if we're not working on the target command
    if (!processcmd)
        return;
    // Process it by copying it into the target buffer
    len = strlen(str);
    BufferNeeded += len;
    if (len > BufferLeft)
        len = BufferLeft;
    if (len) {
        memcpy(pBuffer, str, len);
        BufferLeft -= len;
        pBuffer += len;
        *pBuffer = '\0';
    }
}
void modem_stream_print(uint8_t databyte, uint8_t format) {
    char out[3];
    out[0] = 
    out[2] = '\0';
    HexChars(databyte, &out[0], &out[1]);
    modem_stream_write(out);
}

// This is the only function exposed by this method.  The semantic is such that
// the caller requests "command #N" that needs to be sent to the device.
bool lorafp_get_command(char *region, bool loraWAN, uint16_t cmdno, char *buffer, uint16_t length) {
    uint8_t sf = TTN_DEFAULT_SF;
    uint8_t fsb = TTN_DEFAULT_FSB;
    enum ttn_fp_t fp;

    // Convert from textual to internal TTN region codes
    if (region[2] != '\0')
        return false;
    char ch0 = tolower((int)region[0]);
    char ch1 = tolower((int)region[1]);
    if (ch0 == 'e' && ch1 == 'u')
        fp = TTN_FP_EU868;
    else if (ch0 == 'u' && ch1 == 's')
        fp = TTN_FP_US915;
    else if (ch0 == 'a' && ch1 == 's')
        fp = TTN_FP_AS920_923;
    else
        return false;

    // Process Lora mode
    if (!loraWAN) {

        switch (fp) {

        case TTN_FP_EU868: {
            char *eu_commands[] = {
                "radio set mod lora",
                "radio set freq 868100000",
                "radio set pwr 15",
            };
            if (cmdno < (sizeof(eu_commands)/sizeof(char *))) {
                strlcpy(buffer, eu_commands[cmdno], length);
                return true;
            }
            break;
        }

        case TTN_FP_US915: {
            char *us_commands[] = {
                "radio set mod lora",
                "radio set freq 915000000",
                "radio set pwr 20",
            };
            if (cmdno < (sizeof(us_commands)/sizeof(char *))) {
                strlcpy(buffer, us_commands[cmdno], length);
                return true;
            }
            break;
        }

        case TTN_FP_AS920_923: {
            char *as_commands[] = {
                "radio set mod lora",
                "radio set freq 920000000",
                "radio set pwr 20",
            };
            if (cmdno < (sizeof(as_commands)/sizeof(char *))) {
                strlcpy(buffer, as_commands[cmdno], length);
                return true;
            }
            break;
        }

        }
        return false;;
    }

    // Set up for parsing
    thiscmdno = 0;
    targetcmdno = cmdno;
    targetcmdfound = false;
    pBuffer = buffer;
    BufferLeft = length - 1;
    BufferNeeded = 0;
    *pBuffer = '\0';
    modem.available = modem_stream_available;
    modem.read = modem_stream_read;
    modem.write = modem_stream_write;
    modem.print = modem_stream_print;
    modemStream = &modem;

    // Call the TTN method to configure channels
    configureChannels(fp, fsb);
    setSF(fp, sf);

    if (BufferLeft == 0)
        DEBUG_PRINTF("WARNING: LORAFP buffer length==%d needed==%d\n", length, BufferNeeded);

    return targetcmdfound;

}

#endif // LORA