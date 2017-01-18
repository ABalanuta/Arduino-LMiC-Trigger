/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example transmits data on hardcoded channel and receives data
 * when not transmitting. Running this sketch on two nodes should allow
 * them to communicate.
 *******************************************************************************/

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>


// How often to send a packet. Note that this sketch bypasses the normal
// LMIC duty cycle limiting, so when you change anything in this sketch
// (payload length, frequency, spreading factor), be sure to check if
// this interval should not also be increased.
// See this spreadsheet for an easy airtime and duty cycle calculator:
// https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc

#define NODE_2

//#define RX_FUNCT

#define Freq1 867500000
#define Freq2 867700000
#define Freq3 867900000
#define Freq4 868100000
#define Freq5 868300000
#define Freq6 868500000

#if  defined(NODE_1)
  #define TX_POWER 15  //dBm goes from 2 to 15
  #define TX_DATARATE DR_SF7 //from 7 to 12
#elif defined(NODE_2)
  #define TX_POWER 0
  #define TX_DATARATE DR_SF8 //from 7 to 12
//#elif defined(NODE_3)
//  #define TX_POWER 0
//  #define TX_DATARATE DR_SF9 //from 7 to 12
#endif

//#define TX_DELAY 40  //This - LED_ON_DELAY
#define TX_FREQ Freq1

//#define TX_MAX_PACKETS 1000

#define LED_PIN 9
#define TRIGGER_PIN 6
//#define LED_ON_DELAY 10 //ms


static const char mydata[] = "1";

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x11, 0x22, 0x33,
                                          0x44, 0x55, 0x66, 0x77,
                                          0x88, 0x99, 0xaa, 0xbb,
                                          0xcc, 0xdd, 0xee, 0xff };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x11, 0x22, 0x33,
                                          0x44, 0x55, 0x66, 0x77,
                                          0x88, 0x99, 0xaa, 0xbb,
                                          0xcc, 0xdd, 0xee, 0xff };
//00112233445566778899aabbccddeeff

// LoRaWAN end-device address (DevAddr)
//static const u4_t DEVADDR = 0x07931236 ; // <-- Change this address for every node!

#if defined(NODE_1)
  static const u4_t DEVADDR = 0x06f0e61a	 ; // Viking
#elif defined(NODE_2)
  static const u4_t DEVADDR = 0x06f42b21 ; // Aztecs
#endif

//static const u4_t DEVADDR = 0x??? ; // Romans
//static const u4_t DEVADDR = 0x??? ; // Persian
//static const u4_t DEVADDR = 0x06f42b2e ; // Aztecs

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// Aux functions for the TX messages to work
static void aes_cipher (xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t payload, int len);
static void aes_appendMic (xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t pdu, int len);
static void micB0 (u4_t devaddr, u4_t seqno, int dndir, int len);
static void tx_func (osjob_t* job);
static void txdone_func (osjob_t* job);
void onEvent (ev_t ev);
void buildTxFrame();
void tx(const char *str, osjobcb_t func);

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};

osjob_t txjob;
osjob_t timeoutjob;


// log text to USART and toggle LED
static void tx_func (osjob_t* job) {

  static uint32_t count = 0;
  int waiting = 0;

  while (true) {

    while (!digitalRead(TRIGGER_PIN)) {/* Do nothing*/}

    // say hello
    digitalWrite(LED_PIN, HIGH); // LED on
    tx(mydata, txdone_func);

    waiting = 0;
    while(digitalRead(lmic_pins.dio[0]) == 0){
      delayMicroseconds(1);
      waiting++;
    }
    digitalWrite(LED_PIN, LOW); // LED off, packet has been sent

    Serial.print(count++);
    Serial.print(":    Powar: ");
    Serial.print(LMIC.txpow);

    Serial.print(" Waited ");
    Serial.print(waiting);
    Serial.println("us");


  }


  //if(count >= TX_MAX_PACKETS)
  //  while (1) {
      /* code */
  //}

  // reschedule job every TX_INTERVAL (plus a bit of random to prevent
  // systematic collisions), unless packets are received, then rx_func
  // will reschedule at half this time.
  //os_setTimedCallback(job, os_getTime() + ms2osticks(TX_DELAY), tx_func);
}

// application entry point
void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  pinMode(LED_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, INPUT);

  // initialize runtime env
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  #ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  #else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  // Use a frequency in the g3 which allows 10% duty cycling.
  LMIC.freq = TX_FREQ;

  // Maximum TX power
  LMIC.txpow = TX_POWER;
  // Use a medium spread factor. This can be increased up to SF12 for
  // better range, but then the interval should be (significantly)
  // lowered to comply with duty cycle limits as well.
  LMIC.datarate = TX_DATARATE;
  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

  // This defines the port, if left at zero will cause erros on the server
  LMIC.pendTxPort = 1;

  // You must also enable Relaxed Frame Counter on the LoRa Server in order
  // for the node to be able to use any frame counter

  Serial.println("Started");
  Serial.flush();

  // setup initial job
  os_setCallback(&txjob, tx_func);
}

void loop() {
  // execute scheduled jobs and events
  os_runloop_once();
}

// Do nothing
void onEvent (ev_t ev) {}

void show_frame(int z){
  Serial.print("[");
  if (z<10)
    Serial.print(0);
  Serial.print(z);
  Serial.print("]");
  Serial.print("[");
  for(int i = 0; i<LMIC.dataLen; i++)
    Serial.print(LMIC.frame[i],HEX);
  Serial.print("] ");
  Serial.print(LMIC.dataLen);
  Serial.println(" bytes");
}

// Transmit the given string and call the given function afterwards
void tx(const char *str, osjobcb_t func) {

  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet

  LMIC.dataLen = 0;
  //while (*str)
  //  LMIC.pendTxData[LMIC.dataLen++] = *str++;

  if( (xref2u1_t)mydata != (xref2u1_t)0 )
      os_copyMem(LMIC.pendTxData, mydata, sizeof(mydata));
  LMIC.pendTxConf = 0;
  LMIC.pendTxPort = 1;
  LMIC.pendTxLen  = sizeof(mydata);
  LMIC.txCnt = 0;

  //show_frame(0);

  //LMIC.pendTxLen = LMIC.dataLen;
  LMIC.opmode = OP_TXDATA;

  buildTxFrame();

  //show_frame(99);

  LMIC.osjob.func = func;
  os_radio(RADIO_TX);
}

void buildTxFrame(){
  bit_t txdata = ((LMIC.opmode & (OP_TXDATA|OP_POLL)) != OP_POLL);
  u1_t dlen = txdata ? LMIC.pendTxLen : 0;
  int  end = OFF_DAT_OPTS;
  ASSERT(end <= OFF_DAT_OPTS+16);
  u1_t flen = end + (txdata ? 5+dlen : 4);

  //show_frame(2);

  if( flen > MAX_LEN_FRAME ) {
      // Options and payload too big - delay payload
      txdata = 0;
      flen = end+4;
  }
  LMIC.frame[OFF_DAT_HDR] = HDR_FTYPE_DAUP | HDR_MAJOR_V1;
  LMIC.frame[OFF_DAT_FCT] = (LMIC.dnConf | LMIC.adrEnabled
                            | (LMIC.adrAckReq >= 0 ? FCT_ADRARQ : 0)
                            | (end-OFF_DAT_OPTS));
  //show_frame(3);
  os_wlsbf4(LMIC.frame+OFF_DAT_ADDR, LMIC.devaddr);
  //show_frame(4);

  if( LMIC.txCnt == 0 ) {
      LMIC.seqnoUp += 1;
      DO_DEVDB(LMIC.seqnoUp,seqnoUp);
  } else {
      EV(devCond, INFO, (e_.reason = EV::devCond_t::RE_TX,
                         e_.eui    = MAIN::CDEV->getEui(),
                         e_.info   = LMIC.seqnoUp-1,
                         e_.info2  = ((LMIC.txCnt+1) |
                                      (TABLE_GET_U1(DRADJUST, LMIC.txCnt+1) << 8) |
                                      ((LMIC.datarate|DR_PAGE)<<16))));
  }
  //show_frame(5);
  os_wlsbf2(LMIC.frame+OFF_DAT_SEQNO, LMIC.seqnoUp-1);
  //show_frame(6);
  // Clear pending DN confirmation
  LMIC.dnConf = 0;

  if( txdata ) {
      if( LMIC.pendTxConf ) {
          // Confirmed only makes sense if we have a payload (or at least a port)
          LMIC.frame[OFF_DAT_HDR] = HDR_FTYPE_DCUP | HDR_MAJOR_V1;
          if( LMIC.txCnt == 0 ) LMIC.txCnt = 1;
          //show_frame(7);
      }
      LMIC.frame[end] = LMIC.pendTxPort;
      //show_frame(8);
      os_copyMem(LMIC.frame+end+1, LMIC.pendTxData, dlen);
      //show_frame(9);
      aes_cipher(LMIC.pendTxPort==0 ? LMIC.nwkKey : LMIC.artKey,
                 LMIC.devaddr, LMIC.seqnoUp-1,
                 /*up*/0, LMIC.frame+end+1, dlen);
      //show_frame(10);
  }
  aes_appendMic(LMIC.nwkKey, LMIC.devaddr, LMIC.seqnoUp-1, /*up*/0, LMIC.frame, flen-4);
  //show_frame(11);

  EV(dfinfo, DEBUG, (e_.deveui  = MAIN::CDEV->getEui(),
                     e_.devaddr = LMIC.devaddr,
                     e_.seqno   = LMIC.seqnoUp-1,
                     e_.flags   = (LMIC.pendTxPort < 0 ? EV::dfinfo_t::NOPORT : EV::dfinfo_t::NOP),
                     e_.mic     = Base::lsbf4(&LMIC.frame[flen-4]),
                     e_.hdr     = LMIC.frame[LORA::OFF_DAT_HDR],
                     e_.fct     = LMIC.frame[LORA::OFF_DAT_FCT],
                     e_.port    = LMIC.pendTxPort,
                     e_.plen    = txdata ? dlen : 0,
                     e_.opts.length = end-LORA::OFF_DAT_OPTS,
                     memcpy(&e_.opts[0], LMIC.frame+LORA::OFF_DAT_OPTS, end-LORA::OFF_DAT_OPTS)));
  //show_frame(12);
  LMIC.dataLen = flen;
  //show_frame(13);

}

static void aes_cipher (xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t payload, int len) {
    if( len <= 0 )
        return;
    os_clearMem(AESaux, 16);
    AESaux[0] = AESaux[15] = 1; // mode=cipher / dir=down / block counter=1
    AESaux[5] = dndir?1:0;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
    os_copyMem(AESkey,key,16);
    os_aes(AES_CTR, payload, len);
}

static void aes_appendMic (xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t pdu, int len) {
    micB0(devaddr, seqno, dndir, len);
    os_copyMem(AESkey,key,16);
    // MSB because of internal structure of AES
    os_wmsbf4(pdu+len, os_aes(AES_MIC, pdu, len));
}

static void micB0 (u4_t devaddr, u4_t seqno, int dndir, int len) {
    os_clearMem(AESaux,16);
    AESaux[0]  = 0x49;
    AESaux[5]  = dndir?1:0;
    AESaux[15] = len;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
}

static void txdone_func (osjob_t* job) {
  //rx(rx_func);
}

#ifdef RX_FUNCT

// Enable rx mode and call func when a packet is received
void rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable "continuous" RX (e.g. without a timeout, still stops after
  // receiving a packet)
  os_radio(RADIO_RXON);
  Serial.println("RX");
}

static void rxtimeout_func(osjob_t *job) {
  digitalWrite(LED_BUILTIN, LOW); // off
}

static void rx_func (osjob_t* job) {
  // Blink once to confirm reception and then keep the led on
  digitalWrite(LED_BUILTIN, LOW); // off
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH); // on

  // Timeout RX (i.e. update led status) after 3 periods without RX
  os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(3*TX_DELAY), rxtimeout_func);

  // Reschedule TX so that it should not collide with the other side's
  // next TX
  os_setTimedCallback(&txjob, os_getTime() + ms2osticks(TX_DELAY/2), tx_func);

  Serial.print("Got ");
  Serial.print(LMIC.dataLen);
  Serial.println(" bytes");
  Serial.write(LMIC.frame, LMIC.dataLen);
  Serial.println();

  // Restart RX
  rx(rx_func);
}

#endif /* RX_FUNCT */
