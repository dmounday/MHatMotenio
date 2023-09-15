// **********************************************************************************
// Don Mounday:
// In general removed features and code that is not used by the PoolSmartz package and
// the pool sensors. No LCD, no buzzer.
// Heavily edited, code reformating and style change. More use of modern C++ features to
// experiment with code size and optimizations.
// Replaced float operations with fixed point.
// EEPROM is used to save the last power on/off state. If the mains power fails
// and the low battery voltage results in a RPi shutdown, then when mains power is restored the
// RPi is powered on if the last state is on and the battery is recharged.
// **********************************************************************************************************
// MightyHat gateway base unit sketch that works with MightyHat equipped with RFM69W/RFM69HW/RFM69CW/RFM69HCW
// This will relay all RF data over serial to the host computer (RaspberryPi) and vice versa.
// It will buffer the serial data to ensure host serial requests are not missed.
// http://LowPowerLab.com/MightyHat
// PiGateway project: http://LowPowerLab.com/gateway
// **********************************************************************************
// Copyright Felix Rusu 2020, http://www.LowPowerLab.com/contact
// **********************************************************************************
//

#include <EEPROM.h>

#define MHAT_VERSION 3  // latest is R4, only change to "2" if you have a MightyHat R2
// ****************************************************************************************
#include <PString.h>    //easy string manipulator: http://arduiniana.org/libraries/pstring/
#include <RFM69.h>      //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>  //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>  //get it here: https://github.com/lowpowerlab/RFM69
#include <SPIFlash.h>   //get it here: https://github.com/lowpowerlab/spiflash
#include <Streaming.h>  //easy C++ style output operators: http://arduiniana.org/libraries/streaming/

//*****************************************************************************************************************************
// ADJUST THE SETTINGS BELOW DEPENDING ON YOUR HARDWARE/SCENARIO !
//*****************************************************************************************************************************
#define NODEID 1               // the gateway has ID=1
#define NETWORKID 201          // all nodes on the same network can talk to each other
#define FREQUENCY RF69_433MHZ  // RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
//#define FREQUENCY_EXACT 916000000 //uncomment and set to a specific frequency in Hz, if commented the center frequency is used
#define ENCRYPTKEY "sampleEncryptKey"  // has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW_HCW                 // required for RFM69HW/HCW, comment out for RFM69W/CW!
#define ENABLE_ATC                     // comment out this line to disable AUTO TRANSMISSION CONTROL //more here: http://lowpowerlab.com/blog/2015/11/11/rfm69_atc-automatic-transmission-control/
//#define ENABLE_WIRELESS_PROGRAMMING    //comment out this line to disable Wireless Programming of this gateway node
//*****************************************************************************************************************************
#define SERIAL_BAUD 115200  // 
#define DEBUG_EN            // comment out if you don't want any serial verbose output (keep out in real use)

#define BTN_LED_RED 9
#define BTN_LED_GRN 6  // This will indicate when Pi has power

const uint8_t ON {1};
const uint8_t OFF{0};

#define BUTTON A2   // Power button pin
#define BUTTON1 A4  // Backlight control button
#define BUTTON2 A5  // Backlight control button
#define LATCH_EN 4
#define LATCH_VAL 7
#define SIG_SHUTOFF A3   // Signal to Pi to ask for a shutdown
#define SIG_BOOTOK A6    // Signal from Pi that it's OK to cutoff power
                         // !!NOTE!! Originally this was D7 but it was moved to A0 at least temporarily.
                         // On MightyBoost R1 you need to connect D7 and A0 with a jumper wire.
                         // The explanation for this is given here: http://lowpowerlab.com/mightyboost/#source
#define BATTERYSENSE A7  // Sense VBAT_COND signal (when powered externally should read ~3.25v/3.3v (1000-1023), when external power is cutoff
                         // it should start reading around 2.85v/3.3v * 1023 ~= 880 (ratio given by 10k+4.7K divider from VBAT_COND = 1.47 multiplier)
                         // hence the actual input voltage = analogRead(A7) * 0.00322 (3.3v/1024) * 1.47 (10k+4.7k voltage divider ratio)
                         // when plugged in this should be 4.80v, nothing to worry about
                         // when on battery power this should decrease from 4.15v (fully charged Lipoly) to 3.3v (discharged Lipoly)
                         // trigger a shutdown to the target device once voltage is around 3.4v to allow 30sec safe shutdown


// Lets try the above using fixed point math, avoiding runtime floating point.
// A 10 bit ADC step size at 5V 5/1024 = 0.004883V or at 3.3V 3.3/1024 = 0.003223V.
// Max output values:  1023*4883 = 4995309  and 1023*3223 = 327129
// Results of these multiples require int32_t types. Normalizing 1.023E3 and 4.883E3 
// and multiply yeilds 4.9953E6. Or 4.995V. 
const uint16_t LOWBATTERYTHRESHOLDmv {3500};
const uint16_t CHARGINGTHRESHOLDmv {4300};
//
// The voltage divider is BATT > 1Meg > A7 > 2.2Meg >GND
// A7pin volts = ADCReading * 3.223E3. So 1023*.003223 normalized is
// 1.023E3 * 3.223E3 = 3297129 or 3.29V. 
// Applying voltage drop of voltage divider is the inverse of 3.2/2.2 or 1.45
// 3297129/1000 = 3.297E3 * 1.45 ==> 3297*145 = 478065 with 5 decimal places.
// Divid by 100 to get milliVolts.
//
// To find a scaling factor that includes the voltage divider and the ADC step value
// at 3.3 volts:
// VDivScaling = 3.223E3 * 145 = 467335. Divid by 100 to reduce number magnitude = 4673.
// Then ADCReading 1023*4673 => 4780479/1000 or 4780mV
// or ADCReading 1*4673 => 4673/1000 = 4mv;
//
// We expect the constexpr function to be evaluated by the compiler.

constexpr uint32_t ADCVDivScaling()
{
    // Voltage divider inverse 3.2/2.2
    const uint32_t vdInverse{145};
    const uint32_t adcStepVal{3223};
    return (vdInverse*adcStepVal)/100; // returns 4 digit value
}
//
// Calculate millivolts by applying scalling and round off.
//
uint16_t battMilliVolts(uint16_t adcReading){
    return ((adcReading*ADCVDivScaling())+500)/1000;
}
uint16_t systemVoltageMV {5000};

// Times for power and button actions. Time in milliseconds.
const int16_t RESETHOLDTIME{500};          // Button must be hold this many mseconds before a reset is issued (should be much less than SHUTDOWNHOLDTIME)
const int16_t SHUTDOWNHOLDTIME{2000};      // Button must be hold this many mseconds before a shutdown sequence is started (should be much less than ForcedShutoffDelay)
const int16_t SHUTOFF_TRIGGER_DELAY{6000}; // will start checking the SIG_BOOTOK line after this long
const int16_t PI_SHUTOFF_TIME{20000};      // Time after shutoff pulse for SIGBOOTOK to go low.
const int16_t RESETPULSETIME{500};         // When reset is issued, the SHUTOFF signal is held HIGH this many ms
const int16_t FORCED_SHUTOFF_DELAY{7500};  // when SIG_BOOTOK==0 (PI in unknown state): if button is held
                                           // for this long, force shutdown (this should be less than RecycleTime)
const int16_t SHUTDOWN_FINAL_DELAY{4500};  // after shutdown signal is received, delay for this long
                                           // to allow all PI LEDs to stop activity (pulse LED faster)
const uint16_t RECYCLE_TIME{60000};        // window of time in which SIG_BOOTOK is expected to go HIGH
                                           // should be at least 3000 more than Min
                                           // if nothing happens after this window, if button is
                                           // still pressed, force cutoff power, otherwise switch back to normal ON state
//
const int16_t BATTERYREADINTERVAL{2000};
const uint8_t BAT_REPORT_INTERVALS{10};    // Report following N BATTERYREADINTERVALs

#ifdef DEBUG_EN
#define DEBUG(input) Serial.print(input)
#define DEBUGln(input) Serial.println(input)
#else
#define DEBUG(input)
#define DEBUGln(input)
#endif

#define PRINT_UPTIME Serial << F("UPTIME:") << millis() << endl;
#define PRINT_FREQUENCY Serial << F("SYSFREQ:") << radio.getFrequency() << endl;

#define LED_HIGH digitalWrite(LED_BUILTIN, HIGH)
#define LED_LOW digitalWrite(LED_BUILTIN, LOW)

//******************************************** BEGIN ADVANCED variables ********************************************************************************
#define RAMSIZE 2048
#define MAX_BUFFER_LENGTH 61       // limit parameter update requests to 40 chars. ex: Parameter:LongRequest
#define MAX_ACK_REQUEST_LENGTH 30  // 60 is max for ACK (with ATC enabled), but need to allow appending :OK and :INV to confirmations from node

typedef struct req {
    uint16_t nodeId;
    char data[MAX_BUFFER_LENGTH];  //+1 for the null terminator
    struct req* next;
} REQUEST;

// dynamically allocated queue (FIFO) data structure
REQUEST* queue = NULL;
byte size_of_queue = 0;
//******************************************** END ADVANCED variables ********************************************************************************
//******************************************** BEGIN FUNCTION prototypes ********************************************************************************
boolean bootOK();
void POWER(byte ON_OFF);
int freeRAM();
void handleSerialData();
void printQueue(REQUEST* p);
//******************************************** END FUNCTION prototypes ********************************************************************************
//******************************************** BEGIN GENERAL variables ********************************************************************************

byte PowerState {OFF};
byte restartPowerState {OFF};
long lastPeriod {-1};
int rssi {0};

boolean batteryLow {false};
boolean batteryLowShutdown {false};
uint8_t batteryRptInterval {0};

SPIFlash flash(SS_FLASHMEM, 0xEF30);  // EF30 for 4mbit Windbond FLASH MEM
#ifdef ENABLE_ATC
RFM69_ATC radio;
#else
RFM69 radio;
#endif
//******************************************** END GENERAL variables ********************************************************************************
//******************************************** Messages ********************************************************************************
char buff[80];
PString Pbuff(buff, sizeof(buff));  // easy string manipulator

enum LEDcolor {
    LED_RED,
    LED_GREEN,
    LED_ORANGE,
    LED_OFF
};
void ledControl(LEDcolor c) {
    switch (c) {
        case LED_RED:
            digitalWrite(BTN_LED_RED, HIGH);
            digitalWrite(BTN_LED_GRN, LOW);
            break;
        case LED_GREEN:
            digitalWrite(BTN_LED_RED, LOW);
            digitalWrite(BTN_LED_GRN, HIGH);
            break;
        case LED_ORANGE:
            digitalWrite(BTN_LED_RED, HIGH);
            digitalWrite(BTN_LED_GRN, HIGH);
            break;
        case LED_OFF:
            digitalWrite(BTN_LED_RED, LOW);
            digitalWrite(BTN_LED_GRN, LOW);
            break;
        default:
            break;
    }
}
void setRestartPowerState(byte ONOFF) {
    restartPowerState = ONOFF;
    EEPROM.write(2, ONOFF);
    Serial << "Restart Power State: " << ONOFF << endl;
}


boolean readBattery() {
    // periodically read the battery voltage
    int currPeriod = millis() / BATTERYREADINTERVAL;
    if (currPeriod != lastPeriod) {
        lastPeriod = currPeriod;
        uint16_t ain = analogRead(BATTERYSENSE);
        systemVoltageMV = battMilliVolts(ain);
        if ( ++batteryRptInterval > BAT_REPORT_INTERVALS) {
            Serial << "[1] MHBatt:" << systemVoltageMV << " ain:" << ain << endl;
            batteryRptInterval = 0;
        }
        batteryLow = systemVoltageMV < LOWBATTERYTHRESHOLDmv;
        return true;  // signal that batt has been read
    }
    return false;
}

void setupPowerControl() {
    pinMode(BUTTON, INPUT_PULLUP);
    pinMode(SIG_BOOTOK, INPUT);
    pinMode(SIG_SHUTOFF, OUTPUT);
    digitalWrite(SIG_SHUTOFF, LOW);  // added after sudden shutdown quirks, DO NOT REMOVE!
    pinMode(BTN_LED_RED, OUTPUT);
    pinMode(BTN_LED_GRN, OUTPUT);
    pinMode(LATCH_EN, OUTPUT);
    digitalWrite(LATCH_EN, LOW);
    pinMode(LATCH_VAL, OUTPUT);
    pinMode(BUTTON1, INPUT_PULLUP);
    pinMode(BUTTON2, INPUT_PULLUP);
    pinMode(BATTERYSENSE, INPUT);

    readBattery();
    // Read power state from EEPROM
    byte sig1 = EEPROM.read(0);
    byte sig2 = EEPROM.read(1);
    if (sig1 != 0x55 || sig2 != 0x22) {
        // First use, setup for saving restart power on/off state
        EEPROM.write(0, 0x55);  // signature bytes
        EEPROM.write(1, 0x22);
        EEPROM.write(2, OFF);  // state type: initialize to off.
    }
    restartPowerState = EEPROM.read(2);
    Serial << "EEPROM restart power state: " << restartPowerState << endl;
    Serial << "ADC scaling factor: " << ADCVDivScaling() << endl;
}

bool waitBootOK() {
    unsigned long start = millis();
    boolean recycleDetected = false;
    while (millis() - start < RECYCLE_TIME)  // blink LED while waiting for BOOTOK to go high
    {
        // blink 3 times and pause
        for (int i = 0; i < 3; ++i) {
            ledControl(LED_OFF);
            delay(100);
            ledControl(LED_ORANGE);
            delay(100);
        }
        delay(400);
        if (!bootOK())
            recycleDetected = true;
        else if (bootOK() && recycleDetected) {
            Serial << "Reboot OK!" << endl;
            return true;
        }
    }
    Serial << "Reboot failed." << endl;
    return false;
}

void forcePiShutdown() {
    Serial << "Forced shutdown.." << endl;
    PowerState = OFF;
    POWER(PowerState);
    Serial << "Pi is now OFF" << endl;
}

void shutdownPi() {
    if (batteryLow) {
        Serial << "[1] Battery low: " << systemVoltageMV << " mv Shutting down Pi.." << endl;
        batteryLowShutdown = true;
    } else {
        Serial << "[1] Switched OFF - Shutting down Pi.." << endl;
    }
    // signal Pi to shutdown (HIGH for > 1 sec)
    digitalWrite(SIG_SHUTOFF, HIGH);

    // now wait for the Pi to signal back
    unsigned long start = millis();
    byte blinkFlag {0};
    ledControl(LED_OFF);
    delay(SHUTOFF_TRIGGER_DELAY);
    while ((millis() - start) < PI_SHUTOFF_TIME) {
        // Pi signaling OK to turn off
        if (!bootOK()) {
            ledControl(LED_OFF);  // digitalWrite(POWER_LED, PowerState); //turn off LED to indicate power is being cutoff
            start = millis();
            while (millis() - start < SHUTDOWN_FINAL_DELAY) {
                if ( blinkFlag&0x01) {
                    ledControl(LED_RED);
                } else {
                    ledControl(LED_OFF);
                }
                ++blinkFlag;
                delayMicroseconds(300);
            }
            PowerState = OFF;
            POWER(PowerState);
            break;
        }
    }

    // Power is still on force shutdown
    if (PowerState == ON) {
        PowerState = OFF;
        setRestartPowerState(OFF);
        POWER(PowerState);
    }

    if (PowerState == OFF) {
        Serial << "Pi is now OFF" << endl;
    }
    digitalWrite(SIG_SHUTOFF, LOW);
}

void powerOnPi() {
    PowerState = ON;
    batteryLowShutdown = false;
    POWER(PowerState);
    setRestartPowerState(ON);
    Serial << "Pi is now ON" << endl;
}

enum ButtonAction {
    ACTION_RESET,
    ACTION_SHUTDOWN,
    ACTION_POWERON,
    ACTION_FORCESHUTDOWN,
    NO_ACTION
};
// Measure button hold time and return action;
// make sure the button is held down for at least 'RESETHOLDTIME' before taking action
//  (this is to avoid accidental button presses and consequently Pi shutdowns)
enum ButtonAction getButtonAction() {
    byte button = digitalRead(BUTTON);
    unsigned long start = millis();
    if (button == HIGH)
        return NO_ACTION;
    while ((millis() - start) < FORCED_SHUTOFF_DELAY) {
        if (millis()%100 > 50 )
            ledControl(LED_ORANGE);
        else
            ledControl(LED_OFF);
        if (digitalRead(BUTTON) != LOW) {
            // button has been relased
            unsigned long sampleNow = millis();
            if (sampleNow - start < RESETHOLDTIME) {  // just a button bounce
                return NO_ACTION;
            } else if (sampleNow - start < SHUTDOWNHOLDTIME) {  // reset time has passed. Reset.
                if (PowerState == ON) {
                    //Serial << "ACTION_RESET" << endl;
                    return ACTION_RESET;
                } else {
                    //Serial << "ACTION_POWERON pwr on reset time" << endl;
                    return ACTION_POWERON;
                }
            } else if (PowerState == ON) {
                //Serial << "ACTION_SHUTDOWN" << endl;
                return ACTION_SHUTDOWN;
            } else {
                //Serial << "ACTIon_POWERON pwr off shutdown time" << endl;
                return ACTION_POWERON;
            }
        }
    }
    // Button is still pressed. Time greater than FORCED_SHUTOFF_DELAY
    if (PowerState == ON) {
        //Serial << "ACTION_FORCE pwr on >force shutdown time" << endl;
        return ACTION_FORCESHUTDOWN;
    }
    // If PowerState OFF then No action on long button press.
    //Serial << "NO_ACTION pwr off, time > force shutdown time" << endl;
    return NO_ACTION;
}

// NOTE: A button press pulls the digital input LOW.
void handlePowerControl() {
    digitalWrite(SIG_SHUTOFF, LOW);  // added after sudden shutdown quirks, DO NOT REMOVE!

    // If battery is low and power is on then shutdown Pi.
    if (PowerState == ON && batteryLow) {
        shutdownPi();
        return;
    }
    // Check if system startup following a battery low shutdown and recharged battery.
    // (power failure time exceeded battery life)
    if (PowerState==OFF && (systemVoltageMV>=CHARGINGTHRESHOLDmv) && restartPowerState==ON) {
        powerOnPi();
        return;
    }
    // get any button action.
    switch (getButtonAction()) {
        case ACTION_RESET:
            ledControl(LED_ORANGE);
            if (bootOK())  // SIG_BOOTOK is HIGH so Pi is running the shutdowncheck.sh script, ready to intercept the RESET PULSE
            {
                Serial << "Rebooting Pi.." << endl;
                digitalWrite(SIG_SHUTOFF, HIGH);
                delay(RESETPULSETIME);
                digitalWrite(SIG_SHUTOFF, LOW);
                if (waitBootOK()) {
                    return;
                }
                return;  // reboot pulse sent but it appears a reboot failed; exit all checks
            }
            break;
        case ACTION_SHUTDOWN:
            ledControl(LED_ORANGE);
            restartPowerState = OFF;
            if (bootOK()) {
                shutdownPi();
            } else {
                forcePiShutdown();
            }
            break;
        case ACTION_FORCESHUTDOWN:
            ledControl(LED_ORANGE);
            forcePiShutdown();
            break;
        case ACTION_POWERON:
            ledControl(LED_ORANGE);
            powerOnPi();
            break;
        case NO_ACTION:
        default:
            break;
    }
    if (PowerState == ON)
        ledControl(LED_GREEN);
    else
        ledControl(LED_OFF);
}

#if 0
uint32_t buttonsLastChanged;
void handle2Buttons()
{
  if (millis() - buttonsLastChanged < 200)
    return; // basic button debouncing & prevent changing level too fast

  // button 1 - backlight
  if (digitalRead(BUTTON1) == LOW)
  {
    buttonsLastChanged = millis();

  }

  // button 2 - message history
  if (digitalRead(BUTTON2) == LOW)
  {
    buttonsLastChanged = millis();

  }
}
#endif

boolean bootOK() {
    return analogRead(SIG_BOOTOK) > 800;  // the BOOTOK signal is on an analog pin because a digital may not always pick it up (its less than 3.3v)
}

void POWER(byte onOff) {
    digitalWrite(LATCH_EN, HIGH);
    digitalWrite(LATCH_VAL, onOff);
    delay(5);
    digitalWrite(LATCH_EN, LOW);
    delay(5);
    Serial << "POWER: " << onOff << endl;
}


void setup() {
    Serial.begin(SERIAL_BAUD);
    pinMode(LED_BUILTIN, OUTPUT);
    LED_HIGH;
    setupPowerControl();

    radio.initialize(FREQUENCY, NODEID, NETWORKID);
#ifdef ENCRYPTKEY
    radio.encrypt(ENCRYPTKEY);
#endif
#ifdef IS_RFM69HW_HCW
    radio.setHighPower();
#endif
    Serial << endl
           << "GATEWAYSTART" << endl;
    PRINT_FREQUENCY;
    PRINT_UPTIME;

    if (!flash.initialize()) DEBUGln(F("DEBUG:SPI_Flash_Init_FAIL"));

#ifdef FREQUENCY_EXACT
    radio.setFrequency(FREQUENCY_EXACT);  // set frequency to some custom frequency
#endif
    DEBUG(F("FREERAM:"));
    DEBUGln(freeRAM());
    LED_LOW;
}

boolean newPacketReceived;
void handleRadio() {
    // process any received radio packets
    if (radio.receiveDone()) {
        LED_HIGH;
        rssi = radio.RSSI;      // get this asap from transceiver
        if (radio.DATALEN > 0)  // data packets have a payload
        {
            for (byte i = 9; i < radio.DATALEN; i++) {
                if (radio.DATA[i] == '\n' || radio.DATA[i] == '\r')
                    radio.DATA[i] = ' ';  // remove any newlines in the payload - this should only ever happen with noise data that actually made it through
            }
            Pbuff = "";
            Pbuff << '[' << radio.SENDERID << "] " << (char*)radio.DATA;
            Serial << buff << F(" SS:") << rssi << endl;  // this passes data to MightyHat / RaspberryPi
        }

        // check if the packet is a wireless programming request
#ifdef ENABLE_WIRELESS_PROGRAMMING
        CheckForWirelessHEX(radio, flash, false);  // non verbose DEBUG
#endif

        // respond to any ACK if requested
        if (radio.ACKRequested()) {
            REQUEST* aux = queue;
            Pbuff = "";
            // walk queue and add pending commands to ACK payload (as many it can fit)
            while (aux != NULL) {
                if (aux->nodeId == radio.SENDERID) {
                    // check if payload has room to add this queued command
                    if (Pbuff.length() + 1 + strlen(aux->data) <= MAX_ACK_REQUEST_LENGTH) {
                        if (Pbuff.length()) Pbuff.print(' ');  // prefix with a space any previous command in buffer
                        Pbuff.print(aux->data);                // append command
                    }
                }
                aux = aux->next;
            }
            if (Pbuff.length())
                radio.sendACK(buff, Pbuff.length());
            else
                radio.sendACK();
        }
        LED_LOW;
        newPacketReceived = true;
    }
}
void loop() {
    handlePowerControl();  // checks any button presses and takes action
    // handle2Buttons();     //checks the general purpose buttons next to the LCD (R2+)
    handleSerialData();  // checks for any serial input from the Pi computer
    handleRadio();
    readBattery();
}

boolean insert(uint16_t new_id, char new_data[]) {
    REQUEST* aux;
    REQUEST* new_node = (REQUEST*)malloc(sizeof(REQUEST));
    if (new_node == NULL) return false;
    new_node->nodeId = new_id;
    strcpy(new_node->data, new_data);
    new_node->next = NULL;
    if (queue == NULL)
        queue = new_node;
    else {
        aux = queue;
        while (aux->next != NULL) aux = aux->next;
        aux->next = new_node;
    }
    return true;
}

// processCommand - parse the command and send it to target
// if target is non-responsive it(sleeppy node?) then queue command to send when target wakes and asks for an ACK
// SPECIAL COMMANDS FROM HOST:
//  - RQ:123:MESSAGE - send or (upon fail) queue message
//  - 123:VOID - removes all queued commands for node 123
//  - 123:VOID:command - removes 'command' from queue (if found)
//  - RQ - prints the queued list of nodes on serial port, to host (Pi?)
//  - RQ:VOID - flush entire queue
//  - FREERAM - returns # of unallocated bytes at end of heap
//  - SYSFREQ - returns operating frequency in Hz
//  - UPTIME - returns millis()
void processCommand(char data[], boolean allowDuplicate = false) {
    char* ptr;
    char dataPart[MAX_BUFFER_LENGTH];
    uint16_t targetId;

    byte isQueueRequest = false;
    ptr = strtok(data, ":");

    if (strcmp(data, "FREERAM") == 0)
        Serial << F("FREERAM:") << freeRAM() << ':' << RAMSIZE << endl;
    if (strcmp(data, "RQ") == 0) {
        ptr = strtok(NULL, ":");  // move to next :
        if (ptr == NULL)
            printQueue(queue);
        else
            isQueueRequest = true;
    }
    if (strcmp(data, "SYSFREQ") == 0)
        PRINT_FREQUENCY;
    if (strcmp(data, "UPTIME") == 0)
        PRINT_UPTIME;
    if (strcmp(data, "NETWORKID") == 0)
        Serial << F("NETWORKID:") << NETWORKID << endl;
    if (strcmp(data, "ENCRYPTKEY") == 0)
#ifdef ENCRYPTKEY
        Serial << F("ENCRYPTKEY:") << ENCRYPTKEY << endl;
#else
        Serial << F("ENCRYPTKEY:NONE") << endl;
#endif

    if (ptr != NULL) {  // delimiter found, valid command
        sprintf(dataPart, "%s", ptr);

        // if "RQ:VOID" then flush entire requst queue
        if (isQueueRequest && strcmp(dataPart, "VOID") == 0) {
            REQUEST* aux = queue;
            byte removed = 0;

            while (aux != NULL) {
                if (aux == queue) {
                    if (aux->next == NULL) {
                        free(queue);
                        queue = NULL;
                        removed++;
                        break;
                    } else {
                        queue = queue->next;
                        free(aux);
                        removed++;
                        aux = queue;
                        continue;
                    }
                }
            }
            DEBUG("DEBUG:VOIDED_commands:");
            DEBUGln(removed);
            size_of_queue = size_of_queue - removed;
            return;
        }

        targetId = atoi(dataPart);  // attempt to extract nodeID part
        ptr = strtok(NULL, "");     // get command part to the end of the string
        sprintf(dataPart, "%s", ptr);

        // check for empty commandprocessCommand
        if (strlen(dataPart) == 0) return;

        // check target nodeID is valid
        if (targetId > 0 && targetId != NODEID && targetId <= 1023) {
            REQUEST* aux;
            byte removed = 0;

            // check if VOID command - if YES then remove command(s) to that target nodeID
            if (strstr(dataPart, "VOID") == dataPart)  // string starts with VOID
            {
                // if 'nodeId:VOID' then remove all commands to that node
                // if 'nodeId:VOID:REQUEST' then remove just 'REQUEST' (if found & identical match)
                boolean removeAll = true;
                if (dataPart[4] == ':' && strlen(dataPart) > 5)
                    removeAll = false;

                // iterate over queue
                aux = queue;
                while (aux != NULL) {
                    if (aux->nodeId == targetId) {
                        if (removeAll || (!removeAll && strcmp(aux->data, dataPart + 5) == 0)) {
                            if (aux == queue) {
                                if (aux->next == NULL) {
                                    free(queue);
                                    queue = NULL;
                                    removed++;
                                    break;
                                } else {
                                    queue = queue->next;
                                    free(aux);
                                    removed++;
                                    aux = queue;
                                    continue;
                                }
                            } else {
                                REQUEST* prev = queue;
                                while (prev->next != NULL && prev->next != aux) prev = prev->next;  // find previous
                                if (prev->next == NULL) break;
                                prev->next = prev->next->next;
                                free(aux);
                                removed++;
                                aux = prev->next;
                            }
                        } else
                            aux = aux->next;
                    } else
                        aux = aux->next;
                }
                DEBUG("DEBUG:VOIDED_commands:");
                DEBUGln(removed);
                size_of_queue = size_of_queue - removed;
                return;
            }

            // try sending to node, if it fails, continue & add to pending commands queue
            LED_HIGH;
            if (radio.sendWithRetry(targetId, dataPart, strlen(dataPart))) {
                LED_LOW;
                return;
            }
            LED_LOW;

            if (!isQueueRequest) return;  // just return at this time if not queued request

            // check for duplicate
            if (!allowDuplicate) {
                // walk queue and check for duplicates
                aux = queue;
                while (aux != NULL) {
                    // DEBUGln("While");
                    if (aux->nodeId == targetId) {
                        if (strcmp(aux->data, dataPart) == 0) {
                            DEBUGln(F("DEBUG:processCommand_skip_duplicate"));
                            return;
                        }
                    }
                    aux = aux->next;
                }
            }

            // all checks OK, attempt to add to queue
            if (insert(targetId, dataPart)) {
                // DEBUG(F("-> inserted: "));
                // DEBUG(targetId);
                // DEBUG("_");
                // DEBUGln(dataPart);
                size_of_queue++;
            } else {
                DEBUGln(F("DEBUG:INSERT_FAIL:MEM_FULL"));
                Serial << F("[") << targetId << F("] ") << dataPart << F(":MEMFULL") << endl;
            }
        } else {
            // DEBUG(F("DEBUG:INSERT_FAIL - INVALID nodeId:")); DEBUGln(targetId);
            Serial << '[' << targetId << "] " << dataPart << F(":INV:ID-OUT-OF-RANGE") << endl;
        }
    }
}

void printQueue(REQUEST* p) {
    if (!size_of_queue) {
        Serial << F("RQ:EMPTY") << endl;
        return;
    }

    REQUEST* aux = p;
    while (aux != NULL) {
        Serial << F("RQ:") << aux->nodeId << ':' << aux->data << endl;
        aux = aux->next;
    }
}

// here's the processing of single char/bytes as soon as they're coming from UART
void handleSerialData() {
    static char input_line[100];  // static = these get allocated ONCE!
    static byte input_pos = 0;
    if (Serial.available() > 0) {
        char inByte = Serial.read();
        switch (inByte) {
            case '\r':  // ignore carriage return
                break;

            case '\n':
                if (input_pos == 0) break;  // ignore empty lines
                input_line[input_pos] = 0;  // null terminate the string
                DEBUG("DEBUG:handleSerialData:");
                DEBUGln(input_line);
                processCommand(input_line);  // fill up queue
                input_pos = 0;               // reset buffer for next time
                break;

            default:
                // keep adding if not full ... allow for terminating byte
                if (input_pos < MAX_BUFFER_LENGTH - 1) {
                    input_line[input_pos] = inByte;
                    input_pos++;
                } else {
                    // if theres no EOL coming before MAX_BUFF_CHARS is exceeded we'll just terminate and send it, last char is then lost
                    input_line[input_pos] = 0;  // null terminate the string
                    DEBUG("DEBUG:MAX_BUFF_CHARS is exceeded - attempting to add (default): ");
                    DEBUGln(input_line);
                    processCommand(input_line);  // add to queue
                    input_pos = 0;               // reset buffer for next line
                }
                break;
        }
    }
}

// returns # of unfragmented free RAM bytes (free end of heap)
int freeRAM() {
#ifdef __arm__
    char top;
    return &top - reinterpret_cast<char*>(sbrk(0));
#else
    extern int __heap_start, *__brkval;
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
#endif
}

// returns total # of free RAM bytes (all free heap, including fragmented memory)
int allFreeRAM() {
    int size = 1024;
    byte* buf;
    while ((buf = (byte*)malloc(--size)) == NULL)
        ;
    free(buf);
    return size;
}
