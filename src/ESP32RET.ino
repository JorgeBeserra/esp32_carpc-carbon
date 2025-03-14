/*
 ESP32RET.ino

 Created: June 1, 2020
 Author: Collin Kidder

Copyright (c) 2014-2020 Collin Kidder, Michael Neuweiler

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include "config.h"
#include <esp32_can.h>
#include <SPI.h>
#include <Preferences.h>
#include "ELM327_Emulator.h"
#include "SerialConsole.h"
#include "wifi_manager.h"
#include "gvret_comm.h"
#include "can_manager.h"
#include "lawicel.h"

byte i = 0;

uint32_t lastFlushMicros = 0;

bool markToggle[6];
uint32_t lastMarkTrigger = 0;

EEPROMSettings settings;
SystemSettings SysSettings;
Preferences nvPrefs;
char deviceName[20];
char otaHost[40];
char otaFilename[100];

uint8_t espChipRevision;

ELM327Emu elmEmulator;

WiFiManager wifiManager;

GVRET_Comm_Handler serialGVRET; //gvret protocol over the serial to USB connection
GVRET_Comm_Handler wifiGVRET; //GVRET over the wifi telnet port
CANManager canManager; //keeps track of bus load and abstracts away some details of how things are done
LAWICELHandler lawicel;

SerialConsole console;

CAN_COMMON *canBuses[NUM_BUSES];

//initializes all the system EEPROM values. Chances are this should be broken out a bit but
//there is only one checksum check for all of them so it's simple to do it all here.
void loadSettings()
{
    Logger::console("Loading settings....");

    for (int i = 0; i < NUM_BUSES; i++) canBuses[i] = nullptr;

    nvPrefs.begin(PREF_NAME, false);

    settings.useBinarySerialComm = nvPrefs.getBool("binarycomm", false);
    settings.logLevel = nvPrefs.getUChar("loglevel", 1); //info
    settings.wifiMode = nvPrefs.getUChar("wifiMode", 1); //Wifi defaults to creating an AP
    
    settings.enableBT = nvPrefs.getBool("enable-bt", false);
    settings.enableLawicel = nvPrefs.getBool("enableLawicel", true);
    settings.systemType = 0; //nvPrefs.getUChar("systype", (espChipRevision > 2) ? 0 : 1); //0 = A0, 1 = EVTV ESP32

    if (settings.systemType == 0)
    {
        canBuses[0] = &CAN0;
        SysSettings.LED_CANTX = 26;
        SysSettings.LED_CANRX = 27;
        SysSettings.LED_LOGGING = 35;
        SysSettings.LED_CONNECTION_STATUS = 0;
        SysSettings.fancyLED = false;
        SysSettings.logToggle = false;
        SysSettings.txToggle = true;
        SysSettings.rxToggle = true;
        SysSettings.lawicelAutoPoll = false;
        SysSettings.lawicelMode = false;
        SysSettings.lawicellExtendedMode = false;
        SysSettings.lawicelTimestamping = false;
        SysSettings.numBuses = 1;
        SysSettings.isWifiActive = false;
        SysSettings.isWifiConnected = false;
        strcpy(deviceName, MACC_NAME);
        strcpy(otaHost, "");
        strcpy(otaFilename, "");
        pinMode(SysSettings.LED_CANTX, OUTPUT);
        pinMode(SysSettings.LED_CANRX, OUTPUT);
        digitalWrite(SysSettings.LED_CANTX, LOW);
        digitalWrite(SysSettings.LED_CANRX, LOW);
        delay(100);
        CAN0.setCANPins(GPIO_NUM_4, GPIO_NUM_5);     // use this for shield v1.3 and later
    }

    if (nvPrefs.getString("SSID", settings.SSID, 32) == 0)
    {
        //strcpy(settings.SSID, deviceName);
        strcat(settings.SSID, "Sabidos");
   }

    if (nvPrefs.getString("wpa2Key", settings.WPA2Key, 64) == 0)
    {
        strcpy(settings.WPA2Key, "23Seb10STE5aNT");
    }
    if (nvPrefs.getString("btname", settings.btName, 32) == 0)
    {
        strcpy(settings.btName, "ELM327-");
        strcat(settings.btName, deviceName);
    }

    char buff[80];
    for (int i = 0; i < SysSettings.numBuses; i++)
    {
        sprintf(buff, "can%ispeed", i);
        settings.canSettings[i].nomSpeed = nvPrefs.getUInt(buff, 500000);
        sprintf(buff, "can%i_en", i);
        settings.canSettings[i].enabled = nvPrefs.getBool(buff, (i < 2)?true:false);
        sprintf(buff, "can%i-listenonly", i);
        settings.canSettings[i].listenOnly = nvPrefs.getBool(buff, false);
        sprintf(buff, "can%i-fdspeed", i);
        settings.canSettings[i].fdSpeed = nvPrefs.getUInt(buff, 5000000);
        sprintf(buff, "can%i-fdmode", i);
        settings.canSettings[i].fdMode = nvPrefs.getBool(buff, false);
    }

    nvPrefs.end();

    Logger::setLoglevel((Logger::LogLevel)settings.logLevel);

    for (int rx = 0; rx < NUM_BUSES; rx++) SysSettings.lawicelBusReception[rx] = true; //default to showing messages on RX 
}

void setup()
{
    //delay(5000); //just for testing. Don't use in production

    espChipRevision = ESP.getChipRevision();

    Serial.begin(115200); //for production
    //Serial.begin(115200); //for testing

    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, LOW);  // Inicia com Raspberry desligada

    SysSettings.isWifiConnected = false;

    loadSettings();

    wifiManager.setup();
    
    Serial.println("");
    Serial.println("===================================");
    Serial.println("        MrDIY.ca CAN Shield        ");
    Serial.println("Version: 1.0.0");
    Serial.println("===================================");
    Serial.println("");
    Serial.print("Build number: ");
    Serial.println(CFG_BUILD_NUM);

    canManager.setup();
    SysSettings.lawicelMode = false;
    SysSettings.lawicelAutoPoll = false;
    SysSettings.lawicelTimestamping = false;
    SysSettings.lawicelPollCounter = 0;
    
    //elmEmulator.setup(); -> Limpar isso do código

    // Cria tarefa para ADC
    xTaskCreatePinnedToCore(
        adc_task,         // Função da tarefa
        "ADC Task",       // Nome
        2048,             // Tamanho da pilha
        NULL,             // Parâmetros
        1,                // Prioridade
        NULL,             // Handle
        1                 // Núcleo 1
    );
}
 
/*
Send a fake frame out USB and maybe to file to show where the mark was triggered at. The fake frame has bits 31 through 3
set which can never happen in reality since frames are either 11 or 29 bit IDs. So, this is a sign that it is a mark frame
and not a real frame. The bottom three bits specify which mark triggered.
*/
void sendMarkTriggered(int which)
{
    CAN_FRAME frame;
    frame.id = 0xFFFFFFF8ull + which;
    frame.extended = true;
    frame.length = 0;
    frame.rtr = 0;
    canManager.displayFrame(frame, 0);
}

/*
Loop executes as often as possible all the while interrupts fire in the background.
The serial comm protocol is as follows:
All commands start with 0xF1 this helps to synchronize if there were comm issues
Then the next byte specifies which command this is.
Then the command data bytes which are specific to the command
Lastly, there is a checksum byte just to be sure there are no missed or duped bytes
Any bytes between checksum and 0xF1 are thrown away

Yes, this should probably have been done more neatly but this way is likely to be the
fastest and safest with limited function calls
*/

// volume +

// ADC Click: 3890
// ADC Click: 3891
// ADC Click: 3893
// ADC Click: 3890
// ADC Click: 3892
// ADC Click: 3896
// ADC Click: 3892
// ADC Click: 3892
// ADC Click: 3897
// ADC Click: 3892
// ADC Click: 3890
// ADC Click: 3892

// volume -

// ADC Click: 4048
// ADC Click: 4047
// ADC Click: 4047
// ADC Click: 4047
// ADC Click: 4048
// ADC Click: 4047
// ADC Click: 4047
// ADC Click: 4050
// ADC Click: 4048
// ADC Click: 4040
// ADC Click: 4049
// ADC Click: 4047
// ADC Click: 4048
// ADC Click: 4047


// Proxima Musica

// ADC Click: 3668
// ADC Click: 3664
// ADC Click: 3670
// ADC Click: 3665
// ADC Click: 3668
// ADC Click: 3665
// ADC Click: 3666
// ADC Click: 3667
// ADC Click: 3667
// ADC Click: 3665
// ADC Click: 3665
// ADC Click: 3664
// ADC Click: 3666
// ADC Click: 3666
// ADC Click: 3663


// musica anterior

// ADC Click: 3358
// ADC Click: 3358
// ADC Click: 3358
// ADC Click: 3358
// ADC Click: 3343
// ADC Click: 3364
// ADC Click: 3356
// ADC Click: 3358
// ADC Click: 3357
// ADC Click: 3358
// ADC Click: 3355
// ADC Click: 3357
// ADC Click: 3355
// ADC Click: 3359
// ADC Click: 3358
// ADC Click: 3357
// ADC Click: 3358
// ADC Click: 3359
// ADC Click: 3356
// ADC Click: 3359
// ADC Click: 3356
// ADC Click: 3359
// ADC Click: 3360
// ADC Click: 3358
// ADC Click: 3359
// ADC Click: 3357
// ADC Click: 3354
// ADC Click: 3359
// ADC Click: 3353
// ADC Click: 3353
// ADC Click: 3356
// ADC Click: 3355
// ADC Click: 3356
// ADC Click: 3354
// ADC Click: 3357
// ADC Click: 3354


// muta

// ADC Click: 4095
// ADC Click: 4095
// ADC Click: 4095
// ADC Click: 4095
// ADC Click: 4095
// ADC Click: 4095
// ADC Click: 4095
// ADC Click: 4095
// ADC Click: 4095
// ADC Click: 4095
// ADC Click: 4095
// ADC Click: 4095
// ADC Click: 4095
// ADC Click: 4095
// ADC Click: 4095



void adc_task(void *pvParameters) {
    static int ultimoValorADC = -1;       // Valor inicial improvável
    static uint32_t lastResistanceCheck = 0;
    static uint32_t pressStartTime = 0;   // Marca o início da pressão
    static bool isPressed = false;        // Estado do botão
    static uint32_t lastHoldSent = 0;     // Última vez que "Hold" foi enviado
    const uint32_t HOLD_INTERVAL = 500000; // Intervalo de 500ms para "Hold"

    while (1) {
        if (micros() - lastResistanceCheck > 100000) { // 100ms para maior responsividade
            lastResistanceCheck = micros();
            int valorADC = getAnalog(2); // GPIO 34 (ADC1_CHANNEL_2)
            
            // char buffer[32];
            // snprintf(buffer, sizeof(buffer), "ADC Bruto: %d\n", valorADC);
            // Serial.write(buffer);

            if (valorADC >= 2380 && valorADC <= 2450) { // Neutro (ajustado com base nos logs)
                if (isPressed) { // Botão foi solto
                    uint32_t pressDuration = micros() - pressStartTime;
                    if (pressDuration <= 750000 && ultimoValorADC > 2450) { // Toque rápido
                        char buffer[32];
                        snprintf(buffer, sizeof(buffer), "ADC Click: %d\n", ultimoValorADC);
                        Serial.write(buffer);
                    }
                    isPressed = false;
                    ultimoValorADC = valorADC; // Atualiza para o valor neutro
                }
            } else if (valorADC > 2450) { // Só processa valores > 2450 (botão pressionado)
                if (!isPressed) { // Botão recém-pressionado
                    pressStartTime = micros();
                    isPressed = true;
                    lastHoldSent = 0; // Reseta o temporizador de "Hold"

                } else { // Botão já está pressionado
                    uint32_t pressDuration = micros() - pressStartTime;
                    if (pressDuration > 750000) { // 500ms = pressão mantida
                        if (micros() - lastHoldSent >= HOLD_INTERVAL) { // Envia a cada 500ms
                            char buffer[32];
                            snprintf(buffer, sizeof(buffer), "ADC Hold: %d\n", valorADC);
                            Serial.write(buffer);
                            lastHoldSent = micros();
                            ultimoValorADC = valorADC;
                        }
                    }
                }

                if (isPressed) { // Botão foi solto
                    uint32_t pressDuration = micros() - pressStartTime;
                    if (pressDuration <= 750000 && valorADC > 2450) { // Toque rápido
                        char buffer[32];
                        snprintf(buffer, sizeof(buffer), "ADC Click: %d\n", valorADC);
                        Serial.write(buffer);
                    }
                    isPressed = false;
                    ultimoValorADC = valorADC; // Atualiza para o valor neutro
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Cede controle ao FreeRTOS
    }
}

void loop()
{
    //uint32_t temp32;    
    bool isConnected = false;
    int serialCnt;
    uint8_t in_byte;

    /*if (Serial)*/ isConnected = true;

    if (SysSettings.lawicelPollCounter > 0) SysSettings.lawicelPollCounter--;
    //}

    canManager.loop();
    /*if (!settings.enableBT)*/ 
    /* wifiManager.loop(); */

        // Valor ADC
        // if (micros() - lastResistanceCheck > 500000) { // Medir a cada 1 segundo
        //     lastResistanceCheck = micros();
            
        //     int valorADC = getAnalog(2); // Lê GPIO 34
        //     if (valorADC != ultimoValorADC) { // Só faz log se o valor mudou
        //         Logger::info("Valor ADC bruto 1: %i", valorADC); // Depuração do ADC
        //         ultimoValorADC = valorADC; // Atualiza o último valor lido
        //     }
        // }

    size_t wifiLength = wifiGVRET.numAvailableBytes();
    size_t serialLength = serialGVRET.numAvailableBytes();
    size_t maxLength = (wifiLength>serialLength) ? wifiLength : serialLength;

    //If the max time has passed or the buffer is almost filled then send buffered data out
    if ((micros() - lastFlushMicros > SER_BUFF_FLUSH_INTERVAL) || (maxLength > (WIFI_BUFF_SIZE - 40)) ) 
    {
        lastFlushMicros = micros();
        if (serialLength > 0) 
        {
            Serial.write(serialGVRET.getBufferedBytes(), serialLength);
            serialGVRET.clearBufferedBytes();
        }
        if (wifiLength > 0)
        {
            wifiManager.sendBufferedData();
        }
    }

    serialCnt = 0;
    while ( (Serial.available() > 0) && serialCnt < 128 ) 
    {
        serialCnt++;
        in_byte = Serial.read();
        serialGVRET.processIncomingByte(in_byte);
    }

    elmEmulator.loop();

    vTaskDelay(pdMS_TO_TICKS(20)); // Cede controle ao FreeRTOS no loop principal
}
