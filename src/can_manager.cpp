#include <Arduino.h>
#include "can_manager.h"
#include "esp32_can.h"
#include "config.h"
#include "SerialConsole.h"
#include "gvret_comm.h"
#include "lawicel.h"
#include "ELM327_Emulator.h"

unsigned long lastCANActivity = 0;
bool warningSent = false;
bool shutdownCanceled = false;
bool sleeping = false;

void CANManager::wakeUp()
{
    Serial.println("ESP32 Acordou! Atividade detectada na rede CAN.");
    sleeping = false;
}

CANManager::CANManager()
{

}

void CANManager::setup()
{
    for (int i = 0; i < SysSettings.numBuses; i++)
    {
        if (settings.canSettings[i].enabled)
        {
            canBuses[i]->enable();
            if ((settings.canSettings[i].fdMode == 0) || !canBuses[i]->supportsFDMode())
            {
                canBuses[i]->begin(settings.canSettings[i].nomSpeed, 255);
                Serial.printf("CAN%u Speed: %u\n", i, settings.canSettings[i].nomSpeed);
                if ( (i == 0) && (settings.systemType == 2) )
                {
                  digitalWrite(SW_EN, HIGH); //MUST be HIGH to use CAN0 channel
                  Serial.println("Enabling SWCAN Mode");
                }
                if ( (i == 1) && (settings.systemType == 2) )
                {
                  digitalWrite(SW_EN, LOW); //MUST be LOW to use CAN1 channel
                  Serial.println("Enabling CAN1 will force CAN0 off.");
                }
            }
            else
            {
                canBuses[i]->beginFD(settings.canSettings[i].nomSpeed, settings.canSettings[i].fdSpeed);
                Serial.printf("Enabled CAN1 In FD Mode With Nominal Speed %u and Data Speed %u", 
                                settings.canSettings[i].nomSpeed, settings.canSettings[i].fdSpeed);
            }

            if (settings.canSettings[i].listenOnly) 
            {
                canBuses[i]->setListenOnlyMode(true);
            }
            else
            {
                canBuses[i]->setListenOnlyMode(false);
            }
            canBuses[i]->watchFor();
        } 
        else
        {
            canBuses[i]->disable();
        }

        lastCANActivity = millis();
    }

    if (settings.systemType == 2) //Macchina 5-CAN Board
    {
        uint8_t stdbymode;
        //need to set all MCP2517FD modules to use GPIO0 as XSTBY to control transceivers
        for (int i = 1; i < 5; i++)
        {
            MCP2517FD *can = (MCP2517FD *)canBuses[i];
            stdbymode = can->Read8(0xE04);
            stdbymode |= 0x40; // Set bit 6 to enable XSTBY mode
            can->Write8(0xE04, stdbymode);
            stdbymode = can->Read8(0xE04);
            stdbymode &= 0xFE; // clear low bit so GPIO0 is output
            can->Write8(0xE04, stdbymode);
        }
    }

    for (int j = 0; j < NUM_BUSES; j++)
    {
        busLoad[j].bitsPerQuarter = settings.canSettings[j].nomSpeed / 4;
        busLoad[j].bitsSoFar = 0;
        busLoad[j].busloadPercentage = 0;
        if (busLoad[j].bitsPerQuarter == 0) busLoad[j].bitsPerQuarter = 125000;
    }

    busLoadTimer = millis();

    pinMode(CAN_INT_PIN, INPUT_PULLUP);
    attachInterrupt(CAN_INT_PIN, wakeUp, RISING);
}

void CANManager::addBits(int offset, CAN_FRAME &frame)
{
    if (offset < 0) return;
    if (offset >= NUM_BUSES) return;
    busLoad[offset].bitsSoFar += 41 + (frame.length * 9);
    if (frame.extended) busLoad[offset].bitsSoFar += 18;
}

void CANManager::addBits(int offset, CAN_FRAME_FD &frame)
{
    if (offset < 0) return;
    if (offset >= NUM_BUSES) return;
    busLoad[offset].bitsSoFar += 41 + (frame.length * 9);
    if (frame.extended) busLoad[offset].bitsSoFar += 18;
}

void CANManager::sendFrame(CAN_COMMON *bus, CAN_FRAME &frame)
{
    int whichBus = 0;
    for (int i = 0; i < NUM_BUSES; i++) if (canBuses[i] == bus) whichBus = i;
    bus->sendFrame(frame);
    addBits(whichBus, frame);
}

void CANManager::sendFrame(CAN_COMMON *bus, CAN_FRAME_FD &frame)
{
    int whichBus = 0;
    for (int i = 0; i < NUM_BUSES; i++) if (canBuses[i] == bus) whichBus = i;
    bus->sendFrameFD(frame);
    addBits(whichBus, frame);
}


void CANManager::displayFrame(CAN_FRAME &frame, int whichBus)
{
    if (settings.enableLawicel && SysSettings.lawicelMode) 
    {
        lawicel.sendFrameToBuffer(frame, whichBus);
    } 
    else 
    {
        if (SysSettings.isWifiActive) wifiGVRET.sendFrameToBuffer(frame, whichBus);
        else serialGVRET.sendFrameToBuffer(frame, whichBus);
    }
}

void CANManager::displayFrame(CAN_FRAME_FD &frame, int whichBus)
{
    if (settings.enableLawicel && SysSettings.lawicelMode) 
    {
        //lawicel.sendFrameToBuffer(frame, whichBus);
    } 
    else 
    {
        if (SysSettings.isWifiActive) wifiGVRET.sendFrameToBuffer(frame, whichBus);
        else serialGVRET.sendFrameToBuffer(frame, whichBus);
    }
}

void CANManager::loop()
{
    if (sleeping) return; // Se estiver dormindo, nada acontece até ser acordado

    CAN_FRAME incoming;
    CAN_FRAME_FD inFD;
    size_t wifiLength = wifiGVRET.numAvailableBytes();
    size_t serialLength = serialGVRET.numAvailableBytes();
    size_t maxLength = (wifiLength > serialLength) ? wifiLength : serialLength;

    bool canActive = false;  // Variável para detectar atividade CAN

    if (millis() > (busLoadTimer + 250)) {
        busLoadTimer = millis();
        busLoad[0].busloadPercentage = ((busLoad[0].busloadPercentage * 3) + (((busLoad[0].bitsSoFar * 1000) / busLoad[0].bitsPerQuarter) / 10)) / 4;
        //Force busload percentage to be at least 1% if any traffic exists at all. This forces the LED to light up for any traffic.
        if (busLoad[0].busloadPercentage == 0 && busLoad[0].bitsSoFar > 0) busLoad[0].busloadPercentage = 1;
        busLoad[0].bitsPerQuarter = settings.canSettings[0].nomSpeed / 4;
        busLoad[0].bitsSoFar = 0;
        if(busLoad[0].busloadPercentage > busLoad[1].busloadPercentage){
            //updateBusloadLED(busLoad[0].busloadPercentage);
        } else{
            //updateBusloadLED(busLoad[1].busloadPercentage);
        }
    }

    for (int i = 0; i < SysSettings.numBuses; i++)
    {
        if (!canBuses[i]) continue;
        if (!settings.canSettings[i].enabled) continue;
        while ( (canBuses[i]->available() > 0) && (maxLength < (WIFI_BUFF_SIZE - 80)))
        {
            if (settings.canSettings[i].fdMode == 0)
            {
                canBuses[i]->read(incoming);
                addBits(i, incoming);
                displayFrame(incoming, i);
            }
            else
            {
                canBuses[i]->readFD(inFD);
                addBits(i, inFD);
                displayFrame(inFD, i);
            }
            
            if ( (incoming.id > 0x7DF && incoming.id < 0x7F0) || elmEmulator.getMonitorMode() ) elmEmulator.processCANReply(incoming);
            wifiLength = wifiGVRET.numAvailableBytes();
            serialLength = serialGVRET.numAvailableBytes();
            maxLength = (wifiLength > serialLength) ? wifiLength:serialLength;

            canActive = true;  // Se há mensagens, a rede está ativa
        }
    }

    // Atualiza o estado do MOSFET com base na atividade CAN
    if (canActive) {
        lastCANActivity = millis();
        warningSent = false; // Reseta o aviso se houver tráfego
        shutdownCanceled = false; // Se houver tráfego CAN, o desligamento é cancelado
        digitalWrite(MOSFET_PIN, HIGH);
    } else {
        unsigned long timeSinceLastActivity = millis() - lastCANActivity;
        if (timeSinceLastActivity >= TIMEOUT_WARNING && !warningSent)
        {
            Logger::console("Iniciando Processo de Desligar");
            warningSent = true; // Evita repetição do aviso
        }

        if (timeSinceLastActivity >= TIMEOUT_SHUTDOWN)
        {
            // Aqui o Raspberry deve interpretar esse comando e desligar
            Logger::console("Desligando");
            digitalWrite(MOSFET_PIN, LOW);  // Desativa o MOSFET

            Serial.println("Entrando em Light Sleep...");
            sleeping = true;
            esp_sleep_enable_ext0_wakeup((gpio_num_t)CAN_INT_PIN, 1); // Acorda com atividade no CAN
            esp_light_sleep_start(); // ESP32 dorme
        }
        
    }
}
