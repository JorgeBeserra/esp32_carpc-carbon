#include <Arduino.h>
#include "can_manager.h"
#include "esp32_can.h"
#include "config.h"
#include "SerialConsole.h"
#include "gvret_comm.h"
#include "lawicel.h"
#include "ELM327_Emulator.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

unsigned long lastCANActivity = 0;
extern unsigned long lastSerialActivity;
extern unsigned long startupTime;
bool warningSent = false;
bool shutdownCanceled = false;
bool sleeping = false;
static bool shutdownSent = false; // "ShuttingDown" ja foi enviado neste ciclo de inatividade

// Escreve sempre no pino: o console (MOSFET=0 / SHUTDOWN) tambem altera o pino diretamente,
// entao um estado em cache ficaria dessincronizado e impediria religar o Raspberry.
static void setMosfet(bool state)
{
    digitalWrite(MOSFET_PIN, state ? HIGH : LOW);
}

// Lista de IDs a filtrar apenas na serial
const uint32_t SERIAL_FILTERED_IDS[] = {0x3B3, 0x3AA, 0x202};
const int NUM_SERIAL_FILTERED_IDS = sizeof(SERIAL_FILTERED_IDS) / sizeof(SERIAL_FILTERED_IDS[0]);

CANManager::CANManager()
{

}

void CANManager::setup()
{
    for (int i = 0; i < SysSettings.numBuses; i++)
    {
        if (!canBuses[i]) continue;

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
            if (!can) continue;
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

    // Pull-up mantem o RX em nivel recessivo (alto) se o transceiver estiver desconectado.
    // Nao usar attachInterrupt aqui: este e o pino RX do CAN e geraria uma interrupcao por bit.
    // O despertar do light sleep e feito via ext0 em CANManager::loop().
    pinMode(CAN_INT_PIN, INPUT_PULLUP);
}

void CANManager::addBits(int offset, CAN_FRAME &frame)
{
    if (offset < 0 || offset >= NUM_BUSES) return;
    busLoad[offset].bitsSoFar += 41 + (frame.length * 9);
    if (frame.extended) busLoad[offset].bitsSoFar += 18;
}

void CANManager::addBits(int offset, CAN_FRAME_FD &frame)
{
    if (offset < 0 || offset >= NUM_BUSES) return;
    busLoad[offset].bitsSoFar += 41 + (frame.length * 9);
    if (frame.extended) busLoad[offset].bitsSoFar += 18;
}

void CANManager::sendFrame(CAN_COMMON *bus, CAN_FRAME &frame)
{
    int whichBus = 0;
    if (!bus) return; // barramento inexistente neste hardware
    for (int i = 0; i < NUM_BUSES; i++) if (canBuses[i] == bus) whichBus = i;
    bus->sendFrame(frame);
    addBits(whichBus, frame);
}

void CANManager::sendFrame(CAN_COMMON *bus, CAN_FRAME_FD &frame)
{
    int whichBus = 0;
    if (!bus) return; // barramento inexistente neste hardware
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
        else 
        {
            // Aplica filtro apenas para serial
            bool shouldSendToSerial = false;
            for (int j = 0; j < NUM_SERIAL_FILTERED_IDS; j++) {
                if (frame.id == SERIAL_FILTERED_IDS[j]) {
                    shouldSendToSerial = true;
                    break;
                }
            }
            if (shouldSendToSerial) {
                serialGVRET.sendFrameToBuffer(frame, whichBus);
            }
        }
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
        else 
        {
            // Aplica filtro apenas para serial
            bool shouldSendToSerial = false;
            for (int j = 0; j < NUM_SERIAL_FILTERED_IDS; j++) {
                if (frame.id == SERIAL_FILTERED_IDS[j]) {
                    shouldSendToSerial = true;
                    break;
                }
            }
            if (shouldSendToSerial) {
                serialGVRET.sendFrameToBuffer(frame, whichBus);
            }
        }
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

    // Calcula carga do barramento a cada 250ms
    if (millis() - busLoadTimer > 250) {
        busLoadTimer = millis();
        if (busLoad[0].bitsPerQuarter == 0) busLoad[0].bitsPerQuarter = 125000; // evita divisao por zero
        busLoad[0].busloadPercentage = ((busLoad[0].busloadPercentage * 3) + (((busLoad[0].bitsSoFar * 1000) / busLoad[0].bitsPerQuarter) / 10)) / 4;
        //Force busload percentage to be at least 1% if any traffic exists at all. This forces the LED to light up for any traffic.
        if (busLoad[0].busloadPercentage == 0 && busLoad[0].bitsSoFar > 0) busLoad[0].busloadPercentage = 1;
        busLoad[0].bitsPerQuarter = settings.canSettings[0].nomSpeed / 4;
        if (busLoad[0].bitsPerQuarter == 0) busLoad[0].bitsPerQuarter = 125000;
        busLoad[0].bitsSoFar = 0;
        if(busLoad[0].busloadPercentage > busLoad[1].busloadPercentage){
            //updateBusloadLED(busLoad[0].busloadPercentage);
        } else{
            //updateBusloadLED(busLoad[1].busloadPercentage);
        }
    }

    // Limita o número de mensagens processadas por iteração
    const int MAX_MESSAGES_PER_LOOP = 64; // Ajustável (tamanho da fila de RX da lib)
    int messagesProcessed = 0;

    for (int i = 0; i < SysSettings.numBuses; i++)
    {
        if (!canBuses[i] || !settings.canSettings[i].enabled) continue;

        while (canBuses[i]->available() > 0 && maxLength < (WIFI_BUFF_SIZE - 80) && messagesProcessed < MAX_MESSAGES_PER_LOOP)
        {
            if (settings.canSettings[i].fdMode == 0)
            {
                canBuses[i]->read(incoming);
                addBits(i, incoming);
                // O ELM precisa do ID original: sendFrameToBuffer pode alterar o frame
                if ( (incoming.id > 0x7DF && incoming.id < 0x7F0) || elmEmulator.getMonitorMode() ) elmEmulator.processCANReply(incoming);
                displayFrame(incoming, i);
            }
            else
            {
                canBuses[i]->readFD(inFD);
                addBits(i, inFD);
                displayFrame(inFD, i);
            }

            wifiLength = wifiGVRET.numAvailableBytes();
            serialLength = serialGVRET.numAvailableBytes();
            maxLength = (wifiLength > serialLength) ? wifiLength:serialLength;

            canActive = true;  // Se há mensagens, a rede está ativa
            messagesProcessed++;
        }

        // Cede controle após processar algumas mensagens
        if (messagesProcessed >= MAX_MESSAGES_PER_LOOP) break;
    }

    // Atualiza o estado do MOSFET com base na atividade CAN ou serial
    if (canActive) {
        lastCANActivity = millis();
        warningSent = false; // Reseta o aviso se houver tráfego
        shutdownCanceled = false; // Se houver tráfego CAN, o desligamento é cancelado
        shutdownSent = false;
        setMosfet(true);
    } else {
        unsigned long lastActivity = lastCANActivity;
        if (lastSerialActivity > lastActivity) lastActivity = lastSerialActivity;
        unsigned long timeSinceLastActivity = millis() - lastActivity;

        if (millis() - startupTime < STARTUP_DELAY) {
            // Mantém ligado durante o período inicial
            setMosfet(true);      
        } else {
            if (timeSinceLastActivity < TIMEOUT_WARNING)
            {
                // Houve atividade serial recente: reinicia o ciclo de desligamento
                warningSent = false;
                shutdownSent = false;
            }
            else if (!warningSent)
            {
                Logger::console("ShutdownForInactivity");
                warningSent = true; // Evita repetição do aviso
            }

            if (timeSinceLastActivity >= TIMEOUT_SHUTDOWN && !shutdownSent)
            {
                // Aqui o Raspberry deve interpretar esse comando e desligar
                Logger::console("ShuttingDown");
                shutdownSent = true;
            }

            // Espera TIMEOUT_SLEEP_GRACE apos o "ShuttingDown" antes de dormir, para que o
            // Raspberry ainda consiga responder pela serial (ex.: SHUTDOWN=1)
            if (timeSinceLastActivity >= (TIMEOUT_SHUTDOWN + TIMEOUT_SLEEP_GRACE))
            {
                //digitalWrite(MOSFET_PIN, LOW);  // Desativa o MOSFET

                Serial.println("Entering sleep mode...");
                Serial.flush(); // garante que a mensagem saia antes de parar os clocks
                sleeping = true;
                esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_UART); // Evita acordar pela serial
                // O RX do CAN fica em nivel alto (recessivo) com o barramento parado e vai a
                // nivel baixo (dominante) quando ha trafego: acorda em nivel 0.
                esp_sleep_enable_ext0_wakeup((gpio_num_t)CAN_INT_PIN, 0);
                esp_light_sleep_start(); // ESP32 dorme
                esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);
                // ext0 deixa o pino no dominio RTC; devolve para o GPIO digital (RX do TWAI)
                rtc_gpio_deinit((gpio_num_t)CAN_INT_PIN);
                sleeping = false; // volta a processar apos acordar
                Serial.println("Woke from sleep");
                lastCANActivity = millis();  // evita novo sleep imediato
                warningSent = false;
                shutdownSent = false;
                setMosfet(true);
            }
        }

    }

    // Cede controle ao FreeRTOS ao final de cada iteração
    vTaskDelay(pdMS_TO_TICKS(1)); // 1ms para garantir estabilidade
}
