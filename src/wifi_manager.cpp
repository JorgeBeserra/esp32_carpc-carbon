#include "config.h"
#include "wifi_manager.h"
#include "gvret_comm.h"
#include "SerialConsole.h"
#include <WiFi.h>
#include "ELM327_Emulator.h"

WiFiManager::WiFiManager()
{
    lastBroadcast = 0;
}

void WiFiManager::setup()
{
    // Versao de uso no carro: Wi-Fi desligado por padrao para economizar bateria
    // e evitar que o ESP32 fique tentando conectar/criar AP durante o funcionamento.
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);

    SysSettings.isWifiConnected = false;
    SysSettings.isWifiActive = false;

    Serial.println("WiFi disabled for car build");
}

void WiFiManager::setupAPMode()
{
    // AP bloqueado nesta versao. Quando precisar de atualizacao/configuracao via Wi-Fi,
    // reativar em uma branch separada ou criar um gatilho fisico/serial especifico.
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);

    SysSettings.isWifiConnected = false;
    SysSettings.isWifiActive = false;

    Serial.println("WiFi AP mode disabled for car build");
}

void WiFiManager::loop()
{
    // Intencionalmente vazio: Wi-Fi permanece desligado.
}

void WiFiManager::sendBufferedData()
{
    // Sem clientes Wi-Fi ativos. Limpa o buffer para evitar acumulo caso algo escreva nele.
    wifiGVRET.clearBufferedBytes();
}
