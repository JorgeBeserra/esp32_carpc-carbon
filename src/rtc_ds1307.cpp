#include "rtc_ds1307.h"

static uint8_t bcd2bin(uint8_t val) {
    return val - 6 * (val >> 4);
}

static uint8_t bin2bcd(uint8_t val) {
    return val + 6 * (val / 10);
}

// O RTC e lido pela tarefa do ADC e pela tarefa principal (Logger). Cada acesso sao duas
// transacoes I2C, entao precisam ser serializadas para nao misturar os dados.
static SemaphoreHandle_t rtcMutex = nullptr;

static bool rtcLock() {
    if (!rtcMutex) return true; // begin() ainda nao foi chamado (somente uma tarefa rodando)
    return xSemaphoreTake(rtcMutex, pdMS_TO_TICKS(200)) == pdTRUE;
}

static void rtcUnlock() {
    if (rtcMutex) xSemaphoreGive(rtcMutex);
}

bool RTC_DS1307::begin() {
    if (!rtcMutex) rtcMutex = xSemaphoreCreateMutex();
    Wire.begin();
    return true;
}

bool RTC_DS1307::read(DateTime &dt) {
    if (!rtcLock()) return false;
    Wire.beginTransmission(0x68);
    Wire.write(0);
    bool ok = (Wire.endTransmission() == 0) && (Wire.requestFrom(0x68, 7) == 7);
    if (!ok) {
        rtcUnlock();
        return false;
    }
    dt.second = bcd2bin(Wire.read() & 0x7F);
    dt.minute = bcd2bin(Wire.read());
    dt.hour = bcd2bin(Wire.read() & 0x3F);
    Wire.read(); // skip day of week
    dt.day = bcd2bin(Wire.read());
    dt.month = bcd2bin(Wire.read());
    dt.year = bcd2bin(Wire.read()) + 2000;
    rtcUnlock();
    return true;
}

bool RTC_DS1307::write(const DateTime &dt) {
    //valores fora da faixa gerariam BCD invalido e corromperiam o relogio
    if (dt.year < 2000 || dt.year > 2099 || dt.month < 1 || dt.month > 12 ||
        dt.day < 1 || dt.day > 31 || dt.hour > 23 || dt.minute > 59 || dt.second > 59) return false;
    if (!rtcLock()) return false;
    Wire.beginTransmission(0x68);
    Wire.write(0);
    Wire.write(bin2bcd(dt.second));
    Wire.write(bin2bcd(dt.minute));
    Wire.write(bin2bcd(dt.hour));
    Wire.write(1); // day of week dummy
    Wire.write(bin2bcd(dt.day));
    Wire.write(bin2bcd(dt.month));
    Wire.write(bin2bcd(dt.year - 2000));
    bool ok = Wire.endTransmission() == 0;
    rtcUnlock();
    return ok;
}

String RTC_DS1307::format(const DateTime &dt) {
    char buf[40];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d",
            dt.year, dt.month, dt.day,
            dt.hour, dt.minute, dt.second);
    return String(buf);
}

String RTC_DS1307::nowString() {
    DateTime dt;
    if (read(dt)) {
        return format(dt);
    }
    return String("0000-00-00T00:00:00");
}

RTC_DS1307 rtc;
