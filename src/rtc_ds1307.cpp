#include "rtc_ds1307.h"

static uint8_t bcd2bin(uint8_t val) {
    return val - 6 * (val >> 4);
}

static uint8_t bin2bcd(uint8_t val) {
    return val + 6 * (val / 10);
}

bool RTC_DS1307::begin() {
    Wire.begin();
    return true;
}

bool RTC_DS1307::read(DateTime &dt) {
    Wire.beginTransmission(0x68);
    Wire.write(0);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(0x68, 7) != 7) return false;
    dt.second = bcd2bin(Wire.read() & 0x7F);
    dt.minute = bcd2bin(Wire.read());
    dt.hour = bcd2bin(Wire.read() & 0x3F);
    Wire.read(); // skip day of week
    dt.day = bcd2bin(Wire.read());
    dt.month = bcd2bin(Wire.read());
    dt.year = bcd2bin(Wire.read()) + 2000;
    return true;
}

bool RTC_DS1307::write(const DateTime &dt) {
    Wire.beginTransmission(0x68);
    Wire.write(0);
    Wire.write(bin2bcd(dt.second));
    Wire.write(bin2bcd(dt.minute));
    Wire.write(bin2bcd(dt.hour));
    Wire.write(1); // day of week dummy
    Wire.write(bin2bcd(dt.day));
    Wire.write(bin2bcd(dt.month));
    Wire.write(bin2bcd(dt.year - 2000));
    return Wire.endTransmission() == 0;
}

String RTC_DS1307::format(const DateTime &dt) {
    char buf[20];
    sprintf(buf, "%04d-%02d-%02dT%02d:%02d:%02d",
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
