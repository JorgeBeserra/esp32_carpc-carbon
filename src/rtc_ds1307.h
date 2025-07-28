#ifndef RTC_DS1307_H
#define RTC_DS1307_H

#include <Arduino.h>
#include <Wire.h>

struct DateTime {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

class RTC_DS1307 {
public:
    bool begin();
    bool read(DateTime &dt);
    bool write(const DateTime &dt);
    String format(const DateTime &dt);
    String nowString();
};

extern RTC_DS1307 rtc;

#endif // RTC_DS1307_H
