/*
 * Logger.cpp
 *
 Copyright (c) 2013 Collin Kidder, Michael Neuweiler, Charles Galpin

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

#include "Logger.h"
#include "config.h"
#include "sys_io.h"
#include "EEPROM.h"
#include "rtc_ds1307.h"

Logger::LogLevel Logger::logLevel = Logger::Info;
uint32_t Logger::lastLogTime = 0;

/*
 * Output a debug message with a variable amount of parameters.
 * printf() style, see Logger::log()
 *
 */
void Logger::debug(const char *message, ...)
{
    if (logLevel > Debug) {
        return;
    }

    va_list args;
    va_start(args, message);
    Logger::log(Debug, message, args);
    va_end(args);
}

/*
 * Output a info message with a variable amount of parameters
 * printf() style, see Logger::log()
 */
void Logger::info(const char *message, ...)
{
    if (logLevel > Info) {
        return;
    }

    va_list args;
    va_start(args, message);
    Logger::log(Info, message, args);
    va_end(args);
}

/*
 * Output a warning message with a variable amount of parameters
 * printf() style, see Logger::log()
 */
void Logger::warn(const char *message, ...)
{
    if (logLevel > Warn) {
        return;
    }

    va_list args;
    va_start(args, message);
    Logger::log(Warn, message, args);
    va_end(args);
}

/*
 * Output a error message with a variable amount of parameters
 * printf() style, see Logger::log()
 */
void Logger::error(const char *message, ...)
{
    if (logLevel > Error) {
        return;
    }

    va_list args;
    va_start(args, message);
    Logger::log(Error, message, args);
    va_end(args);
}

/*
 * Output a console message with a variable amount of parameters
 * printf() style, see Logger::logMessage()
 */
void Logger::console(const char *message, ...)
{
    va_list args;
    va_start(args, message);
    Logger::logMessage(message, args);
    va_end(args);
}

/*
 * Set the log level. Any output below the specified log level will be omitted.
 */
void Logger::setLoglevel(LogLevel level)
{
    logLevel = level;
}

/*
 * Retrieve the current log level.
 */
Logger::LogLevel Logger::getLogLevel()
{
    return logLevel;
}

/*
 * Return a timestamp when the last log entry was made.
 */
uint32_t Logger::getLastLogTime()
{
    return lastLogTime;
}

/*
 * Returns if debug log level is enabled. This can be used in time critical
 * situations to prevent unnecessary string concatenation (if the message won't
 * be logged in the end).
 *
 * Example:
 * if (Logger::isDebug()) {
 *    Logger::debug("current time: %d", millis());
 * }
 */
boolean Logger::isDebug()
{
    return logLevel == Debug;
}

/*
 * Output a log message (called by debug(), info(), warn(), error(), console())
 *
 * Supports printf() like syntax:
 *
 * %% - outputs a '%' character
 * %s - prints the next parameter as string
 * %d - prints the next parameter as decimal
 * %f - prints the next parameter as double float
 * %x - prints the next parameter as hex value
 * %X - prints the next parameter as hex value with '0x' added before
 * %b - prints the next parameter as binary value
 * %B - prints the next parameter as binary value with '0b' added before
 * %l - prints the next parameter as long
 * %c - prints the next parameter as a character
 * %t - prints the next parameter as boolean ('T' or 'F')
 * %T - prints the next parameter as boolean ('true' or 'false')
 */
void Logger::log(LogLevel level, const char *format, va_list args)
{
    lastLogTime = millis();
    Serial.print(rtc.nowString());
    Serial.print(" - ");

    switch (level) {
    case Debug:
        Serial.print("DEBUG");
        break;

    case Info:
        Serial.print("INFO");
        break;

    case Warn:
        Serial.print("WARNING");
        break;

    case Error:
        Serial.print("ERROR");
        break;
    }

    Serial.print(": ");

    logMessage(format, args);
}

/*
 * Output a log message (called by log(), console())
 *
 * Supports printf() like syntax:
 *
 * %% - outputs a '%' character
 * %s - prints the next parameter as string
 * %d - prints the next parameter as decimal
 * %f - prints the next parameter as double float
 * %x - prints the next parameter as hex value
 * %X - prints the next parameter as hex value with '0x' added before
 * %l - prints the next parameter as long
 * %c - prints the next parameter as a character
 * %t - prints the next parameter as boolean ('T' or 'F')
 * %T - prints the next parameter as boolean ('true' or 'false')
 */
void Logger::logMessage(const char *format, va_list args)
{
    //two bytes are always kept free for the trailing CR/LF
    const size_t LIMIT = 254;
    char buffer[LIMIT + 2];
    size_t buffLen = 0;

    for (; *format != 0 && buffLen < LIMIT; ++format) {
        if (*format != '%') {
            buffer[buffLen++] = *format;
            continue;
        }

        ++format;
        if (*format == '\0') {
            break;
        }

        //snprintf never writes more than "room" bytes (terminator included) but returns the
        //length it wanted to write, so the result has to be clamped afterward.
        size_t room = LIMIT - buffLen + 1;
        int written = 0;
        char *dest = &buffer[buffLen];

        switch (*format) {
        case '%':
            buffer[buffLen++] = '%';
            continue;
        case 's': {
            const char *str = va_arg(args, const char *);
            written = snprintf(dest, room, "%s", str ? str : "(null)");
            break;
        }
        case 'd':
        case 'i':
            written = snprintf(dest, room, "%i", va_arg(args, int));
            break;
        case 'u':
            written = snprintf(dest, room, "%u", va_arg(args, unsigned int));
            break;
        case 'f':
            written = snprintf(dest, room, "%.2f", va_arg(args, double));
            break;
        case 'x':
            written = snprintf(dest, room, "%X", va_arg(args, unsigned int));
            break;
        case 'X':
            written = snprintf(dest, room, "0x%X", va_arg(args, unsigned int));
            break;
        case 'l':
            written = snprintf(dest, room, "%li", va_arg(args, long));
            break;
        case 'c':
            written = snprintf(dest, room, "%c", va_arg(args, int));
            break;
        case 't':
            written = snprintf(dest, room, "%s", (va_arg(args, int) == 1) ? "T" : "F");
            break;
        case 'T':
            written = snprintf(dest, room, "%s", (va_arg(args, int) == 1) ? "TRUE" : "FALSE");
            break;
        default: //unknown specifier - skip it like the original implementation did
            continue;
        }

        if (written < 0) written = 0;
        if ((size_t)written >= room) written = room - 1; //output was truncated
        buffLen += written;
    }

    buffer[buffLen++] = '\r';
    buffer[buffLen++] = '\n';
    Serial.write((const uint8_t *)buffer, buffLen);
    //If wifi has connected nodes then send to them too.
    for(int i = 0; i < MAX_CLIENTS; i++){
        if (SysSettings.clientNodes[i] && SysSettings.clientNodes[i].connected()){
            SysSettings.clientNodes[i].write((const uint8_t *)buffer, buffLen);
        }
    }
}
