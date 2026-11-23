#include "commbuffer.h"
#include "Logger.h"
#include "gvret_comm.h"

CommBuffer::CommBuffer()
{
    transmitBufferLength = 0;
}

size_t CommBuffer::numAvailableBytes()
{
    return transmitBufferLength;
}

void CommBuffer::clearBufferedBytes()
{
    transmitBufferLength = 0;
}

uint8_t* CommBuffer::getBufferedBytes()
{
    return transmitBuffer;
}

//a bit faster version that blasts through the copy more efficiently.
void CommBuffer::sendBytesToBuffer(uint8_t *bytes, size_t length)
{
    if (length > (size_t)(WIFI_BUFF_SIZE - transmitBufferLength)) return; //sem espaco - descarta
    memcpy(&transmitBuffer[transmitBufferLength], bytes, length);
    transmitBufferLength += length;
}

void CommBuffer::sendByteToBuffer(uint8_t byt)
{
    if (transmitBufferLength >= WIFI_BUFF_SIZE) return; //sem espaco - descarta
    transmitBuffer[transmitBufferLength++] = byt;
}

void CommBuffer::sendString(String str)
{
    char buff[300];
    str.toCharArray(buff, 300);
    sendCharString(buff);
}

void CommBuffer::sendCharString(char *str)
{
    char *p = str;
    int i = 0;
    while (*p)
    {
        sendByteToBuffer(*p++);
        i++;
    }
    Logger::debug("Queued %i bytes", i);
}

void CommBuffer::sendFrameToBuffer(CAN_FRAME &frame, int whichBus)
{
    uint8_t temp;
    size_t writtenBytes;
    uint8_t length = (frame.length > 8) ? 8 : frame.length;
    //pior caso: 20 bytes em binario, ~60 em texto. Sem espaco, descarta o frame.
    if (transmitBufferLength > (WIFI_BUFF_SIZE - 80)) return;
    if (settings.useBinarySerialComm) {
        uint32_t id = frame.id;
        if (frame.extended) id |= 1ul << 31; //nao altera o frame de quem chamou
        transmitBuffer[transmitBufferLength++] = 0xF1;
        transmitBuffer[transmitBufferLength++] = 0; //0 = canbus frame sending
        uint32_t now = micros();
        transmitBuffer[transmitBufferLength++] = (uint8_t)(now & 0xFF);
        transmitBuffer[transmitBufferLength++] = (uint8_t)(now >> 8);
        transmitBuffer[transmitBufferLength++] = (uint8_t)(now >> 16);
        transmitBuffer[transmitBufferLength++] = (uint8_t)(now >> 24);
        transmitBuffer[transmitBufferLength++] = (uint8_t)(id & 0xFF);
        transmitBuffer[transmitBufferLength++] = (uint8_t)(id >> 8);
        transmitBuffer[transmitBufferLength++] = (uint8_t)(id >> 16);
        transmitBuffer[transmitBufferLength++] = (uint8_t)(id >> 24);
        transmitBuffer[transmitBufferLength++] = length + (uint8_t)(whichBus << 4);
        for (int c = 0; c < length; c++) {
            transmitBuffer[transmitBufferLength++] = frame.data.uint8[c];
        }
        //temp = checksumCalc(buff, 11 + frame.length);
        temp = 0;
        transmitBuffer[transmitBufferLength++] = temp;
        //Serial.write(buff, 12 + frame.length);
    } else {
        writtenBytes = sprintf((char *)&transmitBuffer[transmitBufferLength], "%lu : %x", (unsigned long)micros(), (unsigned int)frame.id);
        transmitBufferLength += writtenBytes;
        if (frame.extended) sprintf((char *)&transmitBuffer[transmitBufferLength], " X ");
        else sprintf((char *)&transmitBuffer[transmitBufferLength], " S ");
        transmitBufferLength += 3;
        writtenBytes = sprintf((char *)&transmitBuffer[transmitBufferLength], "%i %i", whichBus, length);
        transmitBufferLength += writtenBytes;
        for (int c = 0; c < length; c++) {
            writtenBytes = sprintf((char *)&transmitBuffer[transmitBufferLength], " %x", frame.data.uint8[c]);
            transmitBufferLength += writtenBytes;
        }
        sprintf((char *)&transmitBuffer[transmitBufferLength], "\r\n");
        transmitBufferLength += 2;
    }
}

void CommBuffer::sendFrameToBuffer(CAN_FRAME_FD &frame, int whichBus)
{
    uint8_t temp;
    size_t writtenBytes;
    uint8_t length = (frame.length > 64) ? 64 : frame.length;
    //pior caso: 77 bytes em binario, ~230 em texto. Sem espaco, descarta o frame.
    if (transmitBufferLength > (WIFI_BUFF_SIZE - 256)) return;
    if (settings.useBinarySerialComm) {
        uint32_t id = frame.id;
        if (frame.extended) id |= 1ul << 31; //nao altera o frame de quem chamou
        transmitBuffer[transmitBufferLength++] = 0xF1;
        transmitBuffer[transmitBufferLength++] = PROTO_BUILD_FD_FRAME;
        uint32_t now = micros();
        transmitBuffer[transmitBufferLength++] = (uint8_t)(now & 0xFF);
        transmitBuffer[transmitBufferLength++] = (uint8_t)(now >> 8);
        transmitBuffer[transmitBufferLength++] = (uint8_t)(now >> 16);
        transmitBuffer[transmitBufferLength++] = (uint8_t)(now >> 24);
        transmitBuffer[transmitBufferLength++] = (uint8_t)(id & 0xFF);
        transmitBuffer[transmitBufferLength++] = (uint8_t)(id >> 8);
        transmitBuffer[transmitBufferLength++] = (uint8_t)(id >> 16);
        transmitBuffer[transmitBufferLength++] = (uint8_t)(id >> 24);
        transmitBuffer[transmitBufferLength++] = length;
        transmitBuffer[transmitBufferLength++] = (uint8_t)(whichBus);
        for (int c = 0; c < length; c++) {
            transmitBuffer[transmitBufferLength++] = frame.data.uint8[c];
        }
        //temp = checksumCalc(buff, 11 + frame.length);
        temp = 0;
        transmitBuffer[transmitBufferLength++] = temp;
        //Serial.write(buff, 12 + frame.length);
    } else {
        writtenBytes = sprintf((char *)&transmitBuffer[transmitBufferLength], "%lu - %x", (unsigned long)micros(), (unsigned int)frame.id);
        transmitBufferLength += writtenBytes;
        if (frame.extended) sprintf((char *)&transmitBuffer[transmitBufferLength], " X ");
        else sprintf((char *)&transmitBuffer[transmitBufferLength], " S ");
        transmitBufferLength += 3;
        writtenBytes = sprintf((char *)&transmitBuffer[transmitBufferLength], "%i %i", whichBus, length);
        transmitBufferLength += writtenBytes;
        for (int c = 0; c < length; c++) {
            writtenBytes = sprintf((char *)&transmitBuffer[transmitBufferLength], " %x", frame.data.uint8[c]);
            transmitBufferLength += writtenBytes;
        }
        sprintf((char *)&transmitBuffer[transmitBufferLength], "\r\n");
        transmitBufferLength += 2;
    }
}

