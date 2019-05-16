#include "serialtransmission.h"


namespace serial_transmission{

bool SerialTransmission::StartSerial(SerialConfig& serialconfig)
{
    // Initial serial
    serial::Timeout timeout = serial::Timeout::simpleTimeout(serialconfig.timeout);
    sp.setPort(serialconfig.port);
    sp.setBaudrate(serialconfig.baudrate);
    sp.setStopbits(serialconfig.stopbits);
    sp.setParity(serialconfig.parity);
    sp.setTimeout(timeout);
    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR("Unable to open port");
        return false;
    }
    if(sp.isOpen())
    {
        ROS_INFO("%s is opened", serialconfig.port.c_str());
    }
    else
    {
        return false;
    }

    return true;
}

bool SerialTransmission::RecvData(uint8_t* data, size_t len)
{
    uint8_t header = 0x0;
    uint8_t tail = 0x0;

    // Wait for head of the package
    while(true)
    {
        while(sp.available() < 1);
        sp.read(&header, 1);
        if(header == 0x7f)
        {
            break;
        }
    }

    // Wait for enough data
    while(sp.available() < len + 1);
    sp.read(data, len);
    sp.read(&tail, 1);

    // Flush the input
    sp.flushInput();

    // Validate the package
    if(tail != 0x7e)
    {
        ROS_ERROR("Package is not valid");
        return false;
    }

    return true;
}

uint8_t* SerialTransmission::RecvDataWithSize(size_t& len)
{
    uint8_t header = 0x0;
    uint8_t tail = 0x0;
    uint16_t _len = 0;
    uint8_t* data = NULL;

    // Wait for head of the package
    while(true)
    {
        while(sp.available() < 1);
        sp.read(&header, 1);
        if(header == 0x7f)
        {
            break;
        }
    }

    // Wait for size data
    while(sp.available() < 2);
    sp.read((uint8_t*)&_len, 2);
    len = _len;
    data = new uint8_t[len];

    // Wait for enough data
    while(sp.available() < len + 1);
    sp.read(data, len);
    sp.read(&tail, 1);

    // Flush the input
    sp.flushInput();

    // Validate the package
    if(tail != 0x7e)
    {
        ROS_ERROR("Package is not valid");
        return NULL;
    }

    return data;
}

bool SerialTransmission::SendData(uint8_t* data, size_t len)
{
    // Add package header and tail
    uint8_t header = 0x7f;
    uint8_t tail = 0x7e;
    size_t txLen = 0;

    txLen += sp.write(&header, 1);
    txLen += sp.write(data, len);
    txLen += sp.write(&tail, 1);

    if(txLen != len + 2)
    {
        ROS_ERROR("Send package size %lu is not equal to %lu", txLen, len + 2);
        return false;
    }

    return true;
}

bool SerialTransmission::SendDataWithSize(uint8_t* data, size_t len)
{
    // Add package header and tail
    uint8_t header = 0x7f;
    uint8_t tail = 0x7e;
    uint16_t _len = len;
    size_t txLen = 0;

    txLen += sp.write(&header, 1);
    txLen += sp.write((uint8_t*)&_len, 2);
    txLen += sp.write(data, len);
    txLen += sp.write(&tail, 1);

    if(txLen != len + 4)
    {
        ROS_ERROR("Send package size %lu is not equal to %lu", txLen, len + 4);
        return false;
    }

    return true;
}

};




