#ifndef SERIALTRANSMISSION_H
#define SERIALTRANSMISSION_H


#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>


using namespace std;


namespace serial_transmission{

struct SerialConfig{
    string port;
    int baudrate;
    serial::stopbits_t stopbits;
    serial::parity_t parity;
    int timeout;

    SerialConfig(){
        port = "ttyUSB0";
        baudrate = 115200;
        stopbits = serial::stopbits_one;
        parity = serial::parity_none;
        timeout = 1000;
    };
};

class SerialTransmission{
public:
    // Interface function
    bool StartSerial(SerialConfig& serialconfig);

    // Receive target position data
    bool RecvData(uint8_t* data, size_t len);
    template<typename T>
    bool RecvData(T* data, size_t len);

    uint8_t* RecvDataWithSize(size_t& len);
    template<typename T>
    T* RecvDataWithSize(size_t& len);
    template<typename T>
    bool RecvDataWithSize(vector<T>& data);

    // Send target position data
    bool SendData(uint8_t* data, size_t len);
    template<typename T>
    bool SendData(T* data, size_t len);

    bool SendDataWithSize(uint8_t* data, size_t len);
    template<typename T>
    bool SendDataWithSize(T* data, size_t len);
    template<typename T>
    bool SendDataWithSize(vector<T>& data);

private:
  // Objects
  serial::Serial sp; // serial port
};


//////////////////////////////// template function ////////////////////////
template <typename T>
bool SerialTransmission::RecvData(T* data, size_t len)
{
    uint8_t* _data = (uint8_t*)data;

    return RecvData(_data, len * sizeof(T));
}

template<typename T>
T* SerialTransmission::RecvDataWithSize(size_t& len)
{
    size_t _len = 0;
    uint8_t* _data = RecvDataWithSize(_len);
    len = _len / sizeof(T);

    return (T*)_data;
}

template<typename T>
bool SerialTransmission::RecvDataWithSize(vector<T>& data)
{
    size_t len;
    T* _data = RecvDataWithSize<T>(len);
    if(_data == NULL)
    {
        return false;
    }
    for(int i = 0;i < len;i++)
    {
        data.push_back(_data[i]);
    }
    delete _data;

    return true;
}

template <typename T>
bool SerialTransmission::SendData(T* data, size_t len)
{
    uint8_t* _data = (uint8_t*)data;

    return SendData(_data, len * sizeof(T));
}

template<typename T>
bool SerialTransmission::SendDataWithSize(T* data, size_t len)
{
    uint8_t* _data = (uint8_t*)data;

    return SendDataWithSize(_data, len * sizeof(T));
}

template<typename T>
bool SerialTransmission::SendDataWithSize(vector<T>& data)
{
    size_t len = data.size();
    T* _data = new T[len];
    for(int i = 0;i < len;i++)
    {
        _data[i] = data[i];
    }

    return SendDataWithSize(_data, len);
}

}

#endif // SERIALTRANSMISSION_H
