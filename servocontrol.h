#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
class ServoControl
{
public:
    ServoControl();
    uint16_t WriteServo(uint16_t value);


private:
    QSerialPort serialPort;
};

#endif // SERVOCONTROL_H
