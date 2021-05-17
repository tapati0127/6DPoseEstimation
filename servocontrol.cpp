#include "servocontrol.h"
#include <iostream>
#include <unistd.h>

ServoControl::ServoControl()
{
    const auto list = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : list) {
        QString s = QObject::tr("Port: ") + info.portName() + "\n"
                           + QObject::tr("Location: ") + info.systemLocation() + "\n"
                           + QObject::tr("Description: ") + info.description() + "\n"
                           + QObject::tr("Manufacturer: ") + info.manufacturer() + "\n"
                           + QObject::tr("Serial number: ") + info.serialNumber() + "\n"
                           + QObject::tr("Vendor Identifier: ") + (info.hasVendorIdentifier() ? QString::number(info.vendorIdentifier(), 16) : QString()) + "\n"
                           + QObject::tr("Product Identifier: ") + (info.hasProductIdentifier() ? QString::number(info.productIdentifier(), 16) : QString()) + "\n"
                           + QObject::tr("Busy: ") + (info.isBusy() ? QObject::tr("Yes") : QObject::tr("No")) + "\n";
    std::cout << s.toStdString() << std::endl;
    }
    serialPort.setPortName("/dev/ttyUSB0");
    serialPort.setParity(QSerialPort::Parity::NoParity);
    serialPort.setBaudRate(115200);
    serialPort.setDataBits(QSerialPort::DataBits::Data8);
    serialPort.setStopBits(QSerialPort::StopBits::OneStop);
    serialPort.open(QIODevice::ReadWrite);
    //char test[32];
    //std::cout << serialPort.write(test,32) << std::endl;
}

uint16_t ServoControl::WriteServo(uint16_t value)
{
    struct request{
        const uint16_t header = 36055;
        const uint16_t command = 0;
        uint16_t value;
        uint16_t crc;
    };
    struct response{
        uint16_t header;
        uint16_t command;
        uint16_t error;
        uint16_t crc;
    };
    request req;
    response res;
    req.value = value;
    req.crc = req.header + req.command + req.value;
    std::cout << "Send " << req.header << " " << req.command << " " << req.value << " " << req.crc << std::endl;
    char buffer_req[8],buffer_res[8];
    memcpy(buffer_req,&req,8);
    std::cout << "Write " << serialPort.write(buffer_req,8) << std::endl;
    //usleep(10000);
    serialPort.setReadBufferSize(8);
    if(serialPort.waitForReadyRead(100)){
        serialPort.read(buffer_res,8);
        memcpy(&res,buffer_res,8);
        std::cout << "Receive " << res.header << " " << res.command << " " << res.error << " " << res.crc << std::endl;
        if(res.error==4){
            serialPort.write(buffer_req,8);
            usleep(50000);
            serialPort.read(buffer_res,8);
        }
        return  res.error;
    }
    else{
        return  5;
    }
}
