#include "motoudp.h"
#include <QUdpSocket>
#include <QString>
#include <QByteArray>
#include <QFile>
#include <math.h>
#include <unistd.h>
#include <iostream>
MotoUDP::MotoUDP(QHostAddress h,quint16 p)
{
    _HostAddress = h;
    _port = p;
    client = new QUdpSocket;
}
MotoUDP::~MotoUDP()
{
    delete client;
}

void MotoUDP::run()
{
    ConnectMotoman();
    while (isRunning) {
        uint8_t buffer[4];
        if(ReadMultipleBytes(32,4,buffer)){
            if(trigger==0&&buffer[0]>0) {
                Q_EMIT triggerPPF();
            }
            if(gripperOpenCommand==0&&buffer[1]>0){
                Q_EMIT triggerGripper(buffer[2]);
            }
            trigger = buffer[0];
            gripperOpenCommand = buffer[1];
            gripperOpenWidth = buffer[2];
            PickingDistance = buffer[3];
        }
        else{
            usleep(3000000);
        }
        uint8_t bufferWrite[4];
        bufferWrite[0] = ready;
        bufferWrite[1] = good;
        bufferWrite[2] = fail;
        bufferWrite[3] = objectID;
        if(!WriteByte(36,ready)){
            usleep(3000000);
        }
        if(triggerWritePositions){
//            WriteVarPulse(32,position);
//            usleep(500000);
//            WriteVarPulse(33,position+6);
            std::cout << "triggerWritePositions ";

            std::cout << WriteMultipleVarInt(32,12,position);
            usleep(100000);
            WriteByte(38,fail);
            std::cout << " fail " << int(fail);
            usleep(10000);
            WriteByte(37,good);
            std::cout << " good " << int(good);
            good = 0;
            WriteByte(37,good);
            std::cout << " good " << int(good) << std::endl;
            triggerWritePositions = false;
        }
        if(triggerDone){
            WriteByte(38,fail);
            std::cout << " fail " << int(fail);
            std::cout << " good " << int(good);
            WriteByte(37,good);
            good = 0;
            WriteByte(37,good);
            std::cout << " good " << int(good) << std::endl;
            triggerDone = false;
        }
        if(triggerTurnOnServo){
            TurnOnServo();
            triggerTurnOnServo = false;
        }
        if(triggerTurnOffServo){
            TurnOffServo();
            triggerTurnOffServo = false;
        }
        if(triggerStartJob){
            StartJob();
            triggerStartJob = false;
        }
    }
}
struct MotoUDP::MotoUDP::TxData {
    char identifier[4] = {'Y','E','R','C'};
    u_int16_t header_size = 32;
    u_int16_t data_size;
    u_int8_t reserve1 = 3;
    u_int8_t processing_division = 1;
    u_int8_t ack = 0;
    u_int8_t id;
    u_int32_t block_no = 0;
    char reserve2[8] = {'9','9','9','9','9','9','9','9'};
    u_int16_t command_no;
    u_int16_t instance;
    u_int8_t attribute;
    u_int8_t service;
    const u_int16_t padding = 0;

};
struct MotoUDP::MotoUDP::RxData {
    char identifier[4];
    u_int16_t header_size;
    u_int16_t data_size;
    u_int8_t reserve1;
    u_int8_t processing_division = 1;
    u_int8_t ack;
    u_int8_t id;
    u_int32_t block_no;
    char reserve2[8];
    u_int8_t service;
    u_int8_t status;
    u_int8_t added_status_size;
    u_int8_t padding;
    u_int16_t added_status;
    u_int16_t padding2;
};
struct MotoUDP::MotoUDP::TxDataWritePosition {
  const u_int32_t control_group_robot = 1;
  const u_int32_t control_group_station = 0;
  u_int32_t classification_in_speed = 0;
  u_int32_t speed;
  const uint32_t coordinate = 0x10;
  int32_t x;
  int32_t y;
  int32_t z;
  int32_t rx;
  int32_t ry;
  int32_t rz;
  const u_int32_t reservation1 = 0;
  const u_int32_t reservation2 = 0;
  const u_int32_t type = 0;
  const u_int32_t expanded_type = 0;
  const u_int32_t tool_no = 0;
  const u_int32_t user_coordinate_no = 0;
  const u_int32_t base_1_position = 0;
  const u_int32_t base_2_position = 0;
  const u_int32_t base_3_position = 0;
  const u_int32_t station_1_position = 0;
  const u_int32_t station_2_position = 0;
  const u_int32_t station_3_position = 0;
  const u_int32_t station_4_position = 0;
  const u_int32_t station_5_position = 0;
  const u_int32_t station_6_position = 0;
};
struct MotoUDP::MotoUDP::TxDataWritePulse {
  const u_int32_t control_group_robot = 1;
  const u_int32_t control_group_station = 0;
  u_int32_t classification_in_speed = 0;
  u_int32_t speed;
  int32_t r1;
  int32_t r2;
  int32_t r3;
  int32_t r4;
  int32_t r5;
  int32_t r6;
  int32_t r7;
  int32_t r8;
  const u_int32_t tool_no = 0;
  const u_int32_t base_1_position = 0;
  const u_int32_t base_2_position = 0;
  const u_int32_t base_3_position = 0;
  const u_int32_t station_1_position = 0;
  const u_int32_t station_2_position = 0;
  const u_int32_t station_3_position = 0;
  const u_int32_t station_4_position = 0;
  const u_int32_t station_5_position = 0;
  const u_int32_t station_6_position = 0;
};
struct MotoUDP::MotoUDP::TxDataWriteVariablePosition
{
  u_int32_t data_type;
  const u_int32_t figure = 0;
  const u_int32_t tool_no = 0;
  const u_int32_t user_coodirnate_no = 0;
  const u_int32_t extended_figure = 0;
  int32_t first_axis_position;
  int32_t second_axis_position;
  int32_t third_axis_position;
  int32_t fourth_axis_position;
  int32_t fifth_axis_position;
  int32_t sixth_axis_position;
  const int32_t seventh_axis_position = 0;
  const int32_t eighth_axis_position = 0;
};
bool MotoUDP::MotoUDP::ConnectMotoman()
{
    client->bind();
    return true;
}
bool MotoUDP::MotoUDP::CloseMotoman()
{
    client->close();
    return true;
}
bool MotoUDP::MotoUDP::SendData (char* buffer, int lenght)
{
  client->writeDatagram(buffer,lenght,_HostAddress,_port);
  return 1;
}
bool  MotoUDP::MotoUDP::TurnOnServo()
{
    TxData sent_data;
    u_int32_t data = 1;
    sent_data.id = RECEIVE_TYPE::ON_SERVO;
    sent_data.command_no = 0x83;
    sent_data.instance = 2;
    sent_data.attribute = 1;
    sent_data.service = 0x10;
    u_int16_t data_length = sizeof (data);
    u_int16_t total_length = 32 + data_length;
    sent_data.data_size = data_length;
    char buffer [total_length];
    memcpy(buffer,&sent_data,32);
    memcpy(buffer+32,&data,data_length);
    SendData(buffer,total_length);
    usleep(300000);
    if(client->pendingDatagramSize()>0){
        QByteArray array;
        RxData rxHeader;
        array.resize(client->pendingDatagramSize());
        client->readDatagram(array.data(),array.size());
        memcpy(&rxHeader,array.data(),32);
        if(rxHeader.status!=0){
            std::cout << "Cannot turn on the servo with error code " << std::hex << rxHeader.added_status << std::endl;
            return false;
        }
        return true;
    }
    else {
        std::cout << "Cannot connect to the controller!" << std::endl;
        return false;
    }
}
bool MotoUDP::MotoUDP::TurnOffServo()
{
  TxData sent_data;
  u_int32_t data = 2;
  sent_data.id = RECEIVE_TYPE::OFF_SERVO;
  sent_data.command_no = 0x83;
  sent_data.instance = 2;
  sent_data.attribute = 1;
  sent_data.service = 0x10;
  u_int16_t data_length = sizeof (data);
  u_int16_t total_length = 32 + data_length;
  sent_data.data_size = data_length;
  char buffer[total_length];
  memcpy(buffer,&sent_data,32);
  memcpy(buffer+32,&data,data_length);
  SendData(buffer,total_length);
  usleep(300000);
  if(client->pendingDatagramSize()>0){
      QByteArray array;
      RxData rxHeader;
      array.resize(client->pendingDatagramSize());
      client->readDatagram(array.data(),array.size());
      memcpy(&rxHeader,array.data(),32);
      if(rxHeader.status!=0){
          std::cout << "Cannot turn off the servo with error code " << std::hex << rxHeader.added_status << std::endl;
          return false;
      }
      return true;
  }
  else {
      std::cout << "Cannot connect to the controller!" << std::endl;
      return false;
  }
}
bool MotoUDP::MotoUDP::GetPosition(int32_t *pos)
{
  TxData sent_data;
  char buffer [sizeof (sent_data)];
  sent_data.id = RECEIVE_TYPE::GET_POSITION;
  sent_data.command_no = 0x75;
  sent_data.instance = 0x65;
  sent_data.attribute = 0;
  sent_data.service = 0x01;
  sent_data.data_size = 0;
  memcpy(buffer,&sent_data,sizeof (sent_data));
  SendData(buffer,sizeof (sent_data));
  //std::cout << "Send data ... " ;
  usleep(30000);
  //std::cout << client->pendingDatagramSize() << "byte, " ;
  if(client->pendingDatagramSize()>0){
      QByteArray array;
      RxData rxHeader;
      array.resize(client->pendingDatagramSize());
      client->readDatagram(array.data(),array.size());
      memcpy(&rxHeader,array.data(),32);
      if(rxHeader.status!=0) return false;
      memcpy(pos,array.data()+52,24);
      //std::cout << *(pos+5) << std::endl ;
      return true;
  }
  else {
      return false;
  }
}
bool MotoUDP::MotoUDP::GetPulsePosition(int32_t *joint)
{
  TxData sent_data;
  char buffer [sizeof (sent_data)];
  sent_data.id = RECEIVE_TYPE::GET_PULSE;
  sent_data.command_no = 0x75;
  sent_data.instance = 0x01;
  sent_data.attribute = 0;
  sent_data.service = 0x01;
  sent_data.data_size = 0;
  memcpy(buffer,&sent_data,sizeof (sent_data));
  SendData(buffer,sizeof (sent_data));
  usleep(30000);
  if(client->pendingDatagramSize()>0){
      QByteArray array;
      RxData rxHeader;
      array.resize(client->pendingDatagramSize());
      client->readDatagram(array.data(),array.size());
      memcpy(&rxHeader,array.data(),32);
      if(rxHeader.status!=0) return false;
      memcpy(joint,array.data()+52,24);
      return true;
  }
  else {
      return false;
  }
}
bool MotoUDP::MotoUDP::GetVarPosition(u_int16_t index,int32_t* pos)
{
  TxData sent_data;
  char buffer [sizeof (sent_data)];
  sent_data.id = 06;
  sent_data.command_no = 0x7f;
  sent_data.instance = index;
  sent_data.attribute = 0;
  sent_data.service = 0x0e;
  sent_data.data_size = 0;

  memcpy(buffer,&sent_data,sizeof (sent_data));
  SendData(buffer,sizeof (sent_data));
  usleep(30000);
  if(client->pendingDatagramSize()>0){
      QByteArray array;
      RxData rxHeader;
      array.resize(client->pendingDatagramSize());
      client->readDatagram(array.data(),array.size());
      memcpy(&rxHeader,array.data(),32);
      if(rxHeader.status!=0) return false;
      memcpy(pos,array.data()+52,24);
      return true;
  }
  else {
      return false;
  }
}
bool MotoUDP::GetMultipleVarPosition(u_int16_t index, u_int32_t number, int32_t *pos)
{
    TxData sent_data;
    char buffer [sizeof (sent_data)+4];
    sent_data.id = 06;
    sent_data.command_no = 0x307;
    sent_data.instance = index;
    sent_data.attribute = 0;
    sent_data.service = 0x33;
    sent_data.data_size = 4;
    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+32,&number,sizeof (number));
    SendData(buffer,sizeof(buffer));
    usleep(40000);
    if(client->pendingDatagramSize()>0){
        QByteArray array;
        RxData rxHeader;
        array.resize(client->pendingDatagramSize());
        client->readDatagram(array.data(),array.size());
        memcpy(&rxHeader,array.data(),32);
        if(rxHeader.status!=0) {
            std::cout << "Cannot read mutiple positions with error code " << std::hex << rxHeader.added_status << std::endl;
            return false;}

        for (u_int32_t i= 0;i<number;i++) {
            //memcpy(pos+24*i,array.data()+53+i*24,24);
            TxDataWriteVariablePosition position;
            memcpy(&position,array.data()+4+sizeof(TxData)+sizeof (TxDataWriteVariablePosition)*i,sizeof (TxDataWriteVariablePosition));

            pos[6*i+0]=position.first_axis_position;
            pos[6*i+1]=position.second_axis_position;
            pos[6*i+2]=position.third_axis_position;
            pos[6*i+3]=position.fourth_axis_position;
            pos[6*i+4]=position.fifth_axis_position;
            pos[6*i+5]=position.sixth_axis_position;
        }
        return true;
    }
    else {
         std::cout << "Cannot connect to the controller!" << std::endl;
        return false;
    }
}
bool MotoUDP::MotoUDP::WriteVarPosition(u_int16_t index, int32_t* pos)
{
  TxData sent_data;
  sent_data.id = 07;
  sent_data.command_no = 0x7f;
  sent_data.instance = index;
  sent_data.attribute = 0;
  sent_data.service = 0x10;
  TxDataWriteVariablePosition position;
  sent_data.data_size = sizeof (position);
  position.data_type = 17;
  position.first_axis_position = pos[0];
  position.second_axis_position = pos[1];
  position.third_axis_position = pos[2];
  position.fourth_axis_position = pos[3];
  position.fifth_axis_position = pos[4];
  position.sixth_axis_position = pos[5];
  char buffer[sizeof(sent_data)+ sizeof (position)];
  memcpy(buffer,&sent_data,sizeof (sent_data));
  memcpy(buffer+sizeof(sent_data),&position,sizeof(position));
  SendData(buffer,sizeof(sent_data)+ sizeof (position));
  usleep(30000);
  if(client->pendingDatagramSize()>0){
      QByteArray array;
      RxData rxHeader;
      array.resize(client->pendingDatagramSize());
      client->readDatagram(array.data(),array.size());
      memcpy(&rxHeader,array.data(),32);
      if(rxHeader.status!=0) return false;
      return true;
  }
  else {
      return false;
  }
}

bool MotoUDP::WriteVarPulse(u_int16_t index, int32_t *pos)
{
    TxData sent_data;
    sent_data.id = 07;
    sent_data.command_no = 0x7f;
    sent_data.instance = index;
    sent_data.attribute = 0;
    sent_data.service = 0x10;
    TxDataWriteVariablePosition position;
    sent_data.data_size = sizeof (position);
    position.data_type = 0;
    position.first_axis_position = pos[0];
    position.second_axis_position = pos[1];
    position.third_axis_position = pos[2];
    position.fourth_axis_position = pos[3];
    position.fifth_axis_position = pos[4];
    position.sixth_axis_position = pos[5];
    char buffer[sizeof(sent_data)+ sizeof (position)];
    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof(sent_data),&position,sizeof(position));
    SendData(buffer,sizeof(sent_data)+ sizeof (position));
    usleep(30000);
    if(client->pendingDatagramSize()>0){
        QByteArray array;
        RxData rxHeader;
        array.resize(client->pendingDatagramSize());
        client->readDatagram(array.data(),array.size());
        memcpy(&rxHeader,array.data(),32);
        if(rxHeader.status!=0) return false;
        return true;
    }
    else {
        return false;
    }
}
bool MotoUDP::WriteMultipleVarPosition(u_int16_t index, u_int32_t number, int32_t *pos)
{
    TxData sent_data;
    sent_data.id = 07;
    sent_data.command_no = 0x307;
    sent_data.instance = index;
    sent_data.attribute = 0;
    sent_data.service = 0x34;

    char buffer[4+sizeof(sent_data)+ sizeof (TxDataWriteVariablePosition)*number];
    sent_data.data_size = sizeof(TxDataWriteVariablePosition)*number+4;

    for (int i = 0;i<number;i++) {
        TxDataWriteVariablePosition position;
        position.data_type = 17;
        position.first_axis_position = pos[6*i];
        position.second_axis_position = pos[6*i+1];
        position.third_axis_position = pos[6*i+2];
        position.fourth_axis_position = pos[6*i+3];
        position.fifth_axis_position = pos[6*i+4];
        position.sixth_axis_position = pos[6*i+5];
        memcpy(buffer+sizeof(sent_data)+sizeof(position)*i+4,&position,sizeof(position));
    }
    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof (sent_data),&number,sizeof (number));
    //std::cout << buffer << std::endl;
    SendData(buffer,sent_data.header_size+sent_data.data_size);
    usleep(40000);
    if(client->pendingDatagramSize()>0){
        QByteArray array;
        RxData rxHeader;
        array.resize(client->pendingDatagramSize());
        client->readDatagram(array.data(),array.size());
        memcpy(&rxHeader,array.data(),32);
        if(rxHeader.status!=0){std::cout << "Multiple Position Variable Status Code " << std::hex << rxHeader.added_status  << std::endl; return false;}
        return true;
    }
    else {
        std::cout << "Cannot connect to the controller!"  << std::endl;
        return false;
    }
}

bool MotoUDP::WriteMultipleVarInt(u_int16_t index, u_int32_t number, int32_t *value)
{
    TxData sent_data;
    sent_data.id = 100;
    sent_data.command_no = 0x304;
    sent_data.instance = index;
    sent_data.attribute = 0;
    sent_data.service = 0x34;
    sent_data.data_size = 4 + number*4;
    char buffer [sizeof(sent_data)+ 4 + number*4];
    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof(sent_data),&number,4);
    memcpy(buffer+sizeof(sent_data)+4,value,number*4);

    SendData(buffer,sent_data.header_size+sent_data.data_size);
    usleep(30000);
    if(client->pendingDatagramSize()>0){
        QByteArray array;
        RxData rxHeader;
        array.resize(client->pendingDatagramSize());
        client->readDatagram(array.data(),array.size());
        memcpy(&rxHeader,array.data(),32);
        if(rxHeader.status!=0)
        {
            std::cout << "Status Code Writing Multiple Double Ints " << std::hex << rxHeader.added_status << std::endl;
            return false;
        }
        return true;
    }
    else {
         std::cout << "Cannot connect to the controller!" << std::endl;
        return false;
    }
}

bool MotoUDP::WriteMultipleVarPulse(u_int16_t index, u_int32_t number, int32_t *pos)
{
    TxData sent_data;
    sent_data.id = 07;
    sent_data.command_no = 0x307;
    sent_data.instance = index;
    sent_data.attribute = 0;
    sent_data.service = 0x34;

    char buffer[4+sizeof(sent_data)+ sizeof (TxDataWriteVariablePosition)*number];
    sent_data.data_size = sizeof(TxDataWriteVariablePosition)*number+4;

    for (int i = 0;i<number;i++) {
        TxDataWriteVariablePosition position;
        position.data_type = 0;
        position.first_axis_position = pos[6*i];
        position.second_axis_position = pos[6*i+1];
        position.third_axis_position = pos[6*i+2];
        position.fourth_axis_position = pos[6*i+3];
        position.fifth_axis_position = pos[6*i+4];
        position.sixth_axis_position = pos[6*i+5];
        memcpy(buffer+sizeof(sent_data)+sizeof(position)*i+4,&position,sizeof(position));
    }
    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof (sent_data),&number,sizeof (number));
    //std::cout << buffer << std::endl;
    SendData(buffer,sent_data.header_size+sent_data.data_size);
    usleep(40000);
    if(client->pendingDatagramSize()>0){
        QByteArray array;
        RxData rxHeader;
        array.resize(client->pendingDatagramSize());
        client->readDatagram(array.data(),array.size());
        memcpy(&rxHeader,array.data(),32);
        if(rxHeader.status!=0){std::cout << "Multiple Pulse Variable Status Code " << std::hex << rxHeader.added_status  << std::endl; return false;}
        return true;
    }
    else {
        std::cout << "Cannot connect to the controller!"  << std::endl;
        return false;
    }
}
bool MotoUDP::MotoUDP::SelectJob(char* jobname){
  TxData sent_data;
  sent_data.id = RECEIVE_TYPE::JOB_SELLECT;
  sent_data.command_no = 0x87;
  sent_data.instance = 1;
  sent_data.attribute = 0;
  sent_data.service = 0x02;
  sent_data.data_size = 36;
  u_int32_t line = 0;
  char buffer [sizeof(sent_data)+ 36];

  memcpy(buffer,&sent_data,sizeof (sent_data));
  memcpy(buffer+sizeof(sent_data),jobname,32);
  memcpy(buffer+sizeof(sent_data)+32,&line,4);
  SendData(buffer,sizeof(sent_data)+36);
  usleep(30000);
  if(client->pendingDatagramSize()>0){
      QByteArray array;
      RxData rxHeader;
      array.resize(client->pendingDatagramSize());
      client->readDatagram(array.data(),array.size());
      memcpy(&rxHeader,array.data(),32);
      if(rxHeader.status!=0) return false;
      return true;
  }
  else {
      return false;
  }
}
bool MotoUDP::MotoUDP::StartJob(){
  TxData sent_data;
  sent_data.id = 9;
  sent_data.command_no = 0x86;
  sent_data.instance = 1;
  sent_data.attribute = 1;
  sent_data.service = 0x10;
  u_int32_t data = 1;
  sent_data.data_size = sizeof (data);
  char buffer [sizeof(sent_data)+ sizeof(data)];
  memcpy(buffer,&sent_data,sizeof (sent_data));
  memcpy(buffer+sizeof(sent_data),&data,sizeof(data));
  SendData(buffer,sizeof(sent_data)+sizeof(data));
  usleep(30000);
  if(client->pendingDatagramSize()>0){
      QByteArray array;
      RxData rxHeader;
      array.resize(client->pendingDatagramSize());
      client->readDatagram(array.data(),array.size());
      memcpy(&rxHeader,array.data(),32);
      if(rxHeader.status!=0) return false;
      return true;
  }
  else {
      return false;
  }
}
bool MotoUDP::MotoUDP::ConnectToPLC(QHostAddress host, u_int port,uint16_t adr,uint16_t no_reg,std::vector<uint16_t> data)
{
  struct tx{
        uint8_t Command_type = 0x11;
        uint8_t Identification_number = 0;
        uint16_t channel_number = 0;
        uint16_t not_use_1 = 0;
        uint16_t total_lenght;
        uint32_t not_use_2 = 0;
        uint16_t data_lenght;
        uint16_t MFC_SFC = 0x0B20;
        uint16_t CPU_number = 16;
        uint16_t reference_address ;
        uint16_t number_register ;
        uint16_t data_reg[];
      } tx_data;
      tx_data.reference_address = adr;
      tx_data.number_register = no_reg;
      memcpy(tx_data.data_reg,data.data(),data.size());
      tx_data.data_lenght = 8 + 2*data.size();
      tx_data.total_lenght = 14+tx_data.data_lenght;
      char buffer[tx_data.total_lenght];
      memcpy(buffer,&tx_data,sizeof(tx_data));
      client->writeDatagram(buffer,sizeof(tx_data),host,port);
      return true;
}
bool MotoUDP::MotoUDP::WriteByte(u_int16_t instance,uint8_t data){
  TxData sent_data;
  sent_data.id = RECEIVE_TYPE::WRITE_BYTE;
  sent_data.command_no = 0x7A;
  sent_data.instance = instance;
  sent_data.attribute = 1;
  sent_data.service = 0x10;
  sent_data.data_size = sizeof (data);
  char buffer [sizeof(sent_data)+ sizeof(data)];
  memcpy(buffer,&sent_data,sizeof (sent_data));
  memcpy(buffer+sizeof(sent_data),&data,sizeof(data));
  SendData(buffer,sizeof(sent_data)+sizeof(data));
  usleep(30000);
  if(client->pendingDatagramSize()>0){
      QByteArray array;
      RxData rxHeader;
      array.resize(client->pendingDatagramSize());
      client->readDatagram(array.data(),array.size());
      memcpy(&rxHeader,array.data(),32);
      if(rxHeader.status!=0) {
          std::cout << "Write Byte Status Code " << std::hex << rxHeader.added_status  << std::endl;
          return false;
      }
      return true;
  }
  else {
      std::cout << "Cannot connect to the controller!"  << std::endl;
      return false;
  }
}

bool MotoUDP::WriteMultipleBytes(u_int16_t instance, uint32_t number, uint8_t *data)
{
    TxData sent_data;
    sent_data.id = RECEIVE_TYPE::WRITE_BYTE;
    sent_data.command_no = 0x302;
    sent_data.instance = instance;
    sent_data.attribute = 0;
    sent_data.service = 0x34;
    sent_data.data_size = 4 + number;
    char buffer [sizeof(sent_data)+ 4 + number];
    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof(sent_data),&number,4);
    memcpy(buffer+sizeof(sent_data)+4,data,number);

    SendData(buffer,sent_data.header_size+sent_data.data_size);
    usleep(30000);
    if(client->pendingDatagramSize()>0){
        QByteArray array;
        RxData rxHeader;
        array.resize(client->pendingDatagramSize());
        client->readDatagram(array.data(),array.size());
        memcpy(&rxHeader,array.data(),32);
        if(rxHeader.status!=0)
        {
            std::cout << "Status Code Reading Multiple Bytes " << std::hex << rxHeader.added_status << std::endl;
            return false;
        }
        return true;
    }
    else {
         std::cout << "Cannot connect to the controller!" << std::endl;
        return false;
    }
}

bool MotoUDP::ReadMultipleBytes(u_int16_t instance, uint32_t number, uint8_t *data)
{
    TxData sent_data;
    sent_data.id = RECEIVE_TYPE::WRITE_BYTE;
    sent_data.command_no = 0x302;
    sent_data.instance = instance;
    sent_data.attribute = 0;
    sent_data.service = 0x33;
    sent_data.data_size = 4;
    char buffer [sizeof(sent_data)+ 4];
    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof(sent_data),&number,4);

    SendData(buffer,sent_data.header_size+sent_data.data_size);
    usleep(30000);
    if(client->pendingDatagramSize()>0){
        QByteArray array;
        RxData rxHeader;
        array.resize(client->pendingDatagramSize());
        client->readDatagram(array.data(),array.size());
        memcpy(&rxHeader,array.data(),32);
        if(rxHeader.status!=0)
        {
            std::cout << "Status Code Writing Multiple Bytes " << std::hex << rxHeader.added_status << std::endl;
            return false;
        }
        memcpy(data,array.data()+36,number);
        return true;
    }
    else {
        std::cout << "Cannot connect to the controller!" << std::endl;
        return false;
    }
}
const double MotoUDP::MotoUDP::PULSE_PER_DEGREE_S = 34816/30;
const double MotoUDP::MotoUDP::PULSE_PER_DEGREE_L = 102400/90;
const double MotoUDP::MotoUDP::PULSE_PER_DEGREE_U = 51200/90;
const double MotoUDP::MotoUDP::PULSE_PER_DEGREE_RBT = 10204/30;
