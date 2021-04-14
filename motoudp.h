#ifndef MOTOUDP_H
#define MOTOUDP_H

#include <QUdpSocket>
#include <QThread>
class MotoUDP : public QThread
{
Q_OBJECT
public:
    MotoUDP(QHostAddress h,quint16 p);
    enum RECEIVE_TYPE  {ON_SERVO = 0x00,
                        OFF_SERVO = 0x01,
                        GET_POSITION = 0x02,
                        GET_PULSE = 0x03,
                        WRITE_POSITION = 0x04,
                        WRITE_PUSLE = 0x05,
                        GET_VAR_POSITION = 0x06,
                        WRITE_VAR_POSITION = 0x07,
                        JOB_SELLECT = 0x08,
                        JOB_START = 0x09,
                        GET_STATUS,
                        ALARM_CLEAR,
                        FILE_TRANSMIT,
                        FILE_RECEIVE,
                        FILE_DELETE,
                        WRITE_BYTE
                       };
    ~MotoUDP();
    void run();
    bool ConnectMotoman();
    bool CloseMotoman();
    bool TurnOnServo();
    bool TurnOffServo();
    bool GetPosition(int32_t *pos);
    bool GetPulsePosition(int32_t *pos);
    bool GetVarPosition(u_int16_t index,int32_t* pos);
    bool GetMultipleVarPosition(u_int16_t index,u_int32_t number, int32_t* pos);
    bool SelectJob(char* jobname);
    bool StartJob();
    bool WriteVarPosition(u_int16_t index, int32_t* pos);
    bool WriteMultipleVarPosition(u_int16_t index,u_int32_t number, int32_t* pos);
    bool ConnectToPLC(QHostAddress host, u_int port,uint16_t adr,uint16_t no_reg,std::vector<uint16_t> data);
    bool WriteByte(u_int16_t instance, uint8_t data);

private:
    bool SendData (char* buffer, int lenght);
    QUdpSocket* client;
    QHostAddress _HostAddress;
    quint16 _port;
    static const double PULSE_PER_DEGREE_S;
    static const double PULSE_PER_DEGREE_L;
    static const double PULSE_PER_DEGREE_U;
    static const double PULSE_PER_DEGREE_RBT;
    struct TxData;
    struct TxDataWritePosition;
    struct TxDataWritePulse;
    struct TxDataWriteVariablePosition;
    struct RxData;
};

#endif // MOTOUDP_H
