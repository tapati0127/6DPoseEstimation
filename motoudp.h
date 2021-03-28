#ifndef MOTOUDP_H
#define MOTOUDP_H

#include <QUdpSocket>

class MotoUDP : public QObject
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
    bool SendData (char* buffer, int lenght);
    bool ConnectMotoman();
    bool CloseMotoman();
    bool TurnOnServo();
    bool TurnOffServo();
    bool GetPosition();
    bool GetPulsePosition();
    bool GetVarPosition(u_int16_t index);
    bool SelectJob(char* jobname);
    bool StartJob();
    bool WritePosition(u_int16_t type,u_int32_t classification_in_speed, u_int32_t speed,int32_t* pos);
    bool WritePulse(u_int16_t type,u_int32_t classification_in_speed, u_int32_t speed,int32_t* pos);
    bool WriteVarPosition(u_int16_t index, std::vector<int32_t> pos);
    bool FileTransmitCommand(char name[],int length);
    bool FileReceiveCommand(char name[],int length);
    bool FileDeleteCommand(char name[],int length);
    bool GetJobFile(QString path);
    bool JobFile2ByteArray(QString path);
    bool ConnectToPLC(QHostAddress host, u_int port,uint16_t adr,uint16_t no_reg,std::vector<uint16_t> data);
    bool WriteByte(u_int16_t instance, uint8_t data);
    int32_t* GetCurrentPosition();
    int32_t* GetCurrentPulse();

    std::vector<double> ByteArray2Joint(QByteArray *pulse_buffer);
    RECEIVE_TYPE GetReceiveType ();
    bool CheckReceivedData(QByteArray buffer);
    QByteArray SplitArray(QByteArray array,int start, int count);
    int32_t Joint2Pulse(double joint, int i);
    double Pulse2Joint(int32_t pulse, int i);

    QByteArray* Get_rx_buffer();
    QString SendCommand(QByteArray buffer);
    QString ByteArray2Hex(QByteArray buffer);
    QByteArray Hex2ByteArray (QString s);
    QByteArray Int32ToByteArray (int32_t value);
    int32_t ByteArray2Int32 (QByteArray* buffer,int start, int number);
    QString rx_data;
    QString tx_data;
    QUdpSocket* client;
    bool isDataReceive;
    void ReceiveData();
    QByteArray* rx_file_buffer;
    QByteArray* tx_file_buffer;
Q_SIGNAL
    void receiveAllData();
Q_SIGNAL
    void transferAllData();
Q_SIGNAL
   void receiveError();
Q_SIGNAL
   void transferError();
private:
    QHostAddress _HostAddress;
    quint16 _port;
    QByteArray* rx_buffer;
    u_int32_t previous;
    u_int32_t index_file_transmit;
    u_int32_t max_index_file_transmit;
    u_int32_t last_byte_number;
    bool last_data;
    int32_t current_position[6];
    int32_t current_pulse[6];
    static const double PULSE_PER_DEGREE_S;
    static const double PULSE_PER_DEGREE_L;
    static const double PULSE_PER_DEGREE_U;
    static const double PULSE_PER_DEGREE_RBT;
    static const QString START_JOB;
    struct TxData;
    struct TxDataWritePosition;
    struct TxDataWritePulse;
    struct TxDataWriteVariablePosition;
};

#endif // MOTOUDP_H
