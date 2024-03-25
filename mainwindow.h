#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <QDebug>
#include <QMessageBox>
#include <QFile>
#include <QFileDialog>
#include <QTextStream>
#include <QtWidgets>
#include <QtEndian>
#include <QMetaEnum>
#include <QMetaObject>

#include "Angel_Kit_D10.h"

#include "filedisplaydialog.h"

/* DOD ID */
/*#define DOD_ID_AM_COMM_HDLR   	0
#define DOD_ID_BLE_COMM_HDLR  	1
#define DOD_ID_DATA_CTRL    	2
#define DOD_ID_DEBUG_CTRL 		3
#define DOD_ID_DEV_COMM_HDLR 	4
#define DOD_ID_EXT_DEV_CTRL	    5
#define DOD_ID_GAIT_CTRL   	    6
#define DOD_ID_SYSTEM_CTRL 	    7
#define DOD_ID_WHOLE_BODY_CTRL	8
*/

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

/*    Q_ENUM(NodeID)
    Q_ENUM(ObjectDictionaryFNCCode)
    Q_ENUM(ObjectDictionaryRoutineOnoff)
    Q_ENUM(ObjectDictionaryStateStatus)
    Q_ENUM(ObjectDictionaryDataTypeEnum)
    Q_ENUM(TaskID)
    Q_ENUM(DODID)
    Q_ENUM(SDOStatus)
    Q_ENUM(SDOIDLowLevel)
    Q_ENUM(SDOIDMidLevel)
    Q_ENUM(SDOIDMsg)
    Q_ENUM(SDOIDWIDM)
    Q_ENUM(SDOIDSysMngt)
    Q_ENUM(SDOIDExtDev)
    Q_ENUM(PDOIDLowlevel)
    Q_ENUM(PDOIDMidLevel)
    Q_ENUM(PDOIDMsg)
    Q_ENUM(PDOIDWIDM)
    Q_ENUM(PDOIDSysMngt)
    Q_ENUM(PDOIDExtDev)
    Q_ENUM(PDOIDWholeBody)
*/

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void centerWindow(int width, int height);
    void handleError(QSerialPort::SerialPortError error);

    void updateAvailablePorts();
    void fillPortsInfo();
    void updateCurrentPort(int index);

    void CreateObject(Object* obj);

    void readData();

    void startSending();
    void stopSending();
    void sendData();//const QByteArray &data);

    void sendSDO(Object* obj);
    void sendPDO(Object* obj);
    void on_pushButton_SerialConnect_clicked();

    void on_pushButton_SerialDisconnect_clicked();

    void on_pushButton_SDOSaveSettings_clicked();

    void on_pushButton_SDOSend_clicked();

/*    void on_pushButton_SDOReceive_clicked();

    void on_pushButton_SDOFeedbackSave_clicked();
*/
    void on_pushButton_PDOSendOnce_clicked();

    void on_pushButton_PDOSaveSettings_clicked();

   // void updateLineEdit();//QString &message);//const QByteArray &data);//QString &message);
//    void updateLineEdit(QLineEdit *lineedit);
    void updateLineEdit(QString message, QLineEdit *lineedit);

    void on_pushButton_PDOStartSending_clicked();

    void on_pushButton_PDOStopSending_clicked();

    void on_pushButton_FileOpen_clicked();

    void on_pushButton_BTUprightAct_clicked();

    void on_pushButton_BTUprightCmd_clicked();

    void on_pushButton_BTMsgTest_clicked();

    void on_pushButton_BTNTC_clicked();

    void on_pushButton_BTLTC2944_clicked();

    void on_pushButton_BTFSR_clicked();

    void on_pushButton_BTLP_clicked();

    void on_pushButton_BTIncEnc_clicked();

    void on_pushButton_BTAbsEnc_clicked();

    void on_pushButton_BTLEDtest_clicked();

    void setPDOList(uint8_t device_id, uint8_t dod_id, uint8_t pdo_id);//uint16_t pdo_id);
    void resetParams();
    void sendPDONothing(uint8_t device_id);
    void sendPDONothingOnce(uint8_t device_id);

    void on_pushButton_BTEndian_clicked();

    void on_pushButton_BTFloatTest_clicked();

/*
    void on_pushButton_MDBattVoltGet_clicked();

    void on_pushButton_MDUprightCmdGet_clicked();

    void on_pushButton_MDUprightActGet_clicked();

    void on_pushButton_MDStatusLEDStop_clicked();

    void on_pushButton_MDBattVoltStop_clicked();

    void on_pushButton_MDIMUStop_clicked();

    void on_pushButton_MDUprightCmdStop_clicked();

    void on_pushButton_MDUprightActStop_clicked();

    void on_pushButton_MDFSRStop_clicked();

    void on_pushButton_MDLPStop_clicked();

    void on_pushButton_MDNTCStop_clicked();

    void on_pushButton_MDCurrentCmdStop_clicked();

    void on_pushButton_MDCurrentActStop_clicked();

    void on_pushButton_MDAbsEncStop_clicked();

    void on_pushButton_MDIncEncStop_clicked();

    void on_pushButton_MDAnkleAbsEncStop_clicked();

*/

    //CM
    void CM_EMCY_GetState();
    void CM_Power_GetState();
    void CM_Guide_Left_GetState();
    void CM_Guide_Right_GetState();
    void CM_StatusLED_Left_GetColor();
    void CM_StatusLED_Right_GetColor();
    void CM_BatteryLED_GetColor();
    void CM_BatteryVolt_Get();
    void CM_BoardTemp_Get();
    void CM_BatteryPctg_Get();
    void CM_PelvisWidth_Left_Act_Get();
    void CM_PelvisWidth_Right_Act_Get();
    void CM_PelvisDepth_Left_Act_Get();
    void CM_PelvisDepth_Right_Act_Get();


    //Set
    void CM_PelvisWidth_Left_Cmd_Set();
    void CM_PelvisWidth_Left_BTPos_Set();
    void CM_PelvisWidth_Left_BTNeg_Set();
    void CM_PelvisWidth_Right_Cmd_Set();
    void CM_PelvisWidth_Right_BTPos_Set();
    void CM_PelvisWidth_Right_BTNeg_Set();
    void CM_PelvisDepth_Left_Cmd_Set();
    void CM_PelvisDepth_Left_BTPos_Set();
    void CM_PelvisDepth_Left_BTNeg_Set();
    void CM_PelvisDepth_Right_Cmd_Set();
    void CM_PelvisDepth_Right_BTPos_Set();
    void CM_PelvisDepth_Right_BTNeg_Set();


    //Stop
/*    void CM_EMCY_Stop();
    void CM_Power_Stop();
    void CM_Guide_Left_Stop();
    void CM_Guide_Right_Stop();
    void CM_StatusLED_Left_Stop();
    void CM_StatusLED_Right_Stop();
    void CM_BatteryLED_Stop();
    void CM_PelvisWidth_Left_Act_Stop();
    void CM_PelvisWidth_Right_Act_Stop();
    void CM_PelvisDepth_Left_Act_Stop();
    void CM_PelvisDepth_Right_Act_Stop();
*/
    //MD
    //Get
    void MD_ID_Get();
    void MD_StatusLED_GetColor();
    void MD_IMU_Get();
    void MD_BatteryVoltage_Get();
    void MD_BoardTemp_Get();
    void MD_UprightCmd_Get();
    void MD_UprightAct_Get();
    void MD_MotorTemp_Get();
    void MD_CurrentCmd_Get();
    void MD_CurrentAct_Get();
    void MD_AbsoluteEncoder_Get();
    void MD_IncrementalEncoder_Get();
    void MD_AnkleAbsoluteEncoder_Get();
    void MD_FSR_Get();
    void MD_LP_Get();

    //Set

    void MD_ID_Set();
    void MD_UprightCmd_Set();
    void MD_UprightCmd_BTPos_Set();
    void MD_UprightCmd_BTNeg_Set();
    void MD_CurrentCmd_Set();
    //Stop
/*
    void MD_ID_Stop();
    void MD_StatusLED_Stop();
    void MD_IMU_Stop();
    void MD_BatteryVoltage_Stop();
    void MD_UprightCmd_Stop();
    void MD_UprightAct_Stop();
    void MD_MotorTemp_Stop();
    void MD_CurrentCmd_Stop();
    void MD_CurrentAct_Stop();
    void MD_AbsoluteEncoder_Stop();
    void MD_IncrementalEncoder_Stop();
    void MD_AnkleAbsoluteEncoder_Stop();
    void MD_FSR_Stop();
    void MD_LP_Stop();
*/

    void displayFeedback(Object* object);
    QString getFloatData(const char* src, float data);
    QString getUint8Data(const char* src, uint8_t data);
    QString getUint16Data(const char* src, uint16_t data);
    QString getUint32Data(const char* src, uint32_t data);
    QString getInt8Data(const char* src, int8_t data);
    QString getInt16Data(const char* src, int16_t data);
    QString getInt32Data(const char* src, int32_t data);




private:
    Ui::MainWindow *ui;
    int selectedPortIndex;
    QSerialPort* m_serialPort=nullptr;
    PDOmsg* pdoMsg=nullptr;
    QTimer* timer;
    char DataToSend[100]={};
    uint8_t DataUint=0;
    QString DataQString;
    uint8_t OriID=0;//DataUint1=0;
    uint8_t DestID=0;//DataUint2=0;
    QString DataQString2;
    int index=0;
    //QString message;
    QString log;
    FileDisplayDialog displayDialog;

};
#endif // MAINWINDOW_H
