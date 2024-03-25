#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_serialPort(new QSerialPort(this))
{
    ui->setupUi(this);
    fillPortsInfo();
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::sendData);
    //connect(timer, &QTimer::timeout, this, &MainWindow::updateLineEdit);

    connect(m_serialPort, &QSerialPort::readyRead, this, &MainWindow::readData);
    connect(m_serialPort, &QSerialPort::errorOccurred, this, &MainWindow::handleError);
    connect(ui->comboBox_SerialPort, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::updateCurrentPort);

    // Timer to periodically update port information (every 1000 milliseconds in this example)
    QTimer* updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &MainWindow::updateAvailablePorts);
    updateTimer->start(1000); // Set the interval as needed

    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::on_pushButton_FileOpen_clicked);

    //Get Buttons
    connect(ui->pushButton_CMEMCYGet, &QPushButton::clicked, this, &MainWindow::CM_EMCY_GetState);//on_pushButton_CMEMCYGet_clicked);
    connect(ui->pushButton_CMPowerGet, &QPushButton::clicked, this, &MainWindow::CM_Power_GetState);
    connect(ui->pushButton_CMGuideLeftGet, &QPushButton::clicked, this, &MainWindow::CM_Guide_Left_GetState);
    connect(ui->pushButton_CMGuideRightGet, &QPushButton::clicked, this, &MainWindow::CM_Guide_Right_GetState);
    connect(ui->pushButton_CMStatusLEDLeftGet, &QPushButton::clicked, this, &MainWindow::CM_StatusLED_Left_GetColor);
    connect(ui->pushButton_CMStatusLEDRightGet, &QPushButton::clicked, this, &MainWindow::CM_StatusLED_Right_GetColor);
    connect(ui->pushButton_CMBatteryLEDGet, &QPushButton::clicked, this, &MainWindow::CM_BatteryLED_GetColor);


    connect(ui->pushButton_CMBattVoltGet, &QPushButton::clicked, this, &MainWindow::CM_BatteryVolt_Get);
    connect(ui->pushButton_CMBattPctgGet, &QPushButton::clicked, this, &MainWindow::CM_BatteryPctg_Get);
    connect(ui->pushButton_CMBoardTempGet, &QPushButton::clicked, this, &MainWindow::CM_BoardTemp_Get);


    connect(ui->pushButton_CMPelvisWidthLeftGet, &QPushButton::clicked, this, &MainWindow::CM_PelvisWidth_Left_Act_Get);
    connect(ui->pushButton_CMPelvisWidthRightGet, &QPushButton::clicked, this, &MainWindow::CM_PelvisWidth_Right_Act_Get);
    connect(ui->pushButton_CMPelvisDepthLeftGet, &QPushButton::clicked, this, &MainWindow::CM_PelvisDepth_Left_Act_Get);
    connect(ui->pushButton_CMPelvisDepthRightGet, &QPushButton::clicked, this, &MainWindow::CM_PelvisDepth_Right_Act_Get);

    //Set Buttons
    connect(ui->pushButton_CMPelvisWidthLeftCmdSet, &QPushButton::clicked, this, &MainWindow::CM_PelvisWidth_Left_Cmd_Set);
    connect(ui->pushButton_CMPelvisWidthLeftPos, &QPushButton::clicked, this, &MainWindow::CM_PelvisWidth_Left_BTPos_Set);
    connect(ui->pushButton_CMPelvisWidthLeftNeg, &QPushButton::clicked, this, &MainWindow::CM_PelvisWidth_Left_BTNeg_Set);

    connect(ui->pushButton_CMPelvisWidthRightCmdSet, &QPushButton::clicked, this, &MainWindow::CM_PelvisWidth_Right_Cmd_Set);
    connect(ui->pushButton_CMPelvisWidthRightPos, &QPushButton::clicked, this, &MainWindow::CM_PelvisWidth_Right_BTPos_Set);
    connect(ui->pushButton_CMPelvisWidthRightNeg, &QPushButton::clicked, this, &MainWindow::CM_PelvisWidth_Right_BTNeg_Set);

    connect(ui->pushButton_CMPelvisDepthLeftCmdSet, &QPushButton::clicked, this, &MainWindow::CM_PelvisDepth_Left_Cmd_Set);
    connect(ui->pushButton_CMPelvisDepthLeftPos, &QPushButton::clicked, this, &MainWindow::CM_PelvisDepth_Left_BTPos_Set);
    connect(ui->pushButton_CMPelvisDepthLeftNeg, &QPushButton::clicked, this, &MainWindow::CM_PelvisDepth_Left_BTNeg_Set);

    connect(ui->pushButton_CMPelvisDepthRightCmdSet, &QPushButton::clicked, this, &MainWindow::CM_PelvisDepth_Right_Cmd_Set);
    connect(ui->pushButton_CMPelvisDepthRightPos, &QPushButton::clicked, this, &MainWindow::CM_PelvisDepth_Right_BTPos_Set);
    connect(ui->pushButton_CMPelvisDepthRightNeg, &QPushButton::clicked, this, &MainWindow::CM_PelvisDepth_Right_BTNeg_Set);


    //Stop Buttons
    connect(ui->pushButton_CMEMCYStop, &QPushButton::clicked, this, &MainWindow::stopSending);//on_pushButton_CMEMCYGet_clicked);
    connect(ui->pushButton_CMPowerStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_CMGuideLeftStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_CMGuideRightStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_CMStatusLEDLeftStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_CMStatusLEDRightStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_CMBatteryLEDStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_CMBattVoltStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_CMBattPctgStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_CMBoardTempStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_CMPelvisWidthLeftStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_CMPelvisWidthRightStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_CMPelvisDepthLeftStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_CMPelvisDepthRightStop, &QPushButton::clicked, this, &MainWindow::stopSending);


    //MD
    //Get
    connect(ui->pushButton_MDIDGet, &QPushButton::clicked, this, &MainWindow::MD_ID_Get);
    connect(ui->pushButton_MDStatusLEDGet, &QPushButton::clicked, this, &MainWindow::MD_StatusLED_GetColor);
    connect(ui->pushButton_MDIMUGet, &QPushButton::clicked, this, &MainWindow::MD_IMU_Get);
    connect(ui->pushButton_MDBattVoltGet, &QPushButton::clicked, this, &MainWindow::MD_BatteryVoltage_Get);
    connect(ui->pushButton_MDBoardTempGet, &QPushButton::clicked, this, &MainWindow::MD_BoardTemp_Get);
    connect(ui->pushButton_MDUprightCmdGet, &QPushButton::clicked, this, &MainWindow::MD_UprightCmd_Get);
    connect(ui->pushButton_MDUprightActGet, &QPushButton::clicked, this, &MainWindow::MD_UprightAct_Get);
    connect(ui->pushButton_MDMotorTempGet, &QPushButton::clicked, this, &MainWindow::MD_MotorTemp_Get);
    connect(ui->pushButton_MDCurrentCmdGet, &QPushButton::clicked, this, &MainWindow::MD_CurrentCmd_Get);
    connect(ui->pushButton_MDCurrentActGet, &QPushButton::clicked, this, &MainWindow::MD_CurrentAct_Get);
    connect(ui->pushButton_MDAbsEncGet, &QPushButton::clicked, this, &MainWindow::MD_AbsoluteEncoder_Get);
    connect(ui->pushButton_MDIncEncGet, &QPushButton::clicked, this, &MainWindow::MD_IncrementalEncoder_Get);
    connect(ui->pushButton_MDAnkleAbsEncGet, &QPushButton::clicked, this, &MainWindow::MD_AnkleAbsoluteEncoder_Get);
    connect(ui->pushButton_MDFSRGet, &QPushButton::clicked, this, &MainWindow::MD_FSR_Get);
    connect(ui->pushButton_MDLPGet, &QPushButton::clicked, this, &MainWindow::MD_LP_Get);

    //Set
    connect(ui->pushButton_MDIDSet, &QPushButton::clicked, this, &MainWindow::MD_ID_Set);
    connect(ui->pushButton_MDUprightCmdSet, &QPushButton::clicked, this, &MainWindow::MD_UprightCmd_Set);
    connect(ui->pushButton_MDUprightCmdPosSet, &QPushButton::clicked, this, &MainWindow::MD_UprightCmd_BTPos_Set);
    connect(ui->pushButton_MDUprightCmdNegSet, &QPushButton::clicked, this, &MainWindow::MD_UprightCmd_BTNeg_Set);
    connect(ui->pushButton_MDCurrentCmdSet, &QPushButton::clicked, this, &MainWindow::MD_CurrentCmd_Set);

    //Stop
    connect(ui->pushButton_MDIDStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_MDStatusLEDStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_MDIMUStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_MDBattVoltStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_MDBoardTempStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_MDUprightCmdStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_MDUprightActStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_MDMotorTempStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_MDCurrentCmdStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_MDCurrentActStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_MDAbsEncStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_MDIncEncStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_MDAnkleAbsEncStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_MDFSRStop, &QPushButton::clicked, this, &MainWindow::stopSending);
    connect(ui->pushButton_MDLPStop, &QPushButton::clicked, this, &MainWindow::stopSending);

    // setGeometry(300, 300, 400, 300); // Set initial geometry
    // setWindowTitle("Centered Window");


    centerWindow(this->width(),this->height()); // Center the window on startup

}

MainWindow::~MainWindow()
{
    delete ui;
}



void MainWindow::centerWindow(int width, int height) {
    QScreen *screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->geometry();
    resize(screenGeometry.width(),screenGeometry.height());
    int x = (screenGeometry.width() - width) / 2;
    int y = (screenGeometry.height() - height) / 2;// -40;
    move(x, y);
}


//Serial Port
void MainWindow::updateAvailablePorts()
{
    fillPortsInfo();
}

void MainWindow::fillPortsInfo(){
    QString currentSelection;
    if (selectedPortIndex >= 0 && selectedPortIndex < ui->comboBox_SerialPort->count()) {
        currentSelection = ui->comboBox_SerialPort->itemText(selectedPortIndex);
    }
    ui->comboBox_SerialPort->clear();
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        ui->comboBox_SerialPort->addItem(info.portName());
    }
    // Try to reselect the previously selected port
    if (!currentSelection.isEmpty()) {
        int newIndex = ui->comboBox_SerialPort->findText(currentSelection);
        if (newIndex >= 0) {
            ui->comboBox_SerialPort->setCurrentIndex(newIndex);
            selectedPortIndex = newIndex;
        }
    }
}

void MainWindow::updateCurrentPort(int index)
{
    selectedPortIndex = index;
    // You can perform actions when the selected port changes, if needed
}

void MainWindow::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        QMessageBox::critical(this, tr("Critical Error"), m_serialPort->errorString());
        qDebug()<<"Error!\n";
        on_pushButton_SerialDisconnect_clicked();
    }
}

void MainWindow::on_pushButton_SerialConnect_clicked()
{
    qDebug()<<"Connect button clicked\n";

    //uint8_t tx_buff[]={0,1,2,3,4,5,6,7,8,9};
    m_serialPort->setPortName(ui->comboBox_SerialPort->currentText()); // define port name
    m_serialPort->setBaudRate(QSerialPort::Baud115200); // baud: nb of signal elements, ex) if have 2bit in 1 baud, send 2bit during 1 baud
    m_serialPort->setDataBits(QSerialPort::Data8); // dataBits
    m_serialPort->setParity(QSerialPort::NoParity); // check if error during data transmission
    m_serialPort->setStopBits(QSerialPort::OneStop); // if set or success before opening port, returns true
    m_serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (m_serialPort->open(QIODevice::ReadWrite)) { //tasks to do after opening serial device
        QMessageBox::information(this,tr("Info"),"Connected!");
        QString selectedPortName = ui->comboBox_SerialPort->currentText();
        qDebug() << "Connecting to " << selectedPortName;
        selectedPortIndex = ui->comboBox_SerialPort->currentIndex();
        ui->pushButton_SerialConnect->setEnabled(false);
        ui->pushButton_SerialDisconnect->setEnabled(true);
        //m_serialPort->write("123456789");
        //m_serialPort->flush();
    }
    else {
        QMessageBox::critical(this, tr("Error"), m_serialPort->errorString());
    }
}

void MainWindow::on_pushButton_SerialDisconnect_clicked()
{
    if (m_serialPort->isOpen()) // if serial port open
    {
        m_serialPort->close(); // close serial
    }
    QMessageBox::information(this,tr("Info"),"Disconnected!");
    qDebug()<<"Disconnected\n";
    ui->pushButton_SerialConnect->setEnabled(true);
    ui->pushButton_SerialDisconnect->setEnabled(false);
}

void MainWindow::resetParams(){
    memset(&DataToSend, 0, sizeof(DataToSend));
    memset(&DataUint, 0, sizeof(DataUint));
    memset(&OriID, 0, sizeof(OriID));
    memset(&DestID, 0, sizeof(DestID));
    index=0;
    log=nullptr;
}

//GUI
void MainWindow::updateLineEdit(QString message, QLineEdit *lineedit)
{
    lineedit->setText(message);
    //lineedit->displayText();
}

void MainWindow::on_pushButton_FileOpen_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(nullptr, "Open File", QDir::homePath(), "Text Files (*.txt);;All Files (*)");
    if (!fileName.isEmpty()){ displayDialog.displayFileContents(fileName); }
}

//Data management
void MainWindow::on_pushButton_SDOSaveSettings_clicked()
{
    //Save File
    QString filename= QFileDialog::getSaveFileName(this, "Save As");
    if (filename.isEmpty()){ return;}
    QFile file(filename);
    //Open the file
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)){ return;}
    QTextStream out(&file);

    QDateTime date = QDateTime::currentDateTime();
    QString formattedTime = date.toString("yyyy.MM.dd hh:mm:ss");
    QByteArray formattedTimeMsg = formattedTime.toLocal8Bit();

    // protocol: get settings with filename
    out << filename <<"\n"; //mode name

    out << "Date: "<< formattedTime << "\n";
    out << "SDO settings saved:" <<"\n";

    out << "Fnc Code: "<< ui->lineEdit_SDOFncCode->text() << "\n";
    out << "Ori ID: "<< ui->lineEdit_SDOOriID->text() << "\n";
    out << "Dest ID: "<< ui->lineEdit_SDODestID->text() << "\n";
    out << "Nb of SDO: "<< ui->lineEdit_SDONb->text() << "\n";
    out << "DOD ID: "<< ui->lineEdit_SDODODID->text() << "\n";
    out << "SDO ID: "<< ui->lineEdit_SDOID->text() << "\n";
    out << "Status: "<< ui->lineEdit_SDOStatus->text() << "\n";
    out << "Size of Data: "<< ui->lineEdit_SDOSizeofData->text() << "\n";
    out << "Data1: "<< ui->lineEdit_SDOData->text() << "\n";
    out << "Data2: "<< ui->lineEdit_SDOData2->text() << "\n";

    file.close();
}

void MainWindow::on_pushButton_PDOSaveSettings_clicked()
{
    //Save File
    QString filename= QFileDialog::getSaveFileName(this, "Save As");
    if (filename.isEmpty()){ return;}
    QFile file(filename);
    //Open the file
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)){ return;}
    QTextStream out(&file);

    QDateTime date = QDateTime::currentDateTime();
    QString formattedTime = date.toString("yyyy.MM.dd hh:mm:ss");
    QByteArray formattedTimeMsg = formattedTime.toLocal8Bit();

    // protocol: get settings with filename
    out << filename <<"\n"; //mode name

    out << "Date: "<< formattedTime << "\n";
    out << "PDO settings saved:" <<"\n";

    out << "Fnc Code: "<< ui->lineEdit_PDOFncCode->text() << "\n";
    out << "Ori ID: "<< ui->lineEdit_PDOOriID->text() << "\n";
    out << "Dest ID: "<< ui->lineEdit_PDODestID->text() << "\n";
    out << "Nb of PDO: "<< ui->lineEdit_PDONb->text() << "\n";
    out << "DOD ID: "<< ui->lineEdit_PDODODID->text() << "\n";
    out << "PDO ID: "<< ui->lineEdit_PDOID->text() << "\n";
    out << "Data: "<< ui->lineEdit_PDOData->text() << "\n";

    file.close();
}


//Send/Receive
void MainWindow::readData(){
    QString imessage=nullptr;
    uint8_t fnc_code, ori_node, nb_obj, task_id, dod_id, obj_id = 0;
    float fData=0;
    uint8_t u8Data=0;
    uint16_t u16Data=0;
    uint32_t u32Data=0;
    int8_t i8Data=0;
    int16_t i16Data=0;
    int32_t i32Data=0;

    Object object;
    //QByteArray
    const QByteArray data=m_serialPort->readAll();
    qDebug()<<"Received QByteArray:"<<data;
    int qsize=data.size();
    qDebug()<<"Size QByteArray:"<<qsize;

    //Const Char*
    const char* pData = data.data();
    int psize=sizeof(pData);
    qDebug()<<"Received char data:";
    for (int i = 0; i < psize; ++i) {
        qDebug() << i << ":"<< static_cast<uint8_t>(pData[i]);
    }

    //FOR MD://///////////////////////////////////////////////////////////////////////////////////////////////////
    if(qsize>11){//HOW TO DIFFERENTIATE TO THINK. this is only for 1 pdo or sdo
        qDebug()<<"Received from MD:";
        object.origin=NODE_ID_MD;

        /* Ver1.13.2, MD */
        //Char[half]
        int hsize=qsize/2;
        char hData[hsize];
        //Uint8_t[half]
        uint8_t uData[hsize];

        qDebug()<<"Half data:";
         for (int j=0;j<hsize;j++){
            hData[j]=pData[2*j];
            uData[j]=static_cast<uint8_t>(hData[j]);
            qDebug()<< j <<":"<<uData[j];
         }


         memcpy(&fnc_code, &uData[0], 1);
         qDebug()<<"Received fnc_code:"<< fnc_code;

         memcpy(&ori_node, &uData[1], 1);
         qDebug()<<"Received from ori_node:"<< ori_node;

         memcpy(&nb_obj, &uData[2], 1);
         qDebug()<<"Received nb of object:"<< nb_obj;

         memcpy(&dod_id, &uData[3], 1);
         qDebug()<<"Received dod_id:"<< dod_id;

         memcpy(&obj_id, &uData[4], 1);
         qDebug()<<"Received obj_id:"<< obj_id;

         object.fnc_code=(uint16_t)fnc_code << 8;

         object.ori_node=ori_node;
         //object.origin=origin;
         object.nb_obj=nb_obj;
         object.dod_id=dod_id;
         object.obj_id=obj_id;

         //    //void*? auto?

         object.fmessage=getFloatData((const char*)uData,fData);
         object.u8message=getUint8Data((const char*)uData,u8Data);
         object.u16message=getUint16Data((const char*)uData,u16Data);
         object.u32message=getUint32Data((const char*)uData,u32Data);
         object.i8message=getUint8Data((const char*)uData,i8Data);
         object.i16message=getUint16Data((const char*)uData,i16Data);
         object.i32message=getUint32Data((const char*)uData,i32Data);


    }

    //FOR CM://///////////////////////////////////////////////////////////////////////////////////////////////////        memcpy(&fnc_code, &pData[0], 1);
    else{
        qDebug()<<"Received from CM:";
        object.origin=NODE_ID_CM;

        memcpy(&fnc_code, &pData[0], 1);
        qDebug()<<"Received fnc_code:"<< fnc_code;

        memcpy(&ori_node, &pData[1], 1);
        qDebug()<<"Received from ori_node:"<< ori_node;

        memcpy(&nb_obj, &pData[2], 1);
        qDebug()<<"Received nb of object:"<< nb_obj;

        memcpy(&dod_id, &pData[3], 1);
        qDebug()<<"Received dod_id:"<< dod_id;

        memcpy(&obj_id, &pData[4], 1);
        qDebug()<<"Received obj_id:"<< obj_id;

        object.fnc_code=(uint16_t)fnc_code << 8;

        object.ori_node=ori_node;
        object.nb_obj=nb_obj;
        object.dod_id=dod_id;
        object.obj_id=obj_id;

        //    //void*? auto?

        object.fmessage=getFloatData(pData,fData);
        object.u8message=getUint8Data(pData,u8Data);
        object.u16message=getUint16Data(pData,u16Data);
        object.u32message=getUint32Data(pData,u32Data);
        object.i8message=getUint8Data(pData,i8Data);
        object.i16message=getUint16Data(pData,i16Data);
        object.i32message=getUint32Data(pData,i32Data);
    }


/*        if(ori_node==NODE_ID_ALL){
            object.origin=NODE_ID_CM;
        }
        else{
            object.origin=NODE_ID_MD;
    }

*/

/*
    if(object.origin==NODE_ID_CM)//ori_node == 0)//NODE_ID_ALL)//NODE_ID_CM||NODE_ID_ALL)
    {
        switch(object.fnc_code){
        case PDO:
            if(object.dod_id == DOD_ID_SYSTEM_CTRL){
                qDebug()<<"Case DOD_ID_SYSTEM_CTRL:";
                switch(object.obj_id){

                case PDO_ID_SYSTEM_VOLT:
                    object.message=getFloatData(pData,fData);
                    qDebug()<<"PDO_ID_SYSTEM_VOLT:"<< object.message;
                    updateLineEdit(object.message, ui->lineEdit_CMBattVolt);
                    break;

                case PDO_ID_SYSTEM_TEMP:
                    object.message=getFloatData(pData,fData);
                    qDebug()<<"PDO_ID_SYSTEM_TEMP:"<< object.message;
                    updateLineEdit(object.message, ui->lineEdit_CMBoardTemp);
                    break;

                case PDO_ID_SYSTEM_PCTG:
                    object.message=getFloatData(pData,fData);
                    qDebug()<<"PDO_ID_SYSTEM_PCTG:"<< object.message;
                    updateLineEdit(object.message, ui->lineEdit_CMBattPctg);
                    break;

                default: break;
                }
            }

            else if(object.dod_id == DOD_ID_EXT_DEV_CTRL){
                qDebug()<<"Case DOD_ID_EXT_DEV_CTRL:";

                switch(object.obj_id){

                case PDO_ID_EXTDEV_PELVIC_LW_LENGTH_REF:
                    object.message=getFloatData(pData,fData);
                    qDebug()<<"PDO_ID_EXTDEV_PELVIC_LW_LENGTH_REF:"<< object.message;
                    updateLineEdit(object.message, ui->lineEdit_CMPelvisWidthLeftCmd);
                    break;

                case PDO_ID_EXTDEV_PELVIC_LD_LENGTH_REF:
                    object.message=getFloatData(pData,fData);
                    qDebug()<<"PDO_ID_EXTDEV_PELVIC_LD_LENGTH_REF:"<< object.message;
                    updateLineEdit(object.message, ui->lineEdit_CMPelvisDepthLeftCmd);
                    break;

                case PDO_ID_EXTDEV_PELVIC_RW_LENGTH_REF:
                    object.message=getFloatData(pData,fData);
                    qDebug()<<"PDO_ID_EXTDEV_PELVIC_RW_LENGTH_REF:"<< object.message;
                    updateLineEdit(object.message, ui->lineEdit_CMPelvisWidthRightCmd);
                    break;

                case PDO_ID_EXTDEV_PELVIC_RD_LENGTH_REF:
                    object.message=getFloatData(pData,fData);
                    qDebug()<<"PDO_ID_EXTDEV_PELVIC_RD_LENGTH_REF:"<< object.message;
                    updateLineEdit(object.message, ui->lineEdit_CMPelvisDepthRightCmd);
                    break;

                case PDO_ID_EXTDEV_PELVIC_LW_LENGTH_ACT:
                    object.message=getFloatData(pData,fData);
                    qDebug()<<"PDO_ID_EXTDEV_PELVIC_LW_LENGTH_ACT:"<< object.message;
                    updateLineEdit(object.message, ui->lineEdit_CMPelvisWidthLeftAct);
                    break;

                case PDO_ID_EXTDEV_PELVIC_LD_LENGTH_ACT:
                    object.message=getFloatData(pData,fData);
                    qDebug()<<"PDO_ID_EXTDEV_PELVIC_LW_LENGTH_ACT:"<< object.message;
                    updateLineEdit(object.message, ui->lineEdit_CMPelvisDepthLeftAct);
                    break;

                case PDO_ID_EXTDEV_PELVIC_RW_LENGTH_ACT:
                    object.message=getFloatData(pData,fData);
                    qDebug()<<"PDO_ID_EXTDEV_PELVIC_LW_LENGTH_ACT:"<< object.message;
                    updateLineEdit(object.message, ui->lineEdit_CMPelvisWidthRightAct);
                    break;

                case PDO_ID_EXTDEV_PELVIC_RD_LENGTH_ACT:
                    object.message=getFloatData(pData,fData);
                    qDebug()<<"PDO_ID_EXTDEV_PELVIC_LW_LENGTH_ACT:"<< object.message;
                    updateLineEdit(object.message, ui->lineEdit_CMPelvisDepthRightAct);
                    break;

              case PDO_ID_EXTDEV_GUIDE_SW_LEFT_STATE:
                    object.message=getUint8Data(pData,uintdata);
                    qDebug()<<"PDO_ID_EXTDEV_GUIDE_SW_LEFT_STATE:";
                    updateLineEdit(object.message, ui->lineEdit_CMGuideLeftState);
                    break;

                case PDO_ID_EXTDEV_GUIDE_SW_RIGHT_STATE:
                    object.message=getUint8Data(pData,uintdata);
                    qDebug()<<"PDO_ID_EXTDEV_GUIDE_SW_RIGHT_STATE:"<< object.message;
                    updateLineEdit(object.message, ui->lineEdit_CMGuideRightState);
                    break;

                case PDO_ID_EXTDEV_EMR_SW_STATE:
                    object.message=getUint8Data(pData,uintdata);
                    qDebug()<<"PDO_ID_EXTDEV_EMR_SW_STATE:"<< object.message;
                    updateLineEdit(object.message, ui->lineEdit_CMEMCYState);
                    break;

                default: break;
                }
            }
/*                PDO_ID_EXTDEV_PELVIC_LW_DIRECTION_CMD,
                PDO_ID_EXTDEV_PELVIC_LD_DIRECTION_CMD,
                PDO_ID_EXTDEV_PELVIC_RW_DIRECTION_CMD,
                PDO_ID_EXTDEV_PELVIC_RD_DIRECTION_CMD,

                PDO_ID_EXTDEV_PELVIC_LW_DIRECTION_ACT,
                PDO_ID_EXTDEV_PELVIC_LD_DIRECTION_ACT,
                PDO_ID_EXTDEV_PELVIC_RW_DIRECTION_ACT,
                PDO_ID_EXTDEV_PELVIC_RD_DIRECTION_ACT,

                PDO_ID_EXTDEV_PELVIC_LW_BUTTON_STATE,//TO CHECK
                PDO_ID_EXTDEV_PELVIC_LD_BUTTON_STATE,
                PDO_ID_EXTDEV_PELVIC_RW_BUTTON_STATE,
                PDO_ID_EXTDEV_PELVIC_RD_BUTTON_STATE,


  */

/*
            else if(object.dod_id == DOD_ID_WHOLE_BODY_CTRL){
                qDebug()<<"Case DOD_ID_WHOLE_BODY_CTRL:";
                switch(object.obj_id){

                case PDO_ID_WHOLEBODY_LH_CURRENT_REF:     qDebug()<<"PDO_ID_WHOLEBODY_LH_CURRENT_REF:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_RH_CURRENT_REF:     qDebug()<<"PDO_ID_WHOLEBODY_RH_CURRENT_REF:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_LK_CURRENT_REF:     qDebug()<<"PDO_ID_WHOLEBODY_LK_CURRENT_REF:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_RK_CURRENT_REF:     qDebug()<<"PDO_ID_WHOLEBODY_RK_CURRENT_REF:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_LH_CURRENT_ACT:     qDebug()<<"PDO_ID_WHOLEBODY_LH_CURRENT_ACT:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_RH_CURRENT_ACT:     qDebug()<<"PDO_ID_WHOLEBODY_RH_CURRENT_ACT:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_LK_CURRENT_ACT:     qDebug()<<"PDO_ID_WHOLEBODY_LK_CURRENT_ACT:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_RK_CURRENT_ACT:     qDebug()<<"PDO_ID_WHOLEBODY_RK_CURRENT_ACT:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_LH_ABS_ENCODER_ACT: qDebug()<<"PDO_ID_WHOLEBODY_LH_ABS_ENCODER_ACT:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_RH_ABS_ENCODER_ACT: qDebug()<<"PDO_ID_WHOLEBODY_RH_ABS_ENCODER_ACT:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_LK_ABS_ENCODER_ACT: qDebug()<<"PDO_ID_WHOLEBODY_LK_ABS_ENCODER_ACT:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_RK_ABS_ENCODER_ACT: qDebug()<<"PDO_ID_WHOLEBODY_RK_ABS_ENCODER_ACT:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_LH_INC_ENCODER_CNT: qDebug()<<"PDO_ID_WHOLEBODY_LH_INC_ENCODER_CNT:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_RH_INC_ENCODER_CNT: qDebug()<<"PDO_ID_WHOLEBODY_RH_INC_ENCODER_CNT:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_LK_INC_ENCODER_CNT: qDebug()<<"PDO_ID_WHOLEBODY_LK_INC_ENCODER_CNT:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_RK_INC_ENCODER_CNT: qDebug()<<"PDO_ID_WHOLEBODY_RK_INC_ENCODER_CNT:"<< object.message;  break;

                case PDO_ID_WHOLEBODY_LH_MD_VOLTAGE:      qDebug()<<"PDO_ID_WHOLEBODY_LH_MD_VOLTAGE:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_RH_MD_VOLTAGE:      qDebug()<<"PDO_ID_WHOLEBODY_RH_MD_VOLTAGE:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_LK_MD_VOLTAGE:      qDebug()<<"PDO_ID_WHOLEBODY_LK_MD_VOLTAGE:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_RK_MD_VOLTAGE:      qDebug()<<"PDO_ID_WHOLEBODY_RK_MD_VOLTAGE:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_LH_MD_CURRENT:      qDebug()<<"PDO_ID_WHOLEBODY_LH_MD_CURRENT:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_RH_MD_CURRENT:      qDebug()<<"PDO_ID_WHOLEBODY_RH_MD_CURRENT:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_RH_LENGTH_ACT:      qDebug()<<"PDO_ID_WHOLEBODY_RH_LENGTH_ACT:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_LH_LENGTH_ACT:      qDebug()<<"PDO_ID_WHOLEBODY_LH_LENGTH_ACT:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_RK_LENGTH_ACT:      qDebug()<<"PDO_ID_WHOLEBODY_RK_LENGTH_ACT:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_LK_LENGTH_ACT:      qDebug()<<"PDO_ID_WHOLEBODY_LK_LENGTH_ACT:"<< object.message;     break;

                case PDO_ID_WHOLEBODY_DEPTH_LENGTH_ACT:   qDebug()<<"PDO_ID_WHOLEBODY_DEPTH_LENGTH_ACT:"<< object.message;  break;
                case PDO_ID_WHOLEBODY_WIDTH_LENGTH_ACT:   qDebug()<<"PDO_ID_WHOLEBODY_WIDTH_LENGTH_ACT:"<< object.message;  break;

                case PDO_ID_WHOLEBODY_LH_ERROR_CODE:      qDebug()<<"PDO_ID_WHOLEBODY_LH_ERROR_CODE:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_RH_ERROR_CODE:      qDebug()<<"PDO_ID_WHOLEBODY_RH_ERROR_CODE:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_LK_ERROR_CODE:      qDebug()<<"PDO_ID_WHOLEBODY_LK_ERROR_CODE:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_RK_ERROR_CODE:      qDebug()<<"PDO_ID_WHOLEBODY_RK_ERROR_CODE:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_LH_MOTOR_TEMP:      qDebug()<<"PDO_ID_WHOLEBODY_LH_MOTOR_TEMP:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_RH_MOTOR_TEMP:      qDebug()<<"PDO_ID_WHOLEBODY_RH_MOTOR_TEMP:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_LK_MOTOR_TEMP:      qDebug()<<"PDO_ID_WHOLEBODY_LK_MOTOR_TEMP:"<< object.message;     break;
                case PDO_ID_WHOLEBODY_RK_MOTOR_TEMP:      qDebug()<<"PDO_ID_WHOLEBODY_RK_MOTOR_TEMP:"<< object.message;     break;

                default: break;
                }
            }

        case SDO:
            //    updateLineEdit(object.message, ui->lineEdit_SDOFeedback);
            break;
        default:
            break;
        }

    }
    else if(object.origin==NODE_ID_MD){//else{
    /*else if(ori_node == NODE_ID_RH_TRA)//4 //NODE_ID_MD)//MD
    {*/
  /*      object.task_id = object.dod_id;

        switch(object.fnc_code){
        case PDO:
            if(object.task_id==TASK_ID_LOWLEVEL){
                qDebug()<<"Case TASK_ID_LOWLEVEL:";
                switch(object.obj_id){

                case PDO_ID_LOWLEVEL_3PHASES_CURRENT_RAW:           qDebug()<<"PDO_ID_LOWLEVEL_3PHASES_CURRENT_RAW:"<< object.message; updateLineEdit(object.message, ui->lineEdit_MDCurrentAct); break;
                case PDO_ID_LOWLEVEL_3PHASES_CURRENT_KF:            qDebug()<<"PDO_ID_LOWLEVEL_3PHASES_CURRENT_KF:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_3PHASES_VOLTAGE_RAW:           qDebug()<<"PDO_ID_LOWLEVEL_3PHASES_VOLTAGE_RAW:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_3PHASES_VOLTAGE_KF:            qDebug()<<"PDO_ID_LOWLEVEL_3PHASES_VOLTAGE_KF:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_POSITION:                      qDebug()<<"PDO_ID_LOWLEVEL_POSITION:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_VELOCITY:                      qDebug()<<"PDO_ID_LOWLEVEL_VELOCITY:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_CLARKE_OUT:                    qDebug()<<"PDO_ID_LOWLEVEL_CLARKE_OUT:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_PARK_OUT:                      qDebug()<<"PDO_ID_LOWLEVEL_PARK_OUT:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_VOLTAGE_IN:                    qDebug()<<"PDO_ID_LOWLEVEL_VOLTAGE_IN:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_ELEC_ANGLE:                    qDebug()<<"PDO_ID_LOWLEVEL_ELEC_ANGLE:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_PRBS_DATA:                     qDebug()<<"PDO_ID_LOWLEVEL_PRBS_DATA:"<< object.message;  break;

                case PDO_ID_LOWLEVEL_TOTAL_CURRENT_INPUT:           qDebug()<<"PDO_ID_LOWLEVEL_TOTAL_CURRENT_INPUT:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_CURRENT_OUTPUT:                qDebug()<<"PDO_ID_LOWLEVEL_CURRENT_OUTPUT:"<< object.message;  break;

                case PDO_ID_LOWLEVEL_AUXILIARY_INPUT:               qDebug()<<"PDO_ID_LOWLEVEL_AUXILIARY_INPUT:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_F_VECTOR_INPUT:                qDebug()<<"PDO_ID_LOWLEVEL_F_VECTOR_INPUT:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_LOW_ID_PROCESS_INPUT:          qDebug()<<"PDO_ID_LOWLEVEL_LOW_ID_PROCESS_INPUT:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_FRICTION_COMPENSATOR_INPUT:    qDebug()<<"PDO_ID_LOWLEVEL_FRICTION_COMPENSATOR_INPUT:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_MID_ID_PROCESS_INPUT:          qDebug()<<"PDO_ID_LOWLEVEL_MID_ID_PROCESS_INPUT:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_IRC_INPUT:                     qDebug()<<"PDO_ID_LOWLEVEL_IRC_INPUT:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_MID_CTRL_INPUT:                qDebug()<<"PDO_ID_LOWLEVEL_MID_CTRL_INPUT:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_ANALYZER_INPUT:                qDebug()<<"PDO_ID_LOWLEVEL_ANALYZER_INPUT:"<< object.message;  break;

                case PDO_ID_LOWLEVEL_COMMUTATION_STEP:              qDebug()<<"PDO_ID_LOWLEVEL_COMMUTATION_STEP:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_FRICTION_ID_REF:               qDebug()<<"PDO_ID_LOWLEVEL_FRICTION_ID_REF:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_HALL_SENSOR_SIG:               qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_SIG:"<< object.message;  break;
                case PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:             qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< object.message;  break;

                default: break;
                }


            }
            else if(object.task_id==TASK_ID_MIDLEVEL){
                qDebug()<<"Case TASK_ID_MIDLEVEL:";

                switch(object.obj_id){
                case PDO_ID_MIDLEVEL_LOOP_CNT:                      qDebug()<<"PDO_ID_MIDLEVEL_LOOP_CNT:"<< object.message;                   break;
                case PDO_ID_MIDLEVEL_REF_POSITION:                object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_REF_POSITION:"<< object.message;               break;
                case PDO_ID_MIDLEVEL_REF_VELOCITY:                object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_REF_VELOCITY:"<< object.message;               break;
                case PDO_ID_MIDLEVEL_ACTUAL_POSITION:             object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_ACTUAL_POSITION:"<< object.message;            break;
                case PDO_ID_MIDLEVEL_ACTUAL_VELOCITY_RAW:         object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_ACTUAL_VELOCITY_RAW:"<< object.message;        break;
                case PDO_ID_MIDLEVEL_SYSTEM_ID_SBS_FREQ:          object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_SYSTEM_ID_SBS_FREQ:"<< object.message;         break;
                case PDO_ID_MIDLEVEL_IMP_CTRL_INPUT:              object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_IMP_CTRL_INPUT:"<< object.message;             break;
                case PDO_ID_MIDLEVEL_POS_PID_CTRL_INPUT:          object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_POS_PID_CTRL_INPUT:"<< object.message;         break;
                case PDO_ID_MIDLEVEL_VEL_PID_CTRL_INPUT:          object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_VEL_PID_CTRL_INPUT:"<< object.message;         break;
                case PDO_ID_MIDLEVEL_VSD_INPUT:                   object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_VSD_INPUT:"<< object.message;                  break;
                case PDO_ID_MIDLEVEL_UNIT_TRAJECTORY_BUFF_COUNT:    qDebug()<<"PDO_ID_MIDLEVEL_UNIT_TRAJECTORY_BUFF_COUNT:"<< object.message; break;
                case PDO_ID_MIDLEVEL_F_VECTOR_INPUT:              object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_F_VECTOR_INPUT:"<< object.message;             break;
                case PDO_ID_MIDLEVEL_ABSENCODER1_POSITION:        object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_ABSENCODER1_POSITION:"<< object.message;       break;
                case PDO_ID_MIDLEVEL_ABSENCODER2_POSITION:        object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_ABSENCODER2_POSITION:"<< object.message;       break;
                case PDO_ID_MIDLEVEL_DOB_DISTURABNCE:             object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_DOB_DISTURABNCE:"<< object.message;            break;
                case PDO_ID_MIDLEVEL_DOB_INPUT:                   object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_DOB_INPUT:"<< object.message;                  break;
                case PDO_ID_MIDLEVEL_FF_INPUT:                    object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_FF_INPUT:"<< object.message;                   break;
                case PDO_ID_MIDLEVEL_VELOCITY_ESTIMATED:          object.message=getFloatData(pData,fData);  qDebug()<<"PDO_ID_MIDLEVEL_VELOCITY_ESTIMATED:"<< object.message;         break;
                case PDO_ID_MIDLEVEL_IMP_EPSILON:                   qDebug()<<"PDO_ID_MIDLEVEL_IMP_EPSILON:"<< object.message;                break;
                case PDO_ID_MIDLEVEL_IMP_KP:                        qDebug()<<"PDO_ID_MIDLEVEL_IMP_KP:"<< object.message;                     break;
                case PDO_ID_MIDLEVEL_IMP_KD:                        qDebug()<<"PDO_ID_MIDLEVEL_IMP_KD:"<< object.message;                     break;
                case PDO_ID_MIDLEVEL_IMP_LAMDA:                     qDebug()<<"PDO_ID_MIDLEVEL_IMP_LAMDA:"<< object.message;                  break;

                default: break;
                }



            }
            else if(object.task_id==TASK_ID_MSG){
                qDebug()<<"Case TASK_ID_MSG:";

                switch(object.obj_id){

                case PDO_ID_MSG_TEST1:  object.message=getFloatData(pData,fData); qDebug()<<"PDO_ID_MSG_TEST1:"<< object.message;    updateLineEdit(object.message, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST2:  object.message=getFloatData(pData,fData); qDebug()<<"PDO_ID_MSG_TEST2:"<< object.message;    updateLineEdit(object.message, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST3:  object.message=getFloatData(pData,fData); qDebug()<<"PDO_ID_MSG_TEST3:"<< object.message;    updateLineEdit(object.message, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST4:  object.message=getFloatData(pData,fData); qDebug()<<"PDO_ID_MSG_TEST4:"<< object.message;    updateLineEdit(object.message, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST5:  object.message=getFloatData(pData,fData); qDebug()<<"PDO_ID_MSG_TEST5:"<< object.message;    updateLineEdit(object.message, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST6:  object.message=getFloatData(pData,fData); qDebug()<<"PDO_ID_MSG_TEST6:"<< object.message;    updateLineEdit(object.message, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST7:  object.message=getFloatData(pData,fData); qDebug()<<"PDO_ID_MSG_TEST7:"<< object.message;    updateLineEdit(object.message, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST8:  object.message=getFloatData(pData,fData); qDebug()<<"PDO_ID_MSG_TEST8:"<< object.message;    updateLineEdit(object.message, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST9:  object.message=getFloatData(pData,fData); qDebug()<<"PDO_ID_MSG_TEST9:"<< object.message;    updateLineEdit(object.message, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST10: object.message=getFloatData(pData,fData); qDebug()<<"PDO_ID_MSG_TEST10:"<< object.message;   updateLineEdit(object.message, ui->lineEdit_BTMsgTest); break;

                default: break;
                }

            }
            else if(object.task_id==TASK_ID_WIDM){
                qDebug()<<"Case TASK_ID_WIDM:";


            }
            else if(object.task_id==TASK_ID_SYSMNGT){
                qDebug()<<"Case TASK_ID_SYSMNGT:";
                //pdo_id_sysmngt=(PDOIDSysMngt)obj_id;
                switch(object.obj_id){//pdo_id_sysmngt){
                case PDO_ID_SYSTEM_VOLT: qDebug()<<"Case PDO_ID_SYSTEM_VOLT:"<< object.message; updateLineEdit(object.message,ui->lineEdit_MDBattVolt); break;
                case PDO_ID_SYSTEM_CURR: qDebug()<<"Case PDO_ID_SYSTEM_CURR:"<< object.message; //updateLineEdit(object.message,ui->lineEdit_MDBattVolt); break;
                case PDO_ID_SYSTEM_TEMP: qDebug()<<"Case PDO_ID_SYSTEM_TEMP:"<< object.message; updateLineEdit(object.message,ui->lineEdit_MDBoardTemp); break;

                default: break;
                }

            }
            else if(object.task_id==TASK_ID_EXTDEV){
                qDebug()<<"Case TASK_ID_EXTDEV:";

                switch(object.obj_id){
                case PDO_ID_EXTDEV_FSR:                 qDebug()<<"Case PDO_ID_EXTDEV_FSR:"<< object.message;                 updateLineEdit(object.message, ui->lineEdit_MDFSR); break;//->lineEdit_BTFSR); break;
                case PDO_ID_EXTDEV_LP:                  qDebug()<<"Case PDO_ID_EXTDEV_LP:"<< object.message;                  updateLineEdit(object.message, ui->lineEdit_MDLP); break;//BTLP); break;
                case PDO_ID_EXTDEV_DC_LENGTH_REF:       qDebug()<<"Case PDO_ID_EXTDEV_DC_LENGTH_REF:"<< object.message;       updateLineEdit(object.message, ui->lineEdit_MDUprightCmd); break;//BTUprightCmd); break;
                case PDO_ID_EXTDEV_DC_DIRECTION_CMD:    qDebug()<<"Case PDO_ID_EXTDEV_DC_DIRECTION_CMD:"<< object.message;    updateLineEdit(object.message, ui->lineEdit_MDUprightCmd); break;//BTUprightCmd); break;
                case PDO_ID_EXTDEV_DC_LENGTH_ACT:       qDebug()<<"Case PDO_ID_EXTDEV_DC_LENGTH_ACT:"<< object.message;       updateLineEdit(object.message, ui->lineEdit_MDUprightAct); break;//BTUprightAct); break;
                case PDO_ID_EXTDEV_DC_DIRECTION_ACT:    qDebug()<<"Case PDO_ID_EXTDEV_DC_DIRECTION_ACT:"<< object.message;    updateLineEdit(object.message, ui->lineEdit_MDUprightAct); break;//BTUprightAct); break;
                case PDO_ID_EXTDEV_DC_BUTTON_STATE:     qDebug()<<"Case PDO_ID_EXTDEV_DC_BUTTON_STATE:"<< object.message;     updateLineEdit(object.message, ui->lineEdit_MDUprightAct); break;//BTUprightAct); break;
                case PDO_ID_EXTDEV_NTC_MOTOR_TEMP:      qDebug()<<"Case PDO_ID_EXTDEV_NTC_MOTOR_TEMP:"<< object.message;      updateLineEdit(object.message, ui->lineEdit_MDMotorTemp); break;//BTNTC); break;

                default: break;
                }
            }
    case SDO:
    //    updateLineEdit(object.message, ui->lineEdit_SDOFeedback);
    break;
    default: break;
        }
    }
*/

    displayFeedback(&object);

}

void MainWindow::sendData(){//const QByteArray &data){
    if(m_serialPort && m_serialPort->isOpen()){
        m_serialPort->write(DataToSend);
        m_serialPort->flush();
        //error handling
       // qDebug()<<"Send data \n";
    }
    else {
        QMessageBox::critical(this, tr("Error"), m_serialPort->errorString());
    }
}

void MainWindow::CreateObject(Object* obj)
{

}




void MainWindow::setPDOList(uint8_t device_id, uint8_t dod_id, uint8_t pdo_id){//uint16_t pdo_id){
    qDebug()<<"Set PDO List..\n";

    resetParams();

    //FNC Code
    DataUint = 2;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Node ID
    if(device_id==NODE_ID_CM){
        //Ori ID
        OriID = 0x01;
        //Dest ID
        DestID = 0x01;
        //Node ID
        DataUint= (OriID<<4)|(DestID);

        //DataUint=0x11;
    }
    else if(device_id==NODE_ID_MD){
        //Ori ID
        OriID = 0x01;
        //Dest ID
        DestID = 0x04;
        //Node ID
        DataUint= (OriID<<4)|(DestID);

        //DataUint=0x14;
    }

    qDebug()<<"Node ID:"<<QString::number(DataUint);
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Nb of SDO
    DataUint = 1;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //DOD ID
    if(device_id==NODE_ID_CM){
        DataUint = DOD_ID_DEV_COMM_HDLR;//4;
        memcpy(&DataToSend[index],&DataUint,1);
        index++;

        //SDO ID
        DataUint = SDO_ID_MSG_PDO_LIST;//SDO_ID_MSG_SET_PDO_LIST;//14;
        memcpy(&DataToSend[index],&DataUint,1);
        index++;
    }

    else if(device_id==NODE_ID_MD){
        DataUint = TASK_ID_MSG;//2;
        memcpy(&DataToSend[index],&DataUint,1);
        index++;

        //SDO ID
        DataUint = SDO_ID_MSG_PDO_LIST;//4;
        memcpy(&DataToSend[index],&DataUint,1);
        index++;
    }
    //Status
    DataUint = 1;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Size of Data
    DataUint = 1;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //DOD ID of desired PDO
    DataUint = dod_id;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //POD ID of desired PDO
    DataUint = pdo_id;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    for(int i=0;i<index;i++)
    {
        log+=QString(DataToSend[i]);
    }

    m_serialPort->write(DataToSend,index);
    m_serialPort->flush();
    qDebug()<<"Sent:"<<log;

    resetParams();
}

void MainWindow::sendPDONothing(uint8_t device_id){//consider generalizing this function with sendpdonothing case as certain params being 0
    qDebug()<<"Send PDO Nothing..\n";

    resetParams();

    //FNC Code
    DataUint = 3;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Node ID
    if(device_id==NODE_ID_CM){
        DataUint=0x11;
    }
    else if(device_id==NODE_ID_MD){
        DataUint=0x14;
    }
    qDebug()<<"Node ID:"<<QString::number(DataUint);
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    for(int i=0;i<index;i++)
    {
        log+=QString(DataToSend[i]);
    }


    /*m_serialPort->write(DataToSend,index);
    m_serialPort->flush();
    */
    startSending();
    qDebug()<<"Sent:"<<log;

    //resetParams();
}

void MainWindow::sendPDONothingOnce(uint8_t device_id){//consider generalizing this function with sendpdonothing case as certain params being 0
    qDebug()<<"Send PDO Nothing Once..\n";

    resetParams();

    //FNC Code
    DataUint = 3;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Node ID
    if(device_id==NODE_ID_CM){
        DataUint=0x11;
    }
    else if(device_id==NODE_ID_MD){
        DataUint=0x14;
    }
    qDebug()<<"Node ID:"<<QString::number(DataUint);
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    for(int i=0;i<index;i++)
    {
        log+=QString(DataToSend[i]);
    }


    m_serialPort->write(DataToSend,index);
    m_serialPort->flush();

    qDebug()<<"Sent:"<<log;

    //resetParams();
}

void MainWindow::on_pushButton_SDOSend_clicked()
{
    qDebug()<<"Sending SDO..\n";
    resetParams();

    //FNC Code
    DataQString = ui->lineEdit_SDOFncCode->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Ori ID
    DataQString = ui->lineEdit_SDOOriID->text();
    OriID = DataQString.toUInt();
    //Dest ID
    DataQString = ui->lineEdit_SDODestID->text();
    DestID = DataQString.toUInt();
    //Node ID
    DataUint= (OriID<<4)|(DestID);
    memcpy(&DataToSend[index],&DataUint,1);
    index++;
    qDebug()<<"Node ID:"<<QString::number(DataUint);

    //Nb of SDO
    DataQString = ui->lineEdit_SDONb->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;


    //DOD ID
    DataQString = ui->lineEdit_SDODODID->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //SDO ID
    DataQString = ui->lineEdit_SDOID->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;


    //Status
    DataQString = ui->lineEdit_SDOStatus->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;


    //Size of Data
    DataQString = ui->lineEdit_SDOSizeofData->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;


    //Data
    DataQString = ui->lineEdit_SDOData->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Data
    DataQString = ui->lineEdit_SDOData2->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    for(int i=0;i<index;i++)
    {
        qDebug()<<QString(DataToSend[i])<<";";
    }

    m_serialPort->write(DataToSend,index);
    m_serialPort->flush();

    resetParams();
}

void MainWindow::on_pushButton_PDOSendOnce_clicked()
{
    qDebug()<<"Sending PDO once..\n";

    resetParams();

    //FNC Code
    DataQString = ui->lineEdit_PDOFncCode->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Ori ID
    DataQString = ui->lineEdit_SDOOriID->text();
    OriID = DataQString.toUInt();
    //Dest ID
    DataQString = ui->lineEdit_SDODestID->text();
    DestID = DataQString.toUInt();
    //Node ID
    DataUint= (OriID<<4)|(DestID);
    memcpy(&DataToSend[index],&DataUint,1);
    index++;
    qDebug()<<"Node ID:"<<QString::number(DataUint);

    //Nb of PDO
    DataQString = ui->lineEdit_PDONb->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //DOD ID
    DataQString = ui->lineEdit_PDODODID->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO ID
    DataQString = ui->lineEdit_PDOID->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO Data
    DataQString = ui->lineEdit_PDOData->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    for(int i=0;i<11;i++){
        qDebug()<<QString(DataToSend[i])<<";";
    }

  m_serialPort->write(DataToSend,index);
  m_serialPort->flush();

    resetParams();
}

void MainWindow::on_pushButton_PDOStartSending_clicked()
{
    qDebug()<<"Start sending PDO..\n";

    resetParams();

    //FNC Code
    DataQString = ui->lineEdit_PDOFncCode->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Ori ID
    DataQString = ui->lineEdit_SDOOriID->text();
    OriID = DataQString.toUInt();
    //Dest ID
    DataQString = ui->lineEdit_SDODestID->text();
    DestID = DataQString.toUInt();
    //Node ID
    DataUint= (OriID<<4)|(DestID);
    memcpy(&DataToSend[index],&DataUint,1);
    index++;
    qDebug()<<"Node ID:"<<QString::number(DataUint);


    //Nb of PDO
    DataQString = ui->lineEdit_PDONb->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //DOD ID
    DataQString = ui->lineEdit_PDODODID->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO ID
    DataQString = ui->lineEdit_PDOID->text();
    DataUint = DataQString.toUInt();
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    startSending();

  /*  memset(&DataToSend, 0, sizeof(DataToSend));
    memset(&DataUint, 0, sizeof(DataUint));
    memset(&DataUint1, 0, sizeof(DataUint1));
    memset(&DataUint2, 0, sizeof(DataUint2));
    index=0;*/
}

void MainWindow::on_pushButton_PDOStopSending_clicked()
{
    stopSending();
}

void MainWindow::startSending(){
    if(!timer->isActive()) {
        timer->start(10); // 10 milliseconds interval
    }
}

void MainWindow::stopSending(){
    if (timer->isActive()) {
        timer->stop();
    }
}

void MainWindow::sendSDO(Object* obj)
{
    qDebug()<<"Sending SDO..\n";
    resetParams();

    //FNC Code
    memcpy(&DataToSend[index],&obj->fnc_code,1);
    index++;

    //Ori ID
    OriID = obj->ori_node;
    //Dest ID
    DestID = obj->dest_node;
    //Node ID
    DataUint= (OriID<<4)|(DestID);
    memcpy(&DataToSend[index],&DataUint,1);
    index++;
    qDebug()<<"Node ID:"<<QString::number(DataUint);


    //Nb of SDO
    memcpy(&DataToSend[index],&obj->nb_obj,1);
    index++;

    //DOD ID
    memcpy(&DataToSend[index],&obj->dod_id,1);
    index++;

    //SDO ID
    memcpy(&DataToSend[index],&obj->obj_id,1);
    index++;

    //Status
    memcpy(&DataToSend[index],&obj->status,1);
    index++;

    //Size of Data
    memcpy(&DataToSend[index],&obj->size,1);
    index++;

    //Desired DOD ID
    memcpy(&DataToSend[index],&obj->desired_dod_id,1);
    index++;

    //Desired PDO ID
    memcpy(&DataToSend[index],&obj->desired_obj_id,1);
    index++;

    for(int i=0;i<index;i++)
    {
        qDebug()<<QString(DataToSend[i])<<";";
    }

    m_serialPort->write(DataToSend,index);
    m_serialPort->flush();

    resetParams();


}

void MainWindow::sendPDO(Object* obj)
{
    qDebug()<<"Sending PDO..\n";
    resetParams();

    //FNC Code
    memcpy(&DataToSend[index],&obj->fnc_code,1);
    index++;

    //Ori ID
    OriID = obj->ori_node;
    //Dest ID
    DestID = obj->dest_node;
    //Node ID
    DataUint= (OriID<<4)|(DestID);
    memcpy(&DataToSend[index],&DataUint,1);
    index++;
    qDebug()<<"Node ID:"<<QString::number(DataUint);


    //Nb of PDO
    memcpy(&DataToSend[index],&obj->nb_obj,1);
    index++;

    //DOD ID
    memcpy(&DataToSend[index],&obj->dod_id,1);
    index++;

    //PDO ID
    memcpy(&DataToSend[index],&obj->obj_id,1);
    index++;

    //Data
    memcpy(&DataToSend[index],&obj->data,4);//to check
    index++;

    startSending();

  /*  memset(&DataToSend, 0, sizeof(DataToSend));
    memset(&DataUint, 0, sizeof(DataUint));
    memset(&DataUint1, 0, sizeof(DataUint1));
    memset(&DataUint2, 0, sizeof(DataUint2));
    index=0;*/
}





//BT
void MainWindow::on_pushButton_BTLEDtest_clicked()
{
    qDebug()<<"LED Test..\n";

    resetParams();

    //FNC Code
    DataUint = 2;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Ori ID
    OriID = 0x01;
    //Dest ID
    DestID = 0x04;
    //Node ID
    DataUint= 0x14;//(DataUint1<<8)|(DataUint2);
    qDebug()<<"node id:"<<QString::number(DataUint);
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Nb of SDO
    DataUint = 1;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;


    //DOD ID
    DataUint = 1;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //SDO ID
    DataUint = 64;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Status
    DataUint = 1;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Size of Data
    DataUint = 1;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;



    for(int i=0;i<index;i++)
    {
        log+=QString(DataToSend[i]);
    }

    qDebug()<<"Sent:"<<log;
    m_serialPort->write(DataToSend,index);
    m_serialPort->flush();

    resetParams();
}

void MainWindow::on_pushButton_BTEndian_clicked()
{
    // Define a 32-bit integer with a known value
    uint32_t value = 0x12345678;

    // Interpret the value as an array of bytes
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&value);

    // Check the value of the first byte to determine endianness
    if (bytes[0] == 0x78) {
        qDebug() << "Little-endian system";
    } else if (bytes[3] == 0x78) {
        qDebug() << "Big-endian system";
    } else {
        qDebug() << "Unable to determine endianness";
    }

}

void MainWindow::on_pushButton_BTFloatTest_clicked()
{
    // Example: Original float value
    float originalFloat = 256.0;

    // Reverse the endianness using QtEndian
    float reversedFloat = qFromLittleEndian(originalFloat);
    float reversedFloat2 = qFromBigEndian(originalFloat);

    // Display the original and reversed float values
    qDebug() << "Original float: " << originalFloat;
    qDebug() << "Reversed float: " << reversedFloat;
    qDebug() << "Reversed float2: " << reversedFloat2;

}

int i=0;
void MainWindow::on_pushButton_BTMsgTest_clicked()
{
    qDebug()<<"Msg Test..\n";
    setPDOList(NODE_ID_MD, TASK_ID_MSG, PDO_ID_MSG_TEST2);
    sendPDONothing(NODE_ID_MD);
}

void MainWindow::on_pushButton_BTUprightAct_clicked()
{
    qDebug()<<"Upright Actual Length Test..\n";
    setPDOList(NODE_ID_MD, TASK_ID_EXTDEV,PDO_ID_EXTDEV_DC_LENGTH_ACT);
    sendPDONothing(NODE_ID_MD);
}

void MainWindow::on_pushButton_BTUprightCmd_clicked()
{
    qDebug()<<"Upright Command Length Test Test..\n";
    setPDOList(NODE_ID_MD, TASK_ID_EXTDEV,PDO_ID_EXTDEV_DC_LENGTH_REF);
    sendPDONothing(NODE_ID_MD);
}

void MainWindow::on_pushButton_BTNTC_clicked()
{
    qDebug()<<"NTC Test..\n";
    setPDOList(NODE_ID_MD, TASK_ID_EXTDEV,PDO_ID_EXTDEV_NTC_MOTOR_TEMP);
    sendPDONothing(NODE_ID_MD);
}

void MainWindow::on_pushButton_BTLTC2944_clicked()
{
    qDebug()<<"LTC2944 Test..\n";
    setPDOList(NODE_ID_MD,TASK_ID_SYSMNGT,PDO_ID_SYSTEM_TEMP);
    sendPDONothing(NODE_ID_MD);
}

void MainWindow::on_pushButton_BTFSR_clicked()
{
    qDebug()<<"FSR Test..\n";
    setPDOList(NODE_ID_MD,TASK_ID_EXTDEV,PDO_ID_EXTDEV_FSR);
    sendPDONothing(NODE_ID_MD);
}

void MainWindow::on_pushButton_BTLP_clicked()
{
    qDebug()<<"LP Test..\n";
    setPDOList(NODE_ID_MD,TASK_ID_EXTDEV,PDO_ID_EXTDEV_LP);
    sendPDONothing(NODE_ID_MD);
}

void MainWindow::on_pushButton_BTIncEnc_clicked()
{
    qDebug()<<"FSR Test..\n";
    setPDOList(NODE_ID_MD,TASK_ID_LOWLEVEL,PDO_ID_EXTDEV_FSR);
    sendPDONothing(NODE_ID_MD);
}

void MainWindow::on_pushButton_BTAbsEnc_clicked()
{
    qDebug()<<"Abs Encoder Test..\n";
    setPDOList(NODE_ID_MD,TASK_ID_MIDLEVEL,PDO_ID_MIDLEVEL_ABSENCODER1_POSITION);
    sendPDONothing(NODE_ID_MD);
}


////////////////////////////New GUI//////////////////////////////////////////
////////////////////////////CM///////////////////////////////////////////////
void MainWindow::CM_EMCY_GetState()
{
    qDebug()<<"Get CM EMCY State..\n";
    setPDOList(NODE_ID_CM,TASK_ID_EXTDEV,PDO_ID_EXTDEV_EMR_SW_STATE);
    sendPDONothing(NODE_ID_CM);
}

void MainWindow::CM_Power_GetState()
{
    qDebug()<<"Get CM Power State..\n";
    /*setPDOList(TASK_ID_EXTDEV,PDO_ID_EXTDEV_DC_LENGTH_ACT);
    sendPDONothing();*/
}

void MainWindow::CM_Guide_Left_GetState()
{
    qDebug()<<"Get CM Left Guide State..\n";
    setPDOList(NODE_ID_CM,TASK_ID_EXTDEV,PDO_ID_EXTDEV_GUIDE_SW_LEFT_STATE);
    sendPDONothing(NODE_ID_CM);
}

void MainWindow::CM_Guide_Right_GetState()
{
    qDebug()<<"Get CM Right Guide State..\n";
    setPDOList(NODE_ID_CM,TASK_ID_EXTDEV,PDO_ID_EXTDEV_GUIDE_SW_RIGHT_STATE);
    sendPDONothing(NODE_ID_CM);
}

void MainWindow::CM_StatusLED_Left_GetColor()
{
    qDebug()<<"Get CM Status LED Left Color..\n";
    /*setPDOList(TASK_ID_EXTDEV,PDO_ID_EXTDEV_DC_LENGTH_ACT);
    sendPDONothing();*/

}

void MainWindow::CM_StatusLED_Right_GetColor()
{
    qDebug()<<"Get CM Status LED Right Color..\n";
    /*setPDOList(TASK_ID_EXTDEV,PDO_ID_EXTDEV_DC_LENGTH_ACT);
    sendPDONothing();*/

}

void MainWindow::CM_BatteryLED_GetColor()
{
    qDebug()<<"Get CM Battery LED Color..\n";
    /*setPDOList(TASK_ID_EXTDEV,PDO_ID_EXTDEV_DC_LENGTH_ACT);
    sendPDONothing();*/

}

void MainWindow::CM_BatteryVolt_Get()
{
    qDebug()<<"Get CM Battery Voltage..\n";
    setPDOList(NODE_ID_CM, DOD_ID_SYSTEM_CTRL, PDO_ID_SYSTEM_VOLT);
    sendPDONothing(NODE_ID_CM);

}

void MainWindow::CM_BatteryPctg_Get()
{
    qDebug()<<"Get CM Battery Percentage..\n";
    setPDOList(NODE_ID_CM, DOD_ID_SYSTEM_CTRL, PDO_ID_SYSTEM_PCTG);
    sendPDONothing(NODE_ID_CM);

}

void MainWindow::CM_BoardTemp_Get()
{
    qDebug()<<"Get CM Board Temperature..\n";
    setPDOList(NODE_ID_CM, DOD_ID_SYSTEM_CTRL, PDO_ID_SYSTEM_TEMP);
    sendPDONothing(NODE_ID_CM);

}



void MainWindow::CM_PelvisWidth_Left_Act_Get()
{
    qDebug()<<"Get CM Pelvis Width Left Act..\n";
    setPDOList(NODE_ID_CM,TASK_ID_EXTDEV,PDO_ID_EXTDEV_PELVIC_LW_LENGTH_ACT); //?? add pdo in data object dict
    sendPDONothing(NODE_ID_CM);
}

void MainWindow::CM_PelvisWidth_Right_Act_Get()
{
    qDebug()<<"Get CM Pelvis Width Right Act..\n";
    setPDOList(NODE_ID_CM,TASK_ID_EXTDEV,PDO_ID_EXTDEV_PELVIC_RW_LENGTH_ACT); //?? add pdo in data object dict
    sendPDONothing(NODE_ID_CM);

}

void MainWindow::CM_PelvisDepth_Left_Act_Get()
{
    qDebug()<<"Get CM Pelvis Depth Left Act..\n";
    setPDOList(NODE_ID_CM,TASK_ID_EXTDEV,PDO_ID_EXTDEV_PELVIC_LD_LENGTH_ACT); //?? add pdo in data object dict
    sendPDONothing(NODE_ID_CM);
}

void MainWindow::CM_PelvisDepth_Right_Act_Get()
{
    qDebug()<<"Get CM Pelvis Depth Right Act..\n";
    setPDOList(NODE_ID_CM,TASK_ID_EXTDEV,PDO_ID_EXTDEV_PELVIC_RD_LENGTH_ACT); //?? add pdo in data object dict
    sendPDONothing(NODE_ID_CM);
}

//Set
void MainWindow::CM_PelvisWidth_Left_Cmd_Set()
{
    qDebug()<<"Set CM Pelvis Width Left Cmd..\n";

    resetParams();

    //FNC Code
    DataUint = 3;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Node ID : orientated node (origin)<<8 | destination node id (target)
    DataUint= 0x11;//qt id=?
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Nb of PDO
    DataUint = 1;//or do multiple pdos
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //DOD ID
    DataUint = TASK_ID_EXTDEV;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO ID
    DataUint = PDO_ID_EXTDEV_PELVIC_LW_LENGTH_REF;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO Data
    DataQString = ui->lineEdit_CMPelvisWidthLeftCmd->text();
    if(DataQString.isEmpty()){
        DataUint=0;
    }
    else{
        DataUint = DataQString.toUInt();
    }
    memcpy(&DataToSend[index],&DataUint,4);
    index++;

    for(int i=0;i<11;i++){
        qDebug()<<QString(DataToSend[i])<<";";
    }

    m_serialPort->write(DataToSend,index);
    m_serialPort->flush();
    resetParams();
}

void MainWindow::CM_PelvisWidth_Left_BTPos_Set()
{
/*    Object pdo;
    pdo.fnc_code=3;
    pdo.ori_node=0x01;
    pdo.origin=NODE_ID_CM;
    pdo.dest_node=0x01;
    //pdo.dod_id=DOD_ID_WHOL



    qDebug()<<"Set CM Pelvis Width Left BT Pos Cmd..\n";

    resetParams();

    //FNC Code
    DataUint = 3;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Node ID : orientated node (origin)<<8 | destination node id (target)
    DataUint= 0x11;//qt id=?
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Nb of PDO
    DataUint = 1;//or do multiple pdos
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //DOD ID
    DataUint = TASK_ID_EXTDEV;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO ID
    DataUint = PDO_ID_EXTDEV_PELVIC_RW_LENGTH_REF;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO Data
    DataQString = ui->lineEdit_CMPelvisWidthLeftCmd->text();
    if(DataQString.isEmpty()){
        DataUint=0;
    }
    else{
        DataUint = DataQString.toUInt();
    }
    memcpy(&DataToSend[index],&DataUint,4);
    index++;

    for(int i=0;i<11;i++){
        qDebug()<<QString(DataToSend[i])<<";";
    }

    m_serialPort->write(DataToSend,index);
    m_serialPort->flush();
    resetParams();
  */
}

void MainWindow::CM_PelvisWidth_Left_BTNeg_Set()
{

}

void MainWindow::CM_PelvisWidth_Right_Cmd_Set()
{
    qDebug()<<"Set CM Pelvis Width Right Cmd..\n";

    resetParams();

    //FNC Code
    DataUint = 3;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Node ID : orientated node (origin)<<8 | destination node id (target)
    DataUint= 0x11;//qt id=?
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Nb of PDO
    DataUint = 1;//or do multiple pdos
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //DOD ID
    DataUint = TASK_ID_EXTDEV;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO ID
    DataUint = PDO_ID_EXTDEV_PELVIC_RW_LENGTH_REF;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO Data
    DataQString = ui->lineEdit_CMPelvisWidthLeftCmd->text();
    if(DataQString.isEmpty()){
        DataUint=0;
    }
    else{
        DataUint = DataQString.toUInt();
    }
    memcpy(&DataToSend[index],&DataUint,4);
    index++;

    for(int i=0;i<11;i++){
        qDebug()<<QString(DataToSend[i])<<";";
    }

    m_serialPort->write(DataToSend,index);
    m_serialPort->flush();
    resetParams();
}

void MainWindow::CM_PelvisWidth_Right_BTPos_Set()
{

}

void MainWindow::CM_PelvisWidth_Right_BTNeg_Set()
{

}

void MainWindow::CM_PelvisDepth_Left_Cmd_Set()
{
    qDebug()<<"Set CM Pelvis Depth Left Cmd..\n";

    resetParams();

    //FNC Code
    DataUint = 3;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Node ID : orientated node (origin)<<8 | destination node id (target)
    DataUint= 0x11;//qt id=?
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Nb of PDO
    DataUint = 1;//or do multiple pdos
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //DOD ID
    DataUint = TASK_ID_EXTDEV;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO ID
    DataUint = PDO_ID_EXTDEV_PELVIC_LD_LENGTH_REF;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO Data
    DataQString = ui->lineEdit_CMPelvisWidthLeftCmd->text();
    if(DataQString.isEmpty()){
        DataUint=0;
    }
    else{
        DataUint = DataQString.toUInt();
    }
    memcpy(&DataToSend[index],&DataUint,4);
    index++;

    for(int i=0;i<11;i++){
        qDebug()<<QString(DataToSend[i])<<";";
    }

    m_serialPort->write(DataToSend,index);
    m_serialPort->flush();
    resetParams();
}

void MainWindow::CM_PelvisDepth_Left_BTPos_Set()
{
}

void MainWindow::CM_PelvisDepth_Left_BTNeg_Set()
{

}

void MainWindow::CM_PelvisDepth_Right_Cmd_Set()
{
    qDebug()<<"Set CM Pelvis Depth Right Cmd..\n";

    resetParams();

    //FNC Code
    DataUint = 3;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Node ID : orientated node (origin)<<8 | destination node id (target)
    DataUint= 0x11;//qt id=?
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Nb of PDO
    DataUint = 1;//or do multiple pdos
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //DOD ID
    DataUint = TASK_ID_EXTDEV;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO ID
    DataUint = PDO_ID_EXTDEV_PELVIC_RD_LENGTH_REF;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO Data
    DataQString = ui->lineEdit_CMPelvisWidthLeftCmd->text();
    if(DataQString.isEmpty()){
        DataUint=0;
    }
    else{
        DataUint = DataQString.toUInt();
    }
    memcpy(&DataToSend[index],&DataUint,4);
    index++;

    for(int i=0;i<11;i++){
        qDebug()<<QString(DataToSend[i])<<";";
    }

    m_serialPort->write(DataToSend,index);
    m_serialPort->flush();
    resetParams();
}

void MainWindow::CM_PelvisDepth_Right_BTPos_Set()
{

}

void MainWindow::CM_PelvisDepth_Right_BTNeg_Set()
{

}



///////////////////////////MD////////////////////////////////////////////////
//Get
void MainWindow::MD_ID_Get()
{
    qDebug()<<"Get MD node ID..\n";
    setPDOList(NODE_ID_MD,TASK_ID_MSG,PDO_ID_MSG_MDID);
    //sendPDONothingOnce(NODE_ID_MD);
    sendPDONothing(NODE_ID_MD);

}

void MainWindow::MD_StatusLED_GetColor()
{
    qDebug()<<"Get MD Status LED color..\n";


}

void MainWindow::MD_IMU_Get()
{
    qDebug()<<"Get MD IMU roll pitch yaw..\n";

}

void MainWindow::MD_BatteryVoltage_Get()
{
    //also temp
    qDebug()<<"Get MD Battery Voltage..\n";
    setPDOList(NODE_ID_MD,TASK_ID_SYSMNGT,PDO_ID_SYSTEM_VOLT); //PDO_ID_SYSTEM_TEMP
    sendPDONothing(NODE_ID_MD);
}

void MainWindow::MD_BoardTemp_Get()
{
    //also temp
    qDebug()<<"Get MD Board Temperature..\n";
    setPDOList(NODE_ID_MD,TASK_ID_SYSMNGT,PDO_ID_SYSTEM_TEMP); //PDO_ID_SYSTEM_TEMP
    sendPDONothing(NODE_ID_MD);

}


void MainWindow::MD_UprightCmd_Get()
{
    qDebug()<<"Get MD Command Upright Length..\n";
    setPDOList(NODE_ID_MD,TASK_ID_EXTDEV,PDO_ID_EXTDEV_DC_LENGTH_REF);
    sendPDONothing(NODE_ID_MD);
}

void MainWindow::MD_UprightAct_Get()
{
    qDebug()<<"Get MD Actual Upright Length..\n";
    setPDOList(NODE_ID_MD,TASK_ID_EXTDEV,PDO_ID_EXTDEV_DC_LENGTH_ACT);
    sendPDONothing(NODE_ID_MD);
}

void MainWindow::MD_MotorTemp_Get()
{
    qDebug()<<"Get MD Motor Temperature..\n";
    setPDOList(NODE_ID_MD,TASK_ID_EXTDEV,PDO_ID_EXTDEV_NTC_MOTOR_TEMP);
    sendPDONothing(NODE_ID_MD);
}

void MainWindow::MD_CurrentCmd_Get()
{
    qDebug()<<"Get MD Current Command..\n";

}

void MainWindow::MD_CurrentAct_Get()
{
    qDebug()<<"Get MD Current Actual Value..\n";
}

void MainWindow::MD_AbsoluteEncoder_Get()
{
    qDebug()<<"Get MD Absolute Encoder 1..\n";
    //crash?
    //setPDOList(NODE_ID_MD,TASK_ID_EXTDEV,PDO_ID_MIDLEVEL_ABSENCODER1_POSITION);
    //sendPDONothing(NODE_ID_MD);

}

void MainWindow::MD_IncrementalEncoder_Get()
{

}

void MainWindow::MD_AnkleAbsoluteEncoder_Get()
{
    qDebug()<<"Get MD Absolute Encoder 2..\n";
    //crash?
    //setPDOList(NODE_ID_MD,TASK_ID_EXTDEV,PDO_ID_MIDLEVEL_ABSENCODER2_POSITION);
    //sendPDONothing(NODE_ID_MD);
}

void MainWindow::MD_FSR_Get()
{
    qDebug()<<"Get MD FSR..\n";
    setPDOList(NODE_ID_MD,TASK_ID_EXTDEV,PDO_ID_EXTDEV_FSR);
    sendPDONothing(NODE_ID_MD);
}

void MainWindow::MD_LP_Get()
{
    qDebug()<<"Get MD Linear Potentiometer..\n";
    setPDOList(NODE_ID_MD,TASK_ID_EXTDEV,PDO_ID_EXTDEV_LP);
    sendPDONothing(NODE_ID_MD);
}

//Set
void MainWindow::MD_ID_Set()
{
    qDebug()<<"Set MD Node ID..\n";

}

void MainWindow::MD_UprightCmd_Set()
{
    qDebug()<<"Set MD Upright Cmd..\n";

    resetParams();

    //FNC Code
    DataUint = 3;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Node ID : orientated node (origin)<<8 | destination node id (target)
    DataUint= 0x14;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Nb of PDO
    DataUint = 1;//or do multiple pdos
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //DOD ID
    DataUint = TASK_ID_EXTDEV;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO ID
    DataUint = PDO_ID_EXTDEV_DC_LENGTH_REF;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO Data
    DataQString = ui->lineEdit_MDUprightCmd->text();
    if(DataQString.isEmpty()){
        DataUint=0;
    }
    else{
        DataUint = DataQString.toUInt();
    }
    memcpy(&DataToSend[index],&DataUint,4);
    index++;

    for(int i=0;i<11;i++){
        qDebug()<<QString(DataToSend[i])<<";";
    }

    m_serialPort->write(DataToSend,index);
    m_serialPort->flush();

    resetParams();
}

void MainWindow::MD_UprightCmd_BTPos_Set()
{
 /*   qDebug()<<"Set Upright Cmd..\n";

    resetParams();

    //FNC Code
    DataUint = 3;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Node ID : orientated node (origin)<<8 | destination node id (target)
    DataUint= 0x14;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //Nb of PDO
    DataUint = 1;//or do multiple pdos
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //DOD ID
    DataUint = TASK_ID_EXTDEV;
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO ID
    DataUint = PDO_ID_EXTDEV_DC_LENGTH_REF;//which one
    memcpy(&DataToSend[index],&DataUint,1);
    index++;

    //PDO Data
    DataQString = ui->lineEdit_MDUprightCmd->text();
    if(DataQString.isEmpty()){
        DataUint=0;
    }
    else{
        DataUint = DataQString.toUInt();
    }
    memcpy(&DataToSend[index],&DataUint,4);
    index++;

    for(int i=0;i<11;i++){
        qDebug()<<QString(DataToSend[i])<<";";
    }

    m_serialPort->write(DataToSend,index);
    m_serialPort->flush();

    resetParams();
    */
}

void MainWindow::MD_UprightCmd_BTNeg_Set()
{

}

void MainWindow::MD_CurrentCmd_Set()
{

}

//Stop
/*
void MainWindow::MD_ID_Stop()
{
    stopSending();
}

void MainWindow::MD_StatusLED_Stop()
{
    stopSending();
}

void MainWindow::MD_IMU_Stop()
{

}

void MainWindow::MD_BatteryVoltage_Stop()
{
    //also percentage
}


void MainWindow::MD_UprightCmd_Stop()
{

}


void MainWindow::MD_UprightAct_Stop()
{

}


void MainWindow::MD_MotorTemp_Stop()
{

}

void MainWindow::MD_CurrentCmd_Stop()
{

}



void MainWindow::MD_CurrentAct_Stop()
{

}


void MainWindow::MD_AbsoluteEncoder_Stop()
{

}

void MainWindow::MD_IncrementalEncoder_Stop()
{

}


void MainWindow::MD_AnkleAbsoluteEncoder_Stop()
{

}


void MainWindow::MD_FSR_Stop()
{

}

void MainWindow::MD_LP_Stop()
{

}
*/
/*
void MainWindow::on_pushButton_SDOFeedbackSave_clicked()
{
    //Save File
    QString filename= QFileDialog::getSaveFileName(this, "Save As");
    if (filename.isEmpty()){ return;}
    QFile file(filename);
    //Open the file
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)){ return;}
    QTextStream out(&file);

    QDateTime date = QDateTime::currentDateTime();
    QString formattedTime = date.toString("yyyy.MM.dd hh:mm:ss");
//    QByteArray formattedTimeMsg = formattedTime.toLocal8Bit();

    out << "Date: "<< formattedTime << "\n";
    out << "SDO Feedback saved:" <<"\n";
    out << ui->lineEdit_SDOFeedback->text() << "\n";
    file.close();
}
*/

/* void MainWindow::on_pushButton_PDOFeedbackSave_clicked()

 //already automatically saved with qDebug log in readData
    //Save File
    QString filename= QFileDialog::getSaveFileName(this, "Save As");
    if (filename.isEmpty()){ return;}
    QFile file(filename);
    //Open the file
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)){ return;}
    QTextStream out(&file);

    QDateTime date = QDateTime::currentDateTime();
    QString formattedTime = date.toString("yyyy.MM.dd hh:mm:ss");
//    QByteArray formattedTimeMsg = formattedTime.toLocal8Bit();

    out << "Date: "<< formattedTime << "\n";
    out << "PDO Feedback saved:" <<"\n";
    out << ui->lineEdit_PDOFeedback->text() << "\n";
    file.close();

}
*/

/*QString MainWindow::getEnumName(MyEnum value)
    {
        const QMetaObject* metaObject = this->metaObject();
        int enumIndex = metaObject->indexOfEnumerator("MyEnum");
        QMetaEnum metaEnum = metaObject->enumerator(enumIndex);

        return metaEnum.valueToKey(value);
    }

static QString getEnumName(EnumType value)
    {
        const QMetaObject* metaObject = QMetaEnum::fromType<EnumType>().metaObject();
        int enumIndex = metaObject->indexOfEnumerator(QMetaEnum::fromType<EnumType>().name());
        QMetaEnum metaEnum = metaObject->enumerator(enumIndex);

        return metaEnum.valueToKey(static_cast<int>(value));
    }
*/
/*
template <typename EnumType>
QString MainWindow::getEnumName(EnumType enumType, int index)
{
    std::string QtEnumToString (const QEnum value)
    {
      return std::string(QMetaEnum::fromType<QEnum>().valueToKey(value));
    }



    QMetaEnum metaEnum = QMetaEnum::fromType<EnumType>();
    qDebug() << metaEnum.valueToKey(ModelApple::Big);

    const QMetaObject* metaObject = QMetaEnum::fromType<EnumType>().metaObject();
    int enumIndex = metaObject->indexOfEnumerator(QMetaEnum::fromType<EnumType>().name());
    QMetaEnum metaEnum = metaObject->enumerator(enumIndex);

    return metaEnum.valueToKey(index);
}
*/


void MainWindow::displayFeedback(Object* obj){
    if(obj->origin==NODE_ID_CM)
    {
        switch(obj->fnc_code){
        case PDO:
            if(obj->dod_id == DOD_ID_SYSTEM_CTRL){
                qDebug()<<"Case DOD_ID_SYSTEM_CTRL:";
                switch(obj->obj_id){
                case PDO_ID_SYSTEM_VOLT: qDebug()<<"PDO_ID_SYSTEM_VOLT:"<< obj->fmessage; updateLineEdit(obj->fmessage, ui->lineEdit_CMBattVolt); break;
                case PDO_ID_SYSTEM_TEMP: qDebug()<<"PDO_ID_SYSTEM_TEMP:"<< obj->fmessage; updateLineEdit(obj->fmessage, ui->lineEdit_CMBoardTemp); break;
                case PDO_ID_SYSTEM_PCTG: qDebug()<<"PDO_ID_SYSTEM_PCTG:"<< obj->fmessage; updateLineEdit(obj->fmessage, ui->lineEdit_CMBattPctg); break;
                default: break;
                }
            }

            else if(obj->dod_id == DOD_ID_EXT_DEV_CTRL){
                qDebug()<<"Case DOD_ID_EXT_DEV_CTRL:";
                switch(obj->dod_id){
                case PDO_ID_EXTDEV_PELVIC_LW_LENGTH_REF: qDebug()<<"PDO_ID_EXTDEV_PELVIC_LW_LENGTH_REF:"<< obj->fmessage; updateLineEdit(obj->fmessage, ui->lineEdit_CMPelvisWidthLeftCmd); break;
                case PDO_ID_EXTDEV_PELVIC_LD_LENGTH_REF: qDebug()<<"PDO_ID_EXTDEV_PELVIC_LD_LENGTH_REF:"<< obj->fmessage; updateLineEdit(obj->fmessage, ui->lineEdit_CMPelvisDepthLeftCmd); break;
                case PDO_ID_EXTDEV_PELVIC_RW_LENGTH_REF: qDebug()<<"PDO_ID_EXTDEV_PELVIC_RW_LENGTH_REF:"<< obj->fmessage; updateLineEdit(obj->fmessage, ui->lineEdit_CMPelvisWidthRightCmd); break;
                case PDO_ID_EXTDEV_PELVIC_RD_LENGTH_REF: qDebug()<<"PDO_ID_EXTDEV_PELVIC_RD_LENGTH_REF:"<< obj->fmessage; updateLineEdit(obj->fmessage, ui->lineEdit_CMPelvisDepthRightCmd); break;


                case PDO_ID_EXTDEV_PELVIC_LW_LENGTH_ACT: qDebug()<<"PDO_ID_EXTDEV_PELVIC_LW_LENGTH_ACT:"<< obj->fmessage; updateLineEdit(obj->fmessage, ui->lineEdit_CMPelvisWidthLeftAct); break;
                case PDO_ID_EXTDEV_PELVIC_LD_LENGTH_ACT: qDebug()<<"PDO_ID_EXTDEV_PELVIC_LW_LENGTH_ACT:"<< obj->fmessage; updateLineEdit(obj->fmessage, ui->lineEdit_CMPelvisDepthLeftAct); break;
                case PDO_ID_EXTDEV_PELVIC_RW_LENGTH_ACT: qDebug()<<"PDO_ID_EXTDEV_PELVIC_LW_LENGTH_ACT:"<< obj->fmessage; updateLineEdit(obj->fmessage, ui->lineEdit_CMPelvisWidthRightAct); break;
                case PDO_ID_EXTDEV_PELVIC_RD_LENGTH_ACT: qDebug()<<"PDO_ID_EXTDEV_PELVIC_LW_LENGTH_ACT:"<< obj->fmessage; updateLineEdit(obj->fmessage, ui->lineEdit_CMPelvisDepthRightAct); break;



                case PDO_ID_EXTDEV_GUIDE_SW_LEFT_STATE: qDebug()<<"PDO_ID_EXTDEV_GUIDE_SW_LEFT_STATE:"<< obj->u8message; updateLineEdit(obj->u8message, ui->lineEdit_CMGuideLeftState); break;
                case PDO_ID_EXTDEV_GUIDE_SW_RIGHT_STATE: qDebug()<<"PDO_ID_EXTDEV_GUIDE_SW_RIGHT_STATE:"<< obj->u8message; updateLineEdit(obj->u8message, ui->lineEdit_CMGuideRightState); break;
                case PDO_ID_EXTDEV_EMR_SW_STATE: qDebug()<<"PDO_ID_EXTDEV_EMR_SW_STATE:"<< obj->u8message; updateLineEdit(obj->u8message, ui->lineEdit_CMEMCYState); break;

                default: break;
                }
            }
/*                PDO_ID_EXTDEV_PELVIC_LW_DIRECTION_CMD,
                PDO_ID_EXTDEV_PELVIC_LD_DIRECTION_CMD,
                PDO_ID_EXTDEV_PELVIC_RW_DIRECTION_CMD,
                PDO_ID_EXTDEV_PELVIC_RD_DIRECTION_CMD,

                PDO_ID_EXTDEV_PELVIC_LW_DIRECTION_ACT,
                PDO_ID_EXTDEV_PELVIC_LD_DIRECTION_ACT,
                PDO_ID_EXTDEV_PELVIC_RW_DIRECTION_ACT,
                PDO_ID_EXTDEV_PELVIC_RD_DIRECTION_ACT,

                PDO_ID_EXTDEV_PELVIC_LW_BUTTON_STATE,//TO CHECK
                PDO_ID_EXTDEV_PELVIC_LD_BUTTON_STATE,
                PDO_ID_EXTDEV_PELVIC_RW_BUTTON_STATE,
                PDO_ID_EXTDEV_PELVIC_RD_BUTTON_STATE,


  */


      /*      else if(obj->dod_id == DOD_ID_WHOLE_BODY_CTRL){
                qDebug()<<"Case DOD_ID_WHOLE_BODY_CTRL:";
                switch(obj->obj_id){
                
                case PDO_ID_WHOLEBODY_LH_CURRENT_REF:     qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_RH_CURRENT_REF:     qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_LK_CURRENT_REF:     qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_RK_CURRENT_REF:     qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_LH_CURRENT_ACT:     qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_RH_CURRENT_ACT:     qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_LK_CURRENT_ACT:     qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_RK_CURRENT_ACT:     qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_LH_ABS_ENCODER_ACT: qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_RH_ABS_ENCODER_ACT: qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_LK_ABS_ENCODER_ACT: qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_RK_ABS_ENCODER_ACT: qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_LH_INC_ENCODER_CNT: qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_RH_INC_ENCODER_CNT: qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_LK_INC_ENCODER_CNT: qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_RK_INC_ENCODER_CNT: qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->message;  break;
                
                case PDO_ID_WHOLEBODY_LH_MD_VOLTAGE:      qDebug()<<"PDO_ID_WHOLEBODY_LH_MD_VOLTAGE:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_RH_MD_VOLTAGE:      qDebug()<<"PDO_ID_WHOLEBODY_RH_MD_VOLTAGE:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_LK_MD_VOLTAGE:      qDebug()<<"PDO_ID_WHOLEBODY_LK_MD_VOLTAGE:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_RK_MD_VOLTAGE:      qDebug()<<"PDO_ID_WHOLEBODY_RK_MD_VOLTAGE:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_LH_MD_CURRENT:      qDebug()<<"PDO_ID_WHOLEBODY_LH_MD_CURRENT:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_RH_MD_CURRENT:      qDebug()<<"PDO_ID_WHOLEBODY_RH_MD_CURRENT:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_RH_LENGTH_ACT:      qDebug()<<"PDO_ID_WHOLEBODY_RH_LENGTH_ACT:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_LH_LENGTH_ACT:      qDebug()<<"PDO_ID_WHOLEBODY_LH_LENGTH_ACT:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_RK_LENGTH_ACT:      qDebug()<<"PDO_ID_WHOLEBODY_RK_LENGTH_ACT:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_LK_LENGTH_ACT:      qDebug()<<"PDO_ID_WHOLEBODY_LK_LENGTH_ACT:"<< obj->message;     break;
                
                case PDO_ID_WHOLEBODY_DEPTH_LENGTH_ACT:   qDebug()<<"PDO_ID_WHOLEBODY_DEPTH_LENGTH_ACT:"<< obj->message;  break;
                case PDO_ID_WHOLEBODY_WIDTH_LENGTH_ACT:   qDebug()<<"PDO_ID_WHOLEBODY_WIDTH_LENGTH_ACT:"<< obj->message;  break;

                case PDO_ID_WHOLEBODY_LH_ERROR_CODE:      qDebug()<<"PDO_ID_WHOLEBODY_LH_ERROR_CODE:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_RH_ERROR_CODE:      qDebug()<<"PDO_ID_WHOLEBODY_RH_ERROR_CODE:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_LK_ERROR_CODE:      qDebug()<<"PDO_ID_WHOLEBODY_LK_ERROR_CODE:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_RK_ERROR_CODE:      qDebug()<<"PDO_ID_WHOLEBODY_RK_ERROR_CODE:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_LH_MOTOR_TEMP:      qDebug()<<"PDO_ID_WHOLEBODY_LH_MOTOR_TEMP:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_RH_MOTOR_TEMP:      qDebug()<<"PDO_ID_WHOLEBODY_RH_MOTOR_TEMP:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_LK_MOTOR_TEMP:      qDebug()<<"PDO_ID_WHOLEBODY_LK_MOTOR_TEMP:"<< obj->message;     break;
                case PDO_ID_WHOLEBODY_RK_MOTOR_TEMP:      qDebug()<<"PDO_ID_WHOLEBODY_RK_MOTOR_TEMP:"<< obj->message;     break;

                default: break;
                }
            }
*/
        case SDO:
            //    updateLineEdit(obj->fmessage, ui->lineEdit_SDOFeedback);
            break;
        default:
            break;
        }

    }
    else if(obj->origin==NODE_ID_MD){//else{
    /*else if(ori_node == NODE_ID_RH_TRA)//4 //NODE_ID_MD)//MD
    {*/
        obj->task_id = obj->dod_id;

        switch(obj->fnc_code){
        case PDO:
            if(obj->task_id==TASK_ID_LOWLEVEL){
                qDebug()<<"Case TASK_ID_LOWLEVEL:";
                switch(obj->obj_id){

                case PDO_ID_LOWLEVEL_3PHASES_CURRENT_RAW:           qDebug()<<"PDO_ID_LOWLEVEL_3PHASES_CURRENT_RAW:"<< obj->i32message; updateLineEdit(obj->i32message, ui->lineEdit_MDCurrentAct); break;
                case PDO_ID_LOWLEVEL_3PHASES_CURRENT_KF:            qDebug()<<"PDO_ID_LOWLEVEL_3PHASES_CURRENT_KF:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_3PHASES_VOLTAGE_RAW:           qDebug()<<"PDO_ID_LOWLEVEL_3PHASES_VOLTAGE_RAW:"<< obj->i32message;  break;
                case PDO_ID_LOWLEVEL_3PHASES_VOLTAGE_KF:            qDebug()<<"PDO_ID_LOWLEVEL_3PHASES_VOLTAGE_KF:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_POSITION:                      qDebug()<<"PDO_ID_LOWLEVEL_POSITION:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_VELOCITY:                      qDebug()<<"PDO_ID_LOWLEVEL_VELOCITY:"<< obj->i32message;  break;
                case PDO_ID_LOWLEVEL_CLARKE_OUT:                    qDebug()<<"PDO_ID_LOWLEVEL_CLARKE_OUT:"<< obj->i32message;  break;
                case PDO_ID_LOWLEVEL_PARK_OUT:                      qDebug()<<"PDO_ID_LOWLEVEL_PARK_OUT:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_VOLTAGE_IN:                    qDebug()<<"PDO_ID_LOWLEVEL_VOLTAGE_IN:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_ELEC_ANGLE:                    qDebug()<<"PDO_ID_LOWLEVEL_ELEC_ANGLE:"<< obj->u16message;  break;
                case PDO_ID_LOWLEVEL_PRBS_DATA:                     qDebug()<<"PDO_ID_LOWLEVEL_PRBS_DATA:"<< obj->fmessage;  break;
    
                case PDO_ID_LOWLEVEL_TOTAL_CURRENT_INPUT:           qDebug()<<"PDO_ID_LOWLEVEL_TOTAL_CURRENT_INPUT:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_CURRENT_OUTPUT:                qDebug()<<"PDO_ID_LOWLEVEL_CURRENT_OUTPUT:"<< obj->fmessage;  break;
    
                case PDO_ID_LOWLEVEL_AUXILIARY_INPUT:               qDebug()<<"PDO_ID_LOWLEVEL_AUXILIARY_INPUT:"<< obj->fmessage;  break;
                //case PDO_ID_LOWLEVEL_F_VECTOR_INPUT:                qDebug()<<"PDO_ID_LOWLEVEL_F_VECTOR_INPUT:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_LOW_ID_PROCESS_INPUT:          qDebug()<<"PDO_ID_LOWLEVEL_LOW_ID_PROCESS_INPUT:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_FRICTION_COMPENSATOR_INPUT:    qDebug()<<"PDO_ID_LOWLEVEL_FRICTION_COMPENSATOR_INPUT:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_MID_ID_PROCESS_INPUT:          qDebug()<<"PDO_ID_LOWLEVEL_MID_ID_PROCESS_INPUT:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_IRC_INPUT:                     qDebug()<<"PDO_ID_LOWLEVEL_IRC_INPUT:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_MID_CTRL_INPUT:                qDebug()<<"PDO_ID_LOWLEVEL_MID_CTRL_INPUT:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_ANALYZER_INPUT:                qDebug()<<"PDO_ID_LOWLEVEL_ANALYZER_INPUT:"<< obj->fmessage;  break;

                case PDO_ID_LOWLEVEL_COMMUTATION_STEP:              qDebug()<<"PDO_ID_LOWLEVEL_COMMUTATION_STEP:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_FRICTION_ID_REF:               qDebug()<<"PDO_ID_LOWLEVEL_FRICTION_ID_REF:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_HALL_SENSOR_SIG:               qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_SIG:"<< obj->fmessage;  break;
                case PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:             qDebug()<<"PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC:"<< obj->fmessage;  break;

                default: break;
                }


            }
            else if(obj->task_id==TASK_ID_MIDLEVEL){
                qDebug()<<"Case TASK_ID_MIDLEVEL:";

                switch(obj->obj_id){
     /*           case PDO_ID_MIDLEVEL_LOOP_CNT:                      qDebug()<<"PDO_ID_MIDLEVEL_LOOP_CNT:"<< obj->message;                   break;
                case PDO_ID_MIDLEVEL_REF_POSITION:                  qDebug()<<"PDO_ID_MIDLEVEL_REF_POSITION:"<< obj->message;               break;
                case PDO_ID_MIDLEVEL_REF_VELOCITY:                  qDebug()<<"PDO_ID_MIDLEVEL_REF_VELOCITY:"<< obj->message;               break;
                case PDO_ID_MIDLEVEL_ACTUAL_POSITION:               qDebug()<<"PDO_ID_MIDLEVEL_ACTUAL_POSITION:"<< obj->message;            break;
                case PDO_ID_MIDLEVEL_ACTUAL_VELOCITY_RAW:           qDebug()<<"PDO_ID_MIDLEVEL_ACTUAL_VELOCITY_RAW:"<< obj->message;        break;
                case PDO_ID_MIDLEVEL_SYSTEM_ID_SBS_FREQ:            qDebug()<<"PDO_ID_MIDLEVEL_SYSTEM_ID_SBS_FREQ:"<< obj->message;         break;
                case PDO_ID_MIDLEVEL_IMP_CTRL_INPUT:                qDebug()<<"PDO_ID_MIDLEVEL_IMP_CTRL_INPUT:"<< obj->message;             break;
                case PDO_ID_MIDLEVEL_POS_PID_CTRL_INPUT:            qDebug()<<"PDO_ID_MIDLEVEL_POS_PID_CTRL_INPUT:"<< obj->message;         break;
                case PDO_ID_MIDLEVEL_VEL_PID_CTRL_INPUT:            qDebug()<<"PDO_ID_MIDLEVEL_VEL_PID_CTRL_INPUT:"<< obj->message;         break;
                case PDO_ID_MIDLEVEL_VSD_INPUT:                     qDebug()<<"PDO_ID_MIDLEVEL_VSD_INPUT:"<< obj->message;                  break;
                case PDO_ID_MIDLEVEL_UNIT_TRAJECTORY_BUFF_COUNT:    qDebug()<<"PDO_ID_MIDLEVEL_UNIT_TRAJECTORY_BUFF_COUNT:"<< obj->message; break;
                case PDO_ID_MIDLEVEL_F_VECTOR_INPUT:                qDebug()<<"PDO_ID_MIDLEVEL_F_VECTOR_INPUT:"<< obj->message;             break;
                case PDO_ID_MIDLEVEL_ABSENCODER1_POSITION:          qDebug()<<"PDO_ID_MIDLEVEL_ABSENCODER1_POSITION:"<< obj->message;       break;
                case PDO_ID_MIDLEVEL_ABSENCODER2_POSITION:          qDebug()<<"PDO_ID_MIDLEVEL_ABSENCODER2_POSITION:"<< obj->message;       break;
                case PDO_ID_MIDLEVEL_DOB_DISTURABNCE:               qDebug()<<"PDO_ID_MIDLEVEL_DOB_DISTURABNCE:"<< obj->message;            break;
                case PDO_ID_MIDLEVEL_DOB_INPUT:                     qDebug()<<"PDO_ID_MIDLEVEL_DOB_INPUT:"<< obj->message;                  break;
                case PDO_ID_MIDLEVEL_FF_INPUT:                      qDebug()<<"PDO_ID_MIDLEVEL_FF_INPUT:"<< obj->message;                   break;
                case PDO_ID_MIDLEVEL_VELOCITY_ESTIMATED:            qDebug()<<"PDO_ID_MIDLEVEL_VELOCITY_ESTIMATED:"<< obj->message;         break;
                case PDO_ID_MIDLEVEL_IMP_EPSILON:                   qDebug()<<"PDO_ID_MIDLEVEL_IMP_EPSILON:"<< obj->message;                break;
                case PDO_ID_MIDLEVEL_IMP_KP:                        qDebug()<<"PDO_ID_MIDLEVEL_IMP_KP:"<< obj->message;                     break;
                case PDO_ID_MIDLEVEL_IMP_KD:                        qDebug()<<"PDO_ID_MIDLEVEL_IMP_KD:"<< obj->message;                     break;
                case PDO_ID_MIDLEVEL_IMP_LAMDA:                     qDebug()<<"PDO_ID_MIDLEVEL_IMP_LAMDA:"<< obj->message;                  break;
       */
                default: break;
                }



            }
            else if(obj->task_id==TASK_ID_MSG){
                qDebug()<<"Case TASK_ID_MSG:";

                switch(obj->obj_id){

                case PDO_ID_MSG_TEST1: qDebug()<<"PDO_ID_MSG_TEST1:"<< obj->fmessage;    updateLineEdit(obj->fmessage, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST2: qDebug()<<"PDO_ID_MSG_TEST2:"<< obj->fmessage;    updateLineEdit(obj->fmessage, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST3: qDebug()<<"PDO_ID_MSG_TEST3:"<< obj->fmessage;    updateLineEdit(obj->fmessage, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST4: qDebug()<<"PDO_ID_MSG_TEST4:"<< obj->fmessage;    updateLineEdit(obj->fmessage, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST5: qDebug()<<"PDO_ID_MSG_TEST5:"<< obj->fmessage;    updateLineEdit(obj->fmessage, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST6: qDebug()<<"PDO_ID_MSG_TEST6:"<< obj->fmessage;    updateLineEdit(obj->fmessage, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST7: qDebug()<<"PDO_ID_MSG_TEST7:"<< obj->fmessage;    updateLineEdit(obj->fmessage, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST8: qDebug()<<"PDO_ID_MSG_TEST8:"<< obj->fmessage;    updateLineEdit(obj->fmessage, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST9: qDebug()<<"PDO_ID_MSG_TEST9:"<< obj->fmessage;    updateLineEdit(obj->fmessage, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_TEST10: qDebug()<<"PDO_ID_MSG_TEST10:"<< obj->fmessage;  updateLineEdit(obj->fmessage, ui->lineEdit_BTMsgTest); break;
                case PDO_ID_MSG_MDID: qDebug()<<"PDO_ID_MSG_MDID:"<< obj->u8message;  updateLineEdit(obj->u8message, ui->lineEdit_MDID); break;

                default: break;
                }

            }
            else if(obj->task_id==TASK_ID_WIDM){
                qDebug()<<"Case TASK_ID_WIDM:";


            }
            else if(obj->task_id==TASK_ID_SYSMNGT){
                qDebug()<<"Case TASK_ID_SYSMNGT:";
                //pdo_id_sysmngt=(PDOIDSysMngt)obj_id;
                switch(obj->obj_id){//pdo_id_sysmngt){
                case PDO_ID_SYSTEM_VOLT: qDebug()<<"Case PDO_ID_SYSTEM_VOLT:"<< obj->fmessage; updateLineEdit(obj->fmessage,ui->lineEdit_MDBattVolt); break;
                case PDO_ID_SYSTEM_CURR: qDebug()<<"Case PDO_ID_SYSTEM_CURR:"<< obj->fmessage; //updateLineEdit(obj->fmessage,ui->lineEdit_MDBattVolt); break;
                case PDO_ID_SYSTEM_TEMP: qDebug()<<"Case PDO_ID_SYSTEM_TEMP:"<< obj->fmessage; updateLineEdit(obj->fmessage,ui->lineEdit_MDBoardTemp); break;
                
                default: break;
                }

            }
            else if(obj->task_id==TASK_ID_EXTDEV){
                qDebug()<<"Case TASK_ID_EXTDEV:";

                switch(obj->obj_id){
                case PDO_ID_EXTDEV_FSR:                 qDebug()<<"Case PDO_ID_EXTDEV_FSR:"<< obj->fmessage;                 updateLineEdit(obj->fmessage, ui->lineEdit_MDFSR); break;//->lineEdit_BTFSR); break;
                case PDO_ID_EXTDEV_LP:                  qDebug()<<"Case PDO_ID_EXTDEV_LP:"<< obj->fmessage;                  updateLineEdit(obj->fmessage, ui->lineEdit_MDLP); break;//BTLP); break;
                case PDO_ID_EXTDEV_DC_LENGTH_REF:       qDebug()<<"Case PDO_ID_EXTDEV_DC_LENGTH_REF:"<< obj->fmessage;       updateLineEdit(obj->fmessage, ui->lineEdit_MDUprightCmd); break;//BTUprightCmd); break;
                case PDO_ID_EXTDEV_DC_DIRECTION_CMD:    qDebug()<<"Case PDO_ID_EXTDEV_DC_DIRECTION_CMD:"<< obj->u8message;    updateLineEdit(obj->fmessage, ui->lineEdit_MDUprightCmd); break;//BTUprightCmd); break;
                case PDO_ID_EXTDEV_DC_LENGTH_ACT:       qDebug()<<"Case PDO_ID_EXTDEV_DC_LENGTH_ACT:"<< obj->fmessage;       updateLineEdit(obj->fmessage, ui->lineEdit_MDUprightAct); break;//BTUprightAct); break;
                case PDO_ID_EXTDEV_DC_DIRECTION_ACT:    qDebug()<<"Case PDO_ID_EXTDEV_DC_DIRECTION_ACT:"<< obj->u8message;    updateLineEdit(obj->u8message, ui->lineEdit_MDUprightAct); break;//BTUprightAct); break;
                case PDO_ID_EXTDEV_DC_BUTTON_STATE:     qDebug()<<"Case PDO_ID_EXTDEV_DC_BUTTON_STATE:"<< obj->u8message;     updateLineEdit(obj->u8message, ui->lineEdit_MDUprightAct); break;//BTUprightAct); break;
                case PDO_ID_EXTDEV_NTC_MOTOR_TEMP:      qDebug()<<"Case PDO_ID_EXTDEV_NTC_MOTOR_TEMP:"<< obj->fmessage;      updateLineEdit(obj->fmessage, ui->lineEdit_MDMotorTemp); break;//BTNTC); break;
                
                default: break;
                }
            }
    case SDO:
    //    updateLineEdit(obj->message, ui->lineEdit_SDOFeedback);
    break;
    default: break;
        }
    }

}


QString MainWindow::getFloatData(const char* src, float data){
    //according to DATA TYPE
    int size=sizeof(src);
    qDebug()<<"Size:"<< size;
    QString imessage=nullptr;
    if(size >= 5 + 1){
    memcpy(&data, &src[5],4);
    qDebug()<<"Received Float Data:"<<data;

    imessage= QString::number(data);
    qDebug()<<"Message:"<<imessage;

    }
    else{
        qDebug()<<"Incomplete data! Check again.";
        imessage="nan";
        qDebug()<<"Message:"<<imessage;

    }
    return imessage;
}


QString MainWindow::getUint8Data(const char* src, uint8_t data){
    //according to DATA TYPE
    int size=sizeof(src);
    qDebug()<<"Size:"<< size;
    QString imessage=nullptr;
    if(size >= 5 + 1){
    memcpy(&data, &src[5],1);
    qDebug()<<"Received Uint8 Data:"<<data;

    imessage= QString::number(data);
    qDebug()<<"Message:"<<imessage;

    }
    else{
        qDebug()<<"Incomplete data! Check again.";
        imessage="nan";
        qDebug()<<"Message:"<<imessage;

    }
    return imessage;
}


QString MainWindow::getUint16Data(const char* src, uint16_t data){
    //according to DATA TYPE
    int size=sizeof(src);
    qDebug()<<"Size:"<< size;
    QString imessage=nullptr;
    if(size >= 5 + 1){
    memcpy(&data, &src[5],2);
    qDebug()<<"Received Uint16 Data:"<<data;

    imessage= QString::number(data);
    qDebug()<<"Message:"<<imessage;

    }
    else{
        qDebug()<<"Incomplete data! Check again.";
        imessage="nan";
        qDebug()<<"Message:"<<imessage;

    }
    return imessage;
}

QString MainWindow::getUint32Data(const char* src, uint32_t data){
    //according to DATA TYPE
    int size=sizeof(src);
    qDebug()<<"Size:"<< size;
    QString imessage=nullptr;
    if(size >= 5 + 1){
    memcpy(&data, &src[5],4);
    qDebug()<<"Received Uint32 Data:"<<data;

    imessage= QString::number(data);
    qDebug()<<"Message:"<<imessage;

    }
    else{
        qDebug()<<"Incomplete data! Check again.";
        imessage="nan";
        qDebug()<<"Message:"<<imessage;

    }
    return imessage;
}

QString MainWindow::getInt8Data(const char* src, int8_t data){
    //according to DATA TYPE
    int size=sizeof(src);
    qDebug()<<"Size:"<< size;
    QString imessage=nullptr;
    if(size >= 5 + 1){
    memcpy(&data, &src[5],1);
    qDebug()<<"Received Int8 Data:"<<data;

    imessage= QString::number(data);
    qDebug()<<"Message:"<<imessage;

    }
    else{
        qDebug()<<"Incomplete data! Check again.";
        imessage="nan";
        qDebug()<<"Message:"<<imessage;

    }
    return imessage;
}


QString MainWindow::getInt16Data(const char* src, int16_t data){
    //according to DATA TYPE
    int size=sizeof(src);
    qDebug()<<"Size:"<< size;
    QString imessage=nullptr;
    if(size >= 5 + 1){
    memcpy(&data, &src[5],2);
    qDebug()<<"Received Uint16 Data:"<<data;

    imessage= QString::number(data);
    qDebug()<<"Message:"<<imessage;

    }
    else{
        qDebug()<<"Incomplete data! Check again.";
        imessage="nan";
        qDebug()<<"Message:"<<imessage;

    }
    return imessage;
}


QString MainWindow::getInt32Data(const char* src, int32_t data){
    //according to DATA TYPE
    int size=sizeof(src);
    qDebug()<<"Size:"<< size;
    QString imessage=nullptr;
    if(size >= 5 + 1){
    memcpy(&data, &src[5],4);
    qDebug()<<"Received Int32 Data:"<<data;

    imessage= QString::number(data);
    qDebug()<<"Message:"<<imessage;

    }
    else{
        qDebug()<<"Incomplete data! Check again.";
        imessage="nan";
        qDebug()<<"Message:"<<imessage;

    }
    return imessage;
}


