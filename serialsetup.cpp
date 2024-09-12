#include "serialsetup.h"
#include "ui_serialsetup.h"
#include <QMessageBox>
#include<QDebug>
serialsetup::serialsetup(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::serialsetup)
{
    ui->setupUi(this);
    loadAvailablePorts();
    setupComboBoxes();
    connect(ui->connectButton, &QPushButton::clicked, this, &serialsetup::connectToSerialPort);
    connect(ui->cancelButton, &QPushButton::clicked, this, &QDialog::reject);
}

serialsetup::~serialsetup() {
    if (serial && serial->isOpen()) {
        serial->close();
    }
    delete ui;
}

void serialsetup::loadAvailablePorts()
{
    ui->SR_combox->clear();
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos) {
        ui->SR_combox->addItem(info.portName() + " - " + info.description(), info.portName());
    }
}
void serialsetup::setupComboBoxes() {
    // 假设已经有这样的函数来设置comboBox的项
    ui->BD_combox->clear();
    ui->BD_combox->addItem("500000", QVariant(500000));
    ui->BD_combox->addItem("1200", QVariant(1200));
    ui->BD_combox->addItem("2400", QVariant(2400));
    ui->BD_combox->addItem("4800", QVariant(4800));
    ui->BD_combox->addItem("9600", QVariant(9600));
    ui->BD_combox->addItem("19200", QVariant(19200));
    ui->BD_combox->addItem("38400", QVariant(38400));
    ui->BD_combox->addItem("57600", QVariant(57600));
    ui->BD_combox->addItem("115200", QVariant(115200));
    ui->BD_combox->addItem("600000", QVariant(600000));

    ui->DA_combox->clear();
    ui->DA_combox->addItem("8 bits", QVariant(8));
    ui->DA_combox->addItem("7 bits", QVariant(7));

    ui->ST_combox->clear();
    ui->ST_combox->addItem("1", QVariant(1));
    ui->ST_combox->addItem("1.5", QVariant(1.5));
    ui->ST_combox->addItem("2", QVariant(2));

    ui->CK_combox->clear();
    ui->CK_combox->addItem("无", QVariant(QSerialPort::NoParity));
    ui->CK_combox->addItem("奇", QVariant(QSerialPort::OddParity));
    ui->CK_combox->addItem("偶", QVariant(QSerialPort::EvenParity));

    ui->FC_combox->clear();
    ui->FC_combox->addItem("无", QVariant(QSerialPort::NoFlowControl));
    ui->FC_combox->addItem("硬件", QVariant(QSerialPort::HardwareControl));
    ui->FC_combox->addItem("软件", QVariant(QSerialPort::SoftwareControl));
}
void serialsetup::connectToSerialPort()
{
    QString selectedPortName = ui->SR_combox->currentData().toString();
    QString selectedBaudRate = ui->BD_combox->currentData().toString();
    QString selectedDataBits = ui->DA_combox->currentData().toString();
    QString selectedStopBits = ui->ST_combox->currentData().toString();
    QString selectedParCheck = ui->CK_combox->currentData().toString();
    QString selectedFlowCtrl = ui->FC_combox->currentData().toString();

    emit portSettingsChanged(selectedPortName, selectedBaudRate, selectedDataBits,
                             selectedStopBits, selectedParCheck, selectedFlowCtrl);

    accept();
}

