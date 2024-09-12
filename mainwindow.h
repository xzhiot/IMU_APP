#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include "serialsetup.h"
#include "qcustomplot.h"
#include <QComboBox>
#include <QWheelEvent>
#include <QModbusClient>
#include <QModbusTcpClient>
#include <QModbusRtuSerialMaster>
#include <QModbusServer>
namespace Ui {
    class MainWindow;
}
class WriteRegisterModel;
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void configureSerialPort(const QString &portName, const QString &baudRate,
                             const QString &dataBits, const QString &stopBits,
                             const QString &parity, const QString &flowControl);
    void on_openSetupButton_clicked();
    void readSerialData();
    void on_BASIC_btn_clicked();
    void on_IMU_btn_clicked();
    void on_WAVE_btn_clicked();
    void updateDisplay(bool showOpenGL, bool sendMessage, bool showPlot, bool showModbus,bool showModSlv);
    void legendClicked(QCPLegend *legend, QCPAbstractLegendItem *item, QMouseEvent *event);
    void changeDataDisplay(int index);
    void sendSerialData();
    void onPlotMouseWheel(QWheelEvent *event);
    void togglePlotUpdating();
    void handleMouseRelease(QMouseEvent *event);
    void on_modbusButton_clicked();
    void configureModbusConnection();
    void configureModSlaveConnection();
    void onModbusConnectionTypeChanged(int index);
    void onModSlaveConnectionTypeChanged(int index);
    void readReady();
    void writeReady();


    void on_readButton_clicked();

    void on_writeButton_clicked();

    void on_modmstButton_clicked();

    void on_connectMastBt_clicked();

    void on_connectButton_clicked();
    void onStateChanged(int state);
    void updateWidgets(QModbusDataUnit::RegisterType table, int address, int size);
    void handleDeviceError(QModbusDevice::Error newError);
    void setupDeviceData();
    void setRegister(const QString &value);
    void coilChanged(int id);
    void discreteInputChanged(int id);
    void bitChanged(int id, QModbusDataUnit::RegisterType table, bool value);
private:
    Ui::MainWindow *ui;
    serialsetup *setupDialog;
    QSerialPort *serial;
    QComboBox *dataSelector;
    QModbusClient *modbusDevice;  // General Modbus client for both TCP and RTU
    QModbusServer *modbusDevice1 = nullptr;
    float currentRoll, currentPitch, currentYaw;
    float currentAx, currentAy, currentAz;
    float currentMx, currentMy, currentMz;
    int currentDataType;
    bool isUpdating;
    bool verifyData(const QByteArray &data);
    void setupPlot();
    void updatePlot(float roll, float pitch, float yaw);
    void processICM20602Data(const QByteArray &data);
    void processEulerData(const QByteArray &data);
    void setupUiComponents();
    void setupConnections();
    void processValidData(const QByteArray &data);
    void ensureSerialConnection();
    void readFromModbus();
    void writeToModbus();
    void setupWidgetContainers();
    QButtonGroup coilButtons;
    QButtonGroup discreteButtons;
    QHash<QString, QLineEdit *> registers;
    WriteRegisterModel *writeModel = nullptr;
};

#endif // MAINWINDOW_H
