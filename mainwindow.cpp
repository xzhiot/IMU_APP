#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "qcustomplot.h"
#include "openglwidget.h"
#include "writeregistermodel.h"
#include <cstdint>  // 包含对 int16_t 的支持
#include <QIcon>
#include <QUrl>
#include <QModbusTcpServer>
#include <QModbusRtuSerialSlave>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    setupDialog(nullptr),
    serial(new QSerialPort(this)),
    modbusDevice(nullptr), // 初始化指针为nullptr
    currentRoll(0), currentPitch(0), currentYaw(0),
    currentAx(0), currentAy(0), currentAz(0),
    currentMx(0), currentMy(0), currentMz(0),
    isUpdating(true)        // 初始化时允许更新
{
    ui->setupUi(this);
    setupUiComponents();    // Setup UI components including the data selector
    setupPlot();            // Initialize the plotting area
    setupConnections();
}

MainWindow::~MainWindow()
{
    delete ui;
    if (setupDialog) delete setupDialog;
    delete serial;
    if (modbusDevice) {
        delete modbusDevice;  // 安全删除对象
        modbusDevice = nullptr;  // 重置指针为nullptr防止野指针
    }
    if (modbusDevice1) {
        delete modbusDevice1;  // 安全删除对象
        modbusDevice1 = nullptr;  // 重置指针为nullptr防止野指针
    }
}

void MainWindow::setupConnections() {
    connect(ui->openSetupButton, &QPushButton::clicked, this, &MainWindow::on_openSetupButton_clicked);
    connect(ui->BASIC_btn, &QPushButton::clicked, this, &MainWindow::on_BASIC_btn_clicked);
    connect(ui->IMU_btn, &QPushButton::clicked, this, &MainWindow::on_IMU_btn_clicked);
    connect(ui->WAVE_btn, &QPushButton::clicked, this, &MainWindow::on_WAVE_btn_clicked);
    connect(serial, &QSerialPort::readyRead, this, &MainWindow::readSerialData);
    connect(dataSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::changeDataDisplay);
    connect(ui->sendButton, &QPushButton::clicked, this, &MainWindow::sendSerialData);
    connect(ui->modbusButton, &QPushButton::clicked, this, &MainWindow::on_modbusButton_clicked);
    auto valueChanged = QOverload<int>::of(&QSpinBox::valueChanged);
    connect(ui->spinBoxNumValuesWrite, valueChanged, writeModel, &WriteRegisterModel::setNumberOfValues);
    connect(ui->spinBoxStartAddressWrite, valueChanged, writeModel, &WriteRegisterModel::setStartAddress);
}

void MainWindow::setupUiComponents() {
    // 设置样式表
    QString buttonStyle = R"(
        QPushButton {
            background-color: #555555;
            color: white;
            border-radius: 10px;
            padding: 5px;
            border: 2px solid #777777;
        }
        QPushButton:hover {
            background-color: #999999;
        }
        QPushButton:pressed {
            background-color: #BBBBBB;
        }
    )";

    // 应用样式表到按钮
    ui->openSetupButton->setStyleSheet(buttonStyle);
    ui->BASIC_btn->setStyleSheet(buttonStyle);
    ui->IMU_btn->setStyleSheet(buttonStyle);
    ui->WAVE_btn->setStyleSheet(buttonStyle);
    ui->modbusButton->setStyleSheet(buttonStyle);
    ui->modmstButton->setStyleSheet(buttonStyle);
    // 其他 UI 组件初始化
    dataSelector = new QComboBox(this);
    dataSelector->addItem("欧拉角");
    dataSelector->addItem("加速度");
    dataSelector->addItem("磁力计");
    dataSelector->setEnabled(false);

    QLabel *label = new QLabel("曲线选择: ", this);
    label->setStyleSheet("QLabel { font-weight: bold; color: black; }");  // 标签样式
    ui->toolBar->addWidget(label);
    ui->toolBar->addWidget(dataSelector);

    // 设置图标
    ui->openSetupButton->setIcon(QIcon(":/icons/setup.svg"));
    ui->BASIC_btn->setIcon(QIcon(":/icons/bits.svg"));
    ui->IMU_btn->setIcon(QIcon(":/icons/imu.svg"));
    ui->WAVE_btn->setIcon(QIcon(":/icons/wave.svg"));
    ui->modbusButton->setIcon(QIcon(":/icons/modbus.svg"));
    ui->modmstButton->setIcon(QIcon(":/icons/modbusslv.svg"));
    // 设置图标大小
    QSize iconSize(32, 32);
    ui->openSetupButton->setIconSize(iconSize);
    ui->BASIC_btn->setIconSize(iconSize);
    ui->IMU_btn->setIconSize(iconSize);
    ui->WAVE_btn->setIconSize(iconSize);

    updateDisplay(true,false,false,false,false);
    statusBar()->showMessage("串口未连接");
    writeModel = new WriteRegisterModel(this);
    writeModel->setStartAddress(ui->spinBoxStartAddressWrite->value());
    writeModel->setNumberOfValues(ui->spinBoxNumValuesWrite->value());

    ui->textEditWriteResult->setModel(writeModel);
    ui->textEditWriteResult->setColumnHidden(1,1);
    ui->textEditWriteResult->resizeColumnToContents(0);
    //modbus slave
    setupWidgetContainers();

}

void MainWindow::setupPlot()
{
    // Configure the plotWidget for three generic data streams
    ui->plotWidget->addGraph(); // First data stream
    QColor data1Color = Qt::red; // 为 Roll 设置一个颜色
    ui->plotWidget->graph(0)->setPen(data1Color);
    ui->plotWidget->graph(0)->setProperty("originalColor", data1Color);
    ui->plotWidget->graph(0)->setName("Roll");

    ui->plotWidget->addGraph(); // Second data stream
    QColor data2Color = Qt::green; // 为 Pitch 设置一个颜色
    ui->plotWidget->graph(1)->setPen(data2Color);
    ui->plotWidget->graph(1)->setProperty("originalColor", data2Color);
    ui->plotWidget->graph(1)->setName("Pitch");

    ui->plotWidget->addGraph(); // Third data stream
    QColor data3Color = Qt::blue; // 为 Yaw 设置一个颜色
    ui->plotWidget->graph(2)->setPen(data3Color);
    ui->plotWidget->graph(2)->setProperty("originalColor", data3Color);
    ui->plotWidget->graph(2)->setName("Yaw");

    ui->plotWidget->xAxis->setLabel("样点");
    ui->plotWidget->yAxis->setLabel("值");
    ui->plotWidget->legend->setVisible(true);
    ui->plotWidget->legend->setSelectableParts(QCPLegend::spItems);

    ui->plotWidget->xAxis->setRange(0, 100);
    ui->plotWidget->yAxis->setRange(-180, 180);
    ui->plotWidget->setInteraction(QCP::iRangeZoom, true);
    ui->plotWidget->axisRect()->setRangeZoom(Qt::Horizontal);
    ui->plotWidget->axisRect()->setRangeDrag(Qt::Horizontal);

    // 启用选择矩形模式以及缩放和拖拽
    ui->plotWidget->setInteraction(QCP::iRangeZoom, true); // 启用缩放
    ui->plotWidget->setInteraction(QCP::iRangeDrag, true); // 启用拖拽
    ui->plotWidget->setSelectionRectMode(QCP::srmZoom); // 设置选择矩形模式为缩放
    connect(ui->plotWidget, &QCustomPlot::mouseRelease, this, &MainWindow::handleMouseRelease);

    connect(ui->plotWidget, &QCustomPlot::mouseDoubleClick, this, &MainWindow::togglePlotUpdating);

    connect(ui->plotWidget, &QCustomPlot::mouseWheel, this, &MainWindow::onPlotMouseWheel);

    connect(ui->plotWidget, &QCustomPlot::legendClick, this, &MainWindow::legendClicked);
    // 初始化 Modbus 连接类型的下拉菜单
    ui->comboBoxConnectionType->addItem("TCP", QVariant("TCP"));
    ui->comboBoxConnectionType->addItem("串口(RTU)", QVariant("RTU"));
    ui->comboBoxConnectionType->addItem("串口(ASCII)", QVariant("ASCII"));

    // 初始化 Modbus 连接类型的下拉菜单
    ui->comboBoxConnectionType1->addItem("TCP", QVariant("TCP"));
    ui->comboBoxConnectionType1->addItem("串口(RTU)", QVariant("RTU"));
    ui->comboBoxConnectionType1->addItem("串口(ASCII)", QVariant("ASCII"));

    connect(ui->comboBoxConnectionType, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onModbusConnectionTypeChanged);  // 添加响应函数
    connect(ui->comboBoxConnectionType1, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onModSlaveConnectionTypeChanged);
}


void MainWindow::on_openSetupButton_clicked() {
    if (!setupDialog) {
        setupDialog = new serialsetup(this);
        connect(setupDialog, &serialsetup::portSettingsChanged, this, &MainWindow::configureSerialPort);
    }
    if (!setupDialog->isVisible()) {
        setupDialog->exec();
    }
}

void MainWindow::configureSerialPort(const QString &portName, const QString &baudRate,
                                     const QString &dataBits, const QString &stopBits,
                                     const QString &parity, const QString &flowControl)
{
    serial->setPortName(portName);
    serial->setBaudRate(baudRate.toInt());
    serial->setDataBits(static_cast<QSerialPort::DataBits>(dataBits.toInt()));
    // 根据选择设置停止位
    if (stopBits == "1") {
        serial->setStopBits(QSerialPort::OneStop);
    } else if (stopBits == "1.5") {
        serial->setStopBits(QSerialPort::OneAndHalfStop);
    } else if (stopBits == "2") {
        serial->setStopBits(QSerialPort::TwoStop);
    } else {
        serial->setStopBits(QSerialPort::OneStop); // 如果没有有效的选项，设置为未知
    }

    // 根据奇偶校验选择设置奇偶校验
    if (parity == "无") {
        serial->setParity(QSerialPort::NoParity);
    } else if (parity == "奇") {
        serial->setParity(QSerialPort::OddParity);
    } else if (parity == "偶") {
        serial->setParity(QSerialPort::EvenParity);
    } else {
        serial->setParity(QSerialPort::NoParity); // 如果没有有效的选项，设置为未知
    }

    // 流控制设置
    if (flowControl == "无") {
        serial->setFlowControl(QSerialPort::NoFlowControl);
    } else if (flowControl == "硬件") {
        serial->setFlowControl(QSerialPort::HardwareControl);
    } else if (flowControl == "软件") {
        serial->setFlowControl(QSerialPort::SoftwareControl);
    } else {
        serial->setFlowControl(QSerialPort::NoFlowControl); // 如果没有有效的选项，设置为未知
    }

    if (!serial->open(QIODevice::ReadWrite)) {
        statusBar()->showMessage("连接串口失败: " + serial->errorString());
    } else {
        statusBar()->showMessage("已连接到： " + portName);
    }
}
void MainWindow::legendClicked(QCPLegend *legend, QCPAbstractLegendItem *item, QMouseEvent *event)
{
    Q_UNUSED(event);
    if (item) // 如果点击了图例项
    {
        QCPPlottableLegendItem *plItem = qobject_cast<QCPPlottableLegendItem*>(item);
        if (plItem)
        {
            QCPGraph *graph = qobject_cast<QCPGraph *>(plItem->plottable());
            if (graph) {
                bool isVisible = graph->visible();
                graph->setVisible(!isVisible); // 切换图形的可见性

                // 更新图形的线条颜色，以及图例标记的颜色
                QPen graphPen = graph->pen();
                if (isVisible) {
                    // 如果当前是可见的，隐藏它并将颜色改为灰色
                    graphPen.setColor(QColor(200, 200, 200));
                } else {
                    // 如果当前是隐藏的，显示它并恢复原始颜色
                    graphPen.setColor(graph->property("originalColor").value<QColor>());
                }
                graph->setPen(graphPen);

                ui->plotWidget->replot(); // 重绘图表以更新变化
            }
        }
    }
}


void MainWindow::handleMouseRelease(QMouseEvent *event) {
    if (ui->plotWidget->selectionRect()->isActive() && !isUpdating) {
        QRect selectionRect = ui->plotWidget->selectionRect()->rect();
        QPoint topLeft = selectionRect.topLeft();
        QPoint bottomRight = selectionRect.bottomRight();

        // 判断选择框是否足够大以触发缩放
        int minSize = 10; // 最小的有效选择框尺寸
        if (std::abs(topLeft.x() - bottomRight.x()) < minSize ||
            std::abs(topLeft.y() - bottomRight.y()) < minSize) {
            return; // 选择框太小，忽略此次操作
        }

        // 计算选择框的中心点坐标
        double rectCenterX = ui->plotWidget->xAxis->pixelToCoord(selectionRect.center().x());
        double rectCenterY = ui->plotWidget->yAxis->pixelToCoord(selectionRect.center().y());

        // 根据选择框的对角线方向决定是放大还是缩小
        bool isZoomIn = (topLeft.x() < bottomRight.x() && topLeft.y() < bottomRight.y());
        double factor = isZoomIn ? 0.8 : 1.25; // 放大时使用的因子小于1，缩小时大于1

        // 使用选择框的中心作为缩放中心
        ui->plotWidget->xAxis->scaleRange(factor, rectCenterX);
        ui->plotWidget->yAxis->scaleRange(factor, rectCenterY);

        ui->plotWidget->replot();
    }
}

void MainWindow::togglePlotUpdating() {
    isUpdating = !isUpdating; // 切换状态
    if (isUpdating) {
        statusBar()->showMessage("更新已启动");
        QString selected = dataSelector->currentText();
        if (selected == "欧拉角") {
            ui->plotWidget->yAxis->setRange(-180, 180);
        } else if (selected == "加速度") {
            ui->plotWidget->yAxis->setRange(-5000, 5000);
        } else if (selected == "磁力计") {
            ui->plotWidget->yAxis->setRange(-500, 500);
        }
        ui->plotWidget->setSelectionRectMode(QCP::srmNone); // 禁用选择矩形模式
    } else {
        statusBar()->showMessage("更新已停止");
        ui->plotWidget->setSelectionRectMode(QCP::srmZoom); // 启用选择矩形模式为缩放
    }
}


void MainWindow::onPlotMouseWheel(QWheelEvent *event) {
    // Check the direction of the wheel roll
    double factor = event->angleDelta().y() > 0 ? 0.85 : 1.15; // Zoom in or out
    ui->plotWidget->xAxis->scaleRange(factor, ui->plotWidget->xAxis->pixelToCoord(event->pos().x()));
    ui->plotWidget->replot();
}

void MainWindow::changeDataDisplay(int index) {
    QString selected = dataSelector->itemText(index);
    statusBar()->showMessage("Switching data display to: " + selected);

    if (selected == "欧拉角") {
        ui->plotWidget->graph(0)->setName("Roll");
        ui->plotWidget->graph(1)->setName("Pitch");
        ui->plotWidget->graph(2)->setName("Yaw");
        ui->plotWidget->yAxis->setRange(-180, 180); // 欧拉角一般在-180到180度之间
        updatePlot(currentRoll, currentPitch, currentYaw);
    } else if (selected == "加速度") {
        ui->plotWidget->graph(0)->setName("AccX");
        ui->plotWidget->graph(1)->setName("AccY");
        ui->plotWidget->graph(2)->setName("AccZ");
        ui->plotWidget->yAxis->setRange(-5000, 5000); // 假设加速度计量程为±16g
        updatePlot(currentAx, currentAy, currentAz);
    } else if (selected == "磁力计") {
        ui->plotWidget->graph(0)->setName("MagX");
        ui->plotWidget->graph(1)->setName("MagY");
        ui->plotWidget->graph(2)->setName("MagZ");
        ui->plotWidget->yAxis->setRange(-500, 500); // 假设磁力计的适当范围
        updatePlot(currentMx, currentMy, currentMz);
    }
}

void MainWindow::updatePlot(float value1, float value2, float value3) {
    if (!isUpdating) return; // 如果更新被停止，则直接返回

    static int dataIndex = 0;
    static QVector<double> timeStamps, y0, y1, y2;

    ui->plotWidget->setSelectionRectMode(QCP::srmNone); // 禁用选择矩形模式
    // 添加新的数据点
    timeStamps.push_back(dataIndex); // 使用 dataIndex 作为 x 轴时间戳
    y0.push_back(value1);
    y1.push_back(value2);
    y2.push_back(value3);

    // 更新图表的数据集
    ui->plotWidget->graph(0)->setData(timeStamps, y0);
    ui->plotWidget->graph(1)->setData(timeStamps, y1);
    ui->plotWidget->graph(2)->setData(timeStamps, y2);

    // 动态调整 X 轴范围以适应用户的缩放选择
    if (ui->plotWidget->xAxis->range().size() > 100) {
        // 如果用户已经缩放显示超过100个数据点，则维持当前的显示范围
        ui->plotWidget->xAxis->setRange(dataIndex-ui->plotWidget->xAxis->range().size(), dataIndex);
    } else {
        // 否则，自动滚动以显示最新的100个数据点
        ui->plotWidget->xAxis->setRange(dataIndex - 100, dataIndex);
    }

    // 仅在有显著更新时重绘，以优化性能
      if (dataIndex % 1 == 0) {
          ui->plotWidget->replot();
      }

    dataIndex++;
}

void MainWindow::readSerialData() {
    static QByteArray accumulatedData;  // 用于累积数据的缓冲区
    QByteArray data = serial->readAll();
    // 将新接收的数据添加到累积缓冲区
    accumulatedData.append(data);
    QString dataString = data.toHex(' ').toUpper();
    ui->textDisplay->append(dataString);

    // 尝试处理累积的数据
    int startIndex = 0;  // 开始搜索的索引
    while (true) {
        // 查找帧头
        startIndex = accumulatedData.indexOf("\xAA\xFF", startIndex);
        if (startIndex == -1 || (accumulatedData.size() - startIndex) < 4) {
            // 没有找到帧头或数据不足以包含最小长度，等待更多数据
            if (startIndex != -1) {
                // 丢弃直到帧头之前的所有数据
                accumulatedData.remove(0, startIndex);
            }
            break;
        }

        // 检查累积数据是否包含完整的帧头
        if ((accumulatedData.size() - startIndex) >= 4) {
            uint8_t dataLength = static_cast<uint8_t>(accumulatedData[startIndex + 3]);
            int expectedFrameLength = 6 + dataLength;  // 完整帧的预期长度

            if ((accumulatedData.size() - startIndex) < expectedFrameLength) {
                // 数据不足以构成完整帧，等待更多数据
                break;
            }

            // 尝试验证
            if (verifyData(accumulatedData.mid(startIndex, expectedFrameLength))) {
                // 数据验证成功，处理数据
                processValidData(accumulatedData.mid(startIndex, expectedFrameLength));
                accumulatedData.remove(0, startIndex + expectedFrameLength);  // 移除已处理的数据
                startIndex = 0;  // 重置开始索引，从新的数据开始
            } else {
                // 数据验证失败，从下一个字节继续寻找帧头
                startIndex++;
            }
        } else {
            break;  // 数据不足，等待更多数据
        }
    }
}
void MainWindow::processValidData(const QByteArray &data) {
    // 你的数据处理逻辑，例如更新UI、解析数据等
    int functionCode = static_cast<uint8_t>(data[2]);
    switch (functionCode) {
        case 1:
            if (data.size() >= 14) {
                processICM20602Data(data);
            } else {
                statusBar()->showMessage("ICM20602 数据长度不足");
            }
            break;
        case 3:
            if (data.size() >= 8) {
                processEulerData(data);
            } else {
                statusBar()->showMessage("欧拉角数据长度不足");
            }
            break;
        default:
            statusBar()->showMessage("无效的功能码");
            break;
    }
}
void MainWindow::processICM20602Data(const QByteArray &data) {
    // Parsing sensor data and updating currentAx, currentAy, etc.
    currentAx = int16_t(uint8_t(data[4]) << 8 | uint8_t(data[5]));
    currentAy = int16_t(uint8_t(data[6]) << 8 | uint8_t(data[7]));
    currentAz = int16_t(uint8_t(data[8]) << 8 | uint8_t(data[9]));
    currentMx = int16_t(uint8_t(data[10]) << 8 | uint8_t(data[11]));
    currentMy = int16_t(uint8_t(data[12]) << 8 | uint8_t(data[13]));
    currentMz = int16_t(uint8_t(data[14]) << 8 | uint8_t(data[15]));
    statusBar()->showMessage(QString("ICM20602 - ax: %1, ay: %2, az: %3, mx: %4, my: %5, mz: %6").arg(currentAx).arg(currentAy).arg(currentAz).arg(currentMx).arg(currentMy).arg(currentMz));
    if (dataSelector->currentText() == "加速度") {
        updatePlot(currentAx, currentAy, currentAz);
    }
    else if(dataSelector->currentText() == "磁力计")
    {
        updatePlot(currentMx, currentMy, currentMz);
    }
}

void MainWindow::processEulerData(const QByteArray &data) {
    // Parsing Euler angles
    currentRoll = int16_t(uint8_t(data[4]) << 8 | uint8_t(data[5])) / 100.0f;
    currentPitch = int16_t(uint8_t(data[6]) << 8 | uint8_t(data[7])) / 100.0f;
    currentYaw = int16_t(uint8_t(data[8]) << 8 | uint8_t(data[9])) / 100.0f;
    //statusBar()->showMessage(QString("欧拉角 - Roll: %1, Pitch: %2, Yaw: %3").arg(currentRoll).arg(currentPitch).arg(currentYaw));
    // 更新 OpenGL Widget 的姿态
    OpenGLWidget *openGLWidget = dynamic_cast<OpenGLWidget*>(ui->openGLWidget);
    if (openGLWidget) {
        openGLWidget->updateIMUData(currentRoll, currentPitch, currentYaw);
    }

    if (dataSelector->currentText() == "欧拉角") {
        updatePlot(currentRoll, currentPitch, currentYaw);
    }
}

bool MainWindow::verifyData(const QByteArray &data) {
    uint8_t sumcheck = 0, addcheck = 0;
    for (int i = 0; i < data.size() - 2; ++i) {
        sumcheck += static_cast<uint8_t>(data[i]);
        addcheck += sumcheck;
    }

    return (sumcheck == static_cast<uint8_t>(data[data.size() - 2]) &&
            addcheck == static_cast<uint8_t>(data[data.size() - 1]));
}

void MainWindow::on_BASIC_btn_clicked() {
    ensureSerialConnection();  // 确保串口连接
    updateDisplay(false, true, false, false,false);
    dataSelector->setEnabled(false);  //
}

void MainWindow::on_IMU_btn_clicked() {
    ensureSerialConnection();  // 确保串口连接
    updateDisplay(true, false, false, false,false);
    dataSelector->setEnabled(false);  //

}

void MainWindow::on_WAVE_btn_clicked() {
    ensureSerialConnection();  // 确保串口连接
    updateDisplay(false, false, true, false,false);
    dataSelector->setEnabled(true);  //使ComboBox可用
}

void MainWindow::sendSerialData() {
    QString hexData = ui->textInput->toPlainText();
    QByteArray sendData = QByteArray::fromHex(hexData.toLatin1());
    if (sendData.isEmpty()) {
        statusBar()->showMessage("错误：数据格式无效或为空！");
        return;
    }
    serial->write(sendData);
    statusBar()->showMessage("数据已发送");
}

void MainWindow::updateDisplay(bool showOpenGL, bool sendMessage, bool showPlot, bool showModbus,bool showModSlv) {
    if (sendMessage) {
        ui->stackedLayout->setCurrentWidget(ui->BASICWidget);
    } else if (showModbus) {
        ui->stackedLayout->setCurrentWidget(ui->modbusGroup);
    } else if (showPlot) {
        ui->stackedLayout->setCurrentWidget(ui->WAVEWidget);
        ui->plotWidget->setVisible(showPlot);
    } else if (showOpenGL) {
        ui->stackedLayout->setCurrentWidget(ui->IMUWidget);
    } else if(showModSlv){
        ui->stackedLayout->setCurrentWidget(ui->modslvGroup);
    }
}


void MainWindow::ensureSerialConnection() {
    if (!serial->isOpen()) {
        if (modbusDevice && modbusDevice->state() == QModbusDevice::ConnectedState) {
            modbusDevice->disconnectDevice();
            delete modbusDevice;
            modbusDevice = nullptr;
        }
        if (modbusDevice1 && modbusDevice1->state() == QModbusDevice::ConnectedState) {
            modbusDevice1->disconnectDevice();
            delete modbusDevice1;
            modbusDevice1 = nullptr;
        }
        // 重新打开并配置串口
        configureSerialPort(serial->portName(), QString::number(serial->baudRate()), QString::number(serial->dataBits()),
                            QString::number(serial->stopBits()), QString(serial->parity()), QString(serial->flowControl()));
    }
}

void MainWindow::configureModbusConnection() {
    QString connectionType = ui->comboBoxConnectionType->currentText();

    // 断开现有连接并安全释放旧设备
    if (modbusDevice) {
        if (modbusDevice->state() == QModbusDevice::ConnectedState) {
            modbusDevice->disconnectDevice();
        }
        delete modbusDevice;  // 安全删除对象
        modbusDevice = nullptr;  // 重置指针为nullptr防止野指针
    }
    if (modbusDevice1) {
        if (modbusDevice1->state() == QModbusDevice::ConnectedState) {
            modbusDevice1->disconnectDevice();
        }
        delete modbusDevice1;  // 安全删除对象
        modbusDevice1 = nullptr;  // 重置指针为nullptr防止野指针
    }

    // 根据选择的连接类型创建新的Modbus设备
    if (connectionType == "TCP") {
        modbusDevice = new QModbusTcpClient(this);
        modbusDevice->setConnectionParameter(QModbusDevice::NetworkAddressParameter, ui->lineEditIpAddress->text());
        modbusDevice->setConnectionParameter(QModbusDevice::NetworkPortParameter, ui->lineEditPort->text().toInt());
    } else if (connectionType == "串口(RTU)" || connectionType == "串口(ASCII)") {
        if (serial && serial->isOpen()) {
                   serial->close();  // 关闭已打开的串口以免冲突
               }
        modbusDevice = new QModbusRtuSerialMaster(this);
        // 应用串口配置
        modbusDevice->setConnectionParameter(QModbusDevice::SerialPortNameParameter, serial->portName());
        modbusDevice->setConnectionParameter(QModbusDevice::SerialBaudRateParameter, serial->baudRate());
        modbusDevice->setConnectionParameter(QModbusDevice::SerialDataBitsParameter, serial->dataBits());
        modbusDevice->setConnectionParameter(QModbusDevice::SerialStopBitsParameter, serial->stopBits());
        modbusDevice->setConnectionParameter(QModbusDevice::SerialParityParameter, serial->parity());
        //modbusDevice->setConnectionParameter(QModbusDevice::SerialFlowControlParameter, serial->flowControl());
    }

    // 尝试连接设备
    //if (!modbusDevice->connectDevice()) {
    //    statusBar()->showMessage(connectionType+"连接失败");
    //} else {
    //    statusBar()->showMessage(connectionType+"连接成功");
    //}
}

void MainWindow::onModbusConnectionTypeChanged(int index) {
    // 可以在这里添加逻辑来响应连接类型的变更
    // 例如更新界面上的其他组件状态
    QString connectionType = ui->comboBoxConnectionType->itemData(index).toString();
    if (connectionType == "TCP") {
        ui->lineEditIpAddress->setEnabled(true);
        ui->lineEditPort->setEnabled(true);
        //ui->lineEditPortName->setEnabled(false);
        ui->listenOnlyBox->setEnabled(false);
        ui->connectMastBt->setEnabled(true);
        configureModbusConnection();  // 配置Modbus连接
    } else {
        ui->lineEditIpAddress->setEnabled(true);
        ui->lineEditPort->setEnabled(true);
        //ui->lineEditPortName->setEnabled(true);
        ui->listenOnlyBox->setEnabled(true);
        ui->connectMastBt->setEnabled(true);
        configureModbusConnection();  // 配置Modbus连接
    }
}

void MainWindow::on_modbusButton_clicked()
{
    updateDisplay(false, false, false,true,false);  // 显示 Modbus 主机界面
    dataSelector->setEnabled(false);  //
    configureModbusConnection();  // 配置Modbus连接
}

void MainWindow::on_readButton_clicked()
{
    if (!modbusDevice || modbusDevice->state() != QModbusDevice::ConnectedState) {
            statusBar()->showMessage("Modbus 设备未连接");
            return;
        }
        //ui->textEditReadResult->clear();
        int startAddress = ui->spinBoxStartAddressRead->value();
        int numEntries = ui->spinBoxNumValuesRead->value();
        // Get server address, default to 1 if empty
        QString portText = ui->lineEditPort->text();
        int serverAddress = portText.isEmpty() ? 1 : portText.toInt();

        if (auto *reply = modbusDevice->sendReadRequest(QModbusDataUnit(QModbusDataUnit::HoldingRegisters, startAddress, numEntries), serverAddress)) {
            if (!reply->isFinished()) {
                connect(reply, &QModbusReply::finished, this, &MainWindow::readReady);
            } else {
                delete reply; // broadcast reply returns immediately
            }
        } else {
            statusBar()->showMessage("读取失败: " + modbusDevice->errorString());
        }
}

void MainWindow::on_writeButton_clicked() {
    if (!modbusDevice || modbusDevice->state() != QModbusDevice::ConnectedState) {
        statusBar()->showMessage("Modbus 设备未连接");
        return;
    }

    int startAddress = ui->spinBoxStartAddressWrite->value();
    unsigned int sendNum = ui->spinBoxNumValuesWrite->value();
    // Get server address, default to 1 if empty
    QString portText = ui->lineEditPort->text();
    int serverAddress = portText.isEmpty() ? 1 : portText.toInt();
    if (sendNum < 1) {
            sendNum = 1; // 确保至少发送一个寄存器
        }

    /*QString hexValues = ui->lineEditWriteValue->text();
    QVector<quint16> values;

    // 将字符串每两个字符解析为一个16位的整数
    for (int i = 0; i < hexValues.length(); i += 4) {
        bool ok;
        if (i + 4 <= hexValues.length()) {
            quint16 value = hexValues.mid(i, 4).toUInt(&ok, 16);
            if (!ok) {
                statusBar()->showMessage("输入的16进制数无效");
                return;
            }
            values.push_back(value);
        }
    }*/

    QModbusDataUnit writeUnit(QModbusDataUnit::HoldingRegisters, startAddress, sendNum);
    for (int i = 0, total = int(writeUnit.valueCount()); i < total; ++i) {
        writeUnit.setValue(i, writeModel->m_holdingRegisters[i + writeUnit.startAddress()]);
    }

    if (auto *reply = modbusDevice->sendWriteRequest(writeUnit, serverAddress)) {
        connect(reply, &QModbusReply::finished, this, &MainWindow::writeReady);
    } else {
        statusBar()->showMessage("写入请求失败: " + modbusDevice->errorString());
    }
}

void MainWindow::readReady()
{
    auto reply = qobject_cast<QModbusReply *>(sender());
    if (!reply)
        return;

    if (reply->error() == QModbusDevice::NoError) {
        const QModbusDataUnit unit = reply->result();
        QString result;
        for (uint i = 0; i < unit.valueCount(); i++) {
            result += QString::number(unit.value(i)) + " ";
        }
        ui->textEditReadResult->addItem(result);
    } else {
        ui->textEditReadResult->addItem("读取错误: " + reply->errorString());
    }

    reply->deleteLater();
}

void MainWindow::writeReady()
{
    auto reply = qobject_cast<QModbusReply *>(sender());
    if (!reply)
        return;

    if (reply->error() == QModbusDevice::NoError) {
        ui->textEditReadResult->addItem("写入成功");
    } else {
        ui->textEditReadResult->addItem("写入错误: " + reply->errorString());
    }

    reply->deleteLater();
}

void MainWindow::on_modmstButton_clicked()
{
    updateDisplay(false, false, false,false,true);  // 显示 Modbus 主机界面
    dataSelector->setEnabled(false);  //
    configureModSlaveConnection();  // 配置Modbus连接

}

void MainWindow::on_connectMastBt_clicked()
{
    QString connectionType = ui->comboBoxConnectionType->currentText();
    bool intendToConnect = (modbusDevice->state() == QModbusDevice::UnconnectedState);
    if(intendToConnect){
        if (!modbusDevice->connectDevice()) {
                ui->connectMastBt->setText(tr("连接"));
                statusBar()->showMessage(connectionType+"连接失败");
            } else {
                ui->connectMastBt->setText(tr("断开"));
                statusBar()->showMessage(connectionType+"连接成功");
            }
        } else {
        modbusDevice->disconnectDevice();
        ui->connectMastBt->setText(tr("连接"));
        statusBar()->showMessage(connectionType+"连接断开");
    }
}

void MainWindow::setupWidgetContainers()
{
    coilButtons.setExclusive(false);
    discreteButtons.setExclusive(false);

    QRegularExpression regexp(QStringLiteral("coils_(?<ID>\\d+)"));
    const QList<QCheckBox *> coils = findChildren<QCheckBox *>(regexp);
    for (QCheckBox *cbx : coils)
        coilButtons.addButton(cbx, regexp.match(cbx->objectName()).captured("ID").toInt());
    connect(&coilButtons, SIGNAL(buttonClicked(int)), this, SLOT(coilChanged(int)));

    regexp.setPattern(QStringLiteral("disc_(?<ID>\\d+)"));
    const QList<QCheckBox *> discs = findChildren<QCheckBox *>(regexp);
    for (QCheckBox *cbx : discs)
        discreteButtons.addButton(cbx, regexp.match(cbx->objectName()).captured("ID").toInt());
    connect(&discreteButtons, SIGNAL(buttonClicked(int)), this, SLOT(discreteInputChanged(int)));

    regexp.setPattern(QLatin1String("(in|hold)Reg_(?<ID>\\d+)"));
    const QList<QLineEdit *> qle = findChildren<QLineEdit *>(regexp);
    for (QLineEdit *lineEdit : qle)
    {
        registers.insert(lineEdit->objectName(), lineEdit);
        lineEdit->setProperty("ID", regexp.match(lineEdit->objectName()).captured("ID").toInt());
        lineEdit->setValidator(new QRegularExpressionValidator(QRegularExpression(QStringLiteral("[0-9a-f]{0,4}"),
                                                                                  QRegularExpression::CaseInsensitiveOption),
                                                               this));
        connect(lineEdit, &QLineEdit::textChanged, this, &MainWindow::setRegister);
    }
}

void MainWindow::coilChanged(int id)
{
    QAbstractButton *button = coilButtons.button(id);
    bitChanged(id, QModbusDataUnit::Coils, button->isChecked());
}

void MainWindow::discreteInputChanged(int id)
{
    QAbstractButton *button = discreteButtons.button(id);
    bitChanged(id, QModbusDataUnit::DiscreteInputs, button->isChecked());
}

void MainWindow::bitChanged(int id, QModbusDataUnit::RegisterType table, bool value)
{
    if (!modbusDevice1)
        return;

    if (!modbusDevice1->setData(table, quint16(id), value))
        statusBar()->showMessage(tr("Could not set data: ") + modbusDevice1->errorString(), 5000);
}

void MainWindow::setRegister(const QString &value)
{
    if (!modbusDevice1)
        return;

    const QString objectName = QObject::sender()->objectName();
    if (registers.contains(objectName))
    {
        bool ok = true;
        const quint16 id = quint16(QObject::sender()->property("ID").toUInt());
        if (objectName.startsWith(QStringLiteral("inReg")))
            ok = modbusDevice1->setData(QModbusDataUnit::InputRegisters, id, value.toUShort(&ok, 16));
        else if (objectName.startsWith(QStringLiteral("holdReg")))
            ok = modbusDevice1->setData(QModbusDataUnit::HoldingRegisters, id, value.toUShort(&ok, 16));

        if (!ok)
            statusBar()->showMessage(tr("Could not set register: ") + modbusDevice1->errorString(),
                                     5000);
    }
}

void MainWindow::setupDeviceData()
{
    if (!modbusDevice)
        return;

    for (quint16 i = 0; i < coilButtons.buttons().count(); ++i)
        modbusDevice1->setData(QModbusDataUnit::Coils, i, coilButtons.button(i)->isChecked());

    for (quint16 i = 0; i < discreteButtons.buttons().count(); ++i)
    {
        modbusDevice1->setData(QModbusDataUnit::DiscreteInputs, i,
                              discreteButtons.button(i)->isChecked());
    }

    bool ok;
    for (QLineEdit *widget : qAsConst(registers))
    {
        if (widget->objectName().startsWith(QStringLiteral("inReg")))
        {
            modbusDevice1->setData(QModbusDataUnit::InputRegisters, quint16(widget->property("ID").toUInt()),
                                  widget->text().toUShort(&ok, 16));
        }
        else if (widget->objectName().startsWith(QStringLiteral("holdReg")))
        {
            modbusDevice1->setData(QModbusDataUnit::HoldingRegisters, quint16(widget->property("ID").toUInt()),
                                  widget->text().toUShort(&ok, 16));
        }
    }
}

void MainWindow::handleDeviceError(QModbusDevice::Error newError)
{
    if (newError == QModbusDevice::NoError || !modbusDevice)
        return;

    statusBar()->showMessage(modbusDevice->errorString(), 5000);
}

void MainWindow::updateWidgets(QModbusDataUnit::RegisterType table, int address, int size)
{
    for (int i = 0; i < size; ++i)
    {
        quint16 value;
        QString text;
        switch (table)
        {
        case QModbusDataUnit::Coils:
            modbusDevice1->data(QModbusDataUnit::Coils, quint16(address + i), &value);
            coilButtons.button(address + i)->setChecked(value);
            break;
        case QModbusDataUnit::HoldingRegisters:
            modbusDevice1->data(QModbusDataUnit::HoldingRegisters, quint16(address + i), &value);
            registers.value(QStringLiteral("holdReg_%1").arg(address + i))->setText(text.setNum(value, 16));
            break;
        default:
            break;
        }
    }
}

void MainWindow::onStateChanged(int state)
{
    bool connected = (state != QModbusDevice::UnconnectedState);
    //ui->actionConnect->setEnabled(!connected);
    //ui->actionDisconnect->setEnabled(connected);

    if (state == QModbusDevice::UnconnectedState)
        ui->connectButton->setText(tr("连接"));
    else if (state == QModbusDevice::ConnectedState)
        ui->connectButton->setText(tr("断开"));
}

void MainWindow::onModSlaveConnectionTypeChanged(int index) {
    // 可以在这里添加逻辑来响应连接类型的变更
    // 例如更新界面上的其他组件状态
    QString connectionType = ui->comboBoxConnectionType1->itemData(index).toString();
    if (connectionType == "TCP") {
        configureModSlaveConnection();  // 配置Modbus连接
    } else {
        configureModSlaveConnection();  // 配置Modbus连接
    }
}


void MainWindow::configureModSlaveConnection() {
    QString connectionType = ui->comboBoxConnectionType1->currentText();

    // 断开现有连接并安全释放旧设备
    if (modbusDevice) {
        if (modbusDevice->state() == QModbusDevice::ConnectedState) {
            modbusDevice->disconnectDevice();
        }
        delete modbusDevice;  // 安全删除对象
        modbusDevice = nullptr;  // 重置指针为nullptr防止野指针
    }
    if (modbusDevice1) {
        if (modbusDevice1->state() == QModbusDevice::ConnectedState) {
            modbusDevice1->disconnectDevice();
        }
        delete modbusDevice1;  // 安全删除对象
        modbusDevice1 = nullptr;  // 重置指针为nullptr防止野指针
    }

    // 根据选择的连接类型创建新的Modbus设备
    if (connectionType == "TCP") {
        modbusDevice1 = new QModbusTcpServer(this);
        if (ui->portEdit->text().isEmpty())
                    ui->portEdit->setText(QLatin1String("127.0.0.1:502"));
        const QUrl url = QUrl::fromUserInput(ui->portEdit->text());
        modbusDevice1->setConnectionParameter(QModbusDevice::NetworkAddressParameter, url.host());
        modbusDevice1->setConnectionParameter(QModbusDevice::NetworkPortParameter, url.port());
        modbusDevice1->setServerAddress(ui->serverEdit->text().toInt());
        //modbusDevice->setServerAddress
    } else if (connectionType == "串口(RTU)" || connectionType == "串口(ASCII)") {
        if (serial && serial->isOpen()) {
                   serial->close();  // 关闭已打开的串口以免冲突
               }
        modbusDevice1 = new QModbusRtuSerialSlave(this);
        // 应用串口配置
        modbusDevice1->setConnectionParameter(QModbusDevice::SerialPortNameParameter, serial->portName());
        modbusDevice1->setConnectionParameter(QModbusDevice::SerialBaudRateParameter, serial->baudRate());
        modbusDevice1->setConnectionParameter(QModbusDevice::SerialDataBitsParameter, serial->dataBits());
        modbusDevice1->setConnectionParameter(QModbusDevice::SerialStopBitsParameter, serial->stopBits());
        modbusDevice1->setConnectionParameter(QModbusDevice::SerialParityParameter, serial->parity());
        //modbusDevice->setConnectionParameter(QModbusDevice::SerialFlowControlParameter, serial->flowControl());
        modbusDevice1->setServerAddress(ui->serverEdit->text().toInt());
    }

    if (!modbusDevice1)
        {
            ui->connectButton->setDisabled(true);
            if (connectionType == "串口(RTU)" || connectionType == "串口(ASCII)")
                statusBar()->showMessage(tr("Could not create Modbus slave."), 5000);
            else
                statusBar()->showMessage(tr("Could not create Modbus server."), 5000);
        }
        else
        {
            QModbusDataUnitMap reg;
            reg.insert(QModbusDataUnit::Coils, {QModbusDataUnit::Coils, 0, 10});
            reg.insert(QModbusDataUnit::DiscreteInputs, {QModbusDataUnit::DiscreteInputs, 0, 10});
            reg.insert(QModbusDataUnit::InputRegisters, {QModbusDataUnit::InputRegisters, 0, 10});
            reg.insert(QModbusDataUnit::HoldingRegisters, {QModbusDataUnit::HoldingRegisters, 0, 10});

            modbusDevice1->setMap(reg);

            connect(modbusDevice1, &QModbusServer::dataWritten,
                    this, &MainWindow::updateWidgets);
            connect(modbusDevice1, &QModbusServer::stateChanged,
                    this, &MainWindow::onStateChanged);
            connect(modbusDevice1, &QModbusServer::errorOccurred,
                    this, &MainWindow::handleDeviceError);

            connect(ui->listenOnlyBox, &QCheckBox::toggled, this, [this](bool toggled)
                    {
                if (modbusDevice1)
                    modbusDevice1->setValue(QModbusServer::ListenOnlyMode, toggled); });
            emit ui->listenOnlyBox->toggled(ui->listenOnlyBox->isChecked());
            connect(ui->setBusyBox, &QCheckBox::toggled, this, [this](bool toggled)
                    {
                if (modbusDevice1)
                    modbusDevice1->setValue(QModbusServer::DeviceBusy, toggled ? 0xffff : 0x0000); });
            emit ui->setBusyBox->toggled(ui->setBusyBox->isChecked());

            setupDeviceData();
        }
    //ui->listenOnlyBox->setEnabled(connectionType == "串口(RTU)"|| connectionType == "串口(ASCII)");

    // 尝试连接设备
    //if (!modbusDevice->connectDevice()) {
    //    statusBar()->showMessage(connectionType+"连接失败");
    //} else {
    //    statusBar()->showMessage(connectionType+"连接成功");
    //}
}

void MainWindow::on_connectButton_clicked()
{
    QString connectionType = ui->comboBoxConnectionType1->currentText();
    bool intendToConnect = (modbusDevice1->state() == QModbusDevice::UnconnectedState);
    if(intendToConnect){
        if (!modbusDevice1->connectDevice()) {
                statusBar()->showMessage(connectionType+"连接失败");
            } else {
                statusBar()->showMessage(connectionType+"连接成功");
            }
        } else {
        modbusDevice1->disconnectDevice();
        statusBar()->showMessage(connectionType+"连接断开");
    }
}
