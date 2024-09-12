#ifndef SERIALSETUP_H
#define SERIALSETUP_H

#include <QDialog>
#include <QSerialPort>
#include <QSerialPortInfo>

QT_BEGIN_NAMESPACE
namespace Ui { class serialsetup; }
QT_END_NAMESPACE

class serialsetup : public QDialog
{
    Q_OBJECT

public:
    explicit serialsetup(QWidget *parent = nullptr);
    ~serialsetup();

signals:
    void portSettingsChanged(const QString &portName, const QString &baudRate,
                                 const QString &dataBits, const QString &stopBits,
                                 const QString &parity, const QString &flowControl);

private slots:
    void connectToSerialPort();

private:
    Ui::serialsetup *ui;
    QSerialPort *serial;
    void loadAvailablePorts();
    void setupComboBoxes();
};
#endif // SERIALSETUP_H
