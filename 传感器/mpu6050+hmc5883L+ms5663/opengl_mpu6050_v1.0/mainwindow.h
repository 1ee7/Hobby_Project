#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_x_axis_textChanged(const QString &arg1);

    void on_y_axis_textChanged(const QString &arg1);

    void on_z_axis_textChanged(const QString &arg1);

    void on_connectButton_clicked();

    void on_stopButton_clicked();

    void readData();
    void writeData();

private:
    void initSerial();
    void setConnects();
    void print();

private:
    Ui::MainWindow *ui;
    QSerialPort *serial;
    QStringList datas;
    QString serialReaded;
};

#endif // MAINWINDOW_H
