#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    initSerial();
    setConnects();
    ui->GlOpenWin->show();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_x_axis_textChanged(const QString &arg1)
{
    ui->GlOpenWin->x_rotate(arg1.toFloat());
}

void MainWindow::on_y_axis_textChanged(const QString &arg1)
{
    ui->GlOpenWin->y_rotate(arg1.toFloat());
}

void MainWindow::on_z_axis_textChanged(const QString &arg1)
{
    ui->GlOpenWin->z_rotate(arg1.toFloat());
}

void MainWindow::initSerial()
{
    serial=new QSerialPort(this);
    foreach (QSerialPortInfo info, QSerialPortInfo::availablePorts())
    {
       ui->ports_comboBox->addItem(info.portName());
    }


    ui->baudRate_comboBox->addItem(QStringLiteral("115200"), QSerialPort::Baud115200);
    ui->baudRate_comboBox->addItem(QStringLiteral("57600"), QSerialPort::Baud57600);
    ui->baudRate_comboBox->addItem(QStringLiteral("38400"), QSerialPort::Baud38400);
    ui->baudRate_comboBox->addItem(QStringLiteral("19200"), QSerialPort::Baud19200);
    ui->baudRate_comboBox->addItem(QStringLiteral("9600"), QSerialPort::Baud9600);
}

void MainWindow::setConnects()
{
    connect(serial, SIGNAL(readyRead()), this, SLOT(readData()));
//    connect(serial,SIGNAL(bytesWritten(qint64)),this,SLOT(changeRanges(qint64)));
}

void MainWindow::print()
{
  //  QTextStream(stdout)<<" Roll(滚转x) :"<<linea.at(0)<<"Pitch(俯仰角y) :"<<linea.at(1)<<" Yaw(偏航z):"<<linea.at(2);
    ui->plainTextEdit->insertPlainText(serialReaded);
    QScrollBar *scrollbar = ui->plainTextEdit->verticalScrollBar();

    scrollbar->setValue(scrollbar->maximum());
}

void MainWindow::on_connectButton_clicked()
{
    datas.clear();
    serial->setPortName(ui->ports_comboBox->currentText());
    serial->setBaudRate(ui->baudRate_comboBox->currentText().toInt());


    serial->setStopBits(QSerialPort::OneStop);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    if(serial->open(QIODevice::ReadWrite))
    {
        serial->clear();
        ui->connectButton->setDisabled(true);
        ui->stopButton->setDisabled(false);

        ui->label_StatusConnect->setText("connected");
    }
    else
    {
          QMessageBox::critical(this, tr("Error"), serial->errorString());
     }


}

void MainWindow::on_stopButton_clicked()
{
    datas.clear();

    if(serial->isOpen())
    {
        serial->close();
        ui->connectButton->setDisabled(false);

         ui->label_StatusConnect->setText("DisConnected");
    }

}

void MainWindow::readData()
{
    bool ok;
   while(serial->canReadLine())
   {
       const QByteArray serialData = serial->readLine();
       serialReaded = QString(serialData);

       QStringList lineData = serialReaded.split(" ");

       print();
       if(lineData.size() == 3)
       {

           ui->GlOpenWin->x_rotate(QString(lineData.at(0)).toFloat(&ok)*(-10));
           ui->GlOpenWin->z_rotate(QString(lineData.at(1)).toFloat(&ok)*10);
           ui->GlOpenWin->y_rotate(QString(lineData.at(2)).toFloat(&ok)*10);
       }


   }
}

void MainWindow::writeData()
{

}
