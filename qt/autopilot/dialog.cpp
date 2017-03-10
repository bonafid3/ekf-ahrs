#include "dialog.h"
#include "ui_dialog.h"

#include <QSerialPort>
#include <QSerialPortInfo>

#include "utils.h"
#include <QMessageBox>
#include <QByteArray>

#include <QQuaternion>


#include "cmatrix.h"

#include "cekf.h"

struct sSettings
{
    sSettings(){ pkt[0]='a'; pkt[1]='b'; pkt[2]='c'; }
    char pkt[3];

    cMatrix<3,3> mMagCal;

    float mMagXoffset;
    float mMagYoffset;
    float mMagZoffset;
};

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);

    foreach(QSerialPortInfo spi, QSerialPortInfo::availablePorts()) {
        qd << spi.portName();
        ui->mSerialPortsCombo->addItem(spi.portName());
    }

    ui->splitter->setStretchFactor(0, 10);
    ui->splitter->setStretchFactor(1, 1);


    cMatrix<4,4> Sigma;
    cMatrix<1,4> C;
    cMatrix<4,1> Ctranspose;

    C.data[0][0] = 1;
    C.data[0][1] = 2;
    C.data[0][2] = 3;
    C.data[0][3] = 4;

    Ctranspose = C.transposed();

    qd.noquote() << C * Sigma;

    qd.noquote() << C * Sigma * Ctranspose;


    cMatrix<3,3> m;

    m.data[0][0] = 4;
    m.data[0][1] = -3;
    m.data[0][2] = 0;

    m.data[1][0] = 2;
    m.data[1][1] = -1;
    m.data[1][2] = 2;

    m.data[2][0] = 1;
    m.data[2][1] = 5;
    m.data[2][2] = 7;

    qd << "determinant:" << m.determinant();

    gState.accel_ref_x = 0;
    gState.accel_ref_y = 0;
    gState.accel_ref_z = 1.0;

    gState.accel_var = 0.01; // smaller means more sensitive




    gState.mag_ref_x = -0.55;
    gState.mag_ref_y = 0.06;
    gState.mag_ref_z = -0.83;


    gState.mag_var = 0.001;

    gState.process_var = 1.0; // 0.1
}

Dialog::~Dialog()
{
    delete ui;
}

QMatrix3x3 Dialog::ssc(QVector3D v) {
    QMatrix3x3 ssc;

    ssc(0,0) = 0.f;
    ssc(1,0) = v.z();
    ssc(2,0) = -v.y();

    ssc(0,1) = -v.z();
    ssc(1,1) = 0.f;
    ssc(2,1) = v.x();

    ssc(0,2) = v.y();
    ssc(1,2) = -v.x();
    ssc(2,2) = 0.f;

    return ssc;
}

QMatrix3x3 Dialog::getRM(QVector3D a, QVector3D b)
{
    // this code calculates the rotation matrix between two normalized vectors
    // the vectors must not be in the same line
    QMatrix3x3 i;
    i.setToIdentity();

    a.normalize();
    b.normalize();

    QMatrix3x3 sscm = ssc(QVector3D::crossProduct(a,b));
    float dp = 1 - QVector3D::dotProduct(a,b);

    float cpl = QVector3D::crossProduct(a, b).length();

    QMatrix3x3 summand1 = i + sscm;
    QMatrix3x3 summand2 = (sscm*sscm * dp) / (cpl*cpl);

    return summand1 + summand2;
}

void Dialog::on_mConnectButton_clicked()
{

    if(mSerialPort == 0) {
        if(ui->mSerialPortsCombo->currentIndex() != -1) {
            mSerialPort = new QSerialPort(ui->mSerialPortsCombo->currentText());
            mSerialPort->setBaudRate(230400);
            connect(mSerialPort, SIGNAL(readyRead()), this, SLOT(mSerialPort_readyRead()));
            if(!mSerialPort->open(QIODevice::ReadWrite)) {
                QMessageBox::warning(this, "Error", "Can't open serial port!");
            }
            qd << "opened";
            ui->mConnectButton->setText("Disconnect");
        }
    } else {
        mSerialPort->close();
        mSerialPort->disconnect();
        mSerialPort->deleteLater();
        mSerialPort = 0;
        ui->mConnectButton->setText("Connect");
        qd << "closed";
    }
}

void Dialog::mSerialPort_readyRead()
{
    sSensorData sd;
    QByteArray ba  = mSerialPort->readAll();

    if(ba.size() != 48) { return; }

    if(ba.size() == sizeof(sSensorData)) {
        char *ptr = (char*)&sd;
        for(int i=0; i<sizeof(sSensorData); i++) {
            ptr[i] = ba.at(i);
        }

        //qd << mBuffer.size() << sizeof(sSensorData);

        if(ui->mSensorDataGroupBox->isChecked()) {
            ui->mAccX->setText(toStr(sd.mAccX));
            ui->mAccY->setText(toStr(sd.mAccY));
            ui->mAccZ->setText(toStr(sd.mAccZ));

            QVector3D av(sd.mAccX, sd.mAccY, sd.mAccZ);
            QMatrix3x3 rm = getRM(QVector3D(0,0,1), av);

            qd << rm;

            ui->mGyroX->setText(toStr(sd.mGyroX));
            ui->mGyroY->setText(toStr(sd.mGyroY));
            ui->mGyroZ->setText(toStr(sd.mGyroZ));

            ui->mMagX->setText(toStr(sd.mMagX));
            ui->mMagY->setText(toStr(sd.mMagY));
            ui->mMagZ->setText(toStr(sd.mMagZ));

            mMagXMin = qMin(mMagXMin, sd.mMagX);
            mMagYMin = qMin(mMagYMin, sd.mMagY);
            mMagZMin = qMin(mMagZMin, sd.mMagZ);

            mMagXMax = qMax(mMagXMax, sd.mMagX);
            mMagYMax = qMax(mMagYMax, sd.mMagY);
            mMagZMax = qMax(mMagZMax, sd.mMagZ);

            ui->mMagXMin->setText(toStr(mMagXMin));
            ui->mMagYMin->setText(toStr(mMagYMin));
            ui->mMagZMin->setText(toStr(mMagZMin));

            ui->mMagXMax->setText(toStr(mMagXMax));
            ui->mMagYMax->setText(toStr(mMagYMax));
            ui->mMagZMax->setText(toStr(mMagZMax));



            QVector3D nv(sd.mMagX, sd.mMagY, sd.mMagZ);

            if(mMagPoints.size()) {
                int i=mMagPoints.size()-3;
                QVector3D ov(mMagPoints.at(i),mMagPoints.at(i+1),mMagPoints.at(i+2));
                QVector3D dv = ov-nv;
                //qd << ov << nv << dv << dv.length();
                if(dv.length() > 20) {
                    mMagPoints.append(sd.mMagX);
                    mMagPoints.append(sd.mMagY);
                    mMagPoints.append(sd.mMagZ);
                }
            } else {
                mMagPoints.append(sd.mMagX);
                mMagPoints.append(sd.mMagY);
                mMagPoints.append(sd.mMagZ);
            }

            gState.gyro_x = sd.mGyroX;
            gState.gyro_y = sd.mGyroY;
            gState.gyro_z = sd.mGyroZ;

/*
            gState.gyro_x = 0;
            gState.gyro_y = 0;
            gState.gyro_z = 0;
*/


            float length = sqrt(sd.mAccX * sd.mAccX + sd.mAccY * sd.mAccY + sd.mAccZ * sd.mAccZ);

            gState.accel_x = sd.mAccX / length;
            gState.accel_y = sd.mAccY / length;
            gState.accel_z = sd.mAccZ / length;

            length = sqrt(sd.mMagX * sd.mMagX + sd.mMagY * sd.mMagY + sd.mMagZ * sd.mMagZ);

            gState.mag_x = sd.mMagX / length;
            gState.mag_y = sd.mMagY / length;
            gState.mag_z = sd.mMagZ / length;

            cMatrix<3,1> svec;

            svec.data[0][0] = sd.mMagX / length;
            svec.data[0][1] = sd.mMagZ / length;
            svec.data[0][2] = sd.mMagY / length;

            svec = gState.mag_cal * svec;

            gState.mag_x = svec.data[0][0];
            gState.mag_y = svec.data[0][1];
            gState.mag_z = svec.data[0][2];

            float mrx = gState.mag_ref_x;
            float mry = gState.mag_ref_y;
            float mrz = gState.mag_ref_z;

            length = sqrt(mrx*mrx + mry*mry + mrz*mrz);
            gState.mag_ref_x = mrx / length;
            gState.mag_ref_y = mry / length;
            gState.mag_ref_z = mrz / length;

            mEKF.estimateStates(gState, mSensor);
            //qd << gState.qib.a << gState.qib.b << gState.qib.c << gState.qib.d;

            ui->mGLWidget->createPointVBO(mMagPoints);

            ui->mMagBiasX->setText(toStr((mMagXMin+mMagXMax)/2));
            ui->mMagBiasY->setText(toStr((mMagYMin+mMagYMax)/2));
            ui->mMagBiasZ->setText(toStr((mMagZMin+mMagZMax)/2));

            //qd << "magx" << (mMagXMin+mMagXMax)/2 << "magy" << (mMagYMin+mMagYMax)/2 << "magz" << (mMagZMin+mMagZMax)/2;

            ui->mTemp->setText(toStr(sd.mTemp));

            ui->mQ0->setText(toStr(sd.mQ0));
            ui->mQ1->setText(toStr(sd.mQ1));
            ui->mQ2->setText(toStr(sd.mQ2));
            ui->mQ3->setText(toStr(sd.mQ3));
        }

        QQuaternion q(sd.mQ0, sd.mQ1, sd.mQ2, sd.mQ3);

        //QQuaternion q(gState.qib.a, gState.qib.b, gState.qib.c, gState.qib.d);



        if(mUpdateInnitial) {
            mInitialQuat = q.conjugated();
            ui->mGLWidget->setAttitudeQuat(q);
        } else {
            ui->mGLWidget->setAttitudeQuat(mInitialQuat * q);
        }

        ui->mGLWidget->update();

    }

}

void Dialog::on_mSaveQuatButton_clicked()
{
    mUpdateInnitial = false;
}

void Dialog::on_mScaleSlider_valueChanged(int value)
{
    ui->mGLWidget->setPointScale(value);
    ui->mGLWidget->update();
}

void Dialog::on_mAccelVarianceSpinBox_valueChanged(double arg1)
{
    gState.accel_var = arg1;
}

void Dialog::on_mProcessVarianceSpinBox_valueChanged(double arg1)
{
    gState.process_var = arg1;
}

void Dialog::on_mMagXSlider_valueChanged(int value)
{
    float val = value / 100.0;
    gState.mag_ref_x = val;

}

void Dialog::on_mMagYSlider_valueChanged(int value)
{
    float val = value / 100.0;
    gState.mag_ref_y = val;
}

void Dialog::on_mMagZSlider_valueChanged(int value)
{
    float val = value / 100.0;
    gState.mag_ref_z = val;
    qd << "ZVAL:" << val;
}

void Dialog::on_mSendMagCalibButton_clicked()
{
    sSettings s; bool ok=false;
    s.mMagCal.data[0][0] = ui->mMag11->text().toFloat(&ok);
    s.mMagCal.data[0][1] = ui->mMag12->text().toFloat(&ok);
    s.mMagCal.data[0][2] = ui->mMag13->text().toFloat(&ok);

    s.mMagCal.data[1][0] = ui->mMag21->text().toFloat(&ok);
    s.mMagCal.data[1][1] = ui->mMag22->text().toFloat(&ok);
    s.mMagCal.data[1][2] = ui->mMag23->text().toFloat(&ok);

    s.mMagCal.data[2][0] = ui->mMag31->text().toFloat(&ok);
    s.mMagCal.data[2][1] = ui->mMag32->text().toFloat(&ok);
    s.mMagCal.data[2][2] = ui->mMag33->text().toFloat(&ok);

    s.mMagXoffset = ui->mMagBiasX->text().toFloat(&ok);
    s.mMagYoffset = ui->mMagBiasY->text().toFloat(&ok);
    s.mMagZoffset = ui->mMagBiasZ->text().toFloat(&ok);

    if(ok) {
        mPtr = (char*)&s;
        mSerialPort->write(mPtr, sizeof(sSettings));
    }
}

void Dialog::on_mCalcMagCalMatrix_clicked()
{
    float biasX = (mMagXMin+mMagXMax)/2;
    float biasY = (mMagYMin+mMagYMax)/2;
    float biasZ = (mMagZMin+mMagZMax)/2;

    float magx = mMagXMax - biasX;
    float magy = mMagYMax - biasY;
    float magz = mMagZMax - biasZ;

    ui->mMag11->setText(toStr(1));
    ui->mMag22->setText(toStr(magx / magy));
    ui->mMag33->setText(toStr(magx / magz));
}
