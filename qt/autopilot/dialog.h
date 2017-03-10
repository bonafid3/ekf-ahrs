#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QByteArray>
#include <QVector>
#include <QQuaternion>

#include <QtOpenGL>

#include "cekf.h"

class QSerialPort;

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

    QMatrix3x3 ssc(QVector3D v);
    QMatrix3x3 getRM(QVector3D ref, QVector3D b);
private slots:
    void on_mConnectButton_clicked();

    void mSerialPort_readyRead();


    void on_mSaveQuatButton_clicked();

    void on_mScaleSlider_valueChanged(int value);

    void on_mAccelVarianceSpinBox_valueChanged(double arg1);

    void on_mProcessVarianceSpinBox_valueChanged(double arg1);

    void on_mMagXSlider_valueChanged(int value);

    void on_mMagYSlider_valueChanged(int value);

    void on_mMagZSlider_valueChanged(int value);

    void on_mSendMagCalibButton_clicked();

    void on_mCalcMagCalMatrix_clicked();

private:
    Ui::Dialog *ui;
    QByteArray mBuffer;
    QSerialPort *mSerialPort=0;

    bool mUpdateInnitial = true;
    QQuaternion mInitialQuat;

    short mMagXMin=30000;
    short mMagYMin=30000;
    short mMagZMin=30000;

    short mMagXMax=-30000;
    short mMagYMax=-30000;
    short mMagZMax=-30000;

    char *mPtr=0;

    QVector<GLfloat> mMagPoints;

    cEKF mEKF;
    tStateData gState;
    tRawSensorData mSensor;

    void updateQuaternion();
};

#endif // DIALOG_H
