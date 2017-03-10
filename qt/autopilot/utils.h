#ifndef UTILS_H
#define UTILS_H

#include <QVector3D>
#include <QDebug>

#define qd qDebug()
#include <stdint.h>

#define toStr(x) QString::number(x)

const QVector3D up(0,1,0);

struct sPlane {
    sPlane(QVector3D _n, QVector3D _p): n(_n), p(_p){}
    QVector3D n; //plane normal
    QVector3D p; //point on plane
};

struct sSensorData
{
    char pkt[3];

    int16_t mAccX;
    int16_t mAccY;
    int16_t mAccZ;

    float mGyroX;
    float mGyroY;
    float mGyroZ;

    int16_t mMagX;
    int16_t mMagY;
    int16_t mMagZ;

    int16_t mTemp;

    float mQ0;
    float mQ1;
    float mQ2;
    float mQ3;

};

#endif // UTILS_H

