#-------------------------------------------------
#
# Project created by QtCreator 2017-02-04T22:22:04
#
#-------------------------------------------------

QT       += core gui opengl serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = autopilot
TEMPLATE = app

LIBS += opengl32.lib

SOURCES += main.cpp\
        dialog.cpp \
    cglwidget.cpp \
    cekf.cpp \
    cquaternion.cpp \
    cmatrix.cpp

HEADERS  += dialog.h \
    utils.h \
    cglwidget.h \
    cekf.h \
    cquaternion.h \
    cmatrix.h

FORMS    += dialog.ui

DISTFILES += \
    vshader.vert \
    fshader.frag

RESOURCES += \
    res.qrc
