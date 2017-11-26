#-------------------------------------------------
#
# Project created by QtCreator 2017-11-13T16:24:23
#
#-------------------------------------------------

QT       += core gui serialport opengl printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = opengl_test
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    openglwidget.cpp

HEADERS  += mainwindow.h \
    openglwidget.h

FORMS    += mainwindow.ui

LIBS += -lopengl32 -lglut

win32: LIBS += -L$$PWD/glut/ -lglut32
win32: LIBS += -L$$PWD/glut/ -lglut

INCLUDEPATH += $$PWD/glut
DEPENDPATH += $$PWD/glut
