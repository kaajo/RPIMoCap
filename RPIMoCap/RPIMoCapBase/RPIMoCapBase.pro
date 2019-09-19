#-------------------------------------------------
#
# Project created by QtCreator 2019-08-02T18:56:39
#
#-------------------------------------------------

CONFIG += c++14

TARGET = RPIMoCapBase
TEMPLATE = lib

DEFINES += RPIMOCAPBASE_LIBRARY

DEFINES += QT_DEPRECATED_WARNINGS
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

QMAKE_CXXFLAGS += -march=native -mtune=native -O3

SOURCES += \
        frame.cpp \
        line3d.cpp \
        mqttpublisher.cpp \
        mqttsettings.cpp \
        mqttsubscriber.cpp

HEADERS += \
        frame.h \
        line3d.h \
        mqttpublisher.h \
        mqttsettings.h \
        mqttsubscriber.h \
        msgpack_defs.h \
        rpimocapbase_global.h

LIBS += -lmosquitto -lmosquittopp

unix {
    target.path = /usr/lib
    INSTALLS += target
}
