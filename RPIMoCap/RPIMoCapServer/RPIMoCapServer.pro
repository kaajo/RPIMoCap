#-------------------------------------------------
#
# Project created by QtCreator 2019-08-02T19:02:49
#
#-------------------------------------------------

QT += core gui widgets 3dcore 3drender 3dinput 3dlogic 3dextras

TARGET = RPIMoCapServer
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++17
CONFIG += link_pkgconfig

SOURCES += \
        camerasettingswidget.cpp \
        linesaggregator.cpp \
        main.cpp \
        mainwindow.cpp \
        mocapscene3d.cpp \
        munkres.cpp \
        pointchecker.cpp \
        rpimocapserver.cpp

HEADERS += \
        camerasettings.h \
        camerasettingswidget.h \
        linesaggregator.h \
        mainwindow.h \
        matrix.h \
        matrix.tpp \
        mocapscene3d.h \
        munkres.h \
        pointchecker.h \
        rpimocapserver.h

FORMS += \
        camerasettingswidget.ui \
        mainwindow.ui \
        mocapscene3d.ui

PKGCONFIG += msgpack opencv
LIBS += -lmosquitto -lmosquittopp

LIBS += -L$$OUT_PWD/../RPIMoCapBase/ -lRPIMoCapBase
INCLUDEPATH += $$PWD/../RPIMoCapBase
DEPENDPATH += $$PWD/../RPIMoCapBase

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

