QT += gui network concurrent

CONFIG += c++11 console
CONFIG -= app_bundle
CONFIG += link_pkgconfig
PKGCONFIG += msgpack opencv gstreamer-base-1.0 gstreamer-app-1.0

DEFINES += QT_DEPRECATED_WARNINGS
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

QMAKE_CXXFLAGS += -march=native -mtune=native -O3

SOURCES += \
        avahibrowser.cpp \
        main.cpp \
        markerdetector.cpp \
        rpicamera.cpp \
        rpimocapclient.cpp

LIBS += -lmosquitto -lmosquittopp

LIBS += -L$$OUT_PWD/../RPIMoCapBase/ -lRPIMoCapBase
INCLUDEPATH += $$PWD/../RPIMoCapBase
DEPENDPATH += $$PWD/../RPIMoCapBase

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    avahibrowser.h \
    markerdetector.h \
    rpicamera.h \
    rpimocapclient.h
