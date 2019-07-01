include(../backend.pri)

TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += ../../../Common/System ../../../Common/Multimedia

SOURCES += \
        main.cpp \
        radiocontroller.cpp

HEADERS += \
    radiocontroller.h

LIBS += -L../../../CommonLibs/debug -lSystem -lMultimedia
