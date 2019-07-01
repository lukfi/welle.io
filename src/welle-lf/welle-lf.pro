include(../backend.pri)

TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += ../../../Common/System ../../../Common/Multimedia

SOURCES += \
#        Filter.cpp \
#        FmDecoder.cpp \
        main.cpp \
        radiocontroller.cpp \
        radioreceiverfm.cpp

HEADERS += \
#    Filter.h \
#    FmDecoder.h \
    radiocontroller.h \
    radioreceiverfm.h

LIBS += -L../../../CommonLibs/debug -lSystem -lMultimedia
