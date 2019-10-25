include(backend.pri)

INCLUDEPATH += $$PWD/../../Common/Multimedia

INCLUDEPATH += \
    $$PWD/welle-lf

SOURCES += \
        $$PWD/welle-lf/Filter.cpp \
        $$PWD/welle-lf/FmDecoder.cpp \
        $$PWD/welle-lf/radiocontroller.cpp \
        $$PWD/welle-lf/radioreceiverfm.cpp

HEADERS += \
    $$PWD/welle-lf/Filter.h \
    $$PWD/welle-lf/FmDecoder.h \
    $$PWD/welle-lf/radiocontroller.h \
    $$PWD/welle-lf/radioreceiverfm.h

LIBS += -L$$PWD/../../CommonLibs/debug -lSystem -lMultimedia

unix {
LIBS += -pthread -lpulse -lpulse-simple -lasound -latomic
}
