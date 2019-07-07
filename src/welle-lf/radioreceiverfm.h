#ifndef RADIORECEIVERFM_H
#define RADIORECEIVERFM_H

#include "radio-receiver.h"
#include "FmDecoder.h"


class RadioReceiverFM : public RadioReceiver
{
public:
    enum class Mode_t
    {
        DVBT,
        FM
    };

    RadioReceiverFM(RadioControllerInterface& rci,
                    InputInterface* input,
                    RadioReceiverOptions rro,
                    int transmission_mode,// = 1,
                    Mode_t mode,// = Mode_t::DVBT,
                    FmDecoderThreadWelle::FmDecoderOptions* fmOptions = nullptr);
    ~RadioReceiverFM();

    void Start(Mode_t mode, bool doScan);

private:
    InputInterface* mInput;
    FmDecoderThreadWelle* mFmDecoder;
    Mode_t mMode { Mode_t::DVBT };
};

#endif // RADIORECEIVERFM_H
