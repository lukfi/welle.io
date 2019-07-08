#ifndef RADIORECEIVERFM_H
#define RADIORECEIVERFM_H

#include "radio-receiver.h"
#include "FmDecoder.h"

#define INPUT_FM_RATE (1.2e6)
#define PCM_RATE (48000)

class RadioReceiverFM : public RadioReceiver
{
public:
    enum class Mode_t
    {
        DVBT,
        FM
    };

    RadioReceiverFM(RadioControllerInterface& rci,
                    ProgrammeHandlerInterface& output,
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
