#include "radioreceiverfm.h"

#define ENABLE_SDEBUG
#define DEBUG_PREFIX "RadioReceiverFM: "
#include "utils/screenlogger.h"

RadioReceiverFM::RadioReceiverFM(RadioControllerInterface& rci,
                                 ProgrammeHandlerInterface& output,
                                 InputInterface* input,
                                 RadioReceiverOptions rro,
                                 int transmission_mode,
                                 Mode_t mode, FmDecoderThreadWelle::FmDecoderOptions* fmOptions) : RadioReceiver (rci, *input, rro, transmission_mode),
    mInput(input),
    mFmDecoder(nullptr),
    mMode(mode)

{
    if (mode == Mode_t::FM)
    {
        mFmDecoder = new FmDecoderThreadWelle(mInput, &output);
        FmDecoderThreadWelle::FmDecoderOptions options;

        double ifrate = 1.2e6;
        unsigned int downsample = std::max(1, int(ifrate / 215.0e3));

        mFmDecoder->CreateDecoder(ifrate,
                                  -300000,
                                  48000,
                                  true,
                                  50,
                                  100000,
                                  75000,
                                  15000,
                                  downsample);
    }
}

RadioReceiverFM::~RadioReceiverFM()
{
    SUCC("~RadioReceiverFM");
    if (mFmDecoder)
    {
        mFmDecoder->Stop();
    }
    SDEB("FM decoder stopped");
}

void RadioReceiverFM::Start(RadioReceiverFM::Mode_t mode, bool doScan)
{
    bool modeCahnged = (mode != mMode);
//    mMode = mode;
    if (modeCahnged)
    {
        SWAR("Swithcing to mode %s", mode == Mode_t::DVBT ? "DVBT" : "FM");
    }

    if (mMode == Mode_t::DVBT)
    {
//        if (mFmDecoder)
//        {
//            delete mFmDecoder;
//            mFmDecoder = nullptr;
//        }
        restart(doScan);
    }
    else // FM
    {
        mscHandler.stopProcessing();
        ficHandler.clearEnsemble();
        ofdmProcessor.stop();
//        if (mFmDecoder)
//        {
//            delete mFmDecoder;
//        }
//        mFmDecoder = new FmDecoderThreadWelle(mInput);
//        FmDecoderThreadWelle::FmDecoderOptions options;
//        mFmDecoder->CreateDecoder(1.2e6,
//                                 0,
//                                 48000,
//                                 true);
        mFmDecoder->Start(doScan);
    }
}
