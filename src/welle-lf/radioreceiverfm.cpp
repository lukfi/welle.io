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

        if (fmOptions)
        {
            unsigned int downsample = std::max(1, int(fmOptions->mSample_rate_if / 215.0e3));
            mFmDecoder->CreateDecoder(fmOptions->mSample_rate_if,  // sample_rate_if
                                      fmOptions->mTuning_offset,   // tuning_offset
                                      fmOptions->mSample_rate_pcm, // sample_rate_pcm
                                      fmOptions->mStereo,          // stereo
                                      fmOptions->mDeemphasis,      // deemphasis,
                                      fmOptions->mBandwidth_if,    // bandwidth_if
                                      fmOptions->mFreq_dev,        // freq_dev
                                      fmOptions->mBandwidth_pcm,   // bandwidth_pcm
                                      downsample);
        }
        else // some wrong defaults
        {
            unsigned int downsample = std::max(1, int(INPUT_FM_RATE / 215.0e3));
            double offset = -1.0 * (0.25 * INPUT_FM_RATE);
            mFmDecoder->CreateDecoder(INPUT_FM_RATE,                     // sample_rate_if
                                      offset,                            // tuning_offset
                                      PCM_RATE,                          // sample_rate_pcm
                                      true,                              // stereo
                                      50,                                // deemphasis,
                                      FmDecoder::default_bandwidth_if,   // bandwidth_if
                                      FmDecoder::default_freq_dev,       // freq_dev
                                      FmDecoder::default_bandwidth_pcm,  // bandwidth_pcm
                                      downsample);
        }
    }
}

RadioReceiverFM::~RadioReceiverFM()
{
    SUCC("~RadioReceiverFM");
    if (mFmDecoder)
    {
        mFmDecoder->Stop();
        delete mFmDecoder;
    }
    SDEB("FM decoder stopped");
}

void RadioReceiverFM::Start(RadioReceiverFM::Mode_t mode, bool doScan)
{
    bool modeCahnged = (mode != mMode);
//    mMode = mode;
    if (modeCahnged)
    {
        SWAR("Swithcing to mode %s", mode == Mode_t::DVB ? "DVBT" : "FM");
    }

    if (mMode == Mode_t::DVB)
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

void RadioReceiverFM::ResetDecoderStats()
{
    if (mFmDecoder)
    {
        mFmDecoder->ResetDecoderStats();
    }
}
