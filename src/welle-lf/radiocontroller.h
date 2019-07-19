#pragma once

#include "threads/systemtimer.h"
#include "threads/iothread.h"
#include "threads/event.h"
#include "audio/audiobuffer.h"
#include "utils/stopwatch.h"

#include "dab-constants.h"
//#include "radio-receiver.h"
#include "ringbuffer.h"
#include "channels.h"
#include "radioreceiverfm.h"

class RadioController : public RadioControllerInterface, public ProgrammeHandlerInterface
{
public:
    RadioController();

    void CloseDevice();

    CDeviceID OpenDevice();
//    void setDeviceParam(std::string param, int value);
//    void setDeviceParam(std::string param, std::string value);

    void play(std::string channel, std::string title, uint32_t service);
    void play(int fmFreq);

//    void pause();
    void stop();
    void setService(uint32_t service, bool force = false);
    void setChannel(std::string Channel, bool isScan, bool Force = false);
//    Q_INVOKABLE void setManualChannel(QString Channel);
    void startScan(bool backward = false);
    void startScan(RadioReceiverFM::Mode_t mode);
    void FmSeekNext();
    void FmSeekPrev();
    void stopScan(void);
//    void setAutoPlay(QString channel, QString serviceid_as_string);
//    Q_INVOKABLE void setVolume(qreal volume);
    void setAGC(bool isAGC);
//    Q_INVOKABLE void disableCoarseCorrector(bool disable);
//    Q_INVOKABLE void enableTIIDecode(bool enable);
    void selectFFTWindowPlacement(int fft_window_placement_ix);
//    Q_INVOKABLE void setFreqSyncMethod(int fsm_ix);
//    Q_INVOKABLE void setGain(int gain);
//    Q_INVOKABLE void initRecorder(int size);
//    Q_INVOKABLE void triggerRecorder(QString filename);
//    DABParams& getParams(void);
//    int getCurrentFrequency();

    void PrintMeasueres();
    void SetBuffer(LF::audio::AudioBuffer* audioBuffer) { mAudioBuffer = audioBuffer; }
public:
    //called from the backend
    virtual void onFrameErrors(int frameErrors) override;
    virtual void onNewAudio(std::vector<int16_t>&& audioData, int sampleRate, const std::string& mode) override;
    virtual void onRsErrors(bool uncorrectedErrors, int numCorrectedErrors) override;
    virtual void onAacErrors(int aacErrors) override;
    virtual void onNewDynamicLabel(const std::string& label) override;
    virtual void onMOT(const std::vector<uint8_t>& data, int subtype) override;
    virtual void onPADLengthError(size_t announced_xpad_len, size_t xpad_len) override;
    virtual void onFMRmsReport(double dbm, double lvl) override;

    virtual void onSNR(int snr) override;
    virtual void onFrequencyCorrectorChange(int fine, int coarse) override;
    virtual void onSyncChange(char isSync) override;
    virtual void onSignalPresence(bool isSignal) override;
    virtual void onServiceDetected(uint32_t sId) override;
    virtual void onNewEnsemble(uint16_t eId) override;
    virtual void onDateTimeUpdate(const dab_date_time_t& dateTime) override;
    virtual void onFIBDecodeSuccess(bool crcCheckOk, const uint8_t* fib) override;
    virtual void onNewImpulseResponse(std::vector<float>&& data) override;
    virtual void onConstellationPoints(std::vector<DSPCOMPLEX>&& data) override;
    virtual void onNewNullSymbol(std::vector<DSPCOMPLEX>&& data) override;
    virtual void onTIIMeasurement(tii_measurement_t&& m) override;
    virtual void onMessage(message_level_t level, const std::string& text) override;

private:
    enum class FmScanMode
    {
        NoScan,
        FullScan,
        FullScanBackward,
        ForwardScan,
        BackwardScan
    };

    void OpenDeviceInternal();
    void Initialise(void);
    void ResetTechnicalData(void);
    void DeviceRestart(void);
    void StartScanInternal(RadioReceiverFM::Mode_t mode, FmScanMode fmScanMode);

    void ChangeMode(RadioReceiverFM::Mode_t mode);

    void playdvb(std::string channel, std::string title, uint32_t service);
    void playfm(int fmFreq, bool scan);


    std::shared_ptr<CVirtualInput> mDevice;
//    QVariantMap commandLineOptions;
    Channels channels;
    RadioReceiverOptions rro;

    std::unique_ptr<RadioReceiverFM> mRadioReceiver;
    LF::audio::AudioBuffer* mAudioBuffer { nullptr };
//    RingBuffer<int16_t> audioBuffer;
//    CAudio mAudio;
    std::mutex impulseResponseBufferMutex;
    std::vector<float> impulseResponseBuffer;
    std::mutex nullSymbolBufferMutex;
    std::vector<DSPCOMPLEX> nullSymbolBuffer;
    std::mutex constellationPointBufferMutex;
    std::vector<DSPCOMPLEX> constellationPointBuffer;

    std::string errorMsg;
//    QDateTime currentDateTime;
    bool isSync = false;
    bool isFICCRC = false;
    bool isSignal = false;
    bool isDAB = false;
    std::string audioMode = "";
    int snr = 0;
    int frequencyCorrection = 0;
    float frequencyCorrectionPpm = 0.0;
    int bitRate = 0;
    int audioSampleRate = 0;
    int frameErrors = 0;
    int rsErrors = 0;
    int aaErrors = 0;
    int gainCount = 0;
    int stationCount = 0;
//    QImage motImage;

    std::string currentChannel;
    std::list<uint32_t> pendingLabels;
    std::string currentEnsembleLabel;
    uint16_t currentEId;
    int32_t currentFrequency;
    uint32_t currentService;
    std::string currentStationType;
    std::string currentLanguageType;
    std::string currentTitle;
    std::string currentText;
    int32_t currentManualGain;
    float currentManualGainValue = 0.0;
    double currentVolume = 1.0;
    std::string deviceName = "Unknown";
    CDeviceID deviceId = CDeviceID::UNKNOWN;

    LF::threads::IOThread mRadioControllerThread;
    LF::threads::SystemTimer labelTimer;
    LF::threads::SystemTimer stationTimer;
    LF::threads::SystemTimer channelTimer;

    LF::threads::Event mOpenDeviceEvent;

    bool isChannelScan = false;
    bool isAGC = false;
    bool isAutoPlay = false;
    std::string autoChannel;
    uint32_t autoService;

    RadioReceiverFM::Mode_t mMode { RadioReceiverFM::Mode_t::DVB };
    double mFmTunerFreq { 0 };
    struct FMRMS { double mRms; double mLvl; };
    std::map<int32_t, FMRMS> mFmScanRmsMap;
    std::set<int> mFmScanFound;
    FmScanMode mFmScanMode { FmScanMode::NoScan };

    LF::utils::StopWatch mScanningStopwatch;
//public slots:
//    void setErrorMessage(QString Text);
//    void setErrorMessage(const std::string& head, const std::string& text = "");
//    void setInfoMessage(QString Text);

//private slots:
    void ensembleId(uint16_t eId);
    void serviceId(uint32_t sId);
    void labelTimerTimeout(LF::threads::SystemTimer*);
    void stationTimerTimeout(LF::threads::SystemTimer*);
    void channelTimerTimeout(LF::threads::SystemTimer*);
    void nextChannel(bool isWait);
    void displayDateTime(const dab_date_time_t& dateTime);

//    void deviceNameChanged();
//    void deviceIdChanged();
//    void dateTimeChanged(QDateTime);
//    void isSyncChanged(bool);
//    void isFICCRCChanged(bool);
//    void isSignalChanged(bool);
//    void isDABChanged(bool);
//    void audioModeChanged(QString);
//    void snrChanged(int);
//    void frequencyCorrectionChanged(int);
//    void frequencyCorrectionPpmChanged(float);
//    void bitRateChanged(int);
//    void frameErrorsChanged(int);
//    void rsErrorsChanged(int);
//    void aacErrorsChanged(int);
//    void gainCountChanged(int);

//    void isHwAGCSupportedChanged(bool);
//    void hwAgcChanged(bool);
//    void agcChanged(bool);
//    void gainValueChanged(float);
//    void gainChanged(int);
//    void volumeChanged(qreal);
//    void motChanged(QImage MOTImage);

//    void channelChanged();
//    void ensembleChanged();
//    void frequencyChanged();
//    void stationChanged();
//    void stationTypChanged();
//    void titleChanged();
//    void textChanged();
//    void languageTypeChanged();

//    void deviceReady();
//    void deviceClosed();
//    void stationsCleared();
//    void foundStation(QString Station, QString currentChannel);
//    void newStationNameReceived(QString station, quint32 sId, QString channel);
//    void scanStopped();
//    void scanProgress(int Progress);
//    void showErrorMessage(QString Text);
//    void showInfoMessage(QString Text);
};
