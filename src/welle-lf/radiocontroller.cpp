#include "radiocontroller.h"
#include "input_factory.h"

#include "utils/stringutils.h"
#include "threads/threadutils.h"

#define ENABLE_SDEBUG
#define DEBUG_PREFIX "RadioController: "
#include "utils/screenlogger.h"

#define LF_ASSERT_ON
#include "utils/assert.h"

RadioController::RadioController() :
    mRadioControllerThread("RadioControllerThread"),
    labelTimer(40, LF::threads::TimerType::Periodic),
    stationTimer(1000, LF::threads::TimerType::Periodic),
    channelTimer(1000, LF::threads::TimerType::SingleShot)
{
    SDEB("RadioController");
    // Init the technical data
    ResetTechnicalData();

    // Init timers
    //connect(&labelTimer, &QTimer::timeout, this, &CRadioController::labelTimerTimeout);
    CONNECT(labelTimer.TIMEOUT, RadioController, labelTimerTimeout);
    mRadioControllerThread.SheduleTimer(&labelTimer);

    //connect(&stationTimer, &QTimer::timeout, this, &CRadioController::stationTimerTimeout);
    CONNECT(stationTimer.TIMEOUT, RadioController, stationTimerTimeout);
    mRadioControllerThread.SheduleTimer(&stationTimer);

    //connect(&channelTimer, &QTimer::timeout, this, &CRadioController::channelTimerTimeout);
    CONNECT(channelTimer.TIMEOUT, RadioController, channelTimerTimeout);
    mRadioControllerThread.SheduleTimer(&channelTimer);

    // Use the signal slot mechanism is necessary because the backend runs in a different thread
//    connect(this, &CRadioController::switchToNextChannel,
//            this, &CRadioController::nextChannel);

//    connect(this, &CRadioController::ensembleIdUpdated,
//            this, &CRadioController::ensembleId);

//    connect(this, &CRadioController::serviceDetected,
//            this, &CRadioController::serviceId);

//    qRegisterMetaType<dab_date_time_t>("dab_date_time_t");
//    connect(this, &CRadioController::dateTimeUpdated,
//            this, &CRadioController::displayDateTime);
    mRadioControllerThread.Start(true);
    if (mRadioControllerThread.Running())
    {
        SINFO("Started thread id: %d", mRadioControllerThread.GetId());
    }
}

void RadioController::CloseDevice()
{
    SDEB("Close device");

    mRadioReceiver.reset();
    mDevice.reset();
//    mAudio.reset();

    // Reset the technical data
    ResetTechnicalData();

    //    emit deviceClosed();
}

CDeviceID RadioController::OpenDevice()
{
    SCHEDULE_TASK(&mRadioControllerThread, &RadioController::OpenDeviceInternal, this);
    if (mOpenDeviceEvent.Wait(3000))
    {
//        SDEB("OpenDeviceInternal raised event");
        return mDevice->getID();
    }
    SWAR("NULLDEVICE");
    return CDeviceID::NULLDEVICE;
}

void RadioController::play(std::string channel, std::string title, uint32_t service)
{
    EXECUTE_IN_THREAD(&mRadioControllerThread, &RadioController::play, this, channel, title, service);
    if (channel == "") {
        return;
    }

    currentTitle = title;
//    emit titleChanged();

    if (mMode == RadioReceiverFM::Mode_t::FM)
    {
        ResetTechnicalData();
        mMode = RadioReceiverFM::Mode_t::DVBT;
        mDevice->setDeviceParam(DeviceParam::AGC, 0);
        mDevice->setDeviceParam(DeviceParam::InputFreq, 2048000);
    }


    SUCC("Play: %s %x on channel %d", title.c_str(), service, channel.c_str());

    if (isChannelScan == true)
    {
        stopScan();
    }
    DeviceRestart();
    setChannel(channel, false);
    setService(service);

//    QSettings settings;
    //settings.setValue("lastchannel", QStringList() << serialise_serviceid(service) << channel);
}

void RadioController::playfm(int fmFreq)
{
    SDEB("playfm, th %d", LF::threads::GetThisThreadId());
    EXECUTE_IN_THREAD(&mRadioControllerThread, &RadioController::playfm, this, fmFreq);

    if (mMode == RadioReceiverFM::Mode_t::DVBT)
    {
        ResetTechnicalData();
        mMode = RadioReceiverFM::Mode_t::FM;
        mDevice->setDeviceParam(DeviceParam::AGC, 1);
        mDevice->setDeviceParam(DeviceParam::InputFreq, 1.2e6);
    }

    if (isChannelScan == true)
    {
        stopScan();
        channelTimer.Stop();
    }

    DeviceRestart();
    if (mDevice && mDevice->getID() == CDeviceID::RAWFILE)
    {
        currentChannel = "File";
        currentEId = 0;
        currentEnsembleLabel = "";
        currentFrequency = 0;
    }
    else
    { // A real device
        // Convert channel into a frequency
        currentFrequency = fmFreq;

        if(currentFrequency != 0 && mDevice)
        {
            SDEB("Tune to freq: %fMHz", currentFrequency / 1e6);
            mDevice->setFrequency(currentFrequency);
//            mDevice->setDeviceParam(DeviceParam::InputFreq, 1.2e6);
            mDevice->reset(); // Clear buffer
        }
    }

    // Restart demodulator and decoder
    mRadioReceiver = nullptr;
    mRadioReceiver = std::make_unique<RadioReceiverFM>(*this, mDevice.get(), rro, 1, RadioReceiverFM::Mode_t::FM);
    mRadioReceiver->setReceiverOptions(rro);
    mRadioReceiver->Start(RadioReceiverFM::Mode_t::FM, false);
}

void RadioController::stop()
{
    if (mDevice) {
        mDevice->stop();
    }
    else
        throw std::runtime_error("device is null in file " + std::string(__FILE__) +":"+ std::to_string(__LINE__));

    //audio.reset();
    labelTimer.Stop();
}

void RadioController::setService(uint32_t service, bool force)
{
    SDEB("setService");
    if (currentService != service or force) {
        currentService = service;
//        emit stationChanged();

        // Wait if we found the station inside the signal
        stationTimer.Start(1000);

        // Clear old data
        currentStationType = "";
//        emit stationTypChanged();

        currentLanguageType = "";
//        emit languageTypeChanged();

        currentText = "";
//        emit textChanged();

//        motImage.loadFromData(nullptr, 0);
//        emit motChanged(motImage);
    }
}

void RadioController::setChannel(std::string Channel, bool isScan, bool Force)
{
    SDEB("setChannel, th %d", LF::threads::GetThisThreadId());
    EXECUTE_IN_THREAD(&mRadioControllerThread, &RadioController::setChannel, this, Channel, isScan, Force);

    if (currentChannel != Channel || Force == true) {
        if (mDevice && mDevice->getID() == CDeviceID::RAWFILE) {
            currentChannel = "File";
            currentEId = 0;
            currentEnsembleLabel = "";
            currentFrequency = 0;
        }
        else { // A real device
            currentChannel = Channel;
            currentEId = 0;
            currentEnsembleLabel = "";

            // Convert channel into a frequency
            currentFrequency = channels.getFrequency(Channel);

            if(currentFrequency != 0 && mDevice) {
                SDEB("Tune to channel: %s -> %fMHz", Channel.c_str(), currentFrequency/1e6);
                mDevice->setFrequency(currentFrequency);
                mDevice->reset(); // Clear buffer
            }
        }

        // Restart demodulator and decoder
        SERR("befor new receiver!!");
        mRadioReceiver = nullptr;
        mRadioReceiver = std::make_unique<RadioReceiverFM>(*this, mDevice.get(), rro, 1, RadioReceiverFM::Mode_t::DVBT);
        SERR("new receiver!!");
        mRadioReceiver->setReceiverOptions(rro);
        mRadioReceiver->Start(RadioReceiverFM::Mode_t::DVBT, isScan);

//        emit channelChanged();
//        emit ensembleChanged();
//        emit frequencyChanged();
    }
    else
    {
        SDEB("NOT");
    }
}

void RadioController::startScan()
{
    SDEB("Start channel scan, th %d", LF::threads::GetThisThreadId());
    EXECUTE_IN_THREAD(&mRadioControllerThread, &RadioController::startScan, this);

    DeviceRestart();

    if(mDevice && mDevice->getID() == CDeviceID::RAWFILE) {
        currentTitle = "RAW File";
        const auto FirstChannel = Channels::firstChannel;
        setChannel(FirstChannel, false); // Just a dummy
//        emit scanStopped();
    }
    else
    {
        // Start with lowest frequency
//        std::string Channel = Channels::firstChannel;
        std::string Channel = "11B";
        setChannel(Channel, true);

        isChannelScan = true;
        stationCount = 0;
        currentTitle = LF::utils::sformat("Scanning ... %s (%d\\%)", Channel.c_str(), (1 * 100 / NUMBEROFCHANNELS));
//        emit titleChanged();

        currentText = LF::utils::sformat("Found channels: %d", stationCount);
//        emit textChanged();

        currentService = 0;
//        emit stationChanged();

        currentStationType = "";
//        emit stationTypChanged();

        currentLanguageType = "";
//        emit languageTypeChanged();

//        emit scanProgress(0);
    }
}

void RadioController::stopScan()
{
    SINFO("Stop channel scan");

    currentTitle = "No Station";
//    emit titleChanged();

    currentText = "";
//    emit textChanged();

    isChannelScan = false;
    //    emit scanStopped();
}

void RadioController::setAGC(bool isAGC)
{
    this->isAGC = isAGC;

    if (mDevice) {
        mDevice->setAgc(isAGC);

        if (!isAGC) {
            mDevice->setGain(currentManualGain);
        }
        SDEB("AGC %s", isAGC ? "on" : "off");
    }

//    emit agcChanged(isAGC);
}

void RadioController::selectFFTWindowPlacement(int fft_window_placement_ix)
{
    if (fft_window_placement_ix == 0) {
        rro.fftPlacementMethod = FFTPlacementMethod::StrongestPeak;
    }
    else if (fft_window_placement_ix == 1) {
        rro.fftPlacementMethod = FFTPlacementMethod::EarliestPeakWithBinning;
    }
    else if (fft_window_placement_ix == 2) {
        rro.fftPlacementMethod = FFTPlacementMethod::ThresholdBeforePeak;
    }
    else {
        SERR("Invalid FFT window placement %d chosen", fft_window_placement_ix);
        return;
    }

    if (mRadioReceiver) {
        mRadioReceiver->setReceiverOptions(rro);
    }
}

void RadioController::PrintMeasueres()
{
    SDEB("GAIN: %d", mDevice->getGain());
}

void RadioController::onFrameErrors(int frameErrors)
{
    if (this->frameErrors == frameErrors)
        return;
    this->frameErrors = frameErrors;
    //    emit frameErrorsChanged(this->frameErrors);
}

void RadioController::onNewAudio(std::vector<int16_t>&& audioData, int sampleRate, const std::string& mode)
{
    uint32_t copied = mAudioBuffer->PushFramesBytes((uint8_t*)audioData.data(), audioData.size() * 2);
    //SDEB("Audio data %dHz: %s %d", sampleRate, copied ? " OK" : "NOK", audioData.size());
    //audioBuffer.putDataIntoBuffer(audioData.data(), static_cast<int32_t>(audioData.size()));

    if (audioSampleRate != sampleRate) {
        SDEB("Audio sample rate: %d Hz, mode=%s", sampleRate, mode.c_str());
        audioSampleRate = sampleRate;

//        audio.setRate(sampleRate);
    }

    if (audioMode != mode) {
        audioMode = mode;
//        emit audioModeChanged(audioMode);
    }
}

void RadioController::onRsErrors(bool uncorrectedErrors, int numCorrectedErrors)
{
    (void)numCorrectedErrors;

    if (this->rsErrors == uncorrectedErrors ? 1 : 0)
        return;
    this->rsErrors = uncorrectedErrors ? 1 : 0;
    //    emit rsErrorsChanged(this->rsErrors);
}

void RadioController::onAacErrors(int aacErrors)
{
    if (this->aaErrors == aacErrors)
        return;
    this->aaErrors = aacErrors;
    //    emit aacErrorsChanged(this->aaErrors);
}

void RadioController::onNewDynamicLabel(const std::string& label)
{
    auto qlabel = label;//QString::fromUtf8(label.c_str());
    if (this->currentText != qlabel) {
        this->currentText = qlabel;
//        emit textChanged();
    }
}

void RadioController::onMOT(const std::vector<uint8_t>& data, int subtype)
{
//    SDEB("On MOT!");
//    QByteArray qdata(reinterpret_cast<const char*>(Data.data()), static_cast<int>(Data.size()));
//    motImage.loadFromData(qdata, subtype == 0 ? "GIF" : subtype == 1 ? "JPEG" : subtype == 2 ? "BMP" : "PNG");

    //    emit motChanged(motImage);
}

void RadioController::onPADLengthError(size_t announced_xpad_len, size_t xpad_len)
{
    SWAR("X-PAD length mismatch, expected: %lld effective: %lld", announced_xpad_len, xpad_len);
}

void RadioController::onSNR(int snr)
{
    if (this->snr == snr)
        return;
    this->snr = snr;
    //    emit snrChanged(this->snr);
}

void RadioController::onFrequencyCorrectorChange(int fine, int coarse)
{
    if (frequencyCorrection == coarse + fine)
        return;
    frequencyCorrection = coarse + fine;
//    emit frequencyCorrectionChanged(frequencyCorrection);

    if (currentFrequency != 0)
        frequencyCorrectionPpm = -1000000.0f * static_cast<float>(frequencyCorrection) / static_cast<float>(currentFrequency);
    else
        frequencyCorrectionPpm = NAN;
    //    emit frequencyCorrectionPpmChanged(frequencyCorrectionPpm);
}

void RadioController::onSyncChange(char isSync)
{
    bool sync = (isSync == SYNCED) ? true : false;
    if (this->isSync == sync)
        return;
    this->isSync = sync;
    //    emit isSyncChanged(isSync);
}

void RadioController::onSignalPresence(bool isSignal)
{
    SDEB("OnSignalPresence");
    if (this->isSignal != isSignal) {
        this->isSignal = isSignal;
//        emit isSignalChanged(isSignal);
    }

    if (isChannelScan)
        SCHEDULE_TASK(&mRadioControllerThread, &RadioController::nextChannel, this, isSignal);
}

void RadioController::onServiceDetected(uint32_t sId)
{
    // you may not call radioReceiver->getService() because it internally holds the FIG mutex.
    SCHEDULE_TASK(&mRadioControllerThread, &RadioController::serviceId, this, sId);
}

void RadioController::onNewEnsemble(uint16_t eId)
{
    SCHEDULE_TASK(&mRadioControllerThread, &RadioController::ensembleId, this, eId);
}

void RadioController::onDateTimeUpdate(const dab_date_time_t& dateTime)
{
    SCHEDULE_TASK(&mRadioControllerThread, &RadioController::displayDateTime, this, dateTime);
}

void RadioController::onFIBDecodeSuccess(bool crcCheckOk, const uint8_t* fib)
{
    (void)fib;
    if (isFICCRC == crcCheckOk)
        return;
    isFICCRC = crcCheckOk;
    //    emit isFICCRCChanged(isFICCRC);
}

void RadioController::onNewImpulseResponse(std::vector<float>&& data)
{
    std::lock_guard<std::mutex> lock(impulseResponseBufferMutex);
    impulseResponseBuffer = std::move(data);
}

void RadioController::onConstellationPoints(std::vector<DSPCOMPLEX>&& data)
{
    std::lock_guard<std::mutex> lock(constellationPointBufferMutex);
    constellationPointBuffer = std::move(data);
}

void RadioController::onNewNullSymbol(std::vector<DSPCOMPLEX>&& data)
{
    std::lock_guard<std::mutex> lock(nullSymbolBufferMutex);
    nullSymbolBuffer = std::move(data);
}

void RadioController::onTIIMeasurement(tii_measurement_t&& m)
{
    SDEB("TII comb %d pattern %d delay %d = %f km with error %f",
         m.comb, m.pattern, m.delay_samples, m.getDelayKm(), m.error);
}

void RadioController::onMessage(message_level_t level, const std::string& text)
{
    switch (level) {
        case message_level_t::Information:
//            emit showInfoMessage(tr(text.c_str()));
            break;
        case message_level_t::Error:
//            emit showErrorMessage(tr(text.c_str()));
            break;
    }
}

void RadioController::OpenDeviceInternal()
{
    CloseDevice();
    mDevice.reset(CInputFactory::GetDevice(*this, "auto"));
    Initialise();

    // LF#
    selectFFTWindowPlacement(1);

    mOpenDeviceEvent.Raise();
}

void RadioController::Initialise()
{
    gainCount = mDevice->getGainCount();
//    emit gainCountChanged(gainCount);
//    emit deviceReady();

    if (!isAGC) { // Manual AGC
        mDevice->setAgc(false);
        currentManualGainValue = mDevice->setGain(currentManualGain);
//        emit gainValueChanged(currentManualGainValue);

        SDEB("AGC off");
    }
    else {
        mDevice->setAgc(true);
        SDEB("AGC on");
    }

//    audio.setVolume(currentVolume);

    deviceName = mDevice->getDescription();
//    emit deviceNameChanged();

    deviceId = mDevice->getID();
//    emit deviceIdChanged();

    if(isAutoPlay) {
        play(autoChannel, "Playing last station", autoService);
    }
}

void RadioController::ResetTechnicalData()
{
    currentChannel = "Unknown";
//    emit channelChanged();

    currentEId = 0;
    currentEnsembleLabel = "";
//    emit ensembleChanged();

    currentFrequency = 0;
//    emit frequencyChanged();

    currentService = 0;
//    emit stationChanged();

    currentStationType = "";
//    emit stationTypChanged();

    currentLanguageType = "";
//    emit languageTypeChanged();

    currentTitle = "No Station";
//    emit titleChanged();

    currentText = "";
//    emit textChanged();

    errorMsg = "";
    isSync = false;
    isFICCRC = false;
    isSignal = false;
    snr = 0;
    frequencyCorrection = 0;
    frequencyCorrectionPpm = NAN;
    bitRate = 0;
    audioSampleRate = 0;
    isDAB = true;
    frameErrors = 0;
    rsErrors = 0;
    aaErrors = 0;

//    motImage.loadFromData(nullptr, 0);
//    emit motChanged(motImage);
}

void RadioController::DeviceRestart()
{
    SDEB("deviceRestart");
    bool isPlay = false;

    if(mDevice) {
        isPlay = mDevice->restart();
    }
    else {
        SWAR("No device!");
    }

    if(!isPlay) {
        SDEB("Radio device is not ready or does not exist.");
//        emit showErrorMessage(tr("Radio device is not ready or does not exist."));
        return;
    }

    labelTimer.Start(40);
}

void RadioController::ensembleId(uint16_t eId)
{
    LF_ASSERT_THREAD(mRadioControllerThread);
    SDEB("ID of ensemble: %d", eId);

    if (currentEId == eId)
        return;

    currentEId = eId;

    auto label = mRadioReceiver->getEnsembleLabel();
    currentEnsembleLabel = label.utf8_label();

//    emit ensembleChanged();
}

void RadioController::serviceId(uint32_t sId)
{
    LF_ASSERT_THREAD(mRadioControllerThread);
    if (isChannelScan == true) {
        stationCount++;
        currentText = LF::utils::sformat("Found channels: %d", stationCount);
//        emit textChanged();
    }

    if (sId <= 0xFFFF) {
        // Exclude data services from the list
        pendingLabels.push_back(sId);
    }
}

void RadioController::labelTimerTimeout(LF::threads::SystemTimer*)
{
    if (mRadioReceiver and not pendingLabels.empty()) {
        const auto sId = pendingLabels.front();
        pendingLabels.pop_front();

        std::string label;

        auto srv = mRadioReceiver->getService(sId);
        if (srv.serviceId != 0) {
            label = srv.serviceLabel.utf8_label();
        }

        if (not label.empty()) {
//            emit newStationNameReceived(qlabel, sId, currentChannel);
            SDEB("Found service %x %s", sId, label.c_str());

            if (currentService == sId) {
                currentTitle = label;
//                emit titleChanged();
            }
        }
        else {
            // Rotate pending labels to avoid getting stuck on a failing one
            pendingLabels.push_back(sId);
        }
    }
}

void RadioController::stationTimerTimeout(LF::threads::SystemTimer*)
{
    if (!mRadioReceiver)
        return;

    const auto services = mRadioReceiver->getServiceList();

    for (const auto& s : services) {
        if (s.serviceId == currentService) {
            const auto comps = mRadioReceiver->getComponents(s);
            for (const auto& sc : comps) {
                if (sc.transportMode() == TransportMode::Audio && (
                        sc.audioType() == AudioServiceComponentType::DAB ||
                        sc.audioType() == AudioServiceComponentType::DABPlus) ) {
                    const auto& subch = mRadioReceiver->getSubchannel(sc);

                    if (not subch.valid()) {
                        return;
                    }

                    // We found the station inside the signal, lets stop the timer
                    stationTimer.Stop();

                    std::string dumpFileName;
//                    if (commandLineOptions["dumpFileName"] != "") {
//                        dumpFileName = commandLineOptions["dumpFileName"].toString().toStdString();
//                    }

                    bool success = mRadioReceiver->playSingleProgramme(*this, dumpFileName, s);
                    if (!success) {
                        SWAR("Selecting service failed");
                    }
                    else {
//                        currentStationType = QCoreApplication::translate("DABConstants",(DABConstants::getProgramTypeName(s.programType)));
//                        emit stationTypChanged();

//                        currentLanguageType = QCoreApplication::translate("DABConstants",(DABConstants::getLanguageName(s.language)));
//                        emit languageTypeChanged();

                        bitRate = subch.bitrate();
//                        emit bitRateChanged(bitRate);

                        if (sc.audioType() == AudioServiceComponentType::DABPlus)
                            isDAB = false;
                        else
                            isDAB = true;
//                        emit isDABChanged(isDAB);
                    }

                    return;
                }
            }
        }
    }
}

void RadioController::channelTimerTimeout(LF::threads::SystemTimer*)
{
    channelTimer.Stop();

    if(isChannelScan)
        nextChannel(false);
}

void RadioController::nextChannel(bool isWait)
{
    LF_ASSERT_THREAD(mRadioControllerThread);

    SINFO("Next channel");

    if (isWait) { // It might be a channel, wait 10 seconds
        channelTimer.Start(10000);
    }
    else {
        auto Channel = channels.getNextChannel();

        if(!Channel.empty()) {
            setChannel(Channel, true);

            int index = channels.getCurrentIndex() + 1;

            currentTitle = LF::utils::sformat("Scanning ... %s (%d\\%)", Channel.c_str(), (index * 100 / NUMBEROFCHANNELS));
//            emit titleChanged();

//            emit scanProgress(index);
        }
        else {
            stopScan();
        }
    }
}

void RadioController::displayDateTime(const dab_date_time_t& dateTime)
{
    LF_ASSERT_THREAD(mRadioControllerThread);
//    QDate Date;
//    QTime Time;

//    Time.setHMS(dateTime.hour, dateTime.minutes, dateTime.seconds);
//    currentDateTime.setTime(Time);

//    Date.setDate(dateTime.year, dateTime.month, dateTime.day);
//    currentDateTime.setDate(Date);

//    int OffsetFromUtc = dateTime.hourOffset * 3600 +
//                        dateTime.minuteOffset * 60;
//    currentDateTime.setOffsetFromUtc(OffsetFromUtc);
//    currentDateTime.setTimeSpec(Qt::OffsetFromUTC);

//    emit dateTimeChanged(currentDateTime);
}
