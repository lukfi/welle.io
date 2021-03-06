#include <iostream>
#include "radiocontroller.h"
#include "audio/audioplayer.h"
#include "threads/threadutils.h"

using namespace std;

int main()
{
    LF::audio::AudioBuffer buffer(LF::audio::AudioFormat_t::Sint16, 48000, 2);

    LF::audio::AudioParameters params;
    params.Format = LF::audio::AudioFormat_t::Sint16;
    params.NumChannels = 2;
    params.SampleRate = 48000;

    LF::audio::AudioBufferPlayer player(params);

    player.SetEndOfEmptyBuffer(false);
    player.SetBuffer(&buffer);
    auto api = LF::audio::AudioDevice::GetDefaultApi();
    player.SetOutputDevice(api, LF::audio::AudioDevice::GetDefaultOutDeviceId(api));
    player.Start();

    RadioController rc;
    rc.SetBuffer(&buffer);
    rc.OpenDevice();
    rc.setAGC(true);
//    rc.startScan();

    std::string line;
    while (true)
    {
        std::getline(std::cin, line);
        if (line == "play")
        {
            rc.play("11B", "trójka", 0x3233);
        }
        else if (line == "play2")
        {
            rc.play("11B", "radio kraków", 0x37fc);
        }
        else if (line == "fm")
        {
            rc.play(96000000);
        }
        else if (line == "fm2")
        {
            rc.play(101000000);
        }
        else if (line == "scan")
        {
            rc.startScan();
        }
        else if (line == "scanback")
        {
            rc.startScan(true);
        }
        else if (line == "m")
        {
            rc.PrintMeasueres();
        }
        else if (line == "next")
        {
            rc.FmSeekNext();
        }
        else if (line == "prev")
        {
            rc.FmSeekPrev();
        }
        else if (line == "stop")
        {
            rc.stop();
        }
        else if (line == "test")
        {
            while (true)
            {
                rc.play(96000000);
                LF::threads::SleepSec(1);
                rc.stop();
                LF::threads::SleepSec(1);
            }
        }
        else
        {
            // unknown command
        }
    }
    return 0;
}
