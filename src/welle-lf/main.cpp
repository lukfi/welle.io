#include <iostream>
#include "radiocontroller.h"
#include "audio/audioplayer.h"

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
    rc.startScan();

    std::string line;
    while (true)
    {
        std::getline(std::cin, line);
        if (line == "play")
        {
            rc.play("11B", "trójka", 0x3233);
        }
        else
        {
            break;
        }
    }
    return 0;
}
