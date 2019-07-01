#ifndef RADIORECEIVERFM_H
#define RADIORECEIVERFM_H

#include "radio-receiver.h"


class RadioReceiverFM : public RadioReceiver
{
public:
    RadioReceiverFM(RadioControllerInterface& rci,
                    InputInterface& input,
                    RadioReceiverOptions rro,
                    int transmission_mode = 1);
};

#endif // RADIORECEIVERFM_H
