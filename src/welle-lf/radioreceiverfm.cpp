#include "radioreceiverfm.h"

RadioReceiverFM::RadioReceiverFM(RadioControllerInterface& rci,
                                 InputInterface& input,
                                 RadioReceiverOptions rro,
                                 int transmission_mode) : RadioReceiver (rci, input, rro, transmission_mode)

{

}
