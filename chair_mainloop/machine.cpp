#include "machine.h"

void Machine::start()
{
    ModeState::init(modeState, start_mode);
}

