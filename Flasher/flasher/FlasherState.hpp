#pragma once

namespace flasher {

enum class FlasherState {
    Initial,
    FallAsleep,
    Authorize,
    LoadBootloader,
    StartBootloader,
    EraseFlash,
    WriteFlash,
    ReadFlash,
    WakeUp,
    Done,
    Error
};

}
