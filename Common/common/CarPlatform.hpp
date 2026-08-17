#pragma once

namespace common {

// The fork is being reduced to P3-only. CarPlatform::P3 keeps meaning Y285/Y286/Y381 so
// existing commands and scripts are unaffected; the P3_Y* values open the other P3
// configurations that live in data.yaml (Y413, Y283/Y352, P313, Y555, Y312H) but used to be
// unreachable from the CLI. V40 ("P4") is not a separate platform - it is the Y555
// configuration family.
//
// The non-P3 values below still exist while the D2/TP20/KWP code that references them is being
// removed (they must compile until their last caller goes). They are deleted in the final step
// of the P3-only reduction.
enum class CarPlatform
{
    Undefined,
    P80,
    P1,
    P1_UDS,
    P2,
    P2_250,
    P2_UDS,
    P3,
    SPA,
    Ford_KWP,
    Ford_UDS,
    Haval_UDS,
    VAG,
    VAG_MED91,
    VAG_MED912,
    P3_Y413,
    P3_Y283_IAM,
    P3_Y283_ICM,
    P3_P313_ICM,
    P3_P313_IAM,
    P3_Y555_IAM,
    P3_Y555_ICM,
    P3_Y312H_IAM,
    P3_Y312H_ICM
};

}

