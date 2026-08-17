#pragma once

namespace common {

// The fork is P3-only. CarPlatform::P3 means Y285/Y286/Y381 (legacy --platform p3); the extra
// values open the other P3 configurations that live in data.yaml (Y413, Y283/Y352, P313, Y555,
// Y312H). V40 ("P4") is not a separate platform - it is the Y555 configuration family.
enum class CarPlatform
{
    Undefined,
    P3,
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

