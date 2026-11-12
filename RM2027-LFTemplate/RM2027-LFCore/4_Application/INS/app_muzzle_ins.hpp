#pragma once

namespace muzzle_ins {
// Called by INS_Task; consumes DM raw reports once per new gyro timestamp.
// Independent of the onboard BMI088 filter and its temperature calibration.
void update();
}
