#include "subsystems.h"

void subsystems_initialize()
{
    drivebase = new Drivebase();
    drivebase->calibrateChassis(true);

    intake = new Intake();
    outtake = new Outtake();
    loader = new Loader();
}