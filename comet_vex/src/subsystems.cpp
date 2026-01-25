#include "subsystems.h"
#include <subsystems/drivebase.h>

void subsystems_initialize() {
    drivebase = new Drivebase();
}