#pragma once

#include "debug_log.h"

// From this point in the current .cpp/.ino file, Serial.print/println/printf
// go through DebugSerial, which tees output to UART and the web debug log.
#define Serial DebugSerial
