#pragma once

#include "body/proc_body.h"
#include "status.h"

namespace proc {

// Structural and numeric checks for a parsed body. The loader calls this
// before accepting a file. A body that fails is not returned as loaded.
void validateProcBody(const ProcBody& body, Status& status);

}  // namespace proc
