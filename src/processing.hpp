#pragma once

#include "../../gen4/common/sepia.hpp"

#include <vector>

namespace processing {

    // Called once before the first batch, on the processing thread. Put
    // any setup here (allocate frame buffers, open output files, init a
    // model, etc). Runs once, so cost doesn't matter.
    void setup();

    // Called on the processing thread for every batch of events that
    // comes off the camera, in order, one batch at a time (never
    // concurrently). This is where your existing script's logic goes.
    //
    // Still keep this reasonably fast relative to the incoming event
    // rate: if it consistently takes longer than events arrive, the
    // event_queue's backlog will grow and start dropping older batches.
    void process_batch(const std::vector<sepia::dvs_event>& events);

    // Called once after the camera stops (Ctrl+C or an exception), on the
    // processing thread. Flush/close anything opened in setup().
    void teardown();

} // namespace processing