#include "processing.hpp"

#include <cstdint>
#include <iostream>

namespace processing {

    namespace {
        uint64_t total_events = 0;
    }

    void setup() {
        // e.g. allocate accumulation frames, open a model, open output files
    }

    void process_batch(const std::vector<sepia::dvs_event>& events) {
        // Replace this with the logic from your existing script.
        // `events` is one batch (one USB packet's worth), events are in
        // timestamp order within the batch and across batches.
        total_events += events.size();
        std::cout << "received packet of " << events.size() << " events\n";
        
        for (const auto& event : events) {
            // event.t  -> microseconds
            // event.x  -> 0..1279
            // event.y  -> 0..719
            // event.on -> true = ON, false = OFF
        }
    }

    void teardown() {
        std::cout << "processed " << total_events << " events total\n";
    }

} // namespace processing