//
// Created by Jeremy Cote on 2023-08-27.
//

#ifndef UOSM_TELEMETRY_COLLECTOR_HPP
#define UOSM_TELEMETRY_COLLECTOR_HPP

class Collector {
public:
    virtual void collect() = 0;
};

#endif //UOSM_TELEMETRY_COLLECTOR_HPP
