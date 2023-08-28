//
// Created by Jeremy Cote on 2023-08-28.
//

#ifndef UOSM_TELEMETRY_VALUESTORE_HPP
#define UOSM_TELEMETRY_VALUESTORE_HPP

template<typename T>
class ValueStore {
public:
    virtual T getValue() = 0;
    virtual ~ValueStore() = default;
};

#endif //UOSM_TELEMETRY_VALUESTORE_HPP
