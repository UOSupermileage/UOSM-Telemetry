//
// Created by Jeremy Cote on 2023-09-15.
//

#ifndef UOSM_TELEMETRY_MUTEX_HPP
#define UOSM_TELEMETRY_MUTEX_HPP

#define DEFAULT_MUTEX_DELAY 50
#define DEFAULT_MUTEX_RETRIES 6

// Mutex
class Mutex {
private:
    bool locked = false;
public:
    [[nodiscard]] bool getLocked() const { return locked; }
    bool lock() {
        if (locked) {
            return false;
        }
        locked = true;
        return true;
    }
    void unlock() {
        locked = false;
    }
    void execute(const std::function<void()>& callback, uint32_t retryDelay = DEFAULT_MUTEX_RETRIES, uint8_t maxTries = DEFAULT_MUTEX_RETRIES) {
        uint8_t tries = 0;
        while (tries < maxTries) {
            if (!getLocked()) {
                vTaskDelay(retryDelay);
            } else {
                lock();
                callback();
                unlock();
            }

            tries++;
        }
    }
};

#endif //UOSM_TELEMETRY_MUTEX_HPP
