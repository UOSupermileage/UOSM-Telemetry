//
// Created by Jeremy Cote on 2023-09-15.
//

#ifndef UOSM_TELEMETRY_MUTEX_HPP
#define UOSM_TELEMETRY_MUTEX_HPP

#define DEFAULT_MUTEX_DELAY 50
#define DEFAULT_MUTEX_RETRIES 6

/**
 * Ensure that only one core can access a protected resource at once.
 */
class Mutex {
private:
    bool locked = false;
public:
    /**
     * Get the status of the mutex.
     * @return true if the mutex's resource is in use.
     */
    [[nodiscard]] bool getLocked() const { return locked; }

    /**
     * Lock the mutex.
     * Use before accessing the data it protects.
     * @return
     */
    bool lock() {
        if (locked) {
            return false;
        }
        locked = true;
        return true;
    }

    /**
     * Unlock the mutex.
     * Use once an operation is complete
     */
    void unlock() {
        locked = false;
    }

    /**
     * Await mutex unlock and then execute the callback.
     * This will defer execution to other tasks while blocked.
     * @param callback Function to execute when the mutex is unlocked.
     * @param retryDelay Number of milliseconds between retries
     * @param maxTries Max number of retries before continuing without executing callback
     */
    void execute(const std::function<void()>& callback, uint32_t retryDelay = DEFAULT_MUTEX_RETRIES, uint8_t maxTries = DEFAULT_MUTEX_RETRIES) {
        uint8_t tries = 0;
        while (tries < maxTries) {
            if (!getLocked()) {
                rtos::ThisThread::sleep_for(retryDelay);
//                vTaskDelay(retryDelay);
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
