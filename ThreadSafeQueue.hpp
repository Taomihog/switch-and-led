#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>

template <typename T>
class ThreadSafeQueue {
private:
    std::queue<T> raw_queue;
    mutable std::mutex mtx;
    std::condition_variable cv;

public:
    ThreadSafeQueue() = default;
    
    // Prevent copying to avoid resource management conflicts
    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

    // Push an item into the queue
    void push(T value) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            raw_queue.push(std::move(value));
        }
        cv.notify_one(); // Wake up one waiting consumer thread
    }

    // Block until an item is available and pop it
    T pop_blocking() {
        std::unique_lock<std::mutex> lock(mtx);
        // Lambdas guard against spurious wakeups
        cv.wait(lock, [this]() { return !raw_queue.empty(); });
        
        T value = std::move(raw_queue.front());
        raw_queue.pop();
        return value;
    }

    // Try to pop immediately without blocking
    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lock(mtx);
        if (raw_queue.empty()) {
            return std::nullopt;
        }
        
        T value = std::move(raw_queue.front());
        raw_queue.pop();
        return value;
    }

    // Check if the queue is empty (result is ephemeral)
    bool empty() const {
        std::lock_guard<std::mutex> lock(mtx);
        return raw_queue.empty();
    }

    // Get the current size of the queue
    size_t size() const {
        std::lock_guard<std::mutex> lock(mtx);
        return raw_queue.size();
    }
};
