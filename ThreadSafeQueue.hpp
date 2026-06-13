#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <fstream>
#include <string>
#include <optional>

// A basic Thread-Safe Blocking Queue implementation
template <typename T>
class BlockingQueue {
private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool done_ = false; // Flag to indicate production is finished

public:
    // Push an item into the queue and notify the waiting thread
    void push(T value) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(std::move(value));
        }
        cv_.notify_one(); // Wake up the write thread
    }

    // Blocks until an item is available or the queue is shut down
    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        
        // Wait until the queue has elements OR we are shutting down
        cv_.wait(lock, [this]() { 
            return !queue_.empty() || done_; 
        });

        // If queue is empty and we are shutting down, return empty optional
        if (queue_.empty() && done_) {
            return std::nullopt;
        }

        T value = std::move(queue_.front());
        queue_.pop();
        return value;
    }

    // Gracefully shut down the queue
    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            done_ = true;
        }
        cv_.notify_all(); // Wake up any thread stuck in pop()
    }
};