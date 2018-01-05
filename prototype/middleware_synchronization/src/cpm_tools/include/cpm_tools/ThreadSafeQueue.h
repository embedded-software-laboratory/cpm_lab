#pragma once

#include <mutex>
#include <condition_variable>


//! A thread-safe first in first out queue with a maximum capacity. Implemented with a mutex and a simple ring buffer.
template<typename Item, size_t capacity>
class ThreadSafeQueue
{
    std::mutex m_mutex;
    std::condition_variable cond_var_write;
    std::condition_variable cond_var_read;
    Item data[capacity];
    size_t start = 0;
    size_t count = 0;
    bool closed = false;

public:

    /*!
        Blocks until an item is available or the queue is closed.
        Removes the first item from the front of the queue and assigns it to \p item.
        \param[out] item The first queue item, if the queue is open.
        \return True if \p item contains a new value and the queue is open.
    */
    bool read(Item &item) {
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            while (count == 0 && !closed) cond_var_read.wait(lock);
            if (closed) return false;
            item = data[start];
            start = (start + 1) % capacity;
            count--;
        }
        cond_var_write.notify_one();
        return true;
    }

    bool read_nonblocking(Item &item) {
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            if (closed || count == 0) return false;
            item = data[start];
            start = (start + 1) % capacity;
            count--;
        }
        cond_var_write.notify_one();
        return true;
    }

    /*!
        Blocks while the queue is full and open.
        Inserts \p item to the back of the queue.
        \return True if \p item was inserted and the queue is open.
    */
    bool write(Item item) {
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            while (count >= capacity && !closed) cond_var_write.wait(lock);
            if (closed) return false;
            data[(start + count) % capacity] = item;
            count++;
        }
        cond_var_read.notify_one();
        return true;
    }

    bool write_nonblocking(Item item) {
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            if (closed || count >= capacity) return false;
            data[(start + count) % capacity] = item;
            count++;
        }
        cond_var_read.notify_one();
        return true;
    }

    /*!
        Closes the queue. All present and future calls to read() and write() will return false.
    */
    void close() {
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            closed = true;
        }
        cond_var_read.notify_all();
        cond_var_write.notify_all();
    }


    ThreadSafeQueue() = default;
    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

};
