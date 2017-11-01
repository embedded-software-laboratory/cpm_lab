#include <mutex>
#include <condition_variable>


/*!
    Hello
*/
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

    bool read(Item &item) {
        std::unique_lock<std::mutex> lock(m_mutex);
        while (count == 0 && !closed) cond_var_read.wait(lock);
        if(closed) return false;
        item = data[start];
        start = (start + 1) % capacity;
        count--;
        cond_var_write.notify_one();
        return true;
    }

    bool write(Item item) {
        std::unique_lock<std::mutex> lock(m_mutex);
        while (count >= capacity && !closed) cond_var_write.wait(lock);
        if(closed) return false;
        data[(start + count) % capacity] = item;
        count++;
        cond_var_read.notify_one();
        return true;
    }

    void close() {
        std::unique_lock<std::mutex> lock(m_mutex);
        closed = true;
        cond_var_read.notify_all();
        cond_var_write.notify_all();
    }

};
