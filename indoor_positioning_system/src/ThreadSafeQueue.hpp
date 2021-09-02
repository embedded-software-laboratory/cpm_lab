#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>

/**
 * \class ThreadSafeQueue
 * \brief TODO
 * \ingroup ips
 */
template <typename T>
class ThreadSafeQueue
{

private:
    //! queue which stores the elements
    std::queue<T> queue_;

    //! lock for the queue
    std::mutex mutex_;

    //! conditional variable for the queue
    std::condition_variable cond_;


public:

    /**
     * \brief TODO
     */
    T pop()
    {
        std::unique_lock<std::mutex> mlock(mutex_);
        while (queue_.empty())
        {
            cond_.wait(mlock);
        }

        auto item = queue_.front();
        queue_.pop();
        return item;
    }

    /**
     * \brief TODO
     * \param item
     */
    void pop(T& item)
    {
        std::unique_lock<std::mutex> mlock(mutex_);
        while (queue_.empty())
        {
            cond_.wait(mlock);
        }

        item = queue_.front();
        queue_.pop();
    }

    /**
     * \brief TODO
     * \param item
     */
    void push(const T& item)
    {
        std::unique_lock<std::mutex> mlock(mutex_);
        queue_.push(item);
        mlock.unlock();
        cond_.notify_one();
    }

    /**
     * \brief TODO
     * \param item
     */
    void push(T&& item)
    {
        std::unique_lock<std::mutex> mlock(mutex_);
        queue_.push(std::move(item));
        mlock.unlock();
        cond_.notify_one();
    }
};