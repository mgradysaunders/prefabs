/* Copyright (c) 2018-20 M. Grady Saunders
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials
 *      provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*+-+*/
#if !DOXYGEN
#if !(__cplusplus >= 201103L)
#error "preform/thread_pool.hpp requires >=C++11"
#endif // #if !(__cplusplus >= 201103L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_THREAD_POOL_HPP
#define PREFORM_THREAD_POOL_HPP

// for std::chrono::milliseconds
#include <chrono>

// for std::shared_ptr
#include <memory>

// for std::vector
#include <vector>

// for std::thread
#include <thread>

// for std::future, std::packaged_task, ...
#include <future>

// for std::mutex
#include <mutex>

// for std::queue
#include <queue>

// for std::condition_variable
#include <condition_variable>

// for std::function
#include <functional>

namespace pr {

/**
 * @defgroup thread_pool Thread pool
 *
 * `<preform/thread_pool.hpp>`
 *
 * __C++ version__: >=C++11
 */
/**@{*/

/**
 * @brief Thread pool.
 */
class thread_pool
{
public:

    /**
     * @brief Constructor.
     *
     * @param[in] n
     * Number of threads. If less than 1, uses
     * `std::thread::hardware_concurrency()`.
     */
    thread_pool(int n = 0)
    {
        if (n < 1) {
            n = std::thread::hardware_concurrency();
        }

        threads_.reserve(n);

        while (n-- > 0) {
            threads_.emplace_back(worker(*this));
        }
    }

    /**
     * @brief Non-copyable.
     */
    thread_pool(const thread_pool&) = delete;

    /**
     * @brief Destructor.
     */
    ~thread_pool()
    {
        shutdown();
    }

public:

    /**
     * @brief Submit task.
     *
     * @returns
     * `std::future<decltype(std::forward<Func>(func)(std::forward<Args>(args)...))>`.
     */
    template <
        typename Func,
        typename... Args
        >
    inline auto submit(Func&& func, Args&&... args)
#if !DOXYGEN
        -> std::future<decltype(
                std::forward<Func>(func)(
                std::forward<Args>(args)...))>
#endif // #if !DOXYGEN
    {
        // Bind arguments.
        auto bind =
                std::bind(
                std::forward<Func>(func),
                std::forward<Args>(args)...);

        // Allocate packaged task.
        auto bind_ptr =
                std::make_shared<
                std::packaged_task<decltype(bind())()>>(bind);

        // Push.
        queue_.push([bind_ptr]() -> void {
            (*bind_ptr)();
        });

        // Notify one waiting thread.
        cv_.notify_one();

        // Return future.
        return bind_ptr->get_future();
    }

    /**
     * @brief Shutdown pool.
     */
    void shutdown()
    {
        if (shutdown_ == false) {
            shutdown_ = true;
            cv_.notify_all();

            for (std::thread& thread : threads_) {
                if (thread.joinable()) {
                    thread.join();
                }
            }
        }
    }

private:

    /**
     * @brief Task.
     */
    using task_func = std::function<void()>;

    /**
     * @brief Thread-safe task queue.
     */
    class task_queue
    {
    public:

        /**
         * @brief Default constructor.
         */
        task_queue() = default;

        /**
         * @brief Non-copyable.
         */
        task_queue(const task_queue&) = delete;

    public:

        /**
         * @brief Zero tasks in queue?
         */
        bool empty()
        {
            // Lock.
            std::unique_lock<std::mutex> lock(mutex_);

            // Delegate.
            return queue_.empty();
        }

        /**
         * @brief Size of queue.
         */
        std::size_t size()
        {
            // Lock.
            std::unique_lock<std::mutex> lock(mutex_);

            // Delegate.
            return queue_.size();
        }

        /**
         * @brief Push task.
         *
         * @param[in] task
         * Pushed task.
         */
        void push(const task_func& task)
        {
            // Lock.
            std::unique_lock<std::mutex> lock(mutex_);

            // Delegate.
            queue_.push(task);
        }

        /**
         * @brief Pop task.
         *
         * @param[out] task
         * Popped task.
         */
        bool pop(task_func& task)
        {
            // Lock.
            std::unique_lock<std::mutex> lock(mutex_);

            if (queue_.empty()) {
                return false;
            }
            else {
                // Delegate.
                task = std::move(queue_.front());
                queue_.pop();
                return true;
            }
        }

    private:

        /**
         * @brief Queue.
         */
        std::queue<task_func> queue_;

        /**
         * @brief Mutual exclusion object.
         */
        std::mutex mutex_;
    };

    /**
     * @brief Worker.
     */
    class worker
    {
    public:

        /**
         * @brief Constructor.
         *
         * @param[in] pool
         * Thread pool managing this worker.
         */
        worker(thread_pool& pool) : pool_(pool) {}

    public:

        /**
         * @brief Invoke worker loop.
         */
        void operator()()
        {
            task_func task;
            while (!pool_.shutdown_) {

                bool pop_okay = false;
                {
                    std::unique_lock<std::mutex> lock(pool_.cv_mutex_);
                    if (pool_.queue_.empty()) {
                        pool_.cv_.wait_for(lock,
                            std::chrono::milliseconds(50)); // Avoid hanging.
                    }
                    pop_okay = pool_.queue_.pop(task);
                }

                if (pop_okay) {
                    task();
                }
            }
        }

    private:

        /**
         * @brief Thread pool managing this worker.
         */
        thread_pool& pool_;
    };

private:

    /**
     * @brief Threads.
     */
    std::vector<std::thread> threads_;

    /**
     * @brief Shutdown flag.
     */
    bool shutdown_ = false;

    /**
     * @brief Task queue.
     */
    task_queue queue_;

    /**
     * @brief Condition variable.
     */
    std::condition_variable cv_;

    /**
     * @brief Condition variable mutual exclusion object.
     */
    std::mutex cv_mutex_;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_THREAD_POOL_HPP
