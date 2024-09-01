/**
 * @file ThreadPool.h
 * @author Guoqiang Ye (yegq@Yeguoqiang.tech)
 * @brief 
 * @version 0.1
 * @date 2022-07-18
 * 
 * @copyright Copyright (c) 2022 Yeguoqiang Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef UTILS_THREADPOOL_H
#define UTILS_THREADPOOL_H
#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <thread>
#include <unordered_map>
#include <vector>

namespace utils {
class ThreadPool;
class Task {
    enum State {WAIT, RUNNING, COMPLETED};
  public:
    Task();
    ~Task();
    void SetWork(const std::function<void()> &work);
    State TaskState() { return state_; }
    void AddDependency(std::weak_ptr<Task> dependency);
    unsigned int UnCompletedDependency() { return uncompleted_dependencies_; }

    friend class ThreadPool;
  private:
    void SetThreadPool(ThreadPool *thread_pool);
    void AddDependentTask(Task *dependent_task);
    void OnDependencyCompleted();
    void Execute();

    std::function<void()> work_;
    std::mutex task_mtx_;
    State state_;
    std::set<Task*> dependent_tasks_;
    unsigned int uncompleted_dependencies_;
    ThreadPool *thread_pool_;
}; // class Task

class ThreadPool {
  public:
    explicit ThreadPool(int num_threads);
    ~ThreadPool();

    ThreadPool(const ThreadPool &) = delete;
    ThreadPool &operator=(const ThreadPool&) = delete;

    std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task);
    void Stop();
    size_t TaskNum() {
        std::unique_lock<std::mutex> locker(mtx_);
        return ready_tasks_.size();
    }

    size_t ReadyTaskSize() { return ready_tasks_.size(); }
    size_t NotReadyTaskSize() { return tasks_not_ready_.size(); }
    friend class Task;
  private:
    void NotifyTaskToBeReady(Task* task);
    void Run();
    std::mutex mtx_;
    std::condition_variable cv_;
    std::queue<std::shared_ptr<Task>> ready_tasks_;
    std::unordered_map<Task*, std::shared_ptr<Task>> tasks_not_ready_;
    std::vector<std::thread> threads_;
    std::atomic_bool is_running_;
}; // class ThreadPool
} // namespace utils
#endif