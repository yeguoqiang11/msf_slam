#include "utils/ThreadPool.h"

namespace utils {
Task::Task()
  : thread_pool_(nullptr), state_(State::WAIT), uncompleted_dependencies_(0) {
}

Task::~Task() {

}

void Task::SetWork(const std::function<void()> &work) {
    std::unique_lock<std::mutex> locker(task_mtx_);    
    state_ = WAIT;
    this->work_ = work;
}


void Task::Execute() {
    {
        std::unique_lock<std::mutex> locker(task_mtx_);
        state_ = RUNNING;
    }
    if (work_) {
        work_();
    }

    {
        std::unique_lock<std::mutex> locker(task_mtx_);
        state_ = COMPLETED;
        for (Task* dependent_task : dependent_tasks_) {
            dependent_task->OnDependencyCompleted();
        }
    }
}

void Task::AddDependency(std::weak_ptr<Task> dependency) {
    if (state_ != State::WAIT) return;
    std::shared_ptr<Task> dependency_ptr = nullptr;
    std::unique_lock<std::mutex> locker(task_mtx_);
    if (!dependency.expired()) {
        uncompleted_dependencies_++;
        dependency_ptr = dependency.lock();
    }

    if (dependency_ptr) {
        dependency_ptr->AddDependentTask(this);
    }
}

void Task::AddDependentTask(Task *dependent_task) {
    std::unique_lock<std::mutex> locker(task_mtx_);
    if (state_ == State::COMPLETED) {
        dependent_task->OnDependencyCompleted();
    } else {
        dependent_tasks_.insert(dependent_task);
    }
}

void Task::OnDependencyCompleted() {
    std::unique_lock<std::mutex> locker(task_mtx_);
    if (state_ != WAIT) return;
    uncompleted_dependencies_--;
    if (uncompleted_dependencies_ == 0 && thread_pool_ != nullptr) {
        // todo: notify thread pool to deal this task
        thread_pool_->NotifyTaskToBeReady(this);
    }
}

void Task::SetThreadPool(ThreadPool *thread_pool) {
    std::unique_lock<std::mutex> locker(task_mtx_);
    if (state_ != State::WAIT) return;
    thread_pool_ = thread_pool;
    if (uncompleted_dependencies_ == 0 && thread_pool_ != nullptr) {
        thread_pool_->NotifyTaskToBeReady(this);
    }
}


ThreadPool::ThreadPool(int num_threads) {
    is_running_ = true;
    if (num_threads <= 0 || num_threads > 5) {
        std::cout << "cannot create threadpools with threads: " << num_threads << std::endl;
        exit(0);
    }
    for (int i = 0; i < num_threads; i++) {
        threads_.emplace_back(&ThreadPool::Run, this);
    }
}
ThreadPool::~ThreadPool() {
    Stop();
}

void ThreadPool::Run() {
    while (is_running_) {
        std::shared_ptr<Task> task = nullptr;
        {
            std::unique_lock<std::mutex> locker(this->mtx_);
            if(ready_tasks_.empty() && is_running_) {
                cv_.wait(locker);
            }
            if (!ready_tasks_.empty()) {
                task = std::move(this->ready_tasks_.front());
                this->ready_tasks_.pop();
            } else if (!is_running_) {
                return;
            }
        }

        if (task.get()) {
            task->Execute();
        }
    }
}

std::weak_ptr<Task> ThreadPool::Schedule(std::unique_ptr<Task> task) {
    std::shared_ptr<Task> shared_task = nullptr;
    {
        std::unique_lock<std::mutex> locker(mtx_);
        auto result = tasks_not_ready_.insert(std::make_pair(task.get(), std::move(task)));
        shared_task = result.first->second;
    }
    shared_task->SetThreadPool(this);
    return shared_task;
}

void ThreadPool::NotifyTaskToBeReady(Task* task) {
    auto it = tasks_not_ready_.find(task);
    if (it != tasks_not_ready_.end()) {
        {
            std::unique_lock<std::mutex> locker(mtx_);
            ready_tasks_.push(it->second);
            tasks_not_ready_.erase(it);
        }
        cv_.notify_one();
    }
}

void ThreadPool::Stop() {
    is_running_ = false;
    cv_.notify_all();
    for (auto &it : threads_) {
        if (it.joinable())
            it.join();
    }
}
} // namespace utils