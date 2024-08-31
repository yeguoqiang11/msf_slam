#include <iostream>
#include <memory>
#include <set>
#include <unordered_map>
#include "utils/ThreadPool.h"
class Test {
  public:
    Test(utils::ThreadPool &threads): threads_(threads) { count_ = 0; }
    std::vector<std::weak_ptr<utils::Task>> Run(int count);
    void work(int i) {
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::unique_lock<std::mutex> locker(mtx_);
            count_ += i;
            std::cout << "count: " << count_ << std::endl;
            std::cout << "i: " << i << std::endl;
        }
        int count = count_;
        for (int j = 0; j < 5; j++) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::unique_lock<std::mutex> locker(mtx_);
            std::cout << count << "th thread is running!!!" << std::endl;
        }

    }
  private:
    utils::ThreadPool &threads_;
    int count_;
    std::mutex mtx_;
};
std::vector<std::weak_ptr<utils::Task>> Test::Run(int count) {
    int a = 5;
    std::vector<std::weak_ptr<utils::Task>> results;
    for (int i = 0; i < count; i++) {
        std::unique_ptr<utils::Task> task = std::make_unique<utils::Task>();
        task->SetWork([this, &a]() { this->work(a); });
        std::weak_ptr<utils::Task> result = this->threads_.Schedule(std::move(task));
        results.push_back(result);
    }

    while (1) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        bool is_finished = true;
        for (int j = 0; j < results.size(); j++) {
            if (!results[j].expired()) {
                std::shared_ptr<utils::Task> task = results[j].lock();
                if (task->TaskState() != 2) {
                    is_finished = false;
                    std::cout << j << "th week ptr: " << task << std::endl;
                }
            }
        }
        if (is_finished) break;
    }
    return results;
}

void Task0() {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "run task0!!!" << std::endl;
}

int main(int argc, char** argv) {
    utils::ThreadPool thread_pool(5);

    std::unique_ptr<utils::Task> task0_ptr = std::make_unique<utils::Task>();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    task0_ptr->SetWork(Task0);

    std::unique_ptr<utils::Task> task1_ptr = std::make_unique<utils::Task>();
    task1_ptr->SetWork([]() {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        std::cout << "run task1!!!" << std::endl; 
    });
    std::unique_ptr<utils::Task> task2_ptr = std::make_unique<utils::Task>();
    task2_ptr->SetWork([]() { 
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "run task2!!!" << std::endl; 
    });
    auto t0 = thread_pool.Schedule(std::move(task0_ptr));

    // task1_ptr->AddDependency(t0);

    auto t1 = thread_pool.Schedule(std::move(task1_ptr));

    task2_ptr->AddDependency(t0);
    task2_ptr->AddDependency(t1);

    auto t2 = thread_pool.Schedule(std::move(task2_ptr));
    std::unique_ptr<utils::Task> task3_ptr = std::make_unique<utils::Task>();
    task3_ptr->SetWork([]() {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "run task3!!!" << std::endl;
    });
    task3_ptr->AddDependency(t1);
    thread_pool.Schedule(std::move(task3_ptr));
    
    std::this_thread::sleep_for(std::chrono::seconds(15));
    std::cout << "ready task size: " << thread_pool.ReadyTaskSize() << std::endl;
    std::cout << "not ready task size: " << thread_pool.NotReadyTaskSize() << std::endl;

    return 0;


    Test test0(thread_pool);

    std::vector<std::weak_ptr<utils::Task>> results = test0.Run(8);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    for (int i = 0; i < results.size(); i++) {
        auto &res = results[i];
        if (!res.expired()) {
            std::shared_ptr<utils::Task> obj = res.lock();
            std::cout << i << "th task state: " << obj->TaskState() << std::endl;
        }
    }
    std::cout << "finished!!!" << std::endl;
    std::cout << "------------------" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    for (int i = 0; i < results.size(); i++) {
        auto &res = results[i];
        if (!res.expired()) {
            std::shared_ptr<utils::Task> obj = res.lock();
            std::cout << i << "th task state: " << obj->TaskState() << std::endl;
        }
    }
    // thread_pool.Stop();
    // if (thread_pool.TaskNum() == 0) {
    //     thread_pool.Stop();
    // }
    // std::cout << "task num: " << thread_pool.TaskNum() << std::endl;
    
}