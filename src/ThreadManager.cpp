#include "../include/ThreadManager.h"
#include <stdexcept>

ThreadManager::ThreadManager(size_t numThreads) 
    : numThreads(numThreads), running(false), activeThreads(0) {
    if (numThreads <= 0) {
        throw std::invalid_argument("Number of threads must be positive");
    }
    threadLoads.resize(numThreads, 0);
}

ThreadManager::~ThreadManager() {
    stop();
}

void ThreadManager::start() {
    std::lock_guard<std::mutex> lock(taskMutex);
    if (!running) {
        running = true;
        threads.clear();
        for (size_t i = 0; i < numThreads; ++i) {
            threads.emplace_back(&ThreadManager::workerThread, this, i);
        }
    }
}

void ThreadManager::stop() {
    {
        std::lock_guard<std::mutex> lock(taskMutex);
        if (!running) return;
        running = false;
    }
    
    taskCondition.notify_all();
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    threads.clear();
}

void ThreadManager::addTask(std::function<void()> task) {
    {
        std::lock_guard<std::mutex> lock(taskMutex);
        taskQueue.push(task);
    }
    taskCondition.notify_one();
}

bool ThreadManager::isRunning() const {
    return running;
}

size_t ThreadManager::getNumThreads() const {
    return numThreads;
}

void ThreadManager::setNumThreads(size_t newNumThreads) {
    if (newNumThreads <= 0) {
        throw std::invalid_argument("Number of threads must be positive");
    }
    
    if (running) {
        stop();
    }
    
    numThreads = newNumThreads;
    threadLoads.resize(numThreads, 0);
    
    if (running) {
        start();
    }
}

size_t ThreadManager::getTaskCount() const {
    std::lock_guard<std::mutex> lock(taskMutex);
    return taskQueue.size();
}

void ThreadManager::waitForCompletion() {
    std::unique_lock<std::mutex> lock(completionMutex);
    // Use condition variable to wait for completion instead of busy waiting
    taskCondition.wait(lock, [this] {
        return (activeThreads == 0) && taskQueue.empty();
    });
}

size_t ThreadManager::getActiveThreadCount() const {
    return activeThreads;
}

void ThreadManager::processNextTask() {
    // This method should use the same task processing logic as workerThread
    // to avoid duplication and potential race conditions
    std::function<void()> task;
    {
        std::unique_lock<std::mutex> lock(taskMutex);
        if (taskQueue.empty()) {
            return;
        }
        
        task = std::move(taskQueue.front());
        taskQueue.pop();
    }
    
    if (task) {
        activeThreads++;
        try {
            task();
        } catch (const std::exception& e) {
            // Log or handle exceptions from tasks to prevent thread termination
            // In a real implementation, you might want to log this
        } catch (...) {
            // Catch all other exceptions
        }
        activeThreads--;
        
        // Notify waitForCompletion that a task has completed
        taskCondition.notify_all();
    }
}

void ThreadManager::workerThread(size_t threadId) {
    while (running) {
        std::function<void()> task;
        
        {
            std::unique_lock<std::mutex> lock(taskMutex);
            taskCondition.wait(lock, [this] { 
                return !running || !taskQueue.empty(); 
            });
            
            if (!running && taskQueue.empty()) {
                return;
            }
            
            if (!taskQueue.empty()) {
                task = std::move(taskQueue.front());
                taskQueue.pop();
                threadLoads[threadId]++;
            }
        }
        
        if (task) {
            activeThreads++;
            try {
                task();
            } catch (const std::exception& e) {
                // Log or handle exceptions from tasks to prevent thread termination
                // In a real implementation, you might want to log this
            } catch (...) {
                // Catch all other exceptions
            }
            activeThreads--;
            
            // Notify waitForCompletion that a task has completed
            taskCondition.notify_all();
        }
    }
}