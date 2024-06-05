#ifndef COMM_THREAD_H
#define COMM_THREAD_H

// For reading and writing the scalling factor
// and spinning a separate thread with ros publisher interface
// to offload controller `update` method

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <ros/ros.h>
#include <redis_store_msgs/ParamUpdate.h>

class CommThread
{
public:
    CommThread();
    ~CommThread();

    void start();
    void stop();
    void send(const redis_store_msgs::ParamUpdate& message);

private:
    void threadFunction();

    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable cond_var_;
    redis_store_msgs::ParamUpdate message_;
    bool message_ready_;
    std::atomic<bool> running_;
};

CommThread::CommThread() : message_ready_(false), running_(false) {}

CommThread::~CommThread()
{
    stop();
}

void CommThread::start()
{
    running_ = true;
    thread_ = std::thread(&CommThread::threadFunction, this);
}

void CommThread::stop()
{
    running_ = false;
    cond_var_.notify_one();
    if (thread_.joinable())
        thread_.join();
}

void CommThread::send(const redis_store_msgs::ParamUpdate& message)
{
    std::lock_guard<std::mutex> lock(mutex_);
    message_ = message;
    message_ready_ = true;
    cond_var_.notify_one();
}

void CommThread::threadFunction()
{
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<redis_store_msgs::ParamUpdate>("/config_manager/update", 1000);

    std::unique_lock<std::mutex> lock(mutex_);

    while (running_)
    {
        cond_var_.wait(lock, [this] { return !running_ || message_ready_; });

        if (!running_) break;

        message_ready_ = false;
        lock.unlock(); // Unlock during potentially long-running operations
        publisher.publish(message_);
        lock.lock(); // Re-lock for condition variable waiting
    }
}

#endif  // COMM_THREAD_H
