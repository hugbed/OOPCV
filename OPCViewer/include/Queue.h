#pragma once

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

template <typename T>
class Queue
{
public:
	void setSizeLimit(size_t limit)
	{
		std::unique_lock<std::mutex> mlock(mutex_);
		sizeLimit_ = limit;
	}

	T pop()
	{
		std::unique_lock<std::mutex> mlock(mutex_);
		while (queue_.empty())
		{
			cond_.wait(mlock);
		}
		auto val = queue_.front();
		queue_.pop();
		return val;
	}

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

	void push(const T& item)
	{
		bool pushed = false;
		std::unique_lock<std::mutex> mlock(mutex_);
		if (queue_.size() < sizeLimit_) {
			queue_.push(item);
			pushed = true;
		}
		mlock.unlock();
		if (pushed)
			cond_.notify_one();
	}

	bool empty()
	{
		std::unique_lock<std::mutex> mlock(mutex_);
		return queue_.empty();
	}

	Queue() = default;
	Queue(const Queue&) = delete;            // disable copying
	Queue& operator=(const Queue&) = delete; // disable assignment

private:
	std::queue<T> queue_;
	std::mutex mutex_;
	std::condition_variable cond_;
	size_t sizeLimit_;
};
