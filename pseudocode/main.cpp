// ConsoleApplication1.cpp : Defines the entry point for the console application.
//

#include <iostream>

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

template <typename T>
class Queue
{
public:

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
		std::unique_lock<std::mutex> mlock(mutex_);
		queue_.push(item);
		mlock.unlock();
		cond_.notify_one();
	}

	bool empty()
	{
		bool isEmpty = false;
		std::unique_lock<std::mutex> mlock(mutex_);
		isEmpty = queue_.empty();
		mlock.unlock();
		return isEmpty;
	}

	Queue() = default;
	Queue(const Queue&) = delete;            // disable copying
	Queue& operator=(const Queue&) = delete; // disable assignment

private:
	std::queue<T> queue_;
	std::mutex mutex_;
	std::condition_variable cond_;
};

class Mesh
{
public:
	Mesh() : nbVertices(0) {}
	
	void addVertice() { nbVertices++; }
	void draw() { std::cout << nbVertices << "\n" << std::endl;  }

	int nbVertices;
};

// only for sleep, remove this
using namespace std::chrono_literals;

class PointCloudSubscriber
{
public:
	PointCloudSubscriber();

	void run();

	// mocks
	void initNode() {}
	void initSubscriber() {}
	void setCallback(void (PointCloudSubscriber::*callback)(void), PointCloudSubscriber* instance) { }
	void nodeHandleSpin()
	{ 
		callback();
		std::this_thread::sleep_for(1s);
	}

	void callback();
	
	Queue<Mesh> queue;
	std::thread subscriberThread;
};

PointCloudSubscriber::PointCloudSubscriber()
	: subscriberThread(&PointCloudSubscriber::run, this)
{
}

void PointCloudSubscriber::callback(/* stuff */)
{
	int nbItems = 4;

	Mesh mesh;
	for (int i = 0; i < nbItems; ++i)
	{
		mesh.addVertice(/*position, color*/);
	}
	queue.push(mesh);
}

void PointCloudSubscriber::run()
{
	initNode();
	initSubscriber();
	setCallback(&PointCloudSubscriber::run, this);

	while (true) {
		nodeHandleSpin();
	}
}

int main()
{
	PointCloudSubscriber subscriber;

	Mesh currentMesh;
	while (true) {
		if (!subscriber.queue.empty()) {
			currentMesh = subscriber.queue.pop();
		}
		currentMesh.draw();
	}

    return 0;
}

