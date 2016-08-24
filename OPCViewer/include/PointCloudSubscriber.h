// Queue : should be elsewhere

#include "Mesh.h"
#include "Queue.h"

#include <iostream>
#include "ros.h"

// only for sleep, remove this
using namespace std::chrono_literals;

#include <geometry_msgs/Twist.h>

class PointCloudSubscriber
{
public:
	PointCloudSubscriber(char* rosMasterIP);
	PointCloudSubscriber::~PointCloudSubscriber();

	void run();

	// mocks
	void initNode() {}
	void initSubscriber() {}
	void nodeHandleSpin()
	{
		//callback();
		std::this_thread::sleep_for(1s);
	}

	void callback(const geometry_msgs::Twist &msg);

	Queue<Mesh> queue;

private:
	char* m_rosMasterIP;
	std::thread* subscriberThread;
	ros::NodeHandle m_nh;
};

// How to use it
//int main()
//{
//	PointCloudSubscriber subscriber;
//
//	Mesh currentMesh;
//	while (true) {
//		if (!subscriber.queue.empty()) {
//			currentMesh = subscriber.queue.pop();
//		}
//		currentMesh.draw();
//	}
//
//	return 0;
//}

