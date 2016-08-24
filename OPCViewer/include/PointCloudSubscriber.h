// Queue : should be elsewhere

#include "Mesh.h"
#include "Queue.h"

#include <iostream>
#include "ros.h"

// only for sleep, remove this
using namespace std::chrono_literals;

#include <geometry_msgs/Twist.h>
#include <pcl_to_windows/PCLXYZRGB.h>

class PointCloudSubscriber
{
public:
	PointCloudSubscriber(char* rosMasterIP);

	void run();

	// mocks
	void initNode() {}
	void initSubscriber() {}
	void nodeHandleSpin()
	{
		//callback();
		std::this_thread::sleep_for(1s);
	}

	void callback(const pcl_to_windows::PCLXYZRGB &msg);

	Queue<Mesh> queue;

private:
	char* m_rosMasterIP;
	ros::NodeHandle m_nh;
	std::thread subscriberThread;
	size_t nbPoints;
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

