#include "PointCloudSubscriber.h"
#include <boost/bind.hpp>

static void someCallback(const geometry_msgs::Twist &msg)
{
	std::cout << "Hello" << std::endl;
}

PointCloudSubscriber::PointCloudSubscriber(char* rosMasterIP)
	: m_rosMasterIP(rosMasterIP)
{
	subscriberThread = new std::thread(&PointCloudSubscriber::run, this);
}

PointCloudSubscriber::~PointCloudSubscriber()
{
	delete subscriberThread;
}

void PointCloudSubscriber::callback(const geometry_msgs::Twist &msg)
{
	std::cout << "PointCloudSubscriber::callback" << std::endl;

	//int nbVertices = 4;
	//int nbItems = 4;

	//std::vector<Vertex> vertices;
	//std::vector<GLuint> indices;
	//vertices.reserve(nbVertices);
	//indices.reserve(nbItems);

	//for (int i = 0; i < nbVertices; ++i)
	//{
	//	//vertices.addVertice(/*position, color*/);
	//	std::cout << "hello" << std::endl;
	//}

	//Mesh mesh(vertices, indices);
	//queue.push(mesh);
}

void PointCloudSubscriber::run()
{
	m_nh.initNode(m_rosMasterIP); // work in other thread if server is actually connected?
	
	ros::SubscriberClass<geometry_msgs::Twist, PointCloudSubscriber> sub("cmd_vel", &PointCloudSubscriber::callback, this);

	m_nh.subscribe(sub);

	while (true) {
		m_nh.spinOnce();
	}
}
