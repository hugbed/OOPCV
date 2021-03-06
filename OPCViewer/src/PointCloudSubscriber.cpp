#include "PointCloudSubscriber.h"

#include <ctime>

PointCloudSubscriber::PointCloudSubscriber(char* rosMasterIP)
	: m_rosMasterIP(rosMasterIP)
	, subscriberThread(&PointCloudSubscriber::run, this)
	, nbPoints(0)
{
}

void PointCloudSubscriber::callback(const pcl_to_windows::PCLXYZRGB &msg)
{
	std::clock_t c_start = std::clock();

	if (msg.cld_num < nbPoints) {
		if (vertices.size() > 0) {
			Mesh mesh(vertices, indices);
			queue.push(mesh);
		}
		vertices.clear();
		indices.clear();
		nbPoints = 0;
	}

	// let's assume all arrays are of equal length
	size_t nbVertices = msg.x_length;

	for (int i = 0; i < msg.x_length; i++) {
		Vertex vertex =
		{
			glm::vec3(msg.x[i], msg.y[i], msg.z[i]),
			glm::vec3(msg.r[i], msg.g[i], msg.b[i])
		};
		vertices.push_back(vertex);
		indices.push_back(nbPoints++);
	}

	std::cout << "PointCloudSubscriber::callback : " << 1000.0f * (std::clock() - c_start) / (double)CLOCKS_PER_SEC << std::endl;
}

void PointCloudSubscriber::run()
{
	m_nh.initNode(m_rosMasterIP);
	ros::SubscriberClass<pcl_to_windows::PCLXYZRGB, PointCloudSubscriber> sub("pcl_xyzrgb", &PointCloudSubscriber::callback, this);
	m_nh.subscribe(sub);

	while (true) {
		m_nh.spinOnce();
	}
}
