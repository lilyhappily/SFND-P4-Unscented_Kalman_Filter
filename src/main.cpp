/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

//#include "render/render.h"
#include "highway.h"
#include <vector>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc, char** argv)
{

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// set camera position and angle
	viewer->initCameraParameters();
	float x_pos = 0;
	viewer->setCameraPosition ( x_pos-26, 0, 15.0, x_pos+25, 0, 0, 0, 0, 1);

	Highway highway(viewer);

	//initHighway(viewer);

	int frame_per_sec = 30;
	int sec_interval = 10;
	int frame_count = 0;
	int time_us = 0;

	double egoVelocity = 25;

	while (frame_count < (frame_per_sec*sec_interval))
	{
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		//stepHighway(egoVelocity,time_us, frame_per_sec, viewer);
		highway.stepHighway(egoVelocity,time_us, frame_per_sec, viewer);
		viewer->spinOnce(1000/frame_per_sec);
		frame_count++;
		time_us = 1000000*frame_count/frame_per_sec;
		
	}

    // matplotlibcpp: https://readthedocs.org/projects/matplotlib-cpp/downloads/pdf/latest/
	// plot NIS for Traffic[0]
	std::vector<double> lidarNIS = highway.traffic[0].ukf.lidarNIS;
	std::vector<double> radarNIS = highway.traffic[0].ukf.radarNIS;

	float chiSquareLidar = 7.815;
	float chiSquareRadar = 5.991;
	std::vector<float> chiLidar(lidarNIS.size(), chiSquareLidar);
	std::vector<float> chiRadar(radarNIS.size(), chiSquareRadar);

	std::vector<int> x_lidar;
	for(int i = 0; i < lidarNIS.size(); ++i)
	{
		x_lidar.push_back(i);
	}

	std::vector<int> x_radar;
	for(int i = 0; i < radarNIS.size(); ++i)
	{
		x_radar.push_back(i);
	}

	plt::subplot(2,1,1);
	plt::plot(x_lidar, lidarNIS);
	plt::plot(x_lidar, chiLidar, "r--");
	plt::text(175, 9, "Chi-Square-Lidar = 7.815");
	plt::xlabel("Frames");
	plt::ylabel("Lidar-NIS");

	plt::subplot(2,1,2);
	plt::plot(x_radar, radarNIS);
	plt::plot(x_radar, chiRadar, "r--");
	plt::text(175, 9, "Chi-Square-Radar = 5.991");
	plt::xlabel("Frames");
	plt::ylabel("Radar-NIS");

	//plt::save("./NIS.pdf");
	plt::show();
}