// openni_stream.cpp : Defines the entry point for the console application.
//
#include <Kinect.h>
#include <iostream>
#include <opencv2\opencv.hpp>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#define SAFE_RELEASE(p) { if ( (p) ) { (p)->Release(); (p) = 0; } }

using namespace cv;
using namespace std;
HRESULT result = S_OK;
HRESULT hResult = S_OK;
IKinectSensor* pSensor;
ICoordinateMapper* mapper;
IColorFrameSource* colorSource;
IColorFrameReader* pColorReader;
IDepthFrameSource* depthSource;
IDepthFrameReader* pDepthReader;

int colorWidth = 1920;
int colorHeight = 1080;
Mat colorBuffer(colorHeight, colorWidth, CV_8UC4);
Mat colorMat(colorHeight / 2, colorWidth / 2, CV_8UC4);

int depthWidth = 512;
int depthHeight = 424;
Mat depthBuffer(depthHeight, depthWidth, CV_16UC1);
Mat depthMat(depthHeight, depthWidth, CV_8UC1);

template<class Interface> inline void SafeRelease(Interface *& IRelease)
{
	if (IRelease != NULL) {
		IRelease->Release();
		IRelease = NULL;
	}
}

void SavePclFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {

	std::string file_name;
	std::cout << "Please enter a file name�G" << std::endl;
	std::cin >> file_name;
	file_name = file_name + +(".pcd");
	pcl::io::savePCDFile(file_name, *cloud);
}
void init_Kinect();
void release_sensor();
void writeCSV(string filename, Mat m);
int main() {
	init_Kinect();
	UINT16 uDepthMin = 0, uDepthMax = 0;
	pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
	depthSource->get_DepthMinReliableDistance(&uDepthMin);
	depthSource->get_DepthMaxReliableDistance(&uDepthMax);
	while (true)
	{
		IColorFrame* pColorFrame = nullptr;
		IDepthFrame* pDepthFrame = nullptr;
		if (pColorReader->AcquireLatestFrame(&pColorFrame) == S_OK) {
			hResult = pColorFrame->CopyConvertedFrameDataToArray(1920 * 1080 * 4, reinterpret_cast<BYTE*>(colorBuffer.data), ColorImageFormat_Bgra);
			if (SUCCEEDED(hResult)) {
				cv::resize(colorBuffer, colorMat, cv::Size(), 0.5, 0.5);
			}
			pColorFrame->Release();
		}
		if (pDepthReader->AcquireLatestFrame(&pDepthFrame) == S_OK) {
			hResult = pDepthFrame->CopyFrameDataToArray(512 * 424, reinterpret_cast<UINT16*>(depthBuffer.data));
			if (SUCCEEDED(hResult)) {
				depthBuffer.convertTo(depthMat, CV_8U, 255.0f / uDepthMax);
			}
			pDepthFrame->Release();
		}
		//SAFE_RELEASE(pColorFrame);
		//SAFE_RELEASE(pDepthFrame);							// Show Window
		//reinterpret_cast<BYTE*>(colorBuffer.data);
		//reinterpret_cast<UINT16*>(depthBuffer.data);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

		cloud->width = static_cast<uint32_t>(depthWidth);
		cloud->height = static_cast<uint32_t>(depthHeight);
		cloud->is_dense = false;
		for (int y = 300; y < 800; y++) {
			for (int x = 100; x < 500; x++) {
				colorBuffer.at<Vec4b>(y, x)[0] = 0;
				colorBuffer.at<Vec4b>(y, x)[1] = 0;
				colorBuffer.at<Vec4b>(y, x)[2] = 255;
			}
		}
		cv::resize(colorBuffer, colorMat, cv::Size(), 0.5, 0.5);
		imshow("Color", colorMat);
		imshow("Depth", depthMat);
		cloud->points.resize(cloud->height * cloud->width);
		try {
			pcl::PointXYZRGB* pt = &cloud->points[0];
			for (int y = 0; y < depthHeight; y++) {
				for (int x = 0; x < depthWidth; x++, pt++) {
					pcl::PointXYZRGB point;

					DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
					UINT16 depth = depthBuffer.at<UINT16>(y, x);

					// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
					ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
					mapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint);
					int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
					int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));
					if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {//Height,Width  == y,x
						;
						point.b = colorBuffer.at<Vec4b>(colorY, colorX)[0];
						point.g = colorBuffer.at<Vec4b>(colorY, colorX)[1];
						point.r = colorBuffer.at<Vec4b>(colorY, colorX)[2];
					}
					// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
					CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
					mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
					if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
						point.x = cameraSpacePoint.X;
						point.y = cameraSpacePoint.Y;
						point.z = cameraSpacePoint.Z;
					}

					*pt = point;
				}
			}
		}
		catch (Exception e) {
			throw std::exception(e);
		}
		viewer.spinOnce();
		viewer.addCoordinateSystem(0.10);
		auto ret = viewer.updatePointCloud(cloud, "cloud");
		if (!ret) {
			viewer.addPointCloud(cloud, "cloud");
		}
		char key = waitKey(30);
		if (key == VK_ESCAPE) {
			break;
		}
		else if (key == 'P' || key == 'p') {
			writeCSV("depth.csv", depthBuffer);
			writeCSV("color.csv", colorBuffer);
			SavePclFile(cloud);
		}
	}

	release_sensor();
	return 0;
}

void init_Kinect() {
	result = GetDefaultKinectSensor(&pSensor);
	if (FAILED(result)) {
		throw std::exception("Exception : GetDefaultKinectSensor()");
	}
	// Open Sensor
	result = pSensor->Open();
	if (FAILED(result)) {
		throw std::exception("Exception : IKinectSensor::Open()");
	}
	// Retrieved Coordinate Mapper
	result = pSensor->get_CoordinateMapper(&mapper);
	if (FAILED(result)) {
		throw std::exception("Exception : IKinectSensor::get_CoordinateMapper()");
	}
	// Retrieved Color Frame Source
	result = pSensor->get_ColorFrameSource(&colorSource);
	if (FAILED(result)) {
		throw std::exception("Exception : IKinectSensor::get_ColorFrameSource()");
	}

	// Retrieved Depth Frame Source
	result = pSensor->get_DepthFrameSource(&depthSource);
	if (FAILED(result)) {
		throw std::exception("Exception : IKinectSensor::get_DepthFrameSource()");
	}
	result = colorSource->OpenReader(&pColorReader);
	if (FAILED(result)) {
		throw std::exception("Exception : IColorFrameSource::OpenReader()");
	}

	// Open Depth Frame Reader
	result = depthSource->OpenReader(&pDepthReader);
	if (FAILED(result)) {
		throw std::exception("Exception : IDepthFrameSource::OpenReader()");
	}
}

void release_sensor() {
	if (pSensor) {
		pSensor->Close();
	}
	SafeRelease(pSensor);
	SafeRelease(mapper);
	SafeRelease(colorSource);
	SafeRelease(pColorReader);
	SafeRelease(depthSource);
	SafeRelease(pDepthReader);
}
void writeCSV(string filename, Mat m)
{
	ofstream myfile;
	myfile.open(filename.c_str());
	myfile << cv::format(m, cv::Formatter::FMT_CSV) << std::endl;
	myfile.close();
}