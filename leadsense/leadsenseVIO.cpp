#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/opencv.hpp>

#include<ygz/System.h>

//Evo SDK header
#include <evo_global_define.h>//global define
#include <evo_stereocamera.h>//stereo camera
#include <evo_mat.h>//evo Mat define
#include <evo_matconverter.h>//evo Mat converter

using namespace ygz;
using namespace std;
using namespace evo;
using namespace evo::bino;

bool genYaml(StereoParameters param, Resolution_FPS resfps){
	ofstream fs("leadsense.yaml", ios_base::out);
	if (!fs.is_open()){
		cerr << "error to open leadsense.yaml" << endl;
		return false;
	}
	fs << "%YAML:1.0" << endl << "---" << endl << endl;
	fs << "PureVisionMode: false" << endl;
	fs << "UseViewer: true" << endl << endl;
	fs << "Camera.fx: " << param.leftCam.focal.x << endl;
	fs << "Camera.fy: " << param.leftCam.focal.y << endl;
	fs << "Camera.cx: " << param.leftCam.center.x << endl;
	fs << "Camera.cy: " << param.leftCam.center.y << endl;
	fs << "Camera.k1: " << 0.0 << endl;
	fs << "Camera.k2: " << 0.0 << endl;
	fs << "Camera.p1: " << 0.0 << endl;
	fs << "Camera.p2: " << 0.0 << endl;
	fs << "Camera.width: " << resfps.width << endl;
	fs << "Camera.height: " << resfps.height << endl;
	// Camera frames per second 
	fs << "Camera.fps: " << resfps.fps << endl;
	// stereo baseline[m] times fx
	fs << "Camera.bf: " << param.baseline() / 1000 * param.leftCam.focal.x  << endl;

	fs.close();
	return true;
}

StereoCamera camera;
bool running;
// imu
std::mutex imu_lock;
VecIMU vimu;
std::vector<evo::imu::IMUData> imu_rawdata;
void retrieveIMUData()
{
	while (running){
		imu_lock.lock();

		float lastgx = -10000;
		imu_rawdata = camera.retrieveIMUData();

		for (int i = 0; i < imu_rawdata.size(); i++)
		{
			if (lastgx != imu_rawdata.at(i).gyro[0])
			{
				ygz::IMUData imudata = ygz::IMUData(imu_rawdata.at(i).gyro[0], imu_rawdata.at(i).gyro[1], imu_rawdata.at(i).gyro[2],
					imu_rawdata.at(i).accel[0], imu_rawdata.at(i).accel[1], imu_rawdata.at(i).accel[2], imu_rawdata.at(i).timestamp);
				vimu.push_back(imudata);
				lastgx = imu_rawdata.at(i).gyro[0];
			}
		}
		imu_lock.unlock();
		std::this_thread::sleep_for(std::chrono::microseconds(1));
	}
}

int main(int argc, char **argv) {
	cout << endl << "add parameter to change resolution : ./leadsense [800|400(default)]" << endl;
	google::InitGoogleLogging(argv[0]);
	std::cout << "init evo stereo camera... " << std::endl;
	int width, height;

	int cnt = 0, limit = 100;
	double tempt = 0;
	StereoParameters stereoPara;//stereo parameter

	//open camera
	RESOLUTION_FPS_MODE res_mode = RESOLUTION_FPS_MODE_SD400_30;
	if (argc > 1)
	{
		int res_n = atoi(argv[1]);
		if (res_n == 800){
			res_mode = RESOLUTION_FPS_MODE_HD800_30;
		}
	}
	RESULT_CODE res = camera.open(res_mode);
	std::cout << "stereo camera open: " << result_code2str(res) << std::endl;
	//show image size
	width = camera.getImageSizeFPS().width;
	height = camera.getImageSizeFPS().height;
	std::cout << "image width:" << width << ", height:" << height << std::endl;

	//check IMU
	if (camera.isIMUSupported() == false){
		std::cout << "IMU not support for this camera." << std::endl;
		return 2;
	}
	if (camera.startRetrieveIMU() != RESULT_CODE::RESULT_CODE_OK){
		std::cout << "Open IMU failed." << std::endl;
		return 3;
	}
	//camera.setIMUDataType(evo::imu::IMU_DATA_TYPE::IMU_DATA_TYPE_RAW);
	camera.setIMUDataType(evo::imu::IMU_DATA_TYPE::IMU_DATA_TYPE_RAW_CALIBRATED);


	if (genYaml(camera.getStereoParameters(), camera.getImageSizeFPS()) == false){
		return 4;
	}
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	System system("leadsense.yaml");

	// set TBC
	Matrix3d Rbc_;
	Vector3d tbc_;
	//Rbc_ <<
	//	1, 0, 0,
	//	0, -1, 0,
	//	0, 0, -1;
	//tbc_ <<
	//	-0.0475, 0.0032, -0.004;
	Rbc_ <<
		-1, 0, 0,
		0, -1, 0,
		0, 0, 1;
	tbc_ <<
		0.09195, 0.00, -0.0129;

	setting::TBC = SE3d(Rbc_, tbc_);

	if (res == RESULT_CODE_OK)//open camera successed
	{
		//evo Mat
		evo::Mat<unsigned char> evo_image;
		//cv Mat
		cv::Mat cv_image;
		cv::Mat imLeft, imRight;

		running = true;
		VecIMU vimu2;
		//std::thread imu_thread(retrieveIMUData);
		//imu_thread.detach();

		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
		//main loop
		while (running)
		{
			// Get frames and launch the computation
			if (camera.grab(true) == RESULT_CODE_OK)
			{
				float lastgx = -10000;
				imu_rawdata = camera.retrieveIMUData();

				for (int i = 0; i < imu_rawdata.size(); i++)
				{
					//if (lastgx != imu_rawdata.at(i).gyro[0])
					{
						ygz::IMUData imudata = ygz::IMUData(imu_rawdata.at(i).gyro[0] , imu_rawdata.at(i).gyro[1], imu_rawdata.at(i).gyro[2],
							imu_rawdata.at(i).accel[0], imu_rawdata.at(i).accel[1], imu_rawdata.at(i).accel[2], imu_rawdata.at(i).timestamp);
						vimu.push_back(imudata);
						//lastgx = imu_rawdata.at(i).gyro[0];
					}
					//std::cout <<"imu data size " << imu_rawdata.size() << std::endl;
				}

				evo_image = camera.retrieveImage(SIDE_SBS);

				//Mat convert
				cv_image = evoMat2cvMat(evo_image);

				// Read left and right images from file
				imLeft = cv_image(cv::Rect(0, 0, width, height));
				imRight = cv_image(cv::Rect(width, 0, width, height));
				double tframe = camera.getCurrentFrameTimeCode();
				//cv::imshow("right", imRight);

				// Pass the images and imu data to the SLAM system
				system.AddStereoIMU(imLeft, imRight, tframe, vimu);
				double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(std::chrono::steady_clock::now() - t1).count();
				t1 = std::chrono::steady_clock::now();

				cnt++;
				tempt += ttrack;
				if (cnt == limit){
					cout << "avg loop time:" << tempt / limit << endl;
					cnt = 0;
					tempt = 0;
				}
				vimu.clear();
			}

			running = (cv::waitKey(5) != 27);
		}
		camera.stopRetrieveIMU();
		camera.close();
	}
	cout << "finished." << endl;
	return 0;
}
