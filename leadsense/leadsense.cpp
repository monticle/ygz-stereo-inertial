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
	if(!fs.is_open()){
		cerr << "error to open leadsense.yaml" << endl;
		return false;	
	}
	fs << "%YAML:1.0" << endl << "---" << endl << endl;
	fs << "PureVisionMode: true" << endl;
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
	fs << "Camera.bf: " << param.baseline() / 1000 * param.leftCam.focal.x << endl;
	
	fs.close();
	return true;
}

int main(int argc, char **argv)
{
    cout << endl << "add parameter to change resolution : ./leadsense [800|400(default)]" << endl;
    google::InitGoogleLogging(argv[0]);

	std::cout << "init evo stereo camera... " << std::endl;
	StereoCamera camera;
	bool running = false;
	int width, height;

	int cnt = 0, limit = 100;
	double tempt = 0;
	StereoParameters stereoPara;//stereo parameter

	//open camera
	RESOLUTION_FPS_MODE res_mode = RESOLUTION_FPS_MODE_SD400_30;
	if(argc > 1)
	{
		int res_n = atoi(argv[2]);
		if(res_n == 800){
			res_mode = RESOLUTION_FPS_MODE_HD800_30;
		}
	}
	RESULT_CODE res = camera.open(res_mode);
	std::cout << "stereo camera open: " << result_code2str(res) << std::endl;
	//show image size
	width = camera.getImageSizeFPS().width;
	height = camera.getImageSizeFPS().height;
	std::cout << "image width:" << width << ", height:" << height << std::endl;

	if(genYaml(camera.getStereoParameters(), camera.getImageSizeFPS()) == false){
		return 2;
	}
 	// Create SLAM system. It initializes all system threads and gets ready to process frames.
    System system("leadsense.yaml");
    
	if (res == RESULT_CODE_OK)//open camera successed
	{
		//evo Mat
		evo::Mat<unsigned char> evo_image;
		//cv Mat
		cv::Mat cv_image;
		cv::Mat imLeft, imRight;
    
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();


		//running flag
		running = true;
		//main loop
		double preTime = 0;
		while (running)
		{
			// Get frames and launch the computation
			if (camera.grab(true) == RESULT_CODE_OK)
			{
				evo_image = camera.retrieveImage(SIDE_SBS);
				//Mat convert 
				cv_image = evoMat2cvMat(evo_image);
				
				// Read left and right images from file
				imLeft = cv_image(cv::Rect(0,0,width, height));
				imRight = cv_image(cv::Rect(width,0,width, height));
				double tframe = camera.getCurrentFrameTimeCode();
				cv::imshow("right", imRight);
				// Pass the images to the SLAM system
				system.AddStereo(imLeft,imRight,tframe); 

				double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(std::chrono::steady_clock::now() - t1).count();
      			t1 = std::chrono::steady_clock::now();

				cnt++;
				tempt += ttrack;
				if(cnt == limit){
					cout << "avg loop time:" << tempt / limit << endl;
					cnt = 0; 
					tempt = 0;
					cout << "frame time: " << (tframe - preTime) / limit << endl;
					preTime = tframe;
				}
			}      
			std::this_thread::sleep_for(std::chrono::microseconds(5));

			int key = cvWaitKey(10);
			if (key == 27)
			{
				running = false;
			}
		}
		camera.close();
	}
    
    return 0;
}
