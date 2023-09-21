# 视觉定位

## 0.环境配置（Ubuntu 20.04 + OpenCV 4.5.1 + aruco 3.1.12）

### OpenCV 4.5.1

- 安装依赖库
```
sudo apt-get install build-essential libgtk2.0-dev libgtk-3-dev libavcodec-dev libavformat-dev libjpeg-dev libswscale-dev libtiff5-dev
```

- 源码下载

https://opencv.org/releases/

下载对应版本Source并解压
```
unzip opencv.zip
mv opencv-master opencv
```
- 安装
```
cd opencv
mkdir build
cd build
```

编译release模式下的OpenCV
```
cmake -D CMAKE_BUILD_TYPE=Release -D OPENCV_GENERATE_PKGCONFIG=YES -D CMAKE_INSTALL_PREFIX=/usr/local/OpenCV/Release -D WITH_FFMPEG=ON ..
```
编译
```
make -j8
```
安装
```
sudo make install
```
默认安装路径为

/usr/local/bin - executable files

/usr/local/lib - libraries (.so)

/usr/local/cmake/opencv4 - cmake package

/usr/local/include/opencv4 - headers

/usr/local/share/opencv4 - other files (e.g. trained cascades in XML format)




- 环境配置

设置库的搜索路径
``` 
sudo gedit /etc/ld.so.conf.d/opencv.conf
```
在opencv.conf添加函数库所在目录
```
    /usr/local/OpenCV/Debug/lib
    /usr/local/OpenCV/Release/lib
```
将/etc/ld.so.conf.d中的数据读入缓存
```
sudo ldconfig
```
添加pkg-config环境变量，便于pkg-config找到*.pc文件


```
cd /usr/local/OpenCV/Release/lib/pkgconfig
sudo mv opencv4.pc opencv4_release.pc
```

把路径设置在环境变量中
```
export PKG_CONFIG_PATH=/usr/local/OpenCV/Release/lib/pkgconfig:$PKG_CONFIG_PATH
```

激活
```
source /etc/profile
```

验证是否成功
```
pkg-config --libs opencv4_release
```

成功则显示
```
-L/usr/local/OpenCV/Release/lib -lopencv_gapi -lopencv_highgui -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_video -lopencv_calib3d -lopencv_features2d -lopencv_dnn -lopencv_flann -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_core
```

测试
```
pkg-config --modversion opencv4
```
应当会输出OpenCV版本信息

### aruco 3.1.12

- 下载源码

https://sourceforge.net/projects/aruco/files/?source=navbar

解压安装
```
unzip aruco-3.0.12.zip
cd aruco-3.0.12
mkdir build
cd build
cmake ..
make -j4 install
```

默认安装路径为

/usr/local/

## 1.相机标定

- 进入标定文件
```
cd aruco-3.1.12/
cd build/
cd utils_calibration/
```
找到aruco_calibration_grid_board_a4.pdf文件，打印出来

- 运行标定程序
```
./aruco_calibration live:0 out_camera_calibration.yml -m aruco_calibration_grid_board_a4.yml -size 0.038
```

live代表实时模式

0为摄像头标号
```
ls /dev/video*
```
可查看摄像头

0.038为单个marker边长（单位为米）

![Alt text](<img_README/2023-08-09 21-20-14 的屏幕截图.png>)

按照左上角提示标定

注：每次捕捉图像尽量保证角度与距离不同，减少图片相关性，以此减少误差

相机标定文件保存在当前文件夹下

## 2.单个marker实现相机的位姿估计

- 创建工程目录

```
mkdir m_aruco
cd m_aruco/
mkdir bin include lib src test yml
touch CMakeLists.txt README.md
```

- 配置工程
```
gedit CMakelists.txt
```
粘贴以下代码
```
# ----------------------------------------------------------------------------
#   Basic Configuration
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.0)
project(m_aruco LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

find_package( OpenCV REQUIRED )
# set(aruco_DIR "/home/exbot/OpenCV/aruco306-installed/share/aruco/cmake")


# If could not find could not find a package configuration file provided by "aruco", 
# set the "aruco_INCLUDE_DIRS" and the "aruco_LIBS" by mabual setting directly.
set(aruco_INCLUDE_DIRS  /usr/local/include/aruco)
set(aruco_LIBS  /usr/local/lib/libaruco.so)

find_package( aruco REQUIRED )

include_directories( include ${OpenCV_INCLUDE_DIRS} ${aruco_INCLUDE_DIRS})

add_executable( cam_pose_estimator test/cam_pose_estimator.cpp )
target_link_libraries( cam_pose_estimator ${OpenCV_LIBS} ${aruco_LIBS} )
```
其中
```
set(aruco_INCLUDE_DIRS  /usr/local/include/aruco)
set(aruco_LIBS  /usr/local/lib/libaruco.so)
```
按照aruco实际安装路径修改

- 添加源文件

在test目录下

```
touch cam_pose_estimator.cpp
gedit cam_pose_estimator.cpp
```
粘贴下列代码
```
#include "aruco.h"
#include "cvdrawingutils.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>

using namespace std;
using namespace cv;
using namespace aruco;

MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageGrey, TheInputImageCopy;
CameraParameters TheCameraParameters;

int waitTime = 0;
int ref_id = 0;
bool isVideo=false;
class CmdLineParser{int argc;char** argv;public:CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv){}   bool operator[](string param)    {int idx = -1;  for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;return (idx != -1);}    string operator()(string param, string defvalue = "-1")    {int idx = -1;for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;if (idx == -1)return defvalue;else return (argv[idx + 1]);}};
struct TimerAvrg{std::vector<double> times;size_t curr=0,n; std::chrono::high_resolution_clock::time_point begin,end;   TimerAvrg(int _n=30){n=_n;times.reserve(n);   }inline void start(){begin= std::chrono::high_resolution_clock::now();    }inline void stop(){end= std::chrono::high_resolution_clock::now();double duration=double(std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())*1e-6;if ( times.size()<n) times.push_back(duration);else{ times[curr]=duration; curr++;if (curr>=times.size()) curr=0;}}double getAvrg(){double sum=0;for(auto t:times) sum+=t;return sum/double(times.size());}};

TimerAvrg Fps;
char cam_pose[100];
char cam_vect[100];

void putText(cv::Mat &im,string text,cv::Point p,float size){
    float fact=float(im.cols)/float(640);
    if (fact<1) fact=1;
    cv::putText(im,text,p,FONT_HERSHEY_SIMPLEX, size,cv::Scalar(0,0,0),3*fact);
    cv::putText(im,text,p,FONT_HERSHEY_SIMPLEX, size,cv::Scalar(125,255,255),1*fact);
}

void printInfo(cv::Mat &im){
    float fs=float(im.cols)/float(1200);
    putText(im, "fps="+to_string(1./Fps.getAvrg()),cv::Point(10,fs*30),fs*1.0f);
    putText(im, cam_pose, cv::Point(10,fs*60),fs*1.0f);
    putText(im, cam_vect, cv::Point(10,fs*90),fs*1.0f);
}

int main(int argc, char** argv)
{
    try
    {
        CmdLineParser cml(argc, argv);
        if (argc < 2 || cml["-h"])
        {
            cerr << "Invalid number of arguments" << endl;
            cerr << "Usage: (in.avi|live[:camera_index(e.g 0 or 1)]) [-c camera_params.yml] [-s  marker_size_in_meters] [-d dictionaty:"
                    "ALL_DICTS by default] [-ref_id reference_marker's_id for estimating pose of camera] [-e use EnclosedMarker or not] [-h]" << endl;
            return false;
        }

        ///  PARSE ARGUMENTS
        string TheInputVideo = argv[1];
        ref_id = std::stoi(cml("-ref_id"));
        if(-1 == ref_id){cout<<"You need indicate a reference_marker by use the parameter [-ref_id].\n"<<endl;return false;}
	
        // read camera parameters if passed
        float TheMarkerSize = std::stof(cml("-s", "-1"));
		if (cml["-c"]) TheCameraParameters.readFromXMLFile(cml("-c"));
		MDetector.setDictionary(cml("-d", "ALL_DICTS") );  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
		MDetector.getParameters().detectEnclosedMarkers(std::stoi(cml("-e", "0"))); //if use enclosed markers, set -e 1(true), or set -e 0(false). Default value is 0.
		std::map<uint32_t, MarkerPoseTracker> MTracker;  // use a map so that for each id, we use a different pose tracker
		
        ///  OPEN VIDEO
        // read from camera or from  file
        if (TheInputVideo.find("live") != string::npos)
        {
            int vIdx = 0;
            // check if the :idx is here
            char cad[100];
            if (TheInputVideo.find(":") != string::npos)
            {
                std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
                sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
            }
            cout << "Opening camera index " << vIdx << endl;
            TheVideoCapturer.open(vIdx);
            waitTime = 10;
            isVideo=true;
        }
        else{
            TheVideoCapturer.open(TheInputVideo);
            if ( TheVideoCapturer.get(CAP_PROP_FRAME_COUNT)>=2) isVideo=true;
        }
        // check video is open
        if (!TheVideoCapturer.isOpened()) throw std::runtime_error("Could not open video");
		cv::namedWindow("in",cv::WINDOW_AUTOSIZE);
        // CONFIGURE DATA
        // read first image to get the dimensions
        TheVideoCapturer >> TheInputImage;
        if (TheCameraParameters.isValid()) TheCameraParameters.resize(TheInputImage.size());

        char key = 0;
        int index = 0,indexSave=0;
        // capture until press ESC or until the end of the video
         do
        {
            TheVideoCapturer.retrieve(TheInputImage);
		    TheInputImage.copyTo(TheInputImageCopy);

            Fps.start();
            // TheMarkers = MDetector.detect(TheInputImage, TheCameraParameters, TheMarkerSize);
            TheMarkers = MDetector.detect(TheInputImage);
		    for (auto& marker : TheMarkers)  // for each marker
                MTracker[marker.id].estimatePose(marker, TheCameraParameters, TheMarkerSize);  // call its tracker and estimate the pose
            Fps.stop();

            for (unsigned int i = 0; i < TheMarkers.size(); i++)
			{
				if(ref_id == TheMarkers[i].id)
				{
				    Mat camPosMatrix, camVecMatrix;
				    Mat vect = (Mat_<float>(3,1)<<0,0,1);
				    // R、t矩阵法
				    Mat rMatrix, tMatrix;
                    Rodrigues(TheMarkers[i].Rvec, rMatrix);
                    transpose(TheMarkers[i].Tvec, tMatrix);
                    camPosMatrix = rMatrix.inv()*(-tMatrix);
                    camVecMatrix = rMatrix.inv()*vect;
				    cout << "Camara Position: " << camPosMatrix.t() << "\nCamera Direction: " << camVecMatrix.t() << endl;
				    // 齐次矩阵法
				    Mat RTinv = MTracker[ref_id].getRTMatrix().inv();
				    camPosMatrix = RTinv(Rect(3,0,1,3)).clone();
				    camVecMatrix = RTinv(Range(0,3),Range(0,3))*vect;
				    
				    sprintf(cam_pose,"Camera Position: px=%f, py=%f, pz=%f", camPosMatrix.at<float>(0,0), 
					    camPosMatrix.at<float>(1,0), camPosMatrix.at<float>(2,0));
				    sprintf(cam_vect,"Camera Direction: dx=%f, dy=%f, dz=%f", camVecMatrix.at<float>(0,0), 
					    camVecMatrix.at<float>(1,0), camVecMatrix.at<float>(2,0));
					cout << TheMarkers[i].dict_info << " " <<  TheMarkers[i].id << endl;
				    cout << "Rvec = " << TheMarkers[i].Rvec << endl << "Tvec = " << TheMarkers[i].Tvec << endl;
				    cout << TheMarkers[i][0] << TheMarkers[i][1] << TheMarkers[i][2] << TheMarkers[i][3] << endl << endl;
				    CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
				}
				TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255), 2, true);
				// CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
			}

            // show input with augmented information and  the thresholded image
            printInfo(TheInputImageCopy);
            cv::imshow("in", TheInputImageCopy);

            key = cv::waitKey(waitTime);  // wait for key to be pressed
            if (key == 's') waitTime = waitTime == 0 ? 10 : 0;
            index++;  // number of images captured

            if (isVideo) if ( TheVideoCapturer.grab()==false) key=27;
        } while (key != 27 );
    }
    catch (std::exception& ex) {cout << "Exception :" << ex.what() << endl; }
}
```

- 编译运行

在m_aruco目录下
```
mkdir build
cd build
cmake ..
make
```
编译成功后会在bin目录下生成一个名为“cam_pose_estimator”的可执行文件

将之前生成的相机标定文件粘贴到yml文件夹下

运行
```
cd ..
./bin/cam_pose_estimator live:0 -c yml/out_camera_calibration.yml -s 0.038 -d ARUCO_MIP_36h12-ref_id 4 -e 1

```
-c代表相机标定文件

-s代表marker的边长

-d代表marker的字典

-ref_id代表参考marker的id(估计的相机位姿以该ID的marker为参考坐标系)

-e代表使用的是enclosed_marker


![Alt text](<img_README/2023-08-09 21-59-28 的屏幕截图.png>)