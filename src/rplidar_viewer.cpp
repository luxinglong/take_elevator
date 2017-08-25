#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <cmath>

// pangolin visualization package
#include <pangolin/pangolin.h>

// rplidar standard sdk
#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

// Opencv3
#include <opencv2/opencv.hpp>

#define PI 3.14159265352
#define GRID_LENGTH 10000

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#include <unistd.h>
static inline void delay(_word_size_t ms)
{
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

using namespace rp::standalone::rplidar;
using namespace std;
using namespace cv;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool startCheck = false;
struct scanDot {
    _u8   quality;
    float angle;
    float dist;
};

bool checkElevatorOpenStatus(std::vector<scanDot> & frame) 
{
	static bool openStatus = false;
	int sumDist = 0, num = 0;
	static float averDist = 0.0;

	// select points between 335~25 degree
	std::vector<scanDot> elevatorDots;
	for (int i = 0; i < frame.size(); i++)
	{
		if (frame[i].angle < 25 || frame[i].angle > 335)
		{
			sumDist += frame[i].dist * cos(frame[i].angle*PI / 180);
			num++;
			elevatorDots.push_back(frame[i]);
		}
	}

	if (num > 1) // avoid the divident to be zero
	{
		if (startCheck)  // just compute averDist once when call for this function
		{
			averDist = sumDist / num;
			startCheck = false;
		}
		int goodDotsCount = 0;
		for (int i = 0; i < elevatorDots.size(); i++)
		{
			float distance = elevatorDots[i].dist * cos(elevatorDots[i].angle*PI / 180);
			if (distance > averDist - 100 && distance < averDist + 100)
				goodDotsCount++;
		}
		if (goodDotsCount < 0.3 * elevatorDots.size())
		{
			openStatus = true;
		}
		else if (goodDotsCount > 0.6 * elevatorDots.size())
		{
			openStatus = false;
		}
	}
	return openStatus;
}

int main(int argc, const char * argv[]) {
    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 115200;
    u_result     op_result;

    printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
           "Version: " RPLIDAR_SDK_VERSION "\n");

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);

    if (!opt_com_path) {
        opt_com_path = "/dev/cu.SLAB_USBtoUART";
    }

    // Create OpenGL window in single line
    pangolin::CreateWindowAndBind("rplidar_viewer",720,480);
  
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);
  
    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(640,480,420,420,320,240,3,100),
      pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisY)
    );
  
    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(240), 1.0, -640.0f/480.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));

    // Add Panel
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(240));
    pangolin::Var<bool> connect("ui.Connect", false, false);
    pangolin::Var<bool> disconnect("ui.Disconnect", false, false);
    pangolin::Var<bool> start_scan("ui.Start_scan", false, true);
    pangolin::Var<bool> show_grid("ui.Show_grid",false, true);
    pangolin::Var<bool> start_detect("ui.Start_detect",false, false);
    pangolin::Var<bool> save_window("ui.Save_Window",false,false);
    pangolin::Var<bool> record_window("ui.Record_Window",false,false);

    // create the driver instance
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    float  theta, dist;
    _u8 quality;
    // fetech result and print it out...
    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( !pangolin::ShouldQuit() )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        pangolin::glDrawAxis(3);

        if (pangolin::Pushed(connect))
        {
            // make connection...
            if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) 
            {
                fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                    , opt_com_path);
                goto on_finished;        
            }
            // check health...
            if (!checkRPLIDARHealth(drv)) 
            {
                goto on_finished;
            }

            rplidar_response_device_info_t devinfo;

            // retrieving the device info
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_FAIL(op_result)) {
                fprintf(stderr, "Error, cannot get device info.\n");
                goto on_finished;
            }

            // print out the device serial number, firmware and hardware version number..
            printf("RPLIDAR S/N: ");
            for (int pos = 0; pos < 16 ;++pos) {
                printf("%02X", devinfo.serialnum[pos]);
            }

            printf("\n"
                    "Firmware Ver: %d.%02d\n"
                    "Hardware Rev: %d\n"
                    , devinfo.firmware_version>>8
                    , devinfo.firmware_version & 0xFF
                    , (int)devinfo.hardware_version);

            drv->startMotor();
            // start scan...
            drv->startScan();
        }

        if (pangolin::Pushed(disconnect))
        {
            show_grid = false;
            start_scan = false;
            drv->stop();
            drv->stopMotor();
        }

        rplidar_response_measurement_node_t nodes[360*2];
        size_t count = _countof(nodes);

        glPushMatrix();
        vector<GLfloat> Twc = {1,0,0,0,0,1,0,0,0,0,1,0,5,0,5,1};
        glMultMatrixf(Twc.data());

        if (start_scan)
        {
            // 抓取一圈扫描测距数据序列
            op_result = drv->grabScanData(nodes, count);
            if (IS_OK(op_result)) 
            {
                // 对通过grabScanData()获取的扫描数据按照角度递增排序
                drv->ascendScanData(nodes, count);

                glPointSize(5.0f);
                glBegin(GL_POINTS);
                glColor3f(1.0, 0.0, 0.0);
                vector<scanDot> scan;
                for (int pos = 0; pos < (int)count ; ++pos) 
                {
                    scanDot dot;
                    dot.quality = nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
                    dot.angle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
                    dot.dist = nodes[pos].distance_q2/4.0f;
                    glVertex3f(-0.01*dot.dist*sin(dot.angle*PI/180), 0, 0.01*dot.dist*cos(dot.angle*PI/180));
                    scan.push_back(dot);
                }

                glEnd();

                if (pangolin::Pushed(start_detect))
                {
                    startCheck = true;
                    cout << "start detect" << endl;
                }
                    
            
                if( checkElevatorOpenStatus(scan) )
                {
                    // open
                    glColor3f(0.5, 0.5, 0.5);
                    pangolin::GlFont::I().Text("Open").Draw(10,0,10);
                }
                else
                {
                    // close
                    glColor3f(0.5, 0.5, 0.5);
                    pangolin::GlFont::I().Text("Close").Draw(10,0,10);
                }

                glEnd();
            }

            if (show_grid)
            {
                glLineWidth(0.5);
                glColor3f(1.0f,1.0f,1.0f);
                glBegin(GL_LINES);
                glVertex3f(0,0,0);glVertex3f(0,0,GRID_LENGTH);
                glVertex3f(0,0,0);glVertex3f(GRID_LENGTH*0.5,0,GRID_LENGTH*0.866);
                glVertex3f(0,0,0);glVertex3f(GRID_LENGTH*0.866,0,GRID_LENGTH*0.5);
                glVertex3f(0,0,0);glVertex3f(GRID_LENGTH,0,0);
                glVertex3f(0,0,0);glVertex3f(GRID_LENGTH*0.5,0,-GRID_LENGTH*0.866);
                glVertex3f(0,0,0);glVertex3f(GRID_LENGTH*0.866,0,-GRID_LENGTH*0.5);
                glVertex3f(0,0,0);glVertex3f(0,0,-GRID_LENGTH);
                glVertex3f(0,0,0);glVertex3f(-GRID_LENGTH*0.5,0,-GRID_LENGTH*0.866);
                glVertex3f(0,0,0);glVertex3f(-GRID_LENGTH*0.866,0,-GRID_LENGTH*0.5);
                glVertex3f(0,0,0);glVertex3f(-GRID_LENGTH,0,0);
                glVertex3f(0,0,0);glVertex3f(-GRID_LENGTH*0.5,0,GRID_LENGTH*0.866);
                glVertex3f(0,0,0);glVertex3f(-GRID_LENGTH*0.866,0,GRID_LENGTH*0.5);
                glEnd();
            }

            glPopMatrix();
        }

        if( pangolin::Pushed(save_window) )
            d_cam.SaveOnRender("window");
    
        if( pangolin::Pushed(record_window) )
            pangolin::DisplayBase().RecordOnRender("ffmpeg:[fps=50,bps=8388608,unique_filename]//screencap.avi");
     
        
       

        // Render graph, Swap frames and Process Events
        pangolin::FinishFrame();

    }

    drv->stop();
    drv->stopMotor();
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}