#include <iostream>
#include <sstream>
#include <math.h>

#include <pangolin/pangolin.h>

using namespace std;

const int UI_WIDTH = 240;
const float w = 2;
const float h = w*0.75;
const float z = w*0.6;


int main(/*int argc, char* argv[]*/)
{
  // Create OpenGL window in single line
  pangolin::CreateWindowAndBind("rplidar_viewer",640,480);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
    pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisY)
  );

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  // Data logger object
  pangolin::DataLog log;

  // Optionally add named labels
  std::vector<std::string> labels;
  labels.push_back(std::string("sin(t)"));
  labels.push_back(std::string("cos(t)"));
  labels.push_back(std::string("sin(t)+cos(t)"));
  log.SetLabels(labels);

  const float tinc = 0.01f;

  // OpenGL 'view' of data. We might have many views of the same data.
  pangolin::Plotter plotter(&log,0.0f,4.0f*(float)M_PI/tinc,-2.0f,2.0f,(float)M_PI/(4.0f*tinc),0.5f);
  plotter.SetBounds(0.0, 1.0, 0.0, 1.0);
  plotter.Track("$i");

  // Add some sample annotations to the plot
  //plotter.AddMarker(pangolin::Marker::Vertical,   -1000, pangolin::Marker::LessThan, pangolin::Colour::Blue().WithAlpha(0.2f) );
  //plotter.AddMarker(pangolin::Marker::Horizontal,   100, pangolin::Marker::GreaterThan, pangolin::Colour::Red().WithAlpha(0.2f) );
  //plotter.AddMarker(pangolin::Marker::Horizontal,    10, pangolin::Marker::Equal, pangolin::Colour::Green().WithAlpha(0.2f) );

  //pangolin::DisplayBase().AddDisplay(plotter);

  float t = 0;

  // Add Panel
  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
  pangolin::Var<bool> connect("ui.Connect", false, true);
  pangolin::Var<bool> start_scan("ui.Start_scan", false, true);
  pangolin::Var<bool> show_grid("ui.Show_grid",false, true);


  // Default hooks for exiting (Esc) and fullscreen (tab).
  while( !pangolin::ShouldQuit() )
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//ostringstream os;
	//os << cos(t);
	//string str = os.str();
	//labels[0] = str;
	
	if(start_scan)
		t += tinc;
	else
		t = t;
	
    //log.Log(sin(t),cos(t),sin(t)-cos(t));
    //t += tinc;
	
	// Activate efficiently by object
    d_cam.Activate(s_cam);

    // Render some stuff
    //glColor3f(1.0,1.0,1.0);
    //pangolin::glDrawColouredCube();
	
	
	pangolin::glDrawAxis(3);
	glPointSize(10.f);
	glBegin(GL_POINTS);
	glColor3f(1.0,1.0,1.0);
	glVertex3f(0.0f,0.0f,0.0f);
	glVertex3f(1,0,0);
	glVertex3f(0,2,0);
	glEnd();

    glPushMatrix();
	vector<GLfloat> Twc = {1,0,0,0,0,1,0,0,0,0,1,0,5,0,5,1};
	glMultMatrixf(Twc.data());
	glPointSize(5.0f);
	glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0);
	for (float x = -1.8; x < 1.8; x += 0.1)
	{
		glVertex3f(x, 0, sqrt(1.8*1.8-x*x));
		glVertex3f(x, 0, -sqrt(1.8*1.8-x*x));
	}
	glEnd();

	if (show_grid)
	{
		glLineWidth(2);
		glColor3f(0.0f,1.0f,0.0f);
		glBegin(GL_LINES);
		glVertex3f(0,0,0);
		glVertex3f(w,h,z);
		glVertex3f(0,0,0);
		glVertex3f(w,-h,z);
		glVertex3f(0,0,0);
		glVertex3f(-w,-h,z);
		glVertex3f(0,0,0);
		glVertex3f(-w,h,z);

		glVertex3f(w,h,z);
		glVertex3f(w,-h,z);

		glVertex3f(-w,h,z);
		glVertex3f(-w,-h,z);

		glVertex3f(-w,h,z);
		glVertex3f(w,h,z);

		glVertex3f(-w,-h,z);
		glVertex3f(w,-h,z);
		glEnd();
	}

	glPopMatrix();
	

    // Render graph, Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return 0;
}

