#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <random>
#include <chrono>

#include <wx/wx.h>
#include <wx/dir.h>
#include <wx/stdpaths.h>
#include <wx/stdpaths.h>
#include <wx/filename.h>


#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

#include "ESP32-CAM Library.h"

#include "json.hpp"
using json = nlohmann::json;

std::string tof;
int dis_tof = 0,N_1 = 0;
int ax,ay,az,gx,gy,gz;
double pitch,roll,yaw;
std::string imu;

cv::Rect computeROI(cv::Size2i src_sz, cv::Ptr<cv::StereoMatcher> matcher_instance);
cv::Rect computeROI(cv::Size2i src_sz, cv::Ptr<cv::StereoMatcher> matcher_instance)
{
   int min_disparity = matcher_instance->getMinDisparity();
   int num_disparities = matcher_instance->getNumDisparities();
   int block_size = matcher_instance->getBlockSize();

   int bs2 = block_size/2;
   int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

   int xmin = maxD + bs2;
   int xmax = src_sz.width + minD - bs2;
   int ymin = bs2;
   int ymax = src_sz.height - bs2;

   cv::Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
   //std::cout << "roi:" << r << std::endl;
   return r;
}

enum{
   ID_LEFT_CAMERA = 0,
   ID_RIGHT_CAMERA = 1
};

enum{
   ID_LEFT_SCREEN = 100,
   ID_RIGHT_SCREEN
};

enum{
   ID_EXIT = 200,
   ID_START_MJPEG_STREAM,
   ID_STOP_MJPEG_STREAM,
   ID_RESOLUTION,
   ID_VERTICAL_FLIP,
   ID_HORIZONTAL_FLIP,
   ID_ROTATE_90,
   ID_ROTATE_180,
   ID_ROTATE_270,
   ID_RESUME,
   ID_SAVE_POINT_CLOUD,
   ID_SEND_DATA
};

enum{
   ID_CAPTURE = 300,
   ID_AUTO_CAPTURE
};

class App:public wxApp
{
   public:
      bool OnInit();
};

class Frame:public wxFrame
{
   public:
      Frame(const wxString&);
      ~Frame();

      void CreateUI();
      void Display(const uint8_t);

      void OnSavePointCloud(wxCommandEvent&);

      void OnStartMjpegStream(wxCommandEvent&);
      void OnStopMjpegStream(wxCommandEvent&);

      void OnVerticalFlip(wxCommandEvent&);
      void OnHorizontalFlip(wxCommandEvent&);
      void OnRotate90(wxCommandEvent&);
      void OnRotate180(wxCommandEvent&);
      void OnRotate270(wxCommandEvent&);
      void OnResume(wxCommandEvent&);

      void OnCapture(wxCommandEvent&);
      void OnAutoCapture(wxCommandEvent&);

      void OnSetResolution(wxCommandEvent&);

      void SetServo();

      void OnExit(wxCommandEvent&);
   private:

      void SaveStereoMat();
      void GetAllCapture();

      void InitYOLO3();
      void postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs);
      void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);
      std::vector<cv::String> getOutputsNames(const cv::dnn::Net& yolo_net);

      void InitStereoCamera();
      void StereoCalibration(cv::Mat&,cv::Mat&);

      void YoLoV3(cv::Mat&);
      void CameraCalibration(cv::Mat&,const uint8_t);
      void BM(cv::Mat&);
      void SGBM(cv::Mat&);

      int Filter(int);

      static const uint8_t CAMERA_NUM = 2;

      wxPanel *screen[CAMERA_NUM];
      wxThread *thread;

      wxPanel *stereo_screen;
      wxPanel *odometry_screen;

      static const unsigned int SERVO_MAX = 2;
      static const unsigned int PULSE_MAX = 180;
      static const unsigned int PULSE_MIN = 1;      
      wxSlider *servos[SERVO_MAX];
      wxBoxSizer *box[SERVO_MAX];
      wxStaticText *servos_text[SERVO_MAX];  

      ESP32_CAM *esp32_cam[CAMERA_NUM];

      wxTextCtrl *left_cam_ip_ctrl;
      wxTextCtrl *right_cam_ip_ctrl;

      wxButton *start_button;
      wxButton *stop_button;

      wxRadioBox *algorithm_item;

      wxChoice *resolution;

      bool vertical_flag;
      bool horizontal_flag;
      int rotate_select;


      static const uint32_t WINDOW_WIDTH = 1300;
      static const uint32_t WINDOW_HEIGHT = 600;

      static const uint32_t IMAGE_WIDTH = 640;
      static const uint32_t IMAGE_HEIGHT = 480;
      /*
	 static const uint32_t WINDOW_WIDTH = 1300;
	 static const uint32_t WINDOW_HEIGHT = 800;      
	 static const uint32_t IMAGE_WIDTH = 480;
	 static const uint32_t IMAGE_HEIGHT = 640;    */  

      // Initialize the parameters
      static constexpr float confThreshold = 0.3f; // Confidence threshold
      static constexpr float nmsThreshold = 0.25f;  // Non-maximum suppression threshold
      static const int inpWidth = 416;//320;  // Width of network's input image
      static const int inpHeight = 416;//320; // Height of network's input image
      std::vector<std::string> classes;
      std::vector<cv::Scalar> classes_color;

      cv::dnn::Net yolo_net;

      cv::Mat L_Cam, R_Cam;
      unsigned int capture_count;

      std::chrono::steady_clock::time_point auto_capture_time;
      bool auto_capture_flag;
      bool camera_calibration_flag[CAMERA_NUM];
      std::vector<cv::Mat> Left_calibration_list,Right_calibration_list;
      cv::Mat map11, map12, map21, map22;
      cv::Mat Q;
      cv::Mat xyz;

      DECLARE_EVENT_TABLE();
};

class Thread:public wxThread
{
   public:
      Thread(Frame*);

      void* Entry();

   private:
      Frame *frame;
};

   IMPLEMENT_APP(App)
DECLARE_APP(App)

   BEGIN_EVENT_TABLE(Frame,wxFrame)
   EVT_MENU(ID_EXIT,Frame::OnExit)
   EVT_MENU(ID_SAVE_POINT_CLOUD,Frame::OnSavePointCloud)
   EVT_MENU(ID_VERTICAL_FLIP, Frame::OnVerticalFlip)
   EVT_MENU(ID_HORIZONTAL_FLIP, Frame::OnHorizontalFlip)
   EVT_MENU(ID_ROTATE_90, Frame::OnRotate90)
   EVT_MENU(ID_ROTATE_180, Frame::OnRotate180)
   EVT_MENU(ID_ROTATE_270, Frame::OnRotate270)
   EVT_MENU(ID_RESUME, Frame::OnResume)
   EVT_MENU(ID_CAPTURE, Frame::OnCapture)
   EVT_MENU(ID_AUTO_CAPTURE, Frame::OnAutoCapture)
   EVT_BUTTON(ID_START_MJPEG_STREAM,Frame::OnStartMjpegStream)
   EVT_BUTTON(ID_STOP_MJPEG_STREAM,Frame::OnStopMjpegStream)
   EVT_CHOICE(ID_RESOLUTION, Frame::OnSetResolution)
END_EVENT_TABLE()

bool App::OnInit()
{
   Frame *frame = new Frame(wxT("wxStereoCamera"));

   frame->Show(true);

   return true;
}

Frame::Frame(const wxString &title):wxFrame(NULL,wxID_ANY,title,wxDefaultPosition,wxSize(WINDOW_WIDTH,WINDOW_HEIGHT)/*wxSize(800,600)*/,wxMINIMIZE_BOX | wxCLOSE_BOX | wxCAPTION | wxSYSTEM_MENU)
{
   CreateUI();

   InitYOLO3();

   InitStereoCamera();

   esp32_cam[ID_LEFT_CAMERA] = NULL;
   esp32_cam[ID_RIGHT_CAMERA] = NULL;
   thread = NULL;

   vertical_flag = false;
   horizontal_flag = false;
   rotate_select = 0;

   auto_capture_time = std::chrono::steady_clock::now();
   auto_capture_flag = false;
   GetAllCapture();
}

Frame::~Frame()
{
   if(thread != NULL){
      thread->Delete();
      thread = NULL;
   }

   if(esp32_cam[ID_LEFT_CAMERA] != NULL){
      esp32_cam[ID_LEFT_CAMERA]->StopVideoStream();
      //delete esp32_cam[ID_LEFT_CAMERA];
      //esp32_cam[ID_LEFT_CAMERA] = NULL;
   }   

   if(esp32_cam[ID_RIGHT_CAMERA] != NULL){
      esp32_cam[ID_RIGHT_CAMERA]->StopVideoStream();
      //delete esp32_cam[ID_RIGHT_CAMERA];
      //esp32_cam[ID_RIGHT_CAMERA] = NULL;
   }     

   Close();
}

void Frame::CreateUI()
{
   wxMenu *file = new wxMenu;
   file->Append(ID_SAVE_POINT_CLOUD,wxT("Save XYZ"),wxT("Save XYZ"));
   file->AppendSeparator();
   file->Append(ID_EXIT,wxT("E&xit\tAlt-e"),wxT("Exit"));

   wxMenu *image = new wxMenu;
   image->Append(ID_VERTICAL_FLIP,wxT("Vertical Flip"),wxT("Vertical Flip"));
   image->Append(ID_HORIZONTAL_FLIP,wxT("Horizontal Flip"),wxT("Horizontal Flip"));
   image->AppendSeparator();
   image->Append(ID_ROTATE_90,wxT("Rotate 90"),wxT("Rotate 90"));
   image->Append(ID_ROTATE_180,wxT("Rotate 180"),wxT("Rotate 180"));  
   image->Append(ID_ROTATE_270,wxT("Rotate 270"),wxT("Rotate 270"));
   image->AppendSeparator();
   image->Append(ID_RESUME,wxT("Resume"),wxT("Resume"));

   wxMenu *tools = new wxMenu;
   tools->Append(ID_CAPTURE,wxT("Capture"),wxT("Capture"));
   image->AppendSeparator();
   tools->Append(ID_AUTO_CAPTURE,wxT("Auto Capture"),wxT("Auto Capture"));

   wxMenuBar *bar = new wxMenuBar;

   bar->Append(file,wxT("File"));
   bar->Append(image,wxT("Image"));
   bar->Append(tools,wxT("Tools"));
   SetMenuBar(bar);

   wxBoxSizer *top = new wxBoxSizer(wxVERTICAL);
   this->SetSizer(top);

   wxBoxSizer *screen_box = new wxBoxSizer(wxHORIZONTAL);
   top->Add(screen_box,0,wxALIGN_CENTER_HORIZONTAL | wxALL,5);

   screen[ID_RIGHT_CAMERA] = new wxPanel(this,ID_RIGHT_SCREEN,wxDefaultPosition,wxSize(IMAGE_WIDTH,IMAGE_HEIGHT));
   screen[ID_LEFT_CAMERA] = new wxPanel(this,ID_LEFT_SCREEN,wxDefaultPosition,wxSize(IMAGE_WIDTH,IMAGE_HEIGHT));
   screen_box->Add(screen[ID_RIGHT_CAMERA],0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,5);
   screen_box->Add(screen[ID_LEFT_CAMERA],0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,5);


   wxBoxSizer *group1 = new wxBoxSizer(wxVERTICAL);

   wxBoxSizer *group1_1 = new wxBoxSizer(wxHORIZONTAL);
   wxStaticText *left_cam_ip_lable = new wxStaticText(this,wxID_STATIC,wxT("Left-CAM IP: "),wxDefaultPosition,wxDefaultSize,0);
   group1_1->Add(left_cam_ip_lable, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 3);
   left_cam_ip_ctrl = new wxTextCtrl(this,wxID_STATIC,wxT("192.168.110.254"),wxDefaultPosition,wxDefaultSize,0);
   //left_cam_ip_ctrl = new wxTextCtrl(this,wxID_STATIC,wxT("192.168.1.254"),wxDefaultPosition,wxDefaultSize,0);
   group1_1->Add(left_cam_ip_ctrl, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 3);

   wxBoxSizer *group1_2 = new wxBoxSizer(wxHORIZONTAL);   
   wxStaticText *right_cam_ip_lable = new wxStaticText(this,wxID_STATIC,wxT("Right-CAM IP: "),wxDefaultPosition,wxDefaultSize,0);
   group1_2->Add(right_cam_ip_lable, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 3);
   right_cam_ip_ctrl = new wxTextCtrl(this,wxID_STATIC,wxT("192.168.110.253"),wxDefaultPosition,wxDefaultSize,0);
   //right_cam_ip_ctrl = new wxTextCtrl(this,wxID_STATIC,wxT("192.168.1.253"),wxDefaultPosition,wxDefaultSize,0);
   group1_2->Add(right_cam_ip_ctrl, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 3);

   group1->Add(group1_1,0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL);
   group1->Add(group1_2,0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL);

   wxBoxSizer *group2 = new wxBoxSizer(wxHORIZONTAL);

   start_button = new wxButton(this,ID_START_MJPEG_STREAM,wxT("&Start"),wxDefaultPosition,wxSize(100,35));
   group2->Add(start_button, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 3);
   stop_button = new wxButton(this,ID_STOP_MJPEG_STREAM,wxT("&Stop"),wxDefaultPosition,wxSize(100,35));
   group2->Add(stop_button, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 3);
   start_button->Enable(true);
   stop_button->Enable(false);

   wxArrayString algorithm_item_string;
   algorithm_item_string.Add(wxT("None"));
   algorithm_item_string.Add(wxT("YOLO V3"));
   algorithm_item_string.Add(wxT("Clibration"));
   algorithm_item_string.Add(wxT("StereoBM"));
   algorithm_item_string.Add(wxT("SGBM"));
   algorithm_item_string.Add(wxT("Remap"));
   algorithm_item = new wxRadioBox(this,wxID_ANY,wxT("Computer Vision Algorithm"),
	 wxDefaultPosition,wxDefaultSize,algorithm_item_string,3,wxRA_SPECIFY_COLS);
   group2->Add(algorithm_item, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 3);  
   algorithm_item->SetBackgroundColour(wxNullColour);   

   wxArrayString resolution_define;
   resolution_define.Add(wxT("QQVGA 160x120"));
   resolution_define.Add(wxT("QQVGA2 128x160"));
   resolution_define.Add(wxT("QCIF 176x144"));
   resolution_define.Add(wxT("HQVGA 240x176"));
   resolution_define.Add(wxT("QVGA 320x240"));
   resolution_define.Add(wxT("CIF 400x296"));
   resolution_define.Add(wxT("VGA 640x480"));
   resolution_define.Add(wxT("SVGA 800x600"));
   resolution_define.Add(wxT("XGA 1024x768")); 
   resolution_define.Add(wxT("SXGA 1280x1024"));
   resolution_define.Add(wxT("UXGA 1600x1200"));
   //resolution_define.Add(wxT("QXGA 2048*1536"));

   resolution = new wxChoice(this,ID_RESOLUTION,wxDefaultPosition,wxDefaultSize,resolution_define);
   resolution->SetSelection(7);
   group2->Add(resolution,0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,5);


   wxBoxSizer *group_frame = new wxBoxSizer(wxHORIZONTAL);
   group_frame->Add(group1,0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL);
   group_frame->Add(group2,0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL);

   top->Add(group_frame,0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL);

   wxBoxSizer *group_slider = new wxBoxSizer(wxVERTICAL);
   for(int i = 0;i < SERVO_MAX;++i){
      box[i] = new wxBoxSizer(wxHORIZONTAL);
      group_slider->Add(box[i],0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,5);

      wxString text;
      text.Printf("M%d-   ：",i + 1);
      servos_text[i] = new wxStaticText(this,wxID_STATIC,text,wxDefaultPosition,wxDefaultSize,0);
      box[i]->Add(servos_text[i],0,wxALIGN_CENTER_HORIZONTAL |  wxALIGN_CENTER_VERTICAL,5);

      servos[i] = new wxSlider(this,wxID_ANY,(PULSE_MAX+PULSE_MIN)/2,PULSE_MIN,PULSE_MAX,wxDefaultPosition,wxSize(100,20),wxSL_HORIZONTAL | wxSL_AUTOTICKS /*| wxSL_VALUE_LABEL*/);
      box[i]->Add(servos[i],0,wxALIGN_CENTER_HORIZONTAL | wxALL,5);
   }  
   group_frame->Add(group_slider,0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL);   

   CreateStatusBar(2);
   SetStatusText(wxT("wxStereoCamera"));
}

void Frame::OnStartMjpegStream(wxCommandEvent &event)
{
   if(esp32_cam[ID_LEFT_CAMERA] != NULL){
      delete esp32_cam[ID_LEFT_CAMERA];
      esp32_cam[ID_LEFT_CAMERA] = NULL;
   }

   if(esp32_cam[ID_RIGHT_CAMERA] != NULL){
      delete esp32_cam[ID_RIGHT_CAMERA];
      esp32_cam[ID_RIGHT_CAMERA] = NULL;
   }  

   esp32_cam[ID_LEFT_CAMERA] = new ESP32_CAM(std::string(left_cam_ip_ctrl->GetValue().mb_str()));
   esp32_cam[ID_LEFT_CAMERA]->StartVideoStream();
   esp32_cam[ID_LEFT_CAMERA]->SetResolution(resolution->GetCurrentSelection());

   esp32_cam[ID_RIGHT_CAMERA] = new ESP32_CAM(std::string(right_cam_ip_ctrl->GetValue().mb_str()));
   esp32_cam[ID_RIGHT_CAMERA]->StartVideoStream();
   esp32_cam[ID_RIGHT_CAMERA]->SetResolution(resolution->GetCurrentSelection());   

   if(thread != NULL){
      thread->Delete();
      thread = NULL;
   }   
   thread = new Thread(this);
   thread->Create();
   thread->Run(); 

   start_button->Enable(false);
   stop_button->Enable(true);
   left_cam_ip_ctrl->Enable(false);
   right_cam_ip_ctrl->Enable(false);
}

void Frame::OnStopMjpegStream(wxCommandEvent &event)
{
   if(thread != NULL){
      thread->Delete();
      thread = NULL;
   }   

   if(esp32_cam[ID_LEFT_CAMERA] != NULL){
      esp32_cam[ID_LEFT_CAMERA]->StopVideoStream();
      //delete esp32_cam[ID_LEFT_CAMERA];
      //esp32_cam[ID_LEFT_CAMERA] = NULL;
   }

   if(esp32_cam[ID_RIGHT_CAMERA] != NULL){
      esp32_cam[ID_RIGHT_CAMERA]->StopVideoStream();
      //delete esp32_cam[ID_RIGHT_CAMERA];
      //esp32_cam[ID_RIGHT_CAMERA] = NULL;
   }   

   start_button->Enable(true);
   stop_button->Enable(false);  
   left_cam_ip_ctrl->Enable(true);
   right_cam_ip_ctrl->Enable(true);
}

void Frame::OnSetResolution(wxCommandEvent &event)
{
   /*
      typedef enum {
      FRAMESIZE_QQVGA,    // 160x120
      FRAMESIZE_QQVGA2,   // 128x160
      FRAMESIZE_QCIF,     // 176x144
      FRAMESIZE_HQVGA,    // 240x176
      FRAMESIZE_QVGA,     // 320x240
      FRAMESIZE_CIF,      // 400x296
      FRAMESIZE_VGA,      // 640x480
      FRAMESIZE_SVGA,     // 800x600
      FRAMESIZE_XGA,      // 1024x768
      FRAMESIZE_SXGA,     // 1280x1024
      FRAMESIZE_UXGA,     // 1600x1200
      FRAMESIZE_QXGA,     // 2048*1536
      FRAMESIZE_INVALID
      } framesize_t;
      */

   if(esp32_cam[ID_LEFT_CAMERA] != NULL){
      esp32_cam[ID_LEFT_CAMERA]->SetResolution(resolution->GetCurrentSelection());
   }

   if(esp32_cam[ID_RIGHT_CAMERA] != NULL){
      esp32_cam[ID_RIGHT_CAMERA]->SetResolution(resolution->GetCurrentSelection());
   }   
}

void Frame::OnVerticalFlip(wxCommandEvent &event)
{
   vertical_flag ^= true;
}

void Frame::OnHorizontalFlip(wxCommandEvent &event)
{
   horizontal_flag ^= true;
}

void Frame::OnRotate90(wxCommandEvent &event)
{
   rotate_select = 1;
}

void Frame::OnRotate180(wxCommandEvent &event)
{
   rotate_select = 2;
}

void Frame::OnRotate270(wxCommandEvent &event)
{
   rotate_select = 3;
}

void Frame::OnResume(wxCommandEvent &event)
{
   vertical_flag = false;
   horizontal_flag = false;
   rotate_select = 0;
}

static void SaveXYZ(const char *filename, const cv::Mat &mat)
{
   const double max_z = 1.0e4;
   FILE* fp = fopen(filename, "wt");
   for(int y = 0; y < mat.rows; y++){
      for(int x = 0; x < mat.cols; x++){
	 cv::Vec3f point = mat.at<cv::Vec3f>(y, x);
	 if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z){
	    continue;
	 }
	 fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
      }
   }
   fclose(fp);
}

void Frame::OnSavePointCloud(wxCommandEvent &event)
{
   if(algorithm_item->GetSelection() == 5){
      wxStandardPaths& sp = wxStandardPaths::Get();
      wxString root;
      root.Printf("%s/",sp.GetResourcesDir()); 
      wxString xyz_file_path(root);
      xyz_file_path.Append(wxT("point_cloud.ply"));
      SaveXYZ(std::string(xyz_file_path.mb_str()).c_str(),xyz);
   }
}

void Frame::SaveStereoMat()
{
   ++capture_count;

   wxString root,left_camera_path,right_camera_path;

   wxStandardPaths& sp = wxStandardPaths::Get();
   root.Printf("%s/",sp.GetResourcesDir()); 
   left_camera_path = root;
   right_camera_path = root;

   left_camera_path.Append(wxT("Left_Cam."));
   right_camera_path.Append(wxT("Right_Cam."));

   cv::imwrite(std::string(left_camera_path.mb_str()) + std::to_string(capture_count) + ".jpg",L_Cam);
   cv::imwrite(std::string(right_camera_path.mb_str()) + std::to_string(capture_count) + ".jpg",R_Cam);   
}

void Frame::GetAllCapture()
{
   wxStandardPaths& sp = wxStandardPaths::Get();
   wxDir dir;
   wxArrayString files;

   //dir.GetAllFiles(sp.GetResourcesDir(),&files,wxEmptyString,wxDIR_FILES);
   //wxString targer = sp.GetResourcesDir() + wxT("/stereo_calibration_image");
   wxString targer = sp.GetResourcesDir();
   dir.GetAllFiles(targer,&files,wxEmptyString,wxDIR_FILES);

   files.Sort(false);

   unsigned int l_count = 0,r_count = 0;

   for(size_t i = 0;i < files.size();++i){
      wxFileName filename(files[i]);
      wxLogDebug(filename.GetFullName());
      if(filename.GetFullName().find(wxT("Left_Cam")) != wxNOT_FOUND){
	 ++l_count;
      }
      if(filename.GetFullName().find(wxT("Right_Cam")) != wxNOT_FOUND){
	 ++r_count;
      }      
   }

   if(l_count == r_count){
      capture_count = r_count;
   }
   else{
      capture_count = 0;
      wxMessageBox(wxT("Error"));
   }
}

void Frame::OnCapture(wxCommandEvent &event)
{
   SaveStereoMat();
}

void Frame::OnAutoCapture(wxCommandEvent &event)
{
   auto_capture_flag ^= true;
}

void Frame::SetServo()
{

   int m1_pos = 0;
   std::string s = std::to_string(servos[0]->GetValue());
   m1_pos = servos[0]->GetValue();
   servos_text[0]->SetLabel(std::string("M1-") + std::to_string(m1_pos));

   int m2_pos = 0;
   s = std::to_string(servos[1]->GetValue());
   m2_pos = servos[1]->GetValue();
   servos_text[1]->SetLabel(std::string("M2-") + std::to_string(m2_pos));
   

   esp32_cam[ID_RIGHT_CAMERA]->SetServoPosition(m1_pos,m2_pos);
}

int Frame::Filter(int value)
{
   N_1 = N_1 + (1.0f - exp(-50.0f/100.0f) ) * (value - N_1);
   return N_1;
}


void Frame::Display(const uint8_t ID)
{
   wxString str;


   //std::string tof;
   if(ID == ID_LEFT_CAMERA){
      tof = esp32_cam[ID]->GetToFDistance();
      json j = json::parse(tof);
      dis_tof = j.at("distance").get<int>();
      dis_tof = Filter(dis_tof);
   }

   //std::string imu;
   if(ID == ID_RIGHT_CAMERA){
      imu = esp32_cam[ID]->GetIMUStatus();
      json j = json::parse(imu);
      ax = j.at("ax").get<int>();
      ay = j.at("ay").get<int>();
      az = j.at("az").get<int>();
      gx = j.at("gx").get<int>();
      gy = j.at("gy").get<int>();
      gz = j.at("gz").get<int>();
      pitch = j.at("pitch").get<double>();
      roll = j.at("roll").get<double>();
      yaw = j.at("yaw").get<double>();
   }

   str.Printf(wxT("Size: %.2fKb , ToF:%.2fcm, ax:%d, ay:%d, az:%d, gx:%d, gy:%d, gz:%d"),(esp32_cam[ID]->GetFrameSize() / 1024.0f),dis_tof/10.0f,ax,ay,az,gx,gy,gz);
   //str.Printf(wxT(" ,Size: %.2fKb ,Opencv%d, %.2fcm, pitch:%.2f, roll:%.2f, yaw:%.2f"),(esp32_cam[ID]->GetFrameSize() / 1024.0f),CV_MAJOR_VERSION,dis_tof/10.0f,pitch,roll,yaw);
   //str.Printf(wxT("Opencv%d, Size: %.2fKb , ToF:%.2fcm, IMU pitch:%.2f roll:%.2f"),CV_MAJOR_VERSION,(esp32_cam[ID]->GetFrameSize() / 1024.0f),dis_tof/10.0f,pitch,roll);
   //SetStatusText(wxDateTime::Now().Format() + str);
   SetStatusText(str);


   cv::Mat src,dst;
   src = esp32_cam[ID]->GetFrame();
   if(src.empty()){
      wxLogDebug("Empty");
      return;
   }

   if(ID == ID_LEFT_CAMERA){
      //cv::rotate(src,src,cv::ROTATE_90_COUNTERCLOCKWISE);
      L_Cam = src.clone();
   }
   else if(ID == ID_RIGHT_CAMERA){
      //cv::rotate(src,src,cv::ROTATE_90_COUNTERCLOCKWISE);
      R_Cam = src.clone();
   }

   if(vertical_flag){
      cv::flip(src,src,0);
   }

   if(horizontal_flag){
      cv::flip(src,src,1);
   }

   if(rotate_select == 1){
      cv::rotate(src,src,cv::ROTATE_90_CLOCKWISE);
   }
   else if(rotate_select == 2){
      cv::rotate(src,src,cv::ROTATE_180);
   }
   else if(rotate_select == 3){
      cv::rotate(src,src,cv::ROTATE_90_COUNTERCLOCKWISE);
   }

   if(algorithm_item->GetSelection() == 0){
      cv::cvtColor(src,dst,cv::COLOR_RGB2BGR/*CV_RGB2BGR*/);
   }
   else if(algorithm_item->GetSelection() == 1){
      YoLoV3(src);
      cv::cvtColor(src,dst,cv::COLOR_RGB2BGR/*CV_RGB2BGR*/);
   }
   else if(algorithm_item->GetSelection() == 2){
      CameraCalibration(src,ID);
      cv::cvtColor(src,dst,cv::COLOR_RGB2BGR);
   }
   else if(algorithm_item->GetSelection() == 3){
      /*cv::cvtColor(src,dst,cv::COLOR_RGB2GRAY);
	cv::cvtColor(dst,dst,cv::COLOR_GRAY2BGR);*/
      if(!L_Cam.empty() || !R_Cam.empty()){
	 StereoCalibration(L_Cam,R_Cam);

	 //SGBM(dst);
	 BM(dst);

	 if(ID == ID_RIGHT_CAMERA){
	    //src = left_disp_vis.clone();

	    //cv::cvtColor( filtered_disp_vis, filtered_disp_vis, cv::COLOR_GRAY2RGB);
	    src = dst.clone();

	    //cv::cvtColor( left_disp8, left_disp8, cv::COLOR_GRAY2RGB);
	    //src = left_disp8.clone();
	 }
      }


      if(ID == ID_LEFT_CAMERA){
	 src = L_Cam;
      }
      /*else if(ID == ID_RIGHT_CAMERA){
	src = R_Cam;
	}*/

      cv::cvtColor(src,dst,cv::COLOR_RGB2BGR);
   }
   else if(algorithm_item->GetSelection() == 4){
      /*cv::cvtColor(src,dst,cv::COLOR_RGB2GRAY);
	cv::cvtColor(dst,dst,cv::COLOR_GRAY2BGR);*/
      if(!L_Cam.empty() || !R_Cam.empty()){
	 StereoCalibration(L_Cam,R_Cam);

	 SGBM(dst);
	 //BM(dst);

	 if(ID == ID_RIGHT_CAMERA){
	    //src = left_disp_vis.clone();

	    //cv::cvtColor( filtered_disp_vis, filtered_disp_vis, cv::COLOR_GRAY2RGB);
	    src = dst.clone();

	    //cv::cvtColor( left_disp8, left_disp8, cv::COLOR_GRAY2RGB);
	    //src = left_disp8.clone();
	 }
      }


      if(ID == ID_LEFT_CAMERA){
	 src = L_Cam;
      }
      /*else if(ID == ID_RIGHT_CAMERA){
	src = R_Cam;
      }*/

      cv::cvtColor(src,dst,cv::COLOR_RGB2BGR);
   }
   else if(algorithm_item->GetSelection() == 5){
      if( (!L_Cam.empty() || !R_Cam.empty())){
	 StereoCalibration(L_Cam,R_Cam);
      }

      if(ID == ID_LEFT_CAMERA){
	 src = L_Cam;
      }
      else if(ID == ID_RIGHT_CAMERA){
	 src = R_Cam;
      }

      cv::cvtColor(src,dst,cv::COLOR_RGB2BGR);
   }
   

   cv::Size size = dst.size();
   wxImage *image = new wxImage(size.width,size.height,dst.data,true);
   image->Rescale(IMAGE_WIDTH,IMAGE_HEIGHT);

   wxBitmap *bitmap = new wxBitmap(*image);

   int x,y,width,height;

   wxClientDC dc(screen[ID]);
   dc.GetClippingBox(&x,&y,&width,&height);
   dc.DrawBitmap(*bitmap,x,y);

   delete bitmap;
   delete image;
}

void Frame::YoLoV3(cv::Mat &src)
{
   cv::Mat blob;
   // Create a 4D blob from a frame.
   cv::dnn::blobFromImage(src, blob, 1/255.0, cvSize(inpWidth, inpHeight), cv::Scalar(0,0,0), true, false);

   //Sets the input to the network
   yolo_net.setInput(blob);

   // Runs the forward pass to get output of the output layers
   std::vector<cv::Mat> outs;
   yolo_net.forward(outs, getOutputsNames(yolo_net));

   // Remove the bounding boxes with low confidence
   postprocess(src, outs);   

   // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
   std::vector<double> layersTimes;
   double freq = cv::getTickFrequency() / 1000;
   double t = yolo_net.getPerfProfile(layersTimes) / freq;
   std::string label = cv::format("Inference time for a frame : %.2f ms", t);
   cv::putText(src, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));	
}

void Frame::CameraCalibration(cv::Mat &src,const uint8_t ID)
{
   cv::Mat image_gray;
   cv::cvtColor(src, image_gray, cv::COLOR_BGR2GRAY);

   std::vector<cv::Point2f> corners;

   bool ret = cv::findChessboardCorners(image_gray,
	 cv::Size(9, 6),
	 corners,
	 cv::CALIB_CB_ADAPTIVE_THRESH |
	 cv::CALIB_CB_NORMALIZE_IMAGE);

   if(ret){

      //cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30,0.1);
      cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30,0.01);
      //cv::cornerSubPix(image_gray, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
      cv::cornerSubPix(image_gray, corners, cv::Size(11,11), cv::Size(-1, -1), criteria);

      cv::drawChessboardCorners(src, cv::Size(9, 6), corners, ret);

      camera_calibration_flag[ID] = true;

      if(auto_capture_flag){
	 if(ID == ID_RIGHT_CAMERA){
	    if(camera_calibration_flag[ID_LEFT_CAMERA]){
	       if(std::chrono::steady_clock::duration(std::chrono::steady_clock::now() - auto_capture_time).count() > std::chrono::steady_clock::period::den){
		  SaveStereoMat();
		  auto_capture_time = std::chrono::steady_clock::now();
	       }
	    }
	 }
      }

   }
   else{
      camera_calibration_flag[ID] = false;
   }

}

void Frame::InitStereoCamera()
{
   wxStandardPaths& sp = wxStandardPaths::Get();
   std::string intrinsic_filename = std::string(sp.GetResourcesDir().mb_str()) + std::string("/intrinsics.yml");   

   cv::FileStorage fs(intrinsic_filename, cv::FileStorage::READ);

   if(!fs.isOpened()){
      wxLogDebug("intrinsics file error");
   }

   cv::Mat M1, D1, M2, D2;
   fs["M1"] >> M1;
   fs["D1"] >> D1;
   fs["M2"] >> M2;
   fs["D2"] >> D2;

   float scale = 1;
   /*if (scale != 1.f)
     {
     Mat temp1, temp2;
     int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
     resize(img1, temp1, Size(), scale, scale, method);
     img1 = temp1;
     resize(img2, temp2, Size(), scale, scale, method);
     img2 = temp2;
     }*/   
   M1 *= scale;
   M2 *= scale;   

   fs.release();

   std::string extrinsic_filename = std::string(sp.GetResourcesDir().mb_str()) + std::string("/extrinsics.yml");
   fs.open(extrinsic_filename, cv::FileStorage::READ);

   if(!fs.isOpened()){
      wxLogDebug("extrinsics file error");
   }

   cv::Mat R, T, R1, P1, R2, P2;
   fs["R"] >> R;
   fs["T"] >> T;
   fs.release();

   cv::Size img_size(800,600);
   //cv::Size img_size(600,800);
   cv::Rect roi1, roi2;
   cv::stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

   initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
   initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);   

}

void Frame::StereoCalibration(cv::Mat &left,cv::Mat &right)
{
   cv::Mat img1r, img2r;
   cv::remap(left, img1r, map11, map12, cv::INTER_LINEAR);       
   cv::remap(right, img2r, map21, map22, cv::INTER_LINEAR);

   left = img1r.clone();
   right = img2r.clone();   
}

void Frame::InitYOLO3()
{
   wxString str;
   wxStandardPaths& sp = wxStandardPaths::Get();
   str.Printf("%s/coco.names",sp.GetResourcesDir());
   // Load names of classes
   std::string classesFile(str.mb_str());
   std::ifstream ifs(classesFile.c_str());
   std::string line;

   std::random_device rd;
   std::default_random_engine gen = std::default_random_engine(rd());
   std::uniform_int_distribution<int> dis(0,255);

   while(getline(ifs, line)){
      classes.push_back(line);

      classes_color.push_back(cv::Scalar(dis(gen),dis(gen),dis(gen)));
   }

   // Give the configuration and weight files for the model
   str.Printf("%s/yolov3.cfg",sp.GetResourcesDir());
   //str.Printf("%s/yolov3-tiny.cfg",sp.GetResourcesDir());
   cv::String modelConfiguration(str.mb_str());
   str.Printf("%s/yolov3.weights",sp.GetResourcesDir());
   //str.Printf("%s/yolov3-tiny.weights",sp.GetResourcesDir());
   cv::String modelWeights(str.mb_str());

   // Load the network
   yolo_net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);
   yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
   yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);   
}

// Remove the bounding boxes with low confidence using non-maxima suppression
void Frame::postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs)
{
   std::vector<int> classIds;
   std::vector<float> confidences;
   std::vector<cv::Rect> boxes;

   for (size_t i = 0; i < outs.size(); ++i)
   {
      // Scan through all the bounding boxes output from the network and keep only the
      // ones with high confidence scores. Assign the box's class label as the class
      // with the highest score for the box.
      float* data = (float*)outs[i].data;
      for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
      {
	 cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
	 cv::Point classIdPoint;
	 double confidence;
	 // Get the value and location of the maximum score
	 minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
	 if (confidence > confThreshold)
	 {
	    int centerX = (int)(data[0] * frame.cols);
	    int centerY = (int)(data[1] * frame.rows);
	    int width = (int)(data[2] * frame.cols);
	    int height = (int)(data[3] * frame.rows);
	    int left = centerX - width / 2;
	    int top = centerY - height / 2;

	    classIds.push_back(classIdPoint.x);
	    confidences.push_back((float)confidence);
	    boxes.push_back(cv::Rect(left, top, width, height));
	 }
      }
   }

   // Perform non maximum suppression to eliminate redundant overlapping boxes with
   // lower confidences
   std::vector<int> indices;
   cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
   for (size_t i = 0; i < indices.size(); ++i)
   {
      int idx = indices[i];
      cv::Rect box = boxes[idx];
      drawPred(classIds[idx], confidences[idx], box.x, box.y,
	    box.x + box.width, box.y + box.height, frame);
   }
}

// Draw the predicted bounding box
void Frame::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame)
{
   //Draw a rectangle displaying the bounding box
   //cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);

   //Get the label for the class name and its confidence
   std::string label = cv::format("%.2f", conf);
   if (!classes.empty())
   {
      CV_Assert(classId < (int)classes.size());
      label = classes[classId] + ":" + label;
      //Draw a rectangle displaying the bounding box
      cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), classes_color[classId], 3);
   }

   //Display the label at the top of the bounding box
   int baseLine;
   cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
   top = cv::max(top, labelSize.height);
   cv::rectangle(frame, cv::Point(left, top - round(1.5*labelSize.height)), cv::Point(left + round(1.5*labelSize.width), top + baseLine), classes_color[classId]/*cv::Scalar(255, 255, 255)*/, cv::FILLED);
   cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,0),1);
}

// Get the names of the output layers
std::vector<cv::String> Frame::getOutputsNames(const cv::dnn::Net& yolo_net)
{
   static std::vector<cv::String> names;
   if (names.empty())
   {
      //Get the indices of the output layers, i.e. the layers with unconnected outputs
      std::vector<int> outLayers = yolo_net.getUnconnectedOutLayers();

      //get the names of all the layers in the network
      std::vector<cv::String> layersNames = yolo_net.getLayerNames();

      // Get the names of the output layers in names
      names.resize(outLayers.size());
      for (size_t i = 0; i < outLayers.size(); ++i)
	 names[i] = layersNames[outLayers[i] - 1];
   }
   return names;
}

void Frame::BM(cv::Mat &dst)
{
   int sgbmWinSize = 13;
   cv::Ptr<cv::StereoBM> left_matcher = cv::StereoBM::create(64,sgbmWinSize);
   cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
   wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
   cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);  

   cv::Mat left_for_matcher = L_Cam.clone(),right_for_matcher = R_Cam.clone();

   cv::cvtColor(left_for_matcher,  left_for_matcher, cv::COLOR_BGR2GRAY);
   cv::cvtColor(right_for_matcher,  right_for_matcher, cv::COLOR_BGR2GRAY);

   cv::Mat left_disp,right_disp;
   left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
   right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);	   

   cv::Mat filtered_disp;
   wls_filter->setLambda(80000);
   wls_filter->setSigmaColor(1.5f);
   wls_filter->filter(left_disp,L_Cam,filtered_disp,right_disp);

   double vis_mult = 5.0f;

   cv::Mat filtered_disp_vis;
   cv::ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
   cv::applyColorMap( filtered_disp_vis, filtered_disp_vis, cv::COLORMAP_JET);

   /*
   cv::Rect ROI = wls_filter->getROI();
   
   cv::Mat GT_disp;
   cv::ximgproc::computeMSE(GT_disp,filtered_disp,ROI);
   cv::ximgproc::computeBadPixelPercent(GT_disp,filtered_disp,ROI);
   cv::Mat GT_disp_vis;
   cv::ximgproc::getDisparityVis(GT_disp,GT_disp_vis,vis_mult);   
*/
   dst = filtered_disp_vis.clone();	
   //dst = GT_disp_vis.clone();
}

void Frame::SGBM(cv::Mat &dst)
{

   int sgbmWinSize = 13;
   int cn = L_Cam.channels();

   cv::Size img_size(800,600);
   //cv::Size img_size(600,800);
   int numberOfDisparities = ((img_size.width/8) + 15) & -16;
   //numberOfDisparities = 176;

   cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,numberOfDisparities,sgbmWinSize);
   sgbm->setPreFilterCap(63);
   //sgbm->setPreFilterCap(15);
   sgbm->setBlockSize(sgbmWinSize);
   sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
   sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
   //sgbm->setP1(251);
   //sgbm->setP2(751);	 
   sgbm->setMinDisparity(0);
   sgbm->setNumDisparities(numberOfDisparities);
   //sgbm->setNumDisparities(176);
   //sgbm->setUniquenessRatio(10);
   sgbm->setUniquenessRatio(10);
   sgbm->setSpeckleWindowSize(100);
   //sgbm->setSpeckleWindowSize(0);
   sgbm->setSpeckleRange(32);
   //sgbm->setSpeckleRange(0);
   sgbm->setDisp12MaxDiff(1);
   //sgbm->setDisp12MaxDiff(35);
   sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY/*cv::StereoSGBM::MODE_SGBM*/);

   cv::Mat left_disp, left_disp8,right_disp, right_disp8;
   float disparity_multiplier = 1.0f;
   sgbm->compute(L_Cam, R_Cam, left_disp);
   if (left_disp.type() == CV_16S){
      disparity_multiplier = 16.0f;	 
   }
   left_disp.convertTo(left_disp8, CV_8U, 255/(numberOfDisparities*16.));

   cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(sgbm);
   sgbm->compute(R_Cam, L_Cam, right_disp);

   cv::Rect ROI;
   cv::Mat left_for_matcher;
   cv::cvtColor(L_Cam,  left_for_matcher, cv::COLOR_BGR2GRAY);
   ROI = computeROI(left_for_matcher.size(),sgbm);

   cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
   //wls_filter = cv::ximgproc::createDisparityWLSFilter(sgbm);
   wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
   wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*3));
   wls_filter->setLambda(80000.0f);
   wls_filter->setSigmaColor(1.5f);
   cv::Mat filtered_disp;
   //wls_filter->filter(left_disp,L_Cam,filtered_disp,/*cv::Mat()*/right_disp,ROI);
   wls_filter->filter(left_disp,L_Cam,filtered_disp,right_disp);

   /*
      cv::Mat disp_rgb;
      filtered_disp.convertTo(disp_rgb, CV_32F);
      left_disp.convertTo(left_disp8, CV_8U, 255/(numberOfDisparities*16.));
      cv::applyColorMap(left_disp8,disp_rgb,cv::COLORMAP_JET); */
   /* 
      cv::Mat left_disp_vis;
      left_disp.convertTo(left_disp_vis, CV_8U, 255/(numberOfDisparities*16.));
      */

   double vis_mult = 5.0f;
   cv::Mat filtered_disp_vis;
   cv::ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
   cv::applyColorMap( filtered_disp_vis, filtered_disp_vis, cv::COLORMAP_JET);

   dst = filtered_disp_vis.clone();

   cv::Mat floatDisp;
   left_disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);
   cv::reprojectImageTo3D(floatDisp, xyz, Q, true);

}

void Frame::OnExit(wxCommandEvent &event)
{
   Close();
}

Thread::Thread(Frame *parent):wxThread(wxTHREAD_DETACHED)
{
   frame = parent;
}

void* Thread::Entry()
{
   while(!TestDestroy()){
      //wxMutexGuiEnter();

      frame->SetServo();
      frame->Display(ID_LEFT_CAMERA);
      frame->Display(ID_RIGHT_CAMERA);

      //wxMutexGuiLeave();
      //Sleep(25);
   }

   return NULL;
}
