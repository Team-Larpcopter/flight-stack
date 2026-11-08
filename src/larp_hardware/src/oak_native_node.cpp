// Native stereo/IMU acquisition. OAK performs lens undistortion in hardware.
//
// Rectified pair (publish_rectified, default on): each mono camera has a second,
// smaller output (rectified_width x rectified_height, default 800x500) into the
// on-device stereo engine, used only for its rectified, hardware-synced
// left/right images (no depth is published). For stereo networks on the Jetson:
// /native_oak/rectified/{left,right}/image + camera_info, one mid-exposure stamp
// per pair; left is native_oak_depth_optical (the frame stereo depth is in). The
// rectified geometry is exported to <factory_calibration_out>.rectified.json.
// Publish raw-axis SI measurements; optional alignment interpolates acceleration
// between measured samples at native gyro timestamps. No local bias/scale correction.
#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/device/Device.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/IMU.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/common/CameraExposureOffset.hpp>
#include <map>
#include <depthai/pipeline/datatype/IMUData.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/MessageQueue.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <larp_hardware/imu_alignment.hpp>
#include <deque>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <sstream>
#include <thread>
using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;
class NativeOak final:public rclcpp::Node {
 public:
  NativeOak():Node("oak_native") {
    const bool connect=declare_parameter<bool>("connect_device",false);
    publish_images_=declare_parameter<bool>("publish_images",false);
    align_imu_=declare_parameter<bool>("align_imu",false);
    // Reject stale launch arguments rather than silently pretending to apply them.
    if(!declare_parameter<std::string>("imu_calibration_file","").empty())
      throw std::invalid_argument("imu_calibration_file is no longer supported: IMU output is uncorrected raw-axis SI data");
    const auto fps=declare_parameter<double>("camera_fps",30.);
    const int width=declare_parameter<int>("vio_width",1280),height=declare_parameter<int>("vio_height",800);
    const int accel_hz=declare_parameter<int>("accel_hz",250),gyro_hz=declare_parameter<int>("gyro_hz",200);
    const int exposure=declare_parameter<int>("max_exposure_us",0);
    capture_seconds_=declare_parameter<double>("capture_seconds",0.);
    const auto capture_file=declare_parameter<std::string>("capture_csv","");
    if(capture_seconds_<0 || !std::isfinite(capture_seconds_))throw std::invalid_argument("invalid capture duration");
    const auto calibration_out=declare_parameter<std::string>("factory_calibration_out","");
    publish_rectified_=declare_parameter<bool>("publish_rectified",true);
    const int rect_width=declare_parameter<int>("rectified_width",800),rect_height=declare_parameter<int>("rectified_height",500);
    // DepthAI 3.x runs dynamic stereo recalibration at every pipeline start by default
    // and may flash the result to the camera. Off unless chosen explicitly.
    const auto auto_calibration=declare_parameter<std::string>("auto_calibration","OFF");
    if(!connect) {RCLCPP_INFO(get_logger(),"Device connection disabled; no camera opened");return;}
    if(fps<=0 || fps>60 || width<=0 || height<=0 || accel_hz<=0 || gyro_hz<=0 || exposure<0)
      throw std::invalid_argument("invalid acquisition settings");
    if(calibration_out.empty())throw std::invalid_argument("factory_calibration_out is required for provenance");
    if(std::filesystem::exists(calibration_out))throw std::invalid_argument("factory output already exists");
    if(!capture_file.empty()) {
      if(std::filesystem::exists(capture_file))throw std::invalid_argument("capture output already exists");
      capture_prefix_=capture_file;
      capture_.open(capture_file);if(!capture_)throw std::runtime_error("cannot open capture output");
      capture_<<"stream,capture_ns,device_ns,arrival_ns,sequence,exposure_us,iso,x,y,z\n";
      capture_<<std::setprecision(12);
    }
    if(align_imu_) {
      aligned_imu_=create_publisher<sensor_msgs::msg::Imu>("/autonomy/imu/data",rclcpp::SensorDataQoS().keep_last(512));
    }
    if(publish_rectified_ && (rect_width<=0 || rect_height<=0 || rect_width>width || rect_height>height))
      throw std::invalid_argument("invalid rectified size");
    pipeline_=std::make_unique<dai::Pipeline>();
    {
      static const std::map<std::string,dai::PipelineAutoCalibrationMode> modes{
        {"OFF",dai::PipelineAutoCalibrationMode::OFF},{"ON_START",dai::PipelineAutoCalibrationMode::ON_START},
        {"CONTINUOUS",dai::PipelineAutoCalibrationMode::CONTINUOUS}};
      if(!modes.count(auto_calibration))throw std::invalid_argument("auto_calibration must be OFF, ON_START or CONTINUOUS");
      pipeline_->setAutoCalibrationMode(modes.at(auto_calibration));
    }
    auto device=pipeline_->getDefaultDevice();
    auto calibration=device->readCalibration();
    calibration.eepromToJsonFile(calibration_out);
    // Read before running: fail if factory extrinsics are unavailable. Calibration
    // conversion and body mount validation remain a separate admission step.
    nlohmann::json extrinsics;
    extrinsics["camera_to_imu_cm"]["left"]=calibration.getCameraToImuExtrinsics(dai::CameraBoardSocket::CAM_B);
    extrinsics["camera_to_imu_cm"]["right"]=calibration.getCameraToImuExtrinsics(dai::CameraBoardSocket::CAM_C);
    std::ofstream(calibration_out+".extrinsics.json")<<extrinsics.dump(2)<<'\n';
    dai::Node::Output* stereo_inputs[2]={nullptr,nullptr};
    for(int i=0;i<2;++i) {
      auto cam=pipeline_->create<dai::node::Camera>();
      cam->build(i==0?dai::CameraBoardSocket::CAM_B:dai::CameraBoardSocket::CAM_C,std::nullopt,float(fps));
      if(exposure>0)cam->initialControl.setAutoExposureLimit(exposure);
      auto vio=cam->requestOutput({uint32_t(width),uint32_t(height)},dai::ImgFrame::Type::GRAY8,dai::ImgResizeMode::CROP,float(fps),true);
      images_[i]=vio->createOutputQueue(8,false);
      if(publish_rectified_)  // not undistorted: the stereo engine rectifies with the full calibration itself
        stereo_inputs[i]=cam->requestOutput({uint32_t(rect_width),uint32_t(rect_height)},dai::ImgFrame::Type::GRAY8,
                                            dai::ImgResizeMode::CROP,float(fps));
      const std::string side=i==0?"left":"right";
      image_pub_[i]=create_publisher<sensor_msgs::msg::Image>("/native_oak/"+side+"/image_undistorted",rclcpp::SensorDataQoS());
      info_pub_[i]=create_publisher<sensor_msgs::msg::CameraInfo>("/native_oak/"+side+"/camera_info",rclcpp::SensorDataQoS());
    }
    if(publish_rectified_) {
      auto stereo=pipeline_->create<dai::node::StereoDepth>();
      stereo->build(*stereo_inputs[0],*stereo_inputs[1],dai::node::StereoDepth::PresetMode::ROBOTICS);
      stereo->setDepthAlign(dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT);
      // Only the rectified images are used: make the depth part as cheap as possible.
      // The post-filters in particular delay every output (rectified pair 190 ms
      // after capture with them, 60 ms without, on skybeta-15).
      stereo->setLeftRightCheck(false);stereo->setSubpixel(false);
      auto& pp=stereo->initialConfig->postProcessing;
      pp.median=dai::StereoDepthConfig::MedianFilter::MEDIAN_OFF;pp.decimationFilter.decimationFactor=1;
      pp.spatialFilter.enable=false;pp.temporalFilter.enable=false;pp.speckleFilter.enable=false;
      // Non-blocking: a slow stereo stage drops rectified frames, never stalls the VIO images.
      stereo->left.setBlocking(false);stereo->right.setBlocking(false);
      rect_[0]=stereo->rectifiedLeft.createOutputQueue(4,false);
      rect_[1]=stereo->rectifiedRight.createOutputQueue(4,false);
      for(int i=0;i<2;++i) {
        const std::string side=i==0?"left":"right";
        rect_pub_[i]=create_publisher<sensor_msgs::msg::Image>("/native_oak/rectified/"+side+"/image",rclcpp::SensorDataQoS());
        rect_info_pub_[i]=create_publisher<sensor_msgs::msg::CameraInfo>("/native_oak/rectified/"+side+"/camera_info",rclcpp::SensorDataQoS());
      }
      // R maps the left (undistorted) optical frame into the rectified frame.
      const auto r=calibration.getStereoLeftRectificationRotation();
      for(int a=0;a<3;++a)for(int b=0;b<3;++b)rect_r_[a*3+b]=r.at(a).at(b);
      baseline_m_=calibration.getBaselineDistance(dai::CameraBoardSocket::CAM_C,dai::CameraBoardSocket::CAM_B,false)*.01;
      rect_export_=calibration_out+".rectified.json";
      RCLCPP_INFO(get_logger(),"Rectified pair on the camera: %dx%d, baseline %.4f m, auto calibration %s",
                  rect_width,rect_height,baseline_m_,auto_calibration.c_str());
    }
    auto imu=pipeline_->create<dai::node::IMU>();
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW,accel_hz);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW,gyro_hz);
    imu->setBatchReportThreshold(1);imu->setMaxBatchReports(10);
    imu_=imu->out.createOutputQueue(128,false);
    accel_pub_=create_publisher<geometry_msgs::msg::Vector3Stamped>("/native_oak/acceleration_raw",rclcpp::SensorDataQoS().keep_last(256));
    gyro_pub_=create_publisher<geometry_msgs::msg::Vector3Stamped>("/native_oak/gyro_raw",rclcpp::SensorDataQoS().keep_last(256));
    metadata_=create_publisher<std_msgs::msg::String>("/native_oak/frame_metadata",10);
    // One offset per acquisition session preserves intervals; monitor host-clock
    // discontinuities rather than silently moving individual sample timestamps.
    offset_=now().nanoseconds()-ns(Clock::now());
    pipeline_->start();started_=Clock::now();timer_=create_wall_timer(1ms,[this]{poll();});
  }
  ~NativeOak() override {if(pipeline_)pipeline_->stop();}
 private:
  static int64_t ns(Clock::time_point t){return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();}
  builtin_interfaces::msg::Time stamp(Clock::time_point t){return rclcpp::Time(ns(t)+offset_);}
  void image(const std::shared_ptr<dai::ImgFrame>& frame,int side) {
    if(capture_)capture_<<(side==0?"left":"right")<<','<<ns(frame->getTimestamp())<<','
      <<ns(frame->getTimestampDevice())<<','<<ns(Clock::now())<<','<<frame->getSequenceNum()<<','
      <<frame->getExposureTime().count()<<','<<frame->getSensitivity()<<",0,0,0\n";
    if(!capture_prefix_.empty() && !metadata_saved_[side]) {
      const std::string label=side==0?"left":"right";
      nlohmann::json metadata={{"transformation",frame->getTransformation()},
        {"intrinsics",frame->getTransformation().getIntrinsicMatrix()},
        {"D",frame->getTransformation().getDistortionCoefficients()},
        {"width",frame->getWidth()},{"height",frame->getHeight()},{"stride",frame->getStride()}};
      std::ofstream file(capture_prefix_+"."+label+".json");
      if(!file)throw std::runtime_error("cannot write frame metadata");
      file<<metadata.dump(2)<<'\n';metadata_saved_[side]=true;
    }
    if(!publish_images_)return;
    pending_images_[side].push_back(frame);
    if(pending_images_[side].size()>8)pending_images_[side].pop_front();
    while(!pending_images_[0].empty() && !pending_images_[1].empty()) {
      auto l=pending_images_[0].front(),r=pending_images_[1].front();
      const auto lt=l->getTimestamp(dai::CameraExposureOffset::MIDDLE);
      const auto rt=r->getTimestamp(dai::CameraExposureOffset::MIDDLE);
      if(std::abs(ns(lt)-ns(rt))>500'000) {
        pending_images_[lt<rt?0:1].pop_front();continue;
      }
      const auto capture=lt+(rt-lt)/2;
      pending_images_[0].pop_front();pending_images_[1].pop_front();
      if(Clock::now()-capture>200ms)continue;
      publish_image(l,0,capture);publish_image(r,1,capture);
    }
  }
  void publish_image(const std::shared_ptr<dai::ImgFrame>& frame,int side,Clock::time_point capture) {
    sensor_msgs::msg::Image m;m.header.stamp=stamp(capture);
    m.header.frame_id=side==0?"native_oak_left_optical":"native_oak_right_optical";
    m.width=frame->getWidth();m.height=frame->getHeight();m.step=frame->getStride();
    m.encoding="mono8";
    if(m.step==0)m.step=m.width;
    auto data=frame->getData();
    if(data.size()!=size_t(m.step)*m.height)throw std::runtime_error("unexpected device image layout");
    m.data.assign(data.begin(),data.end()); // One bounded native copy, no Python element conversion.
    sensor_msgs::msg::CameraInfo ci;ci.header=m.header;ci.width=m.width;ci.height=m.height;
    ci.distortion_model="plumb_bob";ci.d.assign(5,0.);ci.r={1,0,0,0,1,0,0,0,1};
    auto transform=frame->getTransformation();auto k=transform.getIntrinsicMatrix();
    for(float d:transform.getDistortionCoefficients())if(std::abs(d)>1e-8)throw std::runtime_error("device output not pinhole");
    for(int r=0;r<3;++r)for(int c=0;c<3;++c){ci.k[r*3+c]=k[r][c];ci.p[r*4+c]=k[r][c];}
    info_pub_[side]->publish(ci);image_pub_[side]->publish(std::move(m));
    if(++frames_[side]%30==0) {
      std_msgs::msg::String report;std::ostringstream s;
      s<<"{\"side\":"<<side<<",\"sequence\":"<<frame->getSequenceNum()<<",\"host_capture_ns\":"<<ns(frame->getTimestamp())
       <<",\"device_capture_ns\":"<<ns(frame->getTimestampDevice())<<",\"exposure_us\":"<<frame->getExposureTime().count()<<",\"iso\":"<<frame->getSensitivity()<<"}";
      report.data=s.str();metadata_->publish(report);
    }
  }
  void poll() {
    if(capture_seconds_>0 && std::chrono::duration<double>(Clock::now()-started_).count()>=capture_seconds_) {
      capture_.flush();timer_->cancel();rclcpp::shutdown();return;
    }
    if(std::abs(now().nanoseconds()-ns(Clock::now())-offset_)>20'000'000)
      throw std::runtime_error("host ROS clock moved relative to capture clock");
    for(auto packet=imu_->tryGet<dai::IMUData>();packet;packet=imu_->tryGet<dai::IMUData>())for(const auto& p:packet->packets) {
      const auto a=p.acceleroMeter;const auto g=p.gyroscope;
      auto publish=[this](auto report,int64_t& last,auto pub,const char* stream){
        const int64_t t=ns(report.getTimestamp());if(t<=last)return;last=t;
        if(capture_)capture_<<stream<<','<<t<<','<<ns(report.getTimestampDevice())<<','<<ns(Clock::now())<<','
          <<report.sequence<<",0,0,"<<report.x<<','<<report.y<<','<<report.z<<'\n';
        if(align_imu_) {
          larp_hardware::Sample sample{t,{report.x,report.y,report.z}};
          if(std::string(stream)=="accel") {
            alignment_.accel(sample);
          } else alignment_.gyro(sample);
        }
        geometry_msgs::msg::Vector3Stamped m;m.header.stamp=stamp(report.getTimestamp());m.header.frame_id="native_oak_imu_raw";
        m.vector.x=report.x;m.vector.y=report.y;m.vector.z=report.z;pub->publish(m);
      };
      publish(a,last_a_,accel_pub_,"accel");publish(g,last_g_,gyro_pub_,"gyro");
    }
    if(align_imu_)while(auto sample=alignment_.pop()) {
      sensor_msgs::msg::Imu m;m.header.stamp=rclcpp::Time(sample->ns+offset_);
      m.header.frame_id="native_oak_imu_raw";m.orientation_covariance[0]=-1;
      m.linear_acceleration.x=sample->acceleration[0];m.linear_acceleration.y=sample->acceleration[1];m.linear_acceleration.z=sample->acceleration[2];
      m.angular_velocity.x=sample->gyro[0];m.angular_velocity.y=sample->gyro[1];m.angular_velocity.z=sample->gyro[2];
      aligned_imu_->publish(m);
    }
    for(int i=0;i<2;++i)if(auto f=images_[i]->tryGet<dai::ImgFrame>())image(f,i);
    if(rect_[0])rectified();
  }
  // Pair the stereo engine's rectified outputs by sequence number; one stamp per pair.
  void rectified() {
    for(int i=0;i<2;++i)while(auto f=rect_[i]->tryGet<dai::ImgFrame>()) {
      pending_rect_[i].push_back(f);
      if(pending_rect_[i].size()>8)pending_rect_[i].pop_front();
    }
    while(!pending_rect_[0].empty() && !pending_rect_[1].empty()) {
      auto l=pending_rect_[0].front(),r=pending_rect_[1].front();
      if(l->getSequenceNum()!=r->getSequenceNum()) {
        pending_rect_[l->getSequenceNum()<r->getSequenceNum()?0:1].pop_front();continue;
      }
      pending_rect_[0].pop_front();pending_rect_[1].pop_front();
      const auto lt=l->getTimestamp(dai::CameraExposureOffset::MIDDLE),rt=r->getTimestamp(dai::CameraExposureOffset::MIDDLE);
      const auto capture=lt+(rt-lt)/2;
      if(Clock::now()-capture>200ms)continue;
      for(int i=0;i<2;++i) {
        auto& f=i==0?l:r;
        sensor_msgs::msg::Image m;m.header.stamp=stamp(capture);
        m.header.frame_id=i==0?"native_oak_depth_optical":"native_oak_right_rectified_optical";
        m.width=f->getWidth();m.height=f->getHeight();m.encoding="mono8";
        m.step=f->getStride()?f->getStride():m.width;
        auto data=f->getData();
        if(data.size()<size_t(m.step)*m.height)throw std::runtime_error("unexpected rectified layout");
        m.data.assign(data.begin(),data.begin()+size_t(m.step)*m.height);
        if(!rect_info_[i]) {  // rectified: no distortion, shared K, right P carries -fx*baseline
          rect_info_[i]=std::make_shared<sensor_msgs::msg::CameraInfo>();
          auto& ci=*rect_info_[i];ci.width=m.width;ci.height=m.height;ci.distortion_model="plumb_bob";ci.d.assign(5,0.);
          const auto k=f->getTransformation().getIntrinsicMatrix();
          ci.r={1,0,0,0,1,0,0,0,1};
          for(int a=0;a<3;++a)for(int b=0;b<3;++b){ci.k[a*3+b]=k[a][b];ci.p[a*4+b]=k[a][b];}
          if(i==1)ci.p[3]=-ci.k[0]*baseline_m_;
          if(i==0) {  // geometry for the rig (T_body_depth): frame, size, K, P, left rectification R
            nlohmann::json j={{"frame",m.header.frame_id},{"width",ci.width},{"height",ci.height},
              {"k",std::vector<double>(ci.k.begin(),ci.k.end())},{"p",std::vector<double>(ci.p.begin(),ci.p.end())},
              {"r",std::vector<double>(rect_r_.begin(),rect_r_.end())},{"baseline_m",baseline_m_},
              {"aligned_to","rectified left (CAM_B); r maps left optical into this frame"}};
            std::ofstream(rect_export_)<<j.dump(2)<<'\n';
          }
          RCLCPP_INFO(get_logger(),"Rectified %s %ux%u fx=%.2f cx=%.2f cy=%.2f",i==0?"left":"right",
                      ci.width,ci.height,ci.k[0],ci.k[2],ci.k[5]);
        }
        rect_info_[i]->header=m.header;
        rect_info_pub_[i]->publish(*rect_info_[i]);rect_pub_[i]->publish(std::move(m));
      }
    }
  }
  bool align_imu_=false,publish_rectified_=true;
  std::shared_ptr<dai::MessageQueue> rect_[2];
  std::deque<std::shared_ptr<dai::ImgFrame>> pending_rect_[2];
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_pub_[2];
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rect_info_pub_[2];
  sensor_msgs::msg::CameraInfo::SharedPtr rect_info_[2];
  std::array<double,9> rect_r_{1,0,0,0,1,0,0,0,1};double baseline_m_=0.;std::string rect_export_;
  larp_hardware::ImuAlignment alignment_{10'000'000};
  std::deque<std::shared_ptr<dai::ImgFrame>> pending_images_[2];
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr aligned_imu_;
  bool publish_images_=false,metadata_saved_[2]={false,false};std::string capture_prefix_;
  std::ofstream capture_;double capture_seconds_=0.;Clock::time_point started_;
  int64_t offset_=0,last_a_=0,last_g_=0;uint64_t frames_[2]={0,0};
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> images_[2],imu_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_[2];
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_[2];
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_pub_,gyro_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<NativeOak>());}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}rclcpp::shutdown();}
