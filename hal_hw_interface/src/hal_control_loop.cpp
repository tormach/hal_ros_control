#include <hal_hw_interface/hal_control_loop.h>
#include <hal_hw_interface/hal_ros_logging.h>

#include <cstdlib> // getenv()

// Pre-declare the HAL function
extern "C" void funct(void *arg, long period);

namespace hal_hw_interface
{

HalRosControlLoop::HalRosControlLoop(ros::NodeHandle& nh)
: nh_(nh)
{
  // Run ROS loop in a separate thread
  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::string ros_master_uri = getenv( "ROS_MASTER_URI" );

  HAL_ROS_LOG_INFO(
    CNAME, "%s: Initialized ROS node and spinner; ROS_MASTER_URI = %s",
    ros_master_uri.c_str(), CNAME);

  // HW interface
  hardware_interface_.reset(
    new hal_hw_interface::HalHWInterface(nh_));

  HAL_ROS_LOG_INFO(
    CNAME, "%s: Initialized HAL hardware interface", CNAME);

  // ROS callback thread
  nh_.setCallbackQueue(&non_rt_ros_queue_);
  non_rt_ros_queue_thread_ =
    boost::thread(
      boost::bind(&HalRosControlLoop::serviceNonRtRosQueue, this));

  HAL_ROS_LOG_INFO(
    CNAME, "%s: Done initializing ROS callback thread", CNAME);

  // Controller
  controller_manager_.reset(
    new controller_manager::ControllerManager(
      hardware_interface_.get(), nh_));

  HAL_ROS_LOG_INFO(
    CNAME, "%s: Done initializing ROS controller manager", CNAME);

  // Init HAL hardware interface
  hardware_interface_->init(&funct);

  HAL_ROS_LOG_INFO(
    CNAME, "%s: Done initializing HAL hardware interface", CNAME);

  HAL_ROS_LOG_INFO(
    CNAME, "HAL control loop ready.");
} // constructor

// Non-RT thread CB function
void HalRosControlLoop::serviceNonRtRosQueue()
{
  static const double timeout = 0.001;

  while (this->nh_.ok())
    this->non_rt_ros_queue_.callAvailable(ros::WallDuration(timeout));
}

void HalRosControlLoop::shutdown()
{
  // Shut down HAL, if possible
  hardware_interface_->shutdown();

  // Shut down ROS node
  nh_.shutdown();
  non_rt_ros_queue_thread_.join();
  ros::shutdown();
}

HalRosControlLoop::~HalRosControlLoop()
{
  shutdown();
}

// This and HAL basically replicate generic_hw_control_loop.cpp
void HalRosControlLoop::update(long period)
{
  ros::Duration ros_period (period / 1000000000ull, period % 1000000000ull);

  hardware_interface_->read(ros_period);
  controller_manager_->update(ros::Time::now(), ros_period);
  hardware_interface_->write(ros_period);
}

} // namespace


// The HAL hardware interface control loop object
boost::shared_ptr<hal_hw_interface::HalRosControlLoop> control_loop_;

extern "C" {
  int rtapi_app_main(void)
  {
    // Init ROS node
    // - Pass ROS_MASTER_URI (none causes a sigsegv on ros::init())
    // - Delegate signal handling to rtapi_app
    const ros::M_string remappings = {
      {"__master", getenv( "ROS_MASTER_URI" )},
    };
    ros::init(remappings, CNAME, ros::init_options::NoSigintHandler);

    // ROS node handle
    ros::NodeHandle nh_;    

    // Create HAL controller and hardware interface
    control_loop_.reset(
      new hal_hw_interface::HalRosControlLoop(nh_));

    return 0;
  }

  void funct(void *arg, long period)
  {
    control_loop_->update(period);
  }

  void rtapi_app_exit(void)
  {
    control_loop_->shutdown();
    // hw_interface_ smart pointer doesn't need to be deleted
  }

} // extern "C"
