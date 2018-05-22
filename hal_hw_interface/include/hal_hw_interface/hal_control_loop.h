#ifndef HAL_HW_INTERFACE_HAL_CONTROL_LOOP_H
#define HAL_HW_INTERFACE_HAL_CONTROL_LOOP_H

#include <hal_hw_interface/hal_hw_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>

namespace hal_hw_interface
{

  class HalRosControlLoop
  {
  public:

    /**
     * \brief Constructor
     * \param nh - Node handle for topics.
     * \param hardware_interface - Hardware interface object
     */
    HalRosControlLoop(ros::NodeHandle& nh);

    /**
     * \brief Destructor
     */
    ~HalRosControlLoop();

    /**
     * \brief HAL RT component function that runs one ros_control
     * read()/update()/write() cycle; this is wrapped in funct() for
     * registering the C-linkable HAL component function callback at
     * HAL component creation
     */
    void update(long period);

    /**
     * \brief Shut down the robot hardware interface & controller
     */
    void shutdown();

  
  protected:

    /**
     * \brief The ROS node handle
     */
    ros::NodeHandle nh_;

    /////// Non-RT CB thread pieces ///////
    /**
     * \brief The non-RT ROS thread callback function
     */
    void serviceNonRtRosQueue();

    // - Non-RT thread and callback queue
    boost::thread non_rt_ros_queue_thread_;
    ros::CallbackQueue non_rt_ros_queue_;

    /////// RT pieces ///////
    
    /**
     * \brief The HAL hardware interface
     */
    boost::shared_ptr<hal_hw_interface::HalHWInterface>
      hardware_interface_;

    /**
     * \brief The ros_control controller_manager
     */
    boost::shared_ptr<controller_manager::ControllerManager>
      controller_manager_;

  }; // class


} // namespace

#endif // HAL_HW_INTERFACE_HAL_CONTROL_LOOP_H
