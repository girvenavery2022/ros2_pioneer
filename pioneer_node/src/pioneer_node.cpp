#include <memory>

#include "pioneer_node/pioneer_node.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace Pioneer
{
Pioneer::Pioneer(rclcpp::NodeOptions options)
: Node("Pioneer_node", options)
{
  subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 1, std::bind(&Pioneer::cmdvel_callback, this, std::placeholders::_1));
  
  voltage_pub = this->create_publisher<std_msgs::msg::Float64>("/battery_voltage", 10);
  sonar_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sonar", 10);
  motors_state_pub = this->create_publisher<std_msgs::msg::Bool>("/motor_state", 10);
  bumper_pub = this->create_publisher<rosaria_msgs::msg::BumperState>("/bumpers", 10);

  motor_service = this->create_service<std_srvs::srv::SetBool>("trigger_motors", 
    std::bind(&Pioneer::trigger_motors, this, std::placeholders::_1, std::placeholders::_2));

  battery_voltage_publish_timer = this->create_wall_timer(
    500ms, std::bind(&Pioneer::battery_voltage_publisher, this));

  bumper_publish_timer = this->create_wall_timer(
    100ms, std::bind(&Pioneer::bumper_publisher, this));

  sonar_publish_timer = this->create_wall_timer(
    50ms, std::bind(&Pioneer::sonar_publisher, this));

  motors_state_publish_timer = this->create_wall_timer(
    100ms, std::bind(&Pioneer::motor_state_publisher, this));
}

void Pioneer::connect()
{
  robot = new ArRobot();
  ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
  ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
  argparser->loadDefaultArguments(); 
  args->add("-robotPort %s", "/dev/ttyUSB0"); // pass robot's serial port to Aria
  args->add("-robotBaud %d", 9600);

  // Connect to the robot, get some initial data from it such as type and name,
  // and then load parameter files for this robot.
  robotConnector= new ArRobotConnector(argparser, robot); // warning never freed

  if(!robotConnector->connectRobot())
  {
    ArLog::log(ArLog::Terse, "simpleMotionCommands: Could not connect to the robot.");
    if(argparser->checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
    }
  }
  if (!Aria::parseArgs())
  {
    Aria::logOptions();
    Aria::exit(1);
  }
  setup();
}

void Pioneer::setup()
{
  // Enable the motors
  robot->enableMotors();

  // disable sonars on startup
  robot->disableSonar();

  // Initialize bumpers with robot number of bumpers
  bumpers.front_bumpers.resize(robot->getNumFrontBumpers());
  bumpers.rear_bumpers.resize(robot->getNumRearBumpers());

  // Run ArRobot background processing thread
  robot->runAsync(true);
}

void Pioneer::battery_voltage_publisher()
{
  std_msgs::msg::Float64 batteryVoltage;
  batteryVoltage.data = robot->getRealBatteryVoltageNow();
  voltage_pub->publish(batteryVoltage);
}

void Pioneer::sonar_publisher()
{
  robot->enableSonar();
  // Publish sonar information, if enabled.
  if (publish_sonar)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::msg::PointCloud2 sonar_cloud;	//sonar readings.
    std::vector<geometry_msgs::msg::Point32> Points;
  
    for (int i = 0; i < robot->getNumSonar(); i++) {
      ArSensorReading* reading = NULL;
      reading = robot->getSonarReading(i);
      if(!reading) {
        continue;
      }
    
      //add sonar readings (robot-local coordinate frame) to cloud
      pcl::PointXYZ p;
      p.x = reading->getX() / 1000.0;
      p.y = reading->getY() / 1000.0;
      p.z = 0.0;
      cloud.points.push_back(p);
    }
    pcl::toROSMsg(cloud, sonar_cloud);
    sonar_cloud.header.stamp = this->get_clock()->now();
    sonar_cloud.header.frame_id = "sonar_link";
    sonar_pub->publish(sonar_cloud);
  }
}

void Pioneer::odometry_publisher()
{

}

void Pioneer::bumper_publisher()
{
  // getStallValue returns 2 bytes with stall bit and bumper bits, packed as (00 00 FrontBumpers RearBumpers)
  int stall = robot->getStallValue();
  unsigned char front_bumpers = (unsigned char)(stall >> 8);
  unsigned char rear_bumpers = (unsigned char)(stall);

  bumpers.header.frame_id = "bumper_link";

  std::stringstream bumper_info(std::stringstream::out);
  // Bit 0 is for stall, next bits are for bumpers (leftmost is LSB)
  for (unsigned int i=0; i<robot->getNumFrontBumpers(); i++)
  {
    bumpers.front_bumpers[i] = (front_bumpers & (1 << (i+1))) == 0 ? 0 : 1;
  }
  // Rear bumpers have reverse order (rightmost is LSB)
  unsigned int numRearBumpers = robot->getNumRearBumpers();
  for (unsigned int i=0; i<numRearBumpers; i++)
  {
    bumpers.rear_bumpers[i] = (rear_bumpers & (1 << (numRearBumpers-i))) == 0 ? 0 : 1;
  }
  bumpers.header.stamp = this->get_clock()->now();
  bumper_pub->publish(bumpers);
}

void Pioneer::motor_state_publisher()
{
  // publish motors state if changed
  std_msgs::msg::Bool motors_state;
  bool state = robot->areMotorsEnabled();
  motors_state.data = state;
  motors_state_pub->publish(motors_state);
}

void Pioneer::trigger_motors(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  robot->lock();
  if(request->data)
  {
    if(robot->isEStopPressed())
    {
      response->success = false;
      response->message = "Robots is EStopped, can't enable motors";
    }
    else
      robot->enableMotors();
    
    if(robot->areMotorsEnabled())
    {
      response->success = true;
      response->message = "Robots Motors are Enabled!";
    }
  }
  if(!request->data)
  {
    robot->disableMotors();
    if(!robot->areMotorsEnabled())
    {
      response->success = true;
      response->message = "Robot Motors are disabled!";
    }
    else
    {
      response->success = false;
      response->message = "Robot Motors were not Disabled!";
    }
  }
  robot->unlock();
}

void Pioneer::cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if(robot->areMotorsEnabled())
  {
    robot->lock();
    robot->enableMotors();
    robot->setVel(msg->linear.x*1e3);
    std::cout << robot->getVel() << std::endl;
    if(robot->hasLatVel())
      robot->setLatVel(msg->linear.y*1e3);
    robot->setRotVel(msg->angular.z*180/M_PI);
    robot->unlock();
  }
  else
    RCLCPP_WARN(this->get_logger(), "Motors are not Enabled. Enable them and try again");
}
} // namespace pioneer

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto lp_node = std::make_shared<Pioneer::Pioneer>(options);
  exec.add_node(lp_node);
  lp_node->connect();
  exec.spin();
  rclcpp::shutdown();
  return 0;
}