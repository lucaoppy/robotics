#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

int main(int _argc, char **_argv){
	#if GAZEBO_MAJOR_VERSION < 6
 	gazebo::setupClient(_argc, _argv);
	#else
  	gazebo::client::setup(_argc, _argv);
	#endif

  	gazebo::transport::NodePtr node(new gazebo::transport::Node());
  	node->Init();

  	gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::Vector3d>("~/Kobra/cmd_velocity");

  	pub->WaitForConnection();

  	// Create a a vector3 message
  	gazebo::msgs::Vector3d msg;

	// Put the linear speed in the x-component, the angular speed in the y-component
	#if GAZEBO_MAJOR_VERSION < 6
  	gazebo::msgs::Set(&msg, gazebo::math::Vector3(std::atof(_argv[1]), std::atof(_argv[2]), 0));
	#else
  	gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(_argv[1]), std::atof(_argv[2]), 0));
	#endif

  	pub->Publish(msg);

	#if GAZEBO_MAJOR_VERSION < 6
  	gazebo::shutdown();
	#else
  	gazebo::client::shutdown();
	#endif
}