#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <string>
#include <chrono>
#include <cstdlib>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>


namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
	
	physics::JointPtr FL;
	physics::JointPtr FR;
	physics::JointPtr RL;
	physics::JointPtr RR;
	physics::JointPtr pole;
	  
	physics::LinkPtr verticalCamera;
	physics::LinkPtr lateralCamera;
	  
	math::Pose vertCamPose = gazebo::math::Pose(0, 0.24, 0.78, 0,  0, 0);
	math::Pose horCamPose = gazebo::math::Pose(0, 0.2, 0.78, 0,  0, 0);
	
	common::PID pid;
	  
    transport::NodePtr node;
    transport::SubscriberPtr velSub;
	transport::SubscriberPtr intSub;
	transport::SubscriberPtr cameraSub;
    
  	double linVelToSet = 0;
  	double angVelToSet = 0;
    
	double L = 0.495;
  	double R = 0.0725;
	  
	double velFL;
	double velFR;
	double velRL;
	double velRR;
	  
	double posX = 0;
	double posY = 0;
  	double theta = 0;
	double previousTheta = 0;
	
	int integrationType = 0;
	std::string integration = "Euler";
	  
	std::chrono::high_resolution_clock::time_point finish;
	  
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/){
      	this->model = _parent;
    	this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelPush::OnUpdate, this, _1));
		
		FL = this->model->GetJoint("frontlefthinge");
    	FR = this->model->GetJoint("frontrighthinge");
		RL = this->model->GetJoint("rearlefthinge");
    	RR = this->model->GetJoint("rearrighthinge");
		pole = this->model->GetJoint("verticalarmhinge");
      
		this->pid = common::PID(0.1, 0, 0);
      
    	this->model->GetJointController()->SetVelocityPID(this->FL->GetScopedName(), this->pid);
    	this->model->GetJointController()->SetVelocityPID(this->FR->GetScopedName(), this->pid);
    	this->model->GetJointController()->SetVelocityPID(this->RL->GetScopedName(), this->pid);
    	this->model->GetJointController()->SetVelocityPID(this->RR->GetScopedName(), this->pid);
		
		verticalCamera = this->model->GetLink("verticalcamera");
		lateralCamera = this->model->GetLink("lateralcamera");
    
    	this->node = transport::NodePtr(new transport::Node());
    	this->node->Init(this->model->GetWorld()->GetName());
    
    	std::string velocityTopic = "~/Kobra/cmd_velocity";
    	this->velSub = this->node->Subscribe(velocityTopic, &ModelPush::VelMsg, this);

    	std::string integrationTopic = "~/Kobra/odometry";
    	this->intSub = this->node->Subscribe(integrationTopic, &ModelPush::IntMsg, this);
		
    	std::string cameraTopic = "~/Kobra/ptz";
    	this->cameraSub = this->node->Subscribe(cameraTopic, &ModelPush::MoveCamera, this);
      
		long long int posX = 0;
		long long int posY = 0;
		
		finish = std::chrono::high_resolution_clock::now();
    }

    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
		
		if(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-finish).count() > 100000000)
		{
			this->verticalCamera->SetRelativePose(vertCamPose);
			this->lateralCamera->SetRelativePose(horCamPose);
		
			auto start = std::chrono::high_resolution_clock::now();
			velFL = ((int)(this->FL->GetVelocity(0)*1000)/1000.0);
			velFR = ((int)(this->FR->GetVelocity(0)*1000)/1000.0);
			velRL = ((int)(this->RL->GetVelocity(0)*1000)/1000.0);
			velRR = ((int)(this->RR->GetVelocity(0)*1000)/1000.0);
			
      		long long int measureTime = std::chrono::duration_cast<std::chrono::nanoseconds>(start-finish).count();
			double rightSpdAvg = (velFR + velRR)/2;
			double leftSpdAvg = (velFL + velRL)/2;
			double avgCentralSpd = (rightSpdAvg + leftSpdAvg)/2;
			double angularSpd = (rightSpdAvg - leftSpdAvg)/L;
			theta += angularSpd*measureTime*0.000000001;
			//theta = this->model->GetLink("chassis")->GetWorldPose().rot->GetYaw();
			//theta = pole->GetAngle(2).Radian();
			switch (integrationType){
				case 1:{
					posX += avgCentralSpd*sin(theta + angularSpd*measureTime*0.000000001*0.5)*measureTime*0.000000001;
					posY += avgCentralSpd*cos(theta + angularSpd*measureTime*0.000000001*0.5)*measureTime*0.000000001;
					integration = "Runge-Kutta";
					break;
				}
				case 2:{
					if(angularSpd != 0){
						posX += (avgCentralSpd/angularSpd)*(cos(theta)-cos(previousTheta));
						posY += (avgCentralSpd/angularSpd)*(sin(theta)-sin(previousTheta));
						integration = "Exact";
						break;
					}
				}
				default:{
					posX  += avgCentralSpd*measureTime*0.000000001*sin(theta);
					posY  += avgCentralSpd*measureTime*0.000000001*cos(theta);
					integration = "Euler";
				}
			}
			previousTheta = theta;
			
			std::system("clear");
			printf(" Measurement lasted %lli nanoseconds",measureTime);
			printf("\n\n Linear Speed: %f\n",avgCentralSpd);
			printf("\n          /^\\\n %f<--->%f\n           |\n           |\n %f<--->%f\n",velFL,velFR,velRL,velRR);
			printf("\n Angular Speed: %f, Theta = %f\n",angularSpd,theta);
			printf("\n          /^\\\n %f<--->%f\n           |\n           |\n %f<--->%f\n",velFL/R,velFR/R,velRL/R,velRR/R);
			std::printf("\n Integration Type: %s", integration.c_str());
			printf("\n Displacement:\n\tX axis: %f\n\tY axis: %f\n",posX,posY);
			
			//double relativeSpeed = this->model->GetRelativeLinearVel().y;
			//math::Vector3 worldSpeed = this->model->GetWorldLinearVel();
			//double velX = worldSpeed.x;
			//double velY = worldSpeed.y;
			//printf("\n absolute:%f relative:%f", sqrt(pow(velX, 2) + pow(velY, 2)), relativeSpeed);
			finish = std::chrono::high_resolution_clock::now();
		}
    }
	
	
	public: void SetVelocity(const double &_linVel, const double &_angVel){
		double right = (2*_linVel + _angVel*L)/2;
		double left  = (2*_linVel - _angVel*L)/2;
		this->model->GetJointController()->SetVelocityTarget(this->FL->GetScopedName(), left);
		this->model->GetJointController()->SetVelocityTarget(this->FR->GetScopedName(), right);
		this->model->GetJointController()->SetVelocityTarget(this->RL->GetScopedName(), left);
		this->model->GetJointController()->SetVelocityTarget(this->RR->GetScopedName(), right);
    }
	
	private: void VelMsg(ConstVector3dPtr &_msg){
      	this->SetVelocity(_msg->x(), _msg->y());//fake 3D vector, x is linear, y is angular
    }
	  
	private: void IntMsg(ConstVector3dPtr &_msg){
      	this->integrationType=_msg->x();//fake 3D vector, x is integration type
    }

	public: void MoveCamera(ConstVector3dPtr &_msg){//fake 3D vector, x is tilt, y is pan
	  	vertCamPose = gazebo::math::Pose(0, 0.24, 0.78, _msg->x()*3.14/180,  0, 0);
		horCamPose = gazebo::math::Pose(0, 0.2, 0.78, 0, 0, _msg->y()*3.14/180);
	}
	
    private: physics::ModelPtr model;

    private: event::ConnectionPtr updateConnection;
	  
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}