#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/rendering/rendering.hh>
#include <opencv2/opencv.hpp>
#include <gazebo/physics/Link.hh>
#include <gazebo/gui/GuiIface.hh>
#include <mutex>
#include <string>
#include <ignition/math/Box.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
	class Controller : public ModelPlugin
	{
		
 		private: rendering::CameraPtr camera;
		private: physics::ModelPtr model;
		private: event::ConnectionPtr newFrameConnection;
		private: event::ConnectionPtr updateConnection;
		private: double vel,positive,negative,ratio;
		public: physics::JointPtr Left1;
		public: physics::JointPtr Right1;
		public: physics::JointPtr Left2;
		public: physics::JointPtr Right2;
		private: double wheelSpeed[2];
		private: double value[6];
		private: std::mutex mutex;
		private: bool up;
		public: Controller() 
		{
			this->wheelSpeed[0] = this->wheelSpeed[1] = 0;
			this->up = false;
			this->vel = 0.3;
			ratio = 10;
			positive = 50;
			negative = 70;
		}

		public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			if (_model == NULL || _sdf == NULL)
			{
				gzerr << "Failed to load Plugin. NULL model or sdf" << std::endl;
				return;
			}

			this->model = _model;

			if (!this->FindSensor(this->model))
			{
				gzerr << "sensor not found!" << std::endl;
				return;
			}
			
			auto joints = this->model->GetJoints();
			for (const auto &j : joints)
			{
				if(j->GetName() == "left_back_hinge")
					this->Left1 = j;
				else if(j->GetName() == "right_back_hinge")
					this->Right1 = j;
				else if(j->GetName() == "right_front_hinge")
					this->Right2 = j;
				else if(j->GetName() == "left_front_hinge")
					this->Left2 = j;
			}

			if(!this->Left1 || !this->Right1 || !this->Left2 || !this->Right2)
			{
				gzerr << "left or right joint not found!" << std::endl;
				return;
			}
			
			if (_sdf->HasElement("velocity"))
  			{
    			this->vel = _sdf->Get<double>("velocity");
  			}
  			if (_sdf->HasElement("positive"))
  			{
    			this->positive = _sdf->Get<double>("positive");
  			}
  			if (_sdf->HasElement("negative"))
  			{
    			this->negative = _sdf->Get<double>("negative");
  			}
  			if (_sdf->HasElement("ratio"))
  			{
    			this->ratio = _sdf->Get<double>("ratio");
  			}

			this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Controller::OnUpdate, this));
		}

		public: bool FindSensor(const physics::ModelPtr &_model)
		{
			for (const auto l : _model->GetLinks())
			{
				for (unsigned int i = 0; i < l->GetSensorCount(); ++i)
				{
					std::string sensorName = l->GetSensorName(i);
					sensors::SensorPtr sensor = sensors::get_sensor(sensorName);
					if (!sensor) continue;
					
					if (sensor->Type() == "camera" || sensor->Type() == "Camera")
					{
						sensors::CameraSensorPtr cameraSensor =	std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
						if (cameraSensor)
						{
							rendering::CameraPtr cam = cameraSensor->Camera();
							if (cam)
							{
								this->camera = cam;
								this->newFrameConnection = this->camera->ConnectNewImageFrame(
								std::bind(&Controller::OnNewFrame, this,std::placeholders::_1, std::placeholders::_2,
								std::placeholders::_3, std::placeholders::_4,
								std::placeholders::_5));
								return true;
							}
						}
					}
				}
			}

			for (const auto &m : _model->NestedModels())
			{
				if (this->FindSensor(m))
				return true;
			}

			return false;
		}

		public: void OnUpdate()
		{
			std::lock_guard<std::mutex> lock(this->mutex);
			this->updateFollower();
		}

		public: void OnNewFrame(const unsigned char *_image,const unsigned int _width, const unsigned int _height, const unsigned int _depth, const std::string &_format)
		{
			std::lock_guard<std::mutex> lock(this->mutex);
			unsigned char *img = (unsigned char*)_image;
			cv::Mat image(_height,_width,CV_8UC3,img);

			if(!image.empty()) 
			{	
				cv::Mat patch;
				for (int i = 0; i < 5; ++i)
				{
					cv::Range y(75,165);
					cv::Range x(i*64,(i+1)*64);
					patch = image(y,x);
					cv::cvtColor(patch, patch, CV_BGR2GRAY);
					this->value[i] = cv::sum(patch)[0]/(patch.rows*patch.cols);
					this->value[i]-=65;
				}
				patch = image(cv::Range(0,75),cv::Range(64,256));
				this->value[5] = cv::sum(patch)[0]/(patch.rows*patch.cols);
				this->value[5] -=65;
			}
			this->up = true;
		}

		public: void updateFollower()
		{
			if(!this->up)
				return;
			this->up = false;
			wheelSpeed[0] = wheelSpeed[1]= 0;
			if(value[0]>40) {
				wheelSpeed[0] = -negative*vel;
				wheelSpeed[1] = positive*vel;
			}
			else if(value[5] > 17) {
				wheelSpeed[1] = vel;
				wheelSpeed[0] = vel;
			}
			else if(value[4]>40) {
				wheelSpeed[1] = -negative*vel;
				wheelSpeed[0] = positive*vel;
			}
			else if(value[1]>20) {
				wheelSpeed[1] = ratio*vel;
				wheelSpeed[0] = -ratio*vel;
			}
			else if(value[3]>20) {
				wheelSpeed[1] = -ratio*vel;
				wheelSpeed[0] = ratio*vel;
			}
			else {
				wheelSpeed[0] = -negative*vel/10;
				wheelSpeed[1] = positive*vel/10;
			}
			this->Left1->SetVelocity(0,wheelSpeed[0]);
			this->Right1->SetVelocity(0,wheelSpeed[1]);
			this->Left2->SetVelocity(0,wheelSpeed[0]);
			this->Right2->SetVelocity(0,wheelSpeed[1]);
		}
	};

	GZ_REGISTER_MODEL_PLUGIN(Controller)
}
