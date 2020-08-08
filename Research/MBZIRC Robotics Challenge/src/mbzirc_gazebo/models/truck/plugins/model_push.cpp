#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public:

	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
      // Store the pointer to the model
      this->model = _parent;


      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelPush::OnUpdate, this, _1));

	this->world = this->model->GetWorld();
	this->time = this->world->GetSimTime();
	setVehicle(true);

  }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {

	this->world = this->model->GetWorld();
	this->time = this->world->GetSimTime();
	setVehicle(false);



    }

    // sets the vehicle position and velocity
    public: void setVehicle(bool first)
    {
		//variables of motion
	const double rad2=1.4142135623730950488;
	const double pi=3.14159265358979323846;
	const double R=16.5;
	const double V=4.1;
	const double ts=(R*2.0)/V;
	const double ti=3.0*pi*R/(2.0*V);
	const double td=V/R;

	const double t1=ts;
	const double t2=ts+ti;
	const double t3=2.0*ts+ti;
	const double tRev=2.0*(ts+ti);

	double t, x, y, yaw, xd, yd, yawd, toff;
	const double z = 0;
	const double roll = 0;
	const double pitch = 0;
	const double zd = 0;
	const double rolld = 0;
	const double pitchd = 0;

	double inside1, inside2;

	t = fmod(time.Double(), tRev);

	if(t<t1){
		y=(R-V*t)/rad2;
		x=y;
		yaw=5.0*pi/4.0;
		xd=-V/rad2;
		yd=xd;
		yawd=0.0;
	}else if (t<t2){
		toff=t-t1;
		inside1=-td*toff-pi/4.0;
		double ci=cos(inside1);
		y=R*(ci-rad2);
		x=R*sin(inside1);
		yaw=pi-inside1;
		xd=y*td;
		yd=-R*ci*td;
		yawd=-td;
	}else if(t<t3){
		toff=t-t2;
		y=(V*toff-R)/rad2;
		x=-y;
		yaw=3.0*pi/4.0;
		xd=V/rad2;
		yd=-xd;
		yawd=0.0;
	}else{
		toff=t-t3;
		inside2=td*toff-3.0*pi/4.0;
		double ci=cos(inside2);
		y=R*(ci+rad2);
		x=R*sin(inside2);
		yaw=-inside2;
		xd=-y*td;
		yd=R*ci*td;
		yawd=td;
	}


	math::Vector3 vec(x, y, z);
	math::Quaternion quat(roll, pitch, yaw);

	gazebo::math::Pose pose(vec, quat);

	this->model->SetLinearVel(math::Vector3(xd, yd, zd));
	this->model->SetAngularVel(math::Vector3(rolld, pitchd, yawd*1.683));
//there shouldn't be a factor here, but it doesn't do correct angular vel if it isn't there

	//if(first){//Uncomment this if statement to do pure velocities (no position correction)
	this->model->SetWorldPose(pose);
	//}
    }

    // Pointer to the model
	private:
		physics::ModelPtr model;
		physics::WorldPtr world;

    // Pointer to the update event connection
		event::ConnectionPtr updateConnection;
		common::Time time;


  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
