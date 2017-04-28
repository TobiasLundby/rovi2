#include "StudioPlugin.hpp"
#include <QtPlugin>
#include <QString>
#include <vector>
#include <utility>
#include <string>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>

StudioPlugin::StudioPlugin() : RobWorkStudioPlugin("StudioPluginUI", QIcon("icon.png"))
{
	setupUi(this);

        connect(_btnROS    ,SIGNAL(pressed()), this, SLOT(openROS()) );

        _rosTimer = new QTimer(this);

	connect(_rosTimer, SIGNAL(timeout()), this, SLOT(checkROS()));
}

StudioPlugin::~StudioPlugin()
{
}

void StudioPlugin::initialize()
{
	_rws = getRobWorkStudio();
	_rws->stateChangedEvent().add(boost::bind(&StudioPlugin::stateChangedListener, this, _1), this);

        std::string path = ros::package::getPath("rovi2") + "/WorkStation_3/WC3_Scene.wc.xml";
	rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(path);
	_rws->setWorkCell(wc);



	
}

void StudioPlugin::open(rw::models::WorkCell *workcell)
{
	_wc = workcell;

	_device = _wc->findDevice("UR1");

	_state =  _wc->getDefaultState();

        _BallFrame = (rw::kinematics::MovableFrame*) _wc->findFrame("Ball");




}

void StudioPlugin::openROS()
{

	QString rosmaster = _rosmaster->toPlainText();
	QString rosip = _rosip->toPlainText();
	_rosmaster->setEnabled(false);
	_rosip->setEnabled(false);
	std::vector<std::pair<std::string, std::string> > args(0);
        args.push_back(std::make_pair<std::string, std::string> ("__name", "StudioPlugin"));
        args.push_back(std::make_pair<std::string, std::string> ("__master",rosmaster.toStdString()));
	args.push_back(std::make_pair<std::string, std::string> ("__ns","rovi2"));

        args.push_back(std::make_pair<std::string, std::string> ("__ip", rosip.toStdString()));

        ros::init(args, "StudioPlugin");
        _nh = new ros::NodeHandle;

        _rosTimer->start(20);
        _btnROS->setEnabled(false);
	
	if(_crobot->isChecked())
		_subscriberState = _nh->subscribe("/rovi2/robot_node/Robot_state", 1, &StudioPlugin::updateCallback, this);

	if(_cball->isChecked())
        	_subscriberBall = _nh->subscribe("/ball_locator_3d/pos_triangulated" , 1, &StudioPlugin::ballCallback, this);

	_crobot->setEnabled(false);
	_cball->setEnabled(false);

	//ros::spin();


}

void StudioPlugin::checkROS()
{
	ros::spinOnce();

}

void StudioPlugin::updateCallback(const rovi2::State &state)
{
	_device->setQ(toRw(state.q),_state);
	_rws->setState(_state);
}

void StudioPlugin::ballCallback(const rovi2::position3D &position)
{
	rw::math::Vector3D<double> transP(-0.116075, -1.67057, 1.33754);
	rw::math::Rotation3D<double> transR(rw::math::RPY<double>(-0.0375525, -0.0132076, -2.0942).toRotation3D());
	rw::math::Transform3D<double> trans(transP, transR);
	rw::math::Vector3D<double> newPos(position.x, position.y, position.z);
	newPos = trans*newPos;
	rw::math::Transform3D<double> newTrans(newPos);
	//newTrans = trans*newTrans;
	_BallFrame->setTransform(newTrans, _state);
	getRobWorkStudio()->setState(_state);

}



void StudioPlugin::close()
{
	_wc = NULL;

}

void StudioPlugin::stateChangedListener(const rw::kinematics::State& state) {
	_state = state;
}




Q_EXPORT_PLUGIN(StudioPlugin);
