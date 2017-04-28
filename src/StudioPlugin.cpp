#include "StudioPlugin.hpp"
#include <QtPlugin>
#include <QString>
#include <vector>
#include <utility>
#include <string>

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
	
	_subscriberState = _nh->subscribe("/rovi2/robot_node/Robot_state", 1, &StudioPlugin::updateCallback, this);
        _subscriberBall = _nh->subscribe("/ball_locator_3d/pos_triangulated" , 1, &StudioPlugin::ballCallback, this);

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
	ROS_INFO("Test");

}



void StudioPlugin::close()
{
	_wc = NULL;

}

void StudioPlugin::stateChangedListener(const rw::kinematics::State& state) {
	_state = state;
}




Q_EXPORT_PLUGIN(StudioPlugin);
