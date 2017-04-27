#include "StudioPlugin.hpp"
#include <QtPlugin>
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




}

void StudioPlugin::openROS()
{

	std::vector<std::pair<std::string, std::string> > args(0);
        args.push_back(std::make_pair<std::string, std::string> ("__name", "StudioPlugin"));
        args.push_back(std::make_pair<std::string, std::string> ("__master","http://10.211.55.3:11311"));
	args.push_back(std::make_pair<std::string, std::string> ("__ns","rovi2"));

        args.push_back(std::make_pair<std::string, std::string> ("__ip", "10.211.55.3"));

        int argc = 0;
        ros::init(args, "StudioPlugin");
        _nh = new ros::NodeHandle;

        _rosTimer->start(10);
        _btnROS->setEnabled(false);

	//ros::spin();


}

void StudioPlugin::checkROS()
{
	ros::spinOnce();


}



void StudioPlugin::close()
{
	_wc = NULL;

}

void StudioPlugin::stateChangedListener(const rw::kinematics::State& state) {
	_state = state;
}




Q_EXPORT_PLUGIN(StudioPlugin);
