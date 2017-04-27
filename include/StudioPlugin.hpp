#ifndef StudioPlugin_HPP
#define StudioPlugin_HPP

// Includes
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "../../build/rovi2/ui_StudioPlugin.h"

#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics.hpp>
#include "ros/ros.h"
#include "ros/package.h"
#include "ros/this_node.h"

// QT
#include <QWidget>
#include <QTimer>
#include <QFileDialog>
#include <QMessageBox>


// Class
class StudioPlugin : public rws::RobWorkStudioPlugin, private Ui::StudioPlugin
{
	Q_OBJECT
	Q_INTERFACES(rws::RobWorkStudioPlugin)

	public:
        StudioPlugin();
        virtual ~StudioPlugin();
	virtual void open(rw::models::WorkCell* workcell);
	virtual void close();
	virtual void initialize();

	private slots:
	    void stateChangedListener(const rw::kinematics::State &state);
            void openROS();
            void checkROS();

	private:

            rw::models::WorkCell::Ptr _wc;
            rw::models::Device::Ptr _device;
	    rw::kinematics::State _state;
	    rws::RobWorkStudio *_rws;
            QTimer *_rosTimer;
	    ros::NodeHandle *_nh;
            //ros::Subscriber _subscriber;

};

#endif
