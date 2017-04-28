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
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>

// QT
#include <QWidget>
#include <QTimer>
#include <QFileDialog>
#include <QMessageBox>
#include <rovi2/Q.h>
#include <rovi2/State.h>
#include <rovi2/position3D.h>



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
        void updateCallback(const rovi2::State &state); 
        void ballCallback(const rovi2::position3D &position);

	private slots:
	    void stateChangedListener(const rw::kinematics::State &state);
            void openROS();
            void checkROS();

	private:


	/************************************************************************
	 * Q  -> lend from Caros
	 ************************************************************************/
	rw::math::Q toRw(const rovi2::Q& q)
	{
	  rw::math::Q res(q.data.size());
	  for (std::size_t i = 0; i < q.data.size(); ++i)
	  {
	    res(i) = q.data[i];
	  }
	  return res;
	}

            rw::models::WorkCell::Ptr _wc;
            rw::models::Device::Ptr _device;
	    rw::kinematics::State _state;
	    rws::RobWorkStudio *_rws;
            QTimer *_rosTimer;
	    ros::NodeHandle *_nh;
            ros::Subscriber _subscriberState;
            ros::Subscriber _subscriberBall;
	    rw::kinematics::MovableFrame* _BallFrame;
            

};

#endif
