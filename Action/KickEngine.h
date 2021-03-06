/***************************************************************************
 *   Copyright (C) 2008 by Zhu_Ming,Zheng Yonglei                          *
 *   Copyright (C) 2009 by Zhu_Ming,Zheng Yonglei,Qu Junjun,Liu Zhengliang *
 *                         Pan Yan,Yaoyuan                                 *
 *   zhuming535984@gmail.com,zhengyonglei@gmail.com                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef KICKENGINE_H
#define KICKENGINE_H

#include <boost/shared_array.hpp>
#include "Module.h"
#include "ActionRequest.h"
#include "ActionInfo.h"

/**
* Interfaces of the KickEngine module.
*/
class KickEngineInterfaces
{
public:
  /** Constructor */
  KickEngineInterfaces(
    const ActionRequest::ActionID& lastActionType,
    const ActionRequest& actionRequest,
    ActionInfo& actionInfo,
	boost::shared_array<float> JointAngle,
	boost::shared_array<float> JointVel)
    : lastActionType(lastActionType),
    actionRequest(actionRequest),
    actionInfo(actionInfo),
    mJointAngle(JointAngle),
    mJointVel(JointVel)
    {}
protected:  
  /** The action type of the previous frame */
  const ActionRequest::ActionID& lastActionType;
  
  /** The current action request */
  const ActionRequest& actionRequest;

  /** The neck height and the body tilt to be estimated by the getup engine */
  ActionInfo& actionInfo;

    boost::shared_array<float> mJointAngle;

    boost::shared_array<float> mJointVel;
};

/**
* A generic class for getup engine modules.
*
* The getup engine lets the robot stand up from any position
* by generating appropriate joint data.
*/
class KickEngine : public Module, protected KickEngineInterfaces
{
public:
/*
* Constructor.
* @param interfaces The paramters of the KickEngine module.
  */
  KickEngine(const KickEngineInterfaces& interfaces)
    : KickEngineInterfaces(interfaces)
  {}
  
  /**
  * calculates the next joint data set
  * @param jointData stores calculated frame
  * @return true if next set should be calculated by KickEngine
  *         false if change to other module can occur
  */
  virtual bool executeParameterized() = 0;
  
  /** Destructor */
  virtual ~KickEngine() {}
  
protected:
  /** That function is not used but has to be implemented */
  	virtual void execute() {};
	virtual std::string GetActionCommand() {return "";};
};

#endif