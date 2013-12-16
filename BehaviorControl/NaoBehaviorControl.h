/***************************************************************************
 *   Copyright (C) 2008 by Zhu_Ming,Zheng Yonglei  Qu Junjun *
 *   zhuming535984@gmail.com,zhengyonglei@gmail.com   *
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
#ifndef NAOBEHAVIORCONTROL_H
#define NAOBEHAVIORCONTROL_H

#include "BehaviorControl.h"
#include "ModuleHandler.h"
#include "AdvancedAgent.h"
#include "Net.h"
#include <vector>


class NaoBehaviorControl : public BehaviorControl
{
public:
    NaoBehaviorControl( ModuleHandler& moduleHandler, const BehaviorControlInterfaces& interfaces);
    ~NaoBehaviorControl() {}

    virtual void execute();
    virtual std::string GetActionCommand() {return "";}

    void GoToBall();
    void GoToBallByCommunication();
    void GoToBallWithoutTurning();
    void GoToPoint(salt::Vector2f dest);

    void GoToRelativePoint(salt::Vector3f Destination);
    void AdjustmentForKick( float AdjustX,float AdjustY);
    void Stop();
    void Stand();
    void TurnTo(double angleToDestination);

	bool ArriveKickPosition();

    /** For GUI Debugger*/
    void 	sendMsg2GuiDebugger();
protected:
    /**
    * A reference to the ModuleHandler of the Process.
    * Needed to create new solutions.
    */
    ModuleHandler& moduleHandler;

    std::vector<salt::Vector2f> mTmpPosVect;

    enum BehaviorControlState
    {
        handle_ball ,
        get_to_ball,
        walk_to_position_behind_the_ball,
        kick,
        dribble,
    } behaviorControlState;

	enum Adjustment
	{	
		adjustX,
		adjustY
	}adjustment;

    BehaviorControlState lastBehaviorControlState;
};

#endif