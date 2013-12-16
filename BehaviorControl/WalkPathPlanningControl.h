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

#ifndef WALKPATHPLANNINGCONTROL_H
#define WALKPATHPLANNINGCONTROL_H

#include <vector>
#include <string>
#include <queue>
using std::string;

#include "BehaviorControl.h"
#include "ModuleHandler.h"

#include "AdvancedAgent.h"

#include "spline.h"
#include "vector.h"
using salt::Vector2f;
using salt::Vector3f;

#include "sexp.h"

enum CmdType
{
	CT_NoMsg,
	CT_MsgRcvd,
	CT_WayPoint,
	CT_JoyStick,
	CT_Destination
	//CT_Obstacle
};

class DebugWayPoint
{
public:
	DebugWayPoint(Vector2f p, bool pa = false):
		mPos(p),
		mPassed(pa)
	{}

	DebugWayPoint(const DebugWayPoint& other) :
		mPos(other.pos()),
		mPassed(other.passed())
	{}

	Vector2f pos() const {return mPos;}
	bool passed() const {return mPassed;}
	void setPassed(bool p){mPassed = p;}

private:
	Vector2f mPos;
	bool mPassed;
};

class DebugObstacle
{
public:
	DebugObstacle(Vector2f p, float r) :
		mPos(p),
		mRadius(r)
	{}

	DebugObstacle(const DebugObstacle& other) :
		mPos(other.pos()),
		mRadius(other.radius())
	{}

	Vector2f pos() const {return mPos;}
	float radius() const {return mRadius;}

private:
	Vector2f mPos;
	float mRadius;
};

class DebugMsgParser
{
public:
	DebugMsgParser();
	virtual ~DebugMsgParser();

	void parseDebugMsg(string msg);

	CmdType cmdType() const {return mCmdType;}

	float joyStickX() const {return mJoyStickX;}
	float joyStickY() const {return mJoyStickY;}
	float joyStickR() const {return mJoyStickR;}

	vector<DebugWayPoint>& wayPointVect(){return mWayPointVect;}
	vector<DebugObstacle>& obstacleVect(){return mObstacleVect;}
	Vector2f destination() const {return mDestination;}

private:
	void parseWayPoint( const sexp_t* sexp );
	void parseJoyStick( const sexp_t* sexp );
	void parseObstacle( const sexp_t* sexp );
	void parseDestination( const sexp_t* sexp );

	CmdType mCmdType;

	pcont_t* _pcont;
	sexp_t* _sexp;

	vector<DebugWayPoint> mWayPointVect;
	vector<DebugObstacle> mObstacleVect;

	Vector2f mDestination;
	bool mIsDestValid;

	float mJoyStickX;
	float mJoyStickY;
	float mJoyStickR;
};

enum WayPointType
{
	WPT_Line, //to to the next way point using line
	WPT_Arc, //go to the next way point using arc
	WPT_Dest //destination
};

class WayPoint
{
public:
	WayPoint() :
		mIsValid(false),
		mParent(NULL)
	{
	}

	WayPoint(Vector2f& pos, WayPointType type = WPT_Line) :
		mIsValid(true),
		mPos(pos),
		mWayPointType(type),
		mParent(NULL)
	{
	}

	WayPoint(const WayPoint& other) :
		mIsValid(other.isValid()),
		mPos(other.pos()),
		mWayPointType(other.type()),
		mParent(other.parent()),
		mChildren(other.children())
	{
	}

	bool isValid() const {return mIsValid;}
	Vector2f pos() const {return mPos;}
	WayPointType type() const {return mWayPointType;}

	void setParent(WayPoint* parent){mParent = parent;}
	WayPoint* parent() const {return mParent;}

	void addChild(WayPoint* child)
	{
		mChildren.push_back(child);
		child->setParent(this);
	}

	WayPoint* firstChild() const
	{
		if (!mChildren.empty()) return mChildren[0];
		else return NULL;
	}

	vector<WayPoint*> children() const {return mChildren;}

private:
	bool mIsValid;
	Vector2f mPos;
	WayPointType mWayPointType;

	WayPoint* mParent;
	vector<WayPoint*> mChildren;
};

class PathPlanningTest
{
public:
	PathPlanningTest();
	virtual ~PathPlanningTest();

	void startTest(
			Vector2f& agentPos,
			Vector2f& goalPos,
			vector<DebugObstacle>& obstacleList);

	vector<WayPoint*> tryPath(
			WayPoint* curWayPoint,
			Vector2f& goalPos,
			vector<DebugObstacle>& obstacleList);

	WayPoint* chooseBestPath();
	void clearWayPoints();

	Vector2f getCircleTanPoint(
			Vector2f& centralPoint,
			float radius,
			Vector2f& startPoint,
			int factor); //factor can be 1 or -1

	void saveResults(WayPoint* endPos);
	vector<Vector2f>& finalWayPoints(){return mFinalWayPoints;}
	//void drawPath(WayPoint* endPos, QPainter& painter); //from tail to front
	//void updateWayPointCommand(WayPoint* endPos);

//private:
//	Vector2f mAgentPos;
private:
	//int mSocketDescriptor;

	queue<WayPoint*> mWayPointQueue;
	vector<WayPoint*> mCandidatePath; //the destination way point vector
	vector<WayPoint*> mVisitedWayPoints;
	vector<Vector2f> mFinalWayPoints;

	float mExtraSafeRadius;
};

class WalkPathPlanningControl : public BehaviorControl
{
public:
	WalkPathPlanningControl( ModuleHandler& moduleHandler, const BehaviorControlInterfaces& interfaces);
	~WalkPathPlanningControl() {}

    	virtual void execute();
    	virtual std::string GetActionCommand() {return "";}

	void GoToPoint(Vector2f dest);
	void parseDebugMsg();

	void testWayPoint();
	void testJoyStick();
	void testDestination();

	void sendMsg2GuiDebugger();

protected:
  	/**
  	* A reference to the ModuleHandler of the Process.
  	* Needed to create new solutions.
  	*/
  	ModuleHandler& moduleHandler;

  	DebugMsgParser mDebugMsgParser;
  	PathPlanningTest mPathPlanningTest;

};

#endif
