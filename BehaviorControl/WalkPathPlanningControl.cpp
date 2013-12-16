/***************************************************************************
 *   Copyright (C) 2008 by Zhu_Ming,Zheng Yonglei , Qu Junjun  *
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
#include "WalkPathPlanningControl.h"
#include "gmath.h"
#include "ball.h"
#include "worldmodel.h"

WalkPathPlanningControl::WalkPathPlanningControl( ModuleHandler& moduleHandler, const BehaviorControlInterfaces& interfaces):BehaviorControl(interfaces),moduleHandler(moduleHandler)
{
	//cout<<"WalkPathPlanningControl::WalkPathPlanningControl"<<endl;
}

void WalkPathPlanningControl::parseDebugMsg()
{	
	//aDEBUG << "WM.getSelf()POS: " << WM.getSelf().pos << endl;
	//aDEBUG << "WM.getBall()POS: " << WM.getBall().pos << endl;

	DEBUG<<"DebugMsgParser::parseDebugMsg-->debug_msg="<<debug_msg<<endl;
	if (!debug_msg.empty())
		mDebugMsgParser.parseDebugMsg(debug_msg);
}

void WalkPathPlanningControl::GoToPoint(Vector2f toDestination)
{	
	double maxSpeed = 0.6f;
	double maxSpeedY = 0.2f;
	double maxTurnSpeed = 5.0;
	//Vector2f toDestination( -6.0,-4.0 );
	//Vector2f toDestination = dest;
	Vector3f agentWorldPos = WM.getSelf().pos;
	Vector2f destinationToAgent = toDestination - agentWorldPos.to2D();
	double distanceToDestination = destinationToAgent.Length();
  
  	double angleToDestination = gNormalizeDeg( destinationToAgent.GetAngleDeg()-WM.getSelf().GetTorsoYawAngle());

 	Vector2f destination( distanceToDestination * gCos(gDegToRad(angleToDestination)),distanceToDestination * gSin(gDegToRad(angleToDestination)) );

  	float factorClipping = 1.0; // full speed

  	if (distanceToDestination < 0.5)
  	{
		factorClipping = 0.9; // smoother approach
  	}

	if (angleToDestination >90.0)
	{	angleToDestination -= 180.0;
	}
	else if (angleToDestination < -90.0)
	{	angleToDestination += 180.0;
	}

  	double factor = (90-fabs(angleToDestination))/90;
  	if (factor > factorClipping) factor = factorClipping;
  	if (factor < 0) factor = 0;
  
  	destination.Normalized();
  	destination *= (maxSpeed*factor);
    
  	//motionRequest.motionType = MotionRequest::walk;
  	//motionRequest.walkRequest.walkType = static_cast<WalkRequest::WalkType>(static_cast<int>(walkType));
	actionRequest.walkRequest.walkParams = destination;
  	
	actionRequest.walkRequest.walkParams.x() *= 0.2; 
	actionRequest.walkRequest.walkParams.y() *=-0.2;
	
	//test not need turn so much,just walk formward or backward

	actionRequest.walkRequest.walkParams.x() = gMin(actionRequest.walkRequest.walkParams.x(),0.12f);
	actionRequest.walkRequest.walkParams.x() = gMax(actionRequest.walkRequest.walkParams.x(),-0.12f);
 
	actionRequest.walkRequest.walkParams.y() = gMin(actionRequest.walkRequest.walkParams.y(),0.04f);
	actionRequest.walkRequest.walkParams.y() = gMax(actionRequest.walkRequest.walkParams.y(),-0.04f);
	

	actionRequest.walkRequest.rotation = angleToDestination;
  
  	// clip rotation speed
  	actionRequest.walkRequest.rotation = gMin(actionRequest.walkRequest.rotation, maxTurnSpeed);
  	actionRequest.walkRequest.rotation = gMax(actionRequest.walkRequest.rotation, -maxTurnSpeed);
	
}

void WalkPathPlanningControl::execute()
{
	DEBUG<<"WalkPathPlanningControl::execute"<<endl;
	//get up if necessary

	AA.autoSendMsg = false;
	debug_msg = AA.debug_msg;
	parseDebugMsg();

	switch(mDebugMsgParser.cmdType())
	{
	case CT_NoMsg:
	case CT_MsgRcvd:
		//do nothing
		break;
	case CT_WayPoint:
		testWayPoint();
		break;
	case CT_JoyStick:
		testJoyStick();
		break;
	case CT_Destination:
		testDestination();
		break;
	}
	//GoToPoint();
	sendMsg2GuiDebugger();

	if(WM.getSelf().onMyback()||WM.getSelf().onMyBelly()||WM.getSelf().onMySide())
	{
		actionRequest.actionType = ActionRequest::getup;
		return;
	}
}

void WalkPathPlanningControl::testWayPoint()
{
	vector<DebugWayPoint>& wayPoint = mDebugMsgParser.wayPointVect();
	if (wayPoint.empty())
		return;

	for (int i=0; i<wayPoint.size(); i++)
	{
		DebugWayPoint& wp = wayPoint[i];
		if (wp.passed())
			continue;

		Vector2f displacement = wp.pos() - WM.getSelf().pos.to2D();
		if (displacement.Length() < 0.3)
			wp.setPassed(true);

		if (!wp.passed())
		{
			DEBUG<<"WalkPathPlanningControl::testWayPoint-->GoToPoint("<<wp.pos()<<")"<<endl;
			GoToPoint(wp.pos());
			break;
		}
	}
}

void WalkPathPlanningControl::testJoyStick()
{
	actionRequest.walkRequest.walkParams.x() = mDebugMsgParser.joyStickX();
	actionRequest.walkRequest.walkParams.y() = mDebugMsgParser.joyStickY();
	actionRequest.walkRequest.rotation = mDebugMsgParser.joyStickR();
}

void WalkPathPlanningControl::sendMsg2GuiDebugger()
{
	DEBUG<<"WalkPathPlanningControl::sendMsg2GuiDebugger"<<endl;
	//cout<<"AdvancedAgent::sendInfo2GuiDebugger-->(my_pos("<<WM.getSelf().pos<<"))"<<endl;
	//if (WM.getTmm(1).isValid())
	//	cout<<"AdvancedAgent::sendInfo2GuiDebugger-->WM.getTmm("<<1<<").pos = "<<WM.getTmm(0).pos<<endl;
	if (DEBUG_NET.isValid())
	{
		ostringstream o_msg;
		o_msg<<"<r>";
			o_msg<<"<c>"<<int(WM.getSelf().CurrentCycle)<<"</c>";
			o_msg<<"<rp>";
				o_msg<<"<dir>"<<WM.getSelf().GetTorsoYawAngle()<<"</dir>";
				o_msg<<"<pos>";
					o_msg<<"<t>Strive3D</t>";
					o_msg<<"<u>"<<WM.getSelf().GetUnum()<<"</u>";
					o_msg<<"<h>"<<WM.getSelf().pos<<"</h>";
					o_msg<<"<lla>"<<WM.getSelf().GetLeftArmGloblePosition()<<"</lla>";
					o_msg<<"<rla>"<<WM.getSelf().GetRightArmGloblePosition()<<"</rla>";
					o_msg<<"<lf>"<<WM.getSelf().GetLeftFootGloblePosition()<<"</lf>";
					o_msg<<"<rf>"<<WM.getSelf().GetRightFootGloblePosition()<<"</rf>";
				o_msg<<"</pos>";
			o_msg<<"</rp>";
			//o_msg<<"<c>"<<int(WM.getMySimTime()*100)<<"</c>";
			o_msg<<"<av>";
			for (int i=0; i<MAX_TEAM_SIZE; i++)
			{
				if (WM.getTmm(i).isValid())
				{
					o_msg<<"<p>";
						o_msg<<"<t>Strive3D</t>";
						o_msg<<"<u>"<<WM.getTmm(i).GetUnum()<<"</u>";
						o_msg<<"<h>"<<WM.getTmm(i).pos<<"</h>";
						//o_msg<<"<h>"<<WM.getTmm(i).GetHeadGloblePosition()<<"</h>";
						o_msg<<"<lla>"<<WM.getTmm(i).GetLeftArmGloblePosition()<<"</lla>";
						o_msg<<"<rla>"<<WM.getTmm(i).GetRightArmGloblePosition()<<"</rla>";
						o_msg<<"<lf>"<<WM.getTmm(i).GetLeftFootGloblePosition()<<"</lf>";
						o_msg<<"<rf>"<<WM.getTmm(i).GetRightFootGloblePosition()<<"</rf>";
					o_msg<<"</p>";
				}
			}
			for (int i=0; i<MAX_TEAM_SIZE; i++)
			{
				if (WM.getOpp(i).isValid())
				{
					o_msg<<"<p>";
						o_msg<<"<t>Oponent</t>";
						o_msg<<"<u>"<<WM.getOpp(i).GetUnum()<<"</u>";
						o_msg<<"<h>"<<WM.getOpp(i).pos<<"</h>";
						//o_msg<<"<h>"<<WM.getOpp(i).GetHeadGloblePosition()<<"</h>";
						o_msg<<"<lla>"<<WM.getOpp(i).GetLeftArmGloblePosition()<<"</lla>";
						o_msg<<"<rla>"<<WM.getOpp(i).GetRightArmGloblePosition()<<"</rla>";
						o_msg<<"<lf>"<<WM.getOpp(i).GetLeftFootGloblePosition()<<"</lf>";
						o_msg<<"<rf>"<<WM.getOpp(i).GetRightFootGloblePosition()<<"</rf>";
					o_msg<<"</p>";
				}
			}
				o_msg<<"<o>";
					o_msg<<"<role>ball</role>";
					o_msg<<"<id>1</id>";
					o_msg<<"<pos>"<<WM.getBall().pos<<"</pos>";
				o_msg<<"</o>";
			o_msg<<"</av>";
			o_msg<<"<ad>";
				o_msg<<"<x>"<<actionRequest.walkRequest.walkParams.x()<<"</x>";
				o_msg<<"<y>"<<actionRequest.walkRequest.walkParams.y()<<"</y>";
				o_msg<<"<r>"<<actionRequest.walkRequest.rotation<<"</r>";
				if (mDebugMsgParser.cmdType() == CT_Destination && mPathPlanningTest.finalWayPoints().size() > 0)
				{
				o_msg<<"<wp>";
					vector<Vector2f>& finalWayPoints = mPathPlanningTest.finalWayPoints();
					for (int i=0; i<finalWayPoints.size(); i++)
					{
					o_msg<<"<pos>"<<finalWayPoints[i]<<"</pos>";
					}
				o_msg<<"</wp>";
				}
			o_msg<<"</ad>";
		o_msg<<"</r>";
		//cout<<"AdvancedAgent::sendInfo2GuiDebugger-->o_msg="<<o_msg.str()<<endl;
		DEBUG<<"WalkPathPlanningControl::sendMsg2GuiDebugger-->o_msg="<<o_msg.str()<<endl;
		DEBUG_NET.PutMessage(o_msg.str());
	}
}

void WalkPathPlanningControl::testDestination()
{
	DEBUG<<"WalkPathPlanningControl::testDestination"<<endl;
	Vector2f agentPos = WM.getSelf().pos.to2D();
	Vector2f goalPos = mDebugMsgParser.destination();

	mPathPlanningTest.startTest(agentPos, goalPos, mDebugMsgParser.obstacleVect());
	//GoToPoint();
	GoToPoint(goalPos);
	return;
	//return;
	//double maxSpeed = 0.6f;
	double maxSpeedX = 0.6f;
	double maxSpeedY = 0.2f;
	double maxTurnSpeed = 5.0f;
	Vector2f toDestination = mDebugMsgParser.destination();
	Vector3f agentWorldPos = WM.getSelf().pos;
	Vector2f destinationToAgent = toDestination - agentWorldPos.to2D();
	double distanceToDestination = destinationToAgent.Length();
	//DEBUG<<"WalkPathPlanningControl::testDestination-->distanceToDestination="<<distanceToDestination<<endl;

	//double angleToDestination = gNormalizeDeg( destinationToAgent.GetAngleDeg() - WM.getSelf().GetTorsoYawAngle());
	double angleToDestination = destinationToAgent.GetAngleRad() - WM.getSelf().GetTorsoYawAngle();

	double paramX = maxSpeedX * cos(angleToDestination);
	double paramY = maxSpeedY * sin(angleToDestination);
	//DEBUG<<"WalkPathPlanningControl::testDestination-->paramX="<<paramX<<endl;
	//DEBUG<<"WalkPathPlanningControl::testDestination-->paramY="<<paramY<<endl;
	double paramR = 0.0f;

	double agentBodyDirRad = WM.getSelf().GetTorsoYawAngle();
	Vector2f agent2Dest2D = toDestination - agentWorldPos.to2D();
	Vector2f agentBodyDirVect2D = Vector2f(1.0f * cos(agentBodyDirRad), 1.0f * sin(agentBodyDirRad));
	Vector3f agent2Dest3D = Vector3f(agent2Dest2D.x(), agent2Dest2D.y(), 0.0f);
	Vector3f agentBodyDirVect3D = Vector3f(agentBodyDirVect2D.x(), agentBodyDirVect2D.y(), 0.0f);
	//if (agent2Dest3D.Cross(agentBodyDirVect3D).z() * agent2Dest3D.Dot(agentBodyDirVect3D) < 0)
	//	paramR = - maxTurnSpeed;
	//else
	//	paramR =  maxTurnSpeed;

	//DEBUG<<"WalkPathPlanningControl::testDestination-->paramR="<<paramR<<endl;
	double lastParamX = actionRequest.walkRequest.walkParams.x();
	double lastParamY = actionRequest.walkRequest.walkParams.y();
	double lastParamR = actionRequest.walkRequest.rotation;

	//DEBUG<<"WalkPathPlanningControl::testDestination-->lastParamX="<<lastParamX<<endl;
	//DEBUG<<"WalkPathPlanningControl::testDestination-->lastParamY="<<lastParamY<<endl;

	double maxIncX = 0.1f;
	double maxIncY = 0.05f;
	double maxIncR = 0.5f;

	if (fabs(paramX - lastParamX) > maxIncX)
	{
		if (paramX > 0)
			paramX = lastParamX + maxIncX;
		else
			paramX = lastParamX - maxIncX;
	}

	if (fabs(paramY - lastParamY) > maxIncY)
		paramY = lastParamY + maxIncY;

	if (fabs(paramR - lastParamR) > maxIncR)
		paramR = lastParamR + maxIncR;

	//DEBUG<<"WalkPathPlanningControl::testDestination-->paramX="<<paramX<<endl;
	//DEBUG<<"WalkPathPlanningControl::testDestination-->paramY="<<paramY<<endl;

	if (paramX > 0) actionRequest.walkRequest.walkParams.x() = gMin(maxSpeedX, paramX);
	else actionRequest.walkRequest.walkParams.x() = gMax( - maxSpeedX, paramX);

	if (paramY > 0)  actionRequest.walkRequest.walkParams.y() = gMin(maxSpeedY, paramY);
	else actionRequest.walkRequest.walkParams.y() = gMin( - maxSpeedY, paramY);

	actionRequest.walkRequest.rotation = gMin(maxTurnSpeed, paramR);

	DEBUG<<"WalkPathPlanningControl::testDestination-->actionRequest.walkRequest.walkParams.x()="<<actionRequest.walkRequest.walkParams.x()<<endl;
	DEBUG<<"WalkPathPlanningControl::testDestination-->actionRequest.walkRequest.walkParams.y()="<<actionRequest.walkRequest.walkParams.y()<<endl;
	DEBUG<<"WalkPathPlanningControl::testDestination-->actionRequest.walkRequest.rotation="<<actionRequest.walkRequest.rotation<<endl;

	return;

	/*Vector2f destination( distanceToDestination * gCos(gDegToRad(angleToDestination)),distanceToDestination * gSin(gDegToRad(angleToDestination)) );

	float factorClipping = 1.0; // full speed

	if (distanceToDestination < 0.5)
	{
		factorClipping = 0.9; // smoother approach
	}

	if (angleToDestination >90.0)
	{	angleToDestination -= 180.0;
	}
	else if (angleToDestination < -90.0)
	{	angleToDestination += 180.0;
	}

	double factor = (90-fabs(angleToDestination))/90;
	if (factor > factorClipping) factor = factorClipping;
	if (factor < 0) factor = 0;

	destination.Normalized();
	destination *= (maxSpeed*factor);

	//motionRequest.motionType = MotionRequest::walk;
	//motionRequest.walkRequest.walkType = static_cast<WalkRequest::WalkType>(static_cast<int>(walkType));
	actionRequest.walkRequest.walkParams = destination;
*/
	actionRequest.walkRequest.walkParams.x() *= 0.2;
	actionRequest.walkRequest.walkParams.y() *=-0.2;

	//test not need turn so much,just walk formward or backward

	actionRequest.walkRequest.walkParams.x() = gMin(actionRequest.walkRequest.walkParams.x(),0.12f);
	actionRequest.walkRequest.walkParams.x() = gMax(actionRequest.walkRequest.walkParams.x(),-0.12f);

	actionRequest.walkRequest.walkParams.y() = gMin(actionRequest.walkRequest.walkParams.y(),0.04f);
	actionRequest.walkRequest.walkParams.y() = gMax(actionRequest.walkRequest.walkParams.y(),-0.04f);


	actionRequest.walkRequest.rotation = angleToDestination;

	// clip rotation speed
	actionRequest.walkRequest.rotation = gMin(actionRequest.walkRequest.rotation, maxTurnSpeed);
	actionRequest.walkRequest.rotation = gMax(actionRequest.walkRequest.rotation, -maxTurnSpeed);
}

DebugMsgParser::DebugMsgParser() :
	mCmdType(CT_NoMsg),
	mIsDestValid(false)
{
}

DebugMsgParser::~DebugMsgParser()
{
}

void DebugMsgParser::parseDebugMsg(string msg)
{
	//DEBUG<<"DebugMsgParser::parseDebugMsg-->msg="<<msg<<endl;
	//if (msg.empty())
	//	return;

	//DEBUG<<"DebugMsgParser::parseDebugMsg-->msg="<<msg<<endl;
	int msgLen = msg.size();
	char* c = const_cast<char*>(msg.c_str());
	_pcont = init_continuation(c);
	_sexp = iparse_sexp(c,msgLen,_pcont);
	while( _sexp != 0)
	{
		if ( _sexp->ty == SEXP_LIST )
		{
			sexp_t* tmp = _sexp->list;
			if ( tmp->ty == SEXP_VALUE )
			{
				//DEBUG<<"DebugMsgParser::parseDebugMsg-->tmp->val="<<tmp->val<<endl;
				switch ( *(tmp->val) )
				{
				case 'm':
					if (string(tmp->val) == "message_received")
						//mCmdType = CT_MsgRcvd;
						//do nothing
						break;
					break;
				case 'w': //parse way point
					mCmdType = CT_WayPoint;
					parseWayPoint(tmp->next);
					break;
				case 'j': //parse joystick
					mCmdType = CT_JoyStick;
					parseJoyStick(tmp->next);
					break;
				case 'o': //parse obstacle
					//mCmdType = CT_Obstacle;
					parseObstacle(tmp->next);
					break;
				case 'd':
					mCmdType = CT_Destination;
					parseDestination(tmp->next);
					break;
				default:
					cout<<"DebugMsgParser::parseDebugMsg-->Invalid command"<<endl;
					DEBUG<<"DebugMsgParser::parseDebugMsg-->Invalid command"<<endl;
					assert(0);
				}
			}
		}
		destroy_sexp(_sexp);
		_sexp = iparse_sexp(c,msg.size(),_pcont);
	}
	destroy_continuation(_pcont);
}

void DebugMsgParser::parseWayPoint( const sexp_t* sexp )
{
	DEBUG<<"DebugMsgParser::parseWayPoint"<<endl;
	vector<DebugWayPoint> wayPoint;
	while ( sexp !=0 )
	{
		sexp_t* tmp = sexp->list;
		//DEBUG<<"DebugMsgParser::parseWayPoint-->tmp->val="<<tmp->val<<endl;
		if (string(tmp->val) == "NULL")
		{
			//mWayPointVect.clear();
			break;
		}
		else
		{
			float x = atof(tmp->val);
			tmp = tmp->next;
			float y = atof(tmp->val);
			wayPoint.push_back(DebugWayPoint(Vector2f(x, y)));
		}
		sexp = sexp->next;
	}

	mWayPointVect = wayPoint;
	for (int i=0; i<mWayPointVect.size(); i++)
		DEBUG<<"DebugMsgParser::parseWayPoint-->mWayPointVect["<<i<<"]="<<mWayPointVect[i].pos()<<endl;

}

void DebugMsgParser::parseJoyStick( const sexp_t* sexp )
{
	DEBUG<<"DebugMsgParser::parseJoyStick"<<endl;
	while ( sexp !=0 )
	{
		sexp_t* tmp = sexp->list;
		mJoyStickX = atof(tmp->val);
		DEBUG<<"DebugMsgParser::parseJoyStick-->mJoyStickX="<<mJoyStickX<<endl;
		tmp = tmp->next;
		mJoyStickY = atof(tmp->val);
		DEBUG<<"DebugMsgParser::parseJoyStick-->mJoyStickY="<<mJoyStickY<<endl;
		tmp = tmp->next;
		mJoyStickR = atof(tmp->val);
		DEBUG<<"DebugMsgParser::parseJoyStick-->mJoyStickR="<<mJoyStickR<<endl;

		sexp = sexp->next;
	}
}

void DebugMsgParser::parseObstacle( const sexp_t* sexp )
{
	DEBUG<<"DebugMsgParser::parseObstacle"<<endl;
	vector<DebugObstacle> obstacle;
	while ( sexp !=0 )
	{
		sexp_t* tmp = sexp->list;
		//DEBUG<<"DebugMsgParser::parseObstacle-->tmp->val="<<tmp->val<<endl;
		if (string(tmp->val) == "NULL")
			break;
		else
		{
			float x = atof(tmp->val);
			tmp = tmp->next;
			float y = atof(tmp->val);
			tmp = tmp->next;
			float r = atof(tmp->val);
			obstacle.push_back(DebugObstacle(Vector2f(x, y), r));
		}
		sexp = sexp->next;
	}

	mObstacleVect = obstacle;
	for (int i=0; i<mObstacleVect.size(); i++)
		DEBUG<<"DebugMsgParser::parseObstacle-->mObstacleVect["<<i<<"]="<<mObstacleVect[i].pos()<<"\t"<<mObstacleVect[i].radius()<<endl;
}

void DebugMsgParser::parseDestination( const sexp_t* sexp )
{
	DEBUG<<"DebugMsgParser::parseDestination"<<endl;
	vector<DebugObstacle> obstacle;
	while ( sexp !=0 )
	{
		sexp_t* tmp = sexp->list;
		//DEBUG<<"DebugMsgParser::parseObstacle-->tmp->val="<<tmp->val<<endl;
		if (string(tmp->val) == "NULL")
		{
			mIsDestValid = false;
			break;
		}
		else
		{
			float x = atof(tmp->val);
			tmp = tmp->next;
			float y = atof(tmp->val);
			mDestination = Vector2f(x, y);
		}
		sexp = sexp->next;
	}
}

PathPlanningTest::PathPlanningTest() :
	//mSocketDescriptor(sd),
	mExtraSafeRadius(0.1f)
{
	// TODO Auto-generated constructor stub

}

PathPlanningTest::~PathPlanningTest()
{
	// TODO Auto-generated destructor stub
}

void PathPlanningTest::startTest(
		Vector2f& agentPos,
		Vector2f& goalPos,
		vector<DebugObstacle>& obstacleList)
{
	DEBUG<<"PathPlanningTest::startTest"<<endl;
	//cout<<endl<<"PathPlanningTest::startTest"<<endl;

	int tryCount = 0;
	int tryCountMax = 50;
	clearWayPoints();
	WayPoint* curAgentPoint = new WayPoint(agentPos);
	mVisitedWayPoints.push_back(curAgentPoint);
	mWayPointQueue.push(curAgentPoint);
	while(!mWayPointQueue.empty())
	{
		tryCount++;
		//cout<<endl<<"PathPlanningTest::startTest-->tryCount="<<tryCount<<endl;
		if (tryCount > tryCountMax)
			break; //exception

		WayPoint* startPos = mWayPointQueue.front();
		//cout<<"PathPlanningTest::startTest-->mWayPointQueue.size()="<<mWayPointQueue.size()<<endl;
		mWayPointQueue.pop();
		if (startPos)
		{
			//cout<<"PathPlanningTest::startTest-->startPos->pos()="<<startPos->pos()<<endl;
			vector<WayPoint*> newWayPointList = tryPath(startPos, goalPos, obstacleList);
			assert(newWayPointList.size() <= 2);
			for (int i=0; i<newWayPointList.size(); i++)
			{
				if (newWayPointList[i] != NULL)
					mWayPointQueue.push(newWayPointList[i]);
			}
		}
	}

	WayPoint* start = chooseBestPath();
	if (start)
	{
		saveResults(start);
	}
	//if (start)
	//{
	//	drawPath(start, painter);
	//	updateWayPointCommand(start);
	//}

	//DEBUG<<"PathPlanningTest::startTest-->finished"<<endl;
}

void PathPlanningTest::saveResults(WayPoint* endPos)
{
	mFinalWayPoints.clear();
	//cout<<"PathPlanningTest::drawPath"<<endl;
	WayPoint* wayPoint = endPos;
	//vector<Vector2f> posVect;
	while(wayPoint != NULL)
	{
		Vector2f pos = wayPoint->pos();
		mFinalWayPoints.push_back(pos);
		wayPoint = wayPoint->parent();
	}

	DEBUG<<"PathPlanningTest::saveResults-->mFinalWayPoints.size()="<<mFinalWayPoints.size()<<endl;
	//apply spline
	if (mFinalWayPoints.size() > 1)
	{
		const int posCount = mFinalWayPoints.size();
		float* x = new float[posCount];
		float* y = new float[posCount];
		for (int i=mFinalWayPoints.size() -1; i >= 0; i--)
		{
			x[mFinalWayPoints.size() -1 - i] = mFinalWayPoints[i].x();
			y[mFinalWayPoints.size() -1 - i] = mFinalWayPoints[i].y();
		}

		Spline spline(x, y, posCount);
		float intervalX = 0.5f;
		if (x[0] > x[posCount - 1])
			intervalX *= -1.0f;

		float givenX = x[0];
		while(givenX <= x[posCount - 1])
		{
			//float getY = spline.GetYGivenX(givenX, 0.1);
			float getY = spline.GetYGivenX(givenX);
			givenX += intervalX;
		}

		delete [] x;
		delete [] y;
	}
}

vector<WayPoint*> PathPlanningTest::tryPath(
		WayPoint* curWayPoint,
		Vector2f& goalPos,
		vector<DebugObstacle>& obstacleList)
{
	DEBUG<<"PathPlanningTest::tryPath"<<endl;

	vector<WayPoint*> result;

	//calculate way point
	Vector2f wayPointA = curWayPoint->pos();
	Vector2f wayPointB = goalPos;
	DEBUG<<"PathPlanningTest::startTest-->wayPointA="<<wayPointA<<endl;
	DEBUG<<"PathPlanningTest::startTest-->wayPointB="<<wayPointB<<endl;
	//cout<<endl<<"PathPlanningTest::tryPath"<<endl;
	//cout<<"PathPlanningTest::tryPath-->wayPointA="<<wayPointA<<endl;
	//cout<<"PathPlanningTest::tryPath-->wayPointB="<<wayPointB<<endl;


	float distMin = 999999.9f;
	int distMinIndex = -1;
	float validH = 0;
	for (int i=0; i<obstacleList.size(); i++)
	{
		Vector3f o3D = Vector3f(obstacleList[i].pos().x(), obstacleList[i].pos().y(), 0.0f);
		Vector3f oa3D = Vector3f(wayPointA.x(), wayPointA.y(), 0.0f) - o3D;
		//if (oa3D.Length() < obstacleList[i].radius())
		//	continue;
		Vector3f ob3D = Vector3f(wayPointB.x(), wayPointB.y(), 0.0f) - o3D;
		float area = abs(oa3D.Cross(ob3D).Length());
		//cout<<"PathPlanningTest::startTest-->area="<<area<<endl;
		Vector3f ba3D = ob3D - oa3D;
		float floor = ba3D.Length();
		//DEBUG<<"PathPlanningTest::startTest-->o3D="<<o3D<<endl;
		//DEBUG<<"PathPlanningTest::startTest-->wayPointA="<<wayPointA<<endl;
		//DEBUG<<"PathPlanningTest::startTest-->oa3D="<<oa3D<<endl;
		//DEBUG<<"PathPlanningTest::startTest-->ob3D="<<ob3D<<endl;
		//DEBUG<<"PathPlanningTest::startTest-->ba3D="<<ba3D<<endl;

		//DEBUG<<"PathPlanningTest::startTest-->floor="<<floor<<endl;
		//DEBUG<<"PathPlanningTest::startTest-->area="<<area<<endl;
		float h = area/floor;
		//DEBUG<<"PathPlanningTest::startTest-->h="<<h<<endl;

		if (h < obstacleList[i].radius() && oa3D.Dot(ba3D) < 0 && ob3D.Dot(ba3D) > 0)
		{
			float dist = oa3D.Length();
			if (dist < distMin)
			{
				validH = h;
				distMin = dist;
				distMinIndex = i;
			}
		}
	}

	//most dangerous obstacle found, calculate safe distance
	float safeAngle = 0.1; //about 5.7 degree
	if (distMinIndex >= 0)
	{
		DebugObstacle& targetObstacle = obstacleList[distMinIndex];

		//cout<<"PathPlanningTest::startTest-->validH="<<validH<<endl;
		Vector2f ao = targetObstacle.pos() - wayPointA;
		float length_ao = ao.Length();
		//cout<<"PathPlanningTest::startTest-->length_ao="<<length_ao<<endl;
		float length_ah = sqrt(length_ao*length_ao - validH*validH);
		//cout<<"PathPlanningTest::startTest-->length_ah="<<length_ah<<endl;
		float safeDist = validH/tan(safeAngle);
		if (safeDist < targetObstacle.radius()*5)
			safeDist = targetObstacle.radius()*5;

		if (length_ah > safeDist)
		{
			float length_as = length_ah - safeDist;

			Vector2f ab = wayPointB - wayPointA;
			Vector2f as = (ab / ab.Length()) * length_as;
			Vector2f safe_div_pos = as + wayPointA;
			wayPointA = safe_div_pos;

			WayPoint* safe_div_point = new WayPoint(safe_div_pos);
			mVisitedWayPoints.push_back(safe_div_point);
			curWayPoint->addChild(safe_div_point);
			curWayPoint = safe_div_point;
		}
		//Vector2f unit_ba = Vector2f(wayPointB - wayPointA).Length();

		//calculate way point A1 and A2
		//screenWayPointA = OPT.screenPoint(wayPointA);
		//screenWayPointB = OPT.screenPoint(wayPointB);
		Vector2f centralPos = targetObstacle.pos();
		//float mExtraSafeRadius = 0.05;

		//cout<<"PathPlanningTest::tryPath-->calculate way point A1 and B2"<<endl;
		Vector2f pos_a1 = getCircleTanPoint(
				centralPos,
				targetObstacle.radius() + mExtraSafeRadius,
				wayPointA,
				1);
		WayPoint* wayPointA1 = new WayPoint(pos_a1, WPT_Arc);
		mVisitedWayPoints.push_back(wayPointA1);
		curWayPoint->addChild(wayPointA1);
		DEBUG<<"PathPlanningTest::startTest-->wayPointA1="<<wayPointA1->pos()<<endl;

		Vector2f pos_b2 = getCircleTanPoint(
				centralPos,
				targetObstacle.radius() + mExtraSafeRadius,
				wayPointB,
				- 1);

		//cout<<"PathPlanningTest::tryPath-->pos_b2="<<pos_b2<<endl;
		//if (pos_b2 != wayPointA && pos_b2 != wayPointB)
		//{
		WayPoint* wayPointB2 = new WayPoint(pos_b2);
		mVisitedWayPoints.push_back(wayPointB2);
		wayPointA1->addChild(wayPointB2);
		result.push_back(wayPointB2);
		DEBUG<<"PathPlanningTest::startTest-->wayPointB2="<<wayPointB2->pos()<<endl;

		//cout<<"PathPlanningTest::tryPath-->calculate way point A2 and B1"<<endl;
		Vector2f pos_a2 = getCircleTanPoint(
				centralPos,
				targetObstacle.radius() + mExtraSafeRadius,
				wayPointA,
				-1);
		WayPoint* wayPointA2 = new WayPoint(pos_a2, WPT_Arc);
		mVisitedWayPoints.push_back(wayPointA2);
		curWayPoint->addChild(wayPointA2);

		Vector2f pos_b1 = getCircleTanPoint(
				centralPos,
				targetObstacle.radius() + mExtraSafeRadius,
				wayPointB,
				1);

		//cout<<"PathPlanningTest::tryPath-->pos_b1="<<pos_b1<<endl;
		//if (pos_b1 != wayPointA && pos_b1 != wayPointB)
		//{
		WayPoint* wayPointB1 = new WayPoint(pos_b1);
		mVisitedWayPoints.push_back(wayPointB1);
		wayPointA2->addChild(wayPointB1);
		result.push_back(wayPointB1);
	}
	else
	{
		//cout<<"PathPlanningTest::tryPath-->solution found"<<endl;
		//solution found
		WayPoint* destWayPoint = new WayPoint(goalPos, WPT_Dest);
		mVisitedWayPoints.push_back(destWayPoint);
		mCandidatePath.push_back(destWayPoint);
		curWayPoint->addChild(destWayPoint);
	}

	//wayPointList<<goalPos;

	//cout<<"PathPlanningTest::tryPath-->result:"<<endl;
	//for (int i=0; i<result.size(); i++)
	//	cout<<"result["<<i<<"]="<<result[i]->pos()<<endl;

	return result;
}

Vector2f PathPlanningTest::getCircleTanPoint(
		Vector2f& centralPoint,
		float radius,
		Vector2f& startPoint,
		int factor)
{
	//DEBUG<<"PathPlanningTest::getCircleTanPoint"<<endl;
	Vector2f ao = centralPoint - startPoint;
	float length_ao = ao.Length();
	float length_oa1 = radius;
	float length_aa1 = sqrt(abs(length_ao*length_ao - length_oa1*length_oa1));
	//DEBUG<<"PathPlanningTest::getCircleTanPoint-->ao="<<ao<<endl;
	//DEBUG<<"PathPlanningTest::getCircleTanPoint-->length_ao="<<length_ao<<endl;
	//DEBUG<<"PathPlanningTest::getCircleTanPoint-->length_oa1="<<length_oa1<<endl;
	//DEBUG<<"PathPlanningTest::getCircleTanPoint-->length_aa1="<<length_aa1<<endl;
	//assert(length_ao);
	float angle_o_a_a1;
	if (length_ao == 0 || length_oa1 > length_ao)
		angle_o_a_a1 = M_PI/2;
	else
		angle_o_a_a1 = asin(length_oa1/length_ao);

	angle_o_a_a1 = angle_o_a_a1/M_PI * 180.0f;

	//cout<<"PathPlanningTest::getCircleTanPoint-->angle_o_a_a1="<<angle_o_a_a1<<endl;
	//cout<<"PathPlanningTest::getCircleTanPoint-->ao="<<ao<<endl;
	if (factor < 0)
		angle_o_a_a1 = -angle_o_a_a1;

	Vector2f unit_aa1_1 = ao.Rotated(angle_o_a_a1).Normalized();
	//cout<<"PathPlanningTest::getCircleTanPoint-->unit_aa1_1="<<unit_aa1_1<<endl;
	//cout<<"PathPlanningTest::getCircleTanPoint-->unit_aa1_1="<<unit_aa1_1<<endl;
	Vector2f aa1_1 = unit_aa1_1 * length_aa1;

	Vector2f pos_a1_1 = startPoint + aa1_1;
	//check
	Vector2f distVec = pos_a1_1 - centralPoint;
	//cout<<"PathPlanningTest::getCircleTanPoint-->pos_a1_1="<<pos_a1_1<<endl;
	if (distVec.Length() > radius*2)
		pos_a1_1 = centralPoint;

	return pos_a1_1;
}

/*void PathPlanningTest::drawPath(WayPoint* endPos, QPainter& painter)
{
	//cout<<"PathPlanningTest::drawPath"<<endl;
	float nodeRadius = 0.08;
	int screenNodeRadius = OPT.scale(nodeRadius);
	QPoint lastScreenWayPoint;

	WayPoint* wayPoint = endPos;
	vector<Vector2f> posVect;
	while(wayPoint != NULL)
	{
		//cout<<"PathPlanningTest::drawPath-->wayPoint.pos()="<<wayPoint->pos()<<endl;
		//draw current way points
		Vector2f pos = wayPoint->pos();
		posVect.push_back(pos);

		QPen linePen(Qt::DotLine);
		linePen.setWidth(3);
		painter.setPen(linePen);
		painter.setBrush(Qt::green);

		int screenPosX = OPT.screenX(pos.x());
		int screenPosY = OPT.screenY(pos.y());
		QPoint curScreenWayPoint(screenPosX, screenPosY);
		painter.drawEllipse(screenPosX - screenNodeRadius, screenPosY - screenNodeRadius, screenNodeRadius*2, screenNodeRadius*2);

		if (wayPoint->parent() != NULL) //draw next way points && connection
		{
			QPoint nextScreenWayPoint = OPT.screenPoint(wayPoint->parent()->pos());
			painter.setBrush(Qt::NoBrush);
			painter.drawLine(curScreenWayPoint, nextScreenWayPoint);
		}

		wayPoint = wayPoint->parent();
	}

	if (posVect.size() > 1)
	{
		const int posCount = posVect.size();
		float* x = new float[posCount];
		float* y = new float[posCount];
		for (int i=posVect.size() -1; i >= 0; i--)
		{
			x[posVect.size() -1 - i] = posVect[i].x();
			y[posVect.size() -1 - i] = posVect[i].y();
		}

		Spline spline(x, y, posCount);
		float intervalX = 0.5f;
		if (x[0] > x[posCount - 1])
			intervalX *= -1.0f;

		float givenX = x[0];
		while(givenX <= x[posCount - 1])
		{
			//float getY = spline.GetYGivenX(givenX, 0.1);
			float getY = spline.GetYGivenX(givenX);
			int screenGivenX = OPT.screenX(givenX);
			int screenGetY = OPT.screenY(getY);
			//Vector2f pos(givenX, getY);
			painter.setPen(Qt::yellow);
			painter.setBrush(Qt::cyan);
			QPointF posF(screenGivenX, screenGetY);
			painter.drawEllipse(posF, screenNodeRadius, screenNodeRadius);
			givenX += intervalX;
		}
	}
}

void PathPlanningTest::updateWayPointCommand(WayPoint* endPos)
{
	vector<Vector2f> finalPath;
	WayPoint* wayPoint = endPos;
	while(wayPoint != NULL)
	{
		finalPath.push_back(wayPoint->pos());
		wayPoint = wayPoint->parent();
	}

	QString command;
	if (finalPath.size() < 2)
		command = "(way_point (NULL))";
	else
	{
		QString way_point_s;
		for (int i = finalPath.size() - 2; i >= 0; i--)
		{
			QString way_point_field_x = QString("%1").arg(finalPath[i].x());
			QString way_point_field_y = QString("%1").arg(finalPath[i].y());
			QString way_point = QString("(%1 %2)").arg(way_point_field_x ,way_point_field_y);
			way_point_s.append(way_point);
		}
		command = QString("(way_point %1)").arg(way_point_s);
	}

	AR.addReply(mSocketDescriptor, command);
}
*/
WayPoint* PathPlanningTest::chooseBestPath()
{
	//cout<<"PathPlanningTest::chooseBestPath"<<endl;
	if (mCandidatePath.empty())
		return NULL;
	else //just for test
	{
		float distMin = 999999.9f;
		int distMinIndex = -1;
		for (int i=0; i<mCandidatePath.size(); i++)
		{
			WayPoint* wayPoint = mCandidatePath[i];
			float distance = 0.0f;
			while(wayPoint != NULL && wayPoint->parent() != NULL)
			{
				WayPoint* wayPointA = wayPoint;
				WayPoint* wayPointB = wayPoint->parent();
				Vector2f distVect = wayPointB->pos() - wayPointA->pos();
				distance += distVect.Length();
				wayPoint = wayPoint->parent();
			}

			//cout<<"PathPlanningTest::chooseBestPath-->mCandidatePath["<<i<<"]:distance = "<<distance<<endl;
			if (distance < distMin)
			{
				distMin = distance;
				distMinIndex = i;
			}
		}

		if (distMinIndex >= 0)
			return mCandidatePath[distMinIndex];
		else
			return NULL;
	}
}

void PathPlanningTest::clearWayPoints()
{
	while(!mWayPointQueue.empty())
		mWayPointQueue.pop();

	mCandidatePath.clear();

	for (int i=0; i<mVisitedWayPoints.size(); i++)
		delete mVisitedWayPoints[i];
	mVisitedWayPoints.clear();
}

