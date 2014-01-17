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

#include "NaoBehaviorControl.h"
#include "gmath.h"
#include "ball.h"
#include "worldmodel.h"

NaoBehaviorControl::NaoBehaviorControl( ModuleHandler& moduleHandler, const BehaviorControlInterfaces& interfaces):BehaviorControl(interfaces),moduleHandler(moduleHandler),adjustment(adjustX)
{
}

void NaoBehaviorControl::GoToBall()
{
    //double targetAngle;
    double maxSpeed = 0.6;
    double maxTurnSpeed = 5.0;
    float yOffset = 0.105;

    double angleToBall = gNormalizeDeg(WM.getVisionSense(BALL).theta);
    double distanceToBall = WM.getBall().GetDistanceToSelf();
    Vector3f ballLocalPos = WM.getBall().GetLocalPos();
    aLOG << "distanceToBall: " << distanceToBall<< " angleToBall: "<<angleToBall <<endl;
    Vector2f destination( ballLocalPos[0]-0.042f,ballLocalPos[1]+yOffset );
    aLOG << "ballLocalPos: " << ballLocalPos[0] <<", "<< ballLocalPos[1] <<endl;

    float factorClipping = 1.0; // full speed

    if (distanceToBall < 0.5)
    {
        factorClipping = 0.9; // smoother approach
    }

    // adjust forward speed:
    // if a lot of turning is necessary, don't walk so fast!
    if (angleToBall >90.0)
    {
        angleToBall -= 180.0;
    }
    else if (angleToBall < -90.0)
    {
        angleToBall += 180.0;
    }

    double factor = (90-fabs(angleToBall))/90;
    if (factor > factorClipping) factor = factorClipping;
    if (factor < 0) factor = 0;

    destination.Normalized();
    destination *= (1.5*maxSpeed*factor);


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


    actionRequest.walkRequest.rotation = angleToBall;

    // clip rotation speed
    actionRequest.walkRequest.rotation = gMin(actionRequest.walkRequest.rotation, maxTurnSpeed);
    actionRequest.walkRequest.rotation = gMax(actionRequest.walkRequest.rotation, -maxTurnSpeed);
}

void NaoBehaviorControl::GoToBallByCommunication()
{
    double maxSpeed = 0.6;
    double maxTurnSpeed = 5.0;
    float yOffset = 0.105;

// 	Vector3f ballWorldPos = WM.getBall().pos;
    Vector3f ballWorldPos = WM.getBall().GetLocalPos();
    Vector3f agentWorldPos = WM.getSelf().pos;
    Vector2f ballToAgent = ballWorldPos.to2D() - agentWorldPos.to2D();
    double distanceToBall = ballToAgent.Length();
    double angleToBall = gNormalizeDeg(ballToAgent.GetAngleDeg()-WM.getSelf().GetTorsoYawAngle());
    aLOG << "distanceToBall: " << distanceToBall<< " angleToBall: "<< angleToBall <<endl;
    double angleToBallRad = gDegToRad(angleToBall);
    Vector2f destination( distanceToBall * gCos(angleToBallRad),distanceToBall * gSin(angleToBallRad) );
    aLOG << "destination: " << distanceToBall * gCos(angleToBallRad) <<", "<< distanceToBall * gSin(angleToBallRad) <<endl;

    float factorClipping = 1.0; // full speed

    if (distanceToBall < 0.5)
    {
        factorClipping = 0.9; // smoother approach
    }

    if (angleToBall >90.0)
    {
        angleToBall -= 180.0;
    }
    else if (angleToBall < -90.0)
    {
        angleToBall += 180.0;
    }

    double factor = (90-fabs(angleToBall))/90;
    if (factor > factorClipping) factor = factorClipping;
    if (factor < 0) factor = 0;

    destination.Normalized();
    destination *= (1.5*maxSpeed*factor);



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


    actionRequest.walkRequest.rotation = angleToBall;

    // clip rotation speed
    actionRequest.walkRequest.rotation = gMin(actionRequest.walkRequest.rotation, maxTurnSpeed);
    actionRequest.walkRequest.rotation = gMax(actionRequest.walkRequest.rotation, -maxTurnSpeed);
}

void NaoBehaviorControl::GoToBallWithoutTurning()
{
    //double targetAngle;
    double maxSpeed = 0.6;
    float yOffset = 0.105;

    double angleToBall = gNormalizeDeg(WM.getVisionSense(BALL).theta);
    double distanceToBall = WM.getBall().GetDistanceToSelf();
    Vector3f ballLocalPos = WM.getBall().GetLocalPos();
    aLOG << "distanceToBall: " << distanceToBall<< " angleToBall: "<<angleToBall <<endl;
    Vector2f destination( ballLocalPos[0],ballLocalPos[1] );
    aLOG << "ballLocalPos: " << ballLocalPos[0] <<", "<< ballLocalPos[1] <<endl;

    destination.Normalized();
    destination *= maxSpeed;

    actionRequest.actionType = ActionRequest::walk;
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

    actionRequest.walkRequest.rotation = 0.0;
}

void NaoBehaviorControl::GoToPoint(Vector2f dest)
{
    double maxSpeed = 0.8f;
    double maxSpeedY = 0.2f;
    double maxTurnSpeed = 5.0;

    Vector3f agentWorldPos = WM.getSelf().pos;
    Vector2f destinationToAgent = dest - agentWorldPos.to2D();
    double distanceToDestination = destinationToAgent.Length();

    double angleToDestination = gNormalizeDeg( destinationToAgent.GetAngleDeg()-WM.getSelf().GetTorsoYawAngle());

    Vector2f destination( distanceToDestination * gCos(gDegToRad(angleToDestination)),distanceToDestination * gSin(gDegToRad(angleToDestination)) );

    float factorClipping = 1.0; // full speed

    if (distanceToDestination < 0.5)
    {
        factorClipping = 0.9; // smoother approach
    }

    /**if (angleToDestination >90.0)
    {
        angleToDestination -= 180.0;
    }
    else if (angleToDestination < -90.0)
    {
        angleToDestination += 180.0;
    }*/

    double factor = (180-fabs(angleToDestination))/180;
    if (factor > factorClipping) factor = factorClipping;
    if (factor < 0) factor = 0;

    destination.Normalized();
    destination *= (maxSpeed*factor);

    actionRequest.actionType = ActionRequest::walk;
    //motionRequest.motionType = MotionRequest::walk;
    //motionRequest.walkRequest.walkType = static_cast<WalkRequest::WalkType>(static_cast<int>(walkType));
    actionRequest.walkRequest.walkParams = destination;

    actionRequest.walkRequest.walkParams.x() *= 0.2;
    actionRequest.walkRequest.walkParams.y() *=-0.2;


    aLOG << "angleToDestination : " << angleToDestination << endl;
    if ( angleToDestination > 40.0 || angleToDestination < -40.0)
    {
        actionRequest.walkRequest.walkParams.x() = gMin(actionRequest.walkRequest.walkParams.x(),0.01f);
        actionRequest.walkRequest.walkParams.x() = gMax(actionRequest.walkRequest.walkParams.x(),-0.01f);

        actionRequest.walkRequest.walkParams.y() = gMin(actionRequest.walkRequest.walkParams.y(),0.01f);
        actionRequest.walkRequest.walkParams.y() = gMax(actionRequest.walkRequest.walkParams.y(),-0.01f);

        actionRequest.walkRequest.rotation = 2*angleToDestination;
        // clip rotation speed
        actionRequest.walkRequest.rotation = gMin(actionRequest.walkRequest.rotation, maxTurnSpeed*2);
        actionRequest.walkRequest.rotation = gMax(actionRequest.walkRequest.rotation, -maxTurnSpeed*2);
    }
    else
    {
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
}

void NaoBehaviorControl::GoToRelativePoint(Vector3f Destination)
{
    double maxSpeed = 0.6f;
    double maxSpeedY = 0.04f;
    double maxTurnSpeed = 5.0;

    Vector2f Destination2D(Destination.x(), Destination.y());

    double angleToDestination = gRadToDeg(gArcTan2(Destination.y(),Destination.x()));

    double distanceToDestination = Destination2D.Length();


    aLOG <<"distanceToDestination : "<< distanceToDestination <<endl
    << "angleToDestination : " << angleToDestination << endl;

    float factorClipping = 1.0; // full speed

    if (distanceToDestination < 0.5)
    {
        factorClipping = 0.9; // smoother approach
    }

    // adjust forward speed:
    // if a lot of turning is necessary, don't walk so fast!
    /**if (angleToDestination >90.0)
    {
        angleToDestination -= 180.0;
    }
    else if (angleToDestination < -90.0)
    {
        angleToDestination += 180.0;
    }*/

    double factor = (180-fabs(angleToDestination))/180;
    if (factor > factorClipping) factor = factorClipping;
    if (factor < 0) factor = 0;

    Destination2D.Normalized();
    Destination2D *= (1.5*maxSpeed*factor);

    actionRequest.actionType = ActionRequest::walk;
    //motionRequest.motionType = MotionRequest::walk;
    //motionRequest.walkRequest.walkType = static_cast<WalkRequest::WalkType>(static_cast<int>(walkType));
    actionRequest.walkRequest.walkParams = Destination2D;

    actionRequest.walkRequest.walkParams.x() *= 0.2;
    actionRequest.walkRequest.walkParams.y() *=-0.2;

    aLOG << "angleToDestination : " << angleToDestination << endl;
    if ( angleToDestination > 40.0 || angleToDestination < -40.0)
    {
        actionRequest.walkRequest.walkParams.x() = gMin(actionRequest.walkRequest.walkParams.x(),0.01f);
        actionRequest.walkRequest.walkParams.x() = gMax(actionRequest.walkRequest.walkParams.x(),-0.01f);

        actionRequest.walkRequest.walkParams.y() = gMin(actionRequest.walkRequest.walkParams.y(),0.01f);
        actionRequest.walkRequest.walkParams.y() = gMax(actionRequest.walkRequest.walkParams.y(),-0.01f);

        actionRequest.walkRequest.rotation = 2*angleToDestination;
        // clip rotation speed
        actionRequest.walkRequest.rotation = gMin(actionRequest.walkRequest.rotation, maxTurnSpeed*2);
        actionRequest.walkRequest.rotation = gMax(actionRequest.walkRequest.rotation, -maxTurnSpeed*2);
    }
    else
    {
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

    /**motionRequest.motionType = MotionRequest::walk;
    motionRequest.walkRequest.walkType = static_cast<WalkRequest::WalkType>(static_cast<int>(walkType));*/

    //this time does not necessarily include time for turning!:
    /**double estimatedTimeToReachDestination;
    if (distanceToDestination > 200)
    {
        estimatedTimeToReachDestination = (distanceToDestination + 200) / maxSpeed;

        actionRequest.walkRequest.walkParams.x() = cos(angleToDestination) * maxSpeed;
        actionRequest.walkRequest.walkParams.y() = sin(angleToDestination) * maxSpeed;
    }
    else
    {
        estimatedTimeToReachDestination = 2 * distanceToDestination / maxSpeed + 2 * fabs(angleDifference) / maxTurnSpeed;

        if (distanceToDestination > 30)
        {
            actionRequest.walkRequest.walkParams.x() = cos(angleToDestination) * maxSpeed * distanceToDestination / 200;
            actionRequest.walkRequest.walkParams.y() = sin(angleToDestination) * maxSpeed * distanceToDestination / 200;
        }
        else
        {
            actionRequest.walkRequest.walkParams.x() = 0;
            actionRequest.walkRequest.walkParams.y() = 0;
        }
    }

    // If the estimated time is 0, position is already reached, so nothing has to be done anymore
    if (estimatedTimeToReachDestination != 0)
    {
        if (fabs(toDegrees(angleDifference)) > 20)
        {
            actionRequest.walkRequest.rotation = angleDifference / estimatedTimeToReachDestination;
            actionRequest.walkRequest.rotation = min (maxTurnSpeed, motionRequest.walkRequest.walkParams.rotation);
            actionRequest.walkRequest.rotation = max (-maxTurnSpeed, motionRequest.walkRequest.walkParams.rotation);
        }
        else
        {
            actionRequest.walkRequest.rotation = 2 * angleDifference;
        }

        actionRequest.walkRequest.walkParams.y() = min(maxSpeedY, motionRequest.walkRequest.walkParams.translation.y);

        if ((fabs(toDegrees(angleDifference)) < angleRemain) && (distanceToDestination < distanceRemain))
        {
            actionRequest.walkRequest.walkParams.x() = 0.0;
            actionRequest.walkRequest.walkParams.y() = 0.0;
            actionRequest.walkRequest.rotation = 0.0;
        }
    }
    else	//only to be sure the robot won't move if desired position is reached
    {
        actionRequest.walkRequest.walkParams.x() = 0.0;
        actionRequest.walkRequest.walkParams.y() = 0.0;
        actionRequest.walkRequest.rotation = 0.0;
    }*/
    //aLOG << "Rotation :" << actionRequest.walkRequest.rotation << endl;
}

void NaoBehaviorControl::AdjustmentForKick( float AdjustX,float AdjustY)
{

    actionRequest.actionType = ActionRequest::walk;
    actionRequest.walkRequest.walkParams.x() = AdjustX;
    actionRequest.walkRequest.walkParams.y() =-AdjustY;

    if (gAbs(AdjustX) < 0.1)
    {
        actionRequest.walkRequest.walkParams.x() = gMin(actionRequest.walkRequest.walkParams.x(),0.01f);
        actionRequest.walkRequest.walkParams.x() = gMax(actionRequest.walkRequest.walkParams.x(),-0.01f);

        actionRequest.walkRequest.walkParams.y() = gMin(actionRequest.walkRequest.walkParams.y(),0.00f);
        actionRequest.walkRequest.walkParams.y() = gMax(actionRequest.walkRequest.walkParams.y(),-0.00f);
    }
    else
    {
        actionRequest.walkRequest.walkParams.x() = gMin(actionRequest.walkRequest.walkParams.x(),0.00f);
        actionRequest.walkRequest.walkParams.x() = gMax(actionRequest.walkRequest.walkParams.x(),-0.00f);

        actionRequest.walkRequest.walkParams.y() = gMin(actionRequest.walkRequest.walkParams.y(),0.01f);
        actionRequest.walkRequest.walkParams.y() = gMax(actionRequest.walkRequest.walkParams.y(),-0.01f);
    }
}

void NaoBehaviorControl::Stop()
{
    /*accelerationRestrictor.saveLastWalkParameters();*/

    actionRequest.walkRequest.walkParams.x() = 0.0;
    actionRequest.walkRequest.walkParams.y() = 0.0;
    actionRequest.walkRequest.rotation = 0.0;

    actionRequest.actionType = ActionRequest::stop;
}

void NaoBehaviorControl::Stand()
{
    actionRequest.actionType = ActionRequest::stand;
}

void NaoBehaviorControl::TurnTo(double angleToDestination)
{
    double maxTurnSpeed = 7.0;
    if ( angleToDestination > 0)
    {
        actionRequest.walkRequest.walkParams.x() = 0.0f;
        actionRequest.walkRequest.walkParams.y() = -0.01f;
    }
    else
        actionRequest.walkRequest.walkParams.x() = 0.0f;
    actionRequest.walkRequest.walkParams.y() = 0.01f;

    actionRequest.walkRequest.rotation = 2*angleToDestination;
    // clip rotation speed
    actionRequest.walkRequest.rotation = gMin(actionRequest.walkRequest.rotation, maxTurnSpeed);
    actionRequest.walkRequest.rotation = gMax(actionRequest.walkRequest.rotation, -maxTurnSpeed);
}

bool NaoBehaviorControl::ArriveKickPosition()
{

    float AdjustY=WM.getBall().GetLocalPos().y()-WM.getSelf().GetLeftFootLocalPosition().y();
    float AdjustX=(WM.getBall().GetLocalPos().x()-0.01)-WM.getSelf().GetLeftFootLocalPosition().x();
    switch (adjustment)
    {
    case adjustX:
        actionRequest.walkRequest.walkParams.x() = gMin(actionRequest.walkRequest.walkParams.x(),0.005f);
        actionRequest.walkRequest.walkParams.x() = gMax(actionRequest.walkRequest.walkParams.x(),-0.005f);

        actionRequest.walkRequest.walkParams.y() = gMin(actionRequest.walkRequest.walkParams.y(),0.00f);
        actionRequest.walkRequest.walkParams.y() = gMax(actionRequest.walkRequest.walkParams.y(),-0.00f);
        if (  gAbs( AdjustX) < 0.01)
        {
            adjustment = adjustY;
            return true;
        }
        break;
    case adjustY:
        actionRequest.walkRequest.walkParams.x() = gMin(actionRequest.walkRequest.walkParams.x(),0.00f);
        actionRequest.walkRequest.walkParams.x() = gMax(actionRequest.walkRequest.walkParams.x(),-0.00f);

        actionRequest.walkRequest.walkParams.y() = gMin(actionRequest.walkRequest.walkParams.y(),0.005f);
        actionRequest.walkRequest.walkParams.y() = gMax(actionRequest.walkRequest.walkParams.y(),-0.005f);
        if (  gAbs( AdjustY) < 0.01)
        {
            adjustment = adjustX;
            return true;
        }
        break;
    default:
        break;
    }
}

void NaoBehaviorControl::execute()
{
    Vector2f Destination2D(WM.getBall().GetLocalPos().x(), WM.getBall().GetLocalPos().y());

    double angleToDestination = gRadToDeg(gArcTan2(WM.getBall().GetLocalPos().y(),WM.getBall().GetLocalPos().x()));

    double distanceToDestination = Destination2D.Length();

    if (distanceToDestination > 0.5)
        GoToRelativePoint(WM.getBall().GetLocalPos());
    else if ( ArriveKickPosition())
    {
        Stop();
        if (actionRequest.actionType = ActionRequest::stop)
            actionRequest.actionType = ActionRequest::kick;
    }

//     actionRequest.actionType = ActionRequest::specialAction;
    /**switch (behaviorControlState)
    {
    case handle_ball:
        break;
    case get_to_ball:
        break;
    case walk_to_position_behind_the_ball:
        break;
    case kick:
    //      if ()
            break;
    case dribble:
        break;
    }*/

	
    //Vector2f detc(WM.getBall().pos.x(),WM.getBall().pos.y());
    //GoToBall();
    //GoToPoint(detc);
    //TurnTo(90.0);

    //Stop();

    //GoToBallByCommunication();
    //GoToPoint();
    sendMsg2GuiDebugger();
    if (WM.getSelf().onMyback()||WM.getSelf().onMyBelly()||WM.getSelf().onMySide())
    {
        actionRequest.actionType = ActionRequest::getup;
    }
}

void NaoBehaviorControl::sendMsg2GuiDebugger()
{
    AA.autoSendMsg = false;
    //DEBUG<<"WalkPathPlanningControl::sendMsg2GuiDebugger"<<endl;
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
        //teammate position
        for (int i=0; i<MAX_TEAM_SIZE; i++)
        {
            if (WM.getTmm(i).isValid())
            {
                o_msg<<"<p>";
                o_msg<<"<t>Strive3D</t>";
                o_msg<<"<u>"<<WM.getTmm(i).GetUnum()<<"</u>";
                o_msg<<"<h>"<<WM.getTmm(i).GetHeadGloblePosition()<<"</h>";
                //o_msg<<"<h>"<<WM.getTmm(i).GetHeadGloblePosition()<<"</h>";
                o_msg<<"<lla>"<<WM.getTmm(i).GetLeftArmGloblePosition()<<"</lla>";
                o_msg<<"<rla>"<<WM.getTmm(i).GetRightArmGloblePosition()<<"</rla>";
                o_msg<<"<lf>"<<WM.getTmm(i).GetLeftFootGloblePosition()<<"</lf>";
                o_msg<<"<rf>"<<WM.getTmm(i).GetRightFootGloblePosition()<<"</rf>";
                o_msg<<"</p>";
            }
        }
        //opponent position
        for (int i=0; i<MAX_TEAM_SIZE; i++)
        {
            if (WM.getOpp(i).isValid())
            {
                o_msg<<"<p>";
                o_msg<<"<t>Oponent</t>";
                o_msg<<"<u>"<<WM.getOpp(i).GetUnum()<<"</u>";
                o_msg<<"<h>"<<WM.getOpp(i).GetHeadGloblePosition()<<"</h>";
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
        //convex hull
        o_msg<<"<chm>";
        ConvexHullManager& chm = WM.getConvexHullManager();
        vector< vector<Vector3f> >& chPosVect = chm.GetConvexHull();
        for (int i=0; i<chPosVect.size(); i++)
        {
            vector<Vector3f>& posVect = chPosVect[i];
            o_msg<<"<ch>";
            for (int j=0; j<posVect.size(); j++)
                o_msg<<"<pos>"<<posVect[j]<<"</pos>";
            o_msg<<"</ch>";
        }

        ConvexHullManager& safechm = WM.getSafeConvexHullManager();
        vector< vector<Vector3f> >& safechPosVect = safechm.GetConvexHull();
        for (int i=0; i<safechPosVect.size(); i++)
        {
            vector<Vector3f>& posVect = safechPosVect[i];
            o_msg<<"<ch>";
            for (int j=0; j<posVect.size(); j++)
                o_msg<<"<pos>"<<posVect[j]<<"</pos>";
            o_msg<<"</ch>";
        }
        o_msg<<"</chm>";

        o_msg<<"</av>";
        o_msg<<"<ad>";
        o_msg<<"<x>"<<actionRequest.walkRequest.walkParams.x()<<"</x>";
        o_msg<<"<y>"<<actionRequest.walkRequest.walkParams.y()<<"</y>";
        o_msg<<"<r>"<<actionRequest.walkRequest.rotation<<"</r>";
        //if (mDebugMsgParser.cmdType() == CT_Destination)// && mPathPlanningTest.finalWayPoints().size() > 0)
        //{
        o_msg<<"<wp>";
        //vector<Vector2f>& finalWayPoints = mPathPlanningTest.finalWayPoints();
        //for (int i=0; i<finalWayPoints.size(); i++)
        //{
        //o_msg<<"<pos>"<<finalWayPoints[i]<<"</pos>";
        //}
        for (int i=0; i<mTmpPosVect.size(); i++)
            o_msg<<"<pos>"<<mTmpPosVect[i]<<"</pos>";
        o_msg<<"</wp>";
        //}
        o_msg<<"</ad>";
        o_msg<<"</r>";
        //cout<<"AdvancedAgent::sendInfo2GuiDebugger-->o_msg="<<o_msg.str()<<endl;
        DEBUG<<"WalkPathPlanningControl::sendMsg2GuiDebugger-->o_msg="<<o_msg.str()<<endl;
        DEBUG_NET.PutMessage(o_msg.str());
    }
}
