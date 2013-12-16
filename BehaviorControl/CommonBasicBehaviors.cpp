/***************************************************************************
 *   Copyright (C) 2008 by Zhu_Ming,Zheng Yonglei   *
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
#include "CommonBasicBehaviors.h"

void CommonBasicBehaviors::registerBasicBehaviors(Engine& engine)
{
    engine.registerBasicBehavior(basicBehaviorDoNothing);
    engine.registerBasicBehavior(basicBehaviorStop);
    engine.registerBasicBehavior(basicBehaviorSpecialAction);
    engine.registerBasicBehavior(basicBehaviorStand);
    engine.registerBasicBehavior(basicBehaviorWalk);
}

void BasicBehaviorStop::execute()
{    /*accelerationRestrictor.saveLastWalkParameters();*/

    actionRequest.walkRequest.walkParams.x() = 0.0;
    actionRequest.walkRequest.walkParams.y() = 0.0;
    actionRequest.walkRequest.rotation = 0.0;

    actionRequest.actionType = ActionRequest::stop;
}

void BasicBehaviorDoNothing::execute()
{
    // do nothing;
}

void BasicBehaviorWalk::execute()
{
    actionRequest.actionType = ActionRequest::walk;
    actionRequest.walkRequest.walkType   = static_cast<WalkRequest::WalkType>(static_cast<int>(type));
    if (actionRequest.walkRequest.walkType >= WalkRequest::numOfWalkType || actionRequest.walkRequest.walkType < 0)
        actionRequest.walkRequest.walkType = WalkRequest::normal;
    actionRequest.walkRequest.walkParams.x() = speedX;
    actionRequest.walkRequest.walkParams.y() = speedY;
    actionRequest.walkRequest.rotation = rotationSpeed;
}

void BasicBehaviorSpecialAction::execute()
{
    /**actionRequest.actionType = ActionRequest::specialAction;
    actionRequest.specialActionRequest.specialActionType = static_cast<SpecialActionRequest::SpecialActionID>(static_cast<int>(specialActionID));*/
}

void BasicBehaviorStand::execute()
{
    actionRequest.actionType = ActionRequest::stand;
}
