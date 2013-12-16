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
#ifndef COMMONBASICBEHAVIORS_H
#define COMMONBASICBEHAVIORS_H

#include "BehaviorControl.h"
#include "XabslBasicBehavior.h"
#include "XabslEngineExecutor.h"

using namespace xabsl;

class BasicBehaviorStop : public BasicBehavior, public BehaviorControlInterfaces
{
public:

    BasicBehaviorStop(const BehaviorControlInterfaces& interfaces,
                      MyXabslErrorHandler& errorHandler)
            : BasicBehavior("stop", errorHandler),
            BehaviorControlInterfaces(interfaces)
    {}

    /**  Executes the basic behavior. */
    virtual void execute();
};

class BasicBehaviorDoNothing : public BasicBehavior, public BehaviorControlInterfaces
{
public:

    BasicBehaviorDoNothing(const BehaviorControlInterfaces& interfaces,
                           MyXabslErrorHandler& errorHandler)
            : BasicBehavior("do-nothing", errorHandler),
            BehaviorControlInterfaces(interfaces)
    {}

    /**  Executes the basic behavior. */
    virtual void execute();
};

class BasicBehaviorWalk : public BasicBehavior, public BehaviorControlInterfaces
{
public:

    BasicBehaviorWalk(const BehaviorControlInterfaces& interfaces,
                      MyXabslErrorHandler& errorHandler)
            : BasicBehavior("walk", errorHandler),
            BehaviorControlInterfaces(interfaces)
    {
    }

    /**  Executes the basic behavior. */
    virtual void execute();

    virtual void registerParameters()
    {
        parameters->registerDecimal("walk.type", type);
        parameters->registerDecimal("walk.speed-x", speedX);
        parameters->registerDecimal("walk.speed-y", speedY);
        parameters->registerDecimal("walk.rotation-speed", rotationSpeed);
    }


private:
    /** Parameter "walk.type" */
    double type;

    /** Parameter "walk.speed-x" */
    double speedX;

    /** Parameter "walk.speed-y" */
    double speedY;

    /** Parameter "walk.rotation-speed" */
    double rotationSpeed;
};


class BasicBehaviorSpecialAction : public BasicBehavior, public BehaviorControlInterfaces
{
public:

    BasicBehaviorSpecialAction(const BehaviorControlInterfaces& interfaces,
                               MyXabslErrorHandler& errorHandler)
            : BasicBehavior("special-action", errorHandler),
            BehaviorControlInterfaces(interfaces)
    {
    }

    /**  Executes the basic behavior. */
    virtual void execute();

    virtual void registerParameters()
    {
	 parameters->registerDecimal("special-action-id", specialActionID);
    }

private:
    /** Parameter "special-action-id" */
    double specialActionID;
};



class BasicBehaviorStand : public BasicBehavior, public BehaviorControlInterfaces
{
public:
    /*
    * Constructor.
    * @param errorHandler Is invoked when errors occur
    * @param interfaces The paramters of the BehaviorControl module.
      */
    BasicBehaviorStand(const BehaviorControlInterfaces& interfaces,
                       MyXabslErrorHandler& errorHandler)
            : BasicBehavior("stand", errorHandler),
            BehaviorControlInterfaces(interfaces)
            /**accelerationRestrictor(actionRequest)*/
    {
    }

    /**  Executes the basic behavior. */
    virtual void execute();

    virtual void registerParameters(){}

private:
    /** Restricts the walk acceleration to maximum values */
    //WalkAccelerationRestrictor accelerationRestrictor;
};


/**
* Creates and registers simple basic behaviors
*/
class CommonBasicBehaviors : public BehaviorControlInterfaces
{
public:

    CommonBasicBehaviors(const BehaviorControlInterfaces& interfaces,
                         MyXabslErrorHandler& errorHandler)
            : BehaviorControlInterfaces(interfaces),
            errorHandler(errorHandler),
            basicBehaviorDoNothing(interfaces,errorHandler),
            basicBehaviorStop(interfaces,errorHandler),
            basicBehaviorSpecialAction(interfaces,errorHandler),
            basicBehaviorStand(interfaces,errorHandler),
            basicBehaviorWalk(interfaces,errorHandler)
    {}

    /** Registers basic behaviors at the engine */
    void registerBasicBehaviors(Engine& engine);

private:
    /** Is invoked when errors occurs */
    MyXabslErrorHandler& errorHandler;

    BasicBehaviorDoNothing  basicBehaviorDoNothing;
    BasicBehaviorStop  basicBehaviorStop;
    BasicBehaviorSpecialAction  basicBehaviorSpecialAction;
    BasicBehaviorStand  basicBehaviorStand;
    BasicBehaviorWalk  basicBehaviorWalk;
};

#endif
