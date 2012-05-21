/*
Bullet Continuous Collision Detection and Physics Library
GenericJointDemo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#ifndef GENERIGJOINTDEMO_H
#define GENERIGJOINTDEMO_H

#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication

#include <zmq.hpp>

#include <iostream>

using namespace std;

#include "LinearMath/btAlignedObjectArray.h"
#include "Hexapod.h"

#define SETBIT(x, b)   ((x) |= (b))
#define CLEARBIT(x, b) ((x) &= ~(b))
#define TESTBIT(x, b)  ((x) & (b))

#define SIMPAUSE       0
#define SIMRESET       1
#define SIMCONTINUE    2
#define SIMSTART       3


typedef unsigned int HpodSimCtrl;

void debugSimCtrl(HpodSimCtrl);


class HexapodSimulationDemo : public PlatformDemoApplication
{

	btAlignedObjectArray<class Hexapod*> m_hexapods;
    btAlignedObjectArray<class Leg*> m_legs;
    
	float m_Time;
	float m_fCyclePeriod; // in milliseconds
	float m_fMuscleStrength;
    
public:
    
    HexapodSimulationDemo();
    ~HexapodSimulationDemo();
    
	void initPhysics();
    void setMotorTargets(btScalar deltaTime);

	void spawnHexapod(bool random = false);

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);
    
   	void setMotorTargets(btVector3 transition);
    
    
private:
    
    zmq::context_t zcontext;
    zmq::socket_t  zsocket;
    
    
};

void motorPreTickCallback (btDynamicsWorld *world, btScalar timeStep);

#endif
