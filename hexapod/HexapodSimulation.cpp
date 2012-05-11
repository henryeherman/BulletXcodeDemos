/*
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Originally Written by: Marten Svanfeldt
ReWritten by: Francisco León
*/




#include "BulletDynamics/btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"


#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "HexapodSimulation.h"



GLDebugDrawer debugDrawer;


#include <iostream>
using namespace std;



void GenericJointDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	// Setup the basic world

	btDefaultCollisionConfiguration * collision_config = new btDefaultCollisionConfiguration();

	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collision_config);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	btBroadphaseInterface* overlappingPairCache = new btAxisSweep3 (worldAabbMin, worldAabbMax);

	btConstraintSolver* constraintSolver = new btSequentialImpulseConstraintSolver;


	m_dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,constraintSolver,collision_config);

	m_dynamicsWorld->setGravity(btVector3(0,-30,0));

	m_dynamicsWorld->setDebugDrawer(&debugDrawer);

	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-15,0));
		localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
	}

	// Spawn one ragdoll
	spawnHexapod();

	clientResetScene();
}

void GenericJointDemo::spawnHexapod(bool random)
{
	Hexapod* hexapod = new Hexapod (m_dynamicsWorld, btVector3 (0,10,0),5.f);
	m_hexapods.push_back(hexapod);
    
    //Leg* leg = new Leg (m_dynamicsWorld, btVector3 (0,0,5),5.f);
    //delete leg;
    //m_legs.push_back(leg);
    
}

void GenericJointDemo::setMotorTargets(btScalar deltaTime)
{
    
//	float ms = deltaTime*1000000.;
//	float minFPS = 1000000.f/60.f;
//	if (ms > minFPS)
//		ms = minFPS;
//    
//	m_Time += ms;
//    
//	//
//	// set per-frame sinusoidal position targets using angular motor (hacky?)
//	//	
//	for (int r=0; r<m_rigs.size(); r++)
//	{
//		for (int i=0; i<2*NUM_LEGS; i++)
//		{
//			btHingeConstraint* hingeC = static_cast<btHingeConstraint*>(m_rigs[r]->GetJoints()[i]);
//			btScalar fCurAngle      = hingeC->getHingeAngle();
//			
//			btScalar fTargetPercent = (int(m_Time / 1000) % int(m_fCyclePeriod)) / m_fCyclePeriod;
//			btScalar fTargetAngle   = 0.5 * (1 + sin(2 * M_PI * fTargetPercent));
//			btScalar fTargetLimitAngle = hingeC->getLowerLimit() + fTargetAngle * (hingeC->getUpperLimit() - hingeC->getLowerLimit());
//			btScalar fAngleError  = fTargetLimitAngle - fCurAngle;
//			btScalar fDesiredAngularVel = 1000000.f * fAngleError/ms;
//			hingeC->enableAngularMotor(true, fDesiredAngularVel, m_fMuscleStrength);
//		}
//	}
    
	
}

void GenericJointDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}

	renderme();

	glFlush();

	swapBuffers();
}

void GenericJointDemo::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	renderme();

	glFlush();
	swapBuffers();
}

void GenericJointDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'e':
		spawnHexapod(true);
		break;
        
    case 'j':
        cout << "J KEY WAS PRESSED";
        break;
    case 'k':
        cout << "K KEY WAS PRESSED";
        break;    
        
    default:
		DemoApplication::keyboardCallback(key, x, y);
	}


}
