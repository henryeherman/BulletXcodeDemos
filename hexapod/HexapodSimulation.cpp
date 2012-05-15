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
#include <zmq.hpp>


#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "HexapodSimulation.h"


// Debug
#include <iostream>
using namespace std;



GLDebugDrawer debugDrawer;


HexapodSimulationDemo::HexapodSimulationDemo() {
    // Initialize mutex and create pthread
    pthread_mutex_init(&m_mutex, NULL);
    start_zmq_thread();

}

HexapodSimulationDemo::~HexapodSimulationDemo() {
    pthread_mutex_destroy(&m_mutex);
}

void HexapodSimulationDemo::start_zmq_thread() {
    m_running = true;
    pthread_create(&m_thread, NULL, &HexapodSimulationDemo::run_zmq_thread, this);
    
//    pthread_create(&m_thread, NULL, &HexapodSimulationDemo::run_zmq_thread, this);
}



void *HexapodSimulationDemo::run_zmq_thread(void *obj) {
    // Setup/Bind to Zeromq socket
    zmq::context_t context (1);
    zmq::socket_t socket (context, ZMQ_REP);
    socket.bind ("tcp://*:5555");
    
    while(1) {
        zmq::message_t request;
        
        // Wait for next request 
    }

}


void HexapodSimulationDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

    
    
	m_Time = 0;
	m_fCyclePeriod = 1000.f; // in milliseconds
    
    //	m_fMuscleStrength = 0.05f;
	// new SIMD solver for joints clips accumulated impulse, so the new limits for the motor
	// should be (numberOfsolverIterations * oldLimits)
	// currently solver uses 10 iterations, so:
	m_fMuscleStrength = 4.5f;
    
    
	// Setup the basic world

	btDefaultCollisionConfiguration * collision_config = new btDefaultCollisionConfiguration();

	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collision_config);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	btBroadphaseInterface* overlappingPairCache = new btAxisSweep3 (worldAabbMin, worldAabbMax);

	btConstraintSolver* constraintSolver = new btSequentialImpulseConstraintSolver;


	m_dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,constraintSolver,collision_config);
    
	m_dynamicsWorld->setInternalTickCallback(motorPreTickCallback,this,true);
    
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

void HexapodSimulationDemo::spawnHexapod(bool random)
{
	Hexapod* hexapod = new Hexapod (m_dynamicsWorld, btVector3 (0,10,0),5.f);
	m_hexapods.push_back(hexapod);
    
    //Leg* leg = new Leg (m_dynamicsWorld, btVector3 (0,0,5),5.f);
    //delete leg;
    //m_legs.push_back(leg);
    
}



void HexapodSimulationDemo::setMotorTargets(btVector3 translation)
{
    
    // Animate the bodies
//    btVector3 kinTranslation(-0.01,-1,0);
    int collision_array_size = m_dynamicsWorld->getNumCollisionObjects();
    cout << "Collision array size: " << collision_array_size << endl;
    if(collision_array_size > 1) {
        for(int i = 1; i < collision_array_size; i++) {
            btCollisionObject *colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
            if(btRigidBody::upcast(colObj) && btRigidBody::upcast(colObj)->getMotionState())
            {
                btTransform newTrans;
                btRigidBody::upcast(colObj)->getMotionState()->getWorldTransform(newTrans);
                newTrans.getOrigin()+= translation;
                btRigidBody::upcast(colObj)->getMotionState()->setWorldTransform(newTrans);
            }
            else
            {
                m_dynamicsWorld->getCollisionObjectArray()[0]->getWorldTransform().getOrigin() += translation;
            }
            
        //	cout << "DeltaTime: " << deltaTime << endl;
        }
    }
}

void HexapodSimulationDemo::clientMoveAndDisplay()
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

void HexapodSimulationDemo::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	renderme();

	glFlush();
	swapBuffers();
}

void HexapodSimulationDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
        case 'e':
            spawnHexapod(true);
            break;
        
        case 'j':
            setMotorTargets(btVector3(0, -1, 0));
            break;
        case 'k':
            setMotorTargets(btVector3(0, 1,0));
            break;   
        case 'l':
            setMotorTargets(btVector3(-1, 0, 0));
            break;
        case 'h':
            setMotorTargets(btVector3(1, 0, 0));
            break;
        
        default:
            DemoApplication::keyboardCallback(key, x, y);
	}


}



void HexapodSimulationDemo::setMotorTargets(btScalar deltaTime)
{
    
	float ms = deltaTime*1000000.;
	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;
    
	m_Time += ms;
    
	//
	// set per-frame sinusoidal position targets using angular motor (hacky?)
	//	
	for (int r=0; r<m_hexapods.size(); r++)
	{
		for (int i=0; i<m_hexapods[0]->m_legs.size(); i++)
		{
			
            Hexapod* hpod;
            Leg * leg;
            hpod = m_hexapods[r];
            
            
            leg = hpod->m_legs[i];
            
			btScalar fTargetPercent = (int(m_Time / 1000) % int(m_fCyclePeriod)) / m_fCyclePeriod;
			btScalar fTargetAngle   = 1.0 * (1 + sin(2 * M_PI * fTargetPercent));
            leg->setKneeMaxStrength(m_fMuscleStrength);
            leg->setKneeTarget(fTargetAngle, 0.01);
            leg->setHipMaxStrength(m_fMuscleStrength);
            leg->setHipTarget(M_PI_2-fTargetAngle,0, 0.01);
            
		}
	}
    
	
}


void motorPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
	HexapodSimulationDemo* hexapodDemo = (HexapodSimulationDemo*)world->getWorldUserInfo();
    
    std::cout << "Hello Tick" << std::endl;
	hexapodDemo->setMotorTargets(timeStep);
	
}


//################## END HEXAPOD ######################//




