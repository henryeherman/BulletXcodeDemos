/*
Hexapod Demo
Copyright (c) 2012 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Henry Herman and Jason Tsao
*/


// atof include
#include <stdlib.h>

#include "BulletDynamics/btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"
#include <zmq.hpp>



#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "HexapodSimulation.h"


// Debug
#include <iostream>
#include <sstream>
#include <unistd.h>
using namespace std;


GLDebugDrawer debugDrawer;


void debugSimCtrl(const HpodSimCtrl ctrl) {
    cout << "DEBUG:SIMSTAT:";
    switch (ctrl) {
        case SIMPAUSE:
            cout << "PAUSECMD";
            break;
        case SIMRESET:
            cout << "RESETCMD";
            break;
        case SIMCONTINUE:
            cout << "CONTCMD";
            break;
        case SIMSTART:
            cout << "STARTCMD";
            break;
        default:
            cout << "UNKNCMD";
            break;
    }
    cout << endl;
}


HexapodSimulationDemo::HexapodSimulationDemo(): zcontext(1), zsocket(zcontext, ZMQ_REP) {    
    zsocket.bind("tcp://*:5555");
    state = RUN;
}

HexapodSimulationDemo::~HexapodSimulationDemo() {
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
	m_fMuscleStrength = 3.8f;
    
    
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
	Hexapod* hexapod = new Hexapod (m_dynamicsWorld, btVector3 (0,2,1),5.f);
	m_hexapods.push_back(hexapod);

    
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

void HexapodSimulationDemo::processCommand(HpodSimCtrl cmd,HpodCtrlParams *params,unsigned long size) {
    switch (cmd) {
        case SIMPAUSE:
#ifdef DEBUG_CMD
            cout << "CMD:PAUSE"<<endl;
#endif
            state = PAUSE;
            setIdle(true);
            break;
        case SIMCONTINUE:
#ifdef DEBUG_CMD
            cout << "CMD:CONT"<<endl;
#endif
            setIdle(false);
            state=RUN;
            break;
        case SIMSTART:
#ifdef DEBUG_CMD
            cout << "CMD:START"<<endl;
#endif
            clientResetScene();
            setIdle(false);
            state=RUN;
            break;
        case SIMRESET:
#ifdef DEBUG_CMD
            cout << "CMD:RESET"<<endl;
#endif
            clientResetScene();
            setIdle(true);
            state=RUN;
            break;
        case SIMLOAD:
#ifdef DEBUG_CMD
            cout << "CMD:LOAD"<<endl;
#endif
            clientResetScene();
            setIdle(true);
            for (int r=0; r<m_hexapods.size(); r++)
            {
                Hexapod *hpod = m_hexapods[r];
                hpod->clearCtrlParams();
                hpod->loadCtrlParams(params, size);
            }
            state = PAUSE;
        case SIMLOADIMM:
#ifdef DEBUG_CMD
            cout << "CMD:LOADIMM"<<endl;
#endif
            for (int r=0; r<m_hexapods.size(); r++)
            {
                Hexapod *hpod = m_hexapods[r];
                hpod->setCtrlParams(*params);            
            }
            break;
        case SIMRUNEXP:
#ifdef DEBUG_CMD
            cout << "CMD:RUNEXP"<<endl;
#endif
            setIdle(false);
            for (int r=0; r<m_hexapods.size(); r++)
            {
                Hexapod *hpod = m_hexapods[r];
                hpod->step();            
            }
            state=RUN;
            break;
        case SIMRESETEXP:
#ifdef DEBUG_CMD
            cout << "CMD:RESEREXP"<<endl;
#endif
            setIdle(true);
            clientResetScene();
            for (int r=0; r<m_hexapods.size(); r++)
            {
                Hexapod *hpod = m_hexapods[r];
                hpod->reset();            
            }
        default:
            break;
    }
}


void HexapodSimulationDemo::setMotorTargets(btScalar deltaTime)
{
    
    HpodSimCtrl ctrlParam;
    HpodCtrlParams *params;
    
    zmq::message_t request;
    request.rebuild();
    bool result = false;
    try {
        result = zsocket.recv(&request, ZMQ_NOBLOCK);
    } catch (zmq::error_t e) {
        cout << "ZMQ Pkt #1:" << e.what() << endl;
    }
    
    if(result) {
        ctrlParam = *((HpodSimCtrl *) request.data());

#ifdef DEBUG_SIM_CTRL_PARAM
        debugSimCtrl(ctrlParam);
#endif
        
        request.rebuild();
        try {
            zsocket.recv(&request);
        } catch (zmq::error_t e) {
            cout << "ZMQ Pkt #2:" << e.what() << endl;
        }
        
        params = (HpodCtrlParams *) request.data();
        
        unsigned long numpkts= request.size()/sizeof(HpodCtrlParams);
#ifdef DEBUG_ZMQ_COM
        cout << "Num Pkts:" << numpkts << endl;
#endif
        
#ifdef DEBUG_HPOD_CTRL_PARAMS
        debugCtrlParams(*params);
#endif
        
        // Send reply back
        try
        {
         zmq::message_t reply(3);
         memcpy((void *) reply.data (), "ACK", 3);
         zsocket.send(reply);
        }
        catch (zmq::error_t e)
        {
            cout << "ZMQ Reply 2:" << e.what() << endl;
        }
    
        
        processCommand(ctrlParam, params, numpkts);
        
        
        /*
        for (int r=0; r<m_hexapods.size(); r++)
        {
            
            Hexapod *hpod = m_hexapods[r];
            hpod->clearCtrlParams();
            hpod->loadCtrlParams(params, numpkts);
            //hpod->setCtrlParams(*params);
        }
        */
    
        
    } else {
        // continue
    }
	
}


void motorPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
	HexapodSimulationDemo* hexapodDemo = (HexapodSimulationDemo*)world->getWorldUserInfo();
    
    //std::cout << "Hello Tick" << std::endl;
	hexapodDemo->setMotorTargets(timeStep);
	
}


//################## END HEXAPOD ######################//




