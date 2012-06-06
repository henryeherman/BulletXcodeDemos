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
    isDirty=false;
    isRunningExp=false;
}

HexapodSimulationDemo::~HexapodSimulationDemo() {
}

void HexapodSimulationDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);
    newCamPos.setZero();
    followHexapod = true;
    
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

	// Spawn one Hexapod
	spawnHexapod();

	clientResetScene();
}

void HexapodSimulationDemo::spawnHexapod(bool random)
{
    unsigned int podid =m_hexapods.size();
	Hexapod* hexapod = new Hexapod (m_dynamicsWorld, btVector3 (0,2,1),5.f, podid);
	m_hexapods.push_back(hexapod);    
}

void HexapodSimulationDemo::zmqRecv() {
    
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
        
        param_count= (unsigned int)request.size()/sizeof(HpodCtrlParams);
        
        
        
#ifdef DEBUG_ZMQ_COM
        cout << "Num Pkts:" << param_count << endl;
#endif
        
#ifdef DEBUG_HPOD_CTRL_PARAMS
        debugCtrlParams(*params);
#endif

#ifdef DEBUG_CMD
        cout << "CMD:" << ctrlParam <<endl;
#endif
        
        // Send reply back
        try
        {
            zmq::message_t reply;
            if (ctrlParam==SIMCHKBUSY) {
                if(isBusySim()) {
                    reply.rebuild(4);
                    memcpy((void *) reply.data (), "YES", 4);
                    zsocket.send(reply);
                    cout << "Busy" << endl;
                } else {
                    reply.rebuild(3);
                    memcpy((void *) reply.data (), "NO", 3);
                    zsocket.send(reply);
                    cout << "Not busy" << endl;
                }
            } else if (ctrlParam==SIMGETREPLY) {
                static bool isMulti = false;
                for (int r=0; r<m_hexapods.size(); r++) {
                    Hexapod *pod = m_hexapods[r];
                    for(int q=0;q<pod->m_replys.size();q++) {
                        isMulti = true;
                        HpodReply podreply = pod->m_replys[q];
                        reply.rebuild(sizeof(HpodReply));
                        memcpy((void *) reply.data (), &podreply, sizeof(HpodReply));
                        zsocket.send(reply, ZMQ_SNDMORE);
                    }
                }	
                
                reply.rebuild(5);
                memcpy((void *)reply.data (), "DONE", 5);
                
                if (isMulti) {
                    //cout << "Multi Reply" << endl;
                    zsocket.send(reply, 0);
                } else {
                    //cout << "Single Reply" << endl;
                    zsocket.send(reply);
                }
                isMulti = false;
                
                
            } else {
                reply.rebuild(4);
                memcpy((void *) reply.data (), "ACK", 4);
                zsocket.send(reply);
            }
            
        }
        catch (zmq::error_t e)
        {
            cout << "ZMQ Reply 1:" << e.what() << endl;
        }
        processCommand(ctrlParam, params, param_count);
    }
    
}


void HexapodSimulationDemo::setMotorTargets(btVector3 translation)
{
    
    // Animate the bodies
//    btVector3 kinTranslation(-0.01,-1,0);
    int collision_array_size = m_dynamicsWorld->getNumCollisionObjects();
    //cout << "Collision array size: " << collision_array_size << endl;
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
    if (!isDirty) {
        zmqRecv();
    }
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
    
    if (followHexapod) {
        if (m_hexapods.size() > 0) {
            btVector3 pos =  m_hexapods[0]->getPosition(); 
            m_cameraTargetPosition.setX(pos.x());
            m_cameraTargetPosition.setZ(pos.z());
            updateCamera();
        }
    }
	renderme();

	glFlush();

	swapBuffers();
}

void HexapodSimulationDemo::displayCallback()
{
    if (!isDirty) {
        zmqRecv();
    }
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
        
        case 'F':
            followHexapod = !followHexapod;
            if (!followHexapod) {
                m_cameraTargetPosition.setValue(0, 4, -4);
            }
            break;
            
        default:
            DemoApplication::keyboardCallback(key, x, y);
	}


}


void HexapodSimulationDemo::cameraFollow() {
        
        
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        btScalar rele = m_ele * btScalar(0.01745329251994329547);// rads per deg
        btScalar razi = m_azi * btScalar(0.01745329251994329547);// rads per deg
        
        
        btQuaternion rot(m_cameraUp,razi);
        
        eyePos[m_forwardAxis] = -m_cameraDistance;
        
        btVector3 forward(eyePos[0],eyePos[1],eyePos[2]);
        if (forward.length2() < SIMD_EPSILON)
        {
            forward.setValue(1.f,0.f,0.f);
        }
        btVector3 right = m_cameraUp.cross(forward);
        btQuaternion roll(right,-rele);
        
        eyePos = btMatrix3x3(rot) * btMatrix3x3(roll) * eyePos;
        
        m_cameraPosition[0] = eyePos.getX();
        m_cameraPosition[1] = eyePos.getY();
        m_cameraPosition[2] = eyePos.getZ();
        m_cameraPosition += m_cameraTargetPosition;
        
        if (m_glutScreenWidth == 0 && m_glutScreenHeight == 0)
            return;
        
        btScalar aspect;
        btVector3 extents;
        
        if (m_glutScreenWidth > m_glutScreenHeight) 
        {
            aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;
            extents.setValue(aspect * 1.0f, 1.0f,0);
        } else 
        {
            aspect = m_glutScreenHeight / (btScalar)m_glutScreenWidth;
            extents.setValue(1.0f, aspect*1.f,0);
        }
        
        
        if (m_ortho)
        {
            // reset matrix
            glLoadIdentity();
            
            
            extents *= m_cameraDistance;
            btVector3 lower = m_cameraTargetPosition - extents;
            btVector3 upper = m_cameraTargetPosition + extents;
            //gluOrtho2D(lower.x, upper.x, lower.y, upper.y);
            glOrtho(lower.getX(), upper.getX(), lower.getY(), upper.getY(),-1000,1000);
            
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            //glTranslatef(100,210,0);
        } else
        {
            if (m_glutScreenWidth > m_glutScreenHeight) 
            {
                glFrustum (-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);
            } else 
            {
                glFrustum (-1.0, 1.0, -aspect, aspect, 1.0, 10000.0);
            }
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2], 
                      m_cameraTargetPosition[0], m_cameraTargetPosition[1], m_cameraTargetPosition[2], 
                      m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());
        }
        
}

void HexapodSimulationDemo::processCommand(HpodSimCtrl cmd,HpodCtrlParams *params,unsigned long size) {
    isDirty=true;
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
            if(state==START)
                state=RUN;
            else {
                clientResetScene();
                state=START;
            }
            setIdle(false);
            break;
            
        case SIMRESET:
#ifdef DEBUG_CMD
            cout << "CMD:RESET"<<endl;
#endif
            clientResetScene();
            if (!isIdle()) 
                setIdle(true);
            
            state=RESET;
            break;
            
        case SIMRESETEXP:
#ifdef DEBUG_CMD
            cout << "CMD:RESETEXP"<<endl;
#endif
            clientResetScene();
            
            if (!isIdle()) 
                setIdle(true);
            
            state=RESETEXP;
            break;
            
        case SIMLOAD:
#ifdef DEBUG_CMD
            cout << "CMD:LOAD"<<endl;
#endif    
            state = LOAD;
            break;
            
        case SIMLOADIMM:
#ifdef DEBUG_CMD
            cout << "CMD:LOADIMM"<<endl;
#endif
            immParams = *params;
            state=RUNIMM;
            setIdle(false);
            break;
            
        case SIMRUNEXP:
#ifdef DEBUG_CMD
            cout << "CMD:RUNEXP"<<endl;
#endif
            setIdle(false);
            state=RUNEXP;
            break;
            
        case SIMCHKBUSY:
            cout << "CMD:CHKBUSY"<<endl;
            //Do NOTHING Dealt with in zmqRec
            break;
        default:
            break;
    }
}


void HexapodSimulationDemo::setMotorTargets(btScalar deltaTime)
{
    switch (state) {
        case LOAD:
            cout<<"In Load"<<endl;
            for(int r=0; r<m_hexapods.size();r++)
            {
                Hexapod* hpod= m_hexapods[r];
                hpod->loadCtrlParams(params, param_count);
            }
            state=RESET;
            break;
            
        case RUNEXP:
            for (int r=0; r<m_hexapods.size(); r++) {
                Hexapod* hpod= m_hexapods[r];
                hpod->step();
            }
            break;
        case RESET:
            for (int r=0; r<m_hexapods.size(); r++) {
                Hexapod* hpod=m_hexapods[r];
                hpod->reset_idx();
            }
            state = RUN;
            break;
            
        case RESETEXP:
            for (int r=0; r<m_hexapods.size(); r++) {
                Hexapod* hpod=m_hexapods[r];
                hpod->reset();
            }
            state = RUN;
            break;
            
        case RUNIMM:
            for (int r=0; r<m_hexapods.size(); r++) {
                Hexapod* hpod=m_hexapods[r];
                hpod->setCtrlParams(immParams);
            }
            break;
        case CHKBUSY:
        case RUN:
        default:
            break;
    }
    isDirty=false;
    
    for (int r=0; r<m_hexapods.size(); r++) {
        Hexapod* hpod=m_hexapods[r];
#ifdef DEBUGP_POS
        //debugPos(hpod->getPosition());
#endif
    }
    
 //Set motor shiznit here	
}

bool HexapodSimulationDemo::isBusySim() {
    for (int r=0; r<m_hexapods.size(); r++) {
        Hexapod* hpod=m_hexapods[r];
        if(hpod->isStepping())
            return true;
    }
    return false;
}

void motorPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
	HexapodSimulationDemo* hexapodDemo = (HexapodSimulationDemo*)world->getWorldUserInfo();
    
    //std::cout << "Hello Tick" << std::endl;
	hexapodDemo->setMotorTargets(timeStep);
	
}


//################## END HEXAPOD ######################//




