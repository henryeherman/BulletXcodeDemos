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

Written by: Marten Svanfeldt
*/

#include "Hexapod.h"

//#define RIGID 1

#define LEFT_SIDE 1
#define RIGHT_SIDE (-1)
#define CENTER_SIDE 0
#define FRONT_SIDE 1
#define REAR_SIDE (-1)
#define BOTTOM_SIDE (-2)

#define UPPER_LEG_LENGTH 0.45
#define LOWER_LEG_LENGTH 0.5

#define SIMD_PI_2 ((SIMD_PI)*0.5f)
#define SIMD_PI_4 ((SIMD_PI)*0.25f)

//################## BEGIN BodyPart ######################//

BodyPart::BodyPart(btDynamicsWorld* ownerWorld) : m_ownerWorld (ownerWorld){
    
}

BodyPart::~BodyPart(){
    
}

btRigidBody* BodyPart::localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	bool isDynamic = (mass != 0.f);
    
	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);
    
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	rbInfo.m_additionalDamping = true;
	btRigidBody* body = new btRigidBody(rbInfo);
    
	m_ownerWorld->addRigidBody(body);
    
	return body;
}


//################## END BodyPart ######################//



Leg::Leg (Hexapod *hexapod, btDynamicsWorld* ownerWorld, const btTransform& offset, const btTransform& bodyOffset,
                  btScalar scale_hexapod)	: m_ownerWorld (ownerWorld), BodyPart(ownerWorld)
{
    btRigidBody *parentBody;
    parentBody = hexapod->body;
    
    btTransform globalFrame;
    globalFrame.setIdentity();
    globalFrame*=offset*bodyOffset;
    
	// Setup the geometry
	m_shapes[LEG_UPPER] = new btCapsuleShape(btScalar(scale_hexapod*0.07), btScalar(scale_hexapod*UPPER_LEG_LENGTH));
	m_shapes[LEG_LOWER] = new btCapsuleShape(btScalar(scale_hexapod*0.05), btScalar(scale_hexapod*LOWER_LEG_LENGTH));
    
	// Setup all the rigid bodies
    
    btTransform transform;
	transform.setIdentity();
    btQuaternion temp;
    
	//transform.setOrigin(btVector3(btScalar(-0.18*scale_hexapod), btScalar(0.65*scale_hexapod),                                  btScalar(0.)));
	m_bodies[LEG_UPPER] = localCreateRigidBody(btScalar(1.), globalFrame*transform, m_shapes[LEG_UPPER]);
    
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0*scale_hexapod), btScalar((-1)*LOWER_LEG_LENGTH*scale_hexapod), btScalar(0.)));
	m_bodies[LEG_LOWER] = localCreateRigidBody(btScalar(1.), globalFrame*transform, m_shapes[LEG_LOWER]);
    
    
	// Setup some damping on the m_bodies
	for (int i = 0; i < LEG_COUNT; ++i)
	{
		m_bodies[i]->setDamping(0.05f, 0.85f);
		m_bodies[i]->setDeactivationTime(0.8f);
		m_bodies[i]->setSleepingThresholds(1.6f, 2.5f);
        
	}
    
    ///////////////////////////// SETTING THE CONSTRAINTS /////////////////////////////////////////////7777
	// Now setup the constraints
	btGeneric6DofConstraint * joint6DOF;
    btHingeConstraint *hingeC;
	btTransform localA, localB;
	bool useLinearReferenceFrameA = true;
    
    
    
    /// ******* KNEE ******** ///
    
    /*{
        localA.setIdentity(); localB.setIdentity(); localC.setIdentity();
        localA.getBasis().setEulerZYX(0,-fAngle,0);	localA.setOrigin(btVector3(btScalar(fCos*(fBodySize+fLegLength)), btScalar(0.), btScalar(fSin*(fBodySize+fLegLength))));
        localB = m_bodies[1+2*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
        localC = m_bodies[2+2*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
        hingeC = new btHingeConstraint(*m_bodies[1+2*i], *m_bodies[2+2*i], localB, localC);
        //hingeC->setLimit(btScalar(-0.01), btScalar(0.01));
        hingeC->setLimit(btScalar(-M_PI_8), btScalar(0.2));
        m_joints[1+2*i] = hingeC;
        m_ownerWorld->addConstraint(m_joints[1+2*i], true);
    
    }
    */
    
    
	{

        localA.setIdentity(); localB.setIdentity();
        
        
		localA.setOrigin(btVector3(btScalar(0.), btScalar((-0.5)*UPPER_LEG_LENGTH*scale_hexapod), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar((0.5)*LOWER_LEG_LENGTH*scale_hexapod), btScalar(0.)));
		//joint6DOF =  new btGeneric6DofConstraint (*m_bodies[LEG_UPPER], *m_bodies[LEG_LOWER], localA, localB,useLinearReferenceFrameA);
        hingeC = new btHingeConstraint(*m_bodies[LEG_UPPER], *m_bodies[LEG_LOWER], localA, localB);
        
        hingeC->setLimit(-SIMD_EPSILON, SIMD_PI*0.7f);
		//joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		//joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_PI*0.7f));

		m_joints[JOINT_KNEE] = hingeC;
		m_ownerWorld->addConstraint(m_joints[JOINT_KNEE], true);
	}
    /// *************************** ///
    
    
    
    /// ******* HIP ******** ///
	{
        btVector3 xaxis;
        xaxis.setValue(btScalar(1.), btScalar(0.), btScalar(0.));
        
        btVector3 yaxis;
        yaxis.setValue(btScalar(0.), btScalar(1.), btScalar(0.));
        
        btVector3 zaxis;
        zaxis.setValue(btScalar(0.), btScalar(0.), btScalar(1.));
        
        
        btQuaternion hipRotQuat;
        btTransform hipRotTransform; hipRotTransform.setIdentity();
        
        
        hipRotQuat.setRotation(yaxis,  btRadians(45));
        hipRotTransform.setRotation(hipRotQuat);
        
         

        
        
        localB.setRotation(hipRotQuat);
        localA.setIdentity();
        localA.setOrigin(btVector3(btScalar(0.), btScalar((0.5)*UPPER_LEG_LENGTH*scale_hexapod), btScalar(0.)));
        
        localA*=hipRotTransform;        
        localA.getBasis().setEulerZYX(0, 0, 0);
        
        localB.setIdentity();
        localB.setOrigin(bodyOffset.getOrigin());
		localB.setRotation(bodyOffset.getRotation());
        
        btConeTwistConstraint *local_ctc = new btConeTwistConstraint(*m_bodies[LEG_UPPER], *parentBody, localA, localB);
 		local_ctc->setLimit(btScalar(SIMD_PI_4*0.6f), btScalar(SIMD_PI_4), btScalar(SIMD_PI) * 0.8f, 0.5f);
		m_joints[JOINT_HIP] = local_ctc;
        m_ownerWorld->addConstraint(m_joints[JOINT_HIP], true);
		m_joints[JOINT_HIP]->setDbgDrawSize(btScalar(5.f));
        
        
        
        

    
        
        
//        joint6DOF =  new btGeneric6DofConstraint (*m_bodies[LEG_UPPER], *parentBody, localA, localB,useLinearReferenceFrameA);
        
        
		//joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		//joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.7f,SIMD_EPSILON,SIMD_EPSILON));
        
        //joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		//joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_PI*0.7f,SIMD_EPSILON));
        
//        joint6DOF->setAngularLowerLimit(btVector3(-0.01*SIMD_PI,-SIMD_EPSILON,-0.1*SIMD_PI));
//		joint6DOF->setAngularUpperLimit(btVector3(0.01*SIMD_PI,SIMD_EPSILON,0.3*SIMD_PI));
        
//		m_joints[JOINT_HIP] = joint6DOF;
//		m_ownerWorld->addConstraint(m_joints[JOINT_HIP], true);
	}
    /// *************************** ///
     
    
}

Leg::~Leg()
{
	int i;
    
	// Remove all constraints
	for (i = 0; i < JOINT_COUNT; ++i)
	{
		m_ownerWorld->removeConstraint(m_joints[i]);
		delete m_joints[i]; m_joints[i] = 0;
	}
    
	// Remove all bodies and shapes
	for (i = 0; i < LEG_COUNT; ++i)
	{
		m_ownerWorld->removeRigidBody(m_bodies[i]);
        
		delete m_bodies[i]->getMotionState();
        
		delete m_bodies[i]; m_bodies[i] = 0;
		delete m_shapes[i]; m_shapes[i] = 0;
	}
}






//################## END LEG ######################//


//################## BEGIN HEXAPOD ######################//

Hexapod::Hexapod (btDynamicsWorld* ownerWorld, const btVector3& positionOffset,
	btScalar scale_hexapod)	: m_ownerWorld (ownerWorld), BodyPart(ownerWorld)
{
  
   
    
    btTransform offset; offset.setIdentity();
	offset.setOrigin(positionOffset); 
    
    // Setup the geometry
	
    m_shapes[BODY_THORAX] = new btBoxShape(btVector3(
                                                     btScalar(scale_hexapod*BODY_WIDTH),
                                                     btScalar(scale_hexapod*BODY_HEIGHT),
                                                     btScalar(scale_hexapod*BODY_LENGTH)));
    
    
    
    
	btTransform transform;
	transform.setIdentity();
    m_bodies[BODY_THORAX] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODY_THORAX]);   
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(scale_hexapod*1.6), btScalar(0.)));
    
    body = m_bodies[BODY_THORAX];
    
	// Setup some damping on the m_bodies
	for (int i = 0; i < BODYPART_COUNT; ++i)
	{
		m_bodies[i]->setDamping(0.05f, 0.85f);
		m_bodies[i]->setDeactivationTime(0.8f);
		m_bodies[i]->setSleepingThresholds(1.6f, 2.5f);
	}
    
    
    btVector3 xaxis;
    xaxis.setValue(btScalar(1.), btScalar(0.), btScalar(0.));
    
    btVector3 yaxis;
    yaxis.setValue(btScalar(0.), btScalar(1.), btScalar(0.));
    
    btVector3 zaxis;
    zaxis.setValue(btScalar(0.), btScalar(0.), btScalar(1.));
    
    
    btQuaternion leftLegQuat, rightLegQuat;
    btQuaternion upQuat;
    upQuat.setRotation(zaxis, btRadians(90));
    leftLegQuat.setRotation(yaxis,  btRadians(0));
    leftLegQuat*=upQuat;
    rightLegQuat.setRotation(yaxis, btRadians(180));
    rightLegQuat*=upQuat;
 
    btTransform legPosTransform, legRotTransform;
    legPosTransform.setIdentity(); legRotTransform.setIdentity();
    
    legPosTransform.setRotation(leftLegQuat);
    legPosTransform.setOrigin(
        btVector3(btScalar(LEFT_SIDE*scale_hexapod*BODY_WIDTH),
                  btScalar(BOTTOM_SIDE*scale_hexapod*BODY_HEIGHT*0.5),
                  btScalar(FRONT_SIDE*scale_hexapod*BODY_LENGTH)));
                                 
    legs[FRONT_LEFT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod);
 
    
    legPosTransform.setRotation(rightLegQuat);
    legPosTransform.setOrigin(
                              btVector3(btScalar(RIGHT_SIDE*scale_hexapod*BODY_WIDTH),
                                        btScalar(BOTTOM_SIDE*scale_hexapod*BODY_HEIGHT*0.5),
                                        btScalar(FRONT_SIDE*scale_hexapod*BODY_LENGTH)));
    
    legs[FRONT_RIGHT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod);
    
    
    legPosTransform.setRotation(leftLegQuat);
    legPosTransform.setOrigin(
                              btVector3(btScalar(LEFT_SIDE*scale_hexapod*BODY_WIDTH),
                                        btScalar(BOTTOM_SIDE*scale_hexapod*BODY_HEIGHT*0.5),
                                        btScalar(CENTER_SIDE*scale_hexapod*BODY_LENGTH)));
    
    legs[MIDDLE_LEFT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod);
    

    legPosTransform.setRotation(rightLegQuat);
    legPosTransform.setOrigin(
                              btVector3(btScalar(RIGHT_SIDE*scale_hexapod*BODY_WIDTH),
                                        btScalar(BOTTOM_SIDE*scale_hexapod*BODY_HEIGHT*0.5),
                                        btScalar(CENTER_SIDE*scale_hexapod*BODY_LENGTH)));
    
    legs[MIDDLE_RIGHT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod);
    
    
    legPosTransform.setRotation(leftLegQuat);
    legPosTransform.setOrigin(
                              btVector3(btScalar(LEFT_SIDE*scale_hexapod*BODY_WIDTH),
                                        btScalar(BOTTOM_SIDE*scale_hexapod*BODY_HEIGHT*0.5),
                                        btScalar(REAR_SIDE*scale_hexapod*BODY_LENGTH)));
    
    legs[REAR_LEFT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod);
    
    legPosTransform.setRotation(rightLegQuat);
    legPosTransform.setOrigin(
                              btVector3(btScalar(RIGHT_SIDE*scale_hexapod*BODY_WIDTH),
                                        btScalar(BOTTOM_SIDE*scale_hexapod*BODY_HEIGHT*0.5),
                                        btScalar(REAR_SIDE*scale_hexapod*BODY_LENGTH)));
    
    legs[REAR_RIGHT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod);
    
}

Hexapod::~Hexapod()
{
	int i;

	// Remove all constraints
	for (i = 0; i < JOINT_COUNT; ++i)
	{
		m_ownerWorld->removeConstraint(m_joints[i]);
		delete m_joints[i]; m_joints[i] = 0;
	}

	// Remove all bodies and shapes
	for (i = 0; i < BODYPART_COUNT; ++i)
	{
		m_ownerWorld->removeRigidBody(m_bodies[i]);

		delete m_bodies[i]->getMotionState();

		delete m_bodies[i]; m_bodies[i] = 0;
		delete m_shapes[i]; m_shapes[i] = 0;
	}
    
}


//################## END HEXAPOD ######################//

