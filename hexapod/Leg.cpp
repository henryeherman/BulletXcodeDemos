//
//  Leg.cpp
//  AllBulletDemos
//
//  Created by Henry Herman on 5/16/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#include <unistd.h>
#include <stdint.h>
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/btBulletDynamicsCommon.h"
#include "BodyPart.h"
#include "Leg.h"
#include "Hexapod.h"

Leg::Leg (Hexapod *hexapod, btDynamicsWorld* ownerWorld, const btTransform& offset, const btTransform& bodyOffset,
          btScalar scale_hexapod, bool left)	: m_ownerWorld (ownerWorld), BodyPart(ownerWorld)
{
    isLeft = left;
    btRigidBody *parentBody;
    parentBody = hexapod->body;
    btTransform globalFrame;
    globalFrame.setIdentity();
    globalFrame*=offset*bodyOffset;
    
    xaxis.setValue(btScalar(1.), btScalar(0.), btScalar(0.));
    
    yaxis.setValue(btScalar(0.), btScalar(1.), btScalar(0.));
    
    zaxis.setValue(btScalar(0.), btScalar(0.), btScalar(1.));
    
    if (isLeft) {
        xaxis.setValue(btScalar(-1.), btScalar(0.), btScalar(0.));
        //    yaxis.setValue(btScalar(-1.), btScalar(0), btScalar(0.));
        //    zaxis.setValue(btScalar(0.), btScalar(0.), btScalar(-1.));
    }
    
	// Setup the geometry
	m_shapes[LEG_UPPER] = new btCapsuleShape(btScalar(scale_hexapod*UPPER_LEG_THICK), btScalar(scale_hexapod*UPPER_LEG_LENGTH));
	m_shapes[LEG_LOWER] = new btCapsuleShape(btScalar(scale_hexapod*LOWER_LEG_THICK), btScalar(scale_hexapod*LOWER_LEG_LENGTH));
    
    
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
		//m_bodies[i]->setSleepingThresholds(1.6f, 2.5f);
        m_bodies[i]->setSleepingThresholds(0.0f, 0.0f);
        
	}
    
    /////////////// SETTING THE CONSTRAINTS ////////////////
	// Now setup the constraints
    btHingeConstraint *hingeC;
    btConeTwistConstraint *coneC;
	btTransform localA, localB;
    
    /// ******* KNEE ******** ///
    
    {
        
        localA.setIdentity(); localB.setIdentity();
        
        
		localA.setOrigin(btVector3(
                                   btScalar(0.), 
                                   btScalar((-0.5)*UPPER_LEG_LENGTH*scale_hexapod), 
                                   btScalar(0.)));
        
		localB.setOrigin(btVector3(btScalar(0.), 
                                   btScalar((0.5)*LOWER_LEG_LENGTH*scale_hexapod), 
                                   btScalar(0.)));
        
        hingeC = new btHingeConstraint(*m_bodies[LEG_UPPER], 
                                       *m_bodies[LEG_LOWER], 
                                       localA, localB);
        
        hingeC->setLimit(-SIMD_EPSILON, SIMD_PI*0.7f);
        
		m_joints.push_back(hingeC);
        knee = hingeC;
        knee->setDbgDrawSize(btScalar(1.f));
		m_ownerWorld->addConstraint(knee, true);
	}
    /// *************************** ///
    
    /// ******* HIP ******** ///
	{
        
        
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
        
        coneC = new btConeTwistConstraint(*m_bodies[LEG_UPPER], *parentBody, localA, localB);
 		coneC->setLimit(btScalar(SIMD_PI*0.4f), btScalar(SIMD_PI*0.3f), btScalar(0), 0.3f);
		m_joints.push_back(coneC);
        //m_joints[JOINT_HIP] = coneC;
        m_ownerWorld->addConstraint(coneC, true);
		coneC->setDbgDrawSize(btScalar(1.f));
        hip = coneC;
        
	}
    /// *************************** ///
    
    
}

void Leg::setKneeTarget(const btQuaternion& targetAngleQ, btScalar dt) {
    knee->enableMotor(true);
    knee->setMotorTarget(targetAngleQ, dt);    
}

void Leg::setKneeTarget(const btScalar targetAngle, btScalar dt) {
    knee->enableMotor(true);
    knee->setMotorTarget(targetAngle, dt);
}

void Leg::setHipTarget(const btQuaternion& targetAngleQ, btScalar dt) {
    hip->enableMotor(true);
    hip->setMotorTargetInConstraintSpace(targetAngleQ);
}

void Leg::setHipTarget(const btScalar targetAngleX, const btScalar targetAngleY, btScalar dt) {
    
    angleA = targetAngleX;
    angleB = targetAngleY;
    
    hip->enableMotor(true);
    btQuaternion localQuatX, localQuatY;
    localQuatX.setRotation(zaxis, targetAngleX);
    localQuatY.setRotation(xaxis, targetAngleY);
    localQuatX *= localQuatY;
    
    hip->setMotorTargetInConstraintSpace(localQuatX);
}

void Leg::setHipMaxStrength(const btScalar strength) {
    hip->setMaxMotorImpulse(strength);
}


btScalar Leg::getKneeAngle() {
    return knee->getHingeAngle();
}

void Leg::setKneeMaxStrength(const btScalar strength) {
    knee->setMaxMotorImpulse(strength);
}


btScalar Leg::getHipAngleA(){
    
    //hip->m_qTarget.getAngle();
    //hip->m_qTarget.getAxis();
    return angleA;
}

btScalar Leg::getHipAngleB(){
    return angleB;
}

void Leg::wake() {
    m_bodies[LEG_UPPER]->activate();
    m_bodies[LEG_LOWER]->activate();
}


Leg::~Leg()
{
	int i;
    
	//Remove all constraints
    for (i = 0; i < m_joints.size(); ++i)
	{
		m_ownerWorld->removeConstraint(m_joints[i]);
        delete m_joints[i];
	}
    //m_joints.clear();
    
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