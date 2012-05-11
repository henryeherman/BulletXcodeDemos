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
#define LOWER_LEG_LENGTH 0.37

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

// Flags a body as kinematic
// This should be called on any objects that requires movement
void BodyPart::setAsKinematicBody(btRigidBody *body) {
    body->setCollisionFlags(body->getCollisionFlags() | 
                            btCollisionObject::CF_KINEMATIC_OBJECT);
    body->setActivationState(DISABLE_DEACTIVATION);
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
        
        // Set the legs as kinematic
        setAsKinematicBody(m_bodies[i]);
	}
    
    ///////////////////////////// SETTING THE CONSTRAINTS /////////////////////////////////////////////7777
	// Now setup the constraints
	btGeneric6DofConstraint * joint6DOF;
	btTransform localA, localB;
	bool useLinearReferenceFrameA = true;
    
    
    
    /// ******* KNEE ******** ///
	{
		localA.setIdentity(); localB.setIdentity();
        
		localA.setOrigin(btVector3(btScalar(0.), btScalar((-0.5)*UPPER_LEG_LENGTH*scale_hexapod), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar((0.5)*LOWER_LEG_LENGTH*scale_hexapod), btScalar(0.)));
		joint6DOF =  new btGeneric6DofConstraint (*m_bodies[LEG_UPPER], *m_bodies[LEG_LOWER], localA, localB,useLinearReferenceFrameA);
        //

		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.7f,SIMD_EPSILON,SIMD_EPSILON));

		m_joints[JOINT_KNEE] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_KNEE], true);
	}
    /// *************************** ///
    
    
    
    /// ******* HIP ******** ///
	{
		localA.setIdentity(); localB.setIdentity();
        
        
		localA.setOrigin(btVector3(btScalar(0.), btScalar((0.5)*UPPER_LEG_LENGTH*scale_hexapod), btScalar(0.)));
        
		localB.setOrigin(bodyOffset.getOrigin());
		
        joint6DOF =  new btGeneric6DofConstraint (*m_bodies[LEG_UPPER], *parentBody, localA, localB,useLinearReferenceFrameA);
        
        
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.7f,SIMD_EPSILON,SIMD_EPSILON));
        
		m_joints[JOINT_HIP] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_HIP], true);
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
    
    m_shapes[BODYPART_HEAD] = new btCapsuleShape(btScalar(scale_hexapod*0.10), btScalar(scale_hexapod*0.05));
    
    
    
	btTransform transform;
	transform.setIdentity();
    m_bodies[BODY_THORAX] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODY_THORAX]);   
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(scale_hexapod*1.6), btScalar(0.)));
	m_bodies[BODYPART_HEAD] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_HEAD]);
    
    body = m_bodies[BODY_THORAX];
    
	// Setup some damping on the m_bodies
	for (int i = 0; i < BODYPART_COUNT; ++i)
	{
		m_bodies[i]->setDamping(0.05f, 0.85f);
		m_bodies[i]->setDeactivationTime(0.8f);
		m_bodies[i]->setSleepingThresholds(1.6f, 2.5f);
        
        // Set the body parts as kinematic
        setAsKinematicBody(m_bodies[i]);
        
	}
    
    
    btVector3 xaxis;
    xaxis.setValue(btScalar(1.), btScalar(0.), btScalar(0.));
    
    btVector3 yaxis;
    yaxis.setValue(btScalar(0.), btScalar(1.), btScalar(0.));
    
    btVector3 zaxis;
    zaxis.setValue(btScalar(0.), btScalar(0.), btScalar(1.));
    
    
    btQuaternion leftLegQuat, rightLegQuat;
    leftLegQuat.setRotation(zaxis,  btRadians(0));
    rightLegQuat.setRotation(yaxis, btRadians(180));
 
    btTransform legPosTransform, legRotTransform;
    legPosTransform.setIdentity(); legRotTransform.setIdentity();
    
    legPosTransform.setRotation(leftLegQuat);
    legPosTransform.setOrigin(
        btVector3(btScalar(LEFT_SIDE*scale_hexapod*BODY_WIDTH),
                  btScalar(BOTTOM_SIDE*scale_hexapod*BODY_HEIGHT),
                  btScalar(FRONT_SIDE*scale_hexapod*BODY_LENGTH)));
                                 
    legs[FRONT_LEFT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod);
 
    
    legPosTransform.setRotation(rightLegQuat);
    legPosTransform.setOrigin(
                              btVector3(btScalar(RIGHT_SIDE*scale_hexapod*BODY_WIDTH),
                                        btScalar(BOTTOM_SIDE*scale_hexapod*BODY_HEIGHT),
                                        btScalar(FRONT_SIDE*scale_hexapod*BODY_LENGTH)));
    
    legs[FRONT_RIGHT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod);
    
    
    legPosTransform.setRotation(leftLegQuat);
    legPosTransform.setOrigin(
                              btVector3(btScalar(LEFT_SIDE*scale_hexapod*BODY_WIDTH),
                                        btScalar(BOTTOM_SIDE*scale_hexapod*BODY_HEIGHT),
                                        btScalar(CENTER_SIDE*scale_hexapod*BODY_LENGTH)));
    
    legs[MIDDLE_LEFT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod);
    

    legPosTransform.setRotation(rightLegQuat);
    legPosTransform.setOrigin(
                              btVector3(btScalar(RIGHT_SIDE*scale_hexapod*BODY_WIDTH),
                                        btScalar(BOTTOM_SIDE*scale_hexapod*BODY_HEIGHT),
                                        btScalar(CENTER_SIDE*scale_hexapod*BODY_LENGTH)));
    
    legs[MIDDLE_RIGHT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod);
    
    
    legPosTransform.setRotation(leftLegQuat);
    legPosTransform.setOrigin(
                              btVector3(btScalar(LEFT_SIDE*scale_hexapod*BODY_WIDTH),
                                        btScalar(BOTTOM_SIDE*scale_hexapod*BODY_HEIGHT),
                                        btScalar(REAR_SIDE*scale_hexapod*BODY_LENGTH)));
    
    legs[REAR_LEFT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod);
    
    legPosTransform.setRotation(rightLegQuat);
    legPosTransform.setOrigin(
                              btVector3(btScalar(RIGHT_SIDE*scale_hexapod*BODY_WIDTH),
                                        btScalar(BOTTOM_SIDE*scale_hexapod*BODY_HEIGHT),
                                        btScalar(REAR_SIDE*scale_hexapod*BODY_LENGTH)));
    
    legs[REAR_RIGHT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod);
    


///////////////////////////// SETTING THE CONSTRAINTS /////////////////////////////////////////////7777
	// Now setup the constraints
	btGeneric6DofConstraint * joint6DOF;
	btTransform localA, localB;
	bool useLinearReferenceFrameA = true;
/// ******* SPINE HEAD ********       
    {
        localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.30*scale_hexapod), btScalar(0.)));     
        
        localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14*scale_hexapod), btScalar(0.)));
        
        
        joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODY_THORAX], *m_bodies[BODYPART_HEAD], localA, localB,useLinearReferenceFrameA);


		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.3f,-SIMD_EPSILON,-SIMD_PI*0.3f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.5f,SIMD_EPSILON,SIMD_PI*0.3f));

		m_joints[JOINT_SPINE_HEAD] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);
	}
/// *************************** ///

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

