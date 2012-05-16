//
//  BodyPart.cpp
//  AllBulletDemos
//
//  Created by Henry Herman on 5/16/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#include "BodyPart.h"


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