//
//  BodyPart.h
//  AllBulletDemos
//
//  Created by Henry Herman on 5/16/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef AllBulletDemos_BodyPart_h
#define AllBulletDemos_BodyPart_h

#include <unistd.h>
#include <stdint.h>
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/btBulletDynamicsCommon.h"


class BodyPart 
{
    
    btDynamicsWorld* m_ownerWorld;
    
public:
    BodyPart(btDynamicsWorld* ownerWorld);
    ~BodyPart();
    btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape); 
    
    // Flags a body as kinematic
    // This should be called on any objects that requires movement
    void setAsKinematicBody(btRigidBody *body);
    
};

#endif
