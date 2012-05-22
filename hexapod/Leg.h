//
//  Leg.h
//  AllBulletDemos
//
//  Created by Henry Herman on 5/16/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef AllBulletDemos_Leg_h
#define AllBulletDemos_Leg_h

#include <unistd.h>
#include <stdint.h>
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/btBulletDynamicsCommon.h"
#include "BodyPart.h"



#define UPPER_LEG_LENGTH 0.45
#define LOWER_LEG_LENGTH 0.5

#define UPPER_LEG_THICK  0.07
#define LOWER_LEG_THICK  0.07

class Hexapod;

class Leg : public BodyPart
{
    enum {
        LEG_UPPER = 0,
        LEG_LOWER,
        LEG_COUNT
    };
    
    btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[LEG_COUNT];
	btRigidBody* m_bodies[LEG_COUNT];
	
    Hexapod *hpod;
    bool isLeft;
    
    btVector3 xaxis, yaxis, zaxis;
    btScalar angleA; // = targetAngleX;
    btScalar angleB; // = targetAngleY;
    
public:
    Leg (Hexapod *hexapod,
         btDynamicsWorld* ownerWorld,
         const btTransform& offset,
         const btTransform& bodyOffset,
         btScalar scale_hexapod = btScalar(1.0), bool left=false);
    
    enum {
        JOINT_KNEE = 0,
        JOINT_HIP,
        JOINT_COUNT
    };
    
    inline void setLeft(bool choice) {
        isLeft = choice;
    };
    
    btHingeConstraint* knee;
    btConeTwistConstraint* hip;
    btTypedConstraint* m_joints[JOINT_COUNT];
    ~Leg ();
    void setKneeTarget(const btQuaternion& targetAngleQ, btScalar dt);
    void setKneeTarget(const btScalar targetAngle, btScalar dt);
    void setKneeMaxStrength(const btScalar strength);
    btScalar getKneeAngle();
    
    void setHipTarget(const btQuaternion& targetAngleQ, btScalar dt);
    void setHipTarget(const btScalar targetAngleA, const btScalar targetAngleB, btScalar dt);
    void setHipMaxStrength(const btScalar strength);
    btScalar getHipAngleA();
    btScalar getHipAngleB();
    void wake();
    
};


#endif
