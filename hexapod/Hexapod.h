#ifndef HEXAPOD_H_INCLUDED
#define HEXAPOD_H_INCLUDED

#include <unistd.h>
#include <stdint.h>
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/btBulletDynamicsCommon.h"
#include "BodyPart.h"
#include "Leg.h"

#define BODY_WIDTH 0.30
#define BODY_HEIGHT 0.20
#define BODY_LENGTH 0.60


class Hexapod : public BodyPart
{
	enum
	{
		BODY_THORAX = 0,

		BODYPART_COUNT
	};

	enum
	{
		JOINT_PELVIS_SPINE = 0,

		JOINT_COUNT
	};
    
    enum
    {  
        FRONT_LEFT = 0,
        FRONT_RIGHT,
        MIDDLE_LEFT,
        MIDDLE_RIGHT,
        REAR_LEFT,
        REAR_RIGHT,
        LEG_COUNT
    };
    
    Leg *legs[LEG_COUNT];
	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
    
    btRigidBody* m_bodies[BODYPART_COUNT];
	
	btTypedConstraint* m_joints[JOINT_COUNT];

public:
	Hexapod (btDynamicsWorld* ownerWorld,
				const btVector3& positionOffset,
				btScalar scale_hexapod = btScalar(1.0));

	~Hexapod ();
    
    inline uint64_t legCount() {
        return LEG_COUNT;
    }
    btAlignedObjectArray<class Leg*> m_legs;
    btAlignedObjectArray<class Leg*> m_leftLegs;
    btAlignedObjectArray<class Leg*> m_rightlegs;

    
    btRigidBody* body;
};



#endif // HEXAPOD_H_INCLUDED
