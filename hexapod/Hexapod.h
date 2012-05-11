#ifndef HEXAPOD_H_INCLUDED
#define HEXAPOD_H_INCLUDED

#include "DemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/btBulletDynamicsCommon.h"

#define BODY_WIDTH 0.30
#define BODY_HEIGHT 0.20
#define BODY_LENGTH 0.60

class Hexapod;

class Leg
{
    enum {
        LEG_UPPER = 0,
        LEG_LOWER,
        LEG_COUNT
    };
    
    enum {
        JOINT_KNEE = 0,
        JOINT_COUNT
    };
    
    btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[LEG_COUNT];
	btRigidBody* m_bodies[LEG_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];
    Hexapod *hpod;
    btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape);
    
    void setAsKinematicBody(btRigidBody *body);
    
    public:
        Leg (Hexapod *hexapod,
             btDynamicsWorld* ownerWorld,
                 const btVector3& positionOffset,
                 btScalar scale_hexapod = btScalar(1.0));
        
        ~Leg ();
};

class Hexapod
{
	enum
	{
		BODY_THORAX = 0,
		BODYPART_HEAD,

		BODYPART_COUNT
	};

	enum
	{
		JOINT_PELVIS_SPINE = 0,
		JOINT_SPINE_HEAD,

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

	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape);
    
    void setAsKinematicBody(btRigidBody *body);

public:
	Hexapod (btDynamicsWorld* ownerWorld,
				const btVector3& positionOffset,
				btScalar scale_hexapod = btScalar(1.0));

	~Hexapod ();
};



#endif // HEXAPOD_H_INCLUDED
