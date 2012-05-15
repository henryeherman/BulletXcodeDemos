#ifndef HEXAPOD_H_INCLUDED
#define HEXAPOD_H_INCLUDED

#include "DemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/btBulletDynamicsCommon.h"

#define BODY_WIDTH 0.30
#define BODY_HEIGHT 0.20
#define BODY_LENGTH 0.60


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
    

    
    public:
        Leg (Hexapod *hexapod,
             btDynamicsWorld* ownerWorld,
                 const btTransform& offset,
                 const btTransform& bodyOffset,
                 btScalar scale_hexapod = btScalar(1.0));
        
        enum {
            JOINT_KNEE = 0,
            JOINT_HIP,
            JOINT_COUNT
        };
        
        btHingeConstraint* knee;
        btConeTwistConstraint* hip;
        btTypedConstraint* m_joints[JOINT_COUNT];
        ~Leg ();
};

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

    btRigidBody* body;
};



#endif // HEXAPOD_H_INCLUDED
