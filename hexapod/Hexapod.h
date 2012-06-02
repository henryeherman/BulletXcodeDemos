#ifndef HEXAPOD_H_INCLUDED
#define HEXAPOD_H_INCLUDED

#include <unistd.h>
#include <stdint.h>
#include <vector>

#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/btBulletDynamicsCommon.h"
#include "BodyPart.h"
#include "Leg.h"

#define BODY_WIDTH 0.30
#define BODY_HEIGHT 0.20
#define BODY_LENGTH 0.60
//#define RIGID 1
//#define BODY_SHAPE_BOX  //Otherwise Capsule!
#define LEFT_SIDE 1
#define RIGHT_SIDE (-1)
#define CENTER_SIDE 0
#define FRONT_SIDE 1
#define REAR_SIDE (-1)
#define BOTTOM_SIDE (-2)
#define NUMLEGS 6


//#define FREEZE 1

class HpodCtrlParams {
    public:
        btScalar kneeAngles[NUMLEGS];
        btScalar hipAnglesX[NUMLEGS];
        btScalar hipAnglesY[NUMLEGS];
        btScalar hipStrength;
        btScalar kneeStrength;
        btScalar dtKnee;
        btScalar dtHip;
};

class HpodReply {
    public:
        unsigned int podid;
        btScalar xpos;
        btScalar ypos;
        btScalar zpos;
        btScalar upperlegforce[NUMLEGS];
        btScalar lowerlegforce[NUMLEGS];
};

void debugPos(btVector3 pos);
void debugCtrlParams(HpodCtrlParams params);

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
	
	btAlignedObjectArray<btTypedConstraint*> m_joints;
    
    unsigned int param_idx;
    
    unsigned int podid;
    
    bool stepping;

public:

    Hexapod (btDynamicsWorld* ownerWorld,
             const btVector3& positionOffset,
             btScalar scale_hexapod = btScalar(1.0), unsigned int _podid=0);
    
	~Hexapod ();
    
    inline uint64_t legCount() {
        return LEG_COUNT;
    }
    btAlignedObjectArray<class Leg*> m_legs;
    btAlignedObjectArray<class Leg*> m_leftLegs;
    btAlignedObjectArray<class Leg*> m_rightlegs;
    std::vector<HpodCtrlParams> m_ctrlParams;
    std::vector<HpodReply> m_replys;
    
    
    btAlignedObjectArray<btVector3> m_forces;
    
    void loadCtrlParams(HpodCtrlParams *params, unsigned long size);
    void clearCtrlParams();
    
    void step();
    void reset();
    
    void storeReply();
    void getForces();
    bool isStepping();
    
    btRigidBody* body;
    void wake();
    void setCtrlParams(const HpodCtrlParams params);
    void getCtrlParams(HpodCtrlParams &params);
    btVector3 getPosition();

};

#endif // HEXAPOD_H_INCLUDED
