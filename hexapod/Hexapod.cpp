/*
 Hexapod Demo
 Copyright (c) 2012 Starbreeze Studios
 
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 
 Written by: Henry Herman and Jason Tsao
 */

#include "Hexapod.h"
#include <iostream>
#include <string>



#define SIMD_PI_2 ((SIMD_PI)*0.5f)
#define SIMD_PI_4 ((SIMD_PI)*0.25f)

void debugPos(btVector3 pos) {
    std::cout << "Pos:X<" << pos.x() << ">, Y<" << pos.y() << ">, Z<" << pos.z() << ">" << std::endl; 
}

void debugCtrlParams(HpodCtrlParams params) {
    std::cout << "Knee Angles: ";
    
    int i;
    for(i= 0;i< NUMLEGS;i++) {
        std::cout << params.kneeAngles[i] << " ";   
    }
    
    std::cout << "\nHip AnglesX: ";
    for(i = 0; i < NUMLEGS; i++) {
        std::cout << params.hipAnglesX[i] << " ";
    }
    
    std::cout << "\nHip AnglesY: ";
    for(i = 0; i < NUMLEGS; i++) {
        std::cout << params.hipAnglesY[i] << " ";
    }
    
    std::cout << "\nHip Strength: " << params.hipStrength << std::endl;

    std::cout << "Knee Strength: " << params.kneeStrength << std::endl;
    
    std::cout << "dtKnee: " << params.dtKnee << std::endl;
    
    std::cout << "dtHip: " << params.dtHip << std::endl;
    
    
}

//################## BEGIN HEXAPOD ######################//



Hexapod::Hexapod ( btDynamicsWorld* ownerWorld, const btVector3& positionOffset,
	btScalar scale_hexapod,unsigned int _podid)	: m_ownerWorld (ownerWorld), 
                            BodyPart(ownerWorld),
                            m_ctrlParams(),
                            m_legs(),
                            m_rightlegs(),
                            m_shapes(),
                            m_bodies(),
                            m_joints(),
                            m_leftLegs(),
                            m_forces(),
                            m_replys()
{
    podid = _podid;
    param_idx = 0;
    stepping = false;
    //HpodCtrlParams empty;    
    //m_ctrlParams.push_back(empty);
    //m_ctrlParams.clear();
    
    //std::vector<HpodCtrlParams> m_ctrlParams;
    //m_forces.push_back(btVector3(0.f,0.f,0.f));
    //m_forces.clear();

    //HpodReply empty_reply;
    //std::vector<HpodReply> m_replys;
    //m_replys.push_back(empty_reply);
    //m_replys.clear();
    
    
    btVector3 xaxis;
    xaxis.setValue(btScalar(1.), btScalar(0.), btScalar(0.));
    
    btVector3 yaxis;
    yaxis.setValue(btScalar(0.), btScalar(1.), btScalar(0.));
    
    btVector3 zaxis;
    zaxis.setValue(btScalar(0.), btScalar(0.), btScalar(1.));
    
    btTransform offset; offset.setIdentity();
	offset.setOrigin(positionOffset); 
    
    // Setup the geometry
	
#ifdef BODY_SHAPE_BOX
    m_shapes[BODY_THORAX] = new btBoxShape(btVector3(
                                                     btScalar(scale_hexapod*BODY_WIDTH),
                                                     btScalar(scale_hexapod*BODY_HEIGHT),
                                                     btScalar(scale_hexapod*BODY_LENGTH)));
#else
    m_shapes[BODY_THORAX] = new btCapsuleShapeZ(btScalar(scale_hexapod*BODY_WIDTH*1.2),  
                                               btScalar(scale_hexapod*BODY_LENGTH*1.5));
#endif
    
	btTransform transform;
	transform.setIdentity();
    //btTransform rotateBodyT; rotateBodyT.setIdentity();
    //btQuaternion rotateBodyQ;
    //rotateBodyQ.setRotation(xaxis, btRadians(90));
    //rotateBodyT.setRotation(rotateBodyQ);
    
    
    btScalar bodymass;
    
#ifdef FREEZE
    bodymass = 0.f;
#else
    bodymass = 10.f;
#endif
    
    m_bodies[BODY_THORAX] = localCreateRigidBody(bodymass, offset*transform, m_shapes[BODY_THORAX]);   
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
                                 
    legs[FRONT_LEFT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod,true);
 
    
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
    
    legs[MIDDLE_LEFT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod,true);
    
    
    
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
    
    legs[REAR_LEFT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod,true);
    
    legPosTransform.setRotation(rightLegQuat);
    legPosTransform.setOrigin(
                              btVector3(btScalar(RIGHT_SIDE*scale_hexapod*BODY_WIDTH),
                                        btScalar(BOTTOM_SIDE*scale_hexapod*BODY_HEIGHT*0.5),
                                        btScalar(REAR_SIDE*scale_hexapod*BODY_LENGTH)));
    
    legs[REAR_RIGHT] = new Leg(this, m_ownerWorld,offset,legPosTransform*legRotTransform,scale_hexapod);
    
    m_legs.push_back(legs[FRONT_LEFT]);
    m_legs.push_back(legs[FRONT_RIGHT]);
    m_legs.push_back(legs[MIDDLE_LEFT]);
    m_legs.push_back(legs[MIDDLE_RIGHT]);
    m_legs.push_back(legs[REAR_LEFT]);
    m_legs.push_back(legs[REAR_RIGHT]);
    
    m_leftLegs.push_back(legs[FRONT_LEFT]);
    m_leftLegs.push_back(legs[MIDDLE_LEFT]);
    m_leftLegs.push_back(legs[REAR_LEFT]);
    
    m_rightlegs.push_back(legs[FRONT_RIGHT]);
    m_rightlegs.push_back(legs[MIDDLE_RIGHT]);
    m_rightlegs.push_back(legs[REAR_RIGHT]);
    
    legs[REAR_LEFT]->knee->getMotorTargetVelosity();  // WTH with spelling error
    goDefaultPos();
    
}

void Hexapod::goDefaultPos() {
    HpodCtrlParams param;
    param.dtHip = 0.5;
    param.dtHip = 0.5;
    for(int i=0;i<LEG_COUNT;i++) {
        param.hipAnglesX[i] = 0.75f;
        param.hipAnglesY[i] = 0.0f;
        param.kneeAngles[i] = 2.5f;
    }
    setCtrlParams(param);
}

void Hexapod::setCtrlParams(const HpodCtrlParams params) {

    for(int i=0;i<LEG_COUNT;i++) {
        legs[i]->setKneeTarget(params.kneeAngles[i], params.dtKnee);
        legs[i]->setHipTarget(params.hipAnglesX[i], params.hipAnglesY[i], params.dtHip);
        legs[i]->setKneeMaxStrength(params.kneeStrength);
        legs[i]->setHipMaxStrength(params.hipStrength);
    }
    
}

void Hexapod::getCtrlParams(HpodCtrlParams &params) {
    for(int i=0;i<LEG_COUNT;i++) {
        params.kneeAngles[i] = legs[i]->getKneeAngle();
        params.hipAnglesX[i] = legs[i]->getHipAngleA();
        params.hipAnglesY[i] = legs[i]->getHipAngleB();
    }
}


void Hexapod::loadCtrlParams(HpodCtrlParams *params, unsigned long size) {
    param_idx = 0;
    unsigned int count = 0;
    if (!m_ctrlParams.empty()) {
        m_ctrlParams.clear();
    }
    for (int i=0;i<size;i++) {
        count++;
        m_ctrlParams.push_back(*(params+i));
    }
    std::cout << "Size Added: " << m_ctrlParams.size() << std::endl;
}


void Hexapod::clearCtrlParams(){
        m_ctrlParams.clear();
}

void Hexapod::wake() {
    m_bodies[BODY_THORAX]->activate();
    for(int i=0;i<LEG_COUNT;i++) {
        legs[i]->wake();
    } 
}

void Hexapod::step() {
    //std::cout << "Array sz: " << m_ctrlParams.size() << std::endl;  
    static  bool showComplete = true;
    
    if (param_idx==0) {
        showComplete=true;
    }
    
    if (m_ctrlParams.size() < 1) 
        return;
    
    if (param_idx < m_ctrlParams.size()) {
        std::cout << "Stepping: "<< param_idx << std::endl;
        setCtrlParams(m_ctrlParams[param_idx]);
        debugCtrlParams(m_ctrlParams[param_idx]);
        storeReply();
        stepping=true;
        param_idx++;
    } else {
        if(showComplete) {
            std::cout << "Complete" << std::endl;
            showComplete=false;
            stepping=false;
        }
    }
}

bool Hexapod::isStepping() {
    return stepping;
}

void Hexapod::getForces() {
    
    
}
void Hexapod::reset() {
    m_replys.clear();
    m_ctrlParams.clear();
    reset_idx();
}

void Hexapod::reset_idx() {
    param_idx = 0;
}

btVector3 Hexapod::getPosition() {
    btTransform temp = body->getWorldTransform();
    btVector3 pos = temp.getOrigin();
    return pos;
}

void Hexapod::storeReply() {
    HpodReply reply;
    reply.podid = podid;
    btVector3 pos = getPosition();
    reply.xpos = pos.x();
    reply.ypos = pos.y();
    reply.zpos = pos.z();
    
    for(int i = 0; i<NUMLEGS; i++) {
        reply.upperlegforce[i] =  m_legs[i]->hip->getAppliedImpulse();
        reply.lowerlegforce[i] = m_legs[i]->knee->getAppliedImpulse();
        btVector3 temp;
        btVector3 zero(1.0f,2.0f,3.0f);
        temp = m_legs[i]->upperleg->getAngularVelocity();
        reply.upperLegAngularVelocities[i][0] = temp.x();
        reply.upperLegAngularVelocities[i][1] = temp.y();
        reply.upperLegAngularVelocities[i][2] = temp.z();
        temp = m_legs[i]->upperleg->getLinearVelocity();;
        reply.upperLegVelocities[i][0] = temp.x();
        reply.upperLegVelocities[i][1] = temp.y();
        reply.upperLegVelocities[i][2] = temp.z();
        temp = m_legs[i]->lowerleg->getLinearVelocity();
        reply.lowerLegVelocities[i][0] = temp.x();
        reply.lowerLegVelocities[i][1] = temp.y();
        reply.lowerLegVelocities[i][2] = temp.z();
        temp = m_legs[i]->lowerleg->getAngularVelocity();
        reply.lowerLegAngularVelocities[i][0] = temp.x();
        reply.lowerLegAngularVelocities[i][1] = temp.y();
        reply.lowerLegAngularVelocities[i][2] = temp.z();
    }
    m_replys.push_back(reply);
    std::cout<<"Store reply:"<<m_replys.size()<<std::endl;
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

