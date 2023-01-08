#pragma once
#include "projects/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
class Group;

//COHESION - FLOCKING
//*******************
class Cohesion : public Seek
{
public:
	Cohesion(Group* pGroup) :m_pGroup(pGroup) {};

	//Cohesion Behavior
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;

private:
	Group* m_pGroup = nullptr;
};

//SEPARATION - FLOCKING
//*********************
class Separation : public ISteeringBehavior
{
public:
	Separation(Group* pGroup) :m_pGroup(pGroup) {};

	//Separation Behavior
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;

private:
	Group* m_pGroup = nullptr;
};

//VELOCITY MATCH - FLOCKING
//************************
class VelocityMatch : public ISteeringBehavior
{
public:
	VelocityMatch(Group* pGroup) :m_pGroup(pGroup) {};

	//VelocityMatch Behavior
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;

private:
	Group* m_pGroup = nullptr;
};