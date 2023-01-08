#include "stdafx.h"
#include "FlockingSteeringBehaviors.h"
#include "projects/Movement/FormationMovement/UnitAgent.h"
#include "../SteeringHelpers.h"
#include "projects/Movement/FormationMovement/Group.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	m_Target = (m_pGroup->GetNrOfNeighbors() > 0 ? m_pGroup->GetAverageNeighborPos() : pAgent->GetPosition());
	return Seek::CalculateSteering(deltaT, pAgent);
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	Elite::Vector2 currentVector{};

	SteeringOutput steering{};

	const int nrOfNeighbors{ m_pGroup->GetNrOfNeighbors() };

	for (int index{}; index < nrOfNeighbors; ++index)
	{
		currentVector = pAgent->GetPosition() - m_pGroup->GetNeighbors()[index]->GetPosition();
		steering.LinearVelocity += currentVector / currentVector.MagnitudeSquared();
	}

	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	return steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	if (m_pGroup->GetNrOfNeighbors() > 0)
	{
		steering.LinearVelocity = m_pGroup->GetAverageNeighborVelocity();
	}

	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	return steering;
}