#include "stdafx.h"
#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	m_Target = (m_pFlock->GetNrOfNeighbors() > 0 ? m_pFlock->GetAverageNeighborPos() : pAgent->GetPosition());
	return Seek::CalculateSteering(deltaT, pAgent);
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	Elite::Vector2 currentVector{};

	SteeringOutput steering{};

	const int nrOfNeighbors{ m_pFlock->GetNrOfNeighbors() };

	for (int index{}; index < nrOfNeighbors; ++index)
	{
		currentVector = pAgent->GetPosition() - m_pFlock->GetNeighbors()[index]->GetPosition();
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
	if (m_pFlock->GetNrOfNeighbors() > 0)
	{
		steering.LinearVelocity = m_pFlock->GetAverageNeighborVelocity();
	}

	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	return steering;
}