//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "SteeringBehaviors.h"
#include "../SteeringAgent.h"
#include "../Obstacle.h"
#include "framework\EliteMath\EMatrix2x3.h"
#include "../Flocking/Flock.h"

using namespace Elite;

//SEEK
//****
SteeringOutput Seek::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5, { 0,1,0 });
	}

	return steering;
}

//FLEE
//****
SteeringOutput Flee::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	steering.LinearVelocity = pAgent->GetPosition() - m_Target.Position;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5, { 0,1,0 });
	}

	return steering;
}

//ARRIVE
//****
SteeringOutput Arrive::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	const float stopDistance{ 2.f };
	const float slowRadius{ 15.f };
	SteeringOutput steering = {};

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();

	const float distance = steering.LinearVelocity.Magnitude();

	if (distance > stopDistance)
	{
		steering.LinearVelocity.Normalize();
		steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

		if (distance < slowRadius)
		{
			steering.LinearVelocity *= (distance / slowRadius);
		}
	}
	else
	{
		steering.LinearVelocity = { 0.f,0.f };
	}

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, steering.LinearVelocity.Magnitude(), { 0,1,0 });
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, distance, { 1,0,0 });
	}

	return steering;
}

//FACE
//****
SteeringOutput Face::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	const Vector2 desiredDirection{ m_Target.Position - pAgent->GetPosition() };
	const Vector2 currentDirection{ OrientationToVector(pAgent->GetRotation() + ToRadians(90.0f)) };

	const float angle{ AngleBetween(currentDirection, desiredDirection) };
	const float stopAngle{ 0.1f };

	if (abs(angle) < stopAngle)
	{
		steering.AngularVelocity = 0.f;
	}
	else
	{
		steering.AngularVelocity = (angle < 0 ? -1 : 1) * pAgent->GetMaxAngularSpeed();
	}

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), currentDirection, 5, {0,1,0});
	}

	return steering;
}

//WANDER
//****
SteeringOutput Wander::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	m_WanderAngle += randomFloat(-m_MaxAngleChange, m_MaxAngleChange);

	const Vector2 currentDirection{ pAgent->GetDirection() };
	m_Target.Position = { pAgent->GetPosition() + (currentDirection * m_OffsetDistance) + (Vector2{ cosf(m_WanderAngle),sinf(m_WanderAngle) } * m_Radius) };

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), pAgent->GetLinearVelocity(), 5, { 0,1,0 });
		DEBUGRENDERER2D->DrawPoint(m_Target.Position, 5.f, { 0,0,1 });
		DEBUGRENDERER2D->DrawCircle(pAgent->GetPosition() + (currentDirection * m_OffsetDistance), m_Radius, { 1,0,0 }, 0);
	}

	return Seek::CalculateSteering(deltaT, pAgent);
}

//PURSUIT
//****
SteeringOutput Pursuit::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	Vector2 direction{ m_Target.Position - pAgent->GetPosition() };

	float neededTime{}, step{ 0.1f };

	// afstand/snelheid = tijd nodig om bij directionPoint te geraken
	while (direction.Magnitude() / pAgent->GetMaxLinearSpeed() > neededTime && neededTime < 10)
	{
		direction += m_Target.LinearVelocity * step;
		neededTime += step;
	}

	steering.LinearVelocity = direction;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5, { 0,1,0 });
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), direction, direction.Magnitude(), {0,0,1});
	}

	return steering;
}

//EVADE
//****
SteeringOutput Evade::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	Vector2 direction{ m_Target.Position - pAgent->GetPosition() };

	float neededTime{}, step{ 0.1f };

	if (direction.Magnitude() > m_EvadeRadius)
	{
		steering.IsValid = false;
		return steering;
	}

	// distance/velocity = time needed to get to the destination
	while (direction.Magnitude() / pAgent->GetMaxLinearSpeed() > neededTime && neededTime < 10)
	{
		direction += m_Target.LinearVelocity * step;
		neededTime += step;
	}

	steering.LinearVelocity = direction;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
	steering.LinearVelocity *= -1;

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5, { 0,1,0 });
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), direction, direction.Magnitude(), { 0,0,1 });
	}

	return steering;
}