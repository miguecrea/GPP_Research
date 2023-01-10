#include "stdafx.h"
#include "Group.h"

#include "UnitAgent.h"
#include "../SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "../SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
#include "../SteeringBehaviors/Flocking/FlockingSteeringBehaviors.h"

using namespace Elite;

//Constructor & Destructor
Group::Group(int maxGroupSize ) :m_MaxGroupSize{ maxGroupSize }
{
	m_pAgents.reserve(m_MaxGroupSize);
	m_pNeighbors.resize(m_MaxGroupSize);

	//Behaviours
	m_pSeekBehavior = new Seek();
	m_pSeekABehavior = new Seek();
	m_pSeekBBehavior = new Seek();
	m_pSeekOtherSideBehavior = new Seek();
	m_pSeekDesiredLocationBehavior = new Seek();

	m_pEvadeBehavior = new Evade();
	m_pArriveBehavior = new Arrive();
	m_pFaceBehavior = new Face();

	m_pCohesionBehavior = new Cohesion(this);
	m_pSeparationBehavior = new Separation(this);
	m_pVelMatchBehavior = new VelocityMatch(this);
	m_pPrioritySteering = new PrioritySteering({ m_pSeparationBehavior,m_pArriveBehavior });

	//Circle
	m_pBlendedCircleFormAtLocationSteering = new BlendedSteering({ { m_pSeparationBehavior,4.f }, {m_pSeekBehavior,10.f}, {m_pVelMatchBehavior,1.f},{m_pFaceBehavior,2.f} });
	m_pBlendedCircleSteering = new BlendedSteering({ { m_pSeparationBehavior,4.f }, {m_pSeekBehavior,10.f},{m_pSeekDesiredLocationBehavior,10.f}, {m_pVelMatchBehavior,1.f},{m_pFaceBehavior,2.f} });
	//Line
	m_pBlendedLineFormAtLocationSteering = new BlendedSteering({ { m_pSeparationBehavior,4.f }, { m_pSeekBehavior,10.f },{ m_pSeekABehavior,2.f }, { m_pSeekBBehavior,2.f }, { m_pVelMatchBehavior,1.f },{m_pFaceBehavior,2.f } });
	m_pBlendedLineSteering = new BlendedSteering({ { m_pSeparationBehavior,4.f }, { m_pSeekBehavior,10.f },{ m_pSeekABehavior,0.f }, { m_pSeekBBehavior,0.f },{m_pSeekDesiredLocationBehavior,15.f}, { m_pVelMatchBehavior,1.f },{m_pFaceBehavior,2.f } });
}

Group::~Group()
{
	SAFE_DELETE(m_pSeekBehavior);
	SAFE_DELETE(m_pSeekABehavior);
	SAFE_DELETE(m_pSeekBBehavior);
	SAFE_DELETE(m_pSeekOtherSideBehavior);
	SAFE_DELETE(m_pSeekDesiredLocationBehavior);
	SAFE_DELETE(m_pEvadeBehavior);
	SAFE_DELETE(m_pArriveBehavior);
	SAFE_DELETE(m_pCohesionBehavior);
	SAFE_DELETE(m_pSeparationBehavior);
	SAFE_DELETE(m_pPrioritySteering);
	SAFE_DELETE(m_pBlendedLineSteering);
	SAFE_DELETE(m_pBlendedLineFormAtLocationSteering);
	SAFE_DELETE(m_pBlendedCircleSteering);
	SAFE_DELETE(m_pVelMatchBehavior);
	SAFE_DELETE(m_pFaceBehavior);
	SAFE_DELETE(m_pBlendedCircleFormAtLocationSteering);
}

void Group::Update(float deltaT, const Elite::Vector2& targetPos)
{
	Elite::Vector2 averageGroupPos{ GetCenterPos() };
	Elite::Vector2 right{ m_DesiredRightVector.GetNormalized() };
	Elite::Vector2 forward{ -right.y,right.x };

	switch (m_CurrentFormation)
	{
	case FormationType::Line:
	{
		Elite::Vector2 A{ targetPos - m_DesiredRightVector };
		Elite::Vector2 B{ targetPos + m_DesiredRightVector };

		DEBUGRENDERER2D->DrawSegment(averageGroupPos, averageGroupPos + forward * 2.f, Color{ 1.f,0.f,0.f }, -1);
		DEBUGRENDERER2D->DrawPoint(A, 5.f, Color{ 1.f,0.f,0.f }, -1);
		DEBUGRENDERER2D->DrawPoint(B, 5.f, Color{ 1.f,0.f,0.f }, -1);

		if (m_IsLooseMovement)
		{
			Elite::Vector2 AB{ B - A };

			for (UnitAgent* pAgent : m_pAgents)
			{
				RegisterNeighbors(pAgent);

				Elite::Vector2 P{ pAgent->GetPosition() };

				//Dot(QA,QP)=0 -> perpendicular
				float t = (-P.x * AB.x + A.x * AB.x - P.y * AB.y + A.y * AB.y) / (-AB.x * AB.x - AB.y * AB.y);

				t = (std::min)(1.f, t);
				t = std::max(0.f, t);

				m_pSeekBehavior->SetTarget(A + AB * t);

				if (t < 0.5f)
				{
					//Seek A and Seek B
					m_pBlendedLineSteering->GetWeightedBehaviorsRef()[2].weight = t;
					m_pBlendedLineSteering->GetWeightedBehaviorsRef()[3].weight = 3.f * (1.f - t);
				}
				else
				{
					//Seek A and Seek B
					m_pBlendedLineSteering->GetWeightedBehaviorsRef()[2].weight = 3.f * t;
					m_pBlendedLineSteering->GetWeightedBehaviorsRef()[3].weight = 1.f - t;
				}


				m_pSeekABehavior->SetTarget(A);
				m_pSeekBBehavior->SetTarget(B);

				//Rotate 90 degrees -> (-y,x)
				m_pFaceBehavior->SetTarget(Elite::Vector2{ P.x - AB.y, P.y + AB.x });

				pAgent->SetSteeringBehavior(m_pBlendedLineFormAtLocationSteering);

				pAgent->Update(deltaT);
			}
		}
		else
		{
			int m_NrLines{ 1 };
			int allowedNrAgentsInFormation{ static_cast<int>(ceil(static_cast<float>(m_pAgents.size()) / m_NrLines)) };
			int adderPerLine{ allowedNrAgentsInFormation };
			int nrAgentsInFormation{};
	
			//Support for: max 2 lines + equal number of agents in each line
			int currentNrLines{ 1 };
			float lineOffset{ 2.f * m_UnitSpace };
	
			Elite::Vector2 startOffset{ static_cast<float>(m_NrLines / 2) * forward * lineOffset / 2.f };
			A += startOffset;
			Elite::Vector2 currentA{ GetCenterPos() - m_DesiredRightVector + startOffset };
			Elite::Vector2 currentB{ GetCenterPos() + m_DesiredRightVector + startOffset };
			Elite::Vector2 AB{ currentB - currentA };
	
			for (UnitAgent* pAgent : m_pAgents)
			{
				RegisterNeighbors(pAgent);
	
				Elite::Vector2 P{ pAgent->GetPosition() };
	
				//Dot(QA,QP)=0 -> perpendicular
				float t = (-P.x * AB.x + currentA.x * AB.x - P.y * AB.y + currentA.y * AB.y) / (-AB.x * AB.x - AB.y * AB.y);
	
				t = (std::min)(1.f, t);
				t = std::max(0.f, t);
	
				m_pSeekBehavior->SetTarget(A + AB * t);
				m_pSeekDesiredLocationBehavior->SetTarget(currentA + AB * t);
	
				//Rotate 90 degrees -> (-y,x)
				m_pFaceBehavior->SetTarget(Elite::Vector2{ P.x - AB.y, P.y + AB.x });
	
				pAgent->SetSteeringBehavior(m_pBlendedLineSteering);
	
				pAgent->Update(deltaT);
	
				++nrAgentsInFormation;
	
				if (nrAgentsInFormation == allowedNrAgentsInFormation)
				{
					A -= forward * lineOffset;
					currentA -= forward * lineOffset;
					currentB -= forward * lineOffset;
	
					allowedNrAgentsInFormation += adderPerLine;
				}
			}
		}
	}
	break;
	case FormationType::Circle:
	{
		const float radius{ m_DesiredRightVector.Magnitude() };
		const float averageDistanceSquared{ (targetPos - averageGroupPos).MagnitudeSquared() };

		DEBUGRENDERER2D->DrawPoint(targetPos, 5.f, Color{ 1.f,1.f,0.f }, -0.9f);
		DEBUGRENDERER2D->DrawCircle(targetPos, radius, Color{ 1.f,1.f,0.f }, -0.9f);

		DEBUGRENDERER2D->DrawPoint(m_DesiredCenter, 5.f, Color{ 1.f,0.f,0.f }, -1.f);
		DEBUGRENDERER2D->DrawCircle(m_DesiredCenter, radius, Color{ 1.f,0.f,0.f }, -1.f);

		for (UnitAgent* pAgent : m_pAgents)
		{
			RegisterNeighbors(pAgent);

			Elite::Vector2 P{ pAgent->GetPosition() };
			Elite::Vector2 currentDirection{ P - averageGroupPos };

			currentDirection.Normalize();

			if (m_IsLooseMovement)
			{
				m_pSeekBehavior->SetTarget(targetPos + currentDirection * radius);
				m_pFaceBehavior->SetTarget(P + currentDirection);

				pAgent->SetSteeringBehavior(m_pBlendedCircleFormAtLocationSteering);
			}
			else
			{
				m_pSeekBehavior->SetTarget(targetPos + currentDirection * radius);
				m_pSeekDesiredLocationBehavior->SetTarget(averageGroupPos + currentDirection * radius);
				m_pFaceBehavior->SetTarget(P + currentDirection);

				pAgent->SetSteeringBehavior(m_pBlendedCircleSteering);
			}

			pAgent->Update(deltaT);
		}
	}
	break;
	}
}

void Group::Render(float deltaT)
{

}

void Group::AddUnitToGroup(UnitAgent* pAgent)
{
	if (static_cast<int>(m_pAgents.size()) < m_MaxGroupSize)
	{
		if (std::find(m_pAgents.begin(), m_pAgents.end(), pAgent) == m_pAgents.end())
		{
			m_pAgents.emplace_back(pAgent);
			pAgent->SetGroup(this);
		}
	}
}

void Group::RemoveUnitFromGroup(UnitAgent* pAgent)
{
	if (static_cast<int>(m_pAgents.size()) > 0)
	{
		m_pAgents.erase(std::remove(m_pAgents.begin(), m_pAgents.end(), pAgent), m_pAgents.end());
		pAgent->SetGroup(nullptr);
	}
}

Elite::Vector2 Group::GetCenterPos() const
{
	Vector2 centerPos{};

	for (UnitAgent* pAgent: m_pAgents)
	{
		centerPos += pAgent->GetPosition();
	}

	if (m_pAgents.size() > 0)
	{
		return centerPos /= static_cast<float>(m_pAgents.size());
	}

	return centerPos;
}

Elite::Vector2 Group::GetAverageVelocity() const
{
	Vector2 averageVelocity{};

	for (UnitAgent* pAgent : m_pAgents)
	{
		averageVelocity += pAgent->GetLinearVelocity();
	}

	return averageVelocity / static_cast<float>(m_pAgents.size());
}

void Group::RegisterNeighbors(UnitAgent* pAgent)
{
	m_NrOfNeighbors = 0;

	for (UnitAgent* pOtherAgent : m_pAgents)
	{
		if ((pAgent != pOtherAgent) && (pOtherAgent != nullptr) && (Distance(pAgent->GetPosition(), pOtherAgent->GetPosition()) < pAgent->GetNeighborhoodRadius()))
		{
			m_pNeighbors[m_NrOfNeighbors] = pOtherAgent;
			++m_NrOfNeighbors;
		}
	}
}

Elite::Vector2 Group::GetAverageNeighborPos() const
{
	Vector2 averagePosition{};

	for (int index{}; index < m_NrOfNeighbors; ++index)
	{
		averagePosition += m_pNeighbors[index]->GetPosition();
	}

	return averagePosition /= static_cast<float>(m_NrOfNeighbors);
}

Elite::Vector2 Group::GetAverageNeighborVelocity() const
{
	Vector2 averageVelocity{};

	for (int index{}; index < m_NrOfNeighbors; ++index)
	{
		averageVelocity += m_pNeighbors[index]->GetLinearVelocity();
	}

	return averageVelocity / static_cast<float>(m_NrOfNeighbors);
}

void Group::SetFormation(const Elite::Vector2& desiredCenter, const Elite::Vector2& desiredRightVector, FormationType formation, bool isLooseMovement)
{
	m_DesiredCenter = desiredCenter;
	m_DesiredRightVector = desiredRightVector;
	m_CurrentFormation = formation;
	m_IsLooseMovement = isLooseMovement;
}
