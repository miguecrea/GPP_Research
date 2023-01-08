#include "stdafx.h"
#include "Group.h"

#include "UnitAgent.h"
#include "../SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "../SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
#include "../SteeringBehaviors/Flocking/FlockingSteeringBehaviors.h"

using namespace Elite;

//Constructor & Destructor
Group::Group(
	int maxGroupSize ) :m_MaxGroupSize{ maxGroupSize }
{
	m_pAgents.reserve(m_MaxGroupSize);
	m_pNeighbors.reserve(m_MaxGroupSize);

	//Behaviours
	m_pSeekBehavior = new Seek();
	m_pSeekABehavior = new Seek();
	m_pSeekBBehavior = new Seek();
	m_pEvadeBehavior = new Evade();
	m_pArriveBehavior = new Arrive();
	
	m_pCohesionBehavior = new Cohesion(this);
	m_pSeparationBehavior = new Separation(this);
	m_pVelMatchBehavior = new VelocityMatch(this);
	m_pBlendedSteering = new BlendedSteering({ { m_pSeparationBehavior,4.f }, { m_pSeekBehavior,10.f },{ m_pSeekABehavior,5.f }, { m_pSeekBBehavior,5.f }, { m_pVelMatchBehavior,1.f } });
	//m_pBlendedSteering = new BlendedSteering({ { m_pSeparationBehavior,4.f }, { m_pSeekBehavior,10.f },{ m_pSeekABehavior,5.f }, { m_pSeekBBehavior,5.f }, { m_pVelMatchBehavior,1.f } }); //Nice but they split up when moving
	m_pPrioritySteering = new PrioritySteering({ m_pSeparationBehavior,m_pArriveBehavior });
}

Group::~Group()
{
	SAFE_DELETE(m_pSeekBehavior);
	SAFE_DELETE(m_pSeekABehavior);
	SAFE_DELETE(m_pSeekBBehavior);
	SAFE_DELETE(m_pEvadeBehavior);
	SAFE_DELETE(m_pArriveBehavior);
	SAFE_DELETE(m_pCohesionBehavior);
	SAFE_DELETE(m_pSeparationBehavior);
	SAFE_DELETE(m_pPrioritySteering);
	SAFE_DELETE(m_pBlendedSteering);
	SAFE_DELETE(m_pVelMatchBehavior);

	m_pAgents.clear();
}

void Group::Update(float deltaT)
{
	if (m_CurrentFormation == Formation::Line)
	{
		Elite::Vector2 A{ 30.f,30.f };
		Elite::Vector2 B{ -30.f, -30.f };
	
		if (m_vPath.size() > 0)
		{
			Elite::Vector2 targetPos{ m_vPath[0] };

			A += targetPos;
			B += targetPos;
		}

		DEBUGRENDERER2D->DrawPoint(A, 5.f, Color{ 1.f,0.f,0.f }, -1);
		DEBUGRENDERER2D->DrawPoint(B, 5.f, Color{ 1.f,0.f,0.f }, -1);

		Elite::Vector2 AB{ B - A };
	
		for (UnitAgent* pAgent : m_pAgents)
		{
			RegisterNeighbors(pAgent);
	
			Elite::Vector2 P = pAgent->GetPosition();
	
			float t = (-P.x * AB.x + A.x * AB.x - P.y * AB.y + A.y * AB.y) / (-AB.x * AB.x - AB.y * AB.y);
			
			t = (std::min)(1.f, t);
			t = std::max(0.f, t);
			
			m_pArriveBehavior->SetTarget(A + AB * t);
			m_pSeekBehavior->SetTarget(A + AB * t);

			if (t < 0.5f)
			{
				m_pBlendedSteering->GetWeightedBehaviorsRef()[2].weight = t;
				m_pBlendedSteering->GetWeightedBehaviorsRef()[3].weight = 2.f*(1.f-t);
			}
			else
			{
				m_pBlendedSteering->GetWeightedBehaviorsRef()[2].weight = 2.f*t;
				m_pBlendedSteering->GetWeightedBehaviorsRef()[3].weight = 1.f-t;
			}


			m_pSeekABehavior->SetTarget(A);
			m_pSeekBBehavior->SetTarget(B);

			pAgent->SetSteeringBehavior(m_pBlendedSteering);
	
			pAgent->Update(deltaT);
		}
	}

	//if (m_vPath.size() > 0)
	//{
	//	if (m_vPath.size() == 1)
	//	{
	//		//We have reached the last node
	//		for (UnitAgent* pAgent : m_pAgents)
	//		{
	//			//Individual goal
	//			m_pArriveBehavior->SetTarget(m_vPath[0]);
	//			pAgent->SetSteeringBehavior(m_pArriveBehavior);
	//			pAgent->Update(deltaT);
	//		}
	//	}
	//	else
	//	{
	//		//Move to the next node
	//		for (UnitAgent* pAgent : m_pAgents)
	//		{
	//			pAgent->SetSteeringBehavior(m_pBlendedSteering);
	//		}
	//		m_pSeekBehavior->SetTarget(m_vPath[0] + Vector2{ 3.f,3.f });
	//		m_pEvadeBehavior->SetTarget(m_vPath[0]);
	//	}
	//
	//	if (Elite::DistanceSquared(GetCenterPos(), m_vPath[0]) < m_UnitSpace * m_UnitSpace)
	//	{
	//		//If we reached the next point of the path. Remove it 
	//		m_vPath.erase(std::remove(m_vPath.begin(), m_vPath.end(), m_vPath[0]));
	//	}
	//}
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
			pAgent->SetBodyColor(m_LightBlue);
		}
	}
}

void Group::RemoveUnitFromGroup(UnitAgent* pAgent)
{
	if (static_cast<int>(m_pAgents.size()) > 0)
	{
		m_pAgents.erase(std::remove(m_pAgents.begin(), m_pAgents.end(), pAgent), m_pAgents.end());
		pAgent->SetBodyColor(Color{ 1.f,1.f,0.f });
		pAgent->SetSteeringBehavior(nullptr);
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

std::vector<Elite::Vector2> Group::GetPath() const
{
	return m_vPath;
}

void Group::CalculatePath(const Vector2& destiation,NavGraph* pNavGraph,std::vector<Vector2>& debugNodePositions,std::vector<Portal>& portals,std::vector<Vector2>& visitedNodePositions)
{
	m_vPath = NavMeshPathfinding::FindPath(GetCenterPos(), destiation, pNavGraph, debugNodePositions, portals, visitedNodePositions);
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

void Group::SetTarget_Seek(TargetData target)
{
	m_pSeekBehavior->SetTarget(target);
}


