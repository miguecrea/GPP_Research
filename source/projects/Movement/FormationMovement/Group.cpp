#include "stdafx.h"
#include "Group.h"

#include "UnitAgent.h"
#include "../SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "../SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"


using namespace Elite;

//Constructor & Destructor
Group::Group(
	int maxGroupSize ) :m_MaxGroupSize{ maxGroupSize }
{
	m_pAgents.reserve(m_MaxGroupSize);

	//Behaviours
	m_pSeekBehavior = new Seek();
	m_pEvadeBehavior = new Evade();
	m_pArriveBehavior = new Arrive();
	m_pBlendedSteering = new BlendedSteering({ { m_pEvadeBehavior,0.f }, { m_pSeekBehavior,1.f } });
	m_pPrioritySteering = new PrioritySteering({m_pEvadeBehavior,m_pSeekBehavior });
}

Group::~Group()
{
	SAFE_DELETE(m_pEvadeBehavior);
	SAFE_DELETE(m_pSeekBehavior);
	SAFE_DELETE(m_pArriveBehavior);
	//SAFE_DELETE(m_pCohesionBehavior);
	//SAFE_DELETE(m_pSeparationBehavior);
	SAFE_DELETE(m_pPrioritySteering);
	SAFE_DELETE(m_pBlendedSteering);
	//SAFE_DELETE(m_pVelMatchBehavior);

	m_pAgents.clear();
}

void Group::Update(float deltaT)
{
	//m_pEvadeBehavior->SetEvadeRadius(m_EvadeRadius);

	if (m_vPath.size() > 0)
	{
		if (m_vPath.size() == 1)
		{
			//We have reached the last node
			for (UnitAgent* pAgent : m_pAgents)
			{
				pAgent->SetSteeringBehavior(m_pArriveBehavior);
			}

			m_pArriveBehavior->SetTarget(m_vPath[0]);
		}
		else
		{
			//Move to the next node
			for (UnitAgent* pAgent : m_pAgents)
			{
				pAgent->SetSteeringBehavior(m_pBlendedSteering);
			}
			m_pSeekBehavior->SetTarget(m_vPath[0] + Vector2{ 3.f,3.f });
			m_pEvadeBehavior->SetTarget(m_vPath[0]);
		}

		if (Elite::DistanceSquared(GetCenterPos(), m_vPath[0]) < m_UnitSpace * m_UnitSpace)
		{
			//If we reached the next point of the path. Remove it 
			m_vPath.erase(std::remove(m_vPath.begin(), m_vPath.end(), m_vPath[0]));
		}
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
			pAgent->SetBodyColor(m_Blue);
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

