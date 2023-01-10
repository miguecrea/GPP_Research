#include "stdafx.h"
#include "Formation.h"
#include "Group.h"

#include "UnitAgent.h"
#include "../SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "../SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
#include "../SteeringBehaviors/Flocking/FlockingSteeringBehaviors.h"

Formation::Formation(
	int maxGroupSize) :m_MaxGroupSize{ maxGroupSize }
{
	m_pAgents.reserve(m_MaxGroupSize);
	m_NrGroups = 1;
}

Formation::~Formation()
{
	m_pAgents.clear();

	for (Group* pGroup : m_pGroups)
	{
		SAFE_DELETE(pGroup);
		pGroup = nullptr;
	}
}

void Formation::Update(float deltaT)
{
	Elite::Vector2 targetPos{ m_DesiredFormationCenter };

	if (m_Path.size() > 0)
	{
		targetPos = m_Path[0];

		m_CurrentCenter = CalculateCenterPos();

		if (Elite::DistanceSquared(m_CurrentCenter, m_Path[0]) < m_UnitSpace * m_UnitSpace)
		{
			//If we reached the next point of the path. Remove it 
			m_Path.erase(std::remove(m_Path.begin(), m_Path.end(), m_Path[0]));
		}
	}

	UpdateGroupMovements(deltaT, targetPos);
}

void Formation::CalculatePath(const Elite::Vector2& destiation, Elite::NavGraph* pNavGraph, std::vector<Elite::Vector2>& debugNodePositions, std::vector<Elite::Portal>& portals, std::vector<Elite::Vector2>& visitedNodePositions)
{
	m_CurrentCenter = CalculateCenterPos();

	const float formationOffset{ m_DesiredFormationRightVector.Magnitude() + 0.5f };

	//Not working 100% correctly -> crop the portals
	//m_Path = Elite::NavMeshPathfinding::FindPath(currentCenter, destiation, pNavGraph, debugNodePositions, debugPortals, visitedNodePositions, formationOffset + m_UnitSpace);

	m_Path = Elite::NavMeshPathfinding::FindPath(m_CurrentCenter, destiation, pNavGraph, debugNodePositions, portals, visitedNodePositions);

	if (m_Path.size() > 0)
	{
		Elite::Vector2 direction{};
		Elite::Vector2 rotatedDirection{};
		Elite::Vector2 previousDirection{ m_Path[0] - m_CurrentCenter };
		previousDirection.Normalize();

		//Don't move the start and end positions
		for (int index{ 0 }; index < static_cast<int>(m_Path.size()) - 1; ++index)
		{
			direction = m_Path[index + 1] - m_Path[index];
			direction.Normalize();

			rotatedDirection.x = -previousDirection.y;
			rotatedDirection.y = previousDirection.x;

			//Left or right
			float crossPrevious{ Cross(direction, previousDirection) };

			m_Path[index] += (crossPrevious > 0.f ? 1.f : -1.f) * rotatedDirection * formationOffset;

			previousDirection = direction;
		}
	}
}

void Formation::AddUnitToFormation(UnitAgent* pAgent)
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

void Formation::RemoveUnitFromFormation(UnitAgent* pAgent)
{
	if (static_cast<int>(m_pAgents.size()) > 0)
	{
		m_pAgents.erase(std::remove(m_pAgents.begin(), m_pAgents.end(), pAgent), m_pAgents.end());
		pAgent->SetBodyColor(Elite::Color{ 1.f,1.f,0.f });
		pAgent->SetSteeringBehavior(nullptr);
		pAgent->SetLinearVelocity(Elite::Vector2{ 0.f,0.f });
		pAgent->SetAngularVelocity(0.f);
		pAgent->SetGroup(nullptr);
	}
}

void Formation::UpdateGroupMovements(float deltaT, const Elite::Vector2& targetPos)
{
	Elite::Vector2 right{ m_DesiredFormationRightVector.GetNormalized() };
	Elite::Vector2 forward{ -right.y,right.x };

	//Update groups
	int counter{};

	switch (m_CurrentFormation)
	{
	case FormationType::Line:
		for (Group* pGroup : m_pGroups)
		{
			pGroup->SetFormation(m_DesiredFormationCenter, m_DesiredFormationRightVector, m_CurrentFormation, m_IsLooseMovement);
			pGroup->Update(deltaT, m_DesiredFormationCenter - counter * forward * m_UnitSpace * 2.f);

			++counter;
		}
		break;
	case FormationType::Circle:
		for (Group* pGroup : m_pGroups)
		{
			pGroup->SetFormation(m_DesiredFormationCenter, m_DesiredFormationRightVector - ((right * m_DesiredFormationRightVector.Magnitude()) / m_NrGroups) * counter, m_CurrentFormation, m_IsLooseMovement);
			pGroup->Update(deltaT, m_DesiredFormationCenter);

			++counter;
		}
		break;

	}
}

void Formation::RegisterOffsetFromCenter(const Elite::Vector2& centerPos, const Elite::Vector2& forward)
{
	for (UnitAgent* pAgent : m_pAgents)
	{
		pAgent->SetOffset(pAgent->GetPosition() - centerPos);
		pAgent->CalculateForwardness(forward);
	}
}

void Formation::SortAgentsBasedOnForwardness()
{
	auto isMoreForward = [&](UnitAgent* first, UnitAgent* second)->float
	{
		return first->GetForwardness() > second->GetForwardness();
	};

	std::sort(m_pAgents.begin(), m_pAgents.end(), isMoreForward);
}

void Formation::SortAgentsBasedOnDistance()
{
	auto isNearer = [&](UnitAgent* first, UnitAgent* second)->float
	{
		return first->GetOffset().MagnitudeSquared() > second->GetOffset().MagnitudeSquared();
	};

	std::sort(m_pAgents.begin(), m_pAgents.end(), isNearer);
}

Elite::Vector2 Formation::CalculateCenterPos() const
{
	Elite::Vector2 centerPos{};

	if (m_pGroups.size() > 0)
	{
		for (const Group* pGroup : m_pGroups)
		{
			centerPos += pGroup->GetCenterPos();
		}

		centerPos /= static_cast<float>(m_pGroups.size());
	}

	return centerPos;
}

std::vector<Elite::Vector2> Formation::GetPath() const
{
	return m_Path;
}

void Formation::SetFormation(const Elite::Vector2& desiredCenter, const Elite::Vector2& desiredRightVector, FormationType formation, bool isLooseMovement, int nrGroups)
{
	m_DesiredFormationCenter = desiredCenter;
	m_DesiredFormationRightVector = desiredRightVector;
	m_CurrentFormation = formation;
	m_IsLooseMovement = isLooseMovement;

	//Minimum number is 1
	m_NrGroups = std::max(1, nrGroups);

	for (Group* pGroup : m_pGroups)
	{
		delete pGroup;
		pGroup = nullptr;
	}

	m_pGroups.clear();

	for (int index{}; index < nrGroups; ++index)
	{
		m_pGroups.push_back(new Group(m_MaxGroupSize));
	}

	Elite::Vector2 right{ m_DesiredFormationRightVector.GetNormalized() };
	Elite::Vector2 forward{ -right.y,right.x };

	RegisterOffsetFromCenter(m_CurrentCenter, forward);

	//Choose a sorting
	int nrUnitsPerGroup{ static_cast<int>(ceil(static_cast<float>(m_pAgents.size()) / m_NrGroups)) };
	int startIndex{};
	int totalIndex{};

	switch (formation)
	{
	case FormationType::Line:
		SortAgentsBasedOnForwardness();
		break;
	case FormationType::Circle:
		SortAgentsBasedOnDistance();
		break;
	}

	//Place in sub formations
	for (Group* pGroup : m_pGroups)
	{
		for (size_t index{}; index < m_pAgents.size(); ++index)
		{
			pGroup->RemoveUnitFromGroup(m_pAgents[index]);
		}

		for (int index{}; index < nrUnitsPerGroup; ++index)
		{
			if (totalIndex == m_pAgents.size()) break;

			pGroup->AddUnitToGroup(m_pAgents[totalIndex]);
			++totalIndex;
		}
	}
}
