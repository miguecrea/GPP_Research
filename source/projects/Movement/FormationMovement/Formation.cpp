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
	m_pCommandingGroup = nullptr;
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
	m_CurrentCenter = CalculateCenterPos();

	if (m_Path.size() > 0)
	{
		targetPos = m_Path[0];

		if (Elite::DistanceSquared(m_CurrentCenter, m_Path[0]) < m_UnitSpace * m_UnitSpace)
		{
			//If we reached the next point of the path. Remove it 
			m_Path.erase(std::remove(m_Path.begin(), m_Path.end(), m_Path[0]));
		}
	}

	UpdateGroupMovements(deltaT, targetPos);
}

void Formation::Render(float deltaT)
{
	DEBUGRENDERER2D->DrawPoint(m_CurrentCenter, 5.f, Elite::Color{ 0.f,0.f,1.f }, -0.9f);
	DEBUGRENDERER2D->DrawPoint(m_DesiredFormationCenter, 5.f, Elite::Color{ 1.f,0.f,0.f }, -0.8f);
}

void Formation::CalculatePath(const Elite::Vector2& destiation, Elite::NavGraph* pNavGraph, std::vector<Elite::Vector2>& debugNodePositions, std::vector<Elite::Portal>& portals, std::vector<Elite::Vector2>& visitedNodePositions)
{
	m_CurrentCenter = CalculateCenterPos();

	//Not working 100% correctly: cropping the portals
	const float formationOffset{ m_DesiredFormationRightVector.Magnitude() + 0.5f };
	//m_Path = Elite::NavMeshPathfinding::FindPath(m_CurrentCenter, destiation, pNavGraph, debugNodePositions, portals, visitedNodePositions, formationOffset + m_UnitSpace);

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
			pGroup->Update(deltaT, targetPos - static_cast<float>(counter) * forward * m_UnitSpace * 2.f);

			++counter;
		}
		break;
	case FormationType::Circle:
		for (Group* pGroup : m_pGroups)
		{
			pGroup->SetFormation(m_DesiredFormationCenter, m_DesiredFormationRightVector - ((right * m_DesiredFormationRightVector.Magnitude()) / static_cast<float>(m_NrGroups)) * static_cast<float>(counter), m_CurrentFormation, m_IsLooseMovement);
			pGroup->Update(deltaT, targetPos);

			++counter;
		}
		break;
	case FormationType::Arrow:
	{
		const Elite::Vector2 top{ forward * m_DesiredFormationRightVector.Magnitude() / 2.f };
		const Elite::Vector2 rightPoint{ m_DesiredFormationRightVector / 2.f };

		Elite::Vector2 rightSide{ top - rightPoint };
		Elite::Vector2 leftSide{ top + rightPoint };

		for (Group* pGroup : m_pGroups)
		{
			//Commander
			if (counter == 0)
			{
				pGroup->SetFormation(m_DesiredFormationCenter, right * m_UnitSpace, FormationType::Line, m_IsLooseMovement);
				pGroup->Update(deltaT, targetPos);
			}
			//Right
			else if (counter == 1)
			{
				pGroup->SetFormation(m_DesiredFormationCenter, -rightSide, FormationType::Line, m_IsLooseMovement);
				pGroup->Update(deltaT, targetPos - rightSide + m_UnitSpace * right);
			}
			//Left
			else
			{
				pGroup->SetFormation(m_DesiredFormationCenter,  leftSide, FormationType::Line, m_IsLooseMovement);
				pGroup->Update(deltaT, targetPos - leftSide - m_UnitSpace * right);
			}
			++counter;
		}
	}
		break;
	}
}

void Formation::SetFormation(const Elite::Vector2& desiredCenter, const Elite::Vector2& desiredRightVector, FormationType formation, bool isLooseMovement, int nrGroups)
{
	m_DesiredFormationCenter = desiredCenter;
	m_DesiredFormationRightVector = desiredRightVector;
	m_CurrentFormation = formation;
	m_IsLooseMovement = isLooseMovement;

	//Minimum number is 1
	m_NrGroups = std::max(1, nrGroups);

	//Reset agents
	for (UnitAgent* pAgent : m_pAgents)
	{
		pAgent->SetGroup(nullptr);
	}

	Elite::Vector2 right{ m_DesiredFormationRightVector.GetNormalized() };
	Elite::Vector2 forward{ -right.y,right.x };

	RegisterOffsetFromCenter(m_CurrentCenter, forward, right);

	//Reset groups
	for (Group* pGroup : m_pGroups)
	{
		delete pGroup;
		pGroup = nullptr;
	}

	m_pGroups.clear();
	m_pCommandingGroup = nullptr;

	//Create new groups
	for (int index{}; index < nrGroups; ++index)
	{
		m_pGroups.push_back(new Group(&m_pAgents, m_MaxGroupSize));
	}

	//Set formations
	if (m_CurrentFormation == FormationType::Line || m_CurrentFormation == FormationType::Circle)
	{
		SetBasicFormation();
	}
	else if (m_CurrentFormation == FormationType::Arrow)
	{
		SetCustomFormation();
	}
}

//////////////////////////////////////////////////
//  Agents
/////////////////////////////////////////////////

void Formation::RegisterOffsetFromCenter(const Elite::Vector2& centerPos, const Elite::Vector2& forward, const Elite::Vector2& right)
{
	for (UnitAgent* pAgent : m_pAgents)
	{
		pAgent->SetOffset(pAgent->GetPosition() - centerPos);
		pAgent->CalculateForwardness(forward);
		pAgent->CalculateRightness(right);
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

void Formation::SortAgentsBasedOnRightness()
{
	auto isMoreRight = [&](UnitAgent* first, UnitAgent* second)->float
	{
		return first->GetRightness() > second->GetRightness();
	};

	std::sort(m_pAgents.begin(), m_pAgents.end(), isMoreRight);
}

void Formation::SortAgentsBasedOnDistanceDecreasing()
{
	auto isFurther = [&](UnitAgent* first, UnitAgent* second)->float
	{
		return first->GetOffset().MagnitudeSquared() > second->GetOffset().MagnitudeSquared();
	};

	std::sort(m_pAgents.begin(), m_pAgents.end(), isFurther);
}

void Formation::SortAgentsBasedOnDistanceIncreasing()
{
	auto isNearer = [&](UnitAgent* first, UnitAgent* second)->float
	{
		return first->GetOffset().MagnitudeSquared() < second->GetOffset().MagnitudeSquared();
	};

	std::sort(m_pAgents.begin(), m_pAgents.end(), isNearer);
}

////////////////////////////////////////////////////
//  Other Helper functions
///////////////////////////////////////////////////

Elite::Vector2 Formation::CalculateCenterPos() const
{
	Elite::Vector2 centerPos{};

	//Calculate center of all groups
	if (m_pCommandingGroup == nullptr)
	{
		if (m_pGroups.size() > 0)
		{
			for (const Group* pGroup : m_pGroups)
			{
				centerPos += pGroup->GetCenterPos();
			}

			centerPos /= static_cast<float>(m_pGroups.size());
		}
	}
	else
	{
		centerPos = m_pCommandingGroup->GetCenterPos();
	}

	return centerPos;
}

std::vector<Elite::Vector2> Formation::GetPath() const
{
	return m_Path;
}

void Formation::SetBasicFormation()
{
	//Choose a sorting and commander
	switch (m_CurrentFormation)
	{
	case FormationType::Line:
		SortAgentsBasedOnForwardness();
		m_pCommandingGroup = m_pGroups[0];
		break;
	case FormationType::Circle:
		SortAgentsBasedOnDistanceDecreasing();
		break;
	}

	//Place in sub formations
	int nrUnitsPerGroup{ static_cast<int>(ceil(static_cast<float>(m_pAgents.size()) / m_NrGroups)) };
	int totalIndex{};

	for (Group* pGroup : m_pGroups)
	{
		for (int index{}; index < nrUnitsPerGroup; ++index)
		{
			if (totalIndex == m_pAgents.size()) break;

			pGroup->AddUnitToGroup(m_pAgents[totalIndex]);
			++totalIndex;
		}
	}
}

void Formation::SetCustomFormation()
{
	m_pCommandingGroup = m_pGroups[0];
	SortAgentsBasedOnForwardness();

	//The nearest agent
	if (m_pAgents.size() > 0)
	{
		m_pGroups[0]->AddUnitToGroup(m_pAgents[0]);
	}

	SortAgentsBasedOnRightness();

	//Place in sub formations
	int nrUnitsPerGroup{ static_cast<int>(ceil(static_cast<float>(m_pAgents.size()) / 2)) };
	int totalIndex{};

	for (Group* pGroup : m_pGroups)
	{
		if (pGroup == m_pGroups[0]) continue;

		for (int index{}; index < nrUnitsPerGroup; ++index)
		{
			if (totalIndex == m_pAgents.size()) break;

			//The commander can't be assigned to both groups
			if (m_pAgents[totalIndex]->GetGroup() == nullptr)
			{
				pGroup->AddUnitToGroup(m_pAgents[totalIndex]);
			}

			++totalIndex;
		}
	}
}

