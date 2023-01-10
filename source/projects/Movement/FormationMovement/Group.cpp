#include "stdafx.h"
#include "Group.h"
#include "projects//Movement/FormationMovement/SubGroup.h"

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
	m_pSeekOtherSideBehavior = new Seek();
	m_pSeekDesiredLocationBehavior = new Seek();

	m_pEvadeBehavior = new Evade();
	m_pArriveBehavior = new Arrive();
	m_pFaceBehavior = new Face();

	m_pCohesionBehavior = new Cohesion(this);
	m_pSeparationBehavior = new Separation(this);
	m_pVelMatchBehavior = new VelocityMatch(this);
	m_pPrioritySteering = new PrioritySteering({ m_pSeparationBehavior,m_pArriveBehavior });

	m_pSubGroups.resize(2);

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

	m_pAgents.clear();

	for (SubGroup* pSubGroup : m_pSubGroups)
	{
		SAFE_DELETE(pSubGroup);
		pSubGroup = nullptr;
	}
}

void Group::Update(float deltaT)
{
	Elite::Vector2 targetPos{ m_DesiredFormationCenter };

	if (m_Path.size() > 0)
	{
		targetPos = m_Path[0];

		if (Elite::DistanceSquared(GetCenterPos() , m_Path[0]) < m_UnitSpace * m_UnitSpace)
		{
			//If we reached the next point of the path. Remove it 
			m_Path.erase(std::remove(m_Path.begin(), m_Path.end(), m_Path[0]));
		}
	}

	CalculateFormationMovement(deltaT, targetPos);
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
			pAgent->SetGroup(this);
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
		pAgent->SetLinearVelocity(Elite::Vector2{ 0.f,0.f });
		pAgent->SetAngularVelocity(0.f);
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

std::vector<Elite::Vector2> Group::GetPath() const
{
	return m_Path;
}

void Group::CalculatePath(const Vector2& destiation,NavGraph* pNavGraph,std::vector<Vector2>& debugNodePositions,std::vector<Portal>& debugPortals,std::vector<Vector2>& visitedNodePositions)
{
	const float formationOffset{ m_FormationRightVector.Magnitude() + 0.5f};
	Elite::Vector2 currentCenter{ GetCenterPos() };

	//Not working 100% correctly -> crop the portals
	//m_Path = NavMeshPathfinding::FindPath(currentCenter, destiation, pNavGraph, debugNodePositions, debugPortals, visitedNodePositions, formationOffset + m_UnitSpace);

	m_Path = NavMeshPathfinding::FindPath(currentCenter, destiation, pNavGraph, debugNodePositions, debugPortals, visitedNodePositions);

	if (m_Path.size() > 0)
	{
		Elite::Vector2 direction{};
		Elite::Vector2 rotatedDirection{};
		Elite::Vector2 previousDirection{ m_Path[0] - currentCenter };
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

void Group::CalculateFormationMovement(float deltaT, const Elite::Vector2& targetPos)
{
	if (m_IsGroupBased)
	{
		CalculateGroupBasedMovement(deltaT, targetPos);
	}
	else
	{
		CalculateSubGroupBasedMovement(deltaT, targetPos);
	}
}

void Group::CalculateSubGroupBasedMovement(float deltaT, const Elite::Vector2& targetPos)
{
	Elite::Vector2 averageGroupPos{ GetCenterPos() };
	Elite::Vector2 right{ m_FormationRightVector.GetNormalized() };
	Elite::Vector2 forward{ -right.y,right.x };
	
	RegisterOffsetFromCenter(averageGroupPos, forward);
	SortAgentsBasedOnForwardness();

	int allowedNrAgentsInFormation{ static_cast<int>(ceil(static_cast<float>(m_pAgents.size()) / m_NrLines)) };
	int nrAgentsInFormation{};

	//Place in sub formations
	for (SubGroup* pSubGroup : m_pSubGroups)
	{
		for (int index{}; index < allowedNrAgentsInFormation; ++index)
		{
			pSubGroup->RemoveUnitFromGroup(m_pAgents[index]);
		}

		for (int index{}; index < allowedNrAgentsInFormation; ++index)
		{
			pSubGroup->AddUnitToGroup(m_pAgents[allowedNrAgentsInFormation + index]);
		}
	}
}


void Group::CalculateGroupBasedMovement(float deltaT, const Elite::Vector2& targetPos)
{
	Elite::Vector2 averageGroupPos{ GetCenterPos() };
	Elite::Vector2 right{ m_FormationRightVector.GetNormalized() };
	Elite::Vector2 forward{ -right.y,right.x };

	RegisterOffsetFromCenter(averageGroupPos, forward);
	SortAgentsBasedOnForwardness();

	switch (m_CurrentFormation)
	{
	case Formation::Line:
	{
		Elite::Vector2 A{ targetPos - m_FormationRightVector };
		Elite::Vector2 B{ targetPos + m_FormationRightVector };

		DEBUGRENDERER2D->DrawSegment(averageGroupPos, averageGroupPos + forward * 2.f, Color{ 1.f,0.f,0.f }, -1);
		DEBUGRENDERER2D->DrawPoint(A, 5.f, Color{ 1.f,0.f,0.f }, -1);
		DEBUGRENDERER2D->DrawPoint(B, 5.f, Color{ 1.f,0.f,0.f }, -1);

		if (m_FormAfterArrival)
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
			int allowedNrAgentsInFormation{ static_cast<int>(ceil(static_cast<float>(m_pAgents.size()) / m_NrLines)) };
			int adderPerLine{ allowedNrAgentsInFormation };
			int nrAgentsInFormation{};

			//Support for: max 2 lines + equal number of agents in each line
			int currentNrLines{ 1 };
			float lineOffset{ 2.f * m_UnitSpace };

			Elite::Vector2 startOffset{ static_cast<float>(m_NrLines / 2) * forward * lineOffset / 2.f };
			A += startOffset;
			Elite::Vector2 currentA{ GetCenterPos() - m_FormationRightVector + startOffset };
			Elite::Vector2 currentB{ GetCenterPos() + m_FormationRightVector + startOffset};
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
	case Formation::Circle:
	{
		const float radius{ m_FormationRightVector.Magnitude() };
		const float averageDistanceSquared{ (targetPos - averageGroupPos).MagnitudeSquared() };

		DEBUGRENDERER2D->DrawPoint(targetPos, 5.f, Color{ 1.f,1.f,0.f }, -0.9f);
		DEBUGRENDERER2D->DrawCircle(targetPos, radius, Color{ 1.f,1.f,0.f }, -0.9f);

		DEBUGRENDERER2D->DrawPoint(m_DesiredFormationCenter, 5.f, Color{ 1.f,0.f,0.f }, -1.f);
		DEBUGRENDERER2D->DrawCircle(m_DesiredFormationCenter, radius, Color{ 1.f,0.f,0.f }, -1.f);

		for (UnitAgent* pAgent : m_pAgents)
		{
			RegisterNeighbors(pAgent);

			Elite::Vector2 P{ pAgent->GetPosition() };
			Elite::Vector2 currentDirection{ P - averageGroupPos };

			currentDirection.Normalize();

			if (m_FormAfterArrival)
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

void Group::RegisterOffsetFromCenter(const Elite::Vector2& centerPos, const Elite::Vector2& forward)
{
	for (UnitAgent* pAgent : m_pAgents)
	{
		pAgent->SetOffset(pAgent->GetPosition() - centerPos);
		pAgent->CalculateForwardness(forward);
	}
}

void Group::SortAgentsBasedOnForwardness()
{
	auto isMoreForward = [&](UnitAgent* first, UnitAgent* second)->float
	{
		return first->GetForwardness() > second->GetForwardness();
	};

	std::sort(m_pAgents.begin(), m_pAgents.end(), isMoreForward);
}

void Group::SetFormation(const Elite::Vector2& center, const Elite::Vector2& difference, Formation formation, bool formAfterArrival, int nrLines)
{
	m_DesiredFormationCenter = center;
	m_FormationRightVector = difference;
	m_CurrentFormation = formation;
	m_FormAfterArrival = formAfterArrival;

	//Clamp between 1 and 6 lines
	nrLines = (std::min)(6, nrLines);
	m_NrLines = std::max(1, nrLines);

	for (SubGroup* pSubGroup : m_pSubGroups)
	{
		SAFE_DELETE(pSubGroup);
		pSubGroup = nullptr;
	}

	m_pSubGroups.resize(m_NrLines);
	
	for(SubGroup* pSubGroup : m_pSubGroups)
	{
		pSubGroup = new SubGroup()
	}
}


