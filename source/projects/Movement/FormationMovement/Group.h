#pragma once
#include "../SteeringBehaviors/SteeringHelpers.h"
#include "framework\EliteAI\EliteNavigation\Algorithms\ENavGraphPathfinding.h"

class Seek;
class Arrive;
class Evade;
class Face;

class Separation;
class Cohesion;
class VelocityMatch;

class ISteeringBehavior;
class UnitAgent;
class BlendedSteering;
class PrioritySteering;

class Group
{
public:
	Group(std::vector<UnitAgent*>* pOtherAgents,int maxGroupSize = 15);

	virtual ~Group();

	void Update(float deltaT, const Elite::Vector2& targetPos);
	void Render(float deltaT);

	//Group
	void AddUnitToGroup(UnitAgent* pAgent);
	void RemoveUnitFromGroup(UnitAgent* pAgent);

	int GetNrOfAgents() const { return m_pAgents.size(); }
	const std::vector<UnitAgent*>& GetAgents() const { return m_pAgents; }

	Elite::Vector2 GetCenterPos() const;
	Elite::Vector2 GetAverageVelocity() const;

	//Neighbors
	int GetNrOfNeighbors() const { return m_NrOfNeighbors; }
	const std::vector<UnitAgent*>& GetNeighbors() const { return m_pNeighbors; }
	Elite::Vector2 GetAverageNeighborPos() const;
	Elite::Vector2 GetAverageNeighborVelocity() const;

	//Steering
	void SetFormation(const Elite::Vector2& desiredCenter, const Elite::Vector2& desiredRightVector, FormationType formation, bool isLooseMovement);
protected:
	void RegisterNeighbors(UnitAgent* pAgent);
	void CalculateFormationMovement(float deltaT, const Elite::Vector2& targetPos);

	//Datamembers
	// --Group--
	int m_MaxGroupSize = 0;
	std::vector<UnitAgent*> m_pAgents;
	std::vector<UnitAgent*>* m_pOtherAgents;

	//Steering Behaviors
	std::vector<UnitAgent*> m_pNeighbors;
	int m_NrOfNeighbors = 0;

	Seek* m_pSeekBehavior = nullptr;
	Seek* m_pSeekABehavior = nullptr;
	Seek* m_pSeekBBehavior = nullptr;
	Seek* m_pSeekOtherSideBehavior = nullptr;
	Seek* m_pSeekDesiredLocationBehavior = nullptr;
	Separation* m_pSeparationBehavior = nullptr;
	Cohesion* m_pCohesionBehavior = nullptr;
	VelocityMatch* m_pVelMatchBehavior = nullptr;
	Arrive* m_pArriveBehavior = nullptr;
	Evade* m_pEvadeBehavior = nullptr;
	Face* m_pFaceBehavior = nullptr;

	BlendedSteering* m_pBlendedLineSteering = nullptr;
	BlendedSteering* m_pBlendedLineFormAtLocationSteering = nullptr;
	BlendedSteering* m_pBlendedCircleSteering = nullptr;
	BlendedSteering* m_pBlendedCircleFormAtLocationSteering = nullptr;
	PrioritySteering* m_pPrioritySteering = nullptr;

	bool m_FormAfterArrival{ false };
	bool m_IsLooseMovement{ false };
	bool m_CanDebugRender{ true };

	float m_EvadeRadius{ 10.f };

	const float m_UnitSpace{ 3.f };
	const Elite::Color m_Red{ 1.f,0.f,0.f };
	const Elite::Color m_DebugGreen{ 0.f,1.f,0.f,0.5f };

	//Formations
	FormationType m_CurrentFormation{ FormationType::Line };
	Elite::Vector2 m_DesiredCenter{};
	Elite::Vector2 m_DesiredRightVector{};

	Group(const Group& other);
	Group& operator=(const Group& other);
};