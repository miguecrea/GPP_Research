#pragma once
#include "../SteeringBehaviors/SteeringHelpers.h"
#include "framework\EliteAI\EliteNavigation\Algorithms\ENavGraphPathfinding.h"

class Seek;
class Arrive;
class Evade;

class ISteeringBehavior;
class UnitAgent;
class BlendedSteering;
class PrioritySteering;


class Group final
{
public:
	Group(int maxGroupSize = 15);

	~Group();

	void Update(float deltaT);
	void Render(float deltaT);

	void AddUnitToGroup(UnitAgent* pAgent);
	void RemoveUnitFromGroup(UnitAgent* pAgent);

	int GetNrOfAgents() const { return m_pAgents.size(); }
	const std::vector<UnitAgent*>& GetAgents() const { return m_pAgents; }

	Elite::Vector2 GetCenterPos() const;
	Elite::Vector2 GetAverageVelocity() const;

	std::vector<Elite::Vector2> GetPath() const;

	void SetTarget_Seek(TargetData target);

	void CalculatePath(const Elite::Vector2& destiation, Elite::NavGraph * pNavGraph, std::vector<Elite::Vector2>& debugNodePositions, std::vector<Elite::Portal>& portals, std::vector<Elite::Vector2>& visitedNodePositions);
private:
	//Datamembers
	// --Group--
	int m_MaxGroupSize = 0;
	std::vector<UnitAgent*> m_pAgents;
	
	// --Pathfinder--
	std::vector<Elite::Vector2> m_vPath;
	
	//Steering Behaviors
	TargetData m_Target = {};

	Seek* m_pSeekBehavior = nullptr;
	//Separation* m_pSeparationBehavior = nullptr;
	//Cohesion* m_pCohesionBehavior = nullptr;
	//VelocityMatch* m_pVelMatchBehavior = nullptr;
	Arrive* m_pArriveBehavior = nullptr;
	Evade* m_pEvadeBehavior = nullptr;

	BlendedSteering* m_pBlendedSteering = nullptr;
	PrioritySteering* m_pPrioritySteering = nullptr;

	bool m_CanDebugRender{ true };

	float m_EvadeRadius{ 10.f };

	const float m_UnitSpace{ 5.f };

	const Elite::Color m_Red{ 1.f,0.f,0.f };
	const Elite::Color m_Blue{ 0.f,0.f,1.f };
	const Elite::Color m_DebugGreen{ 0.f,1.f,0.f,0.5f };

	Group(const Group& other);
	Group& operator=(const Group& other);
};