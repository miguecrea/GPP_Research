#pragma once
#include "../SteeringBehaviors/SteeringHelpers.h"
#include "framework\EliteAI\EliteNavigation\Algorithms\ENavGraphPathfinding.h"

class Group;
class UnitAgent;

class Formation final
{
public:
	Formation(int maxNrUnits = 30);
	~Formation();

	void Update(float deltaT);
	void Render(float deltaT);

	// --Pathfinder--
	void CalculatePath(const Elite::Vector2& destiation, Elite::NavGraph* pNavGraph, std::vector<Elite::Vector2>& debugNodePositions, std::vector<Elite::Portal>& portals, std::vector<Elite::Vector2>& visitedNodePositions);

	//Formation
	void AddUnitToFormation(UnitAgent* pAgent);
	void RemoveUnitFromFormation(UnitAgent* pAgent);

	void SetFormation(const Elite::Vector2& desiredCenter, const Elite::Vector2& desiredRightVector, FormationType formation, bool isLooseMovement, int nrGroups);
	std::vector<Elite::Vector2> GetPath() const;
private:
	void UpdateGroupMovements(float deltaT, const Elite::Vector2& targetPos);
	void RegisterOffsetFromCenter(const Elite::Vector2& centerPos, const Elite::Vector2& forward);
	void SortAgentsBasedOnForwardness();
	void SortAgentsBasedOnDistance();

	Elite::Vector2 CalculateCenterPos() const;

	// --Pathfinder--
	std::vector<Elite::Vector2> m_Path;

	// --Units--
	int m_MaxGroupSize = 0;
	std::vector<UnitAgent*> m_pAgents;
	const float m_UnitSpace{ 3.f };
	const Elite::Color m_LightBlue{ 0.5f,0.5f,1.f };

	// --Groups--
	std::vector<Group*> m_pGroups;
	int m_NrGroups{ 1 };
	
	// --Formation--
	Elite::Vector2 m_CurrentCenter{};
	Elite::Vector2 m_DesiredFormationCenter{};
	Elite::Vector2 m_DesiredFormationRightVector{};

	bool m_IsLooseMovement{ false };
	FormationType m_CurrentFormation{ FormationType::Circle };
};

