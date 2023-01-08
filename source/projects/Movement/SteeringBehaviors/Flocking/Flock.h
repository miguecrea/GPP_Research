#pragma once
#include "../SteeringHelpers.h"
#include "FlockingSteeringBehaviors.h"
#include "../SpacePartitioning/SpacePartitioning.h"

class ISteeringBehavior;
class SteeringAgent;
class BlendedSteering;
class PrioritySteering;

class Flock final
{
public:
	Flock( int flockSize = 50, float worldSize = 100.f);

	~Flock();

	void Update(float deltaT);
	void UpdateAndRenderUI() ;
	void Render(float deltaT);

	void RegisterNeighbors(SteeringAgent* pAgent);
	int GetNrOfNeighbors() const { return m_NrOfNeighbors; }
	const std::vector<SteeringAgent*>& GetNeighbors() const { return m_Neighbors; }

	Elite::Vector2 GetAverageNeighborPos() const;
	Elite::Vector2 GetAverageNeighborVelocity() const;

	void SetTarget_Seek(TargetData target);
	void SetWorldTrimSize(float size) { m_WorldSize = size; }

private:
	//Datamembers
	int m_FlockSize = 0;
	std::vector<SteeringAgent*> m_pAgents;
	std::vector<SteeringAgent*> m_Neighbors;

	bool m_TrimWorld = false;
	float m_WorldSize = 0.f;

	float m_NeighborhoodRadius = 10.f;
	int m_NrOfNeighbors = 0;

	std::vector<Elite::Vector2> m_DebugVertices;

	SteeringAgent* m_pAgentToEvade = nullptr;
	
	//Steering Behaviors
	Seek* m_pSeekBehavior = nullptr;
	Wander* m_pWanderBehavior = nullptr;
	Evade* m_pEvadeBehavior = nullptr;

	BlendedSteering* m_pBlendedSteering = nullptr;
	PrioritySteering* m_pPrioritySteering = nullptr;

	float* GetWeight(ISteeringBehavior* pBehaviour);

	bool m_CanDebugRender{ true };

	float m_EvadeRadius{ 10.f };

	const Elite::Color m_Red{ 1.f,0.f,0.f };
	const Elite::Color m_LightBlue{ 0.f,0.f,1.f };
	const Elite::Color m_DebugGreen{ 0.f,1.f,0.f,0.5f };

	//Space Partitioning
	CellSpace* m_pCellSpace = nullptr;

	bool m_EnableSpacePartitioning{ true };

	Flock(const Flock& other);
	Flock& operator=(const Flock& other);

	void FlockDebugRendering(SteeringAgent* pAgent);
};