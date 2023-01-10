/*=============================================================================*/
// SteeringAgent.h: basic unit in a formation
/*=============================================================================*/
#ifndef UNIT_AGENT_H
#define UNIT_AGENT_H

//-----------------------------------------------------------------
// Includes & Forward Declarations
//-----------------------------------------------------------------
#include "../SteeringBehaviors/SteeringAgent.h"

class Group;
class ISteeringBehavior;

class UnitAgent final : public SteeringAgent
{
public:
	//--- Constructor & Destructor ---
	UnitAgent() = default;
	UnitAgent(float radius) : SteeringAgent(radius) {};
	virtual ~UnitAgent() = default;

	//--- Unit Functions ---
	void SetNeighborhoodRadius(float radius) { m_NeighborhoodRadius = radius; };
	float GetNeighborhoodRadius() const  { return m_NeighborhoodRadius; };
	void SetGroup(Group* pGroup);
	Group* GetGroup() const { return m_pGroup; };
protected:
	//--- Datamembers ---
	float m_NeighborhoodRadius{ 10.f };
	Group* m_pGroup;
};
#endif