/*=============================================================================*/
// SteeringAgent.h: basic unit in a formation
/*=============================================================================*/
#ifndef UNIT_AGENT_H
#define UNIT_AGENT_H

//-----------------------------------------------------------------
// Includes & Forward Declarations
//-----------------------------------------------------------------
#include "../SteeringBehaviors/SteeringAgent.h"

class ISteeringBehavior;

class UnitAgent final : public SteeringAgent
{
public:
	//--- Constructor & Destructor ---
	UnitAgent() = default;
	UnitAgent(float radius) : SteeringAgent(radius) {};
	virtual ~UnitAgent() = default;

	//--- Unit Functions ---
	void SetFormationOffset(Elite::Vector2& offset);

protected:
	//--- Datamembers ---
	Elite::Vector2 m_FormationOffset{};
};
#endif