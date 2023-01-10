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
	Elite::Vector2 GetOffset() const { return m_OffsetFromCenter; };
	float GetForwardness() const { return m_Forwardness; };
	void SetGroup(Group* pGroup);
	void SetOffset(const Elite::Vector2 offset) { m_OffsetFromCenter = offset; };
	void CalculateForwardness(const Elite::Vector2 forward) { m_Forwardness = Dot(forward, m_OffsetFromCenter); };
	Group* GetGroup() const { return m_pGroup; };
protected:
	//--- Datamembers ---
	Elite::Vector2 m_OffsetFromCenter{};
	float m_Forwardness{};
	float m_NeighborhoodRadius{ 10.f };
	Group* m_pGroup;
};
#endif