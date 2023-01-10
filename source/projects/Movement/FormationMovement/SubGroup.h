#pragma once
#include "Group.h"
class SubGroup final: public Group
{
public:
	SubGroup(int maxGroupSize = 15);

	virtual ~SubGroup() = default;
private:
	void CalculateGroupBasedMovement(float deltaT, const Elite::Vector2& targetPos) override;
	SubGroup(const SubGroup& other) = delete;
	SubGroup& operator=(const SubGroup& other) = delete;
};

