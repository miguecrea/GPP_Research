#include "stdafx.h"
#include "SpacePartitioning.h"
#include "projects\Movement\SteeringBehaviors\SteeringAgent.h"

// --- Cell ---
// ------------
Cell::Cell(float left, float bottom, float width, float height)
{
	boundingBox.bottomLeft = { left, bottom };
	boundingBox.width = width;
	boundingBox.height = height;
}

std::vector<Elite::Vector2> Cell::GetRectPoints() const
{
	auto left = boundingBox.bottomLeft.x;
	auto bottom = boundingBox.bottomLeft.y;
	auto width = boundingBox.width;
	auto height = boundingBox.height;

	std::vector<Elite::Vector2> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(float width, float height, int rows, int cols)
	: m_SpaceWidth(width)
	, m_SpaceHeight(height)
	, m_NrOfRows(rows)
	, m_NrOfCols(cols)
{
	m_CellWidth = m_SpaceWidth / m_NrOfCols;
	m_CellHeight = m_SpaceHeight / m_NrOfRows;

	for (int indexY{}; indexY < m_NrOfRows; ++indexY)
	{
		for (int indexX{}; indexX < m_NrOfCols; ++indexX)
		{
			m_Cells.push_back(Cell{ indexX * m_CellWidth,indexY * m_CellHeight,m_CellWidth,m_CellHeight });
		}
	}
}

void CellSpace::AddAgent(SteeringAgent* agent)
{
	agent->SetOldPosition(agent->GetPosition());
	m_Cells[PositionToIndex(agent->GetPosition())].agents.push_back(agent);
}

void CellSpace::UpdateAgentCell(SteeringAgent* agent)
{
	int oldIndex{ PositionToIndex(agent->GetOldPosition()) };
	int newIndex{ PositionToIndex(agent->GetPosition()) };

	if (newIndex != oldIndex)
	{
		m_Cells[newIndex].agents.push_back(agent);
		m_Cells[oldIndex].agents.remove(agent);
	}

	agent->SetOldPosition(agent->GetPosition());
}

void CellSpace::RegisterNeighbors(SteeringAgent* agent, float queryRadius, std::vector<SteeringAgent*>& neighbors, int& nrOfNeighbors)
{
	nrOfNeighbors = 0;

	Elite::Vector2 agentPos{ agent->GetPosition() };

	int bottomLeftIndex{ PositionToIndex({agentPos.x - queryRadius,agentPos.y - queryRadius}) };
	int topRightIndex{ PositionToIndex({agentPos.x + queryRadius,agentPos.y + queryRadius}) };

	int bottomLeftX{ bottomLeftIndex % m_NrOfCols };
	int bottomLeftY{ bottomLeftIndex / m_NrOfCols };

	int topRightX{ topRightIndex % m_NrOfCols };
	int topRightY{ topRightIndex / m_NrOfCols };

	for (int indexY{ bottomLeftY }; indexY <= topRightY; ++indexY)
	{
		for (int indexX{ bottomLeftX }; indexX <= topRightX; ++indexX)
		{
			if (agent->CanRenderBehavior())
			{
				DEBUGRENDERER2D->DrawPolygon(&Elite::Polygon{ m_Cells[indexX + (indexY * m_NrOfCols)].GetRectPoints() }, Elite::Color{ 0.5f,0.f,0.f }, 0);
			}

			for (SteeringAgent* pOtherAgent : m_Cells[indexX + (indexY * m_NrOfCols)].agents)
			{
				if ((agent != pOtherAgent) && (Distance(agentPos, pOtherAgent->GetPosition()) < queryRadius))
				{
					neighbors[nrOfNeighbors] = pOtherAgent;
					++nrOfNeighbors;
				}
			}
		}
	}

}

void CellSpace::EmptyCells()
{
	for (Cell& c : m_Cells)
		c.agents.clear();
}

void CellSpace::RenderCells() const
{
	for (const Cell& cell : m_Cells)
	{
		DEBUGRENDERER2D->DrawPolygon(&Elite::Polygon{ cell.GetRectPoints() }, Elite::Color{ 0.5f,0.5f,0.f }, 0.5f);

		const std::string nrAgentsInCell{ std::to_string(cell.agents.size()) };

		Elite::Vector2 stringPosition{ cell.boundingBox.bottomLeft };
		stringPosition.y += m_CellHeight; //offset

		DEBUGRENDERER2D->DrawString(stringPosition, nrAgentsInCell.c_str());
	}
}

int CellSpace::PositionToIndex(const Elite::Vector2 pos) const
{
	const int xIndex{ Elite::Clamp(static_cast<int>(pos.x / m_CellWidth), 0, m_NrOfCols-1) };
	const int yIndex{ Elite::Clamp(static_cast<int>(pos.y / m_CellHeight), 0, m_NrOfRows-1) };

	return  xIndex + (yIndex * m_NrOfCols);
}