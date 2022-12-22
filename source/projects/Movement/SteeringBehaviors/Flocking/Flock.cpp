#include "stdafx.h"
#include "Flock.h"

#include "../SteeringAgent.h"
#include "../Steering/SteeringBehaviors.h"
#include "../CombinedSteering/CombinedSteeringBehaviors.h"

using namespace Elite;

//Constructor & Destructor
Flock::Flock(
	int flockSize /*= 50*/, 
	float worldSize /*= 100.f*/)
	: m_WorldSize{ worldSize }
	, m_FlockSize{ flockSize }
	, m_TrimWorld { true }
	, m_NeighborhoodRadius{ 15 }
	, m_NrOfNeighbors{0}
{
	m_pAgents.resize(m_FlockSize);
	m_Neighbors.resize(m_FlockSize);

	m_pSeekBehavior = new Seek();
	m_pWanderBehavior = new Wander();
	m_pEvadeBehavior = new Evade();

	m_pCohesionBehavior = new Cohesion(this);
	m_pSeparationBehavior = new Separation(this);
	m_pVelMatchBehavior = new VelocityMatch(this);

	m_pBlendedSteering = new BlendedSteering({ { m_pCohesionBehavior,0.f }, { m_pSeparationBehavior,1.f }, { m_pVelMatchBehavior,0.f },{m_pWanderBehavior,0.0f },{m_pSeekBehavior,0.0f} });
	m_pPrioritySteering = new PrioritySteering({ m_pEvadeBehavior,m_pBlendedSteering });

	m_pCellSpace = new CellSpace(m_WorldSize, m_WorldSize, 19, 19);

	Elite::Vector2 randomPosition{};

	for (int index{}; index < m_FlockSize; ++index)
	{
		m_pAgents[index] = new SteeringAgent();
		m_pAgents[index]->SetSteeringBehavior(m_pPrioritySteering);
		m_pAgents[index]->SetMaxLinearSpeed(15.f);
		m_pAgents[index]->SetAutoOrient(true);

		randomPosition.x = static_cast<float>(rand() % static_cast<int>(m_WorldSize));
		randomPosition.y = static_cast<float>(rand() % static_cast<int>(m_WorldSize));

		m_pAgents[index]->SetPosition(randomPosition);

		m_pCellSpace->AddAgent(m_pAgents[index]);
	}

	m_pAgentToEvade = new SteeringAgent();
	m_pAgentToEvade->SetSteeringBehavior(m_pSeekBehavior);
	m_pAgentToEvade->SetMaxLinearSpeed(60.0f);
	m_pAgentToEvade->SetAutoOrient(true);
	m_pAgentToEvade->SetPosition(randomVector2(m_WorldSize));
	m_pAgentToEvade->SetLinearVelocity(randomVector2(m_pAgentToEvade->GetMaxLinearSpeed()));
	m_pAgentToEvade->SetBodyColor({ 1, 0, 0 });

	Elite::Vector2 firstAgentPosition{ m_pAgents[0]->GetPosition() };
	for (int index{}; index < 4; ++index)
	{
		m_DebugVertices.push_back(firstAgentPosition);
	}
}

Flock::~Flock()
{
	SAFE_DELETE(m_pEvadeBehavior);
	SAFE_DELETE(m_pSeekBehavior);
	SAFE_DELETE(m_pWanderBehavior);
	SAFE_DELETE(m_pCohesionBehavior);
	SAFE_DELETE(m_pSeparationBehavior);
	SAFE_DELETE(m_pBlendedSteering);
	SAFE_DELETE(m_pPrioritySteering);
	SAFE_DELETE(m_pVelMatchBehavior);
	SAFE_DELETE(m_pAgentToEvade);

	SAFE_DELETE(m_pCellSpace);

	for(auto pAgent: m_pAgents)
	{
		SAFE_DELETE(pAgent);
	}
	m_pAgents.clear();
}

void Flock::Update(float deltaT)
{
	m_pEvadeBehavior->SetEvadeRadius(m_EvadeRadius);
	m_pEvadeBehavior->SetTarget(m_pAgentToEvade->GetPosition());

	for (SteeringAgent* pAgent : m_pAgents)
	{
		if (m_EnableSpacePartitioning)
		{
			m_pCellSpace->UpdateAgentCell(pAgent);
			m_pCellSpace->RegisterNeighbors(pAgent, m_NeighborhoodRadius, m_Neighbors, m_NrOfNeighbors);
		}
		else
		{
			RegisterNeighbors(pAgent);
		}

		pAgent->Update(deltaT);

		if (m_TrimWorld)
		{
			pAgent->TrimToWorld(m_WorldSize);
		}

		if (pAgent == m_pAgents[0])
		{
			FlockDebugRendering(pAgent);
		}
	}

	if (m_pAgentToEvade)
	{
		m_pAgentToEvade->Update(deltaT);
		m_pAgentToEvade->TrimToWorld(m_WorldSize);
	}

	if (m_EnableSpacePartitioning && m_CanDebugRender)
	{
		m_pCellSpace->RenderCells();
	}
}

void Flock::Render(float deltaT)
{
	//for (SteeringAgent* pAgent : m_pAgents)
	//{
	//	if (pAgent != nullptr)
	//	{
	//		pAgent->Render(deltaT);
	//	}
	//}

	if (m_pAgentToEvade)
	{
		m_pAgentToEvade->Render(deltaT);
	}
}

void Flock::UpdateAndRenderUI()
{
	//Setup
	int menuWidth = 235;
	int const width = DEBUGRENDERER2D->GetActiveCamera()->GetWidth();
	int const height = DEBUGRENDERER2D->GetActiveCamera()->GetHeight();
	bool windowActive = true;
	ImGui::SetNextWindowPos(ImVec2((float)width - menuWidth - 10, 10));
	ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)height - 20));
	ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	ImGui::PushAllowKeyboardFocus(false);

	//Elements
	ImGui::Text("CONTROLS");
	ImGui::Indent();
	ImGui::Text("LMB: place target");
	ImGui::Text("RMB: move cam.");
	ImGui::Text("Scrollwheel: zoom cam.");
	ImGui::Unindent();

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();
	ImGui::Spacing();

	ImGui::Text("STATS");
	ImGui::Indent();
	ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
	ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
	ImGui::Unindent();

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();

	ImGui::Text("Flocking");
	ImGui::Spacing();

	// TODO: Implement checkboxes for debug rendering and weight sliders here
	ImGui::SliderFloat("Evade Radius", &m_EvadeRadius, 0.f, 100.f, "%.2");

	ImGui::Checkbox("Debug Rendering", &m_CanDebugRender);
	ImGui::Checkbox("Space Partitioning", &m_EnableSpacePartitioning);

	ImGui::SliderFloat("Cohesion", &m_pBlendedSteering->GetWeightedBehaviorsRef()[0].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Separation", &m_pBlendedSteering->GetWeightedBehaviorsRef()[1].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("VelocityMatch", &m_pBlendedSteering->GetWeightedBehaviorsRef()[2].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Wander", &m_pBlendedSteering->GetWeightedBehaviorsRef()[3].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Seek", &m_pBlendedSteering->GetWeightedBehaviorsRef()[4].weight, 0.f, 1.f, "%.2");

	//End
	ImGui::PopAllowKeyboardFocus();
	ImGui::End();
	
}

void Flock::RegisterNeighbors(SteeringAgent* pAgent)
{
	m_NrOfNeighbors = 0;

	for (SteeringAgent* pOtherAgent : m_pAgents)
	{
		if ((pAgent != pOtherAgent) && (pOtherAgent != nullptr) && (Distance(pAgent->GetPosition(), pOtherAgent->GetPosition()) < m_NeighborhoodRadius))
		{
			m_Neighbors[m_NrOfNeighbors] = pOtherAgent;
			++m_NrOfNeighbors;
		}
	}
}

Elite::Vector2 Flock::GetAverageNeighborPos() const
{
	Vector2 averagePosition{};

	for (int index{}; index<m_NrOfNeighbors; ++index)
	{
		averagePosition += m_Neighbors[index]->GetPosition();
	}

	return averagePosition /= static_cast<float>(m_NrOfNeighbors);	
}

Elite::Vector2 Flock::GetAverageNeighborVelocity() const
{
	Vector2 averageVelocity{};

	for (int index{}; index < m_NrOfNeighbors; ++index)
	{
		averageVelocity += m_Neighbors[index]->GetLinearVelocity();
	}

	return averageVelocity / static_cast<float>(m_NrOfNeighbors);
}

void Flock::SetTarget_Seek(TargetData target)
{
	m_pSeekBehavior->SetTarget(target);
}


float* Flock::GetWeight(ISteeringBehavior* pBehavior) 
{
	if (m_pBlendedSteering)
	{
		auto& weightedBehaviors = m_pBlendedSteering->GetWeightedBehaviorsRef();
		auto it = find_if(weightedBehaviors.begin(),
			weightedBehaviors.end(),
			[pBehavior](BlendedSteering::WeightedBehavior el)
			{
				return el.pBehavior == pBehavior;
			}
		);

		if(it!= weightedBehaviors.end())
			return &it->weight;
	}

	return nullptr;
}

void Flock::FlockDebugRendering(SteeringAgent* pAgent)
{
	pAgent->SetRenderBehavior(m_CanDebugRender);

	if (m_CanDebugRender)
	{
		const Elite::Vector2 agentPosition{ pAgent->GetPosition() };

		DEBUGRENDERER2D->DrawCircle(agentPosition, m_NeighborhoodRadius, m_Red, -1);

		if (m_EnableSpacePartitioning)
		{
			m_DebugVertices[0] = { agentPosition.x + m_NeighborhoodRadius,agentPosition.y + m_NeighborhoodRadius };
			m_DebugVertices[1] = { agentPosition.x + m_NeighborhoodRadius,agentPosition.y - m_NeighborhoodRadius };
			m_DebugVertices[2] = { agentPosition.x - m_NeighborhoodRadius,agentPosition.y - m_NeighborhoodRadius };
			m_DebugVertices[3] = { agentPosition.x - m_NeighborhoodRadius,agentPosition.y + m_NeighborhoodRadius };

			DEBUGRENDERER2D->DrawPolygon(&Elite::Polygon{ m_DebugVertices }, m_Blue, 1);
		}

		for (int index{}; index < m_NrOfNeighbors; ++index)
		{
			DEBUGRENDERER2D->DrawSolidCircle(m_Neighbors[index]->GetPosition(), m_Neighbors[index]->GetRadius() * 1.5f, { 0,0 }, m_DebugGreen, -1);
		}
	}
}
