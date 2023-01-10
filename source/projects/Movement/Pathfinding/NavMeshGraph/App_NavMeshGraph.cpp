//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"
using namespace Elite;

//Includes
#include "App_NavMeshGraph.h"
#include "projects/Shared/NavigationColliderElement.h"

#include "projects/Movement/FormationMovement/UnitAgent.h"
#include "projects/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "projects/Movement/SteeringBehaviors/Flocking/Flock.h"
#include "projects/Movement/SteeringBehaviors/Flocking/FlockingSteeringBehaviors.h"

#include "framework\EliteAI\EliteNavigation\Algorithms\ENavGraphPathfinding.h"
#include "projects/Movement/FormationMovement/Group.h"

//Statics
bool App_NavMeshGraph::sShowPolygon = true;
bool App_NavMeshGraph::sFormAfterArrival = false;
bool App_NavMeshGraph::sIsLine = true;
bool App_NavMeshGraph::sIsCircle = false;
bool App_NavMeshGraph::sRecalculateFormation = false;
Formation App_NavMeshGraph::sCurrentFormation = Formation::Line;

//Destructor
App_NavMeshGraph::~App_NavMeshGraph()
{
	for (auto pNC : m_vNavigationColliders)
		SAFE_DELETE(pNC);
	m_vNavigationColliders.clear();

	SAFE_DELETE(m_pNavGraph);

	for (UnitAgent* pAgent : m_pAgents)
	{
		SAFE_DELETE(pAgent);
	}

	SAFE_DELETE(m_pGroup);
}

//Functions
void App_NavMeshGraph::Start()
{
	//Initialization of your application. 
	//----------- CAMERA ------------
	DEBUGRENDERER2D->GetActiveCamera()->SetZoom(36.782f);
	DEBUGRENDERER2D->GetActiveCamera()->SetCenter(Elite::Vector2(12.9361f, 0.2661f));

	//----------- WORLD ------------
	m_vNavigationColliders.push_back(new NavigationColliderElement(Elite::Vector2(125.f, 120.f), 45.0f, 7.0f));
	m_vNavigationColliders.push_back(new NavigationColliderElement(Elite::Vector2(-135.f, 17.f), 14.0f, 10.0f));
	m_vNavigationColliders.push_back(new NavigationColliderElement(Elite::Vector2(-113.f, -81.f), 30.0f, 2.0f));
	m_vNavigationColliders.push_back(new NavigationColliderElement(Elite::Vector2(15.f, -21.f), 50.0f, 3.0f));

	//----------- NAVMESH  ------------
	std::list<Elite::Vector2> baseBox
	{ { -200, 200 },{ -200, -200 },{ 200, -200 },{ 200, 200 } };

	m_pNavGraph = new Elite::NavGraph(Elite::Polygon(baseBox), m_AgentRadius);

	//----------- GROUP ------------
	m_pGroup = new Group(30);

	//----------- AGENT ------------
	m_pAgents.reserve(m_NrAgents);

	for (int index{}; index < m_NrAgents; ++index)
	{
		m_pAgents.emplace_back(new UnitAgent());
		m_pAgents[index]->SetMaxLinearSpeed(m_AgentSpeed);
		m_pAgents[index]->SetAutoOrient(false);
		m_pAgents[index]->SetMass(1.f);
		m_pAgents[index]->SetPosition(Vector2{ 0.f,0.f + index * 2.f });
	}

	//-------- SELECTION -------
	for (int index{}; index < 4; ++index)
	{
		m_LeftSelectionPoints.push_back(Vector2{});
	}
}

void App_NavMeshGraph::Update(float deltaTime)
{
	//////////////////////////////////////////
	//  Middle mouse button
	/////////////////////////////////////////

	if (INPUTMANAGER->IsMouseButtonDown(InputMouseButton::eRight))
	{
		m_IsSelectingRight = true;
	
		auto mouseData = INPUTMANAGER->GetMouseData(Elite::InputType::eMouseButton, Elite::InputMouseButton::eRight);
		m_StartRightSelectionPos = DEBUGRENDERER2D->GetActiveCamera()->ConvertScreenToWorld(Elite::Vector2((float)mouseData.X, (float)mouseData.Y));
	}

	//Update Selection
	if (m_IsSelectingRight)
	{
		//Update boundaries of the selection
		if (INPUTMANAGER->IsMouseMoving())
		{
			auto mouseData = INPUTMANAGER->GetMouseData(Elite::eMouseMotion);
			m_EndRightSelectionPos = DEBUGRENDERER2D->GetActiveCamera()->ConvertScreenToWorld(Elite::Vector2((float)mouseData.X, (float)mouseData.Y));
			
			m_DifferenceRight = m_EndRightSelectionPos - m_StartRightSelectionPos;
		}

		//Draw Selection
		DEBUGRENDERER2D->DrawPoint(m_StartRightSelectionPos, 5.f, m_SelectionColor, -1.f);
		DEBUGRENDERER2D->DrawPoint(m_StartRightSelectionPos - m_DifferenceRight, 5.f, m_SelectionColor, -1.f);
		DEBUGRENDERER2D->DrawPoint(m_StartRightSelectionPos + m_DifferenceRight, 5.f, m_SelectionColor, -1.f);
	}

	//Update target/path based on input
	if (INPUTMANAGER->IsMouseButtonUp(InputMouseButton::eRight) || sRecalculateFormation)
	{
		m_IsSelectingRight = false;
		sRecalculateFormation = false;

		m_pGroup->SetFormation(m_StartRightSelectionPos, m_DifferenceRight, sCurrentFormation, sFormAfterArrival);
		m_pGroup->CalculatePath(m_StartRightSelectionPos, m_pNavGraph, m_DebugNodePositions, m_Portals, m_VisitedNodePositions);
	}			



	//////////////////////////////////////////
	//  Left mouse button
	/////////////////////////////////////////

	//Start selecting
	if (INPUTMANAGER->IsMouseButtonDown(InputMouseButton::eLeft))
	{
		m_IsSelectingLeft = true;

		auto mouseData = INPUTMANAGER->GetMouseData(Elite::InputType::eMouseButton, Elite::InputMouseButton::eLeft);
		m_StartLeftSelectionPos = DEBUGRENDERER2D->GetActiveCamera()->ConvertScreenToWorld(Elite::Vector2((float)mouseData.X, (float)mouseData.Y));

		m_LeftSelectionPoints[0] = m_StartLeftSelectionPos;
		m_LeftSelectionPoints[1] = m_StartLeftSelectionPos;
		m_LeftSelectionPoints[2] = m_StartLeftSelectionPos;
		m_LeftSelectionPoints[3] = m_StartLeftSelectionPos;

		m_EndLeftSelectionPos = m_StartLeftSelectionPos;
	}

	//Update Selection
	if (m_IsSelectingLeft)
	{
		//Update boundaries of the selection
		if (INPUTMANAGER->IsMouseMoving())
		{
			auto mouseData = INPUTMANAGER->GetMouseData(Elite::eMouseMotion);
			m_EndLeftSelectionPos = DEBUGRENDERER2D->GetActiveCamera()->ConvertScreenToWorld(Elite::Vector2((float)mouseData.X, (float)mouseData.Y));

			const Vector2 difference{ m_EndLeftSelectionPos - m_StartLeftSelectionPos };

			m_LeftSelectionPoints[0] = m_StartLeftSelectionPos;
			m_LeftSelectionPoints[1] = m_StartLeftSelectionPos + Vector2{ difference.x,0.f };
			m_LeftSelectionPoints[2] = m_EndLeftSelectionPos;
			m_LeftSelectionPoints[3] = m_StartLeftSelectionPos + Vector2{ 0.f,difference.y };
		}

		//Draw Selection
		Elite::Polygon selectionPolygon{ m_LeftSelectionPoints };
		DEBUGRENDERER2D->DrawPolygon(&selectionPolygon, m_SelectionColor, DEBUGRENDERER2D->NextDepthSlice());
	}

	//Stop selection
	if (INPUTMANAGER->IsMouseButtonUp(InputMouseButton::eLeft))
	{
		m_IsSelectingLeft = false;

		const Vector2 max{ std::max(m_StartLeftSelectionPos.x,m_EndLeftSelectionPos.x),std::max(m_StartLeftSelectionPos.y,m_EndLeftSelectionPos.y) };
		const Vector2 min{ (std::min)(m_StartLeftSelectionPos.x,m_EndLeftSelectionPos.x),(std::min)(m_StartLeftSelectionPos.y,m_EndLeftSelectionPos.y) };

		for (UnitAgent* pAgent : m_pAgents)
		{
			Vector2 position{ pAgent->GetPosition() };

			if (position.x > min.x &&
				position.y > min.y &&
				position.x < max.x &&
				position.y < max.y)
			{
				m_pGroup->AddUnitToGroup(pAgent);
			}
			else
			{
				m_pGroup->RemoveUnitFromGroup(pAgent);
			}
		}
	}
	
	//////////////////////////////////////////
	//  Rest
	/////////////////////////////////////////

	m_pGroup->Update(deltaTime);

	UpdateImGui();

}

void App_NavMeshGraph::Render(float deltaTime) const
{
	for (UnitAgent* pAgent : m_pAgents)
	{
		pAgent->Render(deltaTime);
	}

	if (sShowPolygon)
	{
		DEBUGRENDERER2D->DrawPolygon(m_pNavGraph->GetNavMeshPolygon(),
			Color(0.1f, 0.1f, 0.1f));
		DEBUGRENDERER2D->DrawSolidPolygon(m_pNavGraph->GetNavMeshPolygon(),
			Color(0.0f, 0.5f, 0.1f, 0.05f), 0.4f);
	}
}

void App_NavMeshGraph::UpdateImGui()
{
	//------- UI --------
#ifdef PLATFORM_WINDOWS
#pragma region UI
	{
		//Setup
		int menuWidth = 150;
		int const width = DEBUGRENDERER2D->GetActiveCamera()->GetWidth();
		int const height = DEBUGRENDERER2D->GetActiveCamera()->GetHeight();
		bool windowActive = true;
		ImGui::SetNextWindowPos(ImVec2((float)width - menuWidth - 10, 10));
		ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)height - 90));
		ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
		ImGui::PushAllowKeyboardFocus(false);
		ImGui::SetWindowFocus();
		ImGui::PushItemWidth(70);
		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
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
		ImGui::Spacing();

		ImGui::Checkbox("Show Polygon", &sShowPolygon);

		ImGui::Spacing();
		ImGui::Spacing();

		if (ImGui::SliderFloat("AgentSpeed", &m_AgentSpeed, 0.0f, 22.0f))
		{
			for (UnitAgent* pAgent : m_pAgents)
			{
				pAgent->SetMaxLinearSpeed(m_AgentSpeed);
			}
		}
		
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("FORMATIONS");
		if (ImGui::Checkbox("Line  ",&sIsLine))
		{
			sCurrentFormation = Formation::Line;
			sIsCircle = false;
			sRecalculateFormation = true;
		}
		if (ImGui::Checkbox("Circle", &sIsCircle))
		{
			sCurrentFormation = Formation::Circle;
			sIsLine = false;
			sRecalculateFormation = true;
		}
		ImGui::Spacing();
		if (ImGui::Checkbox("Loose Movement", &sFormAfterArrival))
		{
			sRecalculateFormation = true;
		}
		//End
		ImGui::PopAllowKeyboardFocus();
		ImGui::End();
	}
#pragma endregion
#endif
}