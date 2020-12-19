#include "MyEntityManager.h"
using namespace Simplex;
//  MyEntityManager
Simplex::MyEntityManager* Simplex::MyEntityManager::m_pInstance = nullptr;
void Simplex::MyEntityManager::Init(void)
{
	m_uEntityCount = 0;
	m_mEntityArray = nullptr;
	octree = new OctreeNode(0, vector3(0, 0, 0), vector3(0, 0, 0));
	goalIndex = 0;
	died = false;
	victory = false;
}
void Simplex::MyEntityManager::Release(void)
{
	for (uint uEntity = 0; uEntity < m_uEntityCount; ++uEntity)
	{
		MyEntity* pEntity = m_mEntityArray[uEntity];
		SafeDelete(pEntity);
	}
	m_uEntityCount = 0;
	m_mEntityArray = nullptr;
	SafeDelete(octree);
}
Simplex::MyEntityManager* Simplex::MyEntityManager::GetInstance()
{
	if(m_pInstance == nullptr)
	{
		m_pInstance = new MyEntityManager();
	}
	return m_pInstance;
}
void Simplex::MyEntityManager::ReleaseInstance()
{
	if(m_pInstance != nullptr)
	{
		delete m_pInstance;
		m_pInstance = nullptr;
	}
}
int Simplex::MyEntityManager::GetEntityIndex(String a_sUniqueID)
{
	//look one by one for the specified unique id
	for (uint uIndex = 0; uIndex < m_uEntityCount; ++uIndex)
	{
		if (a_sUniqueID == m_mEntityArray[uIndex]->GetUniqueID())
			return uIndex;
	}
	//if not found return -1
	return -1;
}
//Accessors
Simplex::uint Simplex::MyEntityManager::GetEntityCount(void) {	return m_uEntityCount; }
void Simplex::MyEntityManager::RecreateOctree(int m_uOctantLevels)
{
	//updates entity array in oct tree
	octree->RecreateTree(m_mEntityArray, m_uEntityCount, m_uOctantLevels);
}
void Simplex::MyEntityManager::SetGoalIndex(int counter)
{
	goalIndex = counter;
}
int Simplex::MyEntityManager::GetGoalIndex()
{
	return goalIndex;
}
bool Simplex::MyEntityManager::GetVictoryStatus()
{
	return victory;
}
bool Simplex::MyEntityManager::GetDeathStatus()
{
	return died;
}

void Simplex::MyEntityManager::Reset()
{
	Release();
	Init();
}

void Simplex::MyEntityManager::ToggleBoundingSphere(void)
{
	if (m_uEntityCount > 0) 
	{
		//gets opposite of current state of line visibility
		bool showBS = !m_mEntityArray[0]->GetRigidBody()->GetVisibleBS();

		//applies new visibility state to all entities
		for (uint i = 0; i < m_uEntityCount; i++)
		{
			m_mEntityArray[i]->GetRigidBody()->SetVisibleBS(showBS);
		}
	}
}

void Simplex::MyEntityManager::ToggleOrientedBoundingBox(void)
{
	if (m_uEntityCount > 0)
	{
		//gets opposite of current state of line visibility
		bool showOBB = !m_mEntityArray[0]->GetRigidBody()->GetVisibleOBB();

		//applies new visibility state to all entities
		for (uint i = 0; i < m_uEntityCount; i++)
		{
			m_mEntityArray[i]->GetRigidBody()->SetVisibleOBB(showOBB);
		}
	}
}

void Simplex::MyEntityManager::ToggleAxisRealignedboundingBox(void)
{
	if (m_uEntityCount > 0)
	{
		//gets opposite of current state of line visibility
		bool showARBB = !m_mEntityArray[0]->GetRigidBody()->GetVisibleARBB();

		//applies new visibility state to all entities
		for (uint i = 0; i < m_uEntityCount; i++)
		{
			m_mEntityArray[i]->GetRigidBody()->SetVisibleARBB(showARBB);
		}
	}
}

Simplex::Model* Simplex::MyEntityManager::GetModel(uint a_uIndex)
{
	//if the list is empty return
	if (m_uEntityCount == 0)
		return nullptr;

	// if out of bounds
	if (a_uIndex >= m_uEntityCount)
		a_uIndex = m_uEntityCount - 1;

	return m_mEntityArray[a_uIndex]->GetModel();
}
Simplex::Model* Simplex::MyEntityManager::GetModel(String a_sUniqueID)
{
	//Get the entity
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	//if the entity exists
	if (pTemp)
	{
		return pTemp->GetModel();
	}
	return nullptr;
}
Simplex::MyRigidBody* Simplex::MyEntityManager::GetRigidBody(uint a_uIndex)
{
	//if the list is empty return
	if (m_uEntityCount == 0)
		return nullptr;

	// if out of bounds
	if (a_uIndex >= m_uEntityCount)
		a_uIndex = m_uEntityCount - 1;

	return m_mEntityArray[a_uIndex]->GetRigidBody();
}
Simplex::MyRigidBody* Simplex::MyEntityManager::GetRigidBody(String a_sUniqueID)
{
	//Get the entity
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	//if the entity exists
	if (pTemp)
	{
		return pTemp->GetRigidBody();
	}
	return nullptr;
}
Simplex::matrix4 Simplex::MyEntityManager::GetModelMatrix(uint a_uIndex)
{
	//if the list is empty return
	if (m_uEntityCount == 0)
		return matrix4();

	// if out of bounds
	if (a_uIndex >= m_uEntityCount)
		a_uIndex = m_uEntityCount - 1;

	return m_mEntityArray[a_uIndex]->GetModelMatrix();
}
Simplex::matrix4 Simplex::MyEntityManager::GetModelMatrix(String a_sUniqueID)
{
	//Get the entity
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	//if the entity exists
	if (pTemp)
	{
		return pTemp->GetModelMatrix();
	}
	return IDENTITY_M4;
}
void Simplex::MyEntityManager::SetModelMatrix(matrix4 a_m4ToWorld, String a_sUniqueID)
{
	//Get the entity
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	//if the entity exists
	if (pTemp)
	{
		pTemp->SetModelMatrix(a_m4ToWorld);
	}
}
void Simplex::MyEntityManager::SetAxisVisibility(bool a_bVisibility, uint a_uIndex)
{
	//if the list is empty return
	if (m_uEntityCount == 0)
		return;

	//if the index is larger than the number of entries we are asking for the last one
	if (a_uIndex >= m_uEntityCount)
		a_uIndex = m_uEntityCount - 1;

	return m_mEntityArray[a_uIndex]->SetAxisVisible(a_bVisibility);
}
void Simplex::MyEntityManager::SetAxisVisibility(bool a_bVisibility, String a_sUniqueID)
{
	//Get the entity
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	//if the entity exists
	if (pTemp)
	{
		pTemp->SetAxisVisible(a_bVisibility);
	}
}
void Simplex::MyEntityManager::SetModelMatrix(matrix4 a_m4ToWorld, uint a_uIndex)
{
	//if the list is empty return
	if (m_uEntityCount == 0)
		return;

	//if the index is larger than the number of entries we are asking for the last one
	if (a_uIndex >= m_uEntityCount)
		a_uIndex = m_uEntityCount - 1;

	m_mEntityArray[a_uIndex]->SetModelMatrix(a_m4ToWorld);
}
//The big 3
Simplex::MyEntityManager::MyEntityManager(){Init();}
Simplex::MyEntityManager::MyEntityManager(MyEntityManager const& a_pOther){ }
Simplex::MyEntityManager& Simplex::MyEntityManager::operator=(MyEntityManager const& a_pOther) { return *this; }
Simplex::MyEntityManager::~MyEntityManager(){Release();};
// other methods
void Simplex::MyEntityManager::Update(int maxOctreeSubdivision, vector3 playerPosition, vector2 tiltValues)
{
	//get the y pos of armadillo and check if lower than like -50
	if (m_mEntityArray != nullptr)
	{
		if (m_mEntityArray[0]->GetModelMatrix()[3][1] < -50)
		{
			//stage needs to be reset and armadillo needs to be put back
			died = true;
		}

		//if player is touching goal's hitbox, advance to next stage
		if (m_mEntityArray[0]->GetRigidBody()->IsColliding(m_mEntityArray[GetGoalIndex()]->GetRigidBody())) {
			victory = true;
		}

		//Clear all collisions
		for (uint i = 0; i < m_uEntityCount; i++)
		{
			if (m_mEntityArray[i]->GetRigidBody()->GetStartingWorld() == IDENTITY_M4)
			{
				//Store original values of entities
				m_mEntityArray[i]->GetRigidBody()->SetStartingWorld(m_mEntityArray[i]->GetModelMatrix());
				m_mEntityArray[i]->GetRigidBody()->SetStartingCenter(m_mEntityArray[i]->GetRigidBody()->GetCenterGlobal());
			}
			m_mEntityArray[i]->GetRigidBody()->ClearCollidingList();
			if (m_mEntityArray[i]->GetRigidBody()->GetGravity())
			{

			}
		}

		for (uint i = 0; i < m_uEntityCount; i++)
		{
			if (m_mEntityArray[i]->GetIsTile())
			{
				m_mEntityArray[i]->GetRigidBody()->RotateAroundPoint(playerPosition, tiltValues);
				m_mEntityArray[i]->SetModelMatrix(m_mEntityArray[i]->GetRigidBody()->GetModelMatrix());
			}
		}

		//have octree update where entities are and go through its nodes to find collisions
		octree->UpdateTree(maxOctreeSubdivision);
		bool found = false;
		int reboundCount = 0;
		string previousName = "";
		//stop things that are colliding
		for (uint i = 0; i < m_uEntityCount; i++)
		{
			if (m_mEntityArray[i]->GetRigidBody()->GetGravity())
			{
				reboundCount = 0;
				for (uint j = 0; j < m_uEntityCount; ++j)
				{
					if (SharesDimension(m_mEntityArray[i]->GetUniqueID(), m_mEntityArray[j]))
					{
						vector3 previousVelocity = m_mEntityArray[i]->GetRigidBody()->GetDynamicVelocity();

						if (m_mEntityArray[i]->GetRigidBody()->IsColliding(m_mEntityArray[j]->GetRigidBody()))
						{
							if (i == 0)
							{
								if (reboundCount != 0)
								{
									if (glm::dot(previousVelocity, m_mEntityArray[i]->GetRigidBody()->GetDynamicVelocity()) < 0.5f)
									{
										//rebounds at a weaker speed for sphere/sphere collision
										if (m_mEntityArray[j]->GetRigidBody()->GetGravity()) {

											m_mEntityArray[i]->GetRigidBody()->Rebound(m_mEntityArray[j]->GetRigidBody(), 0.2f, m_mEntityArray[j]->GetName());
										}

										//sphere/solid collision
										else if (previousVelocity.y < 0.5f && reboundCount == 1)
										{
											m_mEntityArray[i]->GetRigidBody()->Rebound(m_mEntityArray[j]->GetRigidBody(), 0.85f, m_mEntityArray[j]->GetName());
											//found = true;
											//break;
										}
										else if (previousName != m_mEntityArray[j]->GetName())
										{
											m_mEntityArray[i]->GetRigidBody()->Rebound(m_mEntityArray[j]->GetRigidBody(), 0.85f, m_mEntityArray[j]->GetName());
											//found = true;
											//break;
										}
									}
								}
								else
								{
									previousName = m_mEntityArray[j]->GetName();
									m_mEntityArray[i]->GetRigidBody()->Rebound(m_mEntityArray[j]->GetRigidBody(), 0.85f, m_mEntityArray[j]->GetName());
								}
								reboundCount += 1;
							}
							else
							{
								//rebounds at a weaker speed for sphere/sphere collision
								if (m_mEntityArray[j]->GetRigidBody()->GetGravity()) {

									m_mEntityArray[i]->GetRigidBody()->Rebound(m_mEntityArray[j]->GetRigidBody(), 0.3f, m_mEntityArray[j]->GetName());
								}
								else {
									m_mEntityArray[i]->GetRigidBody()->Rebound(m_mEntityArray[j]->GetRigidBody(), 2.1f, m_mEntityArray[j]->GetName());
								}
							}
						}
					}
					//if (found) { break; }
				}
				//if (found) { break; }
			}
		}
		for (uint i = 0; i < m_uEntityCount; i++)
		{
			if (m_mEntityArray[i]->GetRigidBody()->GetGravity())
			{
				m_mEntityArray[i]->GetRigidBody()->ApplyForce();
				m_mEntityArray[i]->GetRigidBody()->ApplyVelocity();
				m_mEntityArray[i]->MoveEntity();
			}
		}
	}
	if (m_mEntityArray[0]->GetRigidBody()->IsColliding(m_mEntityArray[GetGoalIndex()]->GetRigidBody()))
	{
		victory = true;
	}
}
	
void Simplex::MyEntityManager::AddEntity(String a_sFileName, bool a_bGravity, bool isStageTile, bool dynamic, float a_iMass, float speedMultiplier, String a_sUniqueID)
{
	//Create a temporal entity to store the object
	MyEntity* pTemp = new MyEntity(a_sFileName, a_bGravity, isStageTile, dynamic, a_iMass, speedMultiplier);
	//if I was able to generate it add it to the list
	if (pTemp->IsInitialized())
	{
		//create a new temp array with one extra entry
		PEntity* tempArray = new PEntity[m_uEntityCount + 1];
		//start from 0 to the current count
		uint uCount = 0;
		for (uint i = 0; i < m_uEntityCount; ++i)
		{
			tempArray[uCount] = m_mEntityArray[i];
			++uCount;
		}
		tempArray[uCount] = pTemp;
		//if there was an older array delete
		if (m_mEntityArray)
		{
			delete[] m_mEntityArray;
		}
		//make the member pointer the temp pointer
		m_mEntityArray = tempArray;
		//add one entity to the count
		++m_uEntityCount;
	}
}

//This is not set up correctly. 
void Simplex::MyEntityManager::RemoveEntity(uint a_uIndex)
{
	//if the list is empty return
	if (m_uEntityCount == 0)
		return;

	// if out of bounds choose the last one
	if (a_uIndex >= m_uEntityCount)
		a_uIndex = m_uEntityCount - 1;

	// if the entity is not the very last we swap it for the last one
	if (a_uIndex != m_uEntityCount - 1)
	{
		std::swap(m_mEntityArray[a_uIndex], m_mEntityArray[m_uEntityCount - 1]);
	}
	
	//and then pop the last one
	//create a new temp array with one less entry
	PEntity* tempArray = new PEntity[m_uEntityCount - 1];
	//start from 0 to the current count
	for (uint i = 0; i < m_uEntityCount - 1; ++i)
	{
		tempArray[i] = m_mEntityArray[i];
	}
	//if there was an older array delete
	if (m_mEntityArray)
	{
		delete[] m_mEntityArray;
	}
	//make the member pointer the temp pointer
	m_mEntityArray = tempArray;
	//add one entity to the count
	--m_uEntityCount;

	//updates entity array in oct tree
	octree->RecreateTree(m_mEntityArray, m_uEntityCount, 1);
}
void Simplex::MyEntityManager::RemoveEntity(String a_sUniqueID)
{
	int nIndex = GetEntityIndex(a_sUniqueID);
	RemoveEntity((uint)nIndex);

	//updates entity array in oct tree
	octree->RecreateTree(m_mEntityArray, m_uEntityCount, 1);
}
Simplex::String Simplex::MyEntityManager::GetUniqueID(uint a_uIndex)
{
	//if the list is empty return
	if (m_uEntityCount == 0)
		return "";

	//if the index is larger than the number of entries we are asking for the last one
	if (a_uIndex >= m_uEntityCount)
		a_uIndex = m_uEntityCount - 1;

	return m_mEntityArray[a_uIndex]->GetUniqueID();
}
Simplex::MyEntity* Simplex::MyEntityManager::GetEntity(uint a_uIndex)
{
	//if the list is empty return
	if (m_uEntityCount == 0)
		return nullptr;

	//if the index is larger than the number of entries we are asking for the last one
	if (a_uIndex >= m_uEntityCount)
		a_uIndex = m_uEntityCount - 1;

	return m_mEntityArray[a_uIndex];
}
void Simplex::MyEntityManager::AddEntityToRenderList(uint a_uIndex, bool a_bRigidBody)
{
	//if out of bounds will do it for all
	if (a_uIndex >= m_uEntityCount)
	{
		//add for each one in the entity list
		for (a_uIndex = 0; a_uIndex < m_uEntityCount; ++a_uIndex)
		{
			if (m_uOctantID == -1 || m_mEntityArray[a_uIndex]->IsInDimension(m_uOctantID)) {
				m_mEntityArray[a_uIndex]->AddToRenderList(a_bRigidBody);
			}
		}
	}
	else //do it for the specified one
	{
		m_mEntityArray[a_uIndex]->AddToRenderList(a_bRigidBody);
	}
}
void Simplex::MyEntityManager::AddEntityToRenderList(String a_sUniqueID, bool a_bRigidBody)
{
	//Get the entity
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	//if the entity exists
	if (pTemp)
	{
		pTemp->AddToRenderList(a_bRigidBody);
	}
}
void Simplex::MyEntityManager::AddOctreeToRenderList(bool a_bOutline)
{
	octree->AddToRenderList(a_bOutline, m_uOctantID);
}
void Simplex::MyEntityManager::AddDimension(uint a_uIndex, uint a_uDimension)
{
	//if the list is empty return
	if (m_uEntityCount == 0)
		return;

	//if the index is larger than the number of entries we are asking for the last one
	if (a_uIndex >= m_uEntityCount)
		a_uIndex = m_uEntityCount - 1;

	return m_mEntityArray[a_uIndex]->AddDimension(a_uDimension);
}
void Simplex::MyEntityManager::AddDimension(String a_sUniqueID, uint a_uDimension)
{
	//Get the entity
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	//if the entity exists
	if (pTemp)
	{
		pTemp->AddDimension(a_uDimension);
	}
}
void Simplex::MyEntityManager::RemoveDimension(uint a_uIndex, uint a_uDimension)
{
	//if the list is empty return
	if (m_uEntityCount == 0)
		return;

	//if the index is larger than the number of entries we are asking for the last one
	if (a_uIndex >= m_uEntityCount)
		a_uIndex = m_uEntityCount - 1;

	return m_mEntityArray[a_uIndex]->RemoveDimension(a_uDimension);
}
void Simplex::MyEntityManager::RemoveDimension(String a_sUniqueID, uint a_uDimension)
{
	//Get the entity
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	//if the entity exists
	if (pTemp)
	{
		pTemp->RemoveDimension(a_uDimension);
	}
}
void Simplex::MyEntityManager::ClearDimensionSetAll(void)
{
	for (uint i = 0; i < m_uEntityCount; ++i)
	{
		ClearDimensionSet(i);
	}
}
void Simplex::MyEntityManager::ClearDimensionSet(uint a_uIndex)
{
	//if the list is empty return
	if (m_uEntityCount == 0)
		return;

	//if the index is larger than the number of entries we are asking for the last one
	if (a_uIndex >= m_uEntityCount)
		a_uIndex = m_uEntityCount - 1;

	return m_mEntityArray[a_uIndex]->ClearDimensionSet();
}
void Simplex::MyEntityManager::ClearDimensionSet(String a_sUniqueID)
{
	//Get the entity
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	//if the entity exists
	if (pTemp)
	{
		pTemp->ClearDimensionSet();
	}
}
bool Simplex::MyEntityManager::IsInDimension(uint a_uIndex, uint a_uDimension)
{
	//if the list is empty return
	if (m_uEntityCount == 0)
		return false;

	//if the index is larger than the number of entries we are asking for the last one
	if (a_uIndex >= m_uEntityCount)
		a_uIndex = m_uEntityCount - 1;

	return m_mEntityArray[a_uIndex]->IsInDimension(a_uDimension);
}
bool Simplex::MyEntityManager::IsInDimension(String a_sUniqueID, uint a_uDimension)
{
	//Get the entity
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	//if the entity exists
	if (pTemp)
	{
		return pTemp->IsInDimension(a_uDimension);
	}
	return false;
}
bool Simplex::MyEntityManager::SharesDimension(uint a_uIndex, MyEntity* const a_pOther)
{
	//if the list is empty return
	if (m_uEntityCount == 0)
		return false;

	//if the index is larger than the number of entries we are asking for the last one
	if (a_uIndex >= m_uEntityCount)
		a_uIndex = m_uEntityCount - 1;

	return m_mEntityArray[a_uIndex]->SharesDimension(a_pOther);
}
bool Simplex::MyEntityManager::SharesDimension(String a_sUniqueID, MyEntity* const a_pOther)
{
	//Get the entity
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	//if the entity exists
	if (pTemp)
	{
		return pTemp->SharesDimension(a_pOther);
	}
	return false;
}