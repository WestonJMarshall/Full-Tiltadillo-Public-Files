#include "OctreeNode.h"
using namespace Simplex;

///////////////////////////////////////
//constructor and the big 3

list<int> OctreeNode::OctantIdList;

OctreeNode::OctreeNode(int subdivisionLevelIn, vector3 maxCoordinatesIn, vector3 minCoordinatesIn)
{
	subdivisionLevel = subdivisionLevelIn;
	maxCoordinates = maxCoordinatesIn;
	minCoordinates = minCoordinatesIn;
	widthSizes = ((maxCoordinates - minCoordinates) / 2);
	entityArraySize = 0;
	m_mEntityArray = nullptr;
	m_pMeshMngr = MeshManager::GetInstance();

	//gets a valid ID as no ID will be size of list
	octantID = OctantIdList.size();
	OctantIdList.push_back(octantID);
}

OctreeNode::OctreeNode(OctreeNode const& a_pOther)
{
	subdivisionLevel = a_pOther.subdivisionLevel;
	maxCoordinates = a_pOther.maxCoordinates;
	minCoordinates = a_pOther.minCoordinates;

	//copies tree nodes
	for (int i = 0; i < 8; i++) 
	{
		octreeNodeArray.push_back(a_pOther.octreeNodeArray.at(i));
	}
	

	//copies entity list
	if (a_pOther.m_mEntityArray != nullptr)
	{
		m_mEntityArray = new PEntity[a_pOther.entityArraySize];
		for (int i = 0; i < a_pOther.entityArraySize; i++)
		{
			m_mEntityArray[i] = a_pOther.m_mEntityArray[i];
		}
		entityArraySize = a_pOther.entityArraySize;
	}
}

OctreeNode& OctreeNode::operator=(OctreeNode const& a_pOther)
{
	return *this;
}

OctreeNode::~OctreeNode(void)
{
	for (int i = octreeNodeArray.size() - 1; i >= 0; i--) {
		delete octreeNodeArray.at(i);
		octreeNodeArray.pop_back();
	}
	delete[] m_mEntityArray;
	m_mEntityArray = nullptr;
	m_pMeshMngr = nullptr;
	OctantIdList.remove(octantID);//free up ID
}


////////////////////////////////////////

/*
* To be called in MyEntityManager's Update
*
* Will check if all nodes are in root node and resize if needed,
* move entities to correct node as entity moves,
* and subdivide further if a node gets too many entities.
*/
void Simplex::OctreeNode::UpdateTree(int maxLevel)
{
	//checks to see if any entity is out of bounds in root node.
	//If so, we need to remake entire tree in a new larger size.
	if (subdivisionLevel == 0) 
	{
		for (int i = 0; i < entityArraySize; i++) 
		{
			if (!IsEntityInBoundary(m_mEntityArray[i])) 
			{
				RecreateTree(m_mEntityArray, entityArraySize, maxLevel);

				//tree is remade and so we don't need to continue with rest of code
				return;
			}
		}
	}

	//adds/removes entities to and from subnodes 
	for (int i = entityArraySize -1; i >= 0; i--)
	{
		if (m_mEntityArray[i]->GetRigidBody()->GetGravity())
		{
			for (int nodeIndex = 0; nodeIndex < octreeNodeArray.size(); nodeIndex++)
			{
				//entity thinks it is in subnode but actually is not in the subnode
				if (m_mEntityArray[i]->IsInDimension(octreeNodeArray[nodeIndex]->octantID) 
					&& !octreeNodeArray[nodeIndex]->IsEntityInBoundary(m_mEntityArray[i]))
				{
					octreeNodeArray[nodeIndex]->RemoveEntity(
						octreeNodeArray[nodeIndex]->GetEntityIndex(m_mEntityArray[i]->GetUniqueID()));

					//subnode doesn't have enough entities to have subdivisions 
					//so clear the subnode's subnode list
					if (octreeNodeArray[nodeIndex]->entityArraySize < 3) 
					{
						octreeNodeArray[nodeIndex]->ClearSubnodes();
					}
				}
				//entity thinks it is not in subnode but actually is in the subnode
				else if (!m_mEntityArray[i]->IsInDimension(octreeNodeArray[nodeIndex]->octantID)
					&& octreeNodeArray[nodeIndex]->IsEntityInBoundary(m_mEntityArray[i]))
				{
					octreeNodeArray[nodeIndex]->AddEntity(m_mEntityArray[i]);

					//subnode has too many entities so clear the subnode's subnode list
					//so we can subdivide further if needed
					if (octreeNodeArray[nodeIndex]->entityArraySize > 2)
					{
						octreeNodeArray[nodeIndex]->ClearSubnodes();
					}
				}
			}
		}
	}

	//repeat for subnodes so they can check to see if entity moved around inside them
	//within their own subnodes
	for (int nodeIndex = 0; nodeIndex < octreeNodeArray.size(); nodeIndex++)
	{
		//if subnode has more than 2 entities but no subdivisions, subdivide
		if (octreeNodeArray[nodeIndex]->entityArraySize > 2
			&& octreeNodeArray[nodeIndex]->octreeNodeArray.size() == 0)
		{
			octreeNodeArray[nodeIndex]->CreateSubdivisions(maxLevel);
		}

		octreeNodeArray[nodeIndex]->UpdateTree(maxLevel);
	}
}

/*
* Will try to subdivide if it is a leaf node but if not, then it'll have the 8 nodes attempt to subdivide instead.
*/
void OctreeNode::CreateSubdivisions(int targetLevel)
{
	//if (octantID == 0) {
	//	for (int i = 0; i < entityArraySize; i++) {
	//		//clears the dimensions at root node so the entities gets new updated value of what octant they are in
	//		m_mEntityArray[i]->ClearDimensionSet();
	//	}
	//}
	for (int i = 0; i < entityArraySize; i++) {
		//tells entities what octant it is in
		m_mEntityArray[i]->AddDimension(octantID);
	}

	//if the current node is the target level, ends the recursion and delete subnodes since this is the new leaf node
	//
	// Was testing code to make octree subdivide on its own to min octant but was slow
	// || (targetLevel == -1 && subdivisionLevel <= 3 && entityArraySize <= 3)
	if (subdivisionLevel == targetLevel)
	{
		ClearSubnodes();
		return;
	}

	//if this is a leaf node, attempt to create subdivisions to reach targetLevel 
	if (octreeNodeArray.size() == 0) {
		//if greater than 1, we do have to subdivide into 8 more sections
		if (entityArraySize > 1) 
		{
			//new bounding size of node 
			//does bottom first in counterclockwise fashion and then top nodes
			//remember, I add widthSize to the end of the max parameter so (min+minwidth ) + mindwidth = max
			//It is messy looking but as simple as I can do for now
			octreeNodeArray.push_back(new OctreeNode(subdivisionLevel + 1, 
				minCoordinates + widthSizes, 
				minCoordinates));
			octreeNodeArray.push_back(new OctreeNode(subdivisionLevel + 1,
				vector3(minCoordinates.x + widthSizes.x, minCoordinates.y, minCoordinates.z) + widthSizes, 
				vector3(minCoordinates.x + widthSizes.x, minCoordinates.y, minCoordinates.z)));
			octreeNodeArray.push_back(new OctreeNode(subdivisionLevel + 1,
				vector3(minCoordinates.x + widthSizes.x, minCoordinates.y, minCoordinates.z + widthSizes.z) + widthSizes,
				vector3(minCoordinates.x + widthSizes.x, minCoordinates.y, minCoordinates.z + widthSizes.z)));
			octreeNodeArray.push_back(new OctreeNode(subdivisionLevel + 1,
				vector3(minCoordinates.x, minCoordinates.y, minCoordinates.z + widthSizes.z) + widthSizes,
				vector3(minCoordinates.x, minCoordinates.y, minCoordinates.z + widthSizes.z)));
			octreeNodeArray.push_back(new OctreeNode(subdivisionLevel + 1,
				vector3(minCoordinates.x, minCoordinates.y + widthSizes.y, minCoordinates.z) + widthSizes,
				vector3(minCoordinates.x, minCoordinates.y + widthSizes.y, minCoordinates.z)));
			octreeNodeArray.push_back(new OctreeNode(subdivisionLevel + 1,
				vector3(minCoordinates.x + widthSizes.x, minCoordinates.y + widthSizes.y, minCoordinates.z) + widthSizes,
				vector3(minCoordinates.x + widthSizes.x, minCoordinates.y + widthSizes.y, minCoordinates.z)));
			octreeNodeArray.push_back(new OctreeNode(subdivisionLevel + 1,
				vector3(minCoordinates.x + widthSizes.x, minCoordinates.y + widthSizes.y, minCoordinates.z + widthSizes.z) + widthSizes,
				vector3(minCoordinates.x + widthSizes.x, minCoordinates.y + widthSizes.y, minCoordinates.z + widthSizes.z)));
			octreeNodeArray.push_back(new OctreeNode(subdivisionLevel + 1,
				vector3(minCoordinates.x, minCoordinates.y + widthSizes.y, minCoordinates.z + widthSizes.z) + widthSizes,
				vector3(minCoordinates.x, minCoordinates.y + widthSizes.y, minCoordinates.z + widthSizes.z)));


			//checks to see if entity is colliding with node and if so, add it to the node's list
			for (int nodeIndex = 0; nodeIndex < 8; nodeIndex++)
			{
				for (int entityIndex = 0; entityIndex < entityArraySize; entityIndex++) 
				{
					//will not add entity if it isn't inside this node
					if (octreeNodeArray[nodeIndex]->IsEntityInBoundary(m_mEntityArray[entityIndex])) 
					{
						//attempts to add the entity to the subnode
						octreeNodeArray[nodeIndex]->AddEntity(m_mEntityArray[entityIndex]);
					}
				}
			}

			for (int i = 0; i < octreeNodeArray.size(); i++)
			{
				octreeNodeArray[i]->CreateSubdivisions(targetLevel);
			}
		}
	}
	//if it is not a leaf node, then have the subnodes try to subdivide to targetLevel
	else {
		for (int i = 0; i < octreeNodeArray.size(); i++)
		{
			octreeNodeArray[i]->CreateSubdivisions(targetLevel);
		}
	}
}

void OctreeNode::AddToRenderList(bool a_bOutline, int octantIdToOutline)
{
		if (octantIdToOutline == -1 || octantIdToOutline == octantID)
		{
			//shows outline
			if (a_bOutline)
			{
				m_pMeshMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, minCoordinates + widthSizes) * glm::scale(widthSizes * 2.0f), C_RED);
			}
		}

		for (int i = 0; i < octreeNodeArray.size(); i++)
		{
			octreeNodeArray[i]->AddToRenderList(a_bOutline, octantIdToOutline);
		}
}

bool OctreeNode::IsEntityInBoundary(MyEntity* entityToCheck)
{
	vector3 entityCenter = entityToCheck->GetRigidBody()->GetCenterGlobal();
	vector3 entityHalfWidth = entityToCheck->GetRigidBody()->GetHalfWidth();

	//we are in a way, expanding out the boundary of this node by the entity's halfwidth so we can 
	//check for entities whose rigibodies barely makes it inside the node's boundary. 
	if ((entityCenter.x > minCoordinates.x - entityHalfWidth.x &&
		entityCenter.x < maxCoordinates.x + entityHalfWidth.x) && 

		(entityCenter.y > minCoordinates.y - entityHalfWidth.y &&
		entityCenter.y < maxCoordinates.y + entityHalfWidth.y) &&

		(entityCenter.z > minCoordinates.z - entityHalfWidth.z &&
		entityCenter.z < maxCoordinates.z + entityHalfWidth.z))
	{
		return true;
	}

	return false;
}

/*
* Code is nearly the same as the AddEntity method in MyEntityManager.cpp
* Only this time we pass in a pointer to the entity as it is already made in MyEntityManager
*
* Entities added gets added to all subnodes that it is inside of so we do not need to remake all nodes.
*/
void OctreeNode::AddEntity(MyEntity* entityToAdd)
{
	//entity is being added to tell it what octant it just got in
	entityToAdd->AddDimension(octantID);

	//Create a temporal entity to store the object
	MyEntity* pTemp = entityToAdd;
	//if entity exists
	if (pTemp->IsInitialized())
	{
		//create a new temp array with one extra entry
		PEntity* tempArray = new PEntity[entityArraySize + 1];
		//start from 0 to the current count
		uint uCount = 0;
		for (uint i = 0; i < entityArraySize; ++i)
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
		++entityArraySize;

		//now has subnodes attempt to add the entity
		for (int i = 0; i < octreeNodeArray.size(); i++)
		{
			if (octreeNodeArray[i]->IsEntityInBoundary(entityToAdd)) 
			{
				octreeNodeArray[i]->AddEntity(entityToAdd);
			}
		}
	}
}

/*
* Removes entity from this node any any subnode
* And tells entity to forget this node in its dimension list
*/
void OctreeNode::RemoveEntity(uint a_uIndex)
{
	//if the list is empty or index is -1 return
	if (entityArraySize == 0 || a_uIndex == -1)
		return;

	// if out of bounds choose the last one
	if (a_uIndex >= entityArraySize)
		a_uIndex = entityArraySize - 1;

	// if the entity is not the very last we swap it for the last one
	if (a_uIndex != entityArraySize - 1)
	{
		std::swap(m_mEntityArray[a_uIndex], m_mEntityArray[entityArraySize - 1]);
	}

	
	PEntity tempEntity = m_mEntityArray[entityArraySize - 1];
	tempEntity->RemoveDimension(octantID);

	//create a new temp array with one less entry
	PEntity* tempArray = new PEntity[entityArraySize - 1];
	//start from 0 to the current count
	for (uint i = 0; i < entityArraySize - 1; ++i)
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
	--entityArraySize;


	//repeat removal for every subnode if the entity thinks it is still in it
	//even though it is not in this main node
	for (int nodeIndex = 0; nodeIndex < octreeNodeArray.size(); nodeIndex++)
	{
		for (int i = entityArraySize - 1; i >= 0; i--)
		{
			if (m_mEntityArray[i]->IsInDimension(octreeNodeArray[nodeIndex]->octantID))
			{
				octreeNodeArray[nodeIndex]->RemoveEntity(GetEntityIndex(tempEntity->GetUniqueID()));
			}
		}
	}
}

/*
* Deletes all subnodes
*/
void OctreeNode::ClearSubnodes() 
{
	for (int i = octreeNodeArray.size() - 1; i >= 0; i--) {
		delete octreeNodeArray.at(i);
		octreeNodeArray.pop_back();
	}
}

/*
* finds entity index in this node
*/
int OctreeNode::GetEntityIndex(String a_sUniqueID)
{
	//look one by one for the specified unique id
	for (uint uIndex = 0; uIndex < entityArraySize; ++uIndex)
	{
		if (a_sUniqueID == m_mEntityArray[uIndex]->GetUniqueID())
			return uIndex;
	}
	//if not found return -1
	return -1;
}


/*
* To be called when an entity is removed
*/
void OctreeNode::RecreateTree(PEntity* m_mEntityArrayIn, int arraySize, int targetLevel)
{
	//sets array of all entities for this node
	for (int i = octreeNodeArray.size() - 1; i >= 0; i--) 
	{
		delete octreeNodeArray.at(i);
		octreeNodeArray.pop_back();
	}
	m_mEntityArray = m_mEntityArrayIn;
	entityArraySize = arraySize;

	//reset octree size to start at first entity coordinates 
	if (m_mEntityArray[0] != NULL) {
		maxCoordinates = minCoordinates = m_mEntityArray[0]->GetRigidBody()->GetCenterGlobal();
	}

	// Resize this root node to fit all entities that this holds before subdividing more if needed
	// Subnodes will not call this method and instead are just perfect octants of parent node.
	for (int entityIndex = 0; entityIndex < arraySize; entityIndex++) 
	{
		//adds root node to entity after resetting dimensions
		m_mEntityArray[entityIndex]->ClearDimensionSet();
		m_mEntityArray[entityIndex]->AddDimension(octantID);
		vector3 entityVect = m_mEntityArray[entityIndex]->GetRigidBody()->GetCenterGlobal();

		maxCoordinates = vector3(
			glm::max(maxCoordinates.x, entityVect.x), 
			glm::max(maxCoordinates.y, entityVect.y), 
			glm::max(maxCoordinates.z, entityVect.z));
		minCoordinates = vector3(
			glm::min(minCoordinates.x, entityVect.x),
			glm::min(minCoordinates.y, entityVect.y),
			glm::min(minCoordinates.z, entityVect.z));
	}
	//enclose a slightly larger area to reduce needing to remake tree later if objects move
	vector3 extraSize = vector3(10, 10, 10);
	minCoordinates -= extraSize;
	maxCoordinates += extraSize;
	widthSizes = ((maxCoordinates - minCoordinates) / 2);

	CreateSubdivisions(targetLevel);
}

int Simplex::OctreeNode::OctreeSize(void)
{
	return OctantIdList.size();
}

