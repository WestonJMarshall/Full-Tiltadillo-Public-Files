/*----------------------------------------------
Programmer: Alex Sarnese (ajs2877@gmail.com)
Date: 2019/10
----------------------------------------------*/
#ifndef __OCTREENODE_H_
#define __OCTREENODE_H_

#include "MyEntity.h"
#include "Simplex\Simplex.h"
#include <list> 

using namespace std;

namespace Simplex
{

	//System Class
	class OctreeNode
	{
		static list<int> OctantIdList;
		typedef MyEntity* PEntity; //MyEntity Pointer
		typedef OctreeNode* POctreeNode; //MyEntity Pointer
		MeshManager* m_pMeshMngr = nullptr; //for displaying the bounding box
		vector3 maxCoordinates; //max bounds of this node
		vector3 minCoordinates; //min bounds of this node
		vector3 widthSizes; //widths of box to make subdividing easier
		int subdivisionLevel;
		std::vector<POctreeNode> octreeNodeArray; // holds all 8 subdivisions if any
		PEntity* m_mEntityArray = nullptr; //array of MyEntity pointers in this node
		int entityArraySize = 0;
		int octantID = 0;


	public:
		/*
		Usage: constructor
		Arguments: 
		-   int subdivisionLevelIn -> what subdivision level this node is on
		-   vector3 maxCoordinatesIn -> what is the maximum coordinate of this node's area
		-   vector3 minCoordinatesIn -> what is the minimum coordinate of this node's area
		Output: class object instance
		*/
		OctreeNode(int subdivisionLevelIn, vector3 maxCoordinatesIn, vector3 minCoordinatesIn);
		/*
		Usage: copy constructor
		Arguments: class object to copy
		Output: class object instance
		*/
		OctreeNode(OctreeNode const& a_pOther);
		/*
		Usage: copy assignment operator
		Arguments: class object to copy
		Output: ---
		*/
		OctreeNode& operator=(OctreeNode const& a_pOther);
		/*
		Usage: destructor
		Arguments: ---
		Output: ---
		*/
		~OctreeNode(void);
		/*
		USAGE: creates the 8 subdivisions and will add all entities within their space to them
		ARGUMENTS: 
		-   int maxLevel -> how deep the tree can go on its own
		OUTPUT: ---
		*/
		void UpdateTree(int maxLevel);
		/*
		USAGE: creates the 8 subdivisions and will add all entities within their space to them
		ARGUMENTS: 
		-   int targetLevel -> how deep to make all octants
		OUTPUT: ---
		*/
		void CreateSubdivisions(int targetLevel);
		/*
		USAGE: draws the bounding box of this node and calls AddToRenderList() of subdivision nodes
		ARGUMENTS: 
		-   bool a_bOutline -> draw outline?
		-   int octantIdToOutline -> what octant to outline (-1 outlines all)
		OUTPUT: ---
		*/
		void AddToRenderList(bool a_bOutline, int octantIdToOutline);
		/*
		USAGE: Checks to see if the entity is inside the min and max coordinates of this node
		ARGUMENTS:
		-	String entityToCheck -> entity to check if in bounds
		OUTPUT: whether the entity is in bounds of the node
		*/
		bool IsEntityInBoundary(MyEntity* entityToCheck);
		/*
		USAGE: Will add an entity to the list
		ARGUMENTS:
		-	String entityToAdd -> entity to be added
		OUTPUT: ---
		*/
		void AddEntity(MyEntity* entityToAdd);
		/*
		USAGE: removes entity from this node and its subnodes
		ARGUMENTS:
		-	uint a_uIndex -> index of entity to remove
		OUTPUT: ---
		*/
		void RemoveEntity(uint a_uIndex);
		/*
		USAGE: clears this node's list of all subnodes
		ARGUMENTS: ---
		OUTPUT: ---
		*/
		void ClearSubnodes();
		/*
		USAGE: Finds the index of the entity in this node
		ARGUMENTS:
		-	String a_sUniqueID -> ID of the entity to find
		OUTPUT: index of entity
		*/
		int GetEntityIndex(String a_sUniqueID);
		/*
		USAGE: replaces current entity array with new array and remake all subnodes
		ARGUMENTS:
		-	PEntity* m_mEntityArrayIn -> entity array to copy
		-   int arraySize -> how many entities is coming in
		-   int subdivisionLevelIn -> what subdivision level this node is on
		OUTPUT: ---
		*/
		void OctreeNode::RecreateTree(PEntity* m_mEntityArrayIn, int arraySize, int targetLevel);
		/*
		USAGE: returns how many nodes there are
		ARGUMENTS: ---
		OUTPUT: number of nodes in this octree
		*/
		int OctreeSize(void);
	private:
	};//class

} //namespace Simplex

#endif //__OCTREENODE_H_

/*
USAGE:
ARGUMENTS: ---
OUTPUT: ---
*/