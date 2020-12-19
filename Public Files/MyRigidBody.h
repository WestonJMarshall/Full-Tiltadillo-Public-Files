/*----------------------------------------------
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2017/06
----------------------------------------------*/
#ifndef __MYRIGIDBODY_H_
#define __MYRIGIDBODY_H_

#include "Simplex\Mesh\Model.h"
#include "imgui\ImGuiObject.h"

namespace Simplex
{

//System Class
class MyRigidBody
{
	typedef MyRigidBody* PRigidBody; //Entity Pointer
	MeshManager* m_pMeshMngr = nullptr; //for displaying the Rigid Body

	bool m_bVisibleBS = false; //Visibility of bounding sphere
	bool m_bVisibleOBB = true; //Visibility of Oriented bounding box
	bool m_bVisibleARBB = true; //Visibility of axis (Re)aligned bounding box

	bool dynamicSphere = false;

	float m_fRadius = 0.0f; //Radius

	vector3 startingCenter = ZERO_V3;

	std::vector<vector3> pointListLocal;
	std::vector<vector3> pointListGlobal;

	vector3 m_v3ColorColliding = C_RED; //Color when colliding
	vector3 m_v3ColorNotColliding = C_WHITE; //Color when not colliding

	vector3 m_v3CenterL = ZERO_V3; //center point in local space
	vector3 m_v3CenterG = ZERO_V3; //center point in global space

	vector3 m_v3MinL = ZERO_V3; //minimum coordinate in local space (for OBB)
	vector3 m_v3MaxL = ZERO_V3; //maximum coordinate in local space (for OBB)

	vector3 m_v3MinG = ZERO_V3; //minimum coordinate in global space (for ARBB)
	vector3 m_v3MaxG = ZERO_V3; //maximum coordinate in global space (for ARBB)

	vector3 m_v3HalfWidth = ZERO_V3; //half the size of the Oriented Bounding Box
	vector3 m_v3ARBBSize = ZERO_V3;// size of the Axis (Re)Alligned Bounding Box

	matrix4 m_m4ToWorld = IDENTITY_M4; //Matrix that will take us from local to world coordinate
	matrix4 fixedMatrix = IDENTITY_M4; //Matrix that will take us from local to world coordinate

	uint m_nCollidingCount = 0; //size of the colliding set
	PRigidBody* m_CollidingArray = nullptr; //array of rigid bodies this one is colliding with

	matrix4 startingWorld = IDENTITY_M4;

	bool m_bGravity = false; // whether the object is affected by gravity
	float m_fMass = 0.0f; // the mass of the object, for when forces are applied
	vector3 m_v3Velocity = ZERO_V3; // the velocity vector of the object
	vector3 m_v3Acceleration = ZERO_V3; // the acceleration vector of the object

	vector3 dynamicOffset = ZERO_V3;
	vector3 dynamicVelocity = ZERO_V3;

	float speedMultiplier = 0.0f;

public:
	/*
	Usage: Constructor
	Arguments: std::vector<vector3> a_pointList -> list of points to make the Rigid Body for
	Output: class object instance
	*/
	MyRigidBody(std::vector<vector3> a_pointList);
	/*
	Usage: Constructor
	Arguments: std::vector<vector3> a_pointList -> list of points to make the Rigid Body for
	Output: class object instance
	*/
	MyRigidBody(std::vector<vector3> a_pointList, bool dynamic);
	/*
	Usage: Copy Constructor
	Arguments: class object to copy
	Output: class object instance
	*/
	MyRigidBody(MyRigidBody const& other);
	/*
	Usage: Copy Assignment Operator
	Arguments: class object to copy
	Output: ---
	*/
	MyRigidBody& operator=(MyRigidBody const& other);
	/*
	Usage: Destructor
	Arguments: ---
	Output: ---
	*/
	~MyRigidBody(void);
	/*
	Usage: Changes object contents for other object's
	Arguments: other -> object to swap content from
	Output: ---
	*/
	void Swap(MyRigidBody& other);

	/*
	USAGE: Will render the Rigid Body at the model to world matrix
	ARGUMENTS: ---
	OUTPUT: ---
	*/
	void AddToRenderList(void);

	void UpdatePointList(void);

	/*
	USAGE: Clears the colliding list
	ARGUMENTS: ---
	OUTPUT: ---
	*/
	void ClearCollidingList(void);

	/*
	USAGE: Mark collision with the incoming Rigid Body
	ARGUMENTS: MyRigidBody* other -> inspected rigid body
	OUTPUT: ---
	*/
	void AddCollisionWith(MyRigidBody* other);

	/*
	USAGE: Remove marked collision with the incoming Rigid Body
	ARGUMENTS: MyRigidBody* other -> inspected rigid body
	OUTPUT: ---
	*/
	void RemoveCollisionWith(MyRigidBody* other);

	/*
	USAGE: Tells if the object is colliding with the incoming one
	ARGUMENTS: MyRigidBody* const other -> inspected rigid body
	OUTPUT: are they colliding?
	*/
	bool IsColliding(MyRigidBody* const other);
#pragma region Accessors
	/*
	Usage: Gets visibility of bounding sphere
	Arguments: ---
	Output: visibility
	*/
	bool GetVisibleBS(void);

	bool GetDynamicSphere(void);

	vector3 GetDynamicOffset(void);

	vector3 GetDynamicVelocity(void);

	std::vector<vector3> GetPointListGlobal(void);
	/*
	Usage: Sets visibility of bounding sphere
	Arguments: bool a_bVisibility -> visibility to set
	Output: ---
	*/
	void SetVisibleBS(bool a_bVisibility);
	/*
	Usage: Gets visibility of oriented bounding box
	Arguments: ---
	Output: visibility
	*/
	bool GetVisibleOBB(void);
	/*
	Usage: Sets visibility of oriented bounding box
	Arguments: bool a_bVisibility -> visibility to set
	Output: ---
	*/
	void SetVisibleOBB(bool a_bVisibility);
	/*
	Usage: Gets visibility of axis (re)aligned bounding box
	Arguments: ---
	Output: visibility
	*/
	bool GetVisibleARBB(void);
	/*
	Usage: Sets visibility of axis (re)aligned bounding box
	Arguments: bool a_bVisibility -> visibility to set
	Output: ---
	*/
	void SetVisibleARBB(bool a_bVisibility);
	/*
	Usage: Gets radius
	Arguments: ---
	Output: radius
	*/
	float GetRadius(void);
	/*
	Usage: Gets the color when colliding
	Arguments: ---
	Output: color
	*/
	vector3 GetColorColliding(void);
	/*
	Usage: Sets the color when colliding
	Arguments: vector3 a_v3Color -> color
	Output: ---
	*/
	void SetColorColliding(vector3 a_v3Color);
	/*
	Usage: Gets the color when not colliding
	Arguments: ---
	Output: color
	*/
	vector3 GetColorNotColliding(void);
	/*
	Usage: Sets the color when colliding
	Arguments: vector3 a_v3Color -> color
	Output: ---
	*/
	void SetColorNotColliding(vector3 a_v3Color);
	/*
	Usage: Gets center in local space
	Arguments: ---
	Output: center
	*/
	vector3 GetCenterLocal(void);
	/*
	Usage: Gets minimum vector in local space
	Arguments: ---
	Output: min vector
	*/
	vector3 GetMinLocal(void);
	/*
	Usage: Gets maximum vector in local space
	Arguments: ---
	Output: max vector
	*/
	vector3 GetMaxLocal(void);
	/*
	Usage: Gets center in global space
	Arguments: ---
	Output: center
	*/
	vector3 GetCenterGlobal(void);
	/*
	Usage: Gets minimum vector in global space
	Arguments: ---
	Output: min vector
	*/
	vector3 GetMinGlobal(void);
	/*
	Usage: Gets max vector in global space
	Arguments: ---
	Output: max vector
	*/
	vector3 GetMaxGlobal(void);
	/*
	Usage: Gets the size of the model divided by 2
	Arguments: ---
	Output: halfwidth vector
	*/
	vector3 GetHalfWidth(void);
	/*
	Usage: Gets Model to World matrix
	Arguments: ---
	Output: model to world matrix
	*/
	matrix4 GetModelMatrix(void);
	/*
	Usage: Sets Model to World matrix
	Arguments: Model to World matrix
	Output: ---
	*/
	void SetModelMatrix(matrix4 a_m4ModelMatrix);
	/*
	USAGE: Gets whether the rigidbody's colliding array is empty
	ARGUMENTS: ---
	OUTPUT: whether the rigidbody's colliding array is empty
	*/
	bool CollidingArrayIsEmpty(void);
	/*
	Usage: Sets how strongly this object rebounds
	Arguments: float reboundMultiplierIn -> how much rebounding the object has
	Output: ---
	*/
	void SetSpeedMultiplier(float reboundMultiplierIn);

#pragma endregion
	/*
	USAGE: Checks if the input is in the colliding array
	ARGUMENTS: MyRigidBody* a_pEntry -> Entry queried
	OUTPUT: is it in the array?
	*/
	bool IsInCollidingArray(MyRigidBody* a_pEntry);

	/*
	USAGE: Gets the other rigidbody within the colliding array at the given index
	ARGUMENTS: a_iIndex -> the index containing the desired rigidbody
	OUTPUT: a pointer to the desired rigidbody
	*/
	MyRigidBody* GetCollidingBody(int a_iIndex);

	/*
	USAGE: returns the number of colliding rigidbodies
	ARGUMENTS: ---
	OUTPUT: the number of colliding rigidbodies
	*/
	int GetNumberColliding(void);
	/*
	USAGE: Applies the force to the velocity
	ARGUMENTS: ---
	OUTPUT: ---
	*/
	void ApplyForce(void);
	/*
	USAGE: Applies the velocity to the position
	ARGUMENTS: ---
	OUTPUT: ---
	*/
	void ApplyVelocity(void);
	/*
	USAGE: Rebounds the object off of the other object
	ARGUMENTS: a_pOther -> the object this is rebounding off of
	OUTPUT: ---
	*/
	void Rebound(MyRigidBody* a_pOther, float strength, String otherName);
	/*
	USAGE: Adds to the forces acting on an object
	ARGUMENTS: a_v3Forces -> the force(s) being added
	OUTPUT: ---
	*/
	void AddForces(vector3 a_v3Forces);
	/*
	USAGE: Prevents movement through other models and limits possible speeds
	ARGUMENTS: ---
	OUTPUT: ---
	*/
	void ClampVelocity(void);
	/*
	USAGE: sets whether the entity is affected by gravity
	ARGUMENTS: ---
	OUTPUT: ---
	*/
	void SetGravity(bool a_bGravity);
	/*
	USAGE: Gets whether the entity is affected by gravity
	ARGUMENTS: ---
	OUTPUT: the bool indicating whether the entity is affected by gravity
	*/
	bool GetGravity(void);

	/*
	USAGE: Gets the mass of the entity
	ARGUMENTS: ---
	OUTPUT: the mass of the entity
	*/
	float GetMass(void);
	/*
	USAGE: Gets the mass of the entity
	ARGUMENTS: ---
	OUTPUT: the mass of the entity
	*/
	void SetMass(float a_fMass);
	/*
	USAGE: Gets the velocity of the entity
	ARGUMENTS: ---
	OUTPUT: the velocity of the entity
	*/
	vector3 GetVelocity(void);

	/*
	USAGE: Gets the acceleration of the entity
	ARGUMENTS: ---
	OUTPUT: the acceleration of the entity
	*/
	vector3 GetAcceleration(void);

	bool SphereBoxCollision(MyRigidBody* const a_pOther);

	matrix4 GetStartingWorld();

	void SetStartingWorld(matrix4 startMatrix);

	void RotateAroundPoint(vector3 point, vector2 tiltValues);

	void RotateDynamicAroundPoint(vector2 tiltValues);

	vector3 GetStartingCenter();

	void SetStartingCenter(vector3 startCenter);

private:
	/*
	Usage: Deallocates member fields
	Arguments: ---
	Output: ---
	*/
	void Release(void);
	/*
	Usage: Allocates member fields
	Arguments: ---
	Output: ---
	*/
	void Init(void);
	/*
	USAGE: This will apply the Separation Axis Test
	ARGUMENTS: MyRigidBody* const a_pOther -> other rigid body to test against
	OUTPUT: 0 for colliding, all other first axis that succeeds test
	*/
	uint SAT(MyRigidBody* const a_pOther);
};//class

} //namespace Simplex

#endif //__MYRIGIDBODY_H_

/*
USAGE:
ARGUMENTS: ---
OUTPUT: ---
*/
