#include "MyRigidBody.h"
using namespace Simplex;
//Allocation
void MyRigidBody::Init(void)
{
	m_pMeshMngr = MeshManager::GetInstance();
	m_bVisibleBS = false;
	m_bVisibleOBB = false;
	m_bVisibleARBB = false;

	dynamicOffset = ZERO_V3;
	dynamicVelocity = ZERO_V3;

	m_v3Acceleration = ZERO_V3;
	m_v3Velocity = ZERO_V3;

	m_fRadius = 0.0f;

	m_v3ColorColliding = C_RED;
	m_v3ColorNotColliding = C_WHITE;

	m_v3CenterL = ZERO_V3;
	m_v3CenterG = ZERO_V3;
	m_v3MinL = ZERO_V3;
	m_v3MaxL = ZERO_V3;

	m_v3MinG = ZERO_V3;
	m_v3MaxG = ZERO_V3;

	m_v3HalfWidth = ZERO_V3;
	m_v3ARBBSize = ZERO_V3;

	m_m4ToWorld = IDENTITY_M4;

	m_nCollidingCount = 0;
	m_CollidingArray = nullptr;
}
void MyRigidBody::Swap(MyRigidBody& other)
{
	std::swap(m_pMeshMngr, other.m_pMeshMngr);
	std::swap(m_bVisibleBS, other.m_bVisibleBS);
	std::swap(m_bVisibleOBB, other.m_bVisibleOBB);
	std::swap(m_bVisibleARBB, other.m_bVisibleARBB);

	std::swap(m_fRadius, other.m_fRadius);

	std::swap(m_v3ColorColliding, other.m_v3ColorColliding);
	std::swap(m_v3ColorNotColliding, other.m_v3ColorNotColliding);

	std::swap(m_v3CenterL, other.m_v3CenterL);
	std::swap(m_v3CenterG, other.m_v3CenterG);
	std::swap(m_v3MinL, other.m_v3MinL);
	std::swap(m_v3MaxL, other.m_v3MaxL);

	std::swap(m_v3MinG, other.m_v3MinG);
	std::swap(m_v3MaxG, other.m_v3MaxG);

	std::swap(m_v3HalfWidth, other.m_v3HalfWidth);
	std::swap(m_v3ARBBSize, other.m_v3ARBBSize);

	std::swap(m_m4ToWorld, other.m_m4ToWorld);

	std::swap(m_nCollidingCount, other.m_nCollidingCount);
	std::swap(m_CollidingArray, other.m_CollidingArray);
}
void MyRigidBody::Release(void)
{
	m_pMeshMngr = nullptr;
	ClearCollidingList();
}
//Accessors
bool MyRigidBody::GetVisibleBS(void) { return m_bVisibleBS; }
bool Simplex::MyRigidBody::GetDynamicSphere(void) { return dynamicSphere; }
vector3 Simplex::MyRigidBody::GetDynamicOffset(void) { return dynamicOffset; }
vector3 Simplex::MyRigidBody::GetDynamicVelocity(void) { return dynamicVelocity; }
std::vector<vector3> Simplex::MyRigidBody::GetPointListGlobal(void) { return pointListGlobal; }
void MyRigidBody::SetVisibleBS(bool a_bVisible) { m_bVisibleBS = a_bVisible; }
bool MyRigidBody::GetVisibleOBB(void) { return m_bVisibleOBB; }
void MyRigidBody::SetVisibleOBB(bool a_bVisible) { m_bVisibleOBB = a_bVisible; }
bool MyRigidBody::GetVisibleARBB(void) { return m_bVisibleARBB; }
void MyRigidBody::SetVisibleARBB(bool a_bVisible) { m_bVisibleARBB = a_bVisible; }
float MyRigidBody::GetRadius(void) { return m_fRadius; }
vector3 MyRigidBody::GetColorColliding(void) { return m_v3ColorColliding; }
vector3 MyRigidBody::GetColorNotColliding(void) { return m_v3ColorNotColliding; }
void MyRigidBody::SetColorColliding(vector3 a_v3Color) { m_v3ColorColliding = a_v3Color; }
void MyRigidBody::SetColorNotColliding(vector3 a_v3Color) { m_v3ColorNotColliding = a_v3Color; }
vector3 MyRigidBody::GetCenterLocal(void) { return m_v3CenterL; }
vector3 MyRigidBody::GetMinLocal(void) { return m_v3MinL; }
vector3 MyRigidBody::GetMaxLocal(void) { return m_v3MaxL; }
vector3 MyRigidBody::GetCenterGlobal(void){ return m_v3CenterG; }
vector3 MyRigidBody::GetMinGlobal(void) { return m_v3MinG; }
vector3 MyRigidBody::GetMaxGlobal(void) { return m_v3MaxG; }
vector3 MyRigidBody::GetHalfWidth(void) { return m_v3HalfWidth; }
matrix4 MyRigidBody::GetModelMatrix(void) { return m_m4ToWorld; }
bool MyRigidBody::CollidingArrayIsEmpty(void) { return m_CollidingArray == nullptr; }
void Simplex::MyRigidBody::SetSpeedMultiplier(float speedMultiplierIn)
{
	speedMultiplier = speedMultiplierIn;
}
void MyRigidBody::SetModelMatrix(matrix4 a_m4ModelMatrix)
{
	//to save some calculations if the model matrix is the same there is nothing to do here
	if (a_m4ModelMatrix == m_m4ToWorld)
		return;

	//Assign the model matrix
	m_m4ToWorld = a_m4ModelMatrix;
	if(fixedMatrix == IDENTITY_M4)
	{
		fixedMatrix = a_m4ModelMatrix;
	}
	

	m_v3CenterG = vector3(m_m4ToWorld * vector4(m_v3CenterL, 1.0f));

	//Calculate the 8 corners of the cube
	vector3 v3Corner[8];
	//Back square
	v3Corner[0] = m_v3MinL;
	v3Corner[1] = vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z);
	v3Corner[2] = vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z);
	v3Corner[3] = vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z);

	//Front square
	v3Corner[4] = vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z);
	v3Corner[5] = vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z);
	v3Corner[6] = vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z);
	v3Corner[7] = m_v3MaxL;

	//Place them in world space
	for (uint uIndex = 0; uIndex < 8; ++uIndex)
	{
		v3Corner[uIndex] = vector3(m_m4ToWorld * vector4(v3Corner[uIndex], 1.0f));
	}

	//Identify the max and min as the first corner
	m_v3MaxG = m_v3MinG = v3Corner[0];

	//get the new max and min for the global box
	for (uint i = 1; i < 8; ++i)
	{
		if (m_v3MaxG.x < v3Corner[i].x) m_v3MaxG.x = v3Corner[i].x;
		else if (m_v3MinG.x > v3Corner[i].x) m_v3MinG.x = v3Corner[i].x;

		if (m_v3MaxG.y < v3Corner[i].y) m_v3MaxG.y = v3Corner[i].y;
		else if (m_v3MinG.y > v3Corner[i].y) m_v3MinG.y = v3Corner[i].y;

		if (m_v3MaxG.z < v3Corner[i].z) m_v3MaxG.z = v3Corner[i].z;
		else if (m_v3MinG.z > v3Corner[i].z) m_v3MinG.z = v3Corner[i].z;
	}

	//we calculate the distance between min and max vectors
	m_v3ARBBSize = m_v3MaxG - m_v3MinG;
}

void MyRigidBody::UpdatePointList(void)
{
	//Edit pointlist
	if (pointListLocal.size() > 1)
	{
		for (int i = 0; i < pointListLocal.size(); i++)
		{
			pointListGlobal[i] = vector3(m_m4ToWorld * vector4(pointListLocal[i], 1.0f));
		}
	}
}

//The big 3
MyRigidBody::MyRigidBody(std::vector<vector3> a_pointList)
{
	Init();
	//Count the points of the incoming list
	uint uVertexCount = a_pointList.size();

	//If there are none just return, we have no information to create the BS from
	if (uVertexCount == 0)
		return;

	//Max and min as the first vector of the list
	m_v3MaxL = m_v3MinL = a_pointList[0];

	//Get the max and min out of the list
	for (uint i = 1; i < uVertexCount; ++i)
	{
		if (m_v3MaxL.x < a_pointList[i].x) m_v3MaxL.x = a_pointList[i].x;
		else if (m_v3MinL.x > a_pointList[i].x) m_v3MinL.x = a_pointList[i].x;

		if (m_v3MaxL.y < a_pointList[i].y) m_v3MaxL.y = a_pointList[i].y;
		else if (m_v3MinL.y > a_pointList[i].y) m_v3MinL.y = a_pointList[i].y;

		if (m_v3MaxL.z < a_pointList[i].z) m_v3MaxL.z = a_pointList[i].z;
		else if (m_v3MinL.z > a_pointList[i].z) m_v3MinL.z = a_pointList[i].z;
	}

	//Push all 36 box points
	//Top 6
	pointListLocal.push_back(m_v3MaxL); //L
	pointListGlobal.push_back(m_v3MaxL);

	pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));//SLL
	pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));

	pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));//SLS
	pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));

	pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));//SLS
	pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));

	pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));//LLS
	pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));

	pointListLocal.push_back(m_v3MaxL);//L
	pointListGlobal.push_back(m_v3MaxL);

	//Bottom 6
	pointListLocal.push_back(m_v3MinL);//S
	pointListGlobal.push_back(m_v3MinL);

	pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));//SSL
	pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));

	pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));//LSL
	pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));

	pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));//LSL
	pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));

	pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));//LSS
	pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));

	pointListLocal.push_back(m_v3MinL);//S
	pointListGlobal.push_back(m_v3MinL);

	//Left 6
	pointListLocal.push_back(m_v3MaxL);//L
	pointListGlobal.push_back(m_v3MaxL);

	pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));//LLS
	pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));

	pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));//LSS
	pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));

	pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));//LSS
	pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));

	pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));//LSL
	pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));

	pointListLocal.push_back(m_v3MaxL);//L
	pointListGlobal.push_back(m_v3MaxL);

	//Right 6
	pointListLocal.push_back(m_v3MinL);//S
	pointListGlobal.push_back(m_v3MinL);

	pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));//SSL
	pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));

	pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));//SLL
	pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));

	pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));//SLL
	pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));

	pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));//SLS
	pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));

	pointListLocal.push_back(m_v3MinL);//S
	pointListGlobal.push_back(m_v3MinL);

	//Front 6
	pointListLocal.push_back(m_v3MinL);//S
	pointListGlobal.push_back(m_v3MinL);

	pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));//SLS
	pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));

	pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));//LLS
	pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));

	pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));//LLS
	pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));

	pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));//LSS
	pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));

	pointListLocal.push_back(m_v3MinL);//S
	pointListGlobal.push_back(m_v3MinL);

	//Back 6
	pointListLocal.push_back(m_v3MaxL);//L
	pointListGlobal.push_back(m_v3MaxL);

	pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));//LSL
	pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));

	pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));//SSL
	pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));

	pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));//SSL
	pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));

	pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));//SLL
	pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));

	pointListLocal.push_back(m_v3MaxL);//L
	pointListGlobal.push_back(m_v3MaxL);

	//with model matrix being the identity, local and global are the same
	m_v3MinG = m_v3MinL;
	m_v3MaxG = m_v3MaxL;

	//with the max and the min we calculate the center
	m_v3CenterL = (m_v3MaxL + m_v3MinL) / 2.0f;

	//we calculate the distance between min and max vectors
	m_v3HalfWidth = (m_v3MaxL - m_v3MinL) / 2.0f;

	//Get the distance between the center and either the min or the max
	m_fRadius = glm::distance(m_v3CenterL, m_v3MinL) / 2.0f;
}
MyRigidBody::MyRigidBody(std::vector<vector3> a_pointList, bool dynamic)
{
	Init();

	dynamicSphere = dynamic;

	//Count the points of the incoming list
	uint uVertexCount = a_pointList.size();

	//If there are none just return, we have no information to create the BS from
	if (uVertexCount == 0)
		return;

	//Max and min as the first vector of the list
	m_v3MaxL = m_v3MinL = a_pointList[0];

	//Get the max and min out of the list
	for (uint i = 1; i < uVertexCount; ++i)
	{
		if (m_v3MaxL.x < a_pointList[i].x) m_v3MaxL.x = a_pointList[i].x;
		else if (m_v3MinL.x > a_pointList[i].x) m_v3MinL.x = a_pointList[i].x;

		if (m_v3MaxL.y < a_pointList[i].y) m_v3MaxL.y = a_pointList[i].y;
		else if (m_v3MinL.y > a_pointList[i].y) m_v3MinL.y = a_pointList[i].y;

		if (m_v3MaxL.z < a_pointList[i].z) m_v3MaxL.z = a_pointList[i].z;
		else if (m_v3MinL.z > a_pointList[i].z) m_v3MinL.z = a_pointList[i].z;
	}

	//with model matrix being the identity, local and global are the same
	m_v3MinG = m_v3MinL;
	m_v3MaxG = m_v3MaxL;

	//with the max and the min we calculate the center
	m_v3CenterL = (m_v3MaxL + m_v3MinL) / 2.0f;

	//we calculate the distance between min and max vectors
	m_v3HalfWidth = (m_v3MaxL - m_v3MinL) / 2.0f;

	//Get the distance between the center and either the min or the max
	m_fRadius = glm::distance(m_v3CenterL, m_v3MinL) / 1.8f;

	if (dynamic)
	{
		//Create points for the dynamic sphere
		float a_nSubdivisions = 8.0f; //8 = 360 vertices
		float rotNum = (2 * PI) / a_nSubdivisions;

		vector3 pointA(0.0f);
		vector3 pointB(0.0f);
		vector3 pointC(0.0f);

		//Triangle 1 Loop
		//Loop Top to Bottom
		for (int c = 0; c < a_nSubdivisions; c++)
		{
			//Set Points Y value
			pointA.y = cosf(c * (rotNum / 2.0f)) * m_fRadius;
			pointB.y = cosf((c + 1.0f) * (rotNum / 2.0f)) * m_fRadius;
			pointC.y = cosf((c + 1.0f) * (rotNum / 2.0f)) * m_fRadius;

			//Loop Left to Right
			for (int b = 0; b < a_nSubdivisions; b++)
			{
				//Set Points X value
				pointA.x = (cosf(b * rotNum) * m_fRadius) * (sinf(c * (rotNum / 2.0f)));
				pointB.x = (cosf(b * rotNum) * m_fRadius) * (sinf((c + 1.0f) * (rotNum / 2.0f)));
				pointC.x = (cosf((b + 1.0f) * rotNum) * m_fRadius) * (sinf((c + 1.0f) * (rotNum / 2.0f)));

				//Set Points Z value
				pointA.z = (sinf(b * rotNum) * m_fRadius) * (sinf(c * (rotNum / 2.0f)));
				pointB.z = (sinf(b * rotNum) * m_fRadius) * (sinf((c + 1.0f) * (rotNum / 2.0f)));
				pointC.z = (sinf((b + 1.0f) * rotNum) * m_fRadius) * (sinf((c + 1.0f) * (rotNum / 2.0f)));

				if (c != a_nSubdivisions)
				{
					pointListLocal.push_back(pointA);
					pointListGlobal.push_back(pointA);
					pointListLocal.push_back(pointB);
					pointListGlobal.push_back(pointB);
					pointListLocal.push_back(pointC);
					pointListGlobal.push_back(pointC);
				}	
			}
		}

		//Triangle 2 Loop
		//Loop Top to Bottom
		for (int c = 0; c < a_nSubdivisions; c++)
		{
			//Set Points Y value
			pointA.y = cosf(c * (rotNum / 2.0f)) * m_fRadius;
			pointB.y = cosf(c * (rotNum / 2.0f)) * m_fRadius;
			pointC.y = cosf((c + 1.0f) * (rotNum / 2.0f)) * m_fRadius;

			//Loop Left to Right
			for (int b = 0; b < a_nSubdivisions; b++)
			{
				//Set Points X value
				pointA.x = (cosf(b * rotNum) * m_fRadius) * (sinf(c * (rotNum / 2.0f)));
				pointB.x = (cosf((b + 1.0f) * rotNum) * m_fRadius) * (sinf(c * (rotNum / 2.0f)));
				pointC.x = (cosf((b + 1.0f) * rotNum) * m_fRadius) * (sinf((c + 1.0f) * (rotNum / 2.0f)));

				//Set Points Z value
				pointA.z = (sinf(b * rotNum) * m_fRadius) * (sinf(c * (rotNum / 2.0f)));
				pointB.z = (sinf((b + 1.0f) * rotNum) * m_fRadius) * (sinf(c * (rotNum / 2.0f)));
				pointC.z = (sinf((b + 1.0f) * rotNum) * m_fRadius) * (sinf((c + 1.0f) * (rotNum / 2.0f)));

				if (c != 0)
				{
					pointListLocal.push_back(pointA);
					pointListGlobal.push_back(pointA);
					pointListLocal.push_back(pointB);
					pointListGlobal.push_back(pointB);
					pointListLocal.push_back(pointC);
					pointListGlobal.push_back(pointC);
				}
			}
		}
	}
	else
	{
		//Push all 36 box points
		//Top 6
		pointListLocal.push_back(m_v3MaxL); //L
		pointListGlobal.push_back(m_v3MaxL);

		pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));//SLL
		pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));

		pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));//SLS
		pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));

		pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));//SLS
		pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));

		pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));//LLS
		pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));

		pointListLocal.push_back(m_v3MaxL);//L
		pointListGlobal.push_back(m_v3MaxL);

		//Bottom 6
		pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));//LSL
		pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));

		pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));//SSL
		pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));

		pointListLocal.push_back(m_v3MinL);//S
		pointListGlobal.push_back(m_v3MinL);

		pointListLocal.push_back(m_v3MinL);//S
		pointListGlobal.push_back(m_v3MinL);

		pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));//LSS
		pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));

		pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));//LSL
		pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));

		//Left 6
		pointListLocal.push_back(m_v3MaxL);//L
		pointListGlobal.push_back(m_v3MaxL);

		pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));//LLS
		pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));

		pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));//LSS
		pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));

		pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));//LSS
		pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));

		pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));//LSL
		pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));

		pointListLocal.push_back(m_v3MaxL);//L
		pointListGlobal.push_back(m_v3MaxL);

		//Right 6
		pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));//SLL
		pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));

		pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));//SSL
		pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));

		pointListLocal.push_back(m_v3MinL);//S
		pointListGlobal.push_back(m_v3MinL);

		pointListLocal.push_back(m_v3MinL);//S
		pointListGlobal.push_back(m_v3MinL);

		pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));//SLS
		pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));

		pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));//SLL
		pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));

		//Front 6
		pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));//LLS
		pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));

		pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));//SLS
		pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z));

		pointListLocal.push_back(m_v3MinL);//S
		pointListGlobal.push_back(m_v3MinL);

		pointListLocal.push_back(m_v3MinL);//S
		pointListGlobal.push_back(m_v3MinL);

		pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));//LSS
		pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z));

		pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));//LLS
		pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));

		//Back 6
		pointListLocal.push_back(m_v3MaxL);//L
		pointListGlobal.push_back(m_v3MaxL);

		pointListLocal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));//LSL
		pointListGlobal.push_back(vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));

		pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));//SSL
		pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));

		pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));//SSL
		pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z));

		pointListLocal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));//SLL
		pointListGlobal.push_back(vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));

		pointListLocal.push_back(m_v3MaxL);//L
		pointListGlobal.push_back(m_v3MaxL);
	}
}
MyRigidBody::MyRigidBody(MyRigidBody const& other)
{
	m_pMeshMngr = other.m_pMeshMngr;

	dynamicSphere = other.dynamicSphere;

	m_bVisibleBS = other.m_bVisibleBS;
	m_bVisibleOBB = other.m_bVisibleOBB;
	m_bVisibleARBB = other.m_bVisibleARBB;

	m_fRadius = other.m_fRadius;

	m_v3ColorColliding = other.m_v3ColorColliding;
	m_v3ColorNotColliding = other.m_v3ColorNotColliding;

	m_v3CenterL = other.m_v3CenterL;
	m_v3CenterG = other.m_v3CenterG;
	m_v3MinL = other.m_v3MinL;
	m_v3MaxL = other.m_v3MaxL;

	m_v3MinG = other.m_v3MinG;
	m_v3MaxG = other.m_v3MaxG;

	m_v3HalfWidth = other.m_v3HalfWidth;
	m_v3ARBBSize = other.m_v3ARBBSize;

	m_m4ToWorld = other.m_m4ToWorld;

	m_nCollidingCount = other.m_nCollidingCount;
	m_CollidingArray = other.m_CollidingArray;
}
MyRigidBody& MyRigidBody::operator=(MyRigidBody const& other)
{
	if (this != &other)
	{
		Release();
		Init();
		MyRigidBody temp(other);
		Swap(temp);
	}
	return *this;
}
MyRigidBody::~MyRigidBody() { Release(); };
//--- other Methods
void MyRigidBody::AddCollisionWith(MyRigidBody* other)
{
	//if its already in the list return
	if (IsInCollidingArray(other))
		return;
	/*
		check if the object is already in the colliding set, if
		the object is already there return with no changes
	*/

	//insert the entry
	PRigidBody* pTemp;
	pTemp = new PRigidBody[m_nCollidingCount + 1];
	if (m_CollidingArray)
	{
		memcpy(pTemp, m_CollidingArray, sizeof(MyRigidBody*) * m_nCollidingCount);
		delete[] m_CollidingArray;
		m_CollidingArray = nullptr;
	}
	pTemp[m_nCollidingCount] = other;
	m_CollidingArray = pTemp;

	++m_nCollidingCount;
}
void MyRigidBody::RemoveCollisionWith(MyRigidBody* other)
{
	//if there are no dimensions return
	if (m_nCollidingCount == 0)
		return;

	//we look one by one if its the one wanted
	for (uint i = 0; i < m_nCollidingCount; i++)
	{
		if (m_CollidingArray[i] == other)
		{
			//if it is, then we swap it with the last one and then we pop
			std::swap(m_CollidingArray[i], m_CollidingArray[m_nCollidingCount - 1]);
			PRigidBody* pTemp;
			pTemp = new PRigidBody[m_nCollidingCount - 1];
			if (m_CollidingArray)
			{
				memcpy(pTemp, m_CollidingArray, sizeof(uint) * (m_nCollidingCount - 1));
				delete[] m_CollidingArray;
				m_CollidingArray = nullptr;
			}
			m_CollidingArray = pTemp;

			--m_nCollidingCount;
			return;
		}
	}
}
void MyRigidBody::ClearCollidingList(void)
{
	m_nCollidingCount = 0;
	if (m_CollidingArray)
	{
		delete[] m_CollidingArray;
		m_CollidingArray = nullptr;
	}
}
uint MyRigidBody::SAT(MyRigidBody* const a_pOther)
{
	float ra, rb;
	glm::mat3 R, AbsR;

	for (uint i = 0; i < 3; i++)
	{
		for (uint j = 0; j < 3; j++)
		{
			R[i][j] = glm::dot(GetModelMatrix()[i], a_pOther->GetModelMatrix()[j]);
		}
	}

	//compute translation vector t
	vector3 t = vector3(a_pOther->GetCenterGlobal() - GetCenterGlobal());
	//bring it into a's coord frame
	t = vector3(glm::dot(t, vector3(GetModelMatrix()[0])),
		glm::dot(t, vector3(GetModelMatrix()[1])),			//this is were the book error was, fixed here
		glm::dot(t, vector3(GetModelMatrix()[2])));

	for (uint i = 0; i < 3; i++)
		for (uint j = 0; j < 3; j++)
			AbsR[i][j] = abs(R[i][j]) + FLT_EPSILON;

	// Test axes L = A0, L = A1, L = A2
	for (uint i = 0; i < 3; i++) {
		ra = GetHalfWidth()[i];
		rb = a_pOther->GetHalfWidth()[0] * AbsR[i][0] + a_pOther->GetHalfWidth()[1] * AbsR[i][1] + a_pOther->GetHalfWidth()[2] * AbsR[i][2];

		if (abs(t[i]) > ra + rb)
		{
			return 1;
		}
	}

	// Test axes L = B0, L = B1, L = B2
	for (uint i = 0; i < 3; i++) {
		ra = GetHalfWidth()[0] * AbsR[0][i] + GetHalfWidth()[1] * AbsR[1][i] + GetHalfWidth()[2] * AbsR[2][i];
		rb = a_pOther->GetHalfWidth()[i];

		if (abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] * R[2][i]) > ra + rb)
		{
			return 1;
		}
	}

	// Test axis L = A0 x B0
	ra = GetHalfWidth()[1] * AbsR[2][0] + GetHalfWidth()[2] * AbsR[1][0];
	rb = a_pOther->GetHalfWidth()[1] * AbsR[0][2] + a_pOther->GetHalfWidth()[2] * AbsR[0][1];
	if (abs(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb) return  1;
	// Test axis L = A0 x B1
	ra = GetHalfWidth()[1] * AbsR[2][1] + GetHalfWidth()[2] * AbsR[1][1];
	rb = a_pOther->GetHalfWidth()[0] * AbsR[0][2] + a_pOther->GetHalfWidth()[2] * AbsR[0][0];
	if (abs(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb) return  1;
	// Test axis L = A0 x B2
	ra = GetHalfWidth()[1] * AbsR[2][2] + GetHalfWidth()[2] * AbsR[1][2];
	rb = a_pOther->GetHalfWidth()[0] * AbsR[0][1] + a_pOther->GetHalfWidth()[1] * AbsR[0][0];
	if (abs(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb) return  1;
	// Test axis L = A1 x B0
	ra = GetHalfWidth()[0] * AbsR[2][0] + GetHalfWidth()[2] * AbsR[0][0];
	rb = a_pOther->GetHalfWidth()[1] * AbsR[1][2] + a_pOther->GetHalfWidth()[2] * AbsR[1][1];
	if (abs(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb) return  1;
	// Test axis L = A1 x B1
	ra = GetHalfWidth()[0] * AbsR[2][1] + GetHalfWidth()[2] * AbsR[0][1];
	rb = a_pOther->GetHalfWidth()[0] * AbsR[1][2] + a_pOther->GetHalfWidth()[2] * AbsR[1][0];
	if (abs(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb) return  1;
	// Test axis L = A1 x B2
	ra = GetHalfWidth()[0] * AbsR[2][2] + GetHalfWidth()[2] * AbsR[0][2];
	rb = a_pOther->GetHalfWidth()[0] * AbsR[1][1] + a_pOther->GetHalfWidth()[1] * AbsR[1][0];
	if (abs(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb) return  1;
	// Test axis L = A2 x B0
	ra = GetHalfWidth()[0] * AbsR[1][0] + GetHalfWidth()[1] * AbsR[0][0];
	rb = a_pOther->GetHalfWidth()[1] * AbsR[2][2] + a_pOther->GetHalfWidth()[2] * AbsR[2][1];
	if (abs(t[1] * R[0][0] - t[0] * R[1][0]) > ra + rb) return 1;
	// Test axis L = A2 x B1
	ra = GetHalfWidth()[0] * AbsR[1][1] + GetHalfWidth()[1] * AbsR[0][1];
	rb = a_pOther->GetHalfWidth()[0] * AbsR[2][2] + a_pOther->GetHalfWidth()[2] * AbsR[2][0];
	if (abs(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb) return 1;
	// Test axis L = A2 x B2
	ra = GetHalfWidth()[0] * AbsR[1][2] + GetHalfWidth()[1] * AbsR[0][2];
	rb = a_pOther->GetHalfWidth()[0] * AbsR[2][1] + a_pOther->GetHalfWidth()[1] * AbsR[2][0];
	if (abs(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb) return  1;


	//there is no axis test that separates this two objects
	return 0;
}
bool MyRigidBody::IsColliding(MyRigidBody* const a_pOther)
{
	//we cannot collide with ourself
	if (this == a_pOther) {
		return false;
	}

	//check if spheres are colliding
	bool bColliding = true;
	bool collisionRequired = false;
	bool found = false;

	vector3 firstCollision = ZERO_V3;

	//bColliding = (glm::distance(GetCenterGlobal(), other->GetCenterGlobal()) < m_fRadius + other->m_fRadius);
	//if they are check the Axis Aligned Bounding Box

	if (bColliding) //they are colliding with bounding sphere
	{
		if (this->m_v3MaxG.x < a_pOther->m_v3MinG.x) //this to the right of other
			bColliding = false;
		if (this->m_v3MinG.x > a_pOther->m_v3MaxG.x) //this to the left of other
			bColliding = false;
		
		if (this->m_v3MaxG.y < a_pOther->m_v3MinG.y) //this below of other
			bColliding = false;
		if (this->m_v3MinG.y > a_pOther->m_v3MaxG.y) //this above of other
			bColliding = false;
		
		if (this->m_v3MaxG.z < a_pOther->m_v3MinG.z) //this behind of other
			bColliding = false;
		if (this->m_v3MinG.z > a_pOther->m_v3MaxG.z) //this in front of other
			bColliding = false;

		if (bColliding) //they are colliding with bounding sphere
		{
			std::vector<vector3> collisionPoints;
			std::vector<vector3> normalAggregate;
			vector3 planeNormal = ZERO_V3;

			//If this is a dynamic -> static collision
			if (dynamicSphere)
			{
				if (!a_pOther->GetDynamicSphere())
				{
					if (pointListGlobal.size() > 0)
					{
						//Update the points based on the object's model matrix
						UpdatePointList();
						a_pOther->UpdatePointList();

						vector3 planePointA = ZERO_V3;
						vector3 planePointB = ZERO_V3;
						vector3 planePointC = ZERO_V3;

						for (int p = 0; p < a_pOther->pointListGlobal.size() - 3; p += 3) //Run through each triangle in a mesh
						{
							planePointA = a_pOther->pointListGlobal[p]; //Store the three points of the triangle (plane)
							planePointB = a_pOther->pointListGlobal[p + 1];
							planePointC = a_pOther->pointListGlobal[p + 2];

							vector3 QA = vector3(planePointA.x - planePointB.x, planePointA.y - planePointB.y, planePointA.z - planePointB.z);
							vector3 QB = vector3(planePointC.x - planePointB.x, planePointC.y - planePointB.y, planePointC.z - planePointB.z);

							vector3 N = glm::cross(QA, QB); //Get the normal of the plane

							vector3 A = ZERO_V3;
							vector3 B = ZERO_V3;

							//Value along line of the intersection
							float t = 0.0f;

							vector3 possibleCollision = ZERO_V3;

							for (int i = 0; i < pointListGlobal.size() - 1; i += 1) //Run through each line segment in the object
							{
								A = pointListGlobal[i];
								B = pointListGlobal[i + 1];

								vector3 d = A + (B - A); //The equation of the line

								t = (N.x * planePointA.x + N.y * planePointA.y + N.z * planePointA.z) //The distance along the line that it intersects the plane
									/ ((d.x * N.x) + (d.y * N.y) + (d.z * N.z));

								possibleCollision = d * t; //The point that the plane and line collide

								if (glm::distance2(possibleCollision, m_v3CenterG) < m_fRadius) //If that point is close enough to be a collision
								{
									vector3 N1 = glm::cross(possibleCollision - planePointA, planePointB - possibleCollision);
									vector3 N2 = glm::cross(possibleCollision - planePointB, planePointC - possibleCollision);
									vector3 N3 = glm::cross(possibleCollision - planePointC, planePointA - possibleCollision);

									//Check if the line -> plane collision takes place within the bounds of the plane
									if (glm::dot(N1, N2) > 0.9998f)
									{
										if (glm::dot(N2, N3) > 0.9998f)
										{
											//There is a collision, add the normal of the plane
											collisionPoints.push_back(possibleCollision);
											collisionRequired = true;
											if (firstCollision == ZERO_V3) { firstCollision = N; }
											if (firstCollision.y > 0.7f && N.y > 0.7f)
											{
												normalAggregate.push_back(N);
											}
											else if (firstCollision.y <= 0.7f && N.y <= 0.7f)
											{
												normalAggregate.push_back(N);
											}
											//found = true;
											//break;
										}
									}
								}
								if (found) { break; }
							} //End of checking lines
							if (found) { break; }
						} //End of checking planes

						if (collisionPoints.size() > 0)
						{
							//Find the average collision point
							vector3 closestPoint = collisionPoints[0];
							vector3 furthestPoint = collisionPoints[0];

							for each (vector3 v in collisionPoints)
							{
								if (glm::distance(closestPoint, v) > glm::distance(closestPoint, furthestPoint))
								{
									furthestPoint = v;
								}
							}
							vector3 averagePoint = (furthestPoint + closestPoint) / 2.0f;

							vector3 averageNormal = ZERO_V3;

							//Find the average normal vector
							for each (vector3 v in normalAggregate)
							{
								averageNormal += glm::normalize(v);
							}
							
							averageNormal /= normalAggregate.size();

							//Choose which collision method should be used
							//planeNormal = firstCollision;
							planeNormal = averageNormal;

							//Set and normalize dynamic velocity
							dynamicVelocity = glm::normalize(planeNormal);
							if (glm::abs(dynamicVelocity.x) < 0.01f) { dynamicVelocity.x = 0.0f; }
							if (glm::abs(dynamicVelocity.z) < 0.01f) { dynamicVelocity.z = 0.0f; }

							//Find the distance this object should be offset so it is not overlapping the other object
							float mag = m_fRadius - glm::distance(averagePoint, m_v3CenterG);
							dynamicOffset = dynamicVelocity * mag * 1.2f;
						}
					}
					else { collisionRequired = false; } //Reaching one of these means there is no collision
				}

				//seperate code for sphere -> sphere collision
				else
				{
					vector3 apartVector = GetCenterGlobal() - a_pOther->GetCenterGlobal();

					//checks to see if centers of spheres are closer than their radius combined
					//if so, they are colliding
					if (glm::length(apartVector) < m_fRadius + a_pOther->m_fRadius)
					{
						collisionRequired = true;

						//apply force in opposite direction of the vector between their centers
						dynamicVelocity = glm::normalize(apartVector);
						dynamicOffset = ZERO_V3;

						//call rebound on other sphere so both move away from each other
					}
					else {
						collisionRequired = false;
					}
				}
			}
			else { collisionRequired = false; }
		}
		else { collisionRequired = false; }
		bColliding = collisionRequired;
		if (bColliding) //they are colliding with bounding box also
		{
			this->AddCollisionWith(a_pOther);
			a_pOther->AddCollisionWith(this);
		}
		else //they are not colliding with bounding box
		{
			this->RemoveCollisionWith(a_pOther);
			a_pOther->RemoveCollisionWith(this);
		}
	}
	else //they are not colliding with bounding sphere
	{
		this->RemoveCollisionWith(a_pOther);
		a_pOther->RemoveCollisionWith(this);
	}
	return bColliding;
}

void MyRigidBody::AddToRenderList(void)
{
	if (m_bVisibleBS)
	{
		if (m_nCollidingCount > 0)
			m_pMeshMngr->AddWireSphereToRenderList(glm::translate(m_m4ToWorld, m_v3CenterL) * glm::scale(vector3(m_fRadius)), C_BLUE_CORNFLOWER);
		else
			m_pMeshMngr->AddWireSphereToRenderList(glm::translate(m_m4ToWorld, m_v3CenterL) * glm::scale(vector3(m_fRadius)), C_BLUE_CORNFLOWER);
	}
	if (m_bVisibleOBB)
	{
		if (m_nCollidingCount > 0)
			m_pMeshMngr->AddWireCubeToRenderList(glm::translate(m_m4ToWorld, m_v3CenterL) * glm::scale(m_v3HalfWidth * 2.0f), m_v3ColorColliding);
		else
			m_pMeshMngr->AddWireCubeToRenderList(glm::translate(m_m4ToWorld, m_v3CenterL) * glm::scale(m_v3HalfWidth * 2.0f), m_v3ColorNotColliding);
	}
	if (m_bVisibleARBB)
	{
		if (m_nCollidingCount > 0)
			m_pMeshMngr->AddWireCubeToRenderList(glm::translate(m_v3CenterG) * glm::scale(m_v3ARBBSize), C_YELLOW);
		else
			m_pMeshMngr->AddWireCubeToRenderList(glm::translate(m_v3CenterG) * glm::scale(m_v3ARBBSize), C_YELLOW);
	}
}
bool MyRigidBody::IsInCollidingArray(MyRigidBody* a_pEntry)
{
	//see if the entry is in the set
	for (uint i = 0; i < m_nCollidingCount; i++)
	{
		if (m_CollidingArray[i] == a_pEntry)
			return true;
	}
	return false;
}

Simplex::MyRigidBody* Simplex::MyRigidBody::GetCollidingBody(int a_iIndex) 
{
	return m_CollidingArray[a_iIndex];
}
int Simplex::MyRigidBody::GetNumberColliding(void) 
{
	return m_nCollidingCount;
}
void Simplex::MyRigidBody::AddForces(vector3 a_v3Forces)
{
	m_v3Acceleration += a_v3Forces;
}
void Simplex::MyRigidBody::ApplyForce(void)
{
	if (ImGui::GetIO().Framerate > 0.0f)
	{
		m_v3Velocity += (m_v3Acceleration + vector3(0.0f, -0.00961f, 0.0f)) * (1000.0f / ImGui::GetIO().Framerate);
	}
}
void Simplex::MyRigidBody::ApplyVelocity(void)
{
	//Translate
	fixedMatrix[3][0] = m_m4ToWorld[3][0];
	fixedMatrix[3][1] = m_m4ToWorld[3][1];
	fixedMatrix[3][2] = m_m4ToWorld[3][2];
	
	SetModelMatrix(glm::translate(fixedMatrix, (m_v3Velocity * speedMultiplier) / (1000.0f / ImGui::GetIO().Framerate)));
	SetModelMatrix(glm::translate(m_m4ToWorld, GetDynamicOffset() * 0.2f));

	//Rotate
	matrix4 m4LA = IDENTITY_M4;
	if ((glm::abs(m_v3Velocity.x) + glm::abs(m_v3Velocity.y)+ glm::abs(m_v3Velocity.z)) > 0.001f)
	{
		m4LA = glm::lookAt(m_v3CenterG, (m_v3Velocity * 5.0f) + m_v3CenterG, m_v3CenterG + vector3(1.0f, 0.0f, 0.0f));
	}

	//Debugging
	//m_pMeshMngr->AddLineToRenderList(IDENTITY_M4, m_v3CenterG, (m_v3Velocity * 5.0f) + m_v3CenterG, C_BLUE, C_RED);

	m4LA[3][0] = 0.0f;
	m4LA[3][1] = 0.0f;
	m4LA[3][2] = 0.0f;

	SetModelMatrix(m_m4ToWorld * m4LA);

	m_v3Acceleration = ZERO_V3;
}

void Simplex::MyRigidBody::Rebound(MyRigidBody* a_pOther, float strength, String otherName)
{
	vector3 force = ZERO_V3;

	force = 0.00961f * GetDynamicVelocity();

	vector3 cancelForce = glm::abs(glm::abs(GetDynamicVelocity()) - vector3(1,1,1));
	
	if (otherName == "Fulltiltadillo\\RampTileDown.fbx")
	{
		m_v3Velocity.x *= 1.046f;
		m_v3Velocity.y *= cancelForce.y;
		m_v3Velocity.z *= 1.046f;

		force.y *= 4.11f;
		force.z = 0.0f;
		force.x = 0.0f;
	}
	else if (otherName == "Fulltiltadillo\\cactus.fbx")
	{
		m_v3Velocity.x *= 0.028f;
		m_v3Velocity.y *= cancelForce.y;
		m_v3Velocity.z *= 0.028f;

		force.x *= 0.910f;
		force.z *= 0.910f;
	}
	else
	{
		m_v3Velocity *= cancelForce;
	}

	if(glm::abs(glm::length(m_v3Velocity)) > 0.00001f)
	{
		//friction/drag
		m_v3Velocity *= 0.96f;
	}

	force.y *= 2.55f;

	force.x *= 5.0f;
	force.z *= 5.0f;

	AddForces(force * strength);
}
void Simplex::MyRigidBody::ClampVelocity(void)
{
	// clamp speeds
	m_v3Velocity.x = glm::clamp(m_v3Velocity.x, -100.0f, 100.0f);
	m_v3Velocity.y = glm::clamp(m_v3Velocity.y, -100.0f, 100.0f);
	m_v3Velocity.z = glm::clamp(m_v3Velocity.z, -100.0f, 100.0f);
}
void Simplex::MyRigidBody::SetGravity(bool a_bGravity) { m_bGravity = a_bGravity; }
void Simplex::MyRigidBody::SetMass(float a_fMass) { m_fMass = a_fMass; }
bool Simplex::MyRigidBody::GetGravity(void) { return m_bGravity; }
float Simplex::MyRigidBody::GetMass(void) { return m_fMass; }
vector3 Simplex::MyRigidBody::GetVelocity(void) { return m_v3Velocity; }

vector3 Simplex::MyRigidBody::GetAcceleration(void) { return m_v3Acceleration; }

bool Simplex::MyRigidBody::SphereBoxCollision(MyRigidBody* const a_pOther)
{
	// make variable to return
	bool bColliding = true;

	if (bColliding)
	{
		// get the closest point to the sphere on the cube
		float x = std::max(a_pOther->m_v3MinG.x, std::min(m_v3CenterG.x, a_pOther->m_v3MaxG.x)); // x coordinate
		float y = std::max(a_pOther->m_v3MinG.y, std::min(m_v3CenterG.y, a_pOther->m_v3MaxG.y)); // y coordinate
		float z = std::max(a_pOther->m_v3MinG.z, std::min(m_v3CenterG.z, a_pOther->m_v3MaxG.z)); // z coordinate

		// get the distance from said point to the center of the sphere
		float distance = std::sqrt((x - m_v3CenterG.x) * (x - m_v3CenterG.x) + (y - m_v3CenterG.y) * (y - m_v3CenterG.y) + (z - m_v3CenterG.z) * (z - m_v3CenterG.z));

		// if said distance is less than the radius, then it's colliding
		bColliding = distance < m_fRadius;

		if (bColliding) //they are colliding
		{
			this->AddCollisionWith(a_pOther);
			a_pOther->AddCollisionWith(this);
		}
		else //they are not colliding
		{
			this->RemoveCollisionWith(a_pOther);
			a_pOther->RemoveCollisionWith(this);
		}
	}
	return bColliding;
}

matrix4 Simplex::MyRigidBody::GetStartingWorld()
{
	return startingWorld;
}
void Simplex::MyRigidBody::SetStartingWorld(matrix4 startMatrix)
{
	startingWorld = startMatrix;
}
void Simplex::MyRigidBody::RotateAroundPoint(vector3 point, vector2 tiltValues)
{
	//Center -> object
	float xDif = point.x - startingCenter.x;
	float yDif = point.y - startingCenter.y;
	float zDif = point.z - startingCenter.z;

	//Translation Matrices
	matrix4 transA = IDENTITY_M4;
	matrix4 transB = IDENTITY_M4;


	//Rotation around x axis (z/y vals change)
	transA[1][1] = cosf((tiltValues[1]) / (2 * PI));
	transA[2][1] = sinf((tiltValues[1]) / (2 * PI));
	transA[1][2] = -sinf((tiltValues[1]) / (2 * PI));
	transA[2][2] = transA[1][1];

	transA[3][2] = -zDif * cosf((tiltValues[1]) / (2 * PI)) + yDif * sinf((tiltValues[1]) / (2 * PI)) + zDif;
	transA[3][1] = -zDif * sinf((tiltValues[1]) / (2 * PI)) - yDif * cosf((tiltValues[1]) / (2 * PI)) + yDif;

	//Rotation around z axis (x/y vals change)
	transB[0][0] = cosf((tiltValues[0]) / (2 * PI));
	transB[1][0] = -sinf((tiltValues[0]) / (2 * PI));
	transB[0][1] = sinf((tiltValues[0]) / (2 * PI));
	transB[1][1] = transB[0][0];

	transB[3][0] = -xDif * cosf((tiltValues[0]) / (2 * PI)) + yDif * sinf((tiltValues[0]) / (2 * PI)) + xDif;
	transB[3][1] = -xDif * sinf((tiltValues[0]) / (2 * PI)) - yDif * cosf((tiltValues[0]) / (2 * PI)) + yDif;


	matrix4 startingMatrixMain = IDENTITY_M4;
	startingMatrixMain[3] = GetStartingWorld()[3];
	startingMatrixMain = startingMatrixMain * transA * transB;

	matrix4 startingMatrixOriginalRotation = GetStartingWorld();
	startingMatrixOriginalRotation[3] = vector4(0, 0, 0, 1);

	startingMatrixMain *= startingMatrixOriginalRotation;

	//Set Model Matrix but keep the original matrix
	SetModelMatrix(startingMatrixMain);
}

void Simplex::MyRigidBody::RotateDynamicAroundPoint(vector2 tiltValues)
{
	//Center -> object
	float xDif = startingCenter.x;
	float yDif = startingCenter.y;
	float zDif = startingCenter.z;

	//Translation Matrices
	matrix4 transA = IDENTITY_M4;
	matrix4 transB = IDENTITY_M4;

	//Rotation around x axis (z/y vals change)
	transA[1][1] = cosf((tiltValues[1]) / (2 * PI));
	transA[2][1] = sinf((tiltValues[1]) / (2 * PI));
	transA[1][2] = -sinf((tiltValues[1]) / (2 * PI));
	transA[2][2] = transA[1][1];

	//Rotation around z axis (x/y vals change)
	transB[0][0] = cosf((tiltValues[0]) / (2 * PI));
	transB[1][0] = -sinf((tiltValues[0]) / (2 * PI));
	transB[0][1] = sinf((tiltValues[0]) / (2 * PI));
	transB[1][1] = transB[0][0];

	matrix4 modl = GetModelMatrix();
    modl[3][0] = 0.0f;
	modl[3][1] = 0.0f;
	modl[3][2] = 0.0f;

	//Set Model Matrix but keep the original matrix
	//rotationMatrix = modl * transA * transB;
}

vector3 Simplex::MyRigidBody::GetStartingCenter()
{
	return startingCenter;
}
void Simplex::MyRigidBody::SetStartingCenter(vector3 startCenter)
{
	startingCenter = startCenter;
}


