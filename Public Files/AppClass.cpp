#include "AppClass.h"
#include "Simplex\Simplex.h"

using namespace Simplex;
void Application::InitVariables(void)
{
	//Set the position and target of the camera
	m_pCameraMngr->SetPositionTargetAndUpward(
		vector3(0.0f, 0.0f, 100.0f), //Position
		vector3(0.0f, 0.0f, 99.0f),	//Target
		AXIS_Y);					//Up
	timer = 60.00f; //one minute on the clock
	m_pLightMngr->SetPosition(vector3(0.0f, 3.0f, 13.0f), 1); //set the position of first light (0 is reserved for ambient light)

#ifdef DEBUG
	uint uInstances = 50;
#else
	uint uInstances = 50;
#endif
	//this is done to fix a bug.
	//if we try to load the ramp texture, it will throw an error and the texture will be missing.
	//But if we try to access the texture once before the stage starts, the texture will be 
	//fine when the stage starts. Idk. It some bug within the Simplex::Model class that 
	//we cannot access as the texture and model itself is fine from what we can tell.
	m_pEntityMngr->AddEntity("Fulltiltadillo\\RampTileDown.fbx", false, true, false, 3);
	m_pEntityMngr->Reset();


	//creates first stage and sets up octree + player
	level = 1;
	levelLoader.LoadLevel(Levels[level], m_pEntityMngr, m_pMeshMngr);
	playerEntity = m_pEntityMngr->GetEntity(0);
	uint uIndex = -1;
	m_uOctantLevels = 2;
	m_pEntityMngr->RecreateOctree(m_uOctantLevels);
	
	//debugs
	debug = true;
	bDebug = false;
	nDebug = false;
	mDebug = false;

}
void Application::Update(void)
{
	float now = clock.getElapsedTime().asSeconds();
	deltaTime = now - lastTime;
	lastTime = now;

	//Update the system so it knows how much time has passed since the last call
	m_pSystem->Update();

	//Is the ArcBall active?
	//ArcBall();

	//Is the first person camera active?
	//CameraRotation();

	//calculates how much tilt the stage needs
	CalculateTilt();

	//Update Entity Manager
	m_pEntityMngr->Update(m_uOctantLevels, playerEntity->GetRigidBody()->GetCenterGlobal(), tiltValues);

	//WIN-LOSE CONDITIONS HANDLED~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//LOSE STATUS
	//very lame and basic count down
	timer = timer - .016667;
	if (timer <= 0) { Reset(); timer = 60.0f; } //game over, reset

	//for when player goes off the platform, checked in EnityManager
	else if (m_pEntityMngr->GetDeathStatus()) { Reset(); } //resets level

	//VICTORY STATUS
	//increases level count and resets screen to next level
	else if (m_pEntityMngr->GetVictoryStatus()) 
	{ 
		if (level != 5) 
		{ 
			level++; 
		} 
		Reset(); 
		timer = 60.00f; 
		if (level == 5)
			victoryUI = true;
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//Add objects to render list
	m_pEntityMngr->AddEntityToRenderList(-1, true);
	m_pEntityMngr->AddOctreeToRenderList(m_uOctantOutline);

	//Is the camera following the player?
	//CameraFollow(playerEntity);

	//Looks down from sky but follows player. Does not rotate
	CameraBirdsEye(playerEntity);
}
void Application::Display(void)
{
	// Clear the screen
	ClearScreen();

	// draw a skybox
	m_pMeshMngr->AddSkyboxToRenderList();
	
	//render list call
	m_uRenderCallCount = m_pMeshMngr->Render();

	//clear the render list
	m_pMeshMngr->ClearRenderList();
	
	//draw gui,
	DrawLevelGUI();
	if (debug) {
		DrawGUI();
	}


	
	//end the current frame (internally swaps the front and back buffers)
	m_pWindow->display();
}
void Application::Release(void)
{
	//release GUI
	ShutdownGUI();
}

void Simplex::Application::Reset(void) //resets the entity manager and loads up the level
{
	valFB = 0;
	valLR = 0;
	m_pEntityMngr->Reset();
	levelLoader.LoadLevel(Levels[level], m_pEntityMngr, m_pMeshMngr);
	playerEntity = m_pEntityMngr->GetEntity(0);
	uint uIndex = -1;
	m_uOctantLevels = 1;
	m_pEntityMngr->RecreateOctree(m_uOctantLevels);
	if (bDebug)
		m_pEntityMngr->ToggleBoundingSphere();
	if (nDebug)
		m_pEntityMngr->ToggleOrientedBoundingBox();
	if (mDebug)
		m_pEntityMngr->ToggleAxisRealignedboundingBox();
	if (level != 5 && victoryUI)
		victoryUI = false;
}

//figures out how much stage rotation is needed and passes rotation into EntityManager
void Simplex::Application::CalculateTilt(void)
{
	float incrementConstant = deltaTime * 20.0f;

	//move on keypress
	if (arrowFlags & 1) {
		if (valFB <= 0) {
			valFB -= 0.1f * incrementConstant;
		}
		else {
			//speed up tilting if sage is tilted in opposite direction
			valFB -= 0.15f * incrementConstant;
		}
	}
	if (arrowFlags & 2) {
		if (valFB > 0) {
			valFB += 0.1f * incrementConstant;
		}
		else {
			//speed up tilting if sage is tilted in opposite direction
			valFB += 0.15f * incrementConstant;
		}
	}
	if (arrowFlags & 4) {
		if (valLR <= 0) {
			valLR -= 0.04f * incrementConstant;
		}
		else {
			//speed up tilting if sage is tilted in opposite direction
			valLR -= 0.13f * incrementConstant;
		}
	}
	if (arrowFlags & 8) {
		if (valLR > 0) {
			valLR += 0.04f * incrementConstant;
		}
		else {
			//speed up tilting if sage is tilted in opposite direction
			valLR += 0.13f * incrementConstant;
		}
	}

	//reverts back to normal axis if no rotation on that axis
	if (!(arrowFlags & 3)) {
		//incase value becomes almost zero to stop any micro movement from rounding error
		float changeInTilt = glm::sign(valFB) * 0.1f * incrementConstant;
		if (glm::sign(valFB - changeInTilt) == glm::sign(valFB)) {
			valFB -= changeInTilt;
		}
		else {
			valFB = 0.0f;
		}
	}
	if (!(arrowFlags & 12)) {
		//incase value becomes almost zero to stop any micro movement from rounding error
		float changeInTilt = glm::sign(valLR) * 0.1f * incrementConstant;
		if (glm::sign(valLR - changeInTilt) == glm::sign(valLR)) {
			valLR -= changeInTilt;
		}
		else {
			valLR = 0.0f;
		}
	}

	//clamp the maximum rotations
	float maxFBSpeed = 1.8f;
	float maxLRSpeed = 1.5f;

	if (glm::abs(valFB) > maxFBSpeed) {
		valFB = glm::sign(valFB) * maxFBSpeed;
	}
	if (glm::abs(valLR) > maxLRSpeed) {
		valLR = glm::sign(valLR) * maxLRSpeed;
	}

	//creates rotation vector
	tiltValues = vector2(valLR, valFB);

	//ignore prototype tilting code
	return;

	vector2 camForward = glm::normalize(vector2(m_pCameraMngr->GetForward().x, m_pCameraMngr->GetForward().z));
	vector2 worldForward = vector2(0, 1);


	tiltAngle = atan2(camForward.y, camForward.x) - atan2(worldForward.y, worldForward.x);
	//DBOUT(tiltAngle); //debugging angle

	//limit it to -Pi to Pi only
	if (tiltAngle > PI) { tiltAngle -= 2 * PI; }
	else if (tiltAngle <= -PI) { tiltAngle += 2 * PI; }
	

	tiltValues = glm::rotate(tiltValues, (float)PI / 8);
}
void Simplex::Application::CameraFollow(MyEntity* PlayerEntity)
{
	//WHAT TO DO
	//get entity velocity
	//find the xz component of velocity
	//pick past velocity if it cannot be found 
	//move camera to opposite side of velocity
	//BONUS: raise camera and move it back slightly the faster the player is
	//make camera look at point above player position


	//gets entity velocity direction. 
	//if x and z is 0, then use past velocity that was valid (set to 0, 0, 1 by default)
	vector3 entityVelocity = PlayerEntity->GetRigidBody()->GetVelocity() / (1000.0f / ImGui::GetIO().Framerate);
	if (entityVelocity.x == 0 && entityVelocity.z == 0) {
		entityVelocity.x = pastValidEntityVelocity.x;
		entityVelocity.z = pastValidEntityVelocity.z;
	}
	else {
		pastValidEntityVelocity = entityVelocity;
	}

	//move camera to opposite side of velocity around player
	vector3 cameraDirection = m_pCameraMngr->GetPosition() - PlayerEntity->GetRigidBody()->GetCenterGlobal();
	vector2 currentCameraPosDir = glm::normalize(vector2(cameraDirection.x, cameraDirection.z));
	vector2 desiredCameraPosDir = glm::normalize(vector2(-entityVelocity.x, -entityVelocity.z));

	float dot = desiredCameraPosDir.x * currentCameraPosDir.x + desiredCameraPosDir.y * currentCameraPosDir.y;
	float det = desiredCameraPosDir.y * currentCameraPosDir.x - desiredCameraPosDir.x * currentCameraPosDir.y;

	float cameraAngleChange = atan2(desiredCameraPosDir.y, desiredCameraPosDir.x) - atan2(currentCameraPosDir.y, currentCameraPosDir.x);
	
	//limit it to -Pi to Pi only
	if (cameraAngleChange > PI) { cameraAngleChange -= 2 * PI; }
	else if (cameraAngleChange <= -PI) { cameraAngleChange += 2 * PI; }

	DBOUT("result"); //debugging angle
	DBOUT(cameraAngleChange); //debugging angle


	//how fast turning is overall
	float turningConstant = deltaTime * 6.0f;

	if (glm::abs(cameraAngleChange) <= 0.001f) {
		//almost right at desired camera position so snap to it
		currentCameraPosDir = desiredCameraPosDir;
	}
	else if (glm::abs(cameraAngleChange) > 0.6f) {
		//caps maximum turning speed
		currentCameraPosDir = glm::rotate(currentCameraPosDir, 0.6f * glm::sign(cameraAngleChange) * turningConstant);
	}
	else {
		//slows turning to desired position so it ends up being smoother transition
		currentCameraPosDir = glm::rotate(currentCameraPosDir, cameraAngleChange * 0.6f * turningConstant);
	}

	//finds final camera position and goes further up or down depending on player's vertical speed
	vector2 xzBehindPosition = currentCameraPosDir * 8.0f;
	float height = 3.5 + glm::min(-entityVelocity.y * 5, 5.0f);
	vector3 cameraPosition = vector3(xzBehindPosition.x, height, xzBehindPosition.y) + PlayerEntity->GetRigidBody()->GetCenterGlobal();


	m_pCameraMngr->SetPositionTargetAndUpward(
		cameraPosition,        //Position
		vector3(0, 2.2, 0) + PlayerEntity->GetRigidBody()->GetCenterGlobal(),	//Target
		AXIS_Y);					                                           //Up 

	//DBOUT("Camera"); //debugging angle
	//DBOUT(m_pCameraMngr->GetForward().x); //debugging angle
	//DBOUT(m_pCameraMngr->GetForward().y); //debugging angle
	//DBOUT(m_pCameraMngr->GetForward().z); //debugging angle

}
void Simplex::Application::CameraBirdsEye(MyEntity* PlayerEntity)
{
	//WHAT TO DO
	//find player position
	//set camera to sky with offset based on player location
	//look diagonally down at player


	//offset to player's xz coordinate
	vector2 xzOffsetPosition = vector2(0.0f, -7.0f);

	//offset in height. Want this to be high so camera is looking down
	vector3 entityVelocity = PlayerEntity->GetRigidBody()->GetVelocity() / (1000.0f / ImGui::GetIO().Framerate);
	float height = 5 + glm::min(-entityVelocity.y * 5, 5.0f);

	//set final camera position
	vector3 cameraPosition = vector3(xzOffsetPosition.x, height, xzOffsetPosition.y) + PlayerEntity->GetRigidBody()->GetCenterGlobal();

	//camera looks at spot just barely above player ball
	m_pCameraMngr->SetPositionTargetAndUpward(
		cameraPosition,        //Position
		vector3(0, 1.0, 0) + PlayerEntity->GetRigidBody()->GetCenterGlobal(),	//Target
		AXIS_Y);					                                           //Up 

}

//commented out code from initVariables() just in case we would like to look back at it
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//vector3 v3Position;
	//matrix4 m4Position;
	//m_pEntityMngr->AddEntity("Fulltiltadillo\\Tile.obj");
	//m4Position = glm::translate(vector3(0, -35, 0));
	//m_pEntityMngr->SetModelMatrix(m4Position);

	//m_pEntityMngr->AddEntity("Fulltiltadillo\\ArmadilloWorld1.obj");
	//m4Position = glm::translate(vector3(0, -35, 0));
	//m_pEntityMngr->SetModelMatrix(m4Position);
	/*for (int i = 0; i < nSquare; i++)
	{
		for (int j = 0; j < nSquare; j++)
		{
			uIndex++;
			m_pEntityMngr->AddEntity("Fulltiltadillo\\cactus.obj", true, false, 3);
			v3Position = vector3(glm::linearRand(-20.0f, 40.0f), 2, glm::linearRand(0.0f, 80.0f));
			m4Position = glm::translate(v3Position);
			m_pEntityMngr->SetModelMatrix(m4Position);

			m_uOctantLevels = 1;
			m_pEntityMngr->Update(m_uOctantLevels, vector3(0,0,0), vert, horz);
		}
	}*/