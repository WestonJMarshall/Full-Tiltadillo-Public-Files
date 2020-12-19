#include "LevelLoader.h"
#include <iostream>
#include <fstream>
using namespace Simplex;
using namespace std;

Simplex::LevelLoader::LevelLoader()
{
}

void Simplex::LevelLoader::LoadLevel(String fileName, MyEntityManager* entityManager, MeshManager* meshManager)
{
	std::vector<MyEntity*> levelEntityList();

	String line;

	//Position the entity being loaded will be placed at
	float yPos = 0.0f;
	float xPos = 0.0f;
	float zPos = 0.0f;
	float heightOffset = 0.95f;

	//Character being checked in the text file
	char c;
	
	//Position and matrix that will be given to the entity being loaded
	vector3 v3Position;
	glm::quat qRotation;
	matrix4 m4Position;
	bool cactus = false; //for the cactus input

	//adds the armadillo to the entity manager
	entityManager->AddEntity("Fulltiltadillo\\Armadillo.fbx", true, false, true, 3, 3);
	v3Position = vector3(5, -12.3, 10);
	m4Position = glm::translate(v3Position);
	entityManager->SetModelMatrix(m4Position);


	//for debugging tilting
/*
	entityManager->AddEntity("Fulltiltadillo\\unit_floor.fbx", false, true, false, 3);
	v3Position = vector3(5, -12.3, 10);
	m4Position = glm::translate(v3Position);
	entityManager->SetModelMatrix(m4Position);
*/

	//iterates through the file and adds objects to enity manager
	std::ifstream levelFile("Data\\LevelFiles\\" + fileName);
	if (levelFile.is_open())
	{
		//sets the rng seed based on the hash of the stage's name
		//This is so reloading that same stage will have cactus be placed in same spot
		srand(std::hash<String>{}(fileName));

		while (std::getline(levelFile, line))
		{
			zPos++;
			xPos = 0;
			for (int i = 0; i < line.length(); i++)
			{
				c = line[i];
				if (line[i] == '*') 
				{ 
					yPos+=5; xPos = 0.0f; zPos = 0.0f; 
					cactus = false;
				}
				if (line[i] == '-') { cactus = true;  xPos = 0.0f; zPos = 0.0f; }

				if (!cactus) //runs this if its regular map
				{
					switch (c)
					{
					case 'S': //Start
						entityManager->AddEntity("Fulltiltadillo\\Tile.fbx", false, true, false, 3);
						v3Position = vector3(xPos * 5 - 30, (yPos * heightOffset) - 20, zPos * 5);
						m4Position = glm::translate(v3Position);
						entityManager->SetModelMatrix(m4Position);
						break;
					case 'T': //Tile
						entityManager->AddEntity("Fulltiltadillo\\Tile.fbx", false, true, false, 3);
						v3Position = vector3(xPos * 5 - 30, (yPos * heightOffset) - 20, zPos * 5);
						m4Position = glm::translate(v3Position);
						entityManager->SetModelMatrix(m4Position);
						break;
					case 'G': //Goal
						entityManager->AddEntity("Fulltiltadillo\\Tile.fbx", false, true, false, 3);
						v3Position = vector3(xPos * 5 - 30, (yPos * heightOffset) - 20, zPos * 5);
						m4Position = glm::translate(v3Position);
						entityManager->SetModelMatrix(m4Position);
						entityManager->AddEntity("Fulltiltadillo\\GoalStructure.fbx", false, true, false, 3);
						v3Position = vector3(xPos * 5 - 30, (yPos * heightOffset) - 17.2f, zPos * 5);
						m4Position = glm::translate(v3Position);
						entityManager->SetModelMatrix(m4Position);
						entityManager->SetGoalIndex(entityManager->GetEntityCount()-1);
						break;
					case '^': //north Ramp 
						entityManager->AddEntity("Fulltiltadillo\\RampTileDown.fbx", false, true, false, 3);
						v3Position = vector3(xPos * 5 - 30, (yPos * heightOffset) - 17.5f, zPos * 5);
						m4Position = glm::translate(v3Position);
						qRotation = glm::rotate(m4Position, -0.785f, glm::vec3(1, 0, 0));
						m4Position = glm::translate(v3Position) * glm::toMat4(qRotation);
						entityManager->SetModelMatrix(m4Position);
						break;
					case 'v': //south Ramp
						entityManager->AddEntity("Fulltiltadillo\\RampTileDown.fbx", false, true, 3);
						v3Position = vector3(xPos * 5 - 30, (yPos * heightOffset) - 17.5f, zPos * 5);
						m4Position = glm::translate(v3Position);
						qRotation = glm::rotate(m4Position, 0.785f, glm::vec3(1, 0, 0));
						m4Position = glm::translate(v3Position) * glm::toMat4(qRotation);
						entityManager->SetModelMatrix(m4Position);
						break;
					case '<': //west Ramp
						entityManager->AddEntity("Fulltiltadillo\\RampTileDown.fbx", false, true, 3);
						v3Position = vector3(xPos * 5 - 30, (yPos * heightOffset) - 17.5f, zPos * 5);
						m4Position = glm::translate(v3Position);
						qRotation = glm::rotate(m4Position, -0.785f, glm::vec3(1, 0, 0));// *glm::rotate(m4Position, -1.5708f, glm::vec3(0, 0, 0));
						m4Position = glm::translate(v3Position) * glm::toMat4(qRotation);
						entityManager->SetModelMatrix(m4Position);
						break;
					case '>': //east Ramp
						entityManager->AddEntity("Fulltiltadillo\\RampTileDown.fbx", false, true, 3);
						v3Position = vector3(xPos * 5 - 30, (yPos * heightOffset) - 17.5f, zPos * 5);
						m4Position = glm::translate(v3Position);
						qRotation = glm::rotate(m4Position, -0.785f, glm::vec3(1, 0, 0));
						m4Position = glm::translate(v3Position) * glm::toMat4(qRotation);
						entityManager->SetModelMatrix(m4Position);
						break;

					case 'W': //Wall east
						entityManager->AddEntity("Fulltiltadillo\\Wall.fbx", false, true, false, 3);
						v3Position = vector3(xPos * 5 - 30 + 2.5f, (yPos * heightOffset) - 19.5f, zPos * 5);
						m4Position = glm::translate(v3Position);
						entityManager->SetModelMatrix(m4Position);
						break;
					case 'E': //Wall west
						entityManager->AddEntity("Fulltiltadillo\\Wall.fbx", false, true, false, 3);
						v3Position = vector3(xPos * 5 - 30 - 2.5f, (yPos * heightOffset) - 19.5f, zPos * 5);
						m4Position = glm::translate(v3Position);
						entityManager->SetModelMatrix(m4Position);
						break;
					case 'Q': //Wall north
						entityManager->AddEntity("Fulltiltadillo\\QWall.fbx", false, true, false, 3);
						v3Position = vector3(xPos * 5 - 30, (yPos * heightOffset) - 19.5f, zPos * 5 - 2.5f);
						m4Position = glm::translate(v3Position);
						entityManager->SetModelMatrix(m4Position);
						break;
					case 'X': //Wall south
						entityManager->AddEntity("Fulltiltadillo\\QWall.fbx", false, true, false, 3);
						v3Position = vector3(xPos * 5 - 30, (yPos * heightOffset) - 19.5f, zPos * 5 + 2.5f);
						m4Position = glm::translate(v3Position);

						//quick fix to fix a bug. When facing north, balls can phase through the right side
						//and we cannot figure out why. All other sides work just fine and all other walls 
						//work fine too. Best guess is the end vertices used for collision resolution is connecting
						//to the starting vertex of the model and is going the wrong way around, creating a normal in
						//the wrong direction.
						//
						//Solution: flip the wall around lol. It works!
						qRotation = glm::rotate(m4Position, (float)PI, glm::vec3(1, 0, 0)); 
						m4Position = glm::translate(v3Position) * glm::toMat4(qRotation);
						entityManager->SetModelMatrix(m4Position);
						break;
					default:
						break;
					}
					
				}
				else //runs this if its cactus map
				{
					switch (c)
					{
					case '1': //places 1 cactus in the area of a tile
						entityManager->AddEntity("Fulltiltadillo\\cactus.fbx", false, true, false, 3);
						v3Position = vector3(xPos * 5 - 30 + glm::linearRand(-2.5f, 2.5f), (yPos * heightOffset) - 18.0f, zPos * 5 - glm::linearRand(-2.5f, 2.5f));
						m4Position = glm::translate(v3Position);
						entityManager->SetModelMatrix(m4Position);
						break;

					case '4': //other rollable spheres
						entityManager->AddEntity("Fulltiltadillo\\Armadillo.fbx", true, false, true, 3, 3);
						v3Position = vector3(xPos * 5 - 30, (yPos * heightOffset) - 18, zPos * 5);
						m4Position = glm::translate(v3Position);
						entityManager->SetModelMatrix(m4Position);
						break;
					default:
						break;
					}
				}
				xPos++;
			}
		}
		levelFile.close();
	}
}
