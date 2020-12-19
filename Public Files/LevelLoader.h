#ifndef __LEVELLOADER_H_
#define __LEVELLOADER_H_

#include "MyEntityManager.h"

namespace Simplex
{
	//System Class
	class LevelLoader
	{

	public:
		LevelLoader();

		void LoadLevel(String fileName,MyEntityManager* entityManager, MeshManager* meshManager);
		
	private:

	};//class

} //namespace Simplex

#endif //__LEVELLOADER_H_

