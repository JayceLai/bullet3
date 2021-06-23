
#include "BasicBvhTriangleMesh.h"

///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"

#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"

#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"

#include "BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"

class btDynamicsWorld;

class btRigidBody;
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;

#include "../MultiThreadedDemo/CommonRigidBodyMTBase.h"
//#include "../Benchmarks/TaruData.h"
//#include "../Benchmarks/landscapeData.h"

class BasicBvhTriangleMesh : public CommonRigidBodyMTBase
{
	/*
	void createWall(const btVector3& offsetPosition, int stackSize, const btVector3& boxSize);
	void createPyramid(const btVector3& offsetPosition, int stackSize, const btVector3& boxSize);
	void createTowerCircle(const btVector3& offsetPosition, int stackSize, int rotSize, const btVector3& boxSize);
	*/
	void createLargeMeshBody();
	void createTest();
	void createTest1();
	void createTest2();
	void createTest3();

	void initPhysics();
	//void exitPhysics();

	public:
	
	int m_tag;
	btRigidBody* m_character = nullptr;

	BasicBvhTriangleMesh(struct GUIHelperInterface* helper, int tag)
			: CommonRigidBodyMTBase(helper), m_tag(tag)
	{
	}

	virtual ~BasicBvhTriangleMesh()
	{
		CommonRigidBodyMTBase::exitPhysics();
	}

	void stepSimulation(float deltaTime);

	void resetCamera()
	{
		float dist = 30;
		float pitch = -60;
		float yaw = 0;// 45;
		float targetPos[3] = {0, 10, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

};

/////////////////////////////////////////////////////////////////////////////
// LargeMesh
//
extern int LandscapeVtxCount[];
extern int LandscapeIdxCount[];
extern btScalar* LandscapeVtx[];
extern btScalar* LandscapeNml[];
extern btScalar* LandscapeTex[];
extern unsigned short* LandscapeIdx[];

void BasicBvhTriangleMesh::createLargeMeshBody()
{
	btTransform trans;
	trans.setIdentity();

	for (int i = 0; i < 8; i++)
	{
		btTriangleIndexVertexArray* meshInterface = new btTriangleIndexVertexArray();
		btIndexedMesh part;

		part.m_vertexBase = (const unsigned char*)LandscapeVtx[i];
		part.m_vertexStride = sizeof(btScalar) * 3;
		part.m_numVertices = LandscapeVtxCount[i];
		part.m_triangleIndexBase = (const unsigned char*)LandscapeIdx[i];
		part.m_triangleIndexStride = sizeof(short) * 3;
		part.m_numTriangles = LandscapeIdxCount[i] / 3;
		part.m_indexType = PHY_SHORT;

		meshInterface->addIndexedMesh(part, PHY_SHORT);

		bool useQuantizedAabbCompression = true;
		btMultimaterialTriangleMeshShape* trimeshShape = new btMultimaterialTriangleMeshShape(meshInterface, useQuantizedAabbCompression);
		btTransform trimeshTrans;
		trimeshTrans.setIdentity();		
		btCompoundShape* comShape = new btCompoundShape();
		trimeshShape->setUserIndex(comShape->getNumChildShapes());
		comShape->addChildShape(trimeshTrans, trimeshShape);
		// comShape->setMaterial(0, 0.5, 0.1, 0.1);

		btVector3 localInertia(0, 0, 0);
		trans.setOrigin(btVector3(0, -25, 0));
		btRigidBody* body = createRigidBody(0, trans, comShape);
		body->setFriction(btScalar(0.9));
	}
}

void BasicBvhTriangleMesh::createTest()
{
	btVector3 boxSize(1.5f, 1.5f, 1.5f);
	float boxMass = 1.0f;
	float sphereRadius = 1.5f;
	float sphereMass = 1.0f;
	float capsuleHalf = 2.0f;
	float capsuleRadius = 1.0f;
	float capsuleMass = 1.0f;

	{
		/*
		int size = 10;
		int height = 10;*/

		int size = 3;
		int height = 1;

		const float cubeSize = boxSize[0];
		float spacing = 2.0f;
		btVector3 pos(0.0f, 20.0f, 0.0f);
		float offset = -size * (cubeSize * 2.0f + spacing) * 0.5f;

		int numBodies = 0;

		for (int k = 0; k < height; k++)
		{
			for (int j = 0; j < size; j++)
			{
				pos[2] = offset + (float)j * (cubeSize * 2.0f + spacing);
				for (int i = 0; i < size; i++)
				{
					pos[0] = offset + (float)i * (cubeSize * 2.0f + spacing);
					btVector3 bpos = btVector3(0, 25, 0) + btVector3(5.0f, 1.0f, 5.0f) * pos;
					int idx = rand() % 9;
					btTransform trans;
					trans.setIdentity();
					trans.setOrigin(bpos);

					switch (idx)
					{
						case 0:
						case 1:
						case 2:
						{
							float r = 0.5f * (idx + 1);
							btBoxShape* boxShape = new btBoxShape(boxSize * r);
							btTransform boxTrans;
							boxTrans.setIdentity();
							btCompoundShape* comShape = new btCompoundShape();
							comShape->addChildShape(boxTrans, boxShape);
							comShape->setMaterial(0, 0.5, 0.1, 0.1);
							createRigidBody(boxMass * r, trans, comShape);
						}
						break;

						case 3:
						case 4:
						case 5:
						{
							float r = 0.5f * (idx - 3 + 1);
							btSphereShape* sphereShape = new btSphereShape(sphereRadius * r);
							btTransform sphereTrans;
							sphereTrans.setIdentity();
							btCompoundShape* comShape = new btCompoundShape();
							comShape->addChildShape(sphereTrans, sphereShape);
							comShape->setMaterial(0, 0.5, 0.1, 0.1);
							createRigidBody(sphereMass * r, trans, comShape);
						}
						break;

						case 6:
						case 7:
						case 8:
						{
							float r = 0.5f * (idx - 6 + 1);
							btCapsuleShape* capsuleShape = new btCapsuleShape(capsuleRadius * r, capsuleHalf * r);
							btTransform capsuleTrans;
							capsuleTrans.setIdentity();
							btCompoundShape* comShape = new btCompoundShape();
							comShape->addChildShape(capsuleTrans, capsuleShape);
							comShape->setMaterial(0, 0.5, 0.1, 0.1);
							createRigidBody(capsuleMass * r, trans, comShape);
						}
						break;
					}

					numBodies++;
				}
			}
			offset -= 0.05f * spacing * (size - 1);
			spacing *= 1.1f;
			pos[1] += (cubeSize * 2.0f + spacing);
		}
	}

	createLargeMeshBody();
}

void BasicBvhTriangleMesh::createTest1()
{
	btVector3 boxSize(1.5f, 1.5f, 1.5f);
	float boxMass = 1.0f;
	float sphereRadius = 1.5f;
	float sphereMass = 1.0f;
	float capsuleHalf = 2.0f;
	float capsuleRadius = 1.0f;
	float capsuleMass = 1.0f;

	{
		/*
		int size = 10;
		int height = 10;*/

		int size = 3;
		int height = 1;

		const float cubeSize = boxSize[0];
		float spacing = 2.0f;
		btVector3 pos(0.0f, 20.0f, 0.0f);
		float offset = -size * (cubeSize * 2.0f + spacing) * 0.5f;

		int numBodies = 0;

		for (int k = 0; k < height; k++)
		{
			for (int j = 0; j < size; j++)
			{
				pos[2] = offset + (float)j * (cubeSize * 2.0f + spacing);
				for (int i = 0; i < size; i++)
				{
					pos[0] = offset + (float)i * (cubeSize * 2.0f + spacing);
					btVector3 bpos = btVector3(0, 25, 0) + btVector3(5.0f, 1.0f, 5.0f) * pos;
					int idx = rand() % 9;
					btTransform trans;
					trans.setIdentity();
					trans.setOrigin(bpos);

					switch (idx)
					{
						case 0:
						case 1:
						case 2:
						{
							float r = 0.5f * (idx + 1);
							btBoxShape* boxShape = new btBoxShape(boxSize * r);
							btTransform boxTrans;
							boxTrans.setIdentity();
							btCompoundShape* comShape = new btCompoundShape();
							comShape->addChildShape(boxTrans, boxShape);
							comShape->setMaterial(0, 0.5, 0.1, 0.1);
							createRigidBody(boxMass * r, trans, comShape);
						}
						break;

						case 3:
						case 4:
						case 5:
						{
							float r = 0.5f * (idx - 3 + 1);
							btSphereShape* sphereShape = new btSphereShape(sphereRadius * r);
							btTransform sphereTrans;
							sphereTrans.setIdentity();
							btCompoundShape* comShape = new btCompoundShape();
							comShape->addChildShape(sphereTrans, sphereShape);
							comShape->setMaterial(0, 0.5, 0.1, 0.1);
							createRigidBody(sphereMass * r, trans, comShape);
						}
						break;

						case 6:
						case 7:
						case 8:
						{
							float r = 0.5f * (idx - 6 + 1);
							btCapsuleShape* capsuleShape = new btCapsuleShape(capsuleRadius * r, capsuleHalf * r);
							btTransform capsuleTrans;
							capsuleTrans.setIdentity();
							btCompoundShape* comShape = new btCompoundShape();
							comShape->addChildShape(capsuleTrans, capsuleShape);
							comShape->setMaterial(0, 0.5, 0.1, 0.1);
							createRigidBody(capsuleMass * r, trans, comShape);
						}
						break;
					}

					numBodies++;
				}
			}
			offset -= 0.05f * spacing * (size - 1);
			spacing *= 1.1f;
			pos[1] += (cubeSize * 2.0f + spacing);
		}
	}

	btTransform trans;
	trans.setIdentity();

	btTriangleIndexVertexArray* meshInterface;
	btBvhTriangleMeshShape* trimeshShape;
	btTransform trimeshTrans;
	trimeshTrans.setIdentity();
	btCompoundShape* comShape;
	btRigidBody* body;

	{
		int i = 2;
		meshInterface = new btTriangleIndexVertexArray();
		btIndexedMesh part;

		part.m_vertexBase = (const unsigned char*)LandscapeVtx[i];
		part.m_vertexStride = sizeof(btScalar) * 3;
		part.m_numVertices = LandscapeVtxCount[i];
		part.m_triangleIndexBase = (const unsigned char*)LandscapeIdx[i];
		part.m_triangleIndexStride = sizeof(short) * 3;
		part.m_numTriangles = LandscapeIdxCount[i] / 3;
		part.m_indexType = PHY_SHORT;

		meshInterface->addIndexedMesh(part, PHY_SHORT);

		bool useQuantizedAabbCompression = true;
		//btMultimaterialTriangleMeshShape* trimeshShape = new btMultimaterialTriangleMeshShape(meshInterface, useQuantizedAabbCompression);
		trimeshShape = new btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression);
		
		comShape = new btCompoundShape();
		trimeshShape->setUserIndex(comShape->getNumChildShapes());
		comShape->addChildShape(trimeshTrans, trimeshShape);
		comShape->setMaterial(0, 0.5, 0.1, 0.1);

		btVector3 localInertia(0, 0, 0);
		trans.setOrigin(btVector3(0, -25, 0));
	
		body = createRigidBody(0, trans, comShape);
		body->setFriction(btScalar(0.9));
	}

	{
		//return;
		int i = 3;
		meshInterface = new btTriangleIndexVertexArray();
		btIndexedMesh part;

		part.m_vertexBase = (const unsigned char*)LandscapeVtx[i];
		part.m_vertexStride = sizeof(btScalar) * 3;
		part.m_numVertices = LandscapeVtxCount[i];
		part.m_triangleIndexBase = (const unsigned char*)LandscapeIdx[i];
		part.m_triangleIndexStride = sizeof(short) * 3;
		part.m_numTriangles = LandscapeIdxCount[i] / 3;
		part.m_indexType = PHY_SHORT;

		meshInterface->addIndexedMesh(part, PHY_SHORT);

		bool useQuantizedAabbCompression = true;
		//btMultimaterialTriangleMeshShape* trimeshShape = new btMultimaterialTriangleMeshShape(meshInterface, useQuantizedAabbCompression);
		trimeshShape = new btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression);

		/*comShape = new btCompoundShape();
		trimeshShape->setUserIndex(comShape->getNumChildShapes());
		comShape->addChildShape(trimeshTrans, trimeshShape);
		comShape->setMaterial(0, 0.5, 0.1, 0.1);*/

		comShape->removeChildShapeByIndex(0);
		trimeshShape->setUserIndex(comShape->getNumChildShapes());
		comShape->addChildShape(trimeshTrans, trimeshShape);
		comShape->setMaterial(0, 0.5, 0.1, 0.1);

		body->setCollisionShape(comShape);
	}	
	

}

void BasicBvhTriangleMesh::createTest2()
{
	btVector3 boxSize(1.5f, 1.5f, 1.5f);
	float boxMass = 1.0f;
	float sphereRadius = 1.5f;
	float sphereMass = 1.0f;
	float capsuleHalf = 2.0f;
	float capsuleRadius = 1.0f;
	float capsuleMass = 1.0f;

	{
		/*
		int size = 10;
		int height = 10;*/

		int size = 3;
		int height = 1;

		const float cubeSize = boxSize[0];
		float spacing = 2.0f;
		btVector3 pos(0.0f, 20.0f, 0.0f);
		float offset = -size * (cubeSize * 2.0f + spacing) * 0.5f;

		int numBodies = 0;

		for (int k = 0; k < height; k++)
		{
			for (int j = 0; j < size; j++)
			{
				pos[2] = offset + (float)j * (cubeSize * 2.0f + spacing);
				for (int i = 0; i < size; i++)
				{
					pos[0] = offset + (float)i * (cubeSize * 2.0f + spacing);
					btVector3 bpos = btVector3(0, 25, 0) + btVector3(5.0f, 1.0f, 5.0f) * pos;
					int idx = rand() % 9;
					btTransform trans;
					trans.setIdentity();
					trans.setOrigin(bpos);

					switch (idx)
					{
						case 0:
						case 1:
						case 2:
						{
							float r = 0.5f * (idx + 1);
							btBoxShape* boxShape = new btBoxShape(boxSize * r);
							btTransform boxTrans;
							boxTrans.setIdentity();
							btCompoundShape* comShape = new btCompoundShape();
							comShape->addChildShape(boxTrans, boxShape);
							comShape->setMaterial(0, 0.5, 0.1, 0.1);
							createRigidBody(boxMass * r, trans, comShape);
						}
						break;

						case 3:
						case 4:
						case 5:
						{
							float r = 0.5f * (idx - 3 + 1);
							btSphereShape* sphereShape = new btSphereShape(sphereRadius * r);
							btTransform sphereTrans;
							sphereTrans.setIdentity();
							btCompoundShape* comShape = new btCompoundShape();
							comShape->addChildShape(sphereTrans, sphereShape);
							comShape->setMaterial(0, 0.5, 0.1, 0.1);
							createRigidBody(sphereMass * r, trans, comShape);
						}
						break;

						case 6:
						case 7:
						case 8:
						{
							float r = 0.5f * (idx - 6 + 1);
							btCapsuleShape* capsuleShape = new btCapsuleShape(capsuleRadius * r, capsuleHalf * r);
							btTransform capsuleTrans;
							capsuleTrans.setIdentity();
							btCompoundShape* comShape = new btCompoundShape();
							comShape->addChildShape(capsuleTrans, capsuleShape);
							comShape->setMaterial(0, 0.5, 0.1, 0.1);
							createRigidBody(capsuleMass * r, trans, comShape);
						}
						break;
					}

					numBodies++;
				}
			}
			offset -= 0.05f * spacing * (size - 1);
			spacing *= 1.1f;
			pos[1] += (cubeSize * 2.0f + spacing);
		}
	}

	btTransform trans;
	trans.setIdentity();

	btTriangleIndexVertexArray* meshInterface = new btTriangleIndexVertexArray();

	for (int i = 0; i < 8; i++)
	{
		btIndexedMesh part;

		part.m_vertexBase = (const unsigned char*)LandscapeVtx[i];
		part.m_vertexStride = sizeof(btScalar) * 3;
		part.m_numVertices = LandscapeVtxCount[i];
		part.m_triangleIndexBase = (const unsigned char*)LandscapeIdx[i];
		part.m_triangleIndexStride = sizeof(short) * 3;
		part.m_numTriangles = LandscapeIdxCount[i] / 3;
		part.m_indexType = PHY_SHORT;

		meshInterface->addIndexedMesh(part, PHY_SHORT);
	}

	bool useQuantizedAabbCompression = true;
	btMultimaterialTriangleMeshShape* trimeshShape = new btMultimaterialTriangleMeshShape(meshInterface, useQuantizedAabbCompression);
	btTransform trimeshTrans;
	trimeshTrans.setIdentity();
	btCompoundShape* comShape = new btCompoundShape();
	trimeshShape->setUserIndex(comShape->getNumChildShapes());
	comShape->addChildShape(trimeshTrans, trimeshShape);
	comShape->setMaterial(0, 0.5, 0.1, 0.1);

	btVector3 localInertia(0, 0, 0);
	trans.setOrigin(btVector3(0, -25, 0));
	btRigidBody* body = createRigidBody(0, trans, comShape);
	body->setFriction(btScalar(0.9));
}

void BasicBvhTriangleMesh::createTest3()
{
	m_dynamicsWorld->setGravity(btVector3(0., -25., 0.));

	btTransform trans;
	float capsuleHalf = 1.0f;
	float capsuleRadius = 0.5f;
	float capsuleMass = 1.0f;
	btCapsuleShape* capsuleShape = new btCapsuleShape(capsuleRadius, capsuleHalf);
	btTransform capsuleTrans;
	capsuleTrans.setIdentity();
	btCompoundShape* comShape = new btCompoundShape();
	comShape->addChildShape(capsuleTrans, capsuleShape);
	// comShape->setMaterial(0, 0.5, 0.1, 0.1);
	trans.setIdentity();
	trans.setOrigin(btVector3(0.15, 1, 50));
	auto rigid = createRigidBody(capsuleMass, trans, comShape);
	rigid->setAngularFactor(0.);
	m_character = rigid;

	auto meshInterface = new btTriangleMesh();
	// genearte a plane mesh
	const float scales = 1.f;
	const int size = 100;
	const int center = size / 2;
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			auto _i = i - center, _j = j - center;
			auto vertex = new btVector3(_i * scales, 0.f, _j * scales);
			meshInterface->findOrAddVertex(*vertex, false);
			if (i < size - 1 && j < size - 1){
				auto a = j * size + i, b = a + 1, c = (j + 1) * size + i, d = c + 1;
				meshInterface->addTriangleIndices(a, b, c);
				meshInterface->addTriangleIndices(b, d, c);
			}
		}
	}

	bool useQuantizedAabbCompression = true;
	// btBvhTriangleMeshShape* meshShape = new btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression);
	btConvexTriangleMeshShape* meshShape = new btConvexTriangleMeshShape(meshInterface);
	trans.setIdentity();
	// trans.setOrigin(btVector3(0, -25, 0));
	createRigidBody(0, trans, meshShape);
}


void BasicBvhTriangleMesh::initPhysics()
{
	m_guiHelper->setUpAxis(1);
	createEmptyDynamicsWorld();

	///the following 3 lines increase the performance dramatically, with a little bit of loss of quality
	// m_dynamicsWorld->getSolverInfo().m_solverMode |= SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;  //don't recalculate friction values each frame
	// m_dynamicsWorld->getSolverInfo().m_numIterations = 5;                                       //few solver iterations
	//m_defaultContactProcessingThreshold = 0.f;//used when creating bodies: body->setContactProcessingThreshold(...);
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));

	switch (m_tag)
	{
		case 0:
			createTest();
			break;
		case 1:
			createTest1();
			break;
		case 3:
			createTest3();
			break;
		default:
			createTest();
			break;
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

//
//void BasicBvhTriangleMesh::exitPhysics()
//{
//	CommonRigidBodyMTBase::exitPhysics();
//}

	

void BasicBvhTriangleMesh::stepSimulation(float deltaTime)
{
	if (m_dynamicsWorld)
	{
		if (m_character) {
			auto v = m_character->getLinearVelocity();
			v += btVector3(0, 0, -5);
			if (v.length() > 5) {
				v.normalize();
				v *= 5;
			}
			m_character->setLinearVelocity(v);
		}
		m_dynamicsWorld->stepSimulation(deltaTime);
		if (m_character) {
			auto& v = m_character->getLinearVelocity();
			if (v.x() != 0. || v.y() != 0.) {
				b3Printf("velocity v3 = ( %f,  %f,  %f )", v.x(), v.y(), v.z());
				auto& trans = m_character->getWorldTransform();
				auto& origin = trans.getOrigin();
				b3Printf("pos v3 = ( %f,  %f,  %f )", origin.x(), origin.y(), origin.z());
			}
		}
	}
}

CommonExampleInterface* BasicBvhTriangleMeshCreateFunc(struct CommonExampleOptions& options)
{
	return new BasicBvhTriangleMesh(options.m_guiHelper, options.m_option);
}
