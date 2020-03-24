/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "PrimitiveShape.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"

struct PrimitiveShapeExample : public CommonRigidBodyBase
{
	PrimitiveShapeExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~PrimitiveShapeExample() {}
	virtual void initPhysics();
	virtual void renderScene();
	virtual bool keyboardCallback(int key, int state);
	void resetCamera()
	{
		float dist = 10;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0.46, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	void addCapsule(btScalar radius, btScalar height);
	void addCylinder(btVector3& v);
};

void PrimitiveShapeExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));
	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

	// //BOX
	// {
	// 	//create a few dynamic rigidbodies
	// 	// Re-using the same collision is better for memory usage and performance
	// 	btBoxShape* colShape = createBoxShape(btVector3(1, 1, 1));

	// 	int address = (int)colShape;
	// 	btCollisionShape* p = (btCollisionShape*)address;
	// 	p->setLocalScaling(btVector3(btScalar(1.), btScalar(1.), btScalar(1.)));

	// 	m_collisionShapes.push_back(colShape);

	// 	/// Create Dynamic Objects
	// 	btTransform startTransform;
	// 	startTransform.setIdentity();

	// 	btScalar mass(1.f);

	// 	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	// 	bool isDynamic = (mass != 0.f);

	// 	btVector3 localInertia(0, 0, 0);
	// 	if (isDynamic)
	// 		colShape->calculateLocalInertia(mass, localInertia);

	// 	startTransform.setOrigin(btVector3(
	// 		btScalar(3),
	// 		btScalar(20),
	// 		btScalar(0)));
	// 	createRigidBody(mass, startTransform, colShape);
	// }

	// //SPHERE
	// {
	// 	//create a few dynamic rigidbodies
	// 	// Re-using the same collision is better for memory usage and performance
	// 	btSphereShape* colShape = new btSphereShape(0.5);

	// 	int address = (int)colShape;
	// 	btCollisionShape* p = (btCollisionShape*)address;
	// 	p->setLocalScaling(btVector3(btScalar(1.), btScalar(1.), btScalar(1.)));

	// 	m_collisionShapes.push_back(colShape);

	// 	/// Create Dynamic Objects
	// 	btTransform startTransform;
	// 	startTransform.setIdentity();

	// 	btScalar mass(1.f);

	// 	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	// 	bool isDynamic = (mass != 0.f);

	// 	btVector3 localInertia(0, 0, 0);
	// 	if (isDynamic)
	// 		colShape->calculateLocalInertia(mass, localInertia);

	// 	startTransform.setOrigin(btVector3(
	// 		btScalar(0),
	// 		btScalar(20),
	// 		btScalar(3)));
	// 	createRigidBody(mass, startTransform, colShape);
	// }
	btScalar r = 0.5;
	btScalar h = 1;
	btVector3 s(1., 1., 1.);
	//CAPSULE
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		btCapsuleShape* colShape = new btCapsuleShape(r, h);

		int address = (int)colShape;
		btCollisionShape* p = (btCollisionShape*)address;
		p->setLocalScaling(s);

		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(0.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(
			btScalar(-0.5),
			btScalar(3.01),
			btScalar(0)));
		createRigidBody(mass, startTransform, colShape);
	}
	btScalar r2 = 2.;
	//CYLINDER
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		btCylinderShape* colShape = new btCylinderShape(btVector3(r, h, r2));

		int address = (int)colShape;
		btCollisionShape* p = (btCollisionShape*)address;
		p->setLocalScaling(s);

		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(
			btScalar(4),
			btScalar(10),
			btScalar(0)));
		createRigidBody(mass, startTransform, colShape);
	}
	//CYLINDER
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		btCylinderShape* colShape = new btCylinderShape(btVector3(r, 1, r2));

		int address = (int)colShape;
		btCollisionShape* p = (btCollisionShape*)address;
		p->setLocalScaling(s);

		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(0.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(
			btScalar(5),
			btScalar(3),
			btScalar(0)));
		createRigidBody(mass, startTransform, colShape);
	}

	// //CONE
	// {
	// 	//create a few dynamic rigidbodies
	// 	// Re-using the same collision is better for memory usage and performance
	// 	btConeShape* colShape = new btConeShape(0.5, 1);

	// 	int address = (int)colShape;
	// 	btCollisionShape* p = (btCollisionShape*)address;
	// 	p->setLocalScaling(btVector3(btScalar(1.), btScalar(1.), btScalar(1.)));

	// 	m_collisionShapes.push_back(colShape);

	// 	/// Create Dynamic Objects
	// 	btTransform startTransform;
	// 	startTransform.setIdentity();

	// 	btScalar mass(1.f);

	// 	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	// 	bool isDynamic = (mass != 0.f);

	// 	btVector3 localInertia(0, 0, 0);
	// 	if (isDynamic)
	// 		colShape->calculateLocalInertia(mass, localInertia);

	// 	startTransform.setOrigin(btVector3(
	// 		btScalar(6),
	// 		btScalar(20),
	// 		btScalar(0)));
	// 	createRigidBody(mass, startTransform, colShape);
	// }

	// //CONVEX HULL
	// {
	// 	btConvexHullShape* colShape = new btConvexHullShape();

	// 	btScalar scaling(1);

	// 	colShape->setLocalScaling(btVector3(scaling, scaling, scaling));

	// 	for (int i = 0; i < 10; i++)
	// 	{
	// 		btVector3 vtx(rand() % 5 + 1, rand() % 5 + 1, rand() % 5 + 1);
	// 		colShape->addPoint(vtx * btScalar(1. / scaling));
	// 	}
	// 	m_collisionShapes.push_back(colShape);

	// 	/// Create Dynamic Objects
	// 	btTransform startTransform;
	// 	startTransform.setIdentity();

	// 	btScalar mass(1.f);

	// 	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	// 	bool isDynamic = (mass != 0.f);

	// 	btVector3 localInertia(0, 0, 0);
	// 	if (isDynamic)
	// 		colShape->calculateLocalInertia(mass, localInertia);

	// 	startTransform.setOrigin(btVector3(
	// 		btScalar(6),
	// 		btScalar(20),
	// 		btScalar(0)));
	// 	createRigidBody(mass, startTransform, colShape);
	// }

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

bool PrimitiveShapeExample::keyboardCallback(int key, int state)
{
	if (key == 81)
	{
	}
	else if (key == 69)
	{
	}

	if (state)
	{
	}

	return false;
}

void PrimitiveShapeExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

void PrimitiveShapeExample::addCapsule(btScalar radius, btScalar height)
{
	//create a few dynamic rigidbodies
	// Re-using the same collision is better for memory usage and performance
	btCapsuleShape* colShape = new btCapsuleShape(radius, height);

	int address = (int)colShape;
	btCollisionShape* p = (btCollisionShape*)address;
	p->setLocalScaling(btVector3(btScalar(10.), btScalar(10.), btScalar(10.)));

	m_collisionShapes.push_back(colShape);

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar mass(0.f);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass, localInertia);

	startTransform.setOrigin(btVector3(
		btScalar(3),
		btScalar(20),
		btScalar(3)));
	createRigidBody(mass, startTransform, colShape);
}

CommonExampleInterface* ET_PrimitiveShapeCreateFunc(CommonExampleOptions& options)
{
	return new PrimitiveShapeExample(options.m_guiHelper);
}
