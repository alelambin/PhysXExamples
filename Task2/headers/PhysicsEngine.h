#pragma once

#include <iostream>
#include <vector>
#include <unordered_set>
#include "PxPhysicsAPI.h"

#ifdef _DEBUG
#define USE_PVD
#define PVD_HOST "127.0.0.1"
#endif

class PhysicsEngine {
public:
	PhysicsEngine();
	~PhysicsEngine();
	void Simulate(float elapsedTime);
	physx::PxMaterial *CreateMaterial(float staticFriction, float dynamicFriction, float restitution);
	physx::PxMaterial *GetMaterial(float staticFriction, float dynamicFriction, float restitution);
	physx::PxShape *CreateBoxShape(
		physx::PxVec3 size,
		physx::PxTransform transform,
		physx::PxMaterial *material,
		bool isExclusive = false,
		physx::PxShapeFlags shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE
	);
	physx::PxShape *CreateSphereShape(
		float radius,
		physx::PxTransform transform,
		physx::PxMaterial *material,
		bool isExclusive = false,
		physx::PxShapeFlags shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE
	);
	physx::PxShape *CreateCapsuleShape(
		float radius,
		float size,
		physx::PxTransform transform,
		physx::PxMaterial *material,
		bool isExclusive = false,
		physx::PxShapeFlags shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE
	);
	physx::PxRigidStatic *AddGround(physx::PxVec3 normal, float distance, physx::PxMaterial *material);
	physx::PxRigidStatic *AddStaticActor(physx::PxShape *shape, physx::PxTransform transform);
	physx::PxRigidDynamic *AddDynamicActor(physx::PxShape *shape, physx::PxTransform transform, float density);
	physx::PxRigidStatic *AddStaticActor(std::vector<physx::PxShape *> shapes, physx::PxTransform transform);
	physx::PxRigidDynamic *AddDynamicActor(std::vector <physx::PxShape *> shapes, physx::PxTransform transform, float density);
	std::vector<physx::PxRigidActor *> GetActors(physx::PxActorTypeFlags types = physx::PxActorTypeFlag::eRIGID_STATIC | physx::PxActorTypeFlag::eRIGID_DYNAMIC);
	void RemoveActor(physx::PxActor *actor);
	void MarkActor(physx::PxActor *actor);

private:
	physx::PxDefaultAllocator allocatorCallback;
	physx::PxDefaultErrorCallback errorCallback;
	physx::PxFoundation *foundation;
	physx::PxPhysics *physics;
#ifdef USE_PVD
	physx::PxPvd *pvd;
	physx::PxPvdTransport *transport;
#endif
	physx::PxScene *scene;
	std::unordered_set<physx::PxActor *> markedActors;

};
