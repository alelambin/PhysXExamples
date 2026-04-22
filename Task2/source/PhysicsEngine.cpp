#include "PhysicsEngine.h"

#define SAFE_RELEASE(obj) {	\
	if (obj) {				\
		obj->release();		\
		obj = nullptr;		\
	}						\
}

#define IS_FLOATS_EQUAL(f1, f2) (fabsf((f1) - (f2)) < 1e-5)

PhysicsEngine::PhysicsEngine() {
	foundation = PxGetFoundation(PX_PHYSICS_VERSION, allocatorCallback, errorCallback);

#ifdef USE_PVD
	pvd = physx::PxCreatePvd(*foundation);
	transport = physx::PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10000);
	if (!pvd || !transport || !pvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL)) {
		std::cout << "[WARNING] Could not initialize or connect pvd\n";
	}

	physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, physx::PxTolerancesScale(), true, pvd);
#else
	physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, physx::PxTolerancesScale(), false);
#endif

	physx::PxSceneDesc sceneDesc = physx::PxSceneDesc(physics->getTolerancesScale());
	sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
	sceneDesc.cpuDispatcher = physx::PxDefaultCpuDispatcherCreate(2);
	sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;

	scene = physics->createScene(sceneDesc);

#ifdef USE_PVD
	if (pvd && pvd->isConnected()) {
		physx::PxPvdSceneClient *pvdClient = scene->getScenePvdClient();
		if (pvdClient) {
			pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
			pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
			pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
		}
		scene->setVisualizationParameter(physx::PxVisualizationParameter::eSCALE, 1.0f);
		scene->setVisualizationParameter(physx::PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);
		scene->setVisualizationParameter(physx::PxVisualizationParameter::eJOINT_LIMITS, 1.0f);
	}
#endif
}

PhysicsEngine::~PhysicsEngine() {
	std::vector<physx::PxRigidActor *> actors = GetActors();
	for (auto *actor : actors) {
		uint32_t shapesNum = actor->getNbShapes();
		if (shapesNum > 0) {
			std::vector<physx::PxShape *> shapes(shapesNum);
			actor->getShapes(reinterpret_cast<physx::PxShape **>(shapes.data()), shapesNum);
			for (auto *shape : shapes) {
				SAFE_RELEASE(shape);
			}
		}
		SAFE_RELEASE(actor);
	}

	uint32_t materialsNum = physics->getNbMaterials();
	if (materialsNum > 0) {
		std::vector<physx::PxMaterial *> materials(materialsNum);
		physics->getMaterials(reinterpret_cast<physx::PxMaterial **>(materials.data()), materialsNum);
		for (auto *material : materials) {
			SAFE_RELEASE(material);
		}
	}

	SAFE_RELEASE(scene);
	SAFE_RELEASE(physics);
#ifdef USE_PVD
	if (pvd && pvd->isConnected()) {
		pvd->disconnect();
		pvd = nullptr;
	}
	SAFE_RELEASE(transport);
#endif
	SAFE_RELEASE(foundation);
}

void PhysicsEngine::Simulate(float elapsedTime) {
	scene->simulate(elapsedTime);
	scene->fetchResults(true);

	for (physx::PxActor *actor : markedActors) {
		RemoveActor(actor);
		SAFE_RELEASE(actor);
	}
	markedActors.clear();
}

physx::PxMaterial *PhysicsEngine::CreateMaterial(float staticFriction, float dynamicFriction, float restitution) {
	physx::PxMaterial *material = physics->createMaterial(staticFriction, dynamicFriction, restitution);
	return material;
}

physx::PxMaterial *PhysicsEngine::GetMaterial(float staticFriction, float dynamicFriction, float restitution) {
	uint32_t materialsNum = physics->getNbMaterials();
	if (materialsNum > 0) {
		std::vector<physx::PxMaterial *> materials(materialsNum);
		physics->getMaterials(reinterpret_cast<physx::PxMaterial **>(materials.data()), materialsNum);
		for (auto *material : materials) {
			if (IS_FLOATS_EQUAL(material->getStaticFriction(), staticFriction)
				&& IS_FLOATS_EQUAL(material->getDynamicFriction(), dynamicFriction)
				&& IS_FLOATS_EQUAL(material->getRestitution(), restitution)) {
				return material;
			}
		}
	}
	return CreateMaterial(staticFriction, dynamicFriction, restitution);
}

physx::PxShape *PhysicsEngine::CreateBoxShape(
	physx::PxVec3 size,
	physx::PxTransform transform,
	physx::PxMaterial *material,
	bool isExclusive,
	physx::PxShapeFlags shapeFlags
) {
	physx::PxBoxGeometry geometry = physx::PxBoxGeometry(size / 2.0);
	physx::PxShape *shape = physics->createShape(geometry, *material, isExclusive, shapeFlags);
	shape->setLocalPose(transform);
	return shape;
}

physx::PxShape *PhysicsEngine::CreateSphereShape(
	float radius,
	physx::PxTransform transform,
	physx::PxMaterial *material,
	bool isExclusive,
	physx::PxShapeFlags shapeFlags
) {
	physx::PxSphereGeometry geometry = physx::PxSphereGeometry(radius);
	physx::PxShape *shape = physics->createShape(geometry, *material, isExclusive, shapeFlags);
	shape->setLocalPose(transform);
	return shape;
}

physx::PxShape *PhysicsEngine::CreateCapsuleShape(
	float radius,
	float size,
	physx::PxTransform transform,
	physx::PxMaterial *material,
	bool isExclusive,
	physx::PxShapeFlags shapeFlags
) {
	physx::PxCapsuleGeometry geometry = physx::PxCapsuleGeometry(radius, size / 2.0f);
	physx::PxShape *shape = physics->createShape(geometry, *material, isExclusive, shapeFlags);
	shape->setLocalPose(transform);
	return shape;
}

physx::PxRigidStatic *PhysicsEngine::AddGround(physx::PxVec3 normal, float distance, physx::PxMaterial *material) {
	physx::PxPlane plane = physx::PxPlane(IS_FLOATS_EQUAL(normal.magnitudeSquared(), 1.0f) ? normal : normal.getNormalized(), distance);
	physx::PxRigidStatic *groundPlane = PxCreatePlane(*physics, plane, *material);
	scene->addActor(*groundPlane);
	return groundPlane;
}

physx::PxRigidStatic *PhysicsEngine::AddStaticActor(physx::PxShape *shape, physx::PxTransform transform) {
	physx::PxRigidStatic *actor = physics->createRigidStatic(transform);
	actor->attachShape(*shape);
	scene->addActor(*actor);
	return actor;
}

physx::PxRigidDynamic *PhysicsEngine::AddDynamicActor(physx::PxShape *shape, physx::PxTransform transform, float density) {
	physx::PxRigidDynamic *actor = physics->createRigidDynamic(transform);
	actor->attachShape(*shape);
	physx::PxRigidBodyExt::updateMassAndInertia(*actor, density);
	scene->addActor(*actor);
	return actor;
}

physx::PxRigidStatic *PhysicsEngine::AddStaticActor(std::vector<physx::PxShape *> shapes, physx::PxTransform transform) {
	physx::PxRigidStatic *actor = physics->createRigidStatic(transform);
	for (auto *shape : shapes) {
		actor->attachShape(*shape);
	}
	scene->addActor(*actor);
	return actor;
}

physx::PxRigidDynamic *PhysicsEngine::AddDynamicActor(std::vector<physx::PxShape *> shapes, physx::PxTransform transform, float density) {
	physx::PxRigidDynamic *actor = physics->createRigidDynamic(transform);
	for (auto *shape : shapes) {
		actor->attachShape(*shape);
	}
	physx::PxRigidBodyExt::updateMassAndInertia(*actor, density);
	scene->addActor(*actor);
	return actor;
}

std::vector<physx::PxRigidActor *> PhysicsEngine::GetActors(physx::PxActorTypeFlags types) {
	uint32_t actorsNum = scene->getNbActors(types);
	std::vector<physx::PxRigidActor *> actors(actorsNum);
	scene->getActors(types, reinterpret_cast<physx::PxActor **>(actors.data()), actorsNum);
	return actors;
}

void PhysicsEngine::RemoveActor(physx::PxActor *actor) {
	scene->removeActor(*actor);
}

void PhysicsEngine::MarkActor(physx::PxActor *actor) {
	if (actor) {
		markedActors.insert(actor);
	}
}
