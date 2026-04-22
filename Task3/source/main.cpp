#include <iostream>
#include <vector>
#include "snippetrender/SnippetRender.h"
#include "snippetrender/SnippetCamera.h"

#ifdef _DEBUG
#define USE_PVD
#define PVD_HOST "127.0.0.1"
#endif

physx::PxDefaultAllocator allocatorCallback;
physx::PxDefaultErrorCallback errorCallback;
physx::PxFoundation *foundation = nullptr;
physx::PxPhysics *physics = nullptr;
#ifdef USE_PVD
physx::PxPvd *pvd = nullptr;
physx::PxPvdTransport *transport = nullptr;
#endif
physx::PxScene *scene = nullptr;
Snippets::Camera *camera = nullptr;

void fillScene() {}

void keyPressedCallback(unsigned char key, const physx::PxTransform &cameraTransform) {}

void renderCallback() {
	scene->simulate(1.0f / 60.0f);
	scene->fetchResults(true);

	float nearClip = 0.1f;
	float farClip = 10000.0f;
	Snippets::startRender(camera, nearClip, farClip);

	uint32_t actorsNum = scene->getNbActors(physx::PxActorTypeFlag::eRIGID_STATIC | physx::PxActorTypeFlag::eRIGID_DYNAMIC);
	std::vector<physx::PxRigidActor *> actors(actorsNum);
	scene->getActors(physx::PxActorTypeFlag::eRIGID_STATIC | physx::PxActorTypeFlag::eRIGID_DYNAMIC, reinterpret_cast<physx::PxActor **>(actors.data()), actorsNum);
	if (actors.size() > 0) {
		Snippets::renderActors(actors.data(), actors.size());
	}

	Snippets::finishRender();
}

void exitCallback() {
	delete camera;

	uint32_t actorsNum = scene->getNbActors(physx::PxActorTypeFlag::eRIGID_STATIC | physx::PxActorTypeFlag::eRIGID_DYNAMIC);
	std::vector<physx::PxRigidActor *> actors(actorsNum);
	scene->getActors(physx::PxActorTypeFlag::eRIGID_STATIC | physx::PxActorTypeFlag::eRIGID_DYNAMIC, reinterpret_cast<physx::PxActor **>(actors.data()), actorsNum);
	for (auto *actor : actors) {
		uint32_t shapesNum = actor->getNbShapes();
		if (shapesNum > 0) {
			std::vector<physx::PxShape *> shapes(shapesNum);
			actor->getShapes(reinterpret_cast<physx::PxShape **>(shapes.data()), shapesNum);
			for (auto *shape : shapes) {
				shape->release();
				shape = nullptr;
			}
		}
		actor->release();
	}

	uint32_t materialsNum = physics->getNbMaterials();
	if (materialsNum > 0) {
		std::vector<physx::PxMaterial *> materials(materialsNum);
		physics->getMaterials(reinterpret_cast<physx::PxMaterial **>(materials.data()), materialsNum);
		for (auto *material : materials) {
			material->release();
			material = nullptr;
		}
	}

	scene->release();
	physics->release();
#ifdef USE_PVD
	if (pvd && pvd->isConnected()) {
		pvd->disconnect();
	}
	transport->release();
#endif
	foundation->release();
}

int main() {
	camera = new Snippets::Camera(physx::PxVec3(0.0f, 10.0f, 30.0f), physx::PxVec3(0.0f, -0.1f, -0.3f));
	Snippets::setupDefault("PhysX Example", camera, keyPressedCallback, renderCallback, exitCallback);

	foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocatorCallback, errorCallback);

#ifdef USE_PVD
	pvd = physx::PxCreatePvd(*foundation);
	transport = physx::PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10000);
	if (!pvd || !transport || !pvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL)) {
		std::cout << "[WARNING] Could not initialize or connect pvd\n";
	}
#endif
	physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, physx::PxTolerancesScale(), true, pvd);

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
	
	fillScene();

	glutMainLoop();

	return 0;
}
