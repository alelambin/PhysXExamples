#include <iostream>
#include "PhysicsEngine.h"
#include "snippetrender/SnippetRender.h"
#include "snippetrender/SnippetCamera.h"

PhysicsEngine *physicsEngine;
Snippets::Camera *camera;

void keyPressedCallback(unsigned char key, const physx::PxTransform &cameraTransform) {
	std::cout << "\"" << key << "\", " << toupper(key) << '\n';
}

void renderCallback() {
	physicsEngine->Simulate(1.0f / 60.0f);

	Snippets::startRender(camera);

	std::vector<physx::PxRigidActor *> actors = physicsEngine->GetActors();
	if (actors.size() > 0) {
		Snippets::renderActors(actors.data(), actors.size());
	}

	Snippets::finishRender();
}

void exitCallback() {
	delete camera;
	delete physicsEngine;
}

int main() {
	camera = new Snippets::Camera(physx::PxVec3(0.0f, 10.0f, 30.0f), physx::PxVec3(0.0f, -0.1f, -0.3f));
	Snippets::setupDefault("PhysX Example", camera, keyPressedCallback, renderCallback, exitCallback);

	physicsEngine = new PhysicsEngine();

	physx::PxMaterial *woodMaterial = physicsEngine->CreateMaterial(0.5f, 0.4f, 0.2f);
	physx::PxMaterial *iceMaterial = physicsEngine->CreateMaterial(0.1f, 0.03f, 0.05f);
	physx::PxMaterial *rubberMaterial = physicsEngine->CreateMaterial(0.9f, 0.8f, 0.7f);
	const float woodDensity = 500.0f;
	const float iceDensity = 917.0f;
	const float rubberDensity = 1200.0f;

	physicsEngine->AddGround(physx::PxVec3(0.4f, 1.0f, 0.0f), 0.0f, woodMaterial);

	physx::PxQuat identity = physx::PxQuat(1.0f);
	physx::PxQuat rotation = physx::PxQuat(physx::PxPiDivTwo, physx::PxVec3(0.0f, 1.0f, 0.0f));

	const physx::PxVec3 boxSize = physx::PxVec3(1.0f);
	const physx::PxVec3 woodBoxPosition = physx::PxVec3(-5.0f, 3.0f, -5.0f);
	const physx::PxVec3 iceBoxPosition = physx::PxVec3(-5.0f, 3.0f, 0.0f);
	const physx::PxVec3 rubberBoxPosition = physx::PxVec3(-5.0f, 3.0f, 5.0f);
	physx::PxShape *woodBoxShape = physicsEngine->CreateBoxShape(boxSize, woodMaterial);
	physx::PxShape *iceBoxShape = physicsEngine->CreateBoxShape(boxSize, iceMaterial);
	physx::PxShape *rubberBoxShape = physicsEngine->CreateBoxShape(boxSize, rubberMaterial);
	physicsEngine->AddDynamicActor(woodBoxShape, woodBoxPosition, identity, woodDensity);
	physicsEngine->AddDynamicActor(iceBoxShape, iceBoxPosition, identity, iceDensity);
	physicsEngine->AddDynamicActor(rubberBoxShape, rubberBoxPosition, identity, rubberDensity);

	const float sphereRadius = 0.5f;
	const physx::PxVec3 woodSpherePosition = physx::PxVec3(5.0f, 3.0f, -5.0f);
	const physx::PxVec3 iceSpherePosition = physx::PxVec3(5.0f, 3.0f, 0.0f);
	const physx::PxVec3 rubberSpherePosition = physx::PxVec3(5.0f, 3.0f, 5.0f);
	physx::PxShape *woodSphereShape = physicsEngine->CreateSphereShape(sphereRadius, woodMaterial);
	physx::PxShape *iceSphereShape = physicsEngine->CreateSphereShape(sphereRadius, iceMaterial);
	physx::PxShape *rubberSphereShape = physicsEngine->CreateSphereShape(sphereRadius, rubberMaterial);
	physicsEngine->AddDynamicActor(woodSphereShape, woodSpherePosition, identity, woodDensity);
	physicsEngine->AddDynamicActor(iceSphereShape, iceSpherePosition, identity, iceDensity);
	physicsEngine->AddDynamicActor(rubberSphereShape, rubberSpherePosition, identity, rubberDensity);

	const float capsuleRadius = 0.25f;
	const float capsuleSize = 0.5f;
	const physx::PxVec3 woodCapsulePosition = physx::PxVec3(0.0f, 3.0f, -5.0f);
	const physx::PxVec3 iceCapsulePosition = physx::PxVec3(0.0f, 3.0f, 0.0f);
	const physx::PxVec3 rubberCapsulePosition = physx::PxVec3(0.0f, 3.0f, 5.0f);
	physx::PxShape *woodCapsuleShape = physicsEngine->CreateCapsuleShape(capsuleRadius, capsuleSize, woodMaterial);
	physx::PxShape *iceCapsuleShape = physicsEngine->CreateCapsuleShape(capsuleRadius, capsuleSize, iceMaterial);
	physx::PxShape *rubberCapsuleShape = physicsEngine->CreateCapsuleShape(capsuleRadius, capsuleSize, rubberMaterial);
	physicsEngine->AddDynamicActor(woodCapsuleShape, woodCapsulePosition, rotation, woodDensity);
	physicsEngine->AddDynamicActor(iceCapsuleShape, iceCapsulePosition, rotation, iceDensity);
	physicsEngine->AddDynamicActor(rubberCapsuleShape, rubberCapsulePosition, rotation, rubberDensity);

	glutMainLoop();

	return 0;
}
