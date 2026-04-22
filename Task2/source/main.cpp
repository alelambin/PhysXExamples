#include <iostream>
#include "PhysicsEngine.h"
#include "snippetrender/SnippetRender.h"
#include "snippetrender/SnippetCamera.h"

PhysicsEngine *physicsEngine;
Snippets::Camera *camera;

void keyPressedCallback(unsigned char key, const physx::PxTransform &cameraTransform) {}

void renderCallback() {
	physicsEngine->Simulate(1.0f / 60.0f);

	float nearClip = 0.1f;
	float farClip = 10000.0f;
	Snippets::startRender(camera, nearClip, farClip);

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

	physx::PxMaterial *material = physicsEngine->CreateMaterial(0.5f, 0.5f, 0.1f);

	const physx::PxVec3 groundNormal = physx::PxVec3(0.0f, -1.0f, 0.0f);
	const float groundDistance = 0.0f;
	physicsEngine->AddGround(groundNormal, groundDistance, material);

	const float chairDensity = 300.0f;
	const physx::PxVec3 chairLegSize = physx::PxVec3(0.2f, 1.4f, 0.2f);
	const physx::PxVec3 chairSeatSize = physx::PxVec3(1.4f, 0.2f, 1.4f);
	const physx::PxVec3 chairBackSize = physx::PxVec3(1.4f, 1.4f, 0.2f);
	std::vector<physx::PxShape *> chairShapes = {
		physicsEngine->CreateBoxShape( chairLegSize, physx::PxTransform(physx::PxVec3( 0.6f, -0.7f,  0.6f)), material, true),
		physicsEngine->CreateBoxShape( chairLegSize, physx::PxTransform(physx::PxVec3( 0.6f, -0.7f, -0.6f)), material, true),
		physicsEngine->CreateBoxShape( chairLegSize, physx::PxTransform(physx::PxVec3(-0.6f, -0.7f,  0.6f)), material, true),
		physicsEngine->CreateBoxShape( chairLegSize, physx::PxTransform(physx::PxVec3(-0.6f, -0.7f, -0.6f)), material, true),
		physicsEngine->CreateBoxShape(chairSeatSize, physx::PxTransform(physx::PxVec3( 0.0f,  0.0f,  0.0f)), material, true),
		physicsEngine->CreateBoxShape(chairBackSize, physx::PxTransform(physx::PxVec3( 0.0f,  0.7f, -0.6f)), material, true)
	};

	std::vector<physx::PxTransform> chairTransforms = {
		physx::PxTransform(physx::PxVec3( 0.0f, 9.0f,  0.0f)),
		physx::PxTransform(physx::PxVec3(-2.0f, 9.0f,  0.0f)),
		physx::PxTransform(physx::PxVec3(-2.0f, 9.0f, -2.0f)),
		physx::PxTransform(physx::PxVec3( 0.0f, 9.0f, -2.0f)),
		physx::PxTransform(physx::PxVec3( 2.0f, 9.0f, -2.0f)),
		physx::PxTransform(physx::PxVec3( 2.0f, 9.0f,  0.0f)),
		physx::PxTransform(physx::PxVec3( 2.0f, 9.0f,  2.0f)),
		physx::PxTransform(physx::PxVec3( 0.0f, 9.0f,  2.0f)),
		physx::PxTransform(physx::PxVec3(-2.0f, 9.0f,  2.0f)),
		physx::PxTransform(physx::PxVec3(-1.0f, 6.0f, -1.0f)),
		physx::PxTransform(physx::PxVec3( 1.0f, 6.0f, -1.0f)),
		physx::PxTransform(physx::PxVec3(-1.0f, 6.0f,  1.0f)),
		physx::PxTransform(physx::PxVec3( 1.0f, 6.0f,  1.0f)),
		physx::PxTransform(physx::PxVec3( 0.0f, 3.0f,  0.0f))
	};
	for (physx::PxTransform chairTransform : chairTransforms) {
		physicsEngine->AddDynamicActor(chairShapes, chairTransform, chairDensity);
	}
	
	glutMainLoop();

	return 0;
}
