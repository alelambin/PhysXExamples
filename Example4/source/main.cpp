#include <iostream>
#include "PhysicsEngine.h"
#include "snippetrender/SnippetRender.h"
#include "snippetrender/SnippetCamera.h"

PhysicsEngine *physicsEngine;
Snippets::Camera *camera;

void keyPressedCallback(unsigned char key, const physx::PxTransform &cameraTransform) {
	std::cout << "\"" << key << "\", " << toupper(key) << '\n';
	switch (toupper(key)) {
	case ' ':
	{
		static float projectileVelocity = 100.0f;
		static float projectileRadius = 0.25f;
		static physx::PxMaterial *projectileMaterial = physicsEngine->CreateMaterial(0.1f, 0.1f, 0.7f);
		static physx::PxShape *projectileShape = physicsEngine->CreateSphereShape(projectileRadius, projectileMaterial, CustomFilterData::eDYNAMIC);
		physx::PxRigidDynamic *projectileActor = physicsEngine->AddDynamicActor(projectileShape, cameraTransform.p, physx::PxQuat(1.0f), 300.0f);
		projectileActor->setLinearVelocity(camera->getDir() * projectileVelocity);
	}
	break;
	default:
		break;
	}
}

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

	physx::PxMaterial *defaultMaterial = physicsEngine->CreateMaterial(0.5f, 0.5f, 0.1f);

	const physx::PxVec3 boxSize = physx::PxVec3(1.0f, 1.0f, 1.0f);
	const float boxDynamicDensity = 300.0f;
	const physx::PxVec3 jointOffset = physx::PxVec3(2.0f, 0.0f, 0.0f);
	physx::PxShape *boxShape = physicsEngine->CreateBoxShape(boxSize, defaultMaterial, CustomFilterData::eDYNAMIC);
	{
		const physx::PxVec3 jointPosition = physx::PxVec3(-15.5f, 0.0f, 0.0f);
		const physx::PxVec3 boxStaticPosition = jointPosition - jointOffset;
		const physx::PxVec3 boxDynamicPosition = jointPosition + jointOffset;
		physx::PxRigidStatic *boxStatic = physicsEngine->AddStaticActor(boxShape, boxStaticPosition, physx::PxQuat(1.0f));
		physx::PxRigidDynamic *boxDynamic = physicsEngine->AddDynamicActor(boxShape, boxDynamicPosition, physx::PxQuat(1.0f), boxDynamicDensity);

		PhysicsEngine::FixedJointDesc desc;
		memset(&desc, 0, sizeof(desc));
		desc.actor0 = boxStatic;
		desc.transform0 = physx::PxTransform(jointOffset);
		desc.actor1 = boxDynamic;
		desc.transform1 = physx::PxTransform(-jointOffset);
		desc.visualizationEnabled = true;
		desc.isBreakable = true;
		desc.forceThreshold = 10000.0f;
		desc.torqueThreshold = 10000.0f;
		physicsEngine->CreateFixedJoint(desc);
	}
	{
		const physx::PxVec3 jointPosition = physx::PxVec3(-9.0f, 0.0f, 0.0f);
		const physx::PxVec3 boxStaticPosition = jointPosition - jointOffset;
		const physx::PxVec3 boxDynamicPosition = jointPosition + jointOffset;
		physx::PxRigidStatic *boxStatic = physicsEngine->AddStaticActor(boxShape, boxStaticPosition, physx::PxQuat(1.0f));
		physx::PxRigidDynamic *boxDynamic = physicsEngine->AddDynamicActor(boxShape, boxDynamicPosition, physx::PxQuat(1.0f), boxDynamicDensity);

		PhysicsEngine::SphericalJointDesc desc;
		memset(&desc, 0, sizeof(desc));
		desc.actor0 = boxStatic;
		desc.transform0 = physx::PxTransform(jointOffset);
		desc.actor1 = boxDynamic;
		desc.transform1 = physx::PxTransform(-jointOffset);
		desc.collisionEnabled = true;
		desc.visualizationEnabled = true;
		desc.limitEnabled = true;
		desc.yLimitAngle = physx::PxPiDivFour;
		desc.zLimitAngle = physx::PxPiDivTwo;
		physicsEngine->CreateSphericalJoint(desc);
	}
	{
		const physx::PxVec3 jointPosition = physx::PxVec3(-3.0f, 0.0f, 0.0f);
		const physx::PxQuat jointRotation = physx::PxQuat(physx::PxPiDivTwo, physx::PxVec3(0.0f, 0.0f, 1.0f));
		const physx::PxVec3 boxStaticPosition = jointPosition - jointOffset;
		const physx::PxVec3 boxDynamicPosition = jointPosition + jointOffset;
		physx::PxRigidStatic *boxStatic = physicsEngine->AddStaticActor(boxShape, boxStaticPosition, physx::PxQuat(1.0f));
		physx::PxRigidDynamic *boxDynamic = physicsEngine->AddDynamicActor(boxShape, boxDynamicPosition, physx::PxQuat(1.0f), boxDynamicDensity);

		PhysicsEngine::RevoluteJointDesc desc;
		memset(&desc, 0, sizeof(desc));
		desc.actor0 = boxStatic;
		desc.transform0 = physx::PxTransform(jointOffset, jointRotation);
		desc.actor1 = boxDynamic;
		desc.transform1 = physx::PxTransform(-jointOffset, jointRotation);
		desc.collisionEnabled = true;
		desc.visualizationEnabled = true;
		desc.limitEnabled = true;
		desc.lowerLimit = -physx::PxPiDivTwo;
		desc.upperLimit = physx::PxPiDivTwo;
		physicsEngine->CreateRevoluteJoint(desc);
	}
	{
		const physx::PxVec3 jointPosition = physx::PxVec3(3.0f, 0.0f, 0.0f);
		const physx::PxVec3 boxStaticPosition = jointPosition - jointOffset;
		const physx::PxVec3 boxDynamicPosition = jointPosition + jointOffset;
		physx::PxRigidStatic *boxStatic = physicsEngine->AddStaticActor(boxShape, boxStaticPosition, physx::PxQuat(1.0f));
		physx::PxRigidDynamic *boxDynamic = physicsEngine->AddDynamicActor(boxShape, boxDynamicPosition, physx::PxQuat(1.0f), boxDynamicDensity);

		PhysicsEngine::PrismaticJointDesc desc;
		memset(&desc, 0, sizeof(desc));
		desc.actor0 = boxStatic;
		desc.transform0 = physx::PxTransform(jointOffset);
		desc.actor1 = boxDynamic;
		desc.transform1 = physx::PxTransform(-jointOffset);
		desc.collisionEnabled = true;
		desc.visualizationEnabled = true;
		desc.limitEnabled = true;
		desc.lowerLimit = -2.0f;
		desc.upperLimit = 0.0f;
		physicsEngine->CreatePrismaticJoint(desc);
	}
	{
		const physx::PxVec3 jointPosition = physx::PxVec3(9.0f, 0.0f, 0.0f);
		const physx::PxVec3 boxStaticPosition = jointPosition - jointOffset;
		const physx::PxVec3 boxDynamicPosition = jointPosition + jointOffset;
		physx::PxRigidStatic *boxStatic = physicsEngine->AddStaticActor(boxShape, boxStaticPosition, physx::PxQuat(1.0f));
		physx::PxRigidDynamic *boxDynamic = physicsEngine->AddDynamicActor(boxShape, boxDynamicPosition, physx::PxQuat(1.0f), boxDynamicDensity);

		PhysicsEngine::DistanceJointDesc desc;
		memset(&desc, 0, sizeof(desc));
		desc.actor0 = boxStatic;
		desc.transform0 = physx::PxTransform(jointOffset);
		desc.actor1 = boxDynamic;
		desc.transform1 = physx::PxTransform(-jointOffset);
		desc.collisionEnabled = true;
		desc.visualizationEnabled = true;
		desc.minDistanceEnabled = true;
		desc.minDistance = 0.0f;
		desc.maxDistanceEnabled = true;
		desc.maxDistance = 2.0f;
		physicsEngine->CreateDistanceJoint(desc);
	}
	{
		const physx::PxVec3 jointPosition = physx::PxVec3(15.0f, 0.0f, 0.0f);
		const physx::PxVec3 boxStaticPosition = jointPosition - jointOffset;
		const physx::PxVec3 boxDynamicPosition = jointPosition + jointOffset;
		physx::PxRigidStatic *boxStatic = physicsEngine->AddStaticActor(boxShape, boxStaticPosition, physx::PxQuat(1.0f));
		physx::PxRigidDynamic *boxDynamic = physicsEngine->AddDynamicActor(boxShape, boxDynamicPosition, physx::PxQuat(1.0f), boxDynamicDensity);

		PhysicsEngine::D6JointDesc desc;
		memset(&desc, 0, sizeof(desc));
		desc.actor0 = boxStatic;
		desc.transform0 = physx::PxTransform(jointOffset);
		desc.actor1 = boxDynamic;
		desc.transform1 = physx::PxTransform(-jointOffset);
		desc.collisionEnabled = true;
		desc.visualizationEnabled = true;
		desc.motions[physx::PxD6Axis::eX] = physx::PxD6Motion::eLIMITED;
		desc.motions[physx::PxD6Axis::eY] = physx::PxD6Motion::eLOCKED;
		desc.motions[physx::PxD6Axis::eZ] = physx::PxD6Motion::eLOCKED;
		desc.motions[physx::PxD6Axis::eTWIST] = physx::PxD6Motion::eFREE;
		desc.motions[physx::PxD6Axis::eSWING1] = physx::PxD6Motion::eLOCKED;
		desc.motions[physx::PxD6Axis::eSWING2] = physx::PxD6Motion::eLOCKED;
		desc.xLowerLimit = -2.0f;
		desc.xUpperLimit = 1.0f;
		physicsEngine->CreateD6Joint(desc);
	}

	glutMainLoop();

	return 0;
}
