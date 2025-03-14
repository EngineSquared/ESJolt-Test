#include "Core.hpp"
#include "Entity.hpp"
#include "Transform.hpp"
#include "JoltPhysics.hpp"

#include <iostream>

// Jolt includes
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

using namespace JPH; // NOT RECOMMENDED

using namespace JPH::literals;
using namespace ES::Plugin::Physics;

class InitSphereSystem
{
public:
	ES::Engine::Entity sphereEntity;
	mutable bool gotVelocity = false;

	void operator()(ES::Engine::Core &core) const
	{
		if (gotVelocity)
		{
			return;
		}

		auto &rigidBody = core.GetRegistry().get<ES::Plugin::Physics::Component::RigidBody3D>(sphereEntity);

		if (rigidBody.body == nullptr)
		{
			return;
		}

		auto &physicsManager = core.GetResource<ES::Plugin::Physics::Resource::PhysicsManager>();
		auto &physicsSystem = physicsManager.GetPhysicsSystem();
		auto &bodyInterface = physicsSystem.GetBodyInterface();

		bodyInterface.SetLinearVelocity(rigidBody.body->GetID(), Vec3(0.0f, -5.0f, 0.0f));

		gotVelocity = true;
	}
};

class PrintSphereInfoSystem 
{
public:
	ES::Engine::Entity sphereEntity;

	void operator()(ES::Engine::Core &core) const
	{
		auto &rigidBody = core.GetRegistry().get<ES::Plugin::Physics::Component::RigidBody3D>(sphereEntity);

		auto &physicsManager = core.GetResource<ES::Plugin::Physics::Resource::PhysicsManager>();
		auto &physicsSystem = physicsManager.GetPhysicsSystem();
		auto &bodyInterface = physicsSystem.GetBodyInterface();

		RVec3 position = bodyInterface.GetCenterOfMassPosition(rigidBody.body->GetID());
		Vec3 velocity = bodyInterface.GetLinearVelocity(rigidBody.body->GetID());

		std::cout << "Position = (" << position.GetX() << ", " << position.GetY() << ", " << position.GetZ() << "), Velocity = (" << velocity.GetX() << ", " << velocity.GetY() << ", " << velocity.GetZ() << ")" << std::endl;
	}
};

int main(void)
{
    ES::Engine::Core core;

    core.RegisterResource<Resource::PhysicsManager>(std::move(Resource::PhysicsManager()));

    auto &physics_manager = core.GetResource<Resource::PhysicsManager>();

	core.RegisterSystem(ES::Plugin::Physics::System::PhysicsUpdate);

    auto &physics_system = physics_manager.GetPhysicsSystem();

    // The main way to interact with the bodies in the physics system is through the body interface. There is a locking and a non-locking
	// variant of this. We're going to use the locking version (even though we're not planning to access bodies from multiple threads)
	BodyInterface &body_interface = physics_system.GetBodyInterface();

	// Next we can create a rigid body to serve as the floor, we make a large box
	// Create the settings for the collision volume (the shape).
	// Note that for simple shapes (like boxes) you can also directly construct a BoxShape.
	std::shared_ptr<BoxShapeSettings> floor_shape_settings = std::make_shared<BoxShapeSettings>(Vec3(100.0f, 1.0f, 100.0f));
	
	// Create the shape
	ES::Engine::Entity floor = core.CreateEntity();
	floor.AddComponent<ES::Plugin::Object::Component::Transform>(core, ES::Plugin::Object::Component::Transform({0.0f, -1.0f, 0.0f}));
	floor.AddComponent<ES::Plugin::Physics::Component::RigidBody3D>(core, ES::Plugin::Physics::Component::RigidBody3D(floor_shape_settings, EMotionType::Static, Utils::Layers::NON_MOVING));

	// Now create a dynamic body to bounce on the floor
	std::shared_ptr<SphereShapeSettings> sphere_shape_settings = std::make_shared<SphereShapeSettings>(0.5f);

	ES::Engine::Entity sphere = core.CreateEntity();
	sphere.AddComponent<ES::Plugin::Object::Component::Transform>(core, ES::Plugin::Object::Component::Transform({0.0f, 2.0f, 0.0f}));
	sphere.AddComponent<ES::Plugin::Physics::Component::RigidBody3D>(core, ES::Plugin::Physics::Component::RigidBody3D(sphere_shape_settings, EMotionType::Dynamic, Utils::Layers::MOVING));

	// Now that we know which entity is the sphere, we can create its linked system
	// Note that this is for testing purposes only
	core.RegisterSystem(InitSphereSystem{ sphere });
	core.RegisterSystem(PrintSphereInfoSystem{ sphere });

	// Optional step: Before starting the physics simulation you can optimize the broad phase. This improves collision detection performance (it's pointless here because we only have 2 bodies).
	// You should definitely not call this every frame or when e.g. streaming in a new level section as it is an expensive operation.
	// Instead insert all new objects in batches instead of 1 at a time to keep the broad phase efficient.
	physics_system.OptimizeBroadPhase();

	// Now we're ready to simulate the body, keep simulating until it goes to sleep
	while (true)
	{
		// Step the world
		core.RunSystems();

		// Simulate a framerate of 60 Hz
		std::this_thread::sleep_for(std::chrono::duration<float>(1/60.0f));
	}

    return 0;
}