#include "Core.hpp"
#include "Engine.hpp"
#include "Entity.hpp"
#include "Transform.hpp"
#include "JoltPhysics.hpp"
#include "OpenGL.hpp"

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
using namespace ES::Plugin;

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
		auto &transform = core.GetRegistry().get<ES::Plugin::Object::Component::Transform>(sphereEntity);

		auto &physicsManager = core.GetResource<ES::Plugin::Physics::Resource::PhysicsManager>();
		auto &physicsSystem = physicsManager.GetPhysicsSystem();
		auto &bodyInterface = physicsSystem.GetBodyInterface();

		auto position = transform.getPosition();
		Vec3 velocity = bodyInterface.GetLinearVelocity(rigidBody.body->GetID());

		std::cout << "Position = (" << position.x << ", " << position.y << ", " << position.z << "), Velocity = (" << velocity.GetX() << ", " << velocity.GetY() << ", " << velocity.GetZ() << ")" << std::endl;
	}
};

bool SphereIsActive(ES::Engine::Core &core, const ES::Engine::Entity &sphereEntity)
{
	auto &rigidBody = core.GetRegistry().get<ES::Plugin::Physics::Component::RigidBody3D>(sphereEntity);

	auto &physicsManager = core.GetResource<ES::Plugin::Physics::Resource::PhysicsManager>();
	auto &physicsSystem = physicsManager.GetPhysicsSystem();
	auto &bodyInterface = physicsSystem.GetBodyInterface();

	if (rigidBody.body == nullptr)
	{
		return true; // Needs initialization so we want to run
	}

	return bodyInterface.IsActive(rigidBody.body->GetID());
}

void SetupOpenGL(ES::Engine::Core &core)
{
	core.RegisterResource<OpenGL::Resource::Buttons>({});

    core.RegisterSystem<ES::Engine::Scheduler::Startup>(OpenGL::System::InitGLFW);
    core.RegisterSystem<ES::Engine::Scheduler::Startup>(OpenGL::System::SetupGLFWHints);
    core.RegisterSystem<ES::Engine::Scheduler::Startup>(OpenGL::System::CreateGLFWWindow);
    core.RegisterSystem<ES::Engine::Scheduler::Startup>(OpenGL::System::LinkGLFWContextToGL);
    core.RegisterSystem<ES::Engine::Scheduler::Startup>(OpenGL::System::InitGLEW);
    core.RegisterSystem<ES::Engine::Scheduler::Startup>(OpenGL::System::CheckGLEWVersion);
    core.RegisterSystem<ES::Engine::Scheduler::Startup>(OpenGL::System::GLFWEnableVSync);
    core.RegisterSystem<ES::Engine::Scheduler::Startup>(OpenGL::System::SetupGLFWHints);
    core.RegisterSystem<ES::Engine::Scheduler::Startup>(OpenGL::System::LoadMaterialCache);
    core.RegisterSystem<ES::Engine::Scheduler::Startup>(OpenGL::System::LoadShaderManager);
    core.RegisterSystem<ES::Engine::Scheduler::Startup>(OpenGL::System::CreateCamera);
    core.RegisterSystem<ES::Engine::Scheduler::Startup>(OpenGL::System::SetupShaderUniforms);

    core.RunSystems();

    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::UpdateKey);
    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::UpdatePosCursor);
    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::UpdateButton);
    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::SaveLastMousePos);
    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::UpdateMatrices);
    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::GLClearColor);
    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::GLClearDepth);
    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::GLEnableDepth);
    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::GLEnableCullFace);
    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::SetupCamera);
    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::SetupLights);
    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::RenderMeshes);
    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::SwapBuffers);
    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::PollEvents);
    core.RegisterSystem<ES::Engine::Scheduler::Update>(OpenGL::System::MouseDragging);
}

ES::Engine::Entity CreateSphere(ES::Engine::Core &core)
{
	// Create the settings for the collision volume (the shape).
	// Note that for simple shapes (like boxes) you can also directly construct a BoxShape.
	constexpr float radius = .5f;

	std::shared_ptr<SphereShapeSettings> sphere_shape_settings = std::make_shared<SphereShapeSettings>(radius);

	// Create the shape
	ES::Engine::Entity sphere = core.CreateEntity();
	sphere.AddComponent<ES::Plugin::Object::Component::Transform>(core, ES::Plugin::Object::Component::Transform({0.0f, 30.0f, 0.0f}));
	sphere.AddComponent<ES::Plugin::Physics::Component::RigidBody3D>(core, ES::Plugin::Physics::Component::RigidBody3D(sphere_shape_settings, EMotionType::Dynamic, Physics::Utils::Layers::MOVING));

	// Add a mesh to it for rendering
	OpenGL::Component::Model model;

	model.shaderName = "default";
    model.materialName = "default";

    OpenGL::Utils::Mesh mesh;

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec<3, unsigned int>> triIndices;

	// Generate a sphere
	const int numSegments = 16;
	const int numRings = 16;

	// Generate vertices and normals

	const float pi = glm::pi<float>();

	for (int i = 0; i <= numRings; ++i) {
		float phi = pi * (static_cast<float>(i) / numRings); // Latitude angle from 0 to pi

		for (int j = 0; j <= numSegments; ++j) {
			float theta = 2.0f * pi * (static_cast<float>(j) / numSegments); // Longitude angle from 0 to 2*pi

			// Spherical coordinates
			float x = radius * sin(phi) * cos(theta);
			float y = radius * cos(phi);
			float z = radius * sin(phi) * sin(theta);

			vertices.push_back(glm::vec3(x, y, z));
			normals.push_back(glm::normalize(glm::vec3(x, y, z)));
		}
	}

	// Generate indices for triangle strips
	for (int i = 0; i < numRings; ++i) {
		for (int j = 0; j < numSegments; ++j) {
			unsigned int i0 = i * (numSegments + 1) + j;
			unsigned int i1 = (i + 1) * (numSegments + 1) + j;
			unsigned int i2 = (i + 1) * (numSegments + 1) + (j + 1);
			unsigned int i3 = i * (numSegments + 1) + (j + 1);

			// First triangle
			triIndices.push_back(glm::uvec3(i0, i1, i2));

			// Second triangle
			triIndices.push_back(glm::uvec3(i0, i2, i3));
		}
	}

	mesh.vertices = vertices;
    mesh.normals = normals;
    mesh.triIndices = triIndices;
    mesh.generateGlBuffers();

    model.mesh = mesh;

    sphere.AddComponent<OpenGL::Component::Model>(core, model);

	return sphere;
}

ES::Engine::Entity CreateFloor(ES::Engine::Core &core)
{
	glm::vec3 floor_position = glm::vec3(0.0f, -3.0f, 0.0f);
	glm::vec3 floor_scale = glm::vec3(1.0f, 1.0f, 1.0f);

	// Rotated slightly in a 12° angle, so that the sphere will roll on it
	glm::quat floor_rotation = glm::rotate(glm::quat(0.0f, 0.0f, 0.0f, 1.0f), glm::radians(12.0f), glm::vec3(1.0f, 0.0f, 0.0f));

	Vec3 floor_size = Vec3(10.0f, 1.0f, 10.0f);

	std::shared_ptr<BoxShapeSettings> floor_shape_settings = std::make_shared<BoxShapeSettings>(floor_size);
	ES::Engine::Entity floor = core.CreateEntity();
	
	floor.AddComponent<ES::Plugin::Object::Component::Transform>(core, ES::Plugin::Object::Component::Transform(floor_position, floor_scale, floor_rotation));
	floor.AddComponent<ES::Plugin::Physics::Component::RigidBody3D>(core, ES::Plugin::Physics::Component::RigidBody3D(floor_shape_settings, EMotionType::Static, Physics::Utils::Layers::NON_MOVING));

	// Add a mesh to it for rendering
	OpenGL::Component::Model model;

	model.shaderName = "default";
    model.materialName = "default";

    OpenGL::Utils::Mesh mesh;

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec<3, unsigned int>> triIndices;

	// Generate a box
	const float width = floor_size.GetX();
	const float height = floor_size.GetY();
	const float depth = floor_size.GetZ();

	// Generate vertices and normals
	const glm::vec3 front_bottom_left = glm::vec3(-width, -height, -depth);
	const glm::vec3 front_bottom_right = glm::vec3(width, -height, -depth);
	const glm::vec3 front_top_left = glm::vec3(-width, height, -depth);
	const glm::vec3 front_top_right = glm::vec3(width, height, -depth);
	const glm::vec3 back_bottom_left = glm::vec3(-width, -height, depth);
	const glm::vec3 back_bottom_right = glm::vec3(width, -height, depth);
	const glm::vec3 back_top_left = glm::vec3(-width, height, depth);
	const glm::vec3 back_top_right = glm::vec3(width, height, depth);

	// Front face
	vertices.push_back(front_bottom_left);
	vertices.push_back(front_bottom_right);
	vertices.push_back(front_top_left);
	vertices.push_back(front_top_right);

	normals.push_back(glm::vec3(0.0f, 0.0f, -1.0f));
	normals.push_back(glm::vec3(0.0f, 0.0f, -1.0f));
	normals.push_back(glm::vec3(0.0f, 0.0f, -1.0f));
	normals.push_back(glm::vec3(0.0f, 0.0f, -1.0f));

	// Back face
	vertices.push_back(back_bottom_left);
	vertices.push_back(back_bottom_right);
	vertices.push_back(back_top_left);
	vertices.push_back(back_top_right);

	normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
	normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
	normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
	normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));

	// Top face
	vertices.push_back(front_top_left);
	vertices.push_back(front_top_right);
	vertices.push_back(back_top_left);
	vertices.push_back(back_top_right);

	normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
	normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
	normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
	normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));

	// Bottom face
	vertices.push_back(front_bottom_left);
	vertices.push_back(front_bottom_right);
	vertices.push_back(back_bottom_left);
	vertices.push_back(back_bottom_right);

	normals.push_back(glm::vec3(0.0f, -1.0f, 0.0f));
	normals.push_back(glm::vec3(0.0f, -1.0f, 0.0f));
	normals.push_back(glm::vec3(0.0f, -1.0f, 0.0f));
	normals.push_back(glm::vec3(0.0f, -1.0f, 0.0f));

	// Left face
	vertices.push_back(front_bottom_left);
	vertices.push_back(front_top_left);
	vertices.push_back(back_bottom_left);
	vertices.push_back(back_top_left);

	normals.push_back(glm::vec3(-1.0f, 0.0f, 0.0f));
	normals.push_back(glm::vec3(-1.0f, 0.0f, 0.0f));
	normals.push_back(glm::vec3(-1.0f, 0.0f, 0.0f));
	normals.push_back(glm::vec3(-1.0f, 0.0f, 0.0f));

	// Right face
	vertices.push_back(front_bottom_right);
	vertices.push_back(front_top_right);
	vertices.push_back(back_bottom_right);
	vertices.push_back(back_top_right);

	normals.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
	normals.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
	normals.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
	normals.push_back(glm::vec3(1.0f, 0.0f, 0.0f));

	// Generate indices for triangle strips
	triIndices.push_back(glm::uvec3(0, 1, 2));
	triIndices.push_back(glm::uvec3(1, 3, 2));

	triIndices.push_back(glm::uvec3(4, 5, 6));
	triIndices.push_back(glm::uvec3(5, 7, 6));

	triIndices.push_back(glm::uvec3(8, 9, 10));
	triIndices.push_back(glm::uvec3(9, 11, 10));

	triIndices.push_back(glm::uvec3(12, 13, 14));
	triIndices.push_back(glm::uvec3(13, 15, 14));

	triIndices.push_back(glm::uvec3(16, 17, 18));
	triIndices.push_back(glm::uvec3(17, 19, 18));

	triIndices.push_back(glm::uvec3(20, 21, 22));
	triIndices.push_back(glm::uvec3(21, 23, 22));

	mesh.vertices = vertices;
    mesh.normals = normals;
    mesh.triIndices = triIndices;
    mesh.generateGlBuffers();

    model.mesh = mesh;

    floor.AddComponent<OpenGL::Component::Model>(core, model);

	return floor;
}

int main(void)
{
    ES::Engine::Core core;

	SetupOpenGL(core);

    core.RegisterResource<Physics::Resource::PhysicsManager>(std::move(Physics::Resource::PhysicsManager()));

	core.RegisterSystem(ES::Plugin::Physics::System::PhysicsUpdate);

	// Next we can create a rigid body to serve as the floor, we make a large box
	ES::Engine::Entity floor = CreateFloor(core);

	// Now create a dynamic body to bounce on the floor
	ES::Engine::Entity sphere = CreateSphere(core);

	// Now that we know which entity is the sphere, we can create its linked system
	// Note that this is for testing purposes only
	core.RegisterSystem(InitSphereSystem{ sphere });
	core.RegisterSystem(PrintSphereInfoSystem{ sphere });

	// Optional step: Before starting the physics simulation you can optimize the broad phase. This improves collision detection performance (it's pointless here because we only have 2 bodies).
	// You should definitely not call this every frame or when e.g. streaming in a new level section as it is an expensive operation.
	// Instead insert all new objects in batches instead of 1 at a time to keep the broad phase efficient.
	core.GetResource<Physics::Resource::PhysicsManager>().GetPhysicsSystem().OptimizeBroadPhase();

	// Now we're ready to simulate the body
	core.GetResource<Physics::Resource::PhysicsManager>().SetCollisionSteps(10);
	while (!glfwWindowShouldClose(core.GetResource<OpenGL::Resource::GLFWWindow>().window)) {
        core.RunSystems();
    }

    glfwDestroyWindow(core.GetResource<OpenGL::Resource::GLFWWindow>().window);
    glfwTerminate();

    return 0;
}