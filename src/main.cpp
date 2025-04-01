#include "Core.hpp"
#include "Engine.hpp"
#include "Entity.hpp"
#include "Transform.hpp"

#include "JoltPhysics.hpp"
#include "OpenGL.hpp"
#include "Window.hpp"
#include "Camera.hpp"

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

ES::Engine::Entity CreateSphere(ES::Engine::Core &core, bool isSoftBody)
{
    constexpr float radius = 0.5f;

    // Create a more uniform geodesic sphere
    Object::Component::Mesh mesh;

    struct Triangle { int v0, v1, v2; };
    std::vector<glm::vec3> vertices;
    std::vector<Triangle> triangles;

    // Initial icosahedron vertices
    const float t = (1.0f + sqrt(5.0f)) / 2.0f;
    vertices = {
        {-1,  t,  0}, {1,  t,  0}, {-1, -t,  0}, {1, -t,  0},
        {0, -1,  t}, {0,  1,  t}, {0, -1, -t}, {0,  1, -t},
        { t,  0, -1}, { t,  0,  1}, {-t,  0, -1}, {-t,  0,  1}
    };
    
    // Normalize to sphere surface
    for (auto& v : vertices) v = glm::normalize(v) * radius;

    // Initial faces
    triangles = {
        {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
        {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
        {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
        {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
    };

    // Subdivide for higher resolution
    std::unordered_map<uint64_t, int> midpoints;
    auto midpoint = [&](int v1, int v2) -> int {
        uint64_t key = (uint64_t)std::min(v1, v2) << 32 | std::max(v1, v2);
        if (midpoints.count(key)) return midpoints[key];
        glm::vec3 mid = glm::normalize(vertices[v1] + vertices[v2]) * radius;
        int index = vertices.size();
        vertices.push_back(mid);
        midpoints[key] = index;
        return index;
    };

    for (int i = 0; i < 2; ++i) { // 2 levels of subdivision
        std::vector<Triangle> newTriangles;
        for (auto& tri : triangles) {
            int a = midpoint(tri.v0, tri.v1);
            int b = midpoint(tri.v1, tri.v2);
            int c = midpoint(tri.v2, tri.v0);
            newTriangles.push_back({tri.v0, a, c});
            newTriangles.push_back({tri.v1, b, a});
            newTriangles.push_back({tri.v2, c, b});
            newTriangles.push_back({a, b, c});
        }
        triangles = newTriangles;
    }

    // Convert to mesh format
    for (auto& v : vertices) {
        mesh.vertices.push_back(v);
        mesh.normals.push_back(glm::normalize(v));
    }
    for (auto& tri : triangles) {
        mesh.indices.push_back(tri.v0);
        mesh.indices.push_back(tri.v1);
        mesh.indices.push_back(tri.v2);
    }

    // Create entity
    ES::Engine::Entity sphere = core.CreateEntity();
    sphere.AddComponent<ES::Plugin::OpenGL::Component::ShaderHandle>(core, "default");
    sphere.AddComponent<ES::Plugin::OpenGL::Component::MaterialHandle>(core, "default");
    sphere.AddComponent<ES::Plugin::Object::Component::Mesh>(core, mesh);

    if (isSoftBody) {
        sphere.AddComponent<ES::Plugin::Object::Component::Transform>(core, glm::vec3(-7.0f, 6.0f, 0.0f));
        sphere.AddComponent<ES::Plugin::Physics::Component::SoftBody3D>(core);
        sphere.AddComponent<ES::Plugin::OpenGL::Component::ModelHandle>(core, "softSphere");
    } else {
        auto sphere_shape_settings = std::make_shared<SphereShapeSettings>(radius);
        sphere.AddComponent<ES::Plugin::Object::Component::Transform>(core, glm::vec3(7.0f, 30.0f, 0.0f));
        sphere.AddComponent<ES::Plugin::Physics::Component::RigidBody3D>(core, sphere_shape_settings, EMotionType::Dynamic, Physics::Utils::Layers::MOVING);
        sphere.AddComponent<ES::Plugin::OpenGL::Component::ModelHandle>(core, "rigidSphere");
    }

    return sphere;
}

ES::Engine::Entity CreateFloor(ES::Engine::Core &core)
{
	glm::vec3 floor_position = glm::vec3(0.0f, -8.0f, 0.0f);
	glm::vec3 floor_scale = glm::vec3(1.0f, 1.0f, 1.0f);

	// Rotated slightly in a 12° angle, so that the sphere will roll on it
	glm::quat floor_rotation = glm::rotate(glm::quat(0.0f, 0.0f, 0.0f, 1.0f), glm::radians(12.0f), glm::vec3(1.0f, 0.0f, 0.0f));

	Vec3 floor_size = Vec3(10.0f, 1.0f, 10.0f);

	std::shared_ptr<BoxShapeSettings> floor_shape_settings = std::make_shared<BoxShapeSettings>(floor_size);
	ES::Engine::Entity floor = core.CreateEntity();

	floor.AddComponent<ES::Plugin::Object::Component::Transform>(core, floor_position, floor_scale, floor_rotation);
	floor.AddComponent<ES::Plugin::Physics::Component::RigidBody3D>(core, floor_shape_settings, EMotionType::Static, Physics::Utils::Layers::NON_MOVING);

	// Add a mesh to it for rendering
	floor.AddComponent<ES::Plugin::OpenGL::Component::ShaderHandle>(core, "default");
    floor.AddComponent<ES::Plugin::OpenGL::Component::MaterialHandle>(core, "default");
    floor.AddComponent<ES::Plugin::OpenGL::Component::ModelHandle>(core, "floor");

    Object::Component::Mesh mesh;
	{
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
		mesh.vertices.push_back(front_bottom_left);
		mesh.vertices.push_back(front_bottom_right);
		mesh.vertices.push_back(front_top_left);
		mesh.vertices.push_back(front_top_right);

		mesh.normals.push_back(glm::vec3(0.0f, 0.0f, -1.0f));
		mesh.normals.push_back(glm::vec3(0.0f, 0.0f, -1.0f));
		mesh.normals.push_back(glm::vec3(0.0f, 0.0f, -1.0f));
		mesh.normals.push_back(glm::vec3(0.0f, 0.0f, -1.0f));

		// Back face
		mesh.vertices.push_back(back_bottom_left);
		mesh.vertices.push_back(back_bottom_right);
		mesh.vertices.push_back(back_top_left);
		mesh.vertices.push_back(back_top_right);

		mesh.normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
		mesh.normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
		mesh.normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
		mesh.normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));

		// Top face
		mesh.vertices.push_back(front_top_left);
		mesh.vertices.push_back(front_top_right);
		mesh.vertices.push_back(back_top_left);
		mesh.vertices.push_back(back_top_right);

		mesh.normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
		mesh.normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
		mesh.normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
		mesh.normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));

		// Bottom face
		mesh.vertices.push_back(front_bottom_left);
		mesh.vertices.push_back(front_bottom_right);
		mesh.vertices.push_back(back_bottom_left);
		mesh.vertices.push_back(back_bottom_right);

		mesh.normals.push_back(glm::vec3(0.0f, -1.0f, 0.0f));
		mesh.normals.push_back(glm::vec3(0.0f, -1.0f, 0.0f));
		mesh.normals.push_back(glm::vec3(0.0f, -1.0f, 0.0f));
		mesh.normals.push_back(glm::vec3(0.0f, -1.0f, 0.0f));

		// Left face
		mesh.vertices.push_back(front_bottom_left);
		mesh.vertices.push_back(front_top_left);
		mesh.vertices.push_back(back_bottom_left);
		mesh.vertices.push_back(back_top_left);

		mesh.normals.push_back(glm::vec3(-1.0f, 0.0f, 0.0f));
		mesh.normals.push_back(glm::vec3(-1.0f, 0.0f, 0.0f));
		mesh.normals.push_back(glm::vec3(-1.0f, 0.0f, 0.0f));
		mesh.normals.push_back(glm::vec3(-1.0f, 0.0f, 0.0f));

		// Right face
		mesh.vertices.push_back(front_bottom_right);
		mesh.vertices.push_back(front_top_right);
		mesh.vertices.push_back(back_bottom_right);
		mesh.vertices.push_back(back_top_right);

		mesh.normals.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
		mesh.normals.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
		mesh.normals.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
		mesh.normals.push_back(glm::vec3(1.0f, 0.0f, 0.0f));

		// Generate indices for triangle strips

		mesh.indices = {
			// Front
			2, 1, 0,
			2, 3, 1,
			// Back
			4, 5, 6,
			5, 7, 6,
			// Bottom
			10, 9, 8,
			10, 11, 9,
			// Top
			12, 13, 14,
			13, 15, 14,
			// Left
			18, 17, 16,
			18, 19, 17,
			// Right
			20, 21, 22,
			21, 23, 22,
		};
	}

	floor.AddComponent<ES::Plugin::Object::Component::Mesh>(core, mesh);

	return floor;
}

void Setup(ES::Engine::Core &core)
{
	// Next we can create a rigid body to serve as the floor, we make a large box
	ES::Engine::Entity floor = CreateFloor(core);

	// Now create a dynamic body to bounce on the floor
	ES::Engine::Entity rigidSphere = CreateSphere(core, false);

	ES::Engine::Entity softSphere = CreateSphere(core, true);

	// Now that we know which entity is the sphere, we can create its linked system
	// Note that this is for testing purposes only
	// core.RegisterSystem(InitSphereSystem{ sphere });

	core.GetResource<ES::Plugin::OpenGL::Resource::Camera>().viewer.lookFrom(glm::vec3(0.0f, 10.0f, -20.0f));
	// core.RegisterSystem(PrintSphereInfoSystem{ sphere });

	// Optional step: Before starting the physics simulation you can optimize the broad phase. This improves collision detection performance (it's pointless here because we only have 2 bodies).
	// You should definitely not call this every frame or when e.g. streaming in a new level section as it is an expensive operation.
	// Instead insert all new objects in batches instead of 1 at a time to keep the broad phase efficient.
	core.GetResource<Physics::Resource::PhysicsManager>().GetPhysicsSystem().OptimizeBroadPhase();

	// Now we're ready to simulate the body
	core.GetResource<Physics::Resource::PhysicsManager>().SetCollisionSteps(10);
}

int main(void)
{
    ES::Engine::Core core;

	core.AddPlugins<ES::Plugin::OpenGL::Plugin, ES::Plugin::Physics::Plugin>();

	core.RegisterSystem<ES::Engine::Scheduler::Startup>(Setup);

	core.RunCore();

    glfwDestroyWindow(core.GetResource<ES::Plugin::Window::Resource::Window>().GetGLFWWindow());
    glfwTerminate();

    return 0;
}