#include "btBulletDynamicsCommon.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

#include <iostream>
#include <vector>
#include <memory>

struct TriangleMeshData
{
    std::vector<btScalar> vertices;
    std::vector<int> indices;
};

TriangleMeshData createBoxMesh(const btVector3& extent)
{
    const auto& x = extent.x();
    const auto& y = extent.y();
    const auto& z = extent.z();
    return TriangleMeshData{
        {
            x, y, 0,
            x, 0, 0,
            0, 0, 0,
            0, y, 0,
            x, y, z,
            x, 0, z,
            0, 0, z,
            0, y, z
        },
        {
            0, 1, 2,
            0, 2, 3,
            4, 7, 6,
            4, 6, 5,
            0, 4, 5,
            0, 5, 1,
            1, 5, 6,
            1, 6, 2,
            2, 6, 7,
            2, 7, 3,
            4, 0, 3,
            4, 3, 7
        }
    };
}

TriangleMeshData createDiagonalTriangleMesh(bool flipTriangle)
{
    return TriangleMeshData{
        {
             0.12, 0, -0.31,
            -0.31, 0,  0.12,
            -0.31, 1,  0.12,
        },
        flipTriangle ? std::vector<int>{0, 1, 2} : std::vector<int>{ 0, 2, 1}
    };
}

struct BulletObject
{
    explicit BulletObject(TriangleMeshData meshData)
        : m_meshData{std::move(meshData)}
    {
        m_triangleIndexVertexArray = std::make_unique<btTriangleIndexVertexArray>(
            m_meshData.indices.size() / 3, m_meshData.indices.data(),
            3 * sizeof(meshData.indices[0]), static_cast<int>(m_meshData.vertices.size()),
            m_meshData.vertices.data(), 3 * sizeof(m_meshData.vertices[0]));

        m_gImpactMeshShape = std::make_unique<btGImpactMeshShape>(m_triangleIndexVertexArray.get());
        m_gImpactMeshShape->setLocalScaling(btVector3(1.0f, 1.0f, 1.0f));
        m_gImpactMeshShape->setMargin(MARGIN);
        m_gImpactMeshShape->updateBound();

        m_compountShape = std::make_unique<btCompoundShape>();
        btTransform transform;
        transform.setIdentity();
        m_compountShape->addChildShape(transform, m_gImpactMeshShape.get());
        m_compountShape->setMargin(MARGIN);
        
        m_collisionObject = std::make_unique<btCollisionObject>();
        m_collisionObject->setCollisionShape(m_compountShape.get());
    }

    btCollisionObject* getBtCollisionObject()
    {
        return m_collisionObject.get();
    }

private:
    static constexpr btScalar MARGIN = 1.0e-4;

    TriangleMeshData m_meshData;
    std::unique_ptr<btTriangleIndexVertexArray> m_triangleIndexVertexArray;
    std::unique_ptr<btGImpactMeshShape>         m_gImpactMeshShape;
    std::unique_ptr<btCompoundShape>            m_compountShape;
    std::unique_ptr<btCollisionObject>          m_collisionObject;
};

struct BulletContactResultCallback : public btCollisionWorld::ContactResultCallback
{
public:

    bool needsCollision(btBroadphaseProxy* proxy0) const override
    {
        return true;
    }

    btScalar addSingleResult(btManifoldPoint &cp, const btCollisionObjectWrapper *colObj0Wrap,
                             int partId0, int index0, const btCollisionObjectWrapper *colObj1Wrap,
                             int partId1, int index1) override
    {
        btVector3 ptA = cp.getPositionWorldOnA();
        btVector3 ptB = cp.getPositionWorldOnB();
        double distance = cp.getDistance();

        std::cout << "Contact : (" << ptA.getX() << "," << ptA.getY() << "," << ptA.getZ()
                  << ") (" << ptB.getX() << "," << ptB.getY() << "," << ptB.getZ()
                  << ") & d: " << distance << '\n';
        return 0;
    }
};

int main()
{
    btDefaultCollisionConfiguration collisionConfiguration;
    btCollisionDispatcher dispatcher(
        static_cast<btCollisionConfiguration *>(&collisionConfiguration));

    btScalar sceneSize = btScalar(100);
    btVector3 worldAabbMin(-sceneSize, -sceneSize, -sceneSize);
    btVector3 worldAabbMax(sceneSize, sceneSize, sceneSize);

    btAxisSweep3 broadphase(worldAabbMin, worldAabbMax, 16000);

    btCollisionWorld collisionWorld(
        static_cast<btDispatcher *>(&dispatcher), static_cast<btBroadphaseInterface *>(&broadphase),
        static_cast<btCollisionConfiguration *>(&collisionConfiguration));

    // Register Gimpact algorithm
    btGImpactCollisionAlgorithm::registerAlgorithm(&dispatcher);

    auto box = BulletObject(createBoxMesh(btVector3{0.02, 1, 1}));
    auto triangle = BulletObject(createDiagonalTriangleMesh(true));
    collisionWorld.addCollisionObject(box.getBtCollisionObject());
    collisionWorld.addCollisionObject(triangle.getBtCollisionObject());

    BulletContactResultCallback callback;
    collisionWorld.contactTest(triangle.getBtCollisionObject(), callback);

    collisionWorld.removeCollisionObject(box.getBtCollisionObject());
    collisionWorld.removeCollisionObject(triangle.getBtCollisionObject());
    return 0;
}
