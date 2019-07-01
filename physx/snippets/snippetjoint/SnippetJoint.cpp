//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// ****************************************************************************
// This snippet illustrates simple use of joints in physx
//
// It creates a chain of objects joined by limited spherical joints, a chain
// joined by fixed joints which is breakable, and a chain of damped D6 joints
// ****************************************************************************

#include <ctype.h>
#include <iostream>

#include "PxPhysicsAPI.h"

#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"

#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include "tiny_obj_loader.h"

using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene		= NULL;

PxMaterial*				gMaterial	= NULL;

PxPvd*                  gPvd        = NULL;

PxReal chainZ = 10.0f;

// apply forces on these actors
PxRigidDynamic* actor1 = NULL;
PxRigidDynamic* actor2 = NULL;

PxCooking*				gCooking = NULL;

PxRigidDynamic* gRigidDynaMeshActor;
PxRigidStatic* gRigidStaticMeshActor;

bool paused = true;

unsigned int frameCount = 0;

std::vector<std::string> LseriesFiles = { "L01M.obj", "L02M.obj", "L03M.obj", "L04M.obj", "L05M.obj" };

std::vector<std::string> TseriesFiles = { "T01M.obj", "T02M.obj", "T03M.obj", "T04M.obj", "T05M.obj",
"T06M.obj", "T07M.obj", "T08M.obj", "T09M.obj","T10M.obj" ,
"T11M.obj", "T12M.obj" };

std::vector<std::string> CseriesFiles = { "C01M.obj", "C02M.obj", "C03M.obj", "C04M.obj", "C05M.obj",
"C06M.obj", "C07M.obj" };

// Spine angle limits
std::vector<float> xLimit = { 10., 5., 7.5, 10., 10., 8., 4.5, 2., 2., 2., 2., 2., 2.5, 3., 3., 3., 4., 6., 6., 6., 7., 7., 8. };
std::vector<float> yLimit = { 4., 10., 11., 11., 8., 7., 4., 5., 6., 5., 6., 6., 6., 6., 6., 6., 7., 9., 8., 6., 6., 8., 6.};
std::vector<float> zLimit = { 0., 3., 6.5, 6.5, 6.5, 6., 2., 9., 8., 8., 8., 8., 7., 7., 6., 4., 2., 2., 2., 2., 2., 2., 2. };

#define REPORT_INIT_ERROR_IFANY(OBJECT, CLBK, MESG) \
if(!OBJECT) CLBK.reportError(physx::PxErrorCode::eINTERNAL_ERROR, MESG, __FILE__, __LINE__);

void loadObjFile(const std::string &filename, std::vector<PxVec3>& vertices, std::vector <PxU32>& indices)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename.c_str());

    if (!warn.empty()) {
        std::cout << warn << std::endl;
    }

    if (!err.empty()) {
        std::cerr << err << std::endl;
    }

    if (!ret) {
        exit(1);
    }

    for (size_t v = 0; v < attrib.vertices.size() / 3; v++) {       
        vertices.push_back(PxVec3(static_cast<const double>(attrib.vertices[3 * v + 0]),
            static_cast<const double>(attrib.vertices[3 * v + 1]),
            static_cast<const double>(attrib.vertices[3 * v + 2])));

    }

    for (size_t i = 0; i < shapes.size(); i++) 
    {        
        for (size_t j = 0; j < shapes[i].mesh.indices.size(); j++) 
        {
            indices.push_back((PxU32)(shapes[i].mesh.indices[j].vertex_index));
        }
    }

}

PxVec3 computeBoxHalfLengths(const std::vector<PxVec3>& vertices, const std::vector <PxU32>& indices)
{
    PxReal x = (vertices[1] - vertices[6]).magnitude()*0.5;
    PxReal y = (vertices[0] - vertices[1]).magnitude()*0.5;
    PxReal z = (vertices[1] - vertices[3]).magnitude()*0.5;

    return PxVec3(x, y, z);
}

PxTransform computeTransformOfBox(const std::vector<PxVec3>& vertices, const std::vector <PxU32>& indices, const bool includeTranslaton = true)
{
    // computer translation
    PxVec3 center(0., 0., 0.);
    for (auto& v : vertices)
    {
        center += v;
    }
    center /= 8.;

    // compute the rotation
    PxVec3 xT, yT, zT;
    xT = (vertices[1] - vertices[6]).getNormalized();
    yT = (vertices[0] - vertices[1]).getNormalized();
    zT = (vertices[1] - vertices[3]).getNormalized();
    
    if (includeTranslaton)
    {
        return PxTransform(center, PxQuat(PxMat33(xT, yT, zT)));
    }
    else
    {
        return PxTransform(PxQuat(PxMat33(xT, yT, zT)));
    }
}

// Setup common cooking params
void setupCommonCookingParams(PxCookingParams& params, bool skipMeshCleanup, bool skipEdgeData)
{
    // we suppress the triangle mesh remap table computation to gain some speed, as we will not need it 
// in this snippet
    params.suppressTriangleMeshRemapTable = true;

    // If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. The input mesh must be valid. 
    // The following conditions are true for a valid triangle mesh :
    //  1. There are no duplicate vertices(within specified vertexWeldTolerance.See PxCookingParams::meshWeldTolerance)
    //  2. There are no large triangles(within specified PxTolerancesScale.)
    // It is recommended to run a separate validation check in debug/checked builds, see below.

    if (!skipMeshCleanup)
        params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH);
    else
        params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

    // If DISABLE_ACTIVE_EDGES_PREDOCOMPUTE is set, the cooking does not compute the active (convex) edges, and instead 
    // marks all edges as active. This makes cooking faster but can slow down contact generation. This flag may change 
    // the collision behavior, as all edges of the triangle mesh will now be considered active.
    if (!skipEdgeData)
        params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE);
    else
        params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
}

// Creates a triangle mesh using BVH34 mid-phase with different settings

PxTriangleMesh* createBV34TriangleMesh(PxU32 numVertices, const PxVec3* vertices, PxU32 numTriangles, const PxU32* indices,
    bool skipMeshCleanup, bool skipEdgeData, bool inserted, const PxU32 numTrisPerLeaf)
{
    PxU64 startTime = SnippetUtils::getCurrentTimeCounterValue();

    PxTriangleMeshDesc meshDesc;
    meshDesc.points.count = numVertices;
    meshDesc.points.data = vertices;
    meshDesc.points.stride = sizeof(PxVec3);
    meshDesc.triangles.count = numTriangles;
    meshDesc.triangles.data = indices;
    meshDesc.triangles.stride = 3 * sizeof(PxU32);

    PxCookingParams params = gCooking->getParams();

    // Create BVH34 mid-phase
    params.midphaseDesc = PxMeshMidPhase::eBVH34;

    // setup common cooking params
    setupCommonCookingParams(params, skipMeshCleanup, skipEdgeData);

    // Cooking mesh with less triangles per leaf produces larger meshes with better runtime performance
    // and worse cooking performance. Cooking time is better when more triangles per leaf are used.
    params.midphaseDesc.mBVH34Desc.numPrimsPerLeaf = numTrisPerLeaf;

    gCooking->setParams(params);

#if defined(PX_CHECKED) || defined(PX_DEBUG)
    // If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. 
    // We should check the validity of provided triangles in debug/checked builds though.
    if (skipMeshCleanup)
    {
        PX_ASSERT(gCooking->validateTriangleMesh(meshDesc));
    }
#endif // DEBUG
    
    PxU32 meshSize = 0;

    PxTriangleMesh* triMesh = NULL;

    // The cooked mesh may either be saved to a stream for later loading, or inserted directly into PxPhysics.
    if (inserted)
    {
        triMesh = gCooking->createTriangleMesh(meshDesc, gPhysics->getPhysicsInsertionCallback());
    }
    else
    {
        PxDefaultMemoryOutputStream outBuffer;
        gCooking->cookTriangleMesh(meshDesc, outBuffer);

        PxDefaultMemoryInputData stream(outBuffer.getData(), outBuffer.getSize());
        triMesh = gPhysics->createTriangleMesh(stream);

        meshSize = outBuffer.getSize();
    }

    // Print the elapsed time for comparison
    PxU64 stopTime = SnippetUtils::getCurrentTimeCounterValue();
    float elapsedTime = SnippetUtils::getElapsedTimeInMilliseconds(stopTime - startTime);
    printf("\t -----------------------------------------------\n");
    printf("\t Create triangle mesh with %d triangles: \n", numTriangles);
    inserted ? printf("\t\t Mesh inserted on\n") : printf("\t\t Mesh inserted off\n");
    !skipEdgeData ? printf("\t\t Precompute edge data on\n") : printf("\t\t Precompute edge data off\n");
    !skipMeshCleanup ? printf("\t\t Mesh cleanup on\n") : printf("\t\t Mesh cleanup off\n");
    printf("\t\t Num triangles per leaf: %d \n", numTrisPerLeaf);
    printf("\t Elapsed time in ms: %f \n", double(elapsedTime));
    if (!inserted)
    {
        printf("\t Mesh size: %d \n", meshSize);
    }

    return triMesh;
}

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

// spherical joint limited to an angle of at most pi/4 radians (45 degrees)
PxJoint* createLimitedSpherical(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
{
	PxSphericalJoint* j = PxSphericalJointCreate(*gPhysics, a0, t0, a1, t1);
	j->setLimitCone(PxJointLimitCone(PxPi / 20, PxPi / 20, 0.05f));
	j->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
	return j;
}

// revolute joint limited to an angle of at most pi/4 radians (45 degrees)

// fixed, breakable joint
PxJoint* createBreakableFixed(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
{
	PxFixedJoint* j = PxFixedJointCreate(*gPhysics, a0, t0, a1, t1);
	//j->setBreakForce(1000, 100000);	
	j->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, true);
	j->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, true);
	return j;
}

// D6 joint with a spring maintaining its position
PxJoint* createDampedD6(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
{
    PxD6Joint* j = PxD6JointCreate(*gPhysics, a0, t0, a1, t1);
    j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
    j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
    j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
    j->setDrive(PxD6Drive::eSLERP, PxD6JointDrive(0, 1000, FLT_MAX, true));
    
    //j->setTwistLimit(PxJointAngularLimitPair(-0.15, 0.15));
    
    j->setSwingLimit(PxJointLimitCone(0.19, 0.19));
    return j;
}

// D6 joint with a spring maintaining its position
// angle limits are in degrees
PxJoint* createDampedD6(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, 
    const PxTransform& t1, const float limX, const float limY, const float limZ)
{
    PxD6Joint* j = PxD6JointCreate(*gPhysics, a0, t0, a1, t1);
    j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
    j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
    j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
    j->setDrive(PxD6Drive::eSLERP, PxD6JointDrive(0, 1000, FLT_MAX, true));

    // set angular limits
    const float degToRad = 22. / 7. / 180.;    
    j->setTwistLimit(PxJointAngularLimitPair(-limZ, limZ));
    j->setSwingLimit(PxJointLimitCone(degToRad*limX, degToRad*limY));
    
    return j;
}

// D6 joint with a spring maintaining its position customized for spine
PxJoint* createSpineJointD6(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
{
	PxD6Joint* j = PxD6JointCreate(*gPhysics, a0, t0, a1, t1);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
	j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
    j->setDrive(PxD6Drive::eSLERP, PxD6JointDrive(1000, 80000.00, FLT_MAX, true));
	
    j->setTwistLimit(PxJointAngularLimitPair(-0.015, 0.015));
    j->setSwingLimit(PxJointLimitCone(-0.0015, 0.0015));

    return j;
}

// D6 joint with a spring maintaining its position customized for spine
PxJoint* createSpineJointSpherical(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
{
    PxSphericalJoint* j = PxSphericalJointCreate(*gPhysics, a0, t0, a1, t1);
    j->setLimitCone(PxJointLimitCone(PxPi / 20, PxPi / 20));    
    j->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
    return j;
}

typedef PxJoint* (*JointCreateFunction)(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1);

// create a chain rooted at the origin and extending along the x-axis, all transformed by the argument t.
void createChain(const PxTransform& t, PxU32 length, const PxGeometry& g, PxReal separation, JointCreateFunction createJoint)
{
    PxVec3 offset(separation / 2, 0, 0);
    PxTransform localTm(offset);
    PxRigidDynamic* prev = NULL;

    for (PxU32 i = 0; i < length; i++)
    {
        PxRigidDynamic* current = PxCreateDynamic(*gPhysics, t*localTm, g, *gMaterial, 1.0f);
        (*createJoint)(prev, prev ? PxTransform(offset) : t, current, PxTransform(-offset));
        gScene->addActor(*current);
        prev = current;
        localTm.p.x += separation;
    }
}

void addMeshShapeToActor(PxRigidDynamic* dynaActor, const char* filename, const PxTransform& t = PxTransform(PxIdentity), const PxReal scale = 1.)
{
    // Test mesh loading, cooking and custom actor
    std::vector<PxVec3> vertices;
    std::vector <PxU32> indices;
    loadObjFile(filename, vertices, indices);

    // scale the mesh
    for (auto& p : vertices) { p = p * scale; p = t.transform(p); }

    PxTriangleMesh* triMesh = createBV34TriangleMesh(vertices.size(), vertices.data(), indices.size() / 3, indices.data(), false, false, false, 4);
    PxShape* aConvexShape1 = PxRigidActorExt::createExclusiveShape(*dynaActor, PxTriangleMeshGeometry(triMesh), *gMaterial);
}

// create a spine model with custom D6 links between the vertebrae
void createRealSpineModel(const PxU32 length, PxTransform& offset)
{
    std::vector<PxTransform> vertibraeTransforms;
    std::vector<PxVec3> vertibraCenters(length, PxVec3(0., 0., 0.));
    std::vector<PxRigidActor*> vertibrae;

    std::vector<std::string> vertibraeName;
    if (length == 12)
    {
        vertibraeName.insert(vertibraeName.begin(), TseriesFiles.begin(), TseriesFiles.end());
    }    
    else if (length == 17)
    {
        vertibraeName.insert(vertibraeName.begin(), TseriesFiles.begin(), TseriesFiles.end());
        vertibraeName.insert(vertibraeName.end(), LseriesFiles.begin(), LseriesFiles.end());
    }
    else if (length == 24)
    {
        vertibraeName.insert(vertibraeName.begin(), CseriesFiles.begin(), CseriesFiles.end());
        vertibraeName.insert(vertibraeName.end(), TseriesFiles.begin(), TseriesFiles.end());
        vertibraeName.insert(vertibraeName.end(), LseriesFiles.begin(), LseriesFiles.end());
    }
    else
    {
        std::cout << "Supply right number of vertibrae";
        exit(1);
    }

    // Create vertibrae
    for (PxU32 i = 0; i < vertibraeName.size(); ++i)
    {
        std::vector<PxVec3> vertices;
        std::vector <PxU32> indices;

        loadObjFile(std::string("objs/" + vertibraeName[i]), vertices, indices); 
            
        vertibraeTransforms.push_back(offset*computeTransformOfBox(vertices, indices));
        auto boxGeo = PxBoxGeometry(computeBoxHalfLengths(vertices, indices));

        PxRigidDynamic* dynaObj = PxCreateDynamic(*gPhysics, vertibraeTransforms[i], boxGeo, *gMaterial, 1.0f);

        if (i == 10){ actor1 = dynaObj; }
        
        /*if (i >= 7 && i < 19)
        {
            int Lnum = i - 6;
            std::string pre = (Lnum < 10) ? std::string("objs/T0") : std::string("objs/T");

            addMeshShapeToActor(dynaObj, (pre + std::string(std::to_string(Lnum) + "L.obj")).c_str(), offset);
            addMeshShapeToActor(dynaObj, (pre + std::string(std::to_string(Lnum) + "R.obj")).c_str(), offset);
        }*/
        
        dynaObj->setMass(0.2);
        dynaObj->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
        
        vertibrae.push_back(dynaObj);
        
        gScene->addActor(*vertibrae[i]);         

        // compute center
        for (auto& v : vertices)
        {
            vertibraCenters[i] += v;
        }        
        vertibraCenters[i] /= 8.;
    }

    // Create joints in local frame
    for (PxU32 i = 0; i < length - 1; ++i)
    {
        const PxReal zOffset = (vertibraCenters[i + 1] - vertibraCenters[i]).magnitude()*0.5;
        createDampedD6(vertibrae[i], PxTransform(PxVec3(0., 0., -zOffset)), vertibrae[i + 1], PxTransform(PxVec3(0., 0., zOffset)), xLimit[i], yLimit[i], zLimit[i]);
    }

    // create end joints (with angular limits of the nearest joint)
    const PxReal zOffset = (vertibraCenters[1] - vertibraCenters[0]).magnitude()*0.5;
    createDampedD6(NULL, vertibraeTransforms[0] * PxTransform(PxVec3(0., 0., 0)), vertibrae[0], PxTransform(PxVec3(0., 0., -0)), xLimit[0], yLimit[0], zLimit[0]);
    
    const PxReal zOffset2 = (vertibraCenters[length-1] - vertibraCenters[length-2]).magnitude()*0.5;
    createDampedD6(vertibrae[length - 1], PxTransform(PxVec3(0., 0., 0)), NULL, vertibraeTransforms[length-1] * PxTransform(PxVec3(0., 0., -0)), xLimit[length - 2], yLimit[length - 2], zLimit[length - 2]);
}

// create a spine model with custom D6 links between the vertebrae
void createSpineModel(const PxTransform& t, PxU32 length, const PxGeometry& g, PxReal separation, JointCreateFunction createJoint)
{
    PxVec3 offset(0, separation / 2, 0);
    PxTransform localTm(offset);
    PxRigidDynamic* prev = NULL;

    for (PxU32 i = 0; i < length; i++)
    {
        PxRigidDynamic* current = PxCreateDynamic(*gPhysics, t*localTm, g, *gMaterial, 1.0f);
        (*createJoint)(prev, prev ? PxTransform(offset) : t, current, PxTransform(-offset));
        gScene->addActor(*current);
        prev = current;
        localTm.p.y += separation;

        if (i == 4)
        {
            actor1 = current;
        }
        if (i == 11)
        {
            actor2 = current;
        }
    }
    (*createJoint)(prev, PxTransform(offset), NULL, t*PxTransform(PxVec3(0.0f, separation*length, 0.0f)));
}

void createRigidStaticMeshActor(const char* filename, const PxTransform& t = PxTransform(PxIdentity), const PxReal scale = 1., const bool disableSim = true)
{
    // Test mesh loading, cooking and custom actor
    std::vector<PxVec3> vertices;
    std::vector <PxU32> indices;
    loadObjFile(filename, vertices, indices);

    // scale the mesh
    for (auto& p : vertices) { p = p * scale; }

    // Add mesh actor
    gRigidStaticMeshActor = gPhysics->createRigidStatic(t);    
    PxTriangleMesh* triMesh = createBV34TriangleMesh(vertices.size(), vertices.data(), indices.size() / 3, indices.data(), false, false, false, 4);
    PxShape* aConvexShape = PxRigidActorExt::createExclusiveShape(*gRigidStaticMeshActor, PxTriangleMeshGeometry(triMesh), *gMaterial);
    
    if (disableSim)
    {
        gRigidStaticMeshActor->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, true);
    }

    gScene->addActor(*gRigidStaticMeshActor);
}

void createRigidDynamicMeshActor(const char* filename, const PxTransform& t = PxTransform(PxIdentity), const PxReal scale = 1.)
{
    // Test mesh loading, cooking and custom actor
    std::vector<PxVec3> vertices;
    std::vector <PxU32> indices;
    loadObjFile(filename, vertices, indices);

    // scale the mesh
    for (auto& p : vertices) { p = p * scale; }

    //PxTriangleMesh* triMesh = createBV34TriangleMesh(vertices.size(), vertices.data(), indices.size() / 3, indices.data(), false, false, false, 4);
    gRigidDynaMeshActor = gPhysics->createRigidDynamic(t);
    //PxShape* customShape = PxRigidActorExt::createExclusiveShape(*rigidDynamicActor, PxTriangleMeshGeometry(triMesh), *gMaterial, PxShapeFlag::eSIMULATION_SHAPE);    
    //PxShape* customShape2 = PxRigidActorExt::createExclusiveShape(*rigidDynamicActor, PxBoxGeometry(1.5f, 1.8f, 1.5f), *gMaterial, PxShapeFlag::eSIMULATION_SHAPE);  

    // cook convex mesh
    PxConvexMeshDesc convexDesc;
    convexDesc.points.count = vertices.size();
    convexDesc.points.stride = sizeof(PxVec3);
    convexDesc.points.data = vertices.data();
    convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
    PxDefaultMemoryOutputStream buf;
    PxConvexMeshCookingResult::Enum result;
    if (!gCooking->cookConvexMesh(convexDesc, buf, &result))
    {
        std::cout << "Failed to cook convex mesh!" << "File: " << __FILE__ << "Line: " << __LINE__ << std::endl;
    }
    PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
    PxConvexMesh* convexMesh = gPhysics->createConvexMesh(input);
    PxShape* aConvexShape = PxRigidActorExt::createExclusiveShape(*gRigidDynaMeshActor, PxConvexMeshGeometry(convexMesh), *gMaterial);
    
    gScene->addActor(*gRigidDynaMeshActor);
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
    REPORT_INIT_ERROR_IFANY(gFoundation, gErrorCallback, "PxCreateFoundation failed!!")

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true, gPvd);
    REPORT_INIT_ERROR_IFANY(gPhysics, gErrorCallback, "PxCreatePhysics failed!!")

	PxInitExtensions(*gPhysics, gPvd);

    gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(PxTolerancesScale()));
    REPORT_INIT_ERROR_IFANY(gCooking, gErrorCallback, "PxCreateCooking failed!!")

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);// PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);
    gScene->setVisualizationParameter(PxVisualizationParameter::eACTOR_AXES, 40.0f);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);

	//createChain(PxTransform(PxVec3(0.0f, 20.0f, 0.0f)), 15, PxBoxGeometry(2.0f, 0.15f, 0.15f), 4.0f, createLimitedSpherical);
    //createRigidStaticMeshActor("./bowl.obj", PxTransform(PxVec3(10, -20, -25)), 5.0, false);

    //createChain(PxTransform(PxVec3(0.0f, 20.0f, -10.0f)), 15, PxBoxGeometry(2.0f, 0.15f, 0.15f), 4.0f, createBreakableFixed);
    //createChain(PxTransform(PxVec3(0.0f, 20.0f, -20.0f)), 15, PxBoxGeometry(2.0f, 0.15f, 0.15f), 4.0f, createDampedD6);
    //auto s = PxTransform(PxQuat(3.14 / 4, PxVec3(1., 0., 0.)));
    
    createRealSpineModel(24, PxTransform(PxVec3(0.0f, 40.0f, 0.0f)));    
    //createRigidStaticMeshActor("objs/Sacrum.obj", PxTransform(PxVec3(0.0f, 40.0f, 0.0f)), 1.0, false);
    //createSpineModel(s, 15, PxBoxGeometry(1.5f, 1.8f, 1.5f), 4.0f, createSpineJointD6);//PxTransform(PxVec3(0.0f, 20.0f, -20.0f))
    //createRigidDynamicMeshActor("./asianDragon.obj", PxTransform(PxVec3(10, 10, 10)), 5.0);
    //createRigidDynamicMeshActor("objs/T01L.obj", PxTransform(PxIdentity));
    //createRigidStaticMeshActor("./box_stack.obj", PxTransform(PxVec3(10, 10, 10)), 5.0, true);
}

void transferGlobalPose(PxRigidDynamic* sourceActor, PxRigidStatic* destinationActor, PxTransform& offset = PxTransform(PxIdentity))
{    
    if (!sourceActor || !destinationActor) { return;}

    destinationActor->setGlobalPose(offset*sourceActor->getGlobalPose());
}

void stepPhysics(bool /*interactive*/)
{
    // apply and release forces at certain intervals
    /*if (actor1 && actor2)
    {
        if (frameCount > 30000 && frameCount < 50000)
        {
            actor1->addForce(PxVec3(0.0f, 0.0f, -2.5e6f*(20000 - 50000 + count) / 20000), PxForceMode::eFORCE, true);
            actor2->addForce(PxVec3(0.0f, 0.0f, 2.5e6f*(20000 - 50000 + count) / 20000), PxForceMode::eFORCE, true);
        }
        frameCount++;

        if (frameCount > 50000)
        {
            frameCount = 0;
        }
    }*/

    if (!paused && actor1)
    {
        if (frameCount > 60000 && frameCount < 90000)
        {
            actor1->addForce(PxVec3(-1.2e2f, 0.0f, 0.0f), PxForceMode::eFORCE, true);
            //PxRigidBodyExt::addForceAtPos(*actor1, PxVec3(1.2e2f, 0.0f, 0.0f), PxVec3(0., 0., 0.));
        }        
        frameCount++;
    }

    /*if (gRigidStaticMeshActor)
    {
        transferGlobalPose(gRigidDynaMeshActor, gRigidStaticMeshActor, PxTransform(PxVec3(20, 0, 20)));
        gRigidStaticMeshActor->setGlobalPose(PxTransform(PxVec3(0.002, 0.0002, -0.002))*gRigidStaticMeshActor->getGlobalPose());
    }*/

    if (!paused)
    {
        gScene->simulate(1.0f / 400.0f);
        gScene->fetchResults(true);
        //paused = !paused;
    }
}
	
void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PxCloseExtensions();
	PX_RELEASE(gPhysics);
    PX_RELEASE(gCooking);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
	
	printf("SnippetJoint done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch(toupper(key))
	{
	case 'P':	
        createDynamic(camera, PxSphereGeometry(6.0f), camera.rotate(PxVec3(0,0,-1))*200);	
        break;

    case ' ':
        paused=!paused;
        break;
	}
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}
