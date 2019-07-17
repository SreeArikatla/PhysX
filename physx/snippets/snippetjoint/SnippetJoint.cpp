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
#include <limits>
#include <algorithm>

// PhysX includes
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "../../include/extensions/PxRigidBodyExt.h"

#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include "tiny_obj_loader.h"

using namespace physx;

// PhysX var
PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;
PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;
PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene		= NULL;
PxMaterial*				gMaterial	= NULL;
PxPvd*                  gPvd        = NULL;
PxCooking*				gCooking = NULL;

bool paused = false;                        ///> Tracks if the simulation is paused or not
bool firstIteration = true;                 ///> True if before the first simulation iteration
bool includeGroundPlane = true;             ///> Include non-functional ground plane or not
unsigned int frameCount = 0;                ///> Stores the simulation frame number
std::vector<PxRigidDynamic*> vertebrae;     ///> Vector of all the dynamic actors created for vertebrae
float poseDataPrev[168];                    ///> Stores the pose of the spine from last simulation frame
float poseDataCurrent[168];                 ///> Stores the pose of the spine in current simulation frame

///
/// \brief data structure to hold the force specification
/// stores the amount of force, the vertebrae number and 
/// the position (w.r.t local frame) where the force should be applied
///
struct vertebraeForce
{
    PxVec3 force;
    PxVec3 position;
    PxU32 vertebraeNumber;

    vertebraeForce(const PxU32 num, const PxVec3 f, const PxVec3 p) :
        vertebraeNumber(num), force(f), position(p) {}
};
std::vector<vertebraeForce> vertebraeForces; ///>  All the external forces to be applied is stored here

// user parameters
float convergenceTol = 0.5e-3;              ///> Tolerance against which the convergence of the spine pose will be checked
float timeStepSize = 1. / 400;              ///> Time step used for the simulation

///
/// \brief Keep track of the simulation state
///
enum spineSimulationState
{
    preInitialization=0,
    movingToDefaultPose,
    convergedToDefaultPose,
    movingToTargetPose,
    ConvergedToTargetPose
};

spineSimulationState simState = spineSimulationState::preInitialization; ///> Track the simualtion state

/// Store the name of the external files of simplistic vertebrae models registered to the 3D spine
std::vector<std::string> LseriesFiles = { "L01M.obj", "L02M.obj", "L03M.obj", "L04M.obj", "L05M.obj" };

std::vector<std::string> TseriesFiles = { "T01M.obj", "T02M.obj", "T03M.obj", "T04M.obj", "T05M.obj",
                                        "T06M.obj", "T07M.obj", "T08M.obj", "T09M.obj","T10M.obj" ,
                                        "T11M.obj", "T12M.obj" };

std::vector<std::string> CseriesFiles = { "C01M.obj", "C02M.obj", "C03M.obj", "C04M.obj", "C05M.obj",
                                        "C06M.obj", "C07M.obj" };

/// Spine angle limits in Cartesian axes in the local frame of the joint
std::vector<float> xAngularLimits = { 10., 5., 7.5, 10., 10., 8., 4.5, 2., 2., 2., 2., 2., 2.5, 3., 3., 3., 4., 6., 6., 6., 7., 7., 8. };
std::vector<float> yAngularLimits = { 4., 10., 11., 11., 8., 7., 4., 5., 6., 5., 6., 6., 6., 6., 6., 6., 7., 9., 8., 6., 6., 8., 6.};
std::vector<float> zAngularLimits = { 0., 3., 6.5, 6.5, 6.5, 6., 2., 9., 8., 8., 8., 8., 7., 7., 6., 4., 2., 2., 2., 2., 2., 2., 2. };

// forward deceleration
void addForceOnVertebrae(const unsigned int vertebraeNum, const PxVec3& force, const PxVec3& pos);
void addForceOnVertebrae(const unsigned int vertebraeNum,
                         const float fX, const float fY, const float fZ,
                         const float pX, const float pY, const float pZ);
void removeForcesOnVertebrae(const unsigned int vertebraeNum);

#define REPORT_INIT_ERROR_IFANY(OBJECT, CLBK, MESG) \
if(!OBJECT) CLBK.reportError(physx::PxErrorCode::eINTERNAL_ERROR, MESG, __FILE__, __LINE__);

///
/// \brief Load an .obj file
///
void loadObjFile(const std::string &filename, std::vector<PxVec3>& vertices, std::vector <PxU32>& indices)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename.c_str());

    if (!warn.empty()) 
    {
        std::cout << warn << std::endl;
    }

    if (!err.empty()) 
    {
        std::cerr << err << std::endl;
    }

    if (!ret) 
    {
        printf("ERROR loading %s\n", filename);
        exit(1);
    }

    for (size_t v = 0; v < attrib.vertices.size() / 3; v++) 
    {       
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

///
/// \brief Compute the half lengths given the vertices of the box
///
PxVec3 computeBoxHalfLengths(const std::vector<PxVec3>& vertices)
{
    PxReal x = (vertices[1] - vertices[6]).magnitude()*0.5;
    PxReal y = (vertices[0] - vertices[1]).magnitude()*0.5;
    PxReal z = (vertices[1] - vertices[3]).magnitude()*0.5;

    return PxVec3(x, y, z);
}

///
/// \brief Compute the transform given the transformed box
///
PxTransform computeTransformOfBox(const std::vector<PxVec3>& vertices, 
                                  const std::vector <PxU32>& indices, 
                                  const bool includeTranslaton = true)
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

///
/// \brief Create a dynamic actor and add it to the scene
///
PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

///
/// \brief D6 joint with a spring maintaining its position angle limits are in degrees
///
PxJoint* createDampedD6(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, 
                        const PxTransform& t1, const float limX, const float limY, const float limZ)
{
    PxD6Joint* j = PxD6JointCreate(*gPhysics, a0, t0, a1, t1);
    j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
    j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
    j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
    j->setDrive(PxD6Drive::eSLERP, PxD6JointDrive(0, 1000, FLT_MAX, true));

    // set angular limits along different axes
    const float degToRad = 22. / 7. / 180.;    
    j->setTwistLimit(PxJointAngularLimitPair(-limZ, limZ));
    j->setSwingLimit(PxJointLimitCone(degToRad*limX, degToRad*limY));
    
    return j;
}

typedef PxJoint* (*JointCreateFunction)(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1);

///
/// \brief create a spine model with custom D6 links between the vertebrae
///
void createRealSpineModel(PxTransform& offset)
{
    const PxU32 length = 24;
    std::vector<PxTransform> vertebraeTransforms;
    std::vector<PxVec3> vertebraCenters(length, PxVec3(0., 0., 0.));

    std::vector<std::string> vertebraeName;

    vertebraeName.insert(vertebraeName.begin(), CseriesFiles.begin(), CseriesFiles.end());
    vertebraeName.insert(vertebraeName.end(), TseriesFiles.begin(), TseriesFiles.end());
    vertebraeName.insert(vertebraeName.end(), LseriesFiles.begin(), LseriesFiles.end());

    // Create vertebrae
    for (PxU32 i = 0; i < vertebraeName.size(); ++i)
    {
        std::vector<PxVec3> vertices;
        std::vector <PxU32> indices;

        // Load the input file
        loadObjFile(std::string("objs/" + vertebraeName[i]), vertices, indices); 
            
        // Extract the transform and half lengths from the loaded model
        // Use the resulting parameters to create a box geometry
        vertebraeTransforms.push_back(offset*computeTransformOfBox(vertices, indices));
        auto boxGeo = PxBoxGeometry(computeBoxHalfLengths(vertices));

        // Create a dynamic model with the box geometry
        PxRigidDynamic* dynaObj = PxCreateDynamic(*gPhysics, vertebraeTransforms[i], boxGeo, *gMaterial, 1.0f);
        
        dynaObj->setMass(0.2);
        dynaObj->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true); // disable gravity
                
        vertebrae.push_back(dynaObj);
        
        gScene->addActor(*vertebrae[i]);         

        // Compute center center points of the vertebrae
        for (auto& v : vertices)
        {
            vertebraCenters[i] += v;
        }        
        vertebraCenters[i] /= 8.;
    }

    // Create joints in the local frame
    for (PxU32 i = 0; i < length - 1; ++i)
    {
        const PxReal zOffset = (vertebraCenters[i + 1] - vertebraCenters[i]).magnitude()*0.5;
        createDampedD6(vertebrae[i], PxTransform(PxVec3(0., 0., -zOffset)), vertebrae[i + 1], PxTransform(PxVec3(0., 0., zOffset)), 
                       xAngularLimits[i], yAngularLimits[i], zAngularLimits[i]);
    }

    // create end joints (with angular limits of the nearest joint)
    const PxReal zOffset = (vertebraCenters[1] - vertebraCenters[0]).magnitude()*0.5;
    createDampedD6(NULL, vertebraeTransforms[0] * PxTransform(PxVec3(0., 0., 0)), vertebrae[0], PxTransform(PxVec3(0., 0., -0)), 
                   xAngularLimits[0], yAngularLimits[0], zAngularLimits[0]);
    
    const PxReal zOffset2 = (vertebraCenters[length-1] - vertebraCenters[length-2]).magnitude()*0.5;
    createDampedD6(vertebrae[length - 1], PxTransform(PxVec3(0., 0., 0)), NULL, vertebraeTransforms[length-1] * PxTransform(PxVec3(0., 0., -0)), 
                   xAngularLimits[length - 2], yAngularLimits[length - 2], zAngularLimits[length - 2]);
}

///
/// \brief Initialize the physics
///
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
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
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

    if (includeGroundPlane)
    {
        PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
        gScene->addActor(*groundPlane);
    }

    // Create a spine model
    createRealSpineModel(PxTransform(PxVec3(0.0f, 40.0f, 0.0f)));   

    // add sample force
    addForceOnVertebrae(10, PxVec3(-1.2e2f, 0.0f, 0.0f), PxVec3(0.f, 0.f, 0.f));
}

///
/// \brief Get the pose of all the 24 vertebrae
///
void getSpinePose(float* poseData)
{
    unsigned int count = 0;
    for (const auto& v : vertebrae)
    {
        const auto t = v->getGlobalPose();
        poseData[count++] = t.p.x;
        poseData[count++] = t.p.y;
        poseData[count++] = t.p.z;

        poseData[count++] = t.q.w;
        poseData[count++] = t.q.x;
        poseData[count++] = t.q.y;
        poseData[count++] = t.q.z;
    }
}

///
/// \brief Set the time step size
///
void setTimeStepSize(const float timeStep)
{
    timeStepSize = timeStep;
}

///
/// \brief Check if the spine pose has converged with a preset tolerance
///
bool poseConverged()
{
    getSpinePose(poseDataCurrent);

    float norm = 0.;
    for (int i = 0; i < 168; ++i)
    {
        norm += std::abs(poseDataCurrent[i] - poseDataPrev[i]);
    }
    //printf("norm: %f\n", norm);

    if (norm > convergenceTol)
    {
        for (int i = 0; i < 168; ++i)
        {
            poseDataPrev[i] = poseDataCurrent[i];
        }
        return false;
    }
    else
    {
        return true;
    }
}

///
/// \brief Add a force at a given vertebrae
/// \param vertebraeNum vertebrae number [0, 23]
/// \param force force to be applied on the vertebrae
/// \param force Position w.r.t local vertebrae frame where the force will be applied
///
void addForceOnVertebrae(const unsigned int vertebraeNum, const PxVec3& force, const PxVec3& pos)
{
    if (vertebraeNum > 23)
    {
        printf("Warning: @addForceOnVertebrae - The Vertebrae number supplied is not valid!");
        return;
    }
    vertebraeForces.push_back(vertebraeForce(vertebraeNum, force, pos));
}

void addForceOnVertebrae(const unsigned int vertebraeNum, 
                         const float fX, const float fY, const float fZ,
                         const float pX, const float pY, const float pZ)
{
    addForceOnVertebrae(vertebraeNum, PxVec3(fX, fY, fZ), PxVec3(pX, pY, pZ));
}

///
/// \brief Clear all the forces on all the vertebrae
///
void clearAllForces()
{
    vertebraeForces.clear();
}

///
/// \brief Remove all the forces on a certain vertebrae
/// \param vertebraeNum vertebrae number [0, 23]
///
void removeForcesOnVertebrae(const unsigned int vertebraeNum)
{
    if (vertebraeNum > 23)
    {
        printf("Warning: @clearForceOnVertebrae - The Vertebrae number supplied is not valid!");
        return;
    }

    vertebraeForces.erase(std::remove_if(vertebraeForces.begin(), vertebraeForces.end(), [&vertebraeNum](const vertebraeForce& f)
    {
        return (f.vertebraeNumber == vertebraeNum) ? true : false;
    }));
}

///
/// \brief Apply the all the specified forces
///
void applyForces()
{
    for (const auto& f : vertebraeForces)
    {
        PxRigidBodyExt::addForceAtLocalPos(*vertebrae[f.vertebraeNumber], f.force, f.position, PxForceMode::eFORCE, true);
    }
}

///
/// \brief Returns the simulation state
///
spineSimulationState getSimulationState()
{
    return simState;
}

///
/// \brief Step one physics frame
///
void stepPhysics(bool /*interactive*/)
{
    if (firstIteration)
    {
        simState = spineSimulationState::movingToDefaultPose;
    }

    if (!paused)
    {
        if (frameCount > 2000)
        {        
            applyForces();
        }        
        frameCount++;
    }

    if (frameCount > 3000 && simState == spineSimulationState::movingToDefaultPose)
    {
        simState = spineSimulationState::movingToTargetPose;
    }

    if (frameCount > 3000 && !firstIteration && !paused && poseConverged())
    {
        paused = true;
        simState = spineSimulationState::ConvergedToTargetPose;
        printf("Converged!\n");
    }

    if (!paused)
    {
        gScene->simulate(timeStepSize);
        gScene->fetchResults(true);

        firstIteration = false;
    }
}

///
/// \brief Clean up
///
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
}

///
/// \brief Handle key presses
///
void keyPress(unsigned char key, const PxTransform& camera)
{
	switch(toupper(key))
	{
    case ' ':
        paused = !paused;
        break;
	}
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else	
	initPhysics(false);
    while (1)
    {
        stepPhysics(false);
    }		
	cleanupPhysics(false);
#endif

	return 0;
}