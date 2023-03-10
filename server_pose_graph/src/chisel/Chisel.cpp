// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "../../include/chisel/open_chisel/Chisel.h"

#include "../../include/chisel/open_chisel/io/PLY.h"

namespace chisel
{

Chisel::Chisel()
{
    // TODO Auto-generated constructor stub
}

Chisel::Chisel(const Eigen::Vector3i &chunkSize, float voxelResolution, bool useColor)
    : chunkManager(chunkSize, voxelResolution, useColor)
{
}

Chisel::~Chisel()
{
    // TODO Auto-generated destructor stub
}

void Chisel::Reset()
{
    chunkManager.Reset();
    meshesToUpdate.clear();
}

void Chisel::UpdateMeshes()
{
    printf("number of chunk to update: %lu\n", meshesToUpdate.size());
    static int cnt = 0;
    if (cnt++ % 10 == 0)
    {
        chunkManager.RecomputeMeshes(meshesToUpdate);
        meshesToUpdate.clear();
    }
}

void Chisel::UpdateMeshesInstantly()
{
    printf("number of chunk to update: %lu\n", meshesToUpdate.size());
    // static int cnt = 0;
    // if (cnt++ % 10 == 0)
    // {
    chunkManager.RecomputeMeshes(meshesToUpdate);
    meshesToUpdate.clear();
    // }
}


void Chisel::GarbageCollect(const ChunkIDList &chunks)
{
    for (const ChunkID &chunkID : chunks)
    {
        chunkManager.RemoveChunk(chunkID);
    }
}

bool Chisel::SaveAllMeshesToPLY(const std::string &filename)
{
    printf("Saving all meshes to PLY file...\n");

    chisel::MeshPtr fullMesh(new chisel::Mesh());

    size_t v = 0;
    for (const std::pair<ChunkID, MeshPtr> &it : chunkManager.GetAllMeshes())
    {
        for (const Vec3 &vert : it.second->vertices)
        {
            fullMesh->vertices.push_back(vert);
            fullMesh->indices.push_back(v);
            v++;
        }

        for (const Vec3 &color : it.second->colors)
        {
            fullMesh->colors.push_back(color);
        }

        for (const Vec3 &normal : it.second->normals)
        {
            fullMesh->normals.push_back(normal);
        }
    }

    printf("Full mesh has %lu verts\n", v);
    bool success = SaveMeshPLYASCII(filename, fullMesh);

    if (!success)
    {
        printf("Saving failed!\n");
    }

    return success;
}

void Chisel::IntegratePointCloud(const ProjectionIntegrator &integrator, const PointCloud &cloud, const Transform &extrinsic, float truncation, float maxDist)
{
    ChunkIDList chunksIntersecting;
    chunkManager.GetChunkIDsIntersecting(cloud, extrinsic, truncation, maxDist, &chunksIntersecting);
    printf("There are %lu chunks intersecting\n", chunksIntersecting.size());
    if (chunksIntersecting.size() == 0)
        return;
    std::mutex mutex;
    ChunkIDList garbageChunks;
    //for(const ChunkID& chunkID : chunksIntersecting)
    parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID &chunkID)
                 {
            bool chunkNew = false;

            mutex.lock();
            if (!chunkManager.HasChunk(chunkID))
            {
                chunkNew = true;
                chunkManager.CreateChunk(chunkID);
            }

            ChunkPtr chunk = chunkManager.GetChunk(chunkID);
            mutex.unlock();

            puts("integrate 1");
            bool needsUpdate = integrator.Integrate(cloud, extrinsic, chunk.get());
            puts("integrate 2");

            mutex.lock();
            if (needsUpdate)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        for (int dz = -1; dz <= 1; dz++)
                        {
                            meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
                        }
                    }
                }
            }
            else if(chunkNew)
            {
                garbageChunks.push_back(chunkID);
            }
            mutex.unlock();
                 });
    GarbageCollect(garbageChunks);
    chunkManager.PrintMemoryStatistics();
}

} // namespace chisel
