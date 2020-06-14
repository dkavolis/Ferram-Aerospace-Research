using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace FerramAerospaceResearch.UnityJobs
{
    public struct SphereDistanceInfo : IComparable<SphereDistanceInfo>
    {
        public quaternion q;
        public float sqrDistance;

        public int CompareTo(SphereDistanceInfo obj)
        {
            return sqrDistance.CompareTo(obj.sqrDistance);
        }
    }

    public struct EMPTY_STRUCT { }

    //http://extremelearning.com.au/how-to-evenly-distribute-points-on-a-sphere-more-effectively-than-the-canonical-fibonacci-lattice/
    [BurstCompile]
    public struct SpherePointsJob : IJobParallelFor
    {
        [ReadOnly] public int points;
        [ReadOnly] public float epsilon;
        [WriteOnly] public NativeArray<quaternion> results;
        public void Execute(int index)
        {
            float goldenRatio = (1 + math.sqrt(5)) / 2;
            float thetaConstantTerm = 2 * math.PI / goldenRatio;
            if (index == 0)
            {
                results[index] = quaternion.identity;
            }
            else if (index == points - 1)
            {
                results[index] = quaternion.Euler(180, 0, 0);
            }
            else
            {
                float theta = index * thetaConstantTerm;
                float denom = points - 1 + (2 * epsilon);
                float num = 2 * (index + epsilon);
                float phi = math.acos(1 - num / denom);
                float3 res = new float3(math.cos(theta) * math.sin(phi),
                                        math.sin(theta) * math.sin(phi),
                                        math.cos(phi));
                results[index] = Quaternion.FromToRotation(new float3(0,0,1), res);
            }
        }
    }

    [BurstCompile]
    public struct SphereSurfaceDistance : IJobParallelFor
    {
        [ReadOnly] public float3 dir;
        [ReadOnly] public NativeArray<quaternion> rotations;
        [WriteOnly] public NativeArray<SphereDistanceInfo> distance;

        public void Execute(int index)
        {
            float3 rot_dir = math.mul(rotations[index], new float3(0, 0, 1));
            distance[index] = new SphereDistanceInfo
            {
                q = rotations[index],
                sqrDistance = math.distancesq(dir, rot_dir),
            };
        }
    }

    [BurstCompile]
    public struct FilterCompletedQuaternions : IJob
    {
        [ReadOnly] public int count;
        [ReadOnly] public NativeArray<SphereDistanceInfo> input;
        [ReadOnly] public NativeHashMap<quaternion, EMPTY_STRUCT> map;
        [WriteOnly] public NativeList<quaternion> output;

        public void Execute()
        {
            for (int i=0; i<count; i++)
            {
                if (!map.ContainsKey(input[i].q))
                    output.Add(input[i].q);
            }
        }
    }

    [BurstCompile]
    public struct RaycastFilterJob : IJob
    {
        [ReadOnly] public int count;
        [ReadOnly] public NativeArray<RaycastHit> hitsIn;
        [WriteOnly] public NativeList<RaycastHit> hitsOut;

        public void Execute()
        {
            unsafe
            {
                RaycastHit* array_ptr = (RaycastHit*)hitsIn.GetUnsafeReadOnlyPtr();
                for (int index = 0; index < count; index++)
                {
                    int collider_id = *(int*)((byte*)&array_ptr[index] + 40);
                    if (collider_id != 0)
                        hitsOut.Add(array_ptr[index]);
                }
            }
        }
    }

    [BurstCompile]
    public struct OcclusionRaycastBuilder : IJobParallelFor
    {
        [ReadOnly] public float3 forwardDir;
        [ReadOnly] public float3 rightDir;
        [ReadOnly] public float3 upDir;
        [ReadOnly] public float3 startPosition;
        [ReadOnly] public float offset;
        [ReadOnly] public int dim_x;
        [ReadOnly] public int dim_y;
        [ReadOnly] public float2 interval;
        public NativeArray<RaycastHit> results;
        public NativeArray<RaycastCommand> commands;

        public void Execute(int index)
        {
            // Everything is already rotated by the quaternion rotation and transformed to worldspace
            // Iterate raycasts in X, Y

            float3 origin = startPosition + offset * forwardDir;
            int row = index / dim_x;
            int col = index % dim_x;
            float row_eff = row - (dim_x / 2);
            float col_eff = col - (dim_y / 2);

            // All raycasts are in the same direction.
            float3 pos = origin + (row_eff * interval.x * upDir) + (col_eff * interval.y * rightDir);
            commands[index] = new RaycastCommand(pos, -forwardDir, distance: 1e5f, layerMask: 19, maxHits: 1);
        }
    }
}
