using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace FerramAerospaceResearch.UnityJobs
{
    [BurstCompile]
    public struct SphereDistanceInfo : IComparable<SphereDistanceInfo>, IEquatable<SphereDistanceInfo>
    {
        public quaternion q;
        public float distance;

        public int CompareTo(SphereDistanceInfo obj)
        {
            return distance.CompareTo(obj.distance);
        }

        public bool Equals(SphereDistanceInfo other)
        {
            return q.Equals(other.q) && distance.Equals(other.distance);
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
    public struct SetBoundsJob : IJobParallelFor
    {
        [ReadOnly] public float3 boundsCenter;
        [ReadOnly] public float3 extents;
        [ReadOnly] public NativeArray<quaternion> quaternions;
        [WriteOnly] public NativeArray<float2> bounds;

        // We now have center, extents, and the raycaster plane (normal = rotation * vessel.forward)
        // Project each of the 8 corners of the bounding box onto the plane.
        // https://stackoverflow.com/questions/9605556/how-to-project-a-point-onto-a-plane-in-3d/41897378#41897378
        public void Execute(int index)
        {
            float3 normal = math.mul(quaternions[index], new float3(0, 0, 1));
            float3 origin = float3.zero + 10000 * normal;
            float2 min = new float2(float.PositiveInfinity, float.PositiveInfinity);
            float2 max = new float2(float.NegativeInfinity, float.NegativeInfinity);
            float3 ext = extents / 2;
            float3 center = boundsCenter;
            unsafe
            {
                float3* arr = stackalloc[]
                {
                    center - (ext / 2),
                    new float3(center.x - ext.x, center.y - ext.y, center.z + ext.z),
                    new float3(center.x - ext.x, center.y + ext.y, center.z - ext.z),
                    new float3(center.x - ext.x, center.y + ext.y, center.z + ext.z),
                    new float3(center.x + ext.x, center.y - ext.y, center.z - ext.z),
                    new float3(center.x + ext.x, center.y - ext.y, center.z + ext.z),
                    new float3(center.x + ext.x, center.y + ext.y, center.z - ext.z),
                    center + (ext / 2),
                };

                for (int i = 0; i < 8; i++)
                {
                    // Project the bounding box onto the plane
                    float dist = math.dot(arr[i] - origin, normal);
                    float3 projection = arr[i] - dist * normal;
                    // Rotate the projection back towards the vessel alignment
                    projection = math.mul(math.inverse(quaternions[index]), projection);

                    // The x,y coords of projection are again aligned to the vessel axes.
                    min.x = math.min(min.x, projection.x);
                    min.y = math.min(min.y, projection.y);
                    max.x = math.max(max.x, projection.x);
                    max.y = math.max(max.y, projection.y);
                }
            }

            bounds[index] = new float2(math.abs(max.x - min.x), math.abs(max.y - min.y));
        }
    }

    [BurstCompile]
    public struct SetIntervalAndDimensionsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float2> bounds;
        [WriteOnly] public NativeArray<float2> interval;
        [WriteOnly] public NativeArray<int2> dims;

        public void Execute(int index)
        {
            int minXsize = (int)math.ceil(bounds[index].x / 0.1f);
            int minYsize = (int)math.ceil(bounds[index].y / 0.1f);
            dims[index] = new int2(math.min(100, minXsize), math.min(100, minYsize));

            interval[index] = new float2(math.max(bounds[index].x / 100, 0.1f), math.max(bounds[index].y / 100, 0.1f));
        }
    }

    [BurstCompile]
    public struct SetVectorsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<quaternion> quaternions;
        [ReadOnly] public float4x4 localToWorldMatrix;
        [WriteOnly] public NativeArray<float3> forward;
        [WriteOnly] public NativeArray<float3> up;
        [WriteOnly] public NativeArray<float3> right;

        public void Execute(int index)
        {
            float3 dir = math.mul(quaternions[index], new float3(0, 0, 1));
            float4 tmp = math.mul(localToWorldMatrix, new float4(dir.x, dir.y, dir.z, 0));
            forward[index] = new float3(tmp.x, tmp.y, tmp.z);

            dir = math.mul(quaternions[index], new float3(0, 1, 0));
            tmp = math.mul(localToWorldMatrix, new float4(dir.x, dir.y, dir.z, 0));
            up[index] = new float3(tmp.x, tmp.y, tmp.z);

            dir = math.mul(quaternions[index], new float3(1 ,0, 0));
            tmp = math.mul(localToWorldMatrix, new float4(dir.x, dir.y, dir.z, 0));
            right[index] = new float3(tmp.x, tmp.y, tmp.z);
        }
    }

    [BurstCompile]
    public struct AngleBetween : IJobParallelFor
    {
        [ReadOnly] public float3 dir;
        [ReadOnly] public NativeArray<quaternion> rotations;
        [WriteOnly] public NativeArray<float> angles;

        public void Execute(int index)
        {
            float3 rot_dir = math.mul(rotations[index], new float3(0, 0, 1));
            angles[index] = math.degrees(math.acos(math.min(math.max(math.dot(dir, rot_dir), -1f), 1f)));
        }
    }

    [BurstCompile]
    public struct GeneralDistanceInfo : IJob
    {
        [ReadOnly] public NativeArray<float> distances;
        [WriteOnly] public NativeArray<float3> output;

        public void Execute()
        {
            float min = float.PositiveInfinity;
            float max = 0;
            float total = 0;
            for (int i = 0; i < distances.Length; i++)
            {
                total += distances[i];
                max = math.max(max, distances[i]);
                min = math.min(min, distances[i]);
            }
            output[0] = new float3(min, max, total / distances.Length);
        }
    }

    [BurstCompile]
    public struct FilterByDistanceJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<quaternion> quaternions;
        [ReadOnly] public NativeArray<float> distances;
        [ReadOnly] public float cutoffDistance;
        [WriteOnly] public NativeMultiHashMap<int, SphereDistanceInfo>.ParallelWriter map;

        public void Execute(int index)
        {
            if (distances[index] < cutoffDistance)
                map.Add(1, new SphereDistanceInfo { q = quaternions[index], distance = distances[index] });
        }
    }

    [BurstCompile]
    public struct GetSortedListFromMultiHashmap : IJob
    {
        [ReadOnly] public NativeMultiHashMap<int, SphereDistanceInfo> map;
        public NativeList<SphereDistanceInfo> list;
        public void Execute()
        {
            if (map.Length > 0)
            {
                NativeArray<SphereDistanceInfo> tmp = map.GetValueArray(Allocator.Temp);
                for (int i = 0; i < tmp.Length; i++)
                    list.Add(tmp[i]);
                NativeSortExtension.Sort(list);
                tmp.Dispose();
            }
        }
    }

    [BurstCompile]
    public struct MakeQuaternionIndexMapJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<quaternion> arr;
        [WriteOnly] public NativeHashMap<quaternion, int>.ParallelWriter map;

        public void Execute(int index)
        {
            map.TryAdd(arr[index], index);
        }
    }

    [BurstCompile]
    public struct MakeIndexMapJob : IJob
    {
        [ReadOnly] public NativeArray<SphereDistanceInfo> quaternions;
        [WriteOnly] public NativeHashMap<quaternion, int>.ParallelWriter map;

        public void Execute()
        {
            for (int i = 0; i < quaternions.Length; i++)
                map.TryAdd(quaternions[i].q, i);
        }
        public void Execute(int index)
        {
            map.TryAdd(quaternions[index].q, index);
        }
    }

    [BurstCompile]
    public struct BucketSortByPriority : IJobParallelFor
    {
        [ReadOnly] public NativeArray<quaternion> quaternions;
        [ReadOnly] public NativeHashMap<quaternion, int> priorityMap1;
        [ReadOnly] public NativeHashMap<quaternion, int> priorityMap2;
        [ReadOnly] public NativeHashMap<quaternion, EMPTY_STRUCT> completed;
        [ReadOnly] public int maxIndexForPriority0;
        [WriteOnly] public NativeMultiHashMap<int, int>.ParallelWriter map;

        public void Execute(int index)
        {
            if (!completed.ContainsKey(quaternions[index]))
            {
                int pri = 2;
                if (priorityMap1.TryGetValue(quaternions[index], out int i))
                    pri = math.min(pri, (i <= maxIndexForPriority0) ? 0 : 1);
                if (priorityMap2.TryGetValue(quaternions[index], out int j))
                    pri = math.min(pri, (j <= maxIndexForPriority0) ? 0 : 1);
                map.Add(pri, index);
            }
        }
    }

    [BurstCompile]
    public struct ClearNativeHashMapIntFloat : IJob
    {
        public NativeHashMap<int, float> map;
        public void Execute() => map.Clear();
    }

    [BurstCompile]
    public struct ClearNativeMultiHashMapIntInt : IJob
    {
        public NativeMultiHashMap<int, int> map;
        public void Execute() => map.Clear();
    }

    // hitsMap is the mapping from collider to all of the indicies in the hitsIn array.
    [BurstCompile]
    public struct RaycastPrepareJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<RaycastHit> hitsIn;
        [WriteOnly] public NativeMultiHashMap<int, int>.ParallelWriter hitsMap;

        public void Execute(int index)
        {
            unsafe
            {
                RaycastHit* array_ptr = (RaycastHit*)hitsIn.GetUnsafeReadOnlyPtr();
                int collider_id = *(int*)((byte*)&array_ptr[index] + 40);
                if (collider_id != 0)
                    hitsMap.Add(collider_id, index);
            }
        }
    }

    // hits is the array of all Raycasts
    // hitMap is the mapping from collider to all of the indicies in the hits array.
    [BurstCompile]
    public struct RaycastProcessorJob : IJobNativeMultiHashMapMergedSharedKeyIndices
    {
        [ReadOnly] public NativeArray<RaycastHit> hits;
        [ReadOnly] public NativeMultiHashMap<int, int> hitMap;
        [ReadOnly] public float area;
        [WriteOnly] public NativeHashMap<int, float>.ParallelWriter hitsOut;
        public NativeArray<float> areaSum;

        // index is the *value* in the MultiHashMap hitMap
        // The key generating this is a RaycastHitSummary = Tuple<quaternion,collider_id>
        public void ExecuteFirst(int index)
        {
            unsafe
            {
                areaSum[index] = area;
                hitsOut.TryAdd(index, area);
            }
        }

        public void ExecuteNext(int firstIndex, int index)
        {
            float t = areaSum[firstIndex];
            areaSum[firstIndex] = t + area;
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
        [ReadOnly] public int2 dimensions;
        [ReadOnly] public float2 interval;
        [WriteOnly] public NativeArray<RaycastCommand> commands;

        public void Execute(int index)
        {
            // Everything is already rotated by the quaternion rotation and transformed to worldspace
            // Iterate raycasts in X, Y
            float3 origin = startPosition + offset * forwardDir;
            int row = index / dimensions.x;
            int col = index % dimensions.x;
            float row_eff = row - (dimensions.x / 2);
            float col_eff = col - (dimensions.y / 2);

            // All raycasts are in the same direction.
            float3 pos = origin + (row_eff * interval.x * upDir) + (col_eff * interval.y * rightDir);
            commands[index] = new RaycastCommand(pos, -forwardDir, distance: 1e5f, layerMask: 19, maxHits: 1);
        }
    }
}
