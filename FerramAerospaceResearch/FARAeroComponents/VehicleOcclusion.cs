using System;
using System.Collections.Generic;
using FerramAerospaceResearch.UnityJobs;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;
using UnityEngine.Profiling;

/* Theory of Job Control:
 * At beginning of FixedUpdate cycle, pre-calculate the occlusion orientations we need.
 * Convection occlusion calcuation and sun occlusion calculation are separate orientations.
 * Weighted-average / lerp the 3 closest orientations; this can be 6 raycasts on first execution if we missed all caches.
 * On exemplar system (4-core I5-6500 3.2GHz 16GB RAM):
 *   <2ms at FixedUpdate.ObscenelyEarly to gather the raycast data
 *  Expensive raycasts can cost ~4ms (10,000 rays), cheaper ones 0.5ms
 *
 * */
namespace FerramAerospaceResearch.FARAeroComponents
{
    public class VehicleOcclusion : MonoBehaviour
    {
        const int MAX_JOBS = 25;
        const int UPDATE_JOBS_PER_THREAD = 8;
        const int FIBONACCI_LATTICE_SIZE = 1000;

        // 1000 points produces typical misses 2-3 degrees, up to 5.5, outlier of 10.
        // Can try choosing better epsilon for lower average point distance instead of maximizing minimum point separation.

        private FARVesselAero farVesselAero;
        Vessel Vessel => farVesselAero?.Vessel;

        public bool running = false;
        private readonly Dictionary<Transform, Part> partsByTransform = new Dictionary<Transform, Part>();
        private readonly Dictionary<Part, DirectionalOcclusionInfo> partOcclusionInfo = new Dictionary<Part, DirectionalOcclusionInfo>();
        private NativeHashMap<quaternion, EMPTY_STRUCT> processedQuaternionsMap;

        private readonly SphereDistanceInfo[] convectionPoints = new SphereDistanceInfo[3];
        private readonly SphereDistanceInfo[] sunPoints = new SphereDistanceInfo[3];

        private quaternion[] Quaternions;
        private readonly Queue<quaternion> workQueue = new Queue<quaternion>();

        private readonly JobHandle[] setupJobs = new JobHandle[MAX_JOBS];
        private readonly JobHandle[] raycastJobs = new JobHandle[MAX_JOBS];
        private readonly JobHandle[] filterJobs = new JobHandle[MAX_JOBS];
        private readonly JobHandle[] raycastProcessorJobs = new JobHandle[MAX_JOBS];
        private readonly NativeArray<RaycastHit>[] workHitArrays = new NativeArray<RaycastHit>[MAX_JOBS];
        private readonly NativeArray<RaycastCommand>[] workCommandArrays = new NativeArray<RaycastCommand>[MAX_JOBS];
        private readonly NativeQueue<RaycastHit>[] workFilteredHits = new NativeQueue<RaycastHit>[MAX_JOBS];
        private readonly NativeHashMap<int, RaycastHitWithArea>[] workHitsMap = new NativeHashMap<int, RaycastHitWithArea>[MAX_JOBS];
        private readonly quaternion[] workQuaternions = new quaternion[MAX_JOBS];
        private readonly float[] workAreas = new float[MAX_JOBS];

        private Vector3 center = Vector3.zero;
        private Vector3 extents = new Vector3(10, 10, 10);

        private int jobsInProgress;
        private float averageMissOnBestAngle = 0;

        public void Setup(FARVesselAero farVesselAero)
        {
            this.farVesselAero = farVesselAero;
        }

        //Extents = axis-aligned bounding box dimensions of combined vehicle meshes
        public void SetVehicleBounds(Vector3d center, Vector3d extents)
        {
            this.center = center;
            this.extents = extents;
            FARLogger.Info($"[VehicleOcclusion] Learned Center {center} and Extents {extents}");
        }

        public void Reset()
        {
            FARLogger.Info($"[VehicleOcclusion] Resetting occlusion calculation data.");
            running = false;
            processedQuaternionsMap.Clear();
            partsByTransform.Clear();
            partOcclusionInfo.Clear();
            foreach (Part p in Vessel.Parts)
            {
                partsByTransform.Add(p.transform, p);
            }
        }

        public void Start()
        {
            FARLogger.Info($"VehicleOcclusion on {Vessel?.name} reporting startup");
            processedQuaternionsMap = new NativeHashMap<quaternion, EMPTY_STRUCT>(FIBONACCI_LATTICE_SIZE, Allocator.Persistent);

            if (Vessel.isActiveVessel)
            {
                Reset();
                Quaternions = new quaternion[FIBONACCI_LATTICE_SIZE];
                BuildSpherePoints();
                TimingManager.FixedUpdateAdd(TimingManager.TimingStage.ObscenelyEarly, FixedUpdateEarly);
                TimingManager.UpdateAdd(TimingManager.TimingStage.ObscenelyEarly, UpdateEarly);
                TimingManager.LateUpdateAdd(TimingManager.TimingStage.Late, LateUpdateComplete);
            }
        }

        public void OnDestroy()
        {
            processedQuaternionsMap.Dispose();
            TimingManager.FixedUpdateRemove(TimingManager.TimingStage.ObscenelyEarly, FixedUpdateEarly);
            TimingManager.UpdateRemove(TimingManager.TimingStage.ObscenelyEarly, UpdateEarly);
            TimingManager.LateUpdateRemove(TimingManager.TimingStage.Late, LateUpdateComplete);
        }

        public void BuildSpherePoints()
        {
            NativeArray<quaternion> arr = new NativeArray<quaternion>(FIBONACCI_LATTICE_SIZE, Allocator.Temp);
            SpherePointsJob parallelJob = new SpherePointsJob
            {
                points = FIBONACCI_LATTICE_SIZE,
                epsilon = Lattice_epsilon(FIBONACCI_LATTICE_SIZE),
                results = arr,
            };
            JobHandle h = parallelJob.Schedule(FIBONACCI_LATTICE_SIZE, 16);
            h.Complete();
            arr.CopyTo(Quaternions);
            arr.Dispose();
        }

        private void BuildWorkQueue(Transform t, Queue<quaternion> localWorkQueue, SphereDistanceInfo[] convectionPoints, SphereDistanceInfo[] sunPoints, int suggestedJobs = 1)
        {
            Profiler.BeginSample("VehicleOcclusion-BuildWorkQueue");
            Vector3 localVelocity = t.worldToLocalMatrix * (Vector3)Vessel.velocityD;
            if (localVelocity == Vector3.zero)
                localVelocity = Vector3.forward;
            NativeArray<quaternion> quats = new NativeArray<quaternion>(FIBONACCI_LATTICE_SIZE, Allocator.Temp);
            quats.CopyFrom(Quaternions);
            NativeArray<SphereDistanceInfo> convectionSphereDistances = new NativeArray<SphereDistanceInfo>(FIBONACCI_LATTICE_SIZE, Allocator.Temp);
            SphereSurfaceDistance convectionJob = new SphereSurfaceDistance
            {
                dir = localVelocity.normalized,
                rotations = quats,
                distance = convectionSphereDistances,
            };
            Vector3 localSunVec = t.worldToLocalMatrix * (Planetarium.fetch.Sun.transform.position - t.position);
            NativeArray <SphereDistanceInfo> sunSphereDistances = new NativeArray<SphereDistanceInfo>(FIBONACCI_LATTICE_SIZE, Allocator.Temp);
            SphereSurfaceDistance sunJob = new SphereSurfaceDistance
            {
                dir = localSunVec.normalized,
                rotations = quats,
                distance = sunSphereDistances,
            };

            NativeQueue<quaternion> filteredConvectionQuaternions = new NativeQueue<quaternion>(Allocator.Temp);
            FilterCompletedQuaternions filterJob = new FilterCompletedQuaternions
            {
                input = convectionSphereDistances,
                map = processedQuaternionsMap,
                output = filteredConvectionQuaternions.AsParallelWriter(),
            };

            JobHandle convHandle = convectionJob.Schedule(FIBONACCI_LATTICE_SIZE, 32);
            JobHandle sunHandle = sunJob.Schedule(FIBONACCI_LATTICE_SIZE, 32);
            JobHandle sortedConvectionHandle = NativeSortExtension.SortJob(convectionSphereDistances, convHandle);
            JobHandle sortedSunHandle = NativeSortExtension.SortJob(sunSphereDistances, sunHandle);
            JobHandle filteredSortedConvectionHandle = filterJob.Schedule(convectionSphereDistances.Length, 16, sortedConvectionHandle);
            JobHandle.ScheduleBatchedJobs();
            JobHandle.CompleteAll(ref sortedConvectionHandle, ref sortedSunHandle);

            localWorkQueue.Clear();
            for (int i = 0; i < 3; i++)
            {
                sunPoints[i] = sunSphereDistances[i];
                if (!processedQuaternionsMap.ContainsKey(sunSphereDistances[i].q))
                    localWorkQueue.Enqueue(sunSphereDistances[i].q);

                convectionPoints[i] = convectionSphereDistances[i];
                if (!processedQuaternionsMap.ContainsKey(convectionSphereDistances[i].q))
                    localWorkQueue.Enqueue(convectionSphereDistances[i].q);
            }

            filteredSortedConvectionHandle.Complete();

            while (localWorkQueue.Count < suggestedJobs && filteredConvectionQuaternions.Count > 0)
                localWorkQueue.Enqueue(filteredConvectionQuaternions.Dequeue());

            quats.Dispose();
            convectionSphereDistances.Dispose();
            sunSphereDistances.Dispose();
            filteredConvectionQuaternions.Dispose();
            Profiler.EndSample();
        }

        private Vector2 GetBoundaries(quaternion q)
        {
            // We now have center, extents, and the raycaster plane (normal = rotation * vessel.forward)
            // Project each of the 8 corners of the bounding box onto the plane.
            // https://stackoverflow.com/questions/9605556/how-to-project-a-point-onto-a-plane-in-3d/41897378#41897378

            Vector3 normal = math.mul(q, Vector3.forward);
            Vector3 origin = Vector3.zero + 10000 * normal;
            Vector2 min = new Vector2(float.PositiveInfinity, float.PositiveInfinity);
            Vector2 max = new Vector2(float.NegativeInfinity, float.NegativeInfinity);
            Vector3 ext = extents / 2;
            Vector3[] arr = new Vector3[8]
            {
                center - (ext/2),
                new Vector3(center.x - ext.x, center.y - ext.y, center.z + ext.z),
                new Vector3(center.x - ext.x, center.y + ext.y, center.z - ext.z),
                new Vector3(center.x - ext.x, center.y + ext.y, center.z + ext.z),
                new Vector3(center.x + ext.x, center.y - ext.y, center.z - ext.z),
                new Vector3(center.x + ext.x, center.y - ext.y, center.z + ext.z),
                new Vector3(center.x + ext.x, center.y + ext.y, center.z - ext.z),
                center + (ext/2),
            };

            for (int i=0; i<8; i++)
            {
                // Project the bounding box onto the plane
                float dist = math.dot(arr[i] - origin, normal);
                Vector3 projection = arr[i] - dist * normal;
                // Rotate the projection back towards the vessel alignment
                projection = math.mul(math.inverse(q), projection);

                // The x,y coords of projection are again aligned to the vessel axes.
                min.x = Math.Min(min.x, projection.x);
                min.y = Math.Min(min.y, projection.y);
                max.x = Math.Max(max.x, projection.x);
                max.y = Math.Max(max.y, projection.y);
            }
            return new float2(math.abs(max.x - min.x), math.abs(max.y - min.y));
        }

        public void ProcessRaycastResults(ref NativeHashMap<int, RaycastHitWithArea> hits, Quaternion rotation)
        {
            Profiler.BeginSample("VehicleOcclusion-ProcessRaycastResults");
            NativeArray<int> keys = hits.GetKeyArray(Allocator.Temp);
            for (int i=0; i<keys.Length; i++)
            {
                if (hits.TryGetValue(keys[i], out RaycastHitWithArea info) && info.hit.transform is Transform t)
                {
                    while (t.parent != null) t = t.parent;
                    if (partsByTransform.ContainsKey(t) && partsByTransform[t] is Part p)
                    {
                        if (!partOcclusionInfo.ContainsKey(p))
                        {
                            partOcclusionInfo.Add(p, new DirectionalOcclusionInfo());
                        }
                        DirectionalOcclusionInfo occlInfo = partOcclusionInfo[p];
                        if (!occlInfo.convectionArea.ContainsKey(rotation))
                        {
                            occlInfo.convectionArea.Add(rotation, 0);
                        }
                        occlInfo.convectionArea[rotation] += info.area;
                    }
                    else
                    {
                        Debug.LogWarning($"[VehicleOcclusion.ProcessRaycastResults] {info.hit.transform} ancestor {t} not associated with any part!");
                    }
                }
            }
            keys.Dispose();
            Profiler.EndSample();
        }

        public void FixedUpdateEarly()
        {
            if (!farVesselAero.isValid || FlightGlobals.ActiveVessel != Vessel)
                return;

            // We must call this to pre-calculate the nearest quaternions for sun and convection.
            // It may return an empty work queue if we are not otherwise running, which is fine.
            BuildOcclusionJobs();
        }

        public void FixedUpdate()
        {
            if (!farVesselAero.isValid || FlightGlobals.ActiveVessel != Vessel)
                return;
            HandleJobCompletion();
        }

        public void UpdateEarly()
        {
            if (!farVesselAero.isValid || FlightGlobals.ActiveVessel != Vessel || !running)
                return;
            int threadCount = System.Environment.ProcessorCount - 1;
            int updateJobs = math.min(MAX_JOBS, math.max(1, UPDATE_JOBS_PER_THREAD * threadCount));
            BuildOcclusionJobs(updateJobs);
        }

        public void LateUpdateComplete()
        {
            if (!farVesselAero.isValid || FlightGlobals.ActiveVessel != Vessel)
                return;
            HandleJobCompletion();
            if (!running)
                Reset();
        }

        public void BuildOcclusionJobs(int suggestedJobs = 1)
        {
            Profiler.BeginSample("VehicleOcclusion-BuildOcclusionJobs");

            if (Quaternions.Length != FIBONACCI_LATTICE_SIZE)
                BuildSpherePoints();

            BuildWorkQueue(Vessel.transform, workQueue, convectionPoints, sunPoints, suggestedJobs);

            int i = 0;
            Transform t = Vessel.transform;
            while (workQueue.Count > 0)
            {
                quaternion q = workQueue.Dequeue();
                float2 bounds = GetBoundaries(q);
                float2 interval = new float2(math.max(bounds.x / 100, 0.1f), math.max(bounds.y / 100, 0.1f));

                int minXsize = Mathf.CeilToInt(bounds.x / 0.1f);
                int minYsize = Mathf.CeilToInt(bounds.y / 0.1f);
                int elements = math.min(100, minXsize) * math.min(100, minYsize);

                workCommandArrays[i] = new NativeArray<RaycastCommand>(elements, Allocator.TempJob);
                workQuaternions[i] = q;
                workAreas[i] = interval.x * interval.y;
                OcclusionRaycastBuilder job = new OcclusionRaycastBuilder
                {
                    startPosition = t.TransformPoint(t.position),
                    offset = extents.magnitude,
                    forwardDir = t.TransformDirection(math.mul(q, Vector3.forward)),
                    rightDir = t.TransformDirection(math.mul(q, Vector3.right)),
                    upDir = t.TransformDirection(math.mul(q, Vector3.up)),
                    dim_x = math.min(100, minXsize),
                    dim_y = math.min(100, minYsize),
                    interval = interval,
                    commands = workCommandArrays[i],
                };

                workHitArrays[i] = new NativeArray<RaycastHit>(elements, Allocator.TempJob);
                workFilteredHits[i] = new NativeQueue<RaycastHit>(Allocator.TempJob);
                RaycastFilterJob filter = new RaycastFilterJob
                {
                    count = elements,
                    hitsIn = workHitArrays[i],
                    hitsOut = workFilteredHits[i].AsParallelWriter(),
                };

                workHitsMap[i] = new NativeHashMap<int, RaycastHitWithArea>(elements, Allocator.TempJob);
                RaycastProcessorJob processor = new RaycastProcessorJob
                {
                    area = workAreas[i],
                    hitsIn = workFilteredHits[i],
                    hitsOut = workHitsMap[i],
                };

                setupJobs[i] = job.Schedule(elements, 1);
                raycastJobs[i] = RaycastCommand.ScheduleBatch(workCommandArrays[i], workHitArrays[i], 1, setupJobs[i]);
                filterJobs[i] = filter.Schedule(workHitArrays[i].Length, 16, raycastJobs[i]);
                raycastProcessorJobs[i] = processor.Schedule(filterJobs[i]);
                i++;
            }
            jobsInProgress = i;
            JobHandle.ScheduleBatchedJobs();
            Profiler.EndSample();
        }

        public void HandleJobCompletion()
        {
            for (int i = 0; i < jobsInProgress; i++)
            {
                raycastProcessorJobs[i].Complete();
                ProcessRaycastResults(ref workHitsMap[i], workQuaternions[i]);
                processedQuaternionsMap.TryAdd(workQuaternions[i], new EMPTY_STRUCT { });
                workHitArrays[i].Dispose();
                workCommandArrays[i].Dispose();
                workFilteredHits[i].Dispose();
                workHitsMap[i].Dispose();
            }

            running = jobsInProgress > 0;
            if (!running)
                running = false;        // Do nothing, just a breakpoint target
        }

        // http://extremelearning.com.au/how-to-evenly-distribute-points-on-a-sphere-more-effectively-than-the-canonical-fibonacci-lattice/
        private float Lattice_epsilon(int n)
        {
            // Basic episol setting
            /*
            if (n < 24)
                return (1f/3);
            else if (n < 177)
                return (4f/3);
            else if (n < 890)
                return (10f/3);
            else if (n < 11000)
                return 10;
            else if (n < 39000)
                return 27;
            else if (n < 600000)
                return 75;
            else
                return 214;
            */
            // Corollary.Preventing large gaps
            /*
            if (n < 80)
                return 8f / 3;
            else if (n < 1000)
                return 10f / 3;
            else if (n < 40000)
                return 10;
            else
                return 25;
                */
            // 3.Optimizing average nearest-neighbor distance
            return 0.36f;
        }

        public float SunArea(Part p, Vector3 dir)
        {
            if (partOcclusionInfo.ContainsKey(p) && partOcclusionInfo[p] is DirectionalOcclusionInfo info)
                return Area(p, sunPoints, info.area);
            return 0;
        }

        int counter = 0;
        public float ConvectionArea(Part p, Vector3 dir)
        {
            // convectionPoints is a set of 3 quaternions that should be closest to dir.
            // (They were precalculated, if dir is NOT what we expect for occlusion... that's bad.)
            Vector3 localVelocity = Vessel.transform.worldToLocalMatrix * dir;
            Quaternion q = Quaternion.FromToRotation(Vector3.forward, localVelocity);
            float a = 0.25f;

            if (partOcclusionInfo.ContainsKey(p) && partOcclusionInfo[p] is DirectionalOcclusionInfo info)
            {
                float angle = Quaternion.Angle(q, convectionPoints[0].q);
                averageMissOnBestAngle = (a * angle) + ((1f - a) * averageMissOnBestAngle);
                //                if (counter++ % 50 == 0)
                //                    Debug.Log($"[VehicleOcclusion] {p} {localVelocity} missed {angle} [average {averageMissOnBestAngle}]");
                return Area(p, convectionPoints, info.convectionArea);
            }
            return 0;
        }

        public float Area(Part p, SphereDistanceInfo[] sdiArray, Dictionary<Quaternion, float> areaInfos)
        {
            float area = 0;
            float distanceSum = 0;
            foreach (SphereDistanceInfo sdi in sdiArray)
            {
                if (areaInfos.ContainsKey(sdi.q))
                {
                    if (sdi.sqrDistance < float.Epsilon)
                        return areaInfos[sdi.q];
                    area += areaInfos[sdi.q] / sdi.sqrDistance;
                    distanceSum += (1 / sdi.sqrDistance);
                }
                else if (processedQuaternionsMap.ContainsKey(sdi.q))
                {
                    // Processed quaternion but no per-part data means 0 area [no raycasters hit]
                    distanceSum += (1 / sdi.sqrDistance);
                }
                else
                {
                    Debug.LogWarning($"[VehicleOcclusion.Area] Quaternion {sdi.q} was in top-3 but had not been processed!" +
                        $"\nTop 3: {sdiArray[0].q} | {sdiArray[1].q} | {sdiArray[2].q}");
                }
            }
            return area / distanceSum;
        }

        private class DirectionalOcclusionInfo
        {
            public Dictionary<Quaternion, float> convectionArea;
            public Dictionary<Quaternion, float> area;

            public DirectionalOcclusionInfo()
            {
                convectionArea = new Dictionary<Quaternion, float>();
                area = new Dictionary<Quaternion, float>();
            }
        }
    }
}
