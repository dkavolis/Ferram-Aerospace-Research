using System.Collections;
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
        const int MAX_JOBS = 10;
        const int UPDATE_JOBS_PER_THREAD = 3;
        const int FIBONACCI_LATTICE_SIZE = 2000;
        const float RESET_INTERVAL_SEC = 30;
        const double ZERO_VELOCITY_THRESHOLD = 0.01;    // Velocity < threshold will be treated as 0 for vessel travel direction
        // 1000 points produces typical misses 2-3 degrees when optimized for average miss distance.

        public enum State { Invalid, Initialized, Running, Completed }
        public State state = State.Invalid;
        private bool resetCoroutineRunning = false;
        private IEnumerator resetWaitCoroutine;

        private FARVesselAero farVesselAero;
        //private Vessel Vessel => farVesselAero?.Vessel;
        public Vessel Vessel { get; private set; }
        bool OnValidPhysicsVessel => farVesselAero && Vessel && !Vessel.packed;

        private readonly Dictionary<Transform, Part> partsByTransform = new Dictionary<Transform, Part>();
        private readonly Dictionary<Part, DirectionalOcclusionInfo> partOcclusionInfo = new Dictionary<Part, DirectionalOcclusionInfo>();

        // Quaternions is the full list of orientations for measurement
        // processedQuaternionsMap is the map of elements in Quaternions that have been processed
        // quaternionWorkList is the list of quaternions to process on this specific pass.
        private NativeHashMap<quaternion, EMPTY_STRUCT> processedQuaternionsMap;
        private NativeArray<quaternion> Quaternions;
        private NativeList<quaternion> quaternionWorkList = new NativeList<quaternion>(MAX_JOBS, Allocator.Persistent);

        // Closest 3 convection and sun quaternions for each FixedUpdate
        private readonly SphereDistanceInfo[] convectionPoints = new SphereDistanceInfo[3];
        private readonly SphereDistanceInfo[] sunPoints = new SphereDistanceInfo[3];

        // VehicleVoxel center and extents
        private Vector3 center = Vector3.zero;
        private Vector3 extents = new Vector3(10, 10, 10);

        private int jobsInProgress;
        private float averageMissOnBestAngle = 0;

        // Large long-term allocations, memory re-used in each
        // Cleaned and Allocated in ResetCalculations().  Disposed in OnDestroy()
        NativeMultiHashMap<int, int> indexedPriorityMap;

        private NativeArray<RaycastCommand>[] allCasts;
        private NativeArray<RaycastHit>[] allHits;
        private NativeMultiHashMap<int, int>[] allHitsMaps;
        private NativeHashMap<int, float>[] allHitSizeMaps;
        private NativeArray<float>[] allSummedHitAreas;

        // allCasts => allHits
        // allHits => allHitMaps
        // allHitMaps => allHitSizeMaps,
        //               allSummedHitAreas,

        private RaycastJobInfo[] raycastJobTracker;

        public void Setup(FARVesselAero farVesselAero)
        {
            this.farVesselAero = farVesselAero;
            Vessel = farVesselAero.Vessel;
        }

        //Extents = axis-aligned bounding box dimensions of combined vehicle meshes
        public void SetVehicleBounds(Vector3d center, Vector3d extents)
        {
            this.center = center;
            this.extents = extents;
            FARLogger.Info($"[VehicleOcclusion] {Vessel?.name} Learned Center {center} and Extents {extents}");
        }

        private void DisposeLongTermAllocations()
        {
            if (state != State.Invalid)
            {
                Quaternions.Dispose();
                processedQuaternionsMap.Dispose();
                indexedPriorityMap.Dispose();

                foreach (var x in allCasts)
                    x.Dispose();
                foreach (var x in allHits)
                    x.Dispose();
                foreach (var x in allHitsMaps)
                    x.Dispose();
                foreach (var x in allSummedHitAreas)
                    x.Dispose();
                foreach (var x in allHitSizeMaps)
                    x.Dispose();
            }
        }

        public void ResetCalculations()
        {
            FARLogger.Info($"[VehicleOcclusion] {Vessel?.name} Resetting occlusion calculation data.");
            if (resetCoroutineRunning && resetWaitCoroutine != null)
            {
                StopCoroutine(resetWaitCoroutine);
                resetCoroutineRunning = false;
            }

            partsByTransform.Clear();
            partOcclusionInfo.Clear();
            DisposeLongTermAllocations();
            state = State.Initialized;

            allCasts = new NativeArray<RaycastCommand>[MAX_JOBS];
            allHits = new NativeArray<RaycastHit>[MAX_JOBS];
            allHitsMaps = new NativeMultiHashMap<int, int>[MAX_JOBS];
            allSummedHitAreas = new NativeArray<float>[MAX_JOBS];
            allHitSizeMaps = new NativeHashMap<int, float>[MAX_JOBS];

            raycastJobTracker = new RaycastJobInfo[MAX_JOBS];

            indexedPriorityMap = new NativeMultiHashMap<int, int>(FIBONACCI_LATTICE_SIZE, Allocator.Persistent);
            Quaternions = new NativeArray<quaternion>(FIBONACCI_LATTICE_SIZE, Allocator.Persistent);
            var handle = new SpherePointsJob
            {
                points = FIBONACCI_LATTICE_SIZE,
                epsilon = Lattice_epsilon(FIBONACCI_LATTICE_SIZE),
                results = Quaternions,
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16);
            JobHandle.ScheduleBatchedJobs();

            processedQuaternionsMap = new NativeHashMap<quaternion, EMPTY_STRUCT>(FIBONACCI_LATTICE_SIZE, Allocator.Persistent);
            for (int i=0; i<MAX_JOBS; i++)
            {
                allCasts[i] = new NativeArray<RaycastCommand>(10000, Allocator.Persistent);
                allHits[i] = new NativeArray<RaycastHit>(10000, Allocator.Persistent);
                allHitsMaps[i] = new NativeMultiHashMap<int, int>(10000, Allocator.Persistent);
                allSummedHitAreas[i] = new NativeArray<float>(10000, Allocator.Persistent);
                allHitSizeMaps[i] = new NativeHashMap<int, float>(10000, Allocator.Persistent);
            }
            foreach (Part p in Vessel.Parts)
            {
                partsByTransform.Add(p.transform, p);
            }
            handle.Complete();
        }

        public void Start()
        {
            FARLogger.Info($"VehicleOcclusion on {Vessel?.name} reporting startup");
            state = State.Invalid;
            TimingManager.FixedUpdateAdd(TimingManager.TimingStage.ObscenelyEarly, FixedUpdateEarly);
            TimingManager.UpdateAdd(TimingManager.TimingStage.ObscenelyEarly, UpdateEarly);
            TimingManager.LateUpdateAdd(TimingManager.TimingStage.Late, LateUpdateComplete);
        }

        public void OnDestroy()
        {
            quaternionWorkList.Dispose();
            DisposeLongTermAllocations();

            TimingManager.FixedUpdateRemove(TimingManager.TimingStage.ObscenelyEarly, FixedUpdateEarly);
            TimingManager.UpdateRemove(TimingManager.TimingStage.ObscenelyEarly, UpdateEarly);
            TimingManager.LateUpdateRemove(TimingManager.TimingStage.Late, LateUpdateComplete);
        }

        private JobHandle LaunchAngleJobs(Vector3 dir, ref NativeArray<float> angles, ref NativeArray<float3> angleStats)
        {
            var angleJob = new AngleBetween
            {
                dir = dir,
                rotations = Quaternions,
                angles = angles,
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16);

            var statsJob = new GeneralDistanceInfo
            {
                distances = angles,
                output = angleStats,
            }.Schedule(angleJob);

            JobHandle.ScheduleBatchedJobs();
            return statsJob;
        }

        private JobHandle LaunchFilterAndSortJob(
            ref NativeArray<float> angles,
            float cutoff,
            ref NativeList<SphereDistanceInfo> sdiList,
            ref NativeList<quaternion> orderedAngles)
        {
            var handle = new SortedFilterAndMapByDistanceJob
            {
                quaternions = Quaternions,
                distances = angles,
                cutoffDistance = cutoff,
                sdiList = sdiList,
                orderedQuaternions = orderedAngles,
            }.Schedule();
            JobHandle.ScheduleBatchedJobs();
            return handle;
        }

        private void LaunchRaycastJobs(
            float4x4 localToWorldMatrix,
            ref NativeHashMap<quaternion, int> fullQuaternionIndexMap,
            ref NativeArray<float2> boundsArr,
            ref NativeArray<float2> intervalArr,
            ref NativeArray<int2> dimensionsArr,
            ref NativeArray<float3> forwardArr,
            ref NativeArray<float3> rightArr,
            ref NativeArray<float3> upArr,
            out JobHandle indexMapJob,
            out JobHandle vectorJob,
            out JobHandle intervalJob)
        {
            indexMapJob = new MakeQuaternionIndexMapJob
            {
                arr = Quaternions,
                map = fullQuaternionIndexMap.AsParallelWriter(),
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16);

            vectorJob = new SetVectorsJob
            {
                quaternions = Quaternions,
                localToWorldMatrix = localToWorldMatrix,
                forward = forwardArr,
                up = upArr,
                right = rightArr,
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16);

            var setBounds = new SetBoundsJob
            {
                boundsCenter = center,
                extents = extents,
                quaternions = Quaternions,
                bounds = boundsArr,
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16);

            intervalJob = new SetIntervalAndDimensionsJob
            {
                bounds = boundsArr,
                interval = intervalArr,
                dims = dimensionsArr,
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16, setBounds);

            JobHandle.ScheduleBatchedJobs();
        }
        private void LaunchJobs(Transform t, SphereDistanceInfo[] convectionPoints, SphereDistanceInfo[] sunPoints, int suggestedJobs = 1)
        {
            Profiler.BeginSample("VehicleOcclusion-LaunchJobs");

            float4x4 localToWorldMatrix = t.transform.localToWorldMatrix;
            float offset = extents.magnitude * 2;
            float4 tmp = math.mul(localToWorldMatrix, new float4(center.x, center.y, center.z, 1));
            float3 startPosition = new float3(tmp.x, tmp.y, tmp.z);
            float3 angleSelectionWeights = new float3(0.85f, 0, 0.15f);

            // Sphere Angle Properties
            // Always sort conveciton and sun spheres to get best quaternion set
            // Calculate angles between vessel orientation and target orientations (sun, convection)
            Profiler.BeginSample("VehicleOcclusion-LaunchJobs.Allocation_Angles");

            var convectionAngles = new NativeArray<float>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var convectionAngleStats = new NativeArray<float3>(1, Allocator.Temp);

            var sunAngles = new NativeArray<float>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var sunAngleStats = new NativeArray<float3>(1, Allocator.Temp);

            Vector3 localVelocity = Vessel.velocityD.magnitude < ZERO_VELOCITY_THRESHOLD ?
                                    Vector3.forward :
                                    (Vector3)(t.worldToLocalMatrix * (Vector3)Vessel.velocityD);
            Vector3 localSunVec = t.worldToLocalMatrix * (Planetarium.fetch.Sun.transform.position - t.position);

            Profiler.EndSample();
            Profiler.BeginSample("VehicleOcclusion-LaunchJobs.JobSetup_Angles");

            JobHandle convectionAngleStatsJob = LaunchAngleJobs(localVelocity.normalized, ref convectionAngles, ref convectionAngleStats);
            JobHandle sunAngleStatsJob = LaunchAngleJobs(localSunVec.normalized, ref sunAngles, ref sunAngleStats);

            Profiler.EndSample();
            Profiler.BeginSample("VehicleOcclusion-LaunchJobs.Allocation_Filters");

            // Sorting structures
            var sortedConvectionSDIList = new NativeList<SphereDistanceInfo>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var sortedSunSDIList = new NativeList<SphereDistanceInfo>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);

            var orderedConvectionQuaternions = new NativeList<quaternion>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var orderedSunQuaternions = new NativeList<quaternion>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            Profiler.EndSample();

            Profiler.BeginSample("VehicleOcclusion-LaunchJobs.FilterJob_Setup");

            var mapClearJob = new ClearNativeMultiHashMapIntInt { map = indexedPriorityMap }.Schedule();
            //indexedPriorityMap.Clear();

            // Derive cutoff distance for prioritization
            JobHandle.CompleteAll(ref sunAngleStatsJob, ref convectionAngleStatsJob);
            float sunDistanceCutoff = math.dot(angleSelectionWeights, sunAngleStats[0]);
            float convectionDistanceCutoff = math.dot(angleSelectionWeights, convectionAngleStats[0]);

            // Sorting process (fiter nearby, sort)
            JobHandle convectionPriorityMapJob = LaunchFilterAndSortJob(
                ref convectionAngles,
                convectionDistanceCutoff,
                ref sortedConvectionSDIList,
                ref orderedConvectionQuaternions);

            JobHandle sunPriorityMapJob = LaunchFilterAndSortJob(
                ref sunAngles,
                sunDistanceCutoff,
                ref sortedSunSDIList,
                ref orderedSunQuaternions);

            var bucketSortPriority = new BucketSortByPriority
            {
                quaternions = Quaternions,
                priorityList1 = orderedConvectionQuaternions.AsDeferredJobArray(),
                priorityList2 = orderedSunQuaternions.AsDeferredJobArray(),
                completed = processedQuaternionsMap,
                maxIndexForPriority0 = 2,
                map = indexedPriorityMap.AsParallelWriter(),
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16, JobHandle.CombineDependencies(convectionPriorityMapJob, sunPriorityMapJob, mapClearJob));
            JobHandle.ScheduleBatchedJobs();

            JobHandle.CompleteAll(ref convectionPriorityMapJob, ref sunPriorityMapJob);
            for (int j = 0; j < 3; j++)
            {
                convectionPoints[j] = sortedConvectionSDIList[j];
                sunPoints[j] = sortedSunSDIList[j];
            }

            quaternionWorkList.Clear();
            jobsInProgress = 0;

            Profiler.EndSample();
            // Build raycast jobs only if we are processing
            if (state == State.Running)
            {
                Profiler.BeginSample("VehicleOcclusion-LaunchJobs.Allocation_CastPlanes");
                var fullQuaternionIndexMap = new NativeHashMap<quaternion, int>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
                var boundsArr = new NativeArray<float2>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
                var intervalArr = new NativeArray<float2>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                var dimensionsArr = new NativeArray<int2>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                var forwardArr = new NativeArray<float3>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                var rightArr = new NativeArray<float3>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                var upArr = new NativeArray<float3>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                Profiler.EndSample();

                // Calculate raycast plane orientation/offset, dimensions, spacing, area/ray, element count
                Profiler.BeginSample("VehicleOcclusion-LaunchJobs.JobSetup_CastPlanes");
                LaunchRaycastJobs(
                    localToWorldMatrix,
                    ref fullQuaternionIndexMap,
                    ref boundsArr,
                    ref intervalArr,
                    ref dimensionsArr,
                    ref forwardArr,
                    ref rightArr,
                    ref upArr,
                    out var makeFullIndexMap,
                    out var setVectorsJob,
                    out var intervalJob);
                Profiler.EndSample();

                bucketSortPriority.Complete();
                SelectQuaternions(ref quaternionWorkList, ref indexedPriorityMap, MAX_JOBS, 0);  // Fill priority 0 (required)
                SelectQuaternions(ref quaternionWorkList, ref indexedPriorityMap, suggestedJobs, 1);    // Append priority 1 (desired) until size
                SelectQuaternions(ref quaternionWorkList, ref indexedPriorityMap, suggestedJobs, 2);    // Append priority 2 (anything) until size

                JobHandle.CompleteAll(ref setVectorsJob, ref intervalJob, ref makeFullIndexMap);

                int i = 0;
                foreach (quaternion q in quaternionWorkList)
                {
                    Profiler.BeginSample("VehicleOcclusion-LaunchJobs.SingleRaycastJobSetup");

                    Profiler.BeginSample("VehicleOcclusion-LaunchJobs.SingleRaycastJobSetup.Prep");
                    int index = fullQuaternionIndexMap[q];
                    int elements = dimensionsArr[index].x * dimensionsArr[index].y;
                    raycastJobTracker[i] = new RaycastJobInfo
                    {
                        q = q,
                        index = index,
                        area = intervalArr[index].x * intervalArr[index].y,
                    };
                    var clearMap1 = new ClearNativeMultiHashMapIntInt { map = allHitsMaps[i], }.Schedule();
                    var clearMap2 = new ClearNativeHashMapIntFloat { map = allHitSizeMaps[i], }.Schedule();
                    Profiler.EndSample();
                    Profiler.BeginSample("VehicleOcclusion-LaunchJobs.SingleRaycastJobSetup.BuilderJob");
                    raycastJobTracker[i].builderJob = new OcclusionRaycastBuilder
                    {
                        startPosition = startPosition,
                        offset = offset,
                        forwardDir = forwardArr[index],
                        rightDir = rightArr[index],
                        upDir = upArr[index],
                        dimensions = dimensionsArr[index],
                        interval = intervalArr[index],
                        commands = allCasts[i],
                    }.Schedule(elements, 8, JobHandle.CombineDependencies(clearMap1, clearMap2));
                    Profiler.EndSample();
                    Profiler.BeginSample("VehicleOcclusion-LaunchJobs.SingleRaycastJobSetup.RaycastJob");
                    raycastJobTracker[i].raycastJob = RaycastCommand.ScheduleBatch(allCasts[i], allHits[i], 1, raycastJobTracker[i].builderJob);
                    Profiler.EndSample();
                    Profiler.BeginSample("VehicleOcclusion-LaunchJobs.SingleRaycastJobSetup.PreparerJob");
                    raycastJobTracker[i].preparerJob = new RaycastPrepareJob
                    {
                        hitsIn = allHits[i],
                        hitsMap = allHitsMaps[i].AsParallelWriter(),
                    }.Schedule(elements, 8, raycastJobTracker[i].raycastJob);
                    Profiler.EndSample();
                    Profiler.BeginSample("VehicleOcclusion-LaunchJobs.SingleRaycastJobSetup.ProcessorJob");
                    raycastJobTracker[i].processorJob = new RaycastProcessorJob
                    {
                        hits = allHits[i],
                        hitMap = allHitsMaps[i],
                        hitsOut = allHitSizeMaps[i].AsParallelWriter(),
                        area = raycastJobTracker[i].area,
                        areaSum = allSummedHitAreas[i],
                    }.Schedule(allHitsMaps[i], 16, raycastJobTracker[i].preparerJob);
                    Profiler.EndSample();

                    JobHandle.ScheduleBatchedJobs();
                    Profiler.EndSample();
                    i++;
                }
                jobsInProgress = i;

                boundsArr.Dispose();
                intervalArr.Dispose();
                dimensionsArr.Dispose();
                forwardArr.Dispose();
                rightArr.Dispose();
                upArr.Dispose();

                fullQuaternionIndexMap.Dispose();
            }

            convectionAngles.Dispose();
            sunAngles.Dispose();
            convectionAngleStats.Dispose();
            sunAngleStats.Dispose();

            sortedConvectionSDIList.Dispose();
            sortedSunSDIList.Dispose();

            orderedConvectionQuaternions.Dispose();
            orderedSunQuaternions.Dispose();

            Profiler.EndSample();
        }

        public struct RaycastJobInfo
        {
            public quaternion q;
            public int index;
            public float area;
            public JobHandle builderJob;
            public JobHandle raycastJob;
            public JobHandle preparerJob;
            public JobHandle processorJob;
        }

        private void SelectQuaternions(ref NativeList<quaternion> q, ref NativeMultiHashMap<int, int> map, int maxSize = MAX_JOBS, int key=0)
        {
            if (q.Length < maxSize && map.TryGetFirstValue(key, out int index, out NativeMultiHashMapIterator<int> iter))
            {
                q.Add(Quaternions[index]);
                while (q.Length< maxSize && map.TryGetNextValue(out index, ref iter))
                {
                    q.Add(Quaternions[index]);
                }
            }
        }

        public void ProcessRaycastResults(ref RaycastJobInfo jobInfo, ref NativeHashMap<int, float> sizeMap, ref NativeArray<RaycastHit> hits, ref NativeArray<float> summedHitAreas)
        {
            Profiler.BeginSample("VehicleOcclusion-ProcessRaycastResults");
            quaternion rotation = jobInfo.q;
            if (sizeMap.Length > 0)
            {
                // The value in sizeMap is the size of a single element [duplicative of jobInfo.area]
                // The key is the index into summedHitAreas
                NativeArray<int> sizeIndices = sizeMap.GetKeyArray(Allocator.Temp);
                for (int key = 0; key < sizeIndices.Length; key++)
                {
                    int index = sizeIndices[key];
                    float size = summedHitAreas[index];
                    processedQuaternionsMap.TryAdd(rotation, new EMPTY_STRUCT { });
                    if (hits[index].transform is Transform t)
                    {
                        while (t.parent != null)
                            t = t.parent;
                        if (partsByTransform.TryGetValue(t, out Part p))
                        {
                            if (!partOcclusionInfo.TryGetValue(p, out DirectionalOcclusionInfo occlInfo))
                            {
                                occlInfo = new DirectionalOcclusionInfo();
                                partOcclusionInfo.Add(p, occlInfo);
                            }
                            if (!occlInfo.convectionArea.ContainsKey(rotation))
                            {
                                occlInfo.convectionArea.Add(rotation, 0);
                            }
                            occlInfo.convectionArea[rotation] += size;
                        }
                        else if (t.gameObject.GetComponent<Part>() is Part part && Vessel != part.vessel)
                        {
                            //Debug.Log($"[VehicleOcclusion.ProcessRaycastResults] Occlusion of {Vessel?.name} by {part} on {part.vessel}");
                        } else
                        {
                            Debug.LogWarning($"[VehicleOcclusion.ProcessRaycastResults] {Vessel?.name}: {hits[index].transform} ancestor {t} not associated with any part!");
                        }
                    }
                }
                sizeIndices.Dispose();
            }

            Profiler.EndSample();
        }

        public void FixedUpdateEarly()
        {
            if (!OnValidPhysicsVessel)
                return;

            // We must call this to pre-calculate the nearest quaternions for sun and convection.
            // It may return an empty work queue if we are not otherwise running, which is fine.
            if (state == State.Initialized)
                state = State.Running;
            if (state == State.Running || state == State.Completed)
                LaunchJobs(Vessel.transform, convectionPoints, sunPoints, 0);
        }

        public void FixedUpdate()
        {
            if (!OnValidPhysicsVessel)
                return;
            if (state == State.Running)
                HandleJobCompletion();
        }

        public void UpdateEarly()
        {
            if (!OnValidPhysicsVessel)
                return;
            if (state == State.Running)
            {
                int threadCount = System.Environment.ProcessorCount - 1;
                int updateJobs = math.min(MAX_JOBS, math.max(1, UPDATE_JOBS_PER_THREAD * threadCount));
                LaunchJobs(Vessel.transform, convectionPoints, sunPoints, updateJobs);
            }
        }

        public void LateUpdateComplete()
        {
            if (!OnValidPhysicsVessel)
                return;
            if (state == State.Running && jobsInProgress == 0)
                state = State.Completed;
            else if (state == State.Running)
                HandleJobCompletion();
            else if (state == State.Completed)
            {
                if (!resetCoroutineRunning)
                    StartCoroutine(resetWaitCoroutine = WaitForResetCR(RESET_INTERVAL_SEC));
            }
        }

        IEnumerator WaitForResetCR(float interval)
        {
            resetCoroutineRunning = true;
            yield return new WaitForSeconds(interval);
            resetCoroutineRunning = false;
            ResetCalculations();
        }

        public void HandleJobCompletion()
        {
            for (int i=0; i<jobsInProgress; i++)
           {
                raycastJobTracker[i].processorJob.Complete();
                ProcessRaycastResults(ref raycastJobTracker[i], ref allHitSizeMaps[i], ref allHits[i], ref allSummedHitAreas[i]);
            }
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
            if (partOcclusionInfo.TryGetValue(p, out DirectionalOcclusionInfo info))
                return Area(p, sunPoints, info.area);
            return 0;
        }

        int counter = 0;
        public float ConvectionArea(Part p, Vector3 dir)
        {
            // convectionPoints is a set of 3 quaternions that should be closest to dir.
            // (They were precalculated, if dir is NOT what we expect for occlusion... that's bad.)
            Vector3 localVelocity = dir.magnitude < ZERO_VELOCITY_THRESHOLD ?
                                    Vector3.forward :
                                    (Vector3) (Vessel.transform.worldToLocalMatrix * dir);
            Quaternion q = Quaternion.FromToRotation(Vector3.forward, localVelocity);
            float a = 0.25f;

            if (partOcclusionInfo.TryGetValue(p, out DirectionalOcclusionInfo info))
            {
                float angle = Quaternion.Angle(q, convectionPoints[0].q);
                averageMissOnBestAngle = (a * angle) + ((1f - a) * averageMissOnBestAngle);
                if (counter++ % 500 == 0)
                    Debug.Log($"[VehicleOcclusion] {localVelocity} missed {angle} [average {averageMissOnBestAngle}]");
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
                if (areaInfos.TryGetValue(sdi.q, out float newArea))
                {
                    if (sdi.distance < float.Epsilon)
                        return newArea;
                    area += newArea / sdi.distance;
                    distanceSum += (1 / sdi.distance);
                }
                else if (processedQuaternionsMap.ContainsKey(sdi.q))
                {
                    // Processed quaternion but no per-part data means 0 area [no raycasters hit]
                    distanceSum += (1 / sdi.distance);
                }
                else
                {
                    Debug.LogWarning($"[VehicleOcclusion.Area] {Vessel?.name} Quaternion {sdi.q} was in top-3 but had not been processed!");
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
