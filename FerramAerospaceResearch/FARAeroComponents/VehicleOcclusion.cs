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

        // Closest 3 convection and sun quaternions for each FixedUpdate
        private readonly SphereDistanceInfo[] convectionPoints = new SphereDistanceInfo[3];
        private readonly SphereDistanceInfo[] sunPoints = new SphereDistanceInfo[3];
        private readonly SphereDistanceInfo[] bodyPoints = new SphereDistanceInfo[3];
        
        private readonly DirectionPreprocessInfo[] directionPreprocessInfos = new DirectionPreprocessInfo[3];
        private readonly List<Vector3> directionList = new List<Vector3>(MAX_JOBS);
        private readonly List<SphereDistanceInfo[]> occlusionPointsList = new List<SphereDistanceInfo[]>(3);
        public float3 DefaultAngleWeights => new float3(0.85f, 0, 0.15f);

        // VehicleVoxel center and extents
        private Vector3 center = Vector3.zero;
        private Vector3 extents = new Vector3(10, 10, 10);

        private int jobsInProgress;
        private float averageMissOnBestAngle = 0;

        // Large long-term allocations, memory re-used in each
        // Cleaned and Allocated in ResetCalculations().  Disposed in OnDestroy()
        private NativeMultiHashMap<int, int> indexedPriorityMap;

        private NativeArray<RaycastCommand>[] allCasts = new NativeArray<RaycastCommand>[MAX_JOBS];
        private NativeArray<RaycastHit>[] allHits = new NativeArray<RaycastHit>[MAX_JOBS];
        private NativeMultiHashMap<int, int>[] allHitsMaps = new NativeMultiHashMap<int, int>[MAX_JOBS];
        private NativeHashMap<int, float>[] allHitSizeMaps = new NativeHashMap<int, float>[MAX_JOBS];
        private NativeArray<float>[] allSummedHitAreas = new NativeArray<float>[MAX_JOBS];

        // allCasts => allHits
        // allHits => allHitMaps
        // allHitMaps => allHitSizeMaps,
        //               allSummedHitAreas,

        private RaycastJobInfo[] raycastJobTracker = new RaycastJobInfo[MAX_JOBS];
        public bool RequestReset = false;

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

        private void ResetCalculations()
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
            RequestReset = false;
            state = State.Initialized;
        }

        public void Start()
        {
            FARLogger.Info($"VehicleOcclusion on {Vessel?.name} reporting startup");
            state = State.Invalid;
            occlusionPointsList.Clear();
            occlusionPointsList.Add(convectionPoints);
            occlusionPointsList.Add(sunPoints);
            occlusionPointsList.Add(bodyPoints);
            TimingManager.FixedUpdateAdd(TimingManager.TimingStage.ObscenelyEarly, FixedUpdateEarly);
            TimingManager.UpdateAdd(TimingManager.TimingStage.ObscenelyEarly, UpdateEarly);
            TimingManager.LateUpdateAdd(TimingManager.TimingStage.Late, LateUpdateComplete);
        }

        public void OnDestroy()
        {
            DisposeLongTermAllocations();

            TimingManager.FixedUpdateRemove(TimingManager.TimingStage.ObscenelyEarly, FixedUpdateEarly);
            TimingManager.UpdateRemove(TimingManager.TimingStage.ObscenelyEarly, UpdateEarly);
            TimingManager.LateUpdateRemove(TimingManager.TimingStage.Late, LateUpdateComplete);
        }

        private void LaunchAngleJobs(ref DirectionPreprocessInfo info, ref NativeArray<quaternion> quaternions)
        {
            var angleJob = new AngleBetween
            {
                dir = info.dir,
                rotations = quaternions,
                angles = info.angles,
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16);

            var statsJob = new GeneralDistanceInfo
            {
                distances = info.angles,
                output = info.angleStats,
            }.Schedule(angleJob);

            var statsCombineJob = new DotProductJob
            {
                A = info.angleStats,
                B = info.weights,
                result = info.distanceCutoff,
            }.Schedule(info.angleStats.Length, 16, statsJob);

            info.sortJob = new SortedFilterAndMapByDistanceJob
            {
                quaternions = quaternions,
                distances = info.angles,
                cutoffDistance = info.distanceCutoff,
                sdiList = info.sortedSDIList,
                orderedQuaternions = info.orderedQuaternions,
            }.Schedule(statsCombineJob);
            JobHandle.ScheduleBatchedJobs();
        }

        private void LaunchRaycastJobs(
            float4x4 localToWorldMatrix,
            ref NativeArray<quaternion> quaternions,
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
                arr = quaternions,
                map = fullQuaternionIndexMap.AsParallelWriter(),
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16);

            vectorJob = new SetVectorsJob
            {
                quaternions = quaternions,
                localToWorldMatrix = localToWorldMatrix,
                forward = forwardArr,
                up = upArr,
                right = rightArr,
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16);

            var setBounds = new SetBoundsJob
            {
                boundsCenter = center,
                extents = extents,
                quaternions = quaternions,
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

        private void BuildPreprocessInfo(DirectionPreprocessInfo[] infos, List<Vector3> directions)
        {
            for (int i=0; i<directions.Count && i < MAX_JOBS; i++)
            {
                Vector3 v = directions[i];
                infos[i] = new DirectionPreprocessInfo
                {
                    angles = new NativeArray<float>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob),
                    angleStats = new NativeArray<float3>(1, Allocator.TempJob),
                    sortedSDIList = new NativeList<SphereDistanceInfo>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob),
                    orderedQuaternions = new NativeList<quaternion>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob),
                    distanceCutoff = new NativeArray<float>(1, Allocator.TempJob),
                    dir = v.normalized,
                    weights = DefaultAngleWeights,
                };
            }
        }

        private int BuildAndLaunchRaycastJobs(
            Vessel v,
            DirectionPreprocessInfo[] dirInfos,
            RaycastJobInfo[] tracker,
            ref NativeArray<quaternion> quaternions,
            ref NativeHashMap<quaternion, EMPTY_STRUCT> processedMap,
            ref NativeMultiHashMap<int, int> priorityMap,
            int suggestedJobs,
            float3 startPosition,
            float offset,
            NativeArray<RaycastCommand>[] casts,
            NativeArray<RaycastHit>[] hits,
            NativeMultiHashMap<int, int>[] hitsMaps,
            NativeHashMap<int, float>[] hitSizeMaps,
            NativeArray<float>[] summedHitAreas)
        {
            Profiler.BeginSample("VehicleOcclusion-LaunchJobs.Allocation_CastPlanes");
            var mapClearJob = new ClearNativeMultiHashMapIntInt { map = priorityMap }.Schedule();
            var workList = new NativeList<quaternion>(MAX_JOBS, Allocator.TempJob);
            var indexMap = new NativeHashMap<quaternion, int>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            JobHandle lastJob = default;
            for (int j = 0; j < dirInfos.Length; j++)
            {
                var x = new ListToIndexedMap
                {
                    sorted = dirInfos[j].orderedQuaternions.AsDeferredJobArray(),
                    map = indexMap,
                }.Schedule(lastJob);
                lastJob = x;
            }

            var bucketSortPriority = new BucketSortByPriority
            {
                quaternions = quaternions,
                priorityMap = indexMap,
                completed = processedMap,
                maxIndexForPriority0 = 2,
                map = priorityMap.AsParallelWriter(),
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16, JobHandle.CombineDependencies(lastJob, mapClearJob));
            JobHandle.ScheduleBatchedJobs();

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
                v.transform.localToWorldMatrix,
                ref quaternions,
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
            SelectQuaternions(ref workList, ref priorityMap, MAX_JOBS, 0);  // Fill priority 0 (required)
            SelectQuaternions(ref workList, ref priorityMap, suggestedJobs, 1);    // Append priority 1 (desired) until size
            SelectQuaternions(ref workList, ref priorityMap, suggestedJobs, 2);    // Append priority 2 (anything) until size

            JobHandle.CompleteAll(ref setVectorsJob, ref intervalJob, ref makeFullIndexMap);

            int i = 0;
            foreach (quaternion q in workList)
            {
                Profiler.BeginSample("VehicleOcclusion-LaunchJobs.SingleRaycastJobSetup");
                var clearMap1 = new ClearNativeMultiHashMapIntInt { map = hitsMaps[i], }.Schedule();
                var clearMap2 = new ClearNativeHashMapIntFloat { map = hitSizeMaps[i], }.Schedule();
                JobHandle.ScheduleBatchedJobs();

                Profiler.BeginSample("VehicleOcclusion-LaunchJobs.SingleRaycastJobSetup.Prep");
                int index = fullQuaternionIndexMap[q];
                int elements = dimensionsArr[index].x * dimensionsArr[index].y;
                tracker[i] = new RaycastJobInfo
                {
                    q = q,
                    index = index,
                    area = intervalArr[index].x * intervalArr[index].y,
                };
                Profiler.EndSample();
                
                Profiler.BeginSample("VehicleOcclusion-LaunchJobs.SingleRaycastJobSetup.BuilderJob");
                tracker[i].builderJob = new OcclusionRaycastBuilder
                {
                    startPosition = startPosition,
                    offset = offset,
                    forwardDir = forwardArr[index],
                    rightDir = rightArr[index],
                    upDir = upArr[index],
                    dimensions = dimensionsArr[index],
                    interval = intervalArr[index],
                    commands = casts[i],
                }.Schedule(elements, 8);
                Profiler.EndSample();
                
                Profiler.BeginSample("VehicleOcclusion-LaunchJobs.SingleRaycastJobSetup.RaycastJob");
                tracker[i].raycastJob = RaycastCommand.ScheduleBatch(casts[i], hits[i], 1, tracker[i].builderJob);
                Profiler.EndSample();

                Profiler.BeginSample("VehicleOcclusion-LaunchJobs.SingleRaycastJobSetup.PreparerJob");
                tracker[i].preparerJob = new RaycastPrepareJob
                {
                    hitsIn = hits[i],
                    hitsMap = hitsMaps[i].AsParallelWriter(),
                }.Schedule(elements, 8, JobHandle.CombineDependencies(clearMap1, clearMap2, tracker[i].raycastJob));
                Profiler.EndSample();

                Profiler.BeginSample("VehicleOcclusion-LaunchJobs.SingleRaycastJobSetup.ProcessorJob");
                tracker[i].processorJob = new RaycastProcessorJob
                {
                    hits = hits[i],
                    hitMap = hitsMaps[i],
                    hitsOut = hitSizeMaps[i].AsParallelWriter(),
                    area = tracker[i].area,
                    areaSum = summedHitAreas[i],
                }.Schedule(hitsMaps[i], 16, tracker[i].preparerJob);
                Profiler.EndSample();

                JobHandle.ScheduleBatchedJobs();
                Profiler.EndSample();
                i++;
                
            }

            //jobsInProgress = i;

            indexMap.Dispose();
            boundsArr.Dispose();
            intervalArr.Dispose();
            dimensionsArr.Dispose();
            forwardArr.Dispose();
            rightArr.Dispose();
            upArr.Dispose();
            workList.Dispose();
            fullQuaternionIndexMap.Dispose();
            return i;
        }


        private void LaunchJobs(Vessel v, DirectionPreprocessInfo[] dirInfos, List<Vector3> dirs, List<SphereDistanceInfo[]> sdiArrays, int suggestedJobs = 1)
        {
            Profiler.BeginSample("VehicleOcclusion-LaunchJobs");

            float offset = extents.magnitude * 2;
            float4 tmp = math.mul(v.transform.localToWorldMatrix, new float4(center.x, center.y, center.z, 1));
            float3 startPosition = new float3(tmp.x, tmp.y, tmp.z);

            Profiler.BeginSample("VehicleOcclusion-LaunchJobs.FilterJob_Setup");

            BuildPreprocessInfo(dirInfos, dirs);

            for (int i=0; i< dirs.Count; i++)
                LaunchAngleJobs(ref dirInfos[i], ref Quaternions);
            for (int i = 0; i < dirs.Count; i++)
                dirInfos[i].sortJob.Complete();

            NativeList<quaternion> requiredQuaternions = new NativeList<quaternion>(sdiArrays.Count * dirs.Count, Allocator.Temp);

            for (int i = 0; i < sdiArrays.Count; i++)
            {
                for (int j=0; j < dirs.Count; j++)
                {
                    sdiArrays[j][i] = dirInfos[j].sortedSDIList[i];
                    if (!processedQuaternionsMap.ContainsKey(dirInfos[j].sortedSDIList[i].q))
                        requiredQuaternions.Add(dirInfos[j].sortedSDIList[i].q);

                }
            }

            jobsInProgress = 0;

            Profiler.EndSample();
            // Build raycast jobs only if we are processing and require or will accept additional work
            if (state == State.Running && (requiredQuaternions.Length > 0 || suggestedJobs > 0))
            {
                jobsInProgress = BuildAndLaunchRaycastJobs(
                                    Vessel,
                                    dirInfos,
                                    raycastJobTracker,
                                    ref Quaternions,
                                    ref processedQuaternionsMap,
                                    ref indexedPriorityMap,
                                    suggestedJobs,
                                    startPosition,
                                    offset,
                                    allCasts,
                                    allHits,
                                    allHitsMaps,
                                    allHitSizeMaps,
                                    allSummedHitAreas);
            }

            for (int i=0; i< dirs.Count; i++)
            {
                dirInfos[i].DisposeAll();
            }
            requiredQuaternions.Dispose();
            Profiler.EndSample();
        }

        // On demand, return the unoccluded area of Part p in <dir>
        private float AreaOnDemand(Part p, Vector3 dir, bool worldSpace = false)
        {
            float area = 0;
            if (p && p.vessel == Vessel)
            {
                if (worldSpace)
                    dir = Vessel.transform.worldToLocalMatrix * dir;

                float offset = extents.magnitude * 2;
                float4 tmp = math.mul(p.vessel.transform.localToWorldMatrix, new float4(center.x, center.y, center.z, 1));
                float3 startPosition = new float3(tmp.x, tmp.y, tmp.z);

                var infos = new DirectionPreprocessInfo[1];
                BuildPreprocessInfo(infos, new List<Vector3>(1) { dir.normalized });
                LaunchAngleJobs(ref infos[0], ref Quaternions);
                infos[0].sortJob.Complete();

                NativeList<quaternion> requiredQuaternions = new NativeList<quaternion>(3, Allocator.Temp);

                for (int i=0; i<3; i++)
                {
                    if (!processedQuaternionsMap.ContainsKey(infos[0].sortedSDIList[i].q))
                        requiredQuaternions.Add(infos[0].sortedSDIList[i].q);
                }
                // Raycast if necessary
                if (requiredQuaternions.Length > 0)
                {
                    var requiredArr = new NativeArray<quaternion>(requiredQuaternions, Allocator.Temp);
                    var tracker = new RaycastJobInfo[3];

                    var casts = new NativeArray<RaycastCommand>[3];
                    var hits = new NativeArray<RaycastHit>[3];
                    var hitsMaps = new NativeMultiHashMap<int, int>[3];
                    var summedHitAreas = new NativeArray<float>[3];
                    var hitSizeMaps = new NativeHashMap<int, float>[3];
                    var emptyPriMap = new NativeMultiHashMap<int, int>(1, Allocator.TempJob);

                    jobsInProgress = BuildAndLaunchRaycastJobs(
                                        p.vessel,
                                        infos,
                                        tracker,
                                        ref requiredArr,
                                        ref processedQuaternionsMap,
                                        ref emptyPriMap,
                                        MAX_JOBS,
                                        startPosition,
                                        offset,
                                        casts,
                                        hits,
                                        hitsMaps,
                                        hitSizeMaps,
                                        summedHitAreas);

                    int i = 0;
                    foreach (quaternion q in requiredQuaternions)
                    {
                        casts[i].Dispose();
                        hits[i].Dispose();
                        hitsMaps[i].Dispose();
                        summedHitAreas[i].Dispose();
                        hitSizeMaps[i].Dispose();
                        i++;
                    }

                    requiredArr.Dispose();
                }

                // Raycasts are complete, we have saved the angle data.
                if (partOcclusionInfo.TryGetValue(p, out DirectionalOcclusionInfo info))
                    area = Area(p, convectionPoints, info.convectionArea);

                infos[0].DisposeAll();
                requiredQuaternions.Dispose();
            }
            return area;
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

        public struct DirectionPreprocessInfo
        {
            public float3 dir;
            public NativeArray<float> angles;
            public NativeArray<float3> angleStats;
            public NativeList<SphereDistanceInfo> sortedSDIList;
            public NativeList<quaternion> orderedQuaternions;
            public NativeArray<float> distanceCutoff;
            public float3 weights;
            public JobHandle sortJob;

            public void DisposeAll()
            {
                angles.Dispose();
                angleStats.Dispose();
                sortedSDIList.Dispose();
                orderedQuaternions.Dispose();
                distanceCutoff.Dispose();
            }
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
            processedQuaternionsMap.TryAdd(rotation, new EMPTY_STRUCT { });
            if (sizeMap.Length > 0)
            {
                // The value in sizeMap is the size of a single element [duplicative of jobInfo.area]
                // The key is the index into summedHitAreas
                NativeArray<int> sizeIndices = sizeMap.GetKeyArray(Allocator.Temp);
                for (int key = 0; key < sizeIndices.Length; key++)
                {
                    int index = sizeIndices[key];
                    float size = summedHitAreas[index];
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

        private void SetupDefaultDirs(List<Vector3> dirs, Vessel v)
        {
            dirs.Clear();
            Vector3 localVelocity = v.velocityD.magnitude < ZERO_VELOCITY_THRESHOLD ?
                                    Vector3.forward :
                                    (Vector3)(v.transform.worldToLocalMatrix * (Vector3)v.velocityD);
            Vector3 localSunVec = v.transform.worldToLocalMatrix * (Planetarium.fetch.Sun.transform.position - v.transform.position);
            Vector3 bodyVec = v.transform.worldToLocalMatrix * (Vector3)(Vessel.mainBody.position - v.transform.position);
            dirs.Add(localVelocity.normalized);
            dirs.Add(localSunVec.normalized);
            dirs.Add(bodyVec.normalized);
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
            {
                SetupDefaultDirs(directionList, Vessel);
                LaunchJobs(Vessel, directionPreprocessInfos, directionList, occlusionPointsList, 0);
            }
        }

        public void FixedUpdate()
        {
            if (!OnValidPhysicsVessel)
                return;
            if (state == State.Running)
                HandleJobCompletion(raycastJobTracker);
        }

        public void UpdateEarly()
        {
            if (!OnValidPhysicsVessel)
                return;
            if (state == State.Running)
            {
                int threadCount = System.Environment.ProcessorCount - 1;
                int updateJobs = math.min(MAX_JOBS, math.max(1, UPDATE_JOBS_PER_THREAD * threadCount));
                SetupDefaultDirs(directionList, Vessel);
                LaunchJobs(Vessel, directionPreprocessInfos, directionList, occlusionPointsList, updateJobs);
            }
        }

        public void LateUpdateComplete()
        {
            if (!OnValidPhysicsVessel)
                return;
            if (state == State.Running && jobsInProgress == 0 && processedQuaternionsMap.Length == Quaternions.Length)
                state = State.Completed;
            else if (state == State.Running)
                HandleJobCompletion(raycastJobTracker);
            else if (state == State.Completed)
            {
                if (!resetCoroutineRunning)
                    StartCoroutine(resetWaitCoroutine = WaitForResetCR(RESET_INTERVAL_SEC));
            }
            if (RequestReset)
                ResetCalculations();
        }

        IEnumerator WaitForResetCR(float interval)
        {
            resetCoroutineRunning = true;
            yield return new WaitForSeconds(interval);
            resetCoroutineRunning = false;
            ResetCalculations();
        }

        public void HandleJobCompletion(RaycastJobInfo[] tracker)
        {
            for (int i=0; i<jobsInProgress; i++)
           {
                tracker[i].processorJob.Complete();
                ProcessRaycastResults(ref tracker[i], ref allHitSizeMaps[i], ref allHits[i], ref allSummedHitAreas[i]);
            }
        }

        // http://extremelearning.com.au/how-to-evenly-distribute-points-on-a-sphere-more-effectively-than-the-canonical-fibonacci-lattice/
        private float Lattice_epsilon(int n) => 0.36f;

        public float SunArea(Part p, Vector3 dir)
        {
            if (partOcclusionInfo.TryGetValue(p, out DirectionalOcclusionInfo info))
                return Area(p, sunPoints, info.convectionArea);
            return 0;
        }
        public float BodyArea(Part p, Vector3 dir)
        {
            if (partOcclusionInfo.TryGetValue(p, out DirectionalOcclusionInfo info))
                return Area(p, bodyPoints, info.convectionArea);
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
