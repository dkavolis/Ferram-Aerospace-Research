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
        const double ZERO_VELOCITY_THRESHOLD = 0.01;    // Velocity < threshold will be treated as 0 for vessel travel direction
        // 1000 points produces typical misses 2-3 degrees when optimized for average miss distance.

        public enum State { Invalid, Initialized, Running, Completed }
        public State state = State.Invalid;

        private FARVesselAero farVesselAero;
        Vessel Vessel => farVesselAero?.Vessel;

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
        }

        //Extents = axis-aligned bounding box dimensions of combined vehicle meshes
        public void SetVehicleBounds(Vector3d center, Vector3d extents)
        {
            this.center = center;
            this.extents = extents;
            FARLogger.Info($"[VehicleOcclusion] Learned Center {center} and Extents {extents}");
        }

        private void DisposeLongTermAllocations()
        {
            if (state != State.Invalid)
            {
                Quaternions.Dispose();
                processedQuaternionsMap.Dispose();
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
            FARLogger.Info($"[VehicleOcclusion] Resetting occlusion calculation data.");
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
            if (Vessel.isActiveVessel)
            {
                TimingManager.FixedUpdateAdd(TimingManager.TimingStage.ObscenelyEarly, FixedUpdateEarly);
                TimingManager.UpdateAdd(TimingManager.TimingStage.ObscenelyEarly, UpdateEarly);
                TimingManager.LateUpdateAdd(TimingManager.TimingStage.Late, LateUpdateComplete);
            }
        }

        public void OnDestroy()
        {
            quaternionWorkList.Dispose();
            DisposeLongTermAllocations();

            TimingManager.FixedUpdateRemove(TimingManager.TimingStage.ObscenelyEarly, FixedUpdateEarly);
            TimingManager.UpdateRemove(TimingManager.TimingStage.ObscenelyEarly, UpdateEarly);
            TimingManager.LateUpdateRemove(TimingManager.TimingStage.Late, LateUpdateComplete);
        }

        private void LaunchJobs(Transform t, SphereDistanceInfo[] convectionPoints, SphereDistanceInfo[] sunPoints, int suggestedJobs = 1)
        {
            Profiler.BeginSample("VehicleOcclusion-LaunchJobs");

            float4x4 localToWorldMatrix = t.transform.localToWorldMatrix;
            float offset = extents.magnitude * 2;
            float4 tmp = math.mul(localToWorldMatrix, new float4(center.x, center.y, center.z, 1));
            float3 startPosition = new float3(tmp.x, tmp.y, tmp.z);

            Vector3 localVelocity = Vessel.velocityD.magnitude < ZERO_VELOCITY_THRESHOLD ?
                                    Vector3.forward :
                                    (Vector3) (t.worldToLocalMatrix * (Vector3)Vessel.velocityD);

            float3 angleSelectionWeights = new float3(0.85f, 0, 0.15f);

            Profiler.BeginSample("VehicleOcclusion-LaunchJobs.Allocation_Angles");

            var fullQuaternionIndexMap = new NativeHashMap<quaternion, int>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            // Sphere Angle Properties
            var convectionAngles = new NativeArray<float>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var sunAngles = new NativeArray<float>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var convectionAngleStats = new NativeArray<float3>(1, Allocator.Temp);
            var sunAngleStats = new NativeArray<float3>(1, Allocator.Temp);

            Profiler.EndSample();
            Profiler.BeginSample("VehicleOcclusion-LaunchJobs.JobSetup_Angles");

            var makeFullIndexMap = new MakeQuaternionIndexMapJob
            {
                arr = Quaternions,
                map = fullQuaternionIndexMap.AsParallelWriter(),
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16);
            JobHandle.ScheduleBatchedJobs();

            // Calculate angles between vessel orientation and target orientations (sun, convection)

            var convectionAngleJob = new AngleBetween
            {
                dir = localVelocity.normalized,
                rotations = Quaternions,
                angles = convectionAngles,
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16);

            Vector3 localSunVec = t.worldToLocalMatrix * (Planetarium.fetch.Sun.transform.position - t.position);
            var sunAngleJob = new AngleBetween
            {
                dir = localSunVec.normalized,
                rotations = Quaternions,
                angles = sunAngles,
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16);

            // Get properties of sphere coordinates (min/max/mean angular miss)

            var convectionAngleStatsJob = new GeneralDistanceInfo
            {
                distances = convectionAngles,
                output = convectionAngleStats,
            }.Schedule(convectionAngleJob);

            var sunAngleStatsJob = new GeneralDistanceInfo
            {
                distances = sunAngles,
                output = sunAngleStats,
            }.Schedule(sunAngleJob);

            JobHandle.ScheduleBatchedJobs();
            Profiler.EndSample();
            Profiler.BeginSample("VehicleOcclusion-LaunchJobs.Allocation_CastPlanes");

            // Calculate raycast plane orientation/offset, dimensions, spacing, area/ray, element count
            var boundsArr = new NativeArray<float2>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var intervalArr = new NativeArray<float2>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var dimensionsArr = new NativeArray<int2>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var forwardArr = new NativeArray<float3>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var rightArr = new NativeArray<float3>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var upArr = new NativeArray<float3>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);

            Profiler.EndSample();
            Profiler.BeginSample("VehicleOcclusion-LaunchJobs.JobSetup_CastPlanes");

            var setVectors = new SetVectorsJob
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

            _ = new SetIntervalAndDimensionsJob
            {
                bounds = boundsArr,
                interval = intervalArr,
                dims = dimensionsArr,
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16, setBounds);

            JobHandle.ScheduleBatchedJobs();

            Profiler.EndSample();
            Profiler.BeginSample("VehicleOcclusion-LaunchJobs.Allocation_Filters");

            var convectionQuaternionsPriorityMap = new NativeHashMap<quaternion, int>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var sunQuaternionsPriorityMap = new NativeHashMap<quaternion, int>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var indexedPriorityMap = new NativeMultiHashMap<int, int>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);

            // Sorting structures
            var sortableConvectionQuaternionsMap = new NativeMultiHashMap<int, SphereDistanceInfo>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var sortableSunQuaternionsMap = new NativeMultiHashMap<int, SphereDistanceInfo>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var sortableConvectionQuaternions = new NativeList<SphereDistanceInfo>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);
            var sortableSunQuaternions = new NativeList<SphereDistanceInfo>(FIBONACCI_LATTICE_SIZE, Allocator.TempJob);

            Profiler.EndSample();
            Profiler.BeginSample("VehicleOcclusion-LaunchJobs.FilterJob_Setup");

            // Derive cutoff distance for prioritization
            JobHandle.CompleteAll(ref sunAngleStatsJob, ref convectionAngleStatsJob);
            float sunDistanceCutoff = math.dot(angleSelectionWeights, sunAngleStats[0]);
            float convectionDistanceCutoff = math.dot(angleSelectionWeights, convectionAngleStats[0]);

            // Sorting process (fiter nearby, sort)

            var filterConvectionQuaternions = new FilterByDistanceJob
            {
                quaternions = Quaternions,
                distances = convectionAngles,
                cutoffDistance = convectionDistanceCutoff,
                map = sortableConvectionQuaternionsMap.AsParallelWriter(),
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16);

            var filterSunQuaternions = new FilterByDistanceJob
            {
                quaternions = Quaternions,
                distances = sunAngles,
                cutoffDistance = sunDistanceCutoff,
                map = sortableSunQuaternionsMap.AsParallelWriter(),
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16);

            var sortConvectionQuaternions = new GetSortedListFromMultiHashmap
            {
                map = sortableConvectionQuaternionsMap,
                list = sortableConvectionQuaternions,
            }.Schedule(filterConvectionQuaternions);

            var sortSunQuaternions = new GetSortedListFromMultiHashmap
            {
                map = sortableSunQuaternionsMap,
                list = sortableSunQuaternions,
            }.Schedule(filterSunQuaternions);

            JobHandle.ScheduleBatchedJobs();

            var makeConvectionPriorityMap = new MakeIndexMapJob
            {
                quaternions = sortableConvectionQuaternions.AsDeferredJobArray(),
                map = convectionQuaternionsPriorityMap.AsParallelWriter(),
            }.Schedule(sortConvectionQuaternions);

            var makeSunPriorityMap = new MakeIndexMapJob
            {
                quaternions = sortableSunQuaternions.AsDeferredJobArray(),
                map = sunQuaternionsPriorityMap.AsParallelWriter(),
            }.Schedule(sortSunQuaternions);

            var bucketSortPriority = new BucketSortByPriority
            {
                quaternions = Quaternions,
                priorityMap1 = convectionQuaternionsPriorityMap,
                priorityMap2 = sunQuaternionsPriorityMap,
                completed = processedQuaternionsMap,
                maxIndexForPriority0 = 2,
                map = indexedPriorityMap.AsParallelWriter(),
            }.Schedule(FIBONACCI_LATTICE_SIZE, 16, JobHandle.CombineDependencies(makeConvectionPriorityMap, makeSunPriorityMap));

            JobHandle.ScheduleBatchedJobs();
            Profiler.EndSample();

            bucketSortPriority.Complete();

            convectionPoints[0] = sortableConvectionQuaternions[0];
            convectionPoints[1] = sortableConvectionQuaternions[1];
            convectionPoints[2] = sortableConvectionQuaternions[2];
            sunPoints[0] = sortableSunQuaternions[0];
            sunPoints[1] = sortableSunQuaternions[1];
            sunPoints[2] = sortableSunQuaternions[2];

            quaternionWorkList.Clear();
            SelectQuaternions(ref quaternionWorkList, ref indexedPriorityMap, MAX_JOBS, 0);  // Fill priority 0 (required)
            SelectQuaternions(ref quaternionWorkList, ref indexedPriorityMap, suggestedJobs, 1);    // Append priority 1 (desired) until size
            SelectQuaternions(ref quaternionWorkList, ref indexedPriorityMap, suggestedJobs, 2);    // Append priority 2 (anything) until size

            JobHandle.CompleteAll(ref setVectors, ref makeFullIndexMap);

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

            Profiler.BeginSample("VehicleOcclusion-LaunchJobs.Disposal");

            // Destroy everything
            convectionAngles.Dispose();
            sunAngles.Dispose();
            convectionAngleStats.Dispose();
            sunAngleStats.Dispose();

            boundsArr.Dispose();
            intervalArr.Dispose();
            dimensionsArr.Dispose();
            forwardArr.Dispose();
            rightArr.Dispose();
            upArr.Dispose();

            fullQuaternionIndexMap.Dispose();
            convectionQuaternionsPriorityMap.Dispose();
            sunQuaternionsPriorityMap.Dispose();
            indexedPriorityMap.Dispose();

            sortableConvectionQuaternionsMap.Dispose();
            sortableSunQuaternionsMap.Dispose();
            sortableConvectionQuaternions.Dispose();
            sortableSunQuaternions.Dispose();
            Profiler.EndSample();

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
                        else
                        {
                            UnityEngine.Debug.LogWarning($"[VehicleOcclusion.ProcessRaycastResults] {hits[index].transform} ancestor {t} not associated with any part!");
                        }
                    }
                }
                sizeIndices.Dispose();
            }

            Profiler.EndSample();
        }

        public void FixedUpdateEarly()
        {
            if (!farVesselAero.isValid || FlightGlobals.ActiveVessel != Vessel)
                return;

            // We must call this to pre-calculate the nearest quaternions for sun and convection.
            // It may return an empty work queue if we are not otherwise running, which is fine.
            if (state == State.Initialized)
                state = State.Running;
            if (state == State.Running)
                LaunchJobs(Vessel.transform, convectionPoints, sunPoints, 0);
        }

        public void FixedUpdate()
        {
            if (!farVesselAero.isValid || FlightGlobals.ActiveVessel != Vessel)
                return;
            if (state == State.Running)
                HandleJobCompletion();
        }

        public void UpdateEarly()
        {
            if (!farVesselAero.isValid || FlightGlobals.ActiveVessel != Vessel)
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
            if (!farVesselAero.isValid || FlightGlobals.ActiveVessel != Vessel)
                return;
            if (state == State.Running && jobsInProgress == 0)
                state = State.Completed;
            else if (state == State.Running)
                HandleJobCompletion();
            else if (state == State.Completed)
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
//                if (counter++ % 50 == 0)
//                    Debug.Log($"[VehicleOcclusion] {localVelocity} missed {angle} [average {averageMissOnBestAngle}]");
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
