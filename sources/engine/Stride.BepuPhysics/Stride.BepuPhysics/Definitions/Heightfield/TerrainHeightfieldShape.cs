// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Memory;
using NRigidPose = BepuPhysics.RigidPose;

namespace Stride.BepuPhysics.Definitions.Heightfield;

/// <summary>
/// A whole-map terrain heightfield: an <see cref="IHomogeneousCompoundShape{TChildShape, TChildShapeWide}"/> of
/// <see cref="Triangle"/> whose vertices are sampled on demand from a height source. No triangle data is stored; cells
/// are mapped O(1) from a broadphase AABB and emit two triangles per cell at query time.
/// </summary>
/// <remarks>
/// <para>
/// Bepu shapes must be <c>unmanaged</c> (<see cref="Shapes.Add{TShape}"/> constrains <c>TShape : unmanaged</c>), so the
/// shape cannot hold a managed <see cref="IHeightfieldSource"/> directly. Instead it holds a type-erased callback
/// (<see cref="SampleHeightCallback"/> + <see cref="State"/>) that the collider binds to a managed source. The collider
/// (<see cref="Colliders.HeightfieldCollider"/>) bridges any <see cref="IHeightfieldSource"/> into this callback via a
/// <see cref="System.Runtime.InteropServices.GCHandle"/>, so the real Phase 1 layer-stack sampler drops in without
/// shape changes. Heights are sampled <b>live</b> (no baked copy), so physics and data cannot diverge and the design
/// scales to the 4&#xD7;4 km map.
/// </para>
/// <para>
/// Modeled on Bepu's built-in <see cref="Mesh"/> but with implicit triangle storage: the acceleration structure is a
/// direct grid-cell-range lookup rather than a <see cref="Tree"/>. Internal-edge welding is inherited for free from
/// <see cref="MeshReduction"/> via the generic <c>ConvexMeshContinuations&lt;TerrainHeightfieldShape&gt;</c> task
/// registration (see <see cref="HeightfieldCollisionTasks"/>).
/// </para>
/// <para>
/// Triangle winding is clockwise (right-handed), producing upward-facing normals for ground. Each cell is split along
/// the (min,min)-(max,max) diagonal; the two triangles share that edge with opposite winding so MeshReduction sees a
/// continuous manifold across cell boundaries.
/// </para>
/// </remarks>
public unsafe struct TerrainHeightfieldShape : IHomogeneousCompoundShape<Triangle, TriangleWide>
{
    /// <summary>
    /// Fork-local shape type id. Reserved high range (32+) to avoid collision with upstream Bepu built-ins (Sphere=0 .. Mesh=8).
    /// </summary>
    public const int Id = 32;
    public static int TypeId => Id;

    /// <summary>Opaque state token passed to <see cref="SampleHeightCallback"/> (a <see cref="System.Runtime.InteropServices.GCHandle"/> token bound by the collider).</summary>
    public void* State;
    /// <summary>Function pointer that samples the terrain height at a local (x, z). Bound by the collider to the active <see cref="IHeightfieldSource"/>.</summary>
    public delegate*<void*, float, float, float> SampleHeightCallback;
    /// <summary>Local-space origin corner of the field (the static's pose maps local to world).</summary>
    public Vector3 Origin;
    /// <summary>Edge length of one collision cell. Independently tunable from storage resolution.</summary>
    public float CellSize;
    /// <summary>Number of cells along local X.</summary>
    public int CellsX;
    /// <summary>Number of cells along local Z.</summary>
    public int CellsZ;
    /// <summary>Conservative global minimum height over the field (baked at attach from the source).</summary>
    public float MinHeight;
    /// <summary>Conservative global maximum height over the field (baked at attach from the source).</summary>
    public float MaxHeight;

    /// <summary>
    /// Per-block min/max heights (row-major in Z), the coarse vertical rejection level for overlap queries and ray
    /// marching. Owned and released by the collider; may be unallocated, in which case only the global
    /// <see cref="MinHeight"/>/<see cref="MaxHeight"/> are used. See <see cref="HeightfieldCoarseGrid"/>.
    /// </summary>
    public Buffer<HeightRange> CoarseBlocks;
    /// <summary>Cell count per coarse block along each axis. Zero when no coarse grid is present.</summary>
    public int CellsPerBlock;
    /// <summary>Number of coarse blocks along local X.</summary>
    public int BlocksX;
    /// <summary>Number of coarse blocks along local Z.</summary>
    public int BlocksZ;

    /// <summary>True when the callback has been bound (the shape is usable); false for a default-constructed instance.</summary>
    public readonly bool IsBound => SampleHeightCallback != null;

    /// <summary>True when a coarse min/max block grid is available for vertical rejection.</summary>
    public readonly bool HasCoarseGrid => CellsPerBlock > 0 && CoarseBlocks.Allocated;

    /// <summary>Height interval of the coarse block containing the given cell. Only valid when <see cref="HasCoarseGrid"/>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly ref HeightRange BlockOf(int cellX, int cellZ)
        => ref CoarseBlocks[(cellZ / CellsPerBlock) * BlocksX + cellX / CellsPerBlock];

    public static ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
        => new HomogeneousCompoundShapeBatch<TerrainHeightfieldShape, Triangle, TriangleWide>(pool, initialCapacity);

    /// <summary>
    /// Two triangles per cell. Bepu's child index is an <see cref="int"/>, so this caps the field at ~1.07 G cells —
    /// ample for the 4 km map at 0.25 m (256 M cells) but not at 0.1 m (1.6 G). <see cref="Colliders.HeightfieldCollider"/>
    /// rejects configurations that would overflow rather than silently wrapping negative.
    /// </summary>
    public readonly int ChildCount => CellsX * CellsZ * 2;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly unsafe float Sample(float x, float z) => SampleHeightCallback(State, x, z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly void DecodeChild(int childIndex, out int cellX, out int cellZ, out int triHalf)
    {
        int cellIndex = childIndex >> 1;
        triHalf = childIndex & 1;
        cellX = cellIndex % CellsX;
        cellZ = cellIndex / CellsX;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly void GetCellCorners(int cellX, int cellZ, out Vector3 p00, out Vector3 p10, out Vector3 p11, out Vector3 p01)
    {
        float x0 = Origin.X + cellX * CellSize;
        float z0 = Origin.Z + cellZ * CellSize;
        float x1 = x0 + CellSize;
        float z1 = z0 + CellSize;
        p00 = new Vector3(x0, Sample(x0, z0), z0);
        p10 = new Vector3(x1, Sample(x1, z0), z0);
        p11 = new Vector3(x1, Sample(x1, z1), z1);
        p01 = new Vector3(x0, Sample(x0, z1), z1);
    }

    public readonly void GetLocalChild(int childIndex, out Triangle childData)
    {
        DecodeChild(childIndex, out int cx, out int cz, out int triHalf);
        float x0 = Origin.X + cx * CellSize;
        float z0 = Origin.Z + cz * CellSize;
        float x1 = x0 + CellSize;
        float z1 = z0 + CellSize;

        // Bepu triangles are one-sided, "clockwise in right-handed" (Triangle.RayTest uses normal = cross(c-a, b-a)).
        // For an upward-facing ground whose front/contact side is +Y, the winding must appear CW from above.
        // tri0 = (p00,p10,p11), tri1 = (p00,p11,p01); shared diagonal p00-p11 with opposite edge directions.
        //
        // Only three of the cell's four corners are on each triangle. Bepu calls this once per child, so sampling the
        // whole cell here would fetch the unused corner twice per cell — and SampleHeight is this shape's dominant cost.
        childData.A = new Vector3(x0, Sample(x0, z0), z0);
        var p11 = new Vector3(x1, Sample(x1, z1), z1);
        if (triHalf == 0)
        {
            childData.B = new Vector3(x1, Sample(x1, z0), z0);
            childData.C = p11;
        }
        else
        {
            childData.B = p11;
            childData.C = new Vector3(x0, Sample(x0, z1), z1);
        }
    }

    public readonly void GetPosedLocalChild(int childIndex, out Triangle childData, out NRigidPose childPose)
    {
        GetLocalChild(childIndex, out childData);
        var centroid = (childData.A + childData.B + childData.C) * (1f / 3f);
        childPose.Position = centroid;
        childPose.Orientation = Quaternion.Identity;
        childData.A -= centroid;
        childData.B -= centroid;
        childData.C -= centroid;
    }

    public readonly void GetLocalChild(int childIndex, ref TriangleWide target)
    {
        GetLocalChild(childIndex, out var triangle);
        Vector3Wide.WriteFirst(triangle.A, ref target.A);
        Vector3Wide.WriteFirst(triangle.B, ref target.B);
        Vector3Wide.WriteFirst(triangle.C, ref target.C);
    }

    public readonly void ComputeBounds(Quaternion orientation, out Vector3 min, out Vector3 max)
    {
        var localMin = new Vector3(Origin.X, MinHeight, Origin.Z);
        var localMax = new Vector3(Origin.X + CellsX * CellSize, MaxHeight, Origin.Z + CellsZ * CellSize);
        if (orientation == Quaternion.Identity)
        {
            min = localMin;
            max = localMax;
            return;
        }
        Matrix3x3.CreateFromQuaternion(orientation, out var r);
        min = new Vector3(float.MaxValue);
        max = new Vector3(-float.MaxValue);
        for (int i = 0; i < 8; i++)
        {
            var corner = new Vector3(
                (i & 1) != 0 ? localMax.X : localMin.X,
                (i & 2) != 0 ? localMax.Y : localMin.Y,
                (i & 4) != 0 ? localMax.Z : localMin.Z);
            Matrix3x3.Transform(corner, r, out var t);
            min = Vector3.Min(min, t);
            max = Vector3.Max(max, t);
        }
    }

    /// <summary>Clamps an XZ region to the field extent and converts to an inclusive cell range. Returns false if no overlap.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly bool ClampToFieldXz(float minX, float maxX, float minZ, float maxZ, out int cx0, out int cx1, out int cz0, out int cz1)
    {
        cx0 = cx1 = cz0 = cz1 = 0;
        float exMinX = Origin.X, exMaxX = Origin.X + CellsX * CellSize;
        float exMinZ = Origin.Z, exMaxZ = Origin.Z + CellsZ * CellSize;
        float nx = Math.Max(minX, exMinX), xx = Math.Min(maxX, exMaxX);
        float nz = Math.Max(minZ, exMinZ), zz = Math.Min(maxZ, exMaxZ);
        if (xx < nx || zz < nz) return false;
        float invCell = 1f / CellSize;
        cx0 = Math.Clamp((int)Math.Floor((nx - Origin.X) * invCell), 0, CellsX - 1);
        cx1 = Math.Clamp((int)Math.Floor((xx - Origin.X) * invCell), 0, CellsX - 1);
        cz0 = Math.Clamp((int)Math.Floor((nz - Origin.Z) * invCell), 0, CellsZ - 1);
        cz1 = Math.Clamp((int)Math.Floor((zz - Origin.Z) * invCell), 0, CellsZ - 1);
        return true;
    }

    /// <summary>
    /// Enumerates the child indices (two per cell) whose cells overlap the local-space AABB, calling
    /// <paramref name="enumerator"/> for each. Shared by the batch overlap query and the single-AABB query.
    /// </summary>
    /// <remarks>
    /// Rejection happens on all three axes: XZ against the field extent, then Y against the global height range and the
    /// coarse block grid. Without the Y levels a body high above the terrain would still emit every triangle under its
    /// bounding box every frame.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly void EnumerateChildrenInAabb<TEnumerator>(Vector3 min, Vector3 max, ref TEnumerator enumerator)
        where TEnumerator : IBreakableForEach<int>
    {
        if (!ClampToFieldXz(min.X, max.X, min.Z, max.Z, out int cx0, out int cx1, out int cz0, out int cz1))
            return;
        if (max.Y < MinHeight || min.Y > MaxHeight)
            return;
        bool coarse = HasCoarseGrid;
        for (int cz = cz0; cz <= cz1; ++cz)
        {
            int rowBase = cz * CellsX;
            int blockRowBase = coarse ? (cz / CellsPerBlock) * BlocksX : 0;
            // Block ranges are constant across CellsPerBlock consecutive cells; only refetch when the block changes.
            int currentBlockX = -1;
            float blockMin = 0f, blockMax = 0f;
            for (int cx = cx0; cx <= cx1; ++cx)
            {
                if (coarse)
                {
                    int bx = cx / CellsPerBlock;
                    if (bx != currentBlockX)
                    {
                        ref var range = ref CoarseBlocks[blockRowBase + bx];
                        blockMin = range.Min;
                        blockMax = range.Max;
                        currentBlockX = bx;
                    }
                    if (max.Y < blockMin || min.Y > blockMax) continue;
                }
                int cellIndex = rowBase + cx;
                if (!enumerator.LoopBody(cellIndex * 2)) return;
                if (!enumerator.LoopBody(cellIndex * 2 + 1)) return;
            }
        }
    }

    public readonly unsafe void FindLocalOverlaps<TOverlaps, TSubpairOverlaps>(ref Buffer<OverlapQueryForPair> pairs, BufferPool pool, Shapes shapes, ref TOverlaps overlaps)
        where TOverlaps : struct, ICollisionTaskOverlaps<TSubpairOverlaps>
        where TSubpairOverlaps : struct, ICollisionTaskSubpairOverlaps
    {
        if (!IsBound) return;
        var enumerator = new ShapeTreeOverlapEnumerator<TSubpairOverlaps> { Pool = pool };
        for (int i = 0; i < pairs.Length; ++i)
        {
            ref var pair = ref pairs[i];
            ref var field = ref Unsafe.AsRef<TerrainHeightfieldShape>(pair.Container);
            enumerator.Overlaps = Unsafe.AsPointer(ref overlaps.GetOverlapsForPair(i));
            field.EnumerateChildrenInAabb(pair.Min, pair.Max, ref enumerator);
        }
    }

    public readonly void FindLocalOverlaps<TEnumerator>(Vector3 min, Vector3 max, BufferPool pool, Shapes shapes, ref TEnumerator enumerator)
        where TEnumerator : IBreakableForEach<int>
    {
        if (!IsBound) return;
        EnumerateChildrenInAabb(min, max, ref enumerator);
    }

    public readonly unsafe void FindLocalOverlaps<TOverlaps>(Vector3 min, Vector3 max, Vector3 sweep, float maximumT, BufferPool pool, Shapes shapes, void* overlaps)
        where TOverlaps : ICollisionTaskSubpairOverlaps
    {
        if (!IsBound || overlaps == null) return;
        // Conservative: expand the query box by the full sweep extent on every axis (Y included — the Y bounds feed the
        // coarse rejection below, so skipping it there would drop contacts on a body sweeping downward into terrain).
        if (sweep.X < 0) min.X += sweep.X * maximumT; else max.X += sweep.X * maximumT;
        if (sweep.Y < 0) min.Y += sweep.Y * maximumT; else max.Y += sweep.Y * maximumT;
        if (sweep.Z < 0) min.Z += sweep.Z * maximumT; else max.Z += sweep.Z * maximumT;
        if (!ClampToFieldXz(min.X, max.X, min.Z, max.Z, out int cx0, out int cx1, out int cz0, out int cz1))
            return;
        if (max.Y < MinHeight || min.Y > MaxHeight)
            return;
        bool coarse = HasCoarseGrid;
        ref var overlapsRef = ref Unsafe.AsRef<TOverlaps>(overlaps);
        for (int cz = cz0; cz <= cz1; ++cz)
        {
            int rowBase = cz * CellsX;
            int blockRowBase = coarse ? (cz / CellsPerBlock) * BlocksX : 0;
            int currentBlockX = -1;
            float blockMin = 0f, blockMax = 0f;
            for (int cx = cx0; cx <= cx1; ++cx)
            {
                if (coarse)
                {
                    int bx = cx / CellsPerBlock;
                    if (bx != currentBlockX)
                    {
                        ref var range = ref CoarseBlocks[blockRowBase + bx];
                        blockMin = range.Min;
                        blockMax = range.Max;
                        currentBlockX = bx;
                    }
                    if (max.Y < blockMin || min.Y > blockMax) continue;
                }
                int cellIndex = rowBase + cx;
                overlapsRef.Allocate(pool) = cellIndex * 2;
                overlapsRef.Allocate(pool) = cellIndex * 2 + 1;
            }
        }
    }

    public readonly void RayTest<TRayHitHandler>(in NRigidPose pose, in RayData ray, ref float maximumT, BufferPool pool, ref TRayHitHandler hitHandler)
        where TRayHitHandler : struct, IShapeRayHitHandler
    {
        if (!IsBound) return;
        Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
        MarchRay(ray, pose, orientation, ref maximumT, ref hitHandler);
    }

    public readonly unsafe void RayTest<TRayHitHandler>(in NRigidPose pose, ref RaySource rays, BufferPool pool, ref TRayHitHandler hitHandler)
        where TRayHitHandler : struct, IShapeRayHitHandler
    {
        if (!IsBound) return;
        Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
        for (int i = 0; i < rays.RayCount; ++i)
        {
            rays.GetRay(i, out var ray, out var maxT);
            MarchRay(*ray, pose, orientation, ref *maxT, ref hitHandler);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly void MarchRay<TRayHitHandler>(in RayData ray, in NRigidPose pose, Matrix3x3 orientation, ref float maximumT, ref TRayHitHandler hitHandler)
        where TRayHitHandler : struct, IShapeRayHitHandler
    {
        // Bring the ray into shape-local space (transpose == inverse for a rotation).
        Matrix3x3.TransformTranspose(ray.Origin - pose.Position, orientation, out var localOrigin);
        Matrix3x3.TransformTranspose(ray.Direction, orientation, out var localDir);

        float invDirX = localDir.X != 0 ? 1f / localDir.X : 0f;
        float invDirZ = localDir.Z != 0 ? 1f / localDir.Z : 0f;
        float exMinX = Origin.X, exMaxX = Origin.X + CellsX * CellSize;
        float exMinZ = Origin.Z, exMaxZ = Origin.Z + CellsZ * CellSize;

        // A ray parallel to a slab either stays inside it for all t or never enters it; substituting an infinite
        // interval unconditionally conflates the two and lets an off-field ray through the extent test. The DDA then
        // clamps its starting cell into range and marches border cells the ray never actually crosses.
        //
        // This is a work guard, not a correctness one: Triangle.RayTest rejects those cells geometrically, so the result
        // was already right without it. But an axis-parallel ray just outside the field (dir.X == 0, dir.Z != 0) would
        // march an entire column of them before terminating.
        if (localDir.X == 0 && (localOrigin.X < exMinX || localOrigin.X > exMaxX)) return;
        if (localDir.Z == 0 && (localOrigin.Z < exMinZ || localOrigin.Z > exMaxZ)) return;

        // XZ slab intersection with the field extent.
        float tx1 = localDir.X != 0 ? (exMinX - localOrigin.X) * invDirX : float.NegativeInfinity;
        float tx2 = localDir.X != 0 ? (exMaxX - localOrigin.X) * invDirX : float.PositiveInfinity;
        float tz1 = localDir.Z != 0 ? (exMinZ - localOrigin.Z) * invDirZ : float.NegativeInfinity;
        float tz2 = localDir.Z != 0 ? (exMaxZ - localOrigin.Z) * invDirZ : float.PositiveInfinity;
        float tEnter = Math.Max(Math.Min(tx1, tx2), Math.Min(tz1, tz2));
        float tExit = Math.Min(Math.Max(tx1, tx2), Math.Max(tz1, tz2));
        if (tEnter > tExit || tExit < 0) return;
        float tCur = Math.Max(tEnter, 0f);
        if (tCur >= maximumT) return;

        // Global vertical rejection: a ray that starts above the field and climbs (or below and descends) never meets it.
        float yAtEntry = localOrigin.Y + localDir.Y * tCur;
        if ((localDir.Y >= 0 && yAtEntry > MaxHeight) || (localDir.Y <= 0 && yAtEntry < MinHeight)) return;

        // Amanatides-Woo DDA over XZ cells.
        float px = localOrigin.X + localDir.X * tCur;
        float pz = localOrigin.Z + localDir.Z * tCur;
        int cellX = Math.Clamp((int)Math.Floor((px - Origin.X) / CellSize), 0, CellsX - 1);
        int cellZ = Math.Clamp((int)Math.Floor((pz - Origin.Z) / CellSize), 0, CellsZ - 1);
        int stepX = Math.Sign(localDir.X);
        int stepZ = Math.Sign(localDir.Z);

        float tMaxX, tMaxZ, tDeltaX, tDeltaZ;
        if (stepX > 0) { tMaxX = (Origin.X + (cellX + 1) * CellSize - localOrigin.X) * invDirX; tDeltaX = CellSize * invDirX; }
        else if (stepX < 0) { tMaxX = (Origin.X + cellX * CellSize - localOrigin.X) * invDirX; tDeltaX = -CellSize * invDirX; }
        else { tMaxX = float.MaxValue; tDeltaX = 0f; }
        if (stepZ > 0) { tMaxZ = (Origin.Z + (cellZ + 1) * CellSize - localOrigin.Z) * invDirZ; tDeltaZ = CellSize * invDirZ; }
        else if (stepZ < 0) { tMaxZ = (Origin.Z + cellZ * CellSize - localOrigin.Z) * invDirZ; tDeltaZ = -CellSize * invDirZ; }
        else { tMaxZ = float.MaxValue; tDeltaZ = 0f; }

        bool coarse = HasCoarseGrid;
        float tCellEnter = tCur;
        while (cellX >= 0 && cellX < CellsX && cellZ >= 0 && cellZ < CellsZ)
        {
            // The ray only occupies this cell over [tCellEnter, tCellExit]; that segment's Y span is what the coarse
            // block range has to overlap for the cell to be worth two triangle tests plus four height samples.
            float tCellExit = Math.Min(Math.Min(tMaxX, tMaxZ), Math.Min(tExit, maximumT));
            if (tCellEnter > tCellExit) break;
            if (!coarse || SegmentMayHitBlock(cellX, cellZ, localOrigin.Y, localDir.Y, tCellEnter, tCellExit))
                TestCell(cellX, cellZ, localOrigin, localDir, ray, orientation, ref maximumT, ref hitHandler);

            if (tMaxX < tMaxZ) { cellX += stepX; tCellEnter = tMaxX; tMaxX += tDeltaX; }
            else { cellZ += stepZ; tCellEnter = tMaxZ; tMaxZ += tDeltaZ; }
            if (tCellEnter > tExit || tCellEnter >= maximumT) break;
        }
    }

    /// <summary>
    /// True when the ray segment over [<paramref name="t0"/>, <paramref name="t1"/>] vertically overlaps the coarse
    /// block containing the cell. Conservative: only rejects when the segment is entirely above or below the block.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly bool SegmentMayHitBlock(int cellX, int cellZ, float originY, float dirY, float t0, float t1)
    {
        ref var range = ref BlockOf(cellX, cellZ);
        float y0 = originY + dirY * t0;
        // t1 can be an unbounded maximumT; 0 * infinity would produce a NaN that defeats both comparisons below.
        float y1 = dirY == 0 ? y0 : originY + dirY * t1;
        float segMin = Math.Min(y0, y1);
        float segMax = Math.Max(y0, y1);
        return segMax >= range.Min && segMin <= range.Max;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly void TestCell<TRayHitHandler>(int cellX, int cellZ, Vector3 origin, Vector3 dir, in RayData ray, Matrix3x3 orientation, ref float maximumT, ref TRayHitHandler hitHandler)
        where TRayHitHandler : struct, IShapeRayHitHandler
    {
        int cellIndex = cellZ * CellsX + cellX;
        int child0 = cellIndex * 2;
        int child1 = child0 + 1;
        // Bepu's Mesh gates every child on AllowTest (via its Tree's RayLeafTester); skipping it would silently bypass
        // whatever per-child filtering the caller installed.
        bool allow0 = hitHandler.AllowTest(child0);
        bool allow1 = hitHandler.AllowTest(child1);
        if (!allow0 && !allow1) return;

        GetCellCorners(cellX, cellZ, out var p00, out var p10, out var p11, out var p01);
        // tri0 = (p00,p10,p11), tri1 = (p00,p11,p01) — matches GetLocalChild winding.
        if (allow0 && Triangle.RayTest(p00, p10, p11, origin, dir, out float t0, out var n0) && t0 <= maximumT)
        {
            Matrix3x3.Transform(n0, orientation, out var wn);
            hitHandler.OnRayHit(ray, ref maximumT, t0, Vector3.Normalize(wn), child0);
        }
        if (allow1 && Triangle.RayTest(p00, p11, p01, origin, dir, out float t1, out var n1) && t1 <= maximumT)
        {
            Matrix3x3.Transform(n1, orientation, out var wn);
            hitHandler.OnRayHit(ray, ref maximumT, t1, Vector3.Normalize(wn), child1);
        }
    }

    public readonly void Dispose(BufferPool pool)
    {
        // Deliberately empty. The coarse grid buffer, the height source and its GCHandle are all owned by the collider,
        // which releases them in ICollider.Detach — Stride removes shapes with Shapes.Remove, which never calls this.
        // If a caller ever switches to RemoveAndDispose, move the coarse grid's Return here instead of duplicating it.
    }
}
