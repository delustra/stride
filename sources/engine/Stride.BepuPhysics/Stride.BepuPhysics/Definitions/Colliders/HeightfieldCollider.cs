// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Trees;
using BepuUtilities.Memory;
using Stride.BepuPhysics.Definitions.Heightfield;
using Stride.BepuPhysics.Systems;
using Stride.Core;
using Stride.Core.Annotations;
using Stride.Core.Mathematics;
using NRigidPose = BepuPhysics.RigidPose;

namespace Stride.BepuPhysics.Definitions.Colliders;

/// <summary>
/// A static whole-map terrain collider backed by a <see cref="TerrainHeightfieldShape"/>. Triangles are generated on
/// demand from an <see cref="IHeightfieldSource"/>; no collision mesh is stored.
/// </summary>
/// <remarks>
/// The collider bridges the managed <see cref="Source"/> into the unmanaged shape via a <see cref="GCHandle"/>-backed
/// callback (see <see cref="HeightfieldSourceBridge"/>), so any <see cref="IHeightfieldSource"/> (the Phase 1
/// layer-stack sampler, a synthetic stand-in, or a baked-from-Mesh source) drops in without changes to the shape.
/// </remarks>
[DataContract]
public sealed class HeightfieldCollider : ICollider
{
    private static readonly Stride.Core.Diagnostics.Logger Log = Stride.Core.Diagnostics.GlobalLogger.GetLogger(nameof(HeightfieldCollider));

    private CollidableComponent? _component;
    private TerrainHeightfieldShape _shape;
    private GCHandle _handle;

    CollidableComponent? ICollider.Component { get => _component; set => _component = value; }

    /// <summary>
    /// The height source sampled at contact/raycast time. Any <see cref="IHeightfieldSource"/> implementation.
    /// </summary>
    [DataMember]
    [MemberRequired(ReportAs = MemberRequiredReportType.Error)]
    public required IHeightfieldSource Source { get; set; }

    /// <summary>Local-space origin corner of the field (X,Z); the static's pose maps local to world.</summary>
    [DataMember]
    public Vector3 Origin { get; set; } = new(-500f, 0f, -500f);

    /// <summary>Edge length of one collision cell. Independently tunable from storage resolution.</summary>
    [DataMember]
    public float CellSize { get; set; } = 0.5f;

    /// <summary>Number of cells along local X.</summary>
    [DataMember]
    public int CellsX { get; set; } = 2000;

    /// <summary>Number of cells along local Z.</summary>
    [DataMember]
    public int CellsZ { get; set; } = 2000;

    /// <summary>
    /// Cell count per axis of one coarse min/max block — the vertical rejection granularity. 32 cells is 16 m at the
    /// default 0.5 m cell size. Set to 0 to skip building the grid entirely and fall back to the source's global height
    /// range (cheaper attach, much more work per query).
    /// </summary>
    [DataMember]
    public int CoarseBlockCells { get; set; } = 32;

    public int Transforms => 1;

    public void GetLocalTransforms(CollidableComponent collidable, Span<ShapeTransform> transforms)
    {
        transforms[0].PositionLocal = Vector3.Zero;
        transforms[0].RotationLocal = Quaternion.Identity;
        transforms[0].Scale = Vector3.One;
    }

    bool ICollider.TryAttach(Shapes shapes, BufferPool pool, ShapeCacheSystem shapeCache, bool shouldCalculateInertia, out TypedIndex index, out Vector3 centerOfMass, out BodyInertia inertia)
        => TryAttachCore(shapes, pool, out index, out centerOfMass, out inertia);

    private bool TryAttachCore(Shapes shapes, BufferPool pool, out TypedIndex index, out Vector3 centerOfMass, out BodyInertia inertia)
    {
        Debug.Assert(_component is not null);
        index = default;
        // A static collider has no inertia; the value is ignored by StaticComponent.AttachInner.
        centerOfMass = Vector3.Zero;
        inertia = default;

        // ReAttach normally pairs with Detach, but a previously failed attach can leave this collider holding native
        // resources with no shape index to detach against. Release before allocating so re-attach can't accumulate them.
        ReleaseNativeResources(pool);

        // A collider is authored data; bad values arrive from the editor rather than from code, so report and decline
        // rather than throwing out of the processor's attach loop.
        if (CellSize <= 0f || CellsX <= 0 || CellsZ <= 0)
        {
            Log.Error($"{nameof(HeightfieldCollider)} needs a positive {nameof(CellSize)} and cell count; got {CellSize} / {CellsX}x{CellsZ}.");
            return false;
        }
        if ((long)CellsX * CellsZ * 2 > int.MaxValue)
        {
            Log.Error($"{nameof(HeightfieldCollider)} field of {CellsX}x{CellsZ} cells exceeds Bepu's int child index; reduce the cell count or increase {nameof(CellSize)}.");
            return false;
        }

        _shape = HeightfieldShapeFactory.Create(
            Source, pool, Origin.ToNumeric(), CellSize, CellsX, CellsZ, CoarseBlockCells, out _handle);
        index = shapes.Add(_shape);
        return true;
    }

    void ICollider.Detach(Shapes shapes, BufferPool pool, TypedIndex index)
    {
        shapes.Remove(index);
        ReleaseNativeResources(pool);
    }

    private void ReleaseNativeResources(BufferPool pool)
    {
        HeightfieldShapeFactory.Release(pool, ref _shape, ref _handle);
    }

    void ICollider.AppendModel(List<BasicMeshBuffers> buffer, ShapeCacheSystem shapeCache, out object? cacheOut)
    {
        // The heightfield has no Stride Model for the editor gizmo. Emit one empty buffer to satisfy the
        // CollidableGizmo contract (one buffer per Transform); the gizmo skips empty buffers, so no gizmo is drawn.
        // Provide a visual stand-in mesh externally (e.g. via TerrainDebugMeshBuilder) if you need to see the terrain.
        buffer.Add(new BasicMeshBuffers());
        cacheOut = null;
    }

    void ICollider.RayTest<TRayHitHandler>(Shapes shapes, TypedIndex shapeIndex, in NRigidPose pose, in RayData ray, ref float maximumT, ref TRayHitHandler hitHandler, BufferPool pool)
    {
        Debug.Assert(shapeIndex.Type == TerrainHeightfieldShape.TypeId);
        shapes.GetShape<TerrainHeightfieldShape>(shapeIndex.Index).RayTest(pose, in ray, ref maximumT, pool, ref hitHandler);
    }
}
