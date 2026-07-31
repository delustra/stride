// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.CollisionDetection.SweepTasks;

namespace Stride.BepuPhysics.Definitions.Heightfield;

/// <summary>
/// Registers the collision and sweep tasks that let <see cref="TerrainHeightfieldShape"/> collide with the convex and
/// compound shapes the game uses. Mirrors Bepu's built-in <see cref="Mesh"/> registrations in
/// <c>DefaultTypes.CreateDefaultCollisionTaskRegistry</c>/<c>CreateDefaultSweepTaskRegistry</c>, substituting
/// <see cref="TerrainHeightfieldShape"/> for <see cref="Mesh"/>.
/// </summary>
/// <remarks>
/// Internal-edge welding (the make-or-break quality item) is inherited for free: registering with
/// <see cref="ConvexMeshContinuations{TMesh}"/> + <see cref="MeshReduction"/> (both generic over any
/// <c>IHomogeneousCompoundShape&lt;Triangle, TriangleWide&gt;</c>) applies Bepu's mesh boundary-smoothing machinery to
/// the heightfield. No mesh-vs-heightfield or heightfield-vs-heightfield pairs: the game has no dynamic meshes and the
/// terrain mesh collider is being removed.
/// </remarks>
public static class HeightfieldCollisionTasks
{
    /// <summary>
    /// Registers heightfield collision and sweep tasks. Call once per <see cref="BepuPhysics.Simulation"/> after it is
    /// created (the tasks are idempotent across re-registration only if the registry is fresh).
    /// </summary>
    public static void Register(CollisionTaskRegistry collisionTasks, SweepTaskRegistry sweepTasks)
    {
        // Convex vs heightfield — MeshReduction smooths internal edges across cell boundaries.
        collisionTasks.Register(new ConvexCompoundCollisionTask<Sphere, TerrainHeightfieldShape, ConvexCompoundOverlapFinder<Sphere, SphereWide, TerrainHeightfieldShape>, ConvexMeshContinuations<TerrainHeightfieldShape>, MeshReduction>());
        collisionTasks.Register(new ConvexCompoundCollisionTask<Capsule, TerrainHeightfieldShape, ConvexCompoundOverlapFinder<Capsule, CapsuleWide, TerrainHeightfieldShape>, ConvexMeshContinuations<TerrainHeightfieldShape>, MeshReduction>());
        collisionTasks.Register(new ConvexCompoundCollisionTask<Box, TerrainHeightfieldShape, ConvexCompoundOverlapFinder<Box, BoxWide, TerrainHeightfieldShape>, ConvexMeshContinuations<TerrainHeightfieldShape>, MeshReduction>());
        collisionTasks.Register(new ConvexCompoundCollisionTask<Triangle, TerrainHeightfieldShape, ConvexCompoundOverlapFinder<Triangle, TriangleWide, TerrainHeightfieldShape>, ConvexMeshContinuations<TerrainHeightfieldShape>, MeshReduction>());
        collisionTasks.Register(new ConvexCompoundCollisionTask<Cylinder, TerrainHeightfieldShape, ConvexCompoundOverlapFinder<Cylinder, CylinderWide, TerrainHeightfieldShape>, ConvexMeshContinuations<TerrainHeightfieldShape>, MeshReduction>());
        collisionTasks.Register(new ConvexCompoundCollisionTask<ConvexHull, TerrainHeightfieldShape, ConvexCompoundOverlapFinder<ConvexHull, ConvexHullWide, TerrainHeightfieldShape>, ConvexMeshContinuations<TerrainHeightfieldShape>, MeshReduction>());

        // Compound vs heightfield — so creations (Compound/BigCompound hulls) resting on terrain work.
        collisionTasks.Register(new CompoundPairCollisionTask<Compound, TerrainHeightfieldShape, CompoundPairOverlapFinder<Compound, TerrainHeightfieldShape>, CompoundMeshContinuations<Compound, TerrainHeightfieldShape>, CompoundMeshReduction>());
        collisionTasks.Register(new CompoundPairCollisionTask<BigCompound, TerrainHeightfieldShape, CompoundPairOverlapFinder<BigCompound, TerrainHeightfieldShape>, CompoundMeshContinuations<BigCompound, TerrainHeightfieldShape>, CompoundMeshReduction>());

        // Sweeps (CCD) vs heightfield.
        sweepTasks.Register(new ConvexHomogeneousCompoundSweepTask<Sphere, SphereWide, TerrainHeightfieldShape, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Sphere, TerrainHeightfieldShape>>());
        sweepTasks.Register(new ConvexHomogeneousCompoundSweepTask<Capsule, CapsuleWide, TerrainHeightfieldShape, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Capsule, TerrainHeightfieldShape>>());
        sweepTasks.Register(new ConvexHomogeneousCompoundSweepTask<Box, BoxWide, TerrainHeightfieldShape, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Box, TerrainHeightfieldShape>>());
        sweepTasks.Register(new ConvexHomogeneousCompoundSweepTask<Cylinder, CylinderWide, TerrainHeightfieldShape, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Cylinder, TerrainHeightfieldShape>>());
        sweepTasks.Register(new ConvexHomogeneousCompoundSweepTask<Triangle, TriangleWide, TerrainHeightfieldShape, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Triangle, TerrainHeightfieldShape>>());
        sweepTasks.Register(new ConvexHomogeneousCompoundSweepTask<ConvexHull, ConvexHullWide, TerrainHeightfieldShape, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<ConvexHull, TerrainHeightfieldShape>>());
        sweepTasks.Register(new CompoundHomogeneousCompoundSweepTask<Compound, TerrainHeightfieldShape, Triangle, TriangleWide, CompoundPairSweepOverlapFinder<Compound, TerrainHeightfieldShape>>());
        sweepTasks.Register(new CompoundHomogeneousCompoundSweepTask<BigCompound, TerrainHeightfieldShape, Triangle, TriangleWide, CompoundPairSweepOverlapFinder<BigCompound, TerrainHeightfieldShape>>());
    }
}
