// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

namespace Stride.BepuPhysics.Definitions.Heightfield;

/// <summary>
/// Provides height samples for a <see cref="TerrainHeightfieldShape"/>.
/// </summary>
/// <remarks>
/// <para>
/// This is the only dependency of the heightfield collider on the terrain data. The Phase 1 layer-stack
/// sampler (base + detail + stamps) implements this interface so the real sampler drops in without any
/// changes to the shape or its collision tasks.
/// </para>
/// <para>
/// Coordinates are in the shape's <b>local</b> space (the static's pose maps local to world). For a static
/// placed at the map origin with identity rotation, local space == world space.
/// </para>
/// <para>
/// <b>Performance contract:</b> <see cref="SampleHeight"/> is called from the physics narrowphase inner loop
/// (4 calls per overlapped cell). Implementations must be allocation-free and avoid virtual dispatch where
/// possible (prefer a sealed type so the JIT can devirtualize). <see cref="GetHeightRange"/> must be O(1) —
/// return precomputed bounds, do not scan.
/// </para>
/// </remarks>
public interface IHeightfieldSource
{
    /// <summary>
    /// Samples the terrain height at the given local (x, z) coordinate.
    /// </summary>
    /// <param name="x">Local X coordinate.</param>
    /// <param name="z">Local Z coordinate.</param>
    /// <returns>Height (local Y) at (x, z).</returns>
    float SampleHeight(float x, float z);

    /// <summary>
    /// Gets the minimum and maximum height over the whole field, used for the broadphase bounding box.
    /// Must be O(1) (return precomputed values).
    /// </summary>
    void GetHeightRange(out float minHeight, out float maxHeight);
}
