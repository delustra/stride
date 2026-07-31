// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

using System.Numerics;
using BepuUtilities.Memory;

namespace Stride.BepuPhysics.Definitions.Heightfield;

/// <summary>Inclusive height interval of a coarse block of heightfield cells.</summary>
public struct HeightRange
{
    public float Min;
    public float Max;

    public HeightRange(float min, float max)
    {
        Min = min;
        Max = max;
    }
}

/// <summary>
/// Builds the coarse min/max height grid that gives <see cref="TerrainHeightfieldShape"/> a cheap vertical rejection
/// level: one <see cref="HeightRange"/> per block of <c>cellsPerBlock</c>&#xD7;<c>cellsPerBlock</c> cells.
/// </summary>
/// <remarks>
/// <para>
/// Without it the shape can only reject on XZ, so a body flying high above the terrain still gathers every cell under
/// its bounding box every frame, and a long ray marches every cell it crosses. The grid also supplies the shape's exact
/// global height bounds for the broadphase AABB, which is tighter than (and always consistent with) whatever
/// <see cref="IHeightfieldSource.GetHeightRange"/> estimates analytically.
/// </para>
/// <para>
/// <b>Soundness.</b> Every cell corner is sampled exactly once and folded into each of the (up to four) blocks whose
/// cells touch it, so a block's range bounds every triangle vertex it can emit. Rejecting against it can never drop a
/// real contact. One sample per corner is the minimum needed for that guarantee.
/// </para>
/// <para>
/// <b>Cost.</b> <c>(cellsX + 1) &#xD7; (cellsZ + 1)</c> samples at attach: ~4 M for a 1 km field at 0.5 m cells
/// (fractions of a second), but ~256 M for the 4 km map at 0.25 m. When the Phase 1 layer-stack sampler lands its own
/// precomputed min/max pyramid should feed this directly instead of being resampled; until then, set
/// <see cref="Colliders.HeightfieldCollider.CoarseBlockCells"/> to 0 to skip the pass and fall back to the source's
/// global range.
/// </para>
/// </remarks>
public static class HeightfieldCoarseGrid
{
    /// <summary>
    /// Samples the whole corner grid once and reduces it into per-block height ranges.
    /// </summary>
    /// <returns>A pool-allocated buffer of <c>blocksX * blocksZ</c> ranges, row-major in Z. The caller owns it and must
    /// return it to <paramref name="pool"/>.</returns>
    public static Buffer<HeightRange> Build(
        IHeightfieldSource source, BufferPool pool,
        Vector3 origin, float cellSize, int cellsX, int cellsZ, int cellsPerBlock,
        out int blocksX, out int blocksZ, out float globalMin, out float globalMax)
    {
        blocksX = (cellsX + cellsPerBlock - 1) / cellsPerBlock;
        blocksZ = (cellsZ + cellsPerBlock - 1) / cellsPerBlock;
        int blockCount = blocksX * blocksZ;

        pool.Take<HeightRange>(blockCount, out var blocks);
        for (int i = 0; i < blockCount; i++)
            blocks[i] = new HeightRange(float.MaxValue, float.MinValue);

        globalMin = float.MaxValue;
        globalMax = float.MinValue;

        int lastCellX = cellsX - 1;
        int lastCellZ = cellsZ - 1;
        for (int vz = 0; vz <= cellsZ; vz++)
        {
            float z = origin.Z + vz * cellSize;
            // The corner row vz is shared by cell rows vz-1 and vz, hence by the blocks containing either.
            int bz0 = Math.Max(0, vz - 1) / cellsPerBlock;
            int bz1 = Math.Min(lastCellZ, vz) / cellsPerBlock;
            for (int vx = 0; vx <= cellsX; vx++)
            {
                float h = source.SampleHeight(origin.X + vx * cellSize, z);
                if (h < globalMin) globalMin = h;
                if (h > globalMax) globalMax = h;

                int bx0 = Math.Max(0, vx - 1) / cellsPerBlock;
                int bx1 = Math.Min(lastCellX, vx) / cellsPerBlock;
                for (int bz = bz0; bz <= bz1; bz++)
                {
                    int rowBase = bz * blocksX;
                    for (int bx = bx0; bx <= bx1; bx++)
                    {
                        ref var range = ref blocks[rowBase + bx];
                        if (h < range.Min) range.Min = h;
                        if (h > range.Max) range.Max = h;
                    }
                }
            }
        }

        return blocks;
    }
}
