// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

namespace Stride.BepuPhysics.Definitions.Heightfield;

/// <summary>
/// Optional capability of an <see cref="IHeightfieldSource"/>: provides compiler-baked coarse min/max height ranges
/// so the collider skips the attach-time resampling pass.
/// </summary>
/// <remarks>
/// <para>
/// When the source implements this interface and the pyramid is structurally compatible with the collider's grid
/// parameters, <see cref="HeightfieldShapeFactory.Create"/> copies the precomputed ranges into its pool-allocated
/// <see cref="Buffer{T}"/> instead of calling <see cref="HeightfieldCoarseGrid.Build"/> — which would sample every
/// cell corner (~256 M samples for the 4 km map at 0.25 m). The copy is tiny (blocks, not corners); the win being
/// removed is the full-field sampling pass.
/// </para>
/// <para>
/// <b>Compatibility</b> is validated by the factory: the pyramid's <c>cellsPerBlock</c> must equal the collider's
/// <c>CoarseBlockCells</c>, and the block counts must match <c>ceil(cellsX/cellsPerBlock)</c>. A mismatch falls back
/// to resampling with a log warning — never a silent misalignment, which would produce dropped contacts.
/// </para>
/// <para>
/// <b>Conservativeness.</b> The pyramid must bound the composited surface from outside (base + detail bound + stamps).
/// An underestimate silently drops contacts — the dangerous failure mode. The compiler (task 93's
/// <c>TerrainHeightPyramid.Build</c>) errs on the wide side.
/// </para>
/// </remarks>
public interface IHeightfieldPyramidProvider
{
    /// <summary>
    /// Returns the precomputed coarse min/max pyramid if one is available.
    /// </summary>
    /// <param name="minBlocks"><c>blocksX * blocksZ</c> min values, row-major in Z (<c>bz * blocksX + bx</c>).</param>
    /// <param name="maxBlocks"><c>blocksX * blocksZ</c> max values, same layout as <paramref name="minBlocks"/>.</param>
    /// <param name="blocksX">Number of blocks along X.</param>
    /// <param name="blocksZ">Number of blocks along Z.</param>
    /// <param name="cellsPerBlock">Cells per block along each axis (matches the collider's <c>CoarseBlockCells</c> when compatible).</param>
    /// <param name="globalMin">Conservative global minimum height over the composited surface.</param>
    /// <param name="globalMax">Conservative global maximum height over the composited surface.</param>
    /// <returns>True if a pyramid is available; false to fall back to per-corner resampling.</returns>
    bool TryGetHeightPyramid(
        out float[] minBlocks, out float[] maxBlocks,
        out int blocksX, out int blocksZ, out int cellsPerBlock,
        out float globalMin, out float globalMax);
}
