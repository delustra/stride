// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using BepuUtilities.Memory;

namespace Stride.BepuPhysics.Definitions.Heightfield;

/// <summary>
/// Builds a ready-to-use <see cref="TerrainHeightfieldShape"/> from a managed <see cref="IHeightfieldSource"/>,
/// including the callback bridge and the coarse min/max grid, and releases what it allocated.
/// </summary>
/// <remarks>
/// <see cref="Colliders.HeightfieldCollider"/> is the normal entry point; this factory exists so the shape can also be
/// built against a bare <see cref="BepuPhysics.Simulation"/> — benchmarks and physics-only tests that shouldn't have to
/// stand up a Stride scene to measure the narrowphase.
/// </remarks>
public static class HeightfieldShapeFactory
{
    private static readonly Stride.Core.Diagnostics.Logger Log =
        Stride.Core.Diagnostics.GlobalLogger.GetLogger(nameof(HeightfieldShapeFactory));

    /// <summary>
    /// Creates the shape. The caller owns the returned resources and must pass them back to
    /// <see cref="Release"/> once the shape is no longer registered with any simulation.
    /// </summary>
    /// <param name="coarseBlockCells">Cells per coarse min/max block, or 0 to skip the grid and use the source's
    /// global range (see <see cref="HeightfieldCoarseGrid"/>).</param>
    /// <param name="sourceHandle">Keeps the bridge (and through it the source) alive for the shape's lifetime.</param>
    public static unsafe TerrainHeightfieldShape Create(
        IHeightfieldSource source, BufferPool pool,
        Vector3 origin, float cellSize, int cellsX, int cellsZ, int coarseBlockCells,
        out GCHandle sourceHandle)
    {
        ArgumentNullException.ThrowIfNull(source);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(cellSize);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(cellsX);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(cellsZ);
        if ((long)cellsX * cellsZ * 2 > int.MaxValue)
            throw new ArgumentOutOfRangeException(nameof(cellsX), $"A {cellsX}x{cellsZ} field exceeds Bepu's int child index.");

        sourceHandle = GCHandle.Alloc(new HeightfieldSourceBridge(source), GCHandleType.Normal);
        source.GetHeightRange(out float minHeight, out float maxHeight);

        var coarseBlocks = default(Buffer<HeightRange>);
        int blocksX = 0, blocksZ = 0;
        if (coarseBlockCells > 0)
        {
            if (TryAdoptPyramid(source, pool, cellsX, cellsZ, coarseBlockCells,
                    out coarseBlocks, out blocksX, out blocksZ, out minHeight, out maxHeight))
            {
                // Pyramid adopted — no full-field sampling pass needed.
            }
            else
            {
                // No compatible pyramid available: sample every cell corner to build the coarse grid.
                coarseBlocks = HeightfieldCoarseGrid.Build(
                    source, pool, origin, cellSize, cellsX, cellsZ, coarseBlockCells,
                    out blocksX, out blocksZ, out minHeight, out maxHeight);
            }
        }

        return new TerrainHeightfieldShape
        {
            State = (void*)GCHandle.ToIntPtr(sourceHandle),
            SampleHeightCallback = &HeightfieldSourceBridge.InvokeSample,
            Origin = origin,
            CellSize = cellSize,
            CellsX = cellsX,
            CellsZ = cellsZ,
            MinHeight = minHeight,
            MaxHeight = maxHeight,
            CoarseBlocks = coarseBlocks,
            CellsPerBlock = coarseBlockCells > 0 ? coarseBlockCells : 0,
            BlocksX = blocksX,
            BlocksZ = blocksZ,
        };
    }

    /// <summary>
    /// Checks whether the source provides a structurally-compatible compiler-baked pyramid, and if so copies it into a
    /// pool-allocated buffer. Returns false (no allocation) when the source has no pyramid or the layout doesn't match,
    /// so the caller falls back to <see cref="HeightfieldCoarseGrid.Build"/>.
    /// </summary>
    /// <remarks>
    /// <b>Buffer ownership.</b> The pyramid arrays are managed/asset memory; the shape's <see cref="TerrainHeightfieldShape.CoarseBlocks"/>
    /// is a Bepu <see cref="Buffer{T}"/> allocated from the simulation's pool, and <see cref="Release"/> returns it. So
    /// the data is <b>copied</b> into a fresh pool buffer — never pointing the shape at managed memory.
    /// <para>
    /// <b>Compatibility check.</b> The pyramid's <c>cellsPerBlock</c> must equal the collider's <c>coarseBlockCells</c>,
    /// and the block counts must match <c>ceil(cells/cellsPerBlock)</c>. A mismatch produces a warning and falls back to
    /// resampling — never a silent misalignment, which would drop contacts.
    /// </para>
    /// </remarks>
    private static bool TryAdoptPyramid(
        IHeightfieldSource source, BufferPool pool,
        int cellsX, int cellsZ, int coarseBlockCells,
        out Buffer<HeightRange> blocks, out int blocksX, out int blocksZ,
        out float globalMin, out float globalMax)
    {
        blocks = default;
        blocksX = blocksZ = 0;
        globalMin = float.MaxValue;
        globalMax = float.MinValue;

        if (source is not IHeightfieldPyramidProvider provider)
            return false;

        if (!provider.TryGetHeightPyramid(
                out var minArr, out var maxArr,
                out int pyrBlocksX, out int pyrBlocksZ, out int pyrCellsPerBlock,
                out globalMin, out globalMax))
            return false;

        int expectedBlocksX = (cellsX + coarseBlockCells - 1) / coarseBlockCells;
        int expectedBlocksZ = (cellsZ + coarseBlockCells - 1) / coarseBlockCells;

        if (pyrCellsPerBlock != coarseBlockCells || pyrBlocksX != expectedBlocksX || pyrBlocksZ != expectedBlocksZ)
        {
            Log.Warning(
                $"Heightfield source provided a pyramid (cellsPerBlock={pyrCellsPerBlock}, blocks={pyrBlocksX}x{pyrBlocksZ}) " +
                $"that does not match the collider grid (cellsPerBlock={coarseBlockCells}, expected blocks={expectedBlocksX}x{expectedBlocksZ}). " +
                "Falling back to full-field resampling. Align the TerrainMap's PyramidBlockCells and grid dimensions with the collider's CellSize/CellsX/CellsZ.");
            return false;
        }

        int blockCount = pyrBlocksX * pyrBlocksZ;
        pool.Take<HeightRange>(blockCount, out blocks);
        for (int i = 0; i < blockCount; i++)
            blocks[i] = new HeightRange(minArr[i], maxArr[i]);

        blocksX = pyrBlocksX;
        blocksZ = pyrBlocksZ;
        return true;
    }

    /// <summary>Releases the coarse grid buffer and the source handle. Safe to call on already-released values.</summary>
    public static unsafe void Release(BufferPool pool, ref TerrainHeightfieldShape shape, ref GCHandle sourceHandle)
    {
        if (sourceHandle.IsAllocated)
            sourceHandle.Free();
        sourceHandle = default;
        if (shape.CoarseBlocks.Allocated)
            pool.Return(ref shape.CoarseBlocks);
        shape.CellsPerBlock = 0;
        shape.SampleHeightCallback = null;
        shape.State = null;
    }
}

/// <summary>
/// Bridges a managed <see cref="IHeightfieldSource"/> into the unmanaged <see cref="TerrainHeightfieldShape"/> callback.
/// Kept alive for the shape's lifetime via a <see cref="GCHandle"/> stored in the shape's <c>State</c>.
/// </summary>
internal sealed class HeightfieldSourceBridge
{
    public readonly IHeightfieldSource Source;
    public HeightfieldSourceBridge(IHeightfieldSource source) => Source = source;

    /// <summary>
    /// Function-pointer target invoked from the physics narrowphase. Resolves the source through its GCHandle token and
    /// samples its height.
    /// </summary>
    /// <remarks>
    /// The <see cref="GCHandle.FromIntPtr(System.IntPtr)"/> lookup plus the interface dispatch is a small per-sample
    /// cost. It also defeats the devirtualization <see cref="IHeightfieldSource"/>'s performance contract asks for, so
    /// if the benchmark ever shows sampling dominating, the fix is to pin a native sampler struct holding raw buffer
    /// pointers for specific fast source types rather than to shave this indirection.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static unsafe float InvokeSample(void* state, float x, float z)
    {
        var bridge = (HeightfieldSourceBridge)GCHandle.FromIntPtr((IntPtr)state).Target!;
        return bridge.Source.SampleHeight(x, z);
    }
}
