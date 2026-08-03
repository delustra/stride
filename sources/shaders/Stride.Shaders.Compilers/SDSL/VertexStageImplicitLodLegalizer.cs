using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Stride.Shaders.Compilers.SDSL;

/// <summary>
/// Post-merge SPIR-V fixup that rewrites implicit-LOD texture samples which are
/// illegal in their execution model.
/// <para>
/// SPIR-V forbids <c>OpImageSampleImplicitLod</c> (and its Dref/Proj variants) in
/// the Vertex, TessellationControl, TessellationEvaluation and Geometry execution
/// models — those stages have no screen-space derivatives, so the implicit LOD is
/// undefined. HLSL permits <c>Texture.Sample(...)</c> in a vertex shader (FXC lowers
/// it to LOD 0), and the SDSL frontend faithfully emits <c>OpImageSampleImplicitLod</c>
/// for <c>.Sample()</c> regardless of stage (<c>TextureMethodsImplementations.cs</c>).
/// The result is structurally invalid SPIR-V that spirv-opt rejects on input
/// (<c>InternalError</c>) — the whole module fails to legalize.
/// </para>
/// <para>
/// This mirrors what an HLSL compiler does internally: for any implicit-LOD sample
/// reachable from a non-fragment/non-compute entry point, convert it to the matching
/// explicit-LOD opcode with a <c>Lod</c> operand of <c>0.0</c>. Fragment and
/// GLCompute/Mesh/Task entry points (which legitimately use implicit LOD) are left
/// untouched. Operates on the raw SPIR-V word stream so it needs no knowledge of the
/// high-level instruction/GC APIs.
/// </para>
/// </summary>
internal static class VertexStageImplicitLodLegalizer
{
    // Execution models that cannot use implicit-LOD instructions.
    const int ModelVertex = 0;
    const int ModelTessellationControl = 1;
    const int ModelTessellationEvaluation = 2;
    const int ModelGeometry = 3;

    const int OpEntryPoint = 15;
    const int OpTypeFloat = 22;
    const int OpConstant = 43;
    const int OpFunction = 54;
    const int OpFunctionEnd = 56;
    const int OpFunctionCall = 57;

    // ImageOperands mask bits (SPIR-V spec).
    const int BiasBit = 0x1;     // incompatible with explicit LOD — dropped
    const int LodBit = 0x2;      // the operand we add
    const int GradBit = 0x4;

    // ImplicitLod opcode -> ExplicitLod opcode. (operands-word index: 5 for non-Dref, 6 for Dref)
    static readonly Dictionary<int, int> ImplicitToExplicit = new()
    {
        { 87, 88 },  // OpImageSampleImplicitLod           -> OpImageSampleExplicitLod            (idx 5)
        { 89, 90 },  // OpImageSampleDrefImplicitLod       -> OpImageSampleDrefExplicitLod        (idx 6)
        { 91, 92 },  // OpImageSampleProjImplicitLod       -> OpImageSampleProjExplicitLod        (idx 5)
        { 93, 94 },  // OpImageSampleProjDrefImplicitLod   -> OpImageSampleProjDrefExplicitLod    (idx 6)
    };

    // Remaining operand bits (after Bias/Lod) in ascending order with their id-ref counts.
    static readonly (int bit, int count)[] OperandBits =
    {
        (GradBit, 2), (0x8, 1), (0x10, 1), (0x20, 1), (0x40, 1), (0x100, 1),
    };

    /// <summary>
    /// Rewrites implicit-LOD samples reachable from non-implicit-LOD stages to explicit
    /// LOD 0. Returns the original span unchanged if no rewrite was needed.
    /// </summary>
    public static Span<byte> Legalize(Span<byte> spirvBytes)
    {
        if (spirvBytes.Length < 20 || (spirvBytes.Length & 3) != 0)
            return spirvBytes;

        var words = MemoryMarshal.Cast<byte, uint>(spirvBytes);
        var result = LegalizeCore(words);
        return result is null ? spirvBytes : MemoryMarshal.AsBytes<uint>((Span<uint>)result);
    }

    static uint[]? LegalizeCore(ReadOnlySpan<uint> w)
    {
        // Parse into an instruction index.
        var instrs = new List<Instr>(w.Length / 8);
        int i = 5; // skip 5-word header
        while (i < w.Length)
        {
            uint first = w[i];
            int wc = (int)(first >> 16);
            int op = (int)(first & 0xFFFF);
            if (wc == 0 || i + wc > w.Length) break;
            instrs.Add(new Instr(op, i, wc));
            i += wc;
        }

        // Entry points: function id -> execution model. OpEntryPoint = [wc|op, Model, FuncId, name...]
        var entryModels = new Dictionary<int, int>();
        foreach (var ins in instrs)
            if (ins.Op == OpEntryPoint && ins.WC >= 3)
                entryModels[(int)w[ins.Start + 2]] = (int)w[ins.Start + 1];

        // Early-out: no non-implicit-LOD entry point -> nothing to do.
        bool anyTargetStage = false;
        foreach (var model in entryModels.Values)
            if (IsNoImplicitLodStage(model)) { anyTargetStage = true; break; }
        if (!anyTargetStage)
            return null;

        // Call graph: caller -> callees. OpFunction Result Id is word[2]; OpFunctionCall target is word[3].
        var calls = new Dictionary<int, List<int>>();
        var funcRanges = new List<(int funcId, int startIdx, int endIdx)>();
        int curFunc = -1, funcStartIdx = -1;
        for (int idx = 0; idx < instrs.Count; idx++)
        {
            var ins = instrs[idx];
            if (ins.Op == OpFunction && ins.WC >= 3) { curFunc = (int)w[ins.Start + 2]; funcStartIdx = idx; }
            else if (ins.Op == OpFunctionCall && ins.WC >= 4 && curFunc >= 0)
            {
                int callee = (int)w[ins.Start + 3];
                if (!calls.TryGetValue(curFunc, out var list)) calls[curFunc] = list = new();
                list.Add(callee);
            }
            else if (ins.Op == OpFunctionEnd && curFunc >= 0)
            {
                funcRanges.Add((curFunc, funcStartIdx, idx));
                curFunc = -1; funcStartIdx = -1;
            }
        }

        // Reachability (transitive closure) from non-implicit-LOD entry points.
        var reachable = new HashSet<int>();
        var queue = new Queue<int>();
        foreach (var (funcId, model) in entryModels)
            if (IsNoImplicitLodStage(model) && reachable.Add(funcId)) queue.Enqueue(funcId);
        while (queue.Count > 0)
        {
            if (calls.TryGetValue(queue.Dequeue(), out var callees))
                foreach (var c in callees)
                    if (reachable.Add(c)) queue.Enqueue(c);
        }

        // Float-32 type id and an existing float 0.0 constant (Stride modules always have one).
        int float32Type = -1, float0Id = -1;
        foreach (var ins in instrs)
        {
            if (ins.Op == OpTypeFloat && ins.WC >= 3 && w[ins.Start + 2] == 32)
                float32Type = (int)w[ins.Start + 1];
            else if (float0Id < 0 && ins.Op == OpConstant && ins.WC >= 4
                     && w[ins.Start + 1] == (uint)float32Type && w[ins.Start + 3] == 0u)
                float0Id = (int)w[ins.Start + 2];
        }
        if (float32Type < 0)
            return null; // no float type -> no float sampling to fix

        int bound = (int)w[3];
        bool createFloat0 = float0Id < 0;
        if (createFloat0) float0Id = bound++;

        // Per-instruction owning function.
        var instrFunc = new int[instrs.Count];
        Array.Fill(instrFunc, -1);
        foreach (var (funcId, sIdx, eIdx) in funcRanges)
            for (int k = sIdx; k <= eIdx && k < instrFunc.Length; k++) instrFunc[k] = funcId;

        // Emit. Header first (Bound patched last); instructions copied or rewritten;
        // an OpConstant 0.0 appended only if we had to allocate a fresh id.
        var output = new List<uint>(w.Length + 16);
        output.AddRange(w.Slice(0, 5));

        bool changed = false;
        for (int idx = 0; idx < instrs.Count; idx++)
        {
            var ins = instrs[idx];
            if (ImplicitToExplicit.TryGetValue(ins.Op, out int explicitOp))
            {
                int owner = instrFunc[idx];
                if (owner >= 0 && reachable.Contains(owner))
                {
                    output.AddRange(RewriteInstruction(w, ins, explicitOp, float0Id));
                    changed = true;
                    continue;
                }
            }
            output.AddRange(w.Slice(ins.Start, ins.WC));
        }

        if (!changed)
            return null;

        if (createFloat0)
            output.AddRange(new uint[] { (4u << 16) | OpConstant, (uint)float32Type, (uint)(bound - 1), 0u });

        var result = output.ToArray();
        result[3] = (uint)bound; // patch Bound
        return result;
    }

    static IEnumerable<uint> RewriteInstruction(ReadOnlySpan<uint> w, in Instr ins, int explicitOp, int lodId)
    {
        bool isDref = ins.Op == 89 || ins.Op == 93;
        int operandsIdx = isDref ? 6 : 5; // index of the Image Operands mask word (if present)
        bool hasOperands = ins.WC > operandsIdx;
        int oldMask = hasOperands ? (int)w[ins.Start + operandsIdx] : 0;

        // Collect kept operand ids (in ascending bit order), skipping Bias (incompatible) — none for plain .Sample().
        var kept = new List<uint>();
        if (hasOperands && oldMask != 0)
        {
            int p = ins.Start + operandsIdx + 1;
            if ((oldMask & BiasBit) != 0) p += 1; // drop the Bias id
            foreach (var (bit, count) in OperandBits)
            {
                if ((oldMask & bit) != 0)
                    for (int c = 0; c < count && p < ins.Start + ins.WC; c++) kept.Add(w[p++]);
            }
        }

        int newMask = (oldMask & ~BiasBit) | LodBit;

        // [wc|explicitOp, ResultType, ResultId, SampledImage, Coordinate, (Dref,), Mask, LodId, ...kept]
        var result = new List<uint>(operandsIdx + 2 + kept.Count);
        for (int k = 0; k < operandsIdx; k++) result.Add(w[ins.Start + k]);
        result.Add((uint)newMask);
        result.Add((uint)lodId);       // Lod (bit 0x2) immediately follows the mask, before higher bits
        result.AddRange(kept);         // Grad/Offset/etc. (bits >= 0x4) in their original order
        result[0] = ((uint)result.Count << 16) | (uint)explicitOp;
        return result;
    }

    static bool IsNoImplicitLodStage(int model)
        => model is ModelVertex or ModelTessellationControl or ModelTessellationEvaluation or ModelGeometry;

    readonly record struct Instr(int Op, int Start, int WC);
}
