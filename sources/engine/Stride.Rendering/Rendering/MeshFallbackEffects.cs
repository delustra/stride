// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.
using System;
using Stride.Graphics;
using Stride.Rendering.Lights;
using Stride.Rendering.Materials;
using Stride.Rendering.Materials.ComputeColors;
using Stride.Shaders;
using Stride.Shaders.Compiler;

namespace Stride.Rendering
{
    /// <summary>
    /// Builds the effect a <see cref="MeshRenderFeature"/> draws with while a mesh's real effect is still compiling
    /// or failed to compile. It keeps the mesh's diffuse texture or color with Lambert shading, its skinning and its
    /// instancing, and drops every other material feature, so all meshes share a few permutations that are usually
    /// already compiled and never depend on user shaders.
    /// </summary>
    public class MeshFallbackEffects
    {
        private const int MaxSkinningBones = 56;

        private readonly EffectSystem effectSystem;
        private readonly Material colorMaterial;
        private readonly Material textureMaterial;

        public MeshFallbackEffects(GraphicsDevice graphicsDevice, EffectSystem effectSystem)
        {
            this.effectSystem = effectSystem ?? throw new ArgumentNullException(nameof(effectSystem));

            colorMaterial = Material.New(graphicsDevice, new MaterialDescriptor
            {
                Attributes =
                {
                    Diffuse = new MaterialDiffuseMapFeature(new ComputeTextureColor()),
                    DiffuseModel = new MaterialDiffuseLambertModelFeature(),
                },
            });

            textureMaterial = Material.New(graphicsDevice, new MaterialDescriptor
            {
                Attributes =
                {
                    Diffuse = new MaterialDiffuseMapFeature(new ComputeTextureColor { FallbackValue = null }), // Do not use fallback value, we want a DiffuseMap
                    DiffuseModel = new MaterialDiffuseLambertModelFeature(),
                },
            });
        }

        /// <summary>
        /// Creates the compiler parameters of the fallback effect for <paramref name="renderMesh"/>.
        /// </summary>
        public CompilerParameters CreateCompilerParameters(RenderMesh renderMesh)
        {
            return CreateCompilerParameters(renderMesh, null);
        }

        /// <summary>
        /// Creates the compiler parameters of the fallback effect for <paramref name="renderMesh"/> drawn by
        /// <paramref name="renderEffect"/>: the mesh's diffuse map or color and its skinning, and the instancing
        /// <paramref name="renderEffect"/> was validated with, so an instanced mesh keeps its instances while its real
        /// effect compiles.
        /// </summary>
        public CompilerParameters CreateCompilerParameters(RenderMesh renderMesh, RenderEffect renderEffect)
        {
            var hasDiffuseMap = renderMesh.MaterialPass.Parameters.ContainsKey(MaterialKeys.DiffuseMap);

            // Support skinning
            var skinned = renderMesh.Mesh.Skinning != null && renderMesh.Mesh.Skinning.Bones.Length <= MaxSkinningBones;
            var meshParameters = renderMesh.Mesh.Parameters;
            var compilerParameters = CreateCompilerParameters(hasDiffuseMap, hasInstancing: false, modelTransformUsage: 0,
                hasSkinningPosition: skinned && meshParameters.Get(MaterialKeys.HasSkinningPosition),
                hasSkinningNormal: skinned && meshParameters.Get(MaterialKeys.HasSkinningNormal),
                hasSkinningTangent: skinned && meshParameters.Get(MaterialKeys.HasSkinningTangent));

            // Support instancing: the keys InstancingRenderFeature validated the real effect with
            if (renderEffect != null)
            {
                var validated = renderEffect.EffectValidator.EffectValues;
                for (int i = 0; i < validated.Count; i++)
                {
                    var entry = validated.Items[i];
                    if (entry.Key == StrideEffectBaseKeys.HasInstancing)
                        compilerParameters.Set(StrideEffectBaseKeys.HasInstancing, (bool)entry.Value);
                    else if (entry.Key == StrideEffectBaseKeys.ModelTransformUsage)
                        compilerParameters.Set(StrideEffectBaseKeys.ModelTransformUsage, (int)entry.Value);
                }
            }

            return compilerParameters;
        }

        /// <summary>
        /// Creates the compiler parameters of a fallback effect without a mesh: a diffuse map or a color, Lambert
        /// shading, a constant white light, and the instancing permutation when <paramref name="hasInstancing"/>.
        /// Loading an effect with them ahead of its first draw spares that draw the wait for the fallback's own
        /// compilation.
        /// </summary>
        /// <param name="hasDiffuseMap">Whether the fallback draws a diffuse map, or a color.</param>
        /// <param name="hasInstancing">Whether the fallback draws instances.</param>
        /// <param name="modelTransformUsage">With instancing, the <c>ModelTransformUsage</c> of the instances.</param>
        public CompilerParameters CreateCompilerParameters(bool hasDiffuseMap, bool hasInstancing = false, int modelTransformUsage = 0)
        {
            return CreateCompilerParameters(hasDiffuseMap, hasInstancing, modelTransformUsage, hasSkinningPosition: false, hasSkinningNormal: false, hasSkinningTangent: false);
        }

        /// <summary>
        /// Creates the compiler parameters of a fallback effect without a mesh, with the skinning permutation of the
        /// mesh it will stand in for (the keys a skinned mesh carries in its parameters, up to <see cref="MaxSkinningBones"/>
        /// bones): a game loads the skinned fallbacks ahead of their first draw as it loads the others.
        /// </summary>
        /// <param name="hasDiffuseMap">Whether the fallback draws a diffuse map, or a color.</param>
        /// <param name="hasInstancing">Whether the fallback draws instances.</param>
        /// <param name="modelTransformUsage">With instancing, the <c>ModelTransformUsage</c> of the instances.</param>
        /// <param name="hasSkinningPosition">Whether the mesh skins its positions.</param>
        /// <param name="hasSkinningNormal">Whether the mesh skins its normals.</param>
        /// <param name="hasSkinningTangent">Whether the mesh skins its tangents.</param>
        public CompilerParameters CreateCompilerParameters(bool hasDiffuseMap, bool hasInstancing, int modelTransformUsage, bool hasSkinningPosition, bool hasSkinningNormal, bool hasSkinningTangent)
        {
            var fallbackMaterial = hasDiffuseMap ? textureMaterial : colorMaterial;

            // High priority
            var compilerParameters = new CompilerParameters();
            compilerParameters.EffectParameters.TaskPriority = -1;

            // Set material permutations
            compilerParameters.Set(MaterialKeys.PixelStageSurfaceShaders, fallbackMaterial.Passes[0].Parameters.Get(MaterialKeys.PixelStageSurfaceShaders));
            compilerParameters.Set(MaterialKeys.PixelStageStreamInitializer, fallbackMaterial.Passes[0].Parameters.Get(MaterialKeys.PixelStageStreamInitializer));

            // Set lighting permutations (use custom white light, since this effect will not be processed by the lighting render feature)
            compilerParameters.Set(LightingKeys.EnvironmentLights, new ShaderSourceCollection { new ShaderClassSource("LightConstantWhite") });

            if (hasSkinningPosition || hasSkinningNormal || hasSkinningTangent)
            {
                compilerParameters.Set(MaterialKeys.HasSkinningPosition, hasSkinningPosition);
                compilerParameters.Set(MaterialKeys.HasSkinningNormal, hasSkinningNormal);
                compilerParameters.Set(MaterialKeys.HasSkinningTangent, hasSkinningTangent);
                compilerParameters.Set(MaterialKeys.SkinningMaxBones, MaxSkinningBones);
            }

            if (hasInstancing)
            {
                compilerParameters.Set(StrideEffectBaseKeys.ModelTransformUsage, modelTransformUsage);
                compilerParameters.Set(StrideEffectBaseKeys.HasInstancing, true);
            }

            return compilerParameters;
        }

        /// <summary>
        /// Loads the fallback effect for <paramref name="renderEffect"/>. Matches <see cref="RootEffectRenderFeature.ComputeFallbackEffectDelegate"/>.
        /// </summary>
        /// <returns>The fallback effect, or <c>null</c> when it could not be loaded, in which case the render effect is flagged as errored.</returns>
        public Effect ComputeFallbackEffect(RenderObject renderObject, RenderEffect renderEffect, RenderEffectState renderEffectState)
        {
            return ComputeFallbackEffect(renderObject, renderEffect, renderEffectState, null);
        }

        /// <summary>
        /// Same as <see cref="ComputeFallbackEffect(RenderObject, RenderEffect, RenderEffectState)"/>, with a hook to adjust
        /// the compiler parameters before the effect is loaded.
        /// </summary>
        public Effect ComputeFallbackEffect(RenderObject renderObject, RenderEffect renderEffect, RenderEffectState renderEffectState, Action<RenderEffect, RenderEffectState, CompilerParameters> customizeParameters)
        {
            try
            {
                var renderMesh = (RenderMesh)renderObject;

                var compilerParameters = CreateCompilerParameters(renderMesh, renderEffect);
                customizeParameters?.Invoke(renderEffect, renderEffectState, compilerParameters);

                if (renderEffectState == RenderEffectState.Error)
                {
                    // Retry every few seconds
                    renderEffect.RetryTime = DateTime.UtcNow + TimeSpan.FromSeconds(5);
                }

                var effect = effectSystem.LoadEffect(renderEffect.EffectSelector.EffectName, compilerParameters).WaitForResult();

                // Initialize parameters with material ones (need a CopyTo?)
                renderEffect.FallbackParameters = new ParameterCollection(renderMesh.MaterialPass.Parameters);

                return effect;
            }
            catch
            {
                // TODO: Log or rethrow?
                renderEffect.State = RenderEffectState.Error;
                return null;
            }
        }
    }
}
