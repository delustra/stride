// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

using System.Linq;
using System.Threading.Tasks;

using Xunit;

using Stride.Core.Mathematics;
using Stride.Graphics;
using Stride.Graphics.Regression;
using Stride.Rendering;
using Stride.Rendering.Lights;
using Stride.Rendering.Materials;
using Stride.Rendering.Materials.ComputeColors;
using Stride.Rendering.ProceduralModels;
using Stride.Rendering.Skyboxes;
using Stride.Shaders;

namespace Stride.Engine.Tests;

/// <summary>
/// Checks that a skybox light reflects its <see cref="LightSkybox.SpecularCubeMap"/> in place of the cube map of its
/// skybox, with a skybox that reflects and with one that does not, and reflects the skybox again without it.
/// </summary>
/// <remarks>
/// A mirror floor under a skybox light alone. The skybox reflects red, the cube map set on the light is blue.
/// </remarks>
public class SkyboxLightSpecularCubeMapTest : EngineTestBase
{
    private LightSkybox light;
    private Skybox redSkybox;
    private Texture blueCubeMap;

    private Color withTheCubeMap;
    private Color withoutIt;
    private Color withTheCubeMapOverASkyboxThatReflectsNothing;

    public SkyboxLightSpecularCubeMapTest()
    {
        GraphicsDeviceManager.PreferredGraphicsProfile = [GraphicsProfile.Level_11_0];
        GraphicsDeviceManager.ShaderProfile = GraphicsProfile.Level_11_0;
    }

    protected override async Task LoadContent()
    {
        await base.LoadContent();

        AmbientLight.Intensity = 0f;

        redSkybox = new Skybox();
        redSkybox.SpecularLightingParameters.Set(SkyboxKeys.Shader, new ShaderClassSource("RoughnessCubeMapEnvironmentColor"));
        redSkybox.SpecularLightingParameters.Set(SkyboxKeys.CubeMap, CubeMapOf(Color.Red));
        blueCubeMap = CubeMapOf(Color.Blue);

        light = new LightSkybox { Skybox = redSkybox, SpecularCubeMap = blueCubeMap };
        Scene.Entities.Add(new Entity { new LightComponent { Type = light, Intensity = 1f } });

        var mirror = Material.New(GraphicsDevice, new MaterialDescriptor
        {
            Attributes =
            {
                Diffuse = new MaterialDiffuseMapFeature(new ComputeColor(new Color4(0.8f, 0.8f, 0.8f, 1f))),
                DiffuseModel = new MaterialDiffuseLambertModelFeature(),
                MicroSurface = new MaterialGlossinessMapFeature(new ComputeFloat(0.95f)),
                Specular = new MaterialMetalnessMapFeature(new ComputeFloat(1f)),
                SpecularModel = new MaterialSpecularMicrofacetModelFeature(),
            },
        }, Content);
        var floor = new PlaneProceduralModel { Size = new Vector2(10), MaterialInstance = { Material = mirror } };
        Scene.Entities.Add(new Entity { new ModelComponent(new ProceduralModelDescriptor(floor).GenerateModel(Services)) });

        Camera.Transform.Position = new Vector3(0f, 1.2f, 4f);
        Camera.Transform.Rotation = Quaternion.RotationX(-0.6f);
    }

    protected override void RegisterTests()
    {
        base.RegisterTests();

        FrameGameSystem.Draw(3, () => withTheCubeMap = ReadTheFloor());
        FrameGameSystem.Update(4, () => light.SpecularCubeMap = null);
        FrameGameSystem.Draw(6, () => withoutIt = ReadTheFloor());
        FrameGameSystem.Update(7, () =>
        {
            light.Skybox = new Skybox();
            light.SpecularCubeMap = blueCubeMap;
        });
        FrameGameSystem.Draw(9, () => withTheCubeMapOverASkyboxThatReflectsNothing = ReadTheFloor());
    }

    private Texture CubeMapOf(Color color)
    {
        const int size = 16;
        var faces = Enumerable.Range(0, 6).Select(_ => Enumerable.Repeat(color, size * size).ToArray()).ToArray();
        return Texture.NewCube(GraphicsDevice, size, PixelFormat.R8G8B8A8_UNorm, faces);
    }

    /// <summary>
    /// The pixel in the middle of the lower half of the view, where the floor is.
    /// </summary>
    private Color ReadTheFloor()
    {
        var backBuffer = GraphicsDevice.Presenter.BackBuffer;
        var pixels = backBuffer.GetData<Color>(GraphicsContext.CommandList);
        return pixels[backBuffer.Height * 3 / 4 * backBuffer.Width + backBuffer.Width / 2];
    }

    [SkippableFact]
    public void TheLightReflectsItsCubeMapInPlaceOfTheSkybox()
    {
        RunGameTest(this);

        Assert.True(withTheCubeMap.B > 120 && withTheCubeMap.R < 40, $"the floor does not reflect the cube map of the light: {withTheCubeMap}");
        Assert.True(withoutIt.R > 120 && withoutIt.B < 40, $"the floor does not reflect the skybox once the light has no cube map: {withoutIt}");
        Assert.True(withTheCubeMapOverASkyboxThatReflectsNothing.B > 120 && withTheCubeMapOverASkyboxThatReflectsNothing.R < 40,
            $"the floor does not reflect the cube map of the light over a skybox that reflects nothing: {withTheCubeMapOverASkyboxThatReflectsNothing}");
    }
}
