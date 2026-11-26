// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

using System.Threading.Tasks;

using Xunit;

using Stride.Core.Mathematics;
using Stride.Graphics;
using Stride.Graphics.Regression;
using Stride.Rendering;
using Stride.Rendering.Colors;
using Stride.Rendering.Compositing;
using Stride.Rendering.Images;
using Stride.Rendering.Lights;
using Stride.Rendering.Materials;
using Stride.Rendering.Materials.ComputeColors;
using Stride.Rendering.ProceduralModels;

namespace Stride.Engine.Tests;

/// <summary>
/// Checks that a local reflection found on screen replaces the reflection of the environment, and that the floor keeps the
/// environment where no ray finds the scene.
/// </summary>
/// <remarks>
/// A mirror floor under a blue ambient light, with a red glowing cube standing on it. The floor in front of the cube
/// reflects the cube: red, with no blue of the environment left under it. The floor beside it reflects the environment: blue.
/// </remarks>
public class LocalReflectionsEnvironmentTest : EngineTestBase
{
    // The temporal pass of the reflections builds up over the previous frames
    private const int ReadbackFrame = 12;

    private static readonly Vector3 InFrontOfTheCube = new(0f, 0f, 0.9f);
    private static readonly Vector3 BesideTheCube = new(2.2f, 0f, 0.9f);

    private Color reflectingTheCube;
    private Color reflectingTheEnvironment;

    public LocalReflectionsEnvironmentTest()
    {
        GraphicsDeviceManager.PreferredGraphicsProfile = [GraphicsProfile.Level_11_0];
        GraphicsDeviceManager.ShaderProfile = GraphicsProfile.Level_11_0;

        // A fixed time step: the reflections jitter with time
        IsFixedTimeStep = true;
        ForceOneUpdatePerDraw = true;
        IsDrawDesynchronized = false;
    }

    protected override async Task LoadContent()
    {
        await base.LoadContent();

        var postEffects = new PostProcessingEffects();
        postEffects.AmbientOcclusion.Enabled = false;
        postEffects.DepthOfField.Enabled = false;
        postEffects.Bloom.Enabled = false;
        postEffects.LightStreak.Enabled = false;
        postEffects.LensFlare.Enabled = false;
        postEffects.Antialiasing.Enabled = false;
        postEffects.ColorTransforms.Enabled = false;
        postEffects.LocalReflections.Enabled = true;
        ((ForwardRenderer)SceneSystem.GraphicsCompositor.SingleView).PostEffects = postEffects;

        ((LightAmbient)AmbientLight.Type).Color = new ColorRgbProvider(Color.Blue);
        AmbientLight.Intensity = 1f;

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
        var glow = Material.New(GraphicsDevice, new MaterialDescriptor
        {
            Attributes =
            {
                Emissive = new MaterialEmissiveMapFeature(new ComputeColor(new Color4(1f, 0f, 0f, 1f))),
            },
        }, Content);

        Scene.Entities.Add(CreateModel(new PlaneProceduralModel { Size = new Vector2(10), MaterialInstance = { Material = mirror } }, Vector3.Zero));
        Scene.Entities.Add(CreateModel(new CubeProceduralModel { Size = new Vector3(1f), MaterialInstance = { Material = glow } }, new Vector3(0f, 0.5f, 0f)));

        Camera.Transform.Position = new Vector3(0f, 1.2f, 4f);
        Camera.Transform.Rotation = Quaternion.RotationX(-0.3f);
    }

    protected override void RegisterTests()
    {
        base.RegisterTests();

        FrameGameSystem.Draw(ReadbackFrame, () =>
        {
            var backBuffer = GraphicsDevice.Presenter.BackBuffer;
            var pixels = backBuffer.GetData<Color>(GraphicsContext.CommandList);
            reflectingTheCube = pixels[IndexOf(InFrontOfTheCube, backBuffer)];
            reflectingTheEnvironment = pixels[IndexOf(BesideTheCube, backBuffer)];
        });
    }

    private int IndexOf(Vector3 world, Texture target)
    {
        var camera = Camera.Get<CameraComponent>();
        var clip = Vector3.TransformCoordinate(world, camera.ViewProjectionMatrix);
        var x = (int)((clip.X * 0.5f + 0.5f) * target.Width);
        var y = (int)((0.5f - clip.Y * 0.5f) * target.Height);
        return y * target.Width + x;
    }

    private Entity CreateModel(PrimitiveProceduralModelBase model, Vector3 position)
    {
        var entity = new Entity { new ModelComponent(new ProceduralModelDescriptor(model).GenerateModel(Services)) };
        entity.Transform.Position = position;
        return entity;
    }

    [SkippableFact]
    public void AReflectionFoundOnScreenReplacesTheEnvironment()
    {
        RunGameTest(this);

        Assert.True(reflectingTheCube.R > 120, $"the floor in front of the cube does not reflect it: {reflectingTheCube}");
        Assert.True(reflectingTheCube.B < 40, $"the reflection of the environment is left under the reflection of the cube: {reflectingTheCube}");
        Assert.True(reflectingTheEnvironment.B > 120 && reflectingTheEnvironment.R < 40, $"the floor beside the cube does not reflect the environment: {reflectingTheEnvironment}");
    }
}
