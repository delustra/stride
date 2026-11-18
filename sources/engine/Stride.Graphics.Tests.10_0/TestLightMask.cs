// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Graphics.Regression;
using Stride.Rendering;
using Stride.Rendering.Colors;
using Stride.Rendering.Compositing;
using Stride.Rendering.Lights;
using Stride.Rendering.Materials;
using Stride.Rendering.Materials.ComputeColors;
using Stride.Rendering.ProceduralModels;
using Xunit;

namespace Stride.Graphics.Tests
{
    /// <summary>
    /// Draws the same white cube in two views in one frame, through two camera renderers with different light masks.
    /// Each view is lit only by the lights that are in a group of its light mask.
    /// </summary>
    public class TestLightMask : GameTestBase
    {
        private const int Size = 64;

        private enum Lights
        {
            /// <summary>Red lights (ambient and shadowed directional) in the first group, blue ones in the second.</summary>
            RedAndBlueMasked,

            /// <summary>The same red and blue lights, left in every light group.</summary>
            RedAndBlueDefault,

            /// <summary>A skybox light in the first group.</summary>
            SkyboxMasked,
        }

        private readonly Lights lights;

        private Texture group0View;
        private Texture group1View;

        public TestLightMask() : this(Lights.RedAndBlueMasked)
        {
        }

        private TestLightMask(Lights lights)
        {
            this.lights = lights;
            GraphicsDeviceManager.PreferredBackBufferWidth = Size;
            GraphicsDeviceManager.PreferredBackBufferHeight = Size;
            GraphicsDeviceManager.PreferredDepthStencilFormat = PixelFormat.D24_UNorm_S8_UInt;
            GraphicsDeviceManager.PreferredGraphicsProfile = [GraphicsProfile.Level_10_0];
        }

        protected override async Task LoadContent()
        {
            await base.LoadContent();

            group0View = Texture.New2D(GraphicsDevice, Size, Size, PixelFormat.R8G8B8A8_UNorm, TextureFlags.ShaderResource | TextureFlags.RenderTarget).DisposeBy(this);
            group1View = Texture.New2D(GraphicsDevice, Size, Size, PixelFormat.R8G8B8A8_UNorm, TextureFlags.ShaderResource | TextureFlags.RenderTarget).DisposeBy(this);

            var camera = new CameraComponent();
            var compositor = GraphicsCompositorHelper.CreateDefault(enablePostEffects: false, camera: camera, clearColor: Color.Black);
            var group0Forward = (ForwardRenderer)compositor.SingleView;
            var group1Forward = new ForwardRenderer
            {
                Clear = { Color = Color.Black },
                OpaqueRenderStage = group0Forward.OpaqueRenderStage,
                TransparentRenderStage = group0Forward.TransparentRenderStage,
            };
            foreach (var stage in group0Forward.ShadowMapRenderStages)
                group1Forward.ShadowMapRenderStages.Add(stage);

            var cameraSlot = compositor.Cameras[0];
            compositor.Game = new SceneRendererCollection
            {
                new SceneCameraRenderer { Camera = cameraSlot, LightMask = LightGroupMask.Group0, Child = new RenderTextureSceneRenderer { RenderTexture = group0View, Child = group0Forward } },
                new SceneCameraRenderer { Camera = cameraSlot, LightMask = LightGroupMask.Group1, Child = new RenderTextureSceneRenderer { RenderTexture = group1View, Child = group1Forward } },
            };

            var scene = new Scene();

            var cameraEntity = new Entity { camera };
            cameraEntity.Transform.Position = new Vector3(0, 0, 3);
            scene.Entities.Add(cameraEntity);

            var material = Material.New(GraphicsDevice, new MaterialDescriptor
            {
                Attributes =
                {
                    Diffuse = new MaterialDiffuseMapFeature(new ComputeColor(Color.White)),
                    DiffuseModel = new MaterialDiffuseLambertModelFeature(),
                },
            });
            var model = new CubeProceduralModel().Generate(Services);
            model.Materials.Add(material);
            scene.Entities.Add(new Entity { new ModelComponent { Model = model } });

            switch (lights)
            {
                case Lights.RedAndBlueMasked:
                    AddLights(scene, Color.Red, LightGroupMask.Group0);
                    AddLights(scene, Color.Blue, LightGroupMask.Group1);
                    break;
                case Lights.RedAndBlueDefault:
                    AddLights(scene, Color.Red, LightGroupMask.All);
                    AddLights(scene, Color.Blue, LightGroupMask.All);
                    break;
                case Lights.SkyboxMasked:
                    var lightingScene = Content.Load<Scene>("LightingTests/LightingScene");
                    var skybox = lightingScene.Entities.First(x => x.Name == "Skybox");
                    lightingScene.Entities.Remove(skybox);
                    var skyboxLight = skybox.Get<LightComponent>();
                    skyboxLight.Enabled = true;
                    skyboxLight.LightGroups = LightGroupMask.Group0;
                    scene.Entities.Add(skybox);
                    break;
            }

            SceneSystem.SceneInstance = new SceneInstance(Services, scene);
            SceneSystem.GraphicsCompositor = compositor;
        }

        private static void AddLights(Scene scene, Color color, LightGroupMask groups)
        {
            var ambient = new LightComponent { Type = new LightAmbient { Color = new ColorRgbProvider(color) }, Intensity = 0.25f, LightGroups = groups };

            // A directional light shines down -Z, from the camera onto the cube's front face
            var directionalType = new LightDirectional { Color = new ColorRgbProvider(color) };
            directionalType.Shadow.Enabled = true;
            var directional = new LightComponent { Type = directionalType, Intensity = 0.5f, LightGroups = groups };

            scene.Entities.Add(new Entity { ambient });
            scene.Entities.Add(new Entity { directional });
        }

        private Color CenterOf(Texture view)
        {
            var pixels = view.GetData<Color>(GraphicsContext.CommandList);
            return pixels[(Size / 2) * Size + Size / 2];
        }

        private static (Color Group0, Color Group1) RenderViews(Lights lights, [CallerMemberName] string testName = null)
        {
            var game = new TestLightMask(lights);
            Color group0 = default, group1 = default;
            game.Script.AddTask(async () =>
            {
                game.ScreenShotAutomationEnabled = false;

                // Render several frames so the effects compile and the lights are drawn
                for (int i = 0; i < 10; i++)
                    await game.Script.NextFrame();

                group0 = game.CenterOf(game.group0View);
                group1 = game.CenterOf(game.group1View);
                game.Exit();
            });
            RunGameTest(game, testName);
            return (group0, group1);
        }

        [SkippableFact]
        public void EachViewIsLitByTheLightsOfItsLightMask()
        {
            var (group0, group1) = RenderViews(Lights.RedAndBlueMasked);

            Assert.True(group0.R > 64 && group0.G < 8 && group0.B < 8, $"The Group0 view should be lit red alone, got {group0}");
            Assert.True(group1.B > 64 && group1.R < 8 && group1.G < 8, $"The Group1 view should be lit blue alone, got {group1}");
        }

        [SkippableFact]
        public void LightsWithTheDefaultMaskLightEveryView()
        {
            var (group0, group1) = RenderViews(Lights.RedAndBlueDefault);

            Assert.True(group0.R > 64 && group0.B > 64 && group0.G < 8, $"The Group0 view should be lit red and blue, got {group0}");
            Assert.True(group1.R > 64 && group1.B > 64 && group1.G < 8, $"The Group1 view should be lit red and blue, got {group1}");
        }

        [SkippableFact]
        public void SkyboxLightsOnlyTheViewsOfItsLightMask()
        {
            var (group0, group1) = RenderViews(Lights.SkyboxMasked);

            Assert.True(group0.R > 16 || group0.G > 16 || group0.B > 16, $"The Group0 view should be lit by the skybox, got {group0}");
            Assert.True(group1.R < 8 && group1.G < 8 && group1.B < 8, $"The Group1 view should not be lit, got {group1}");
        }
    }
}
