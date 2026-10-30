// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

using System.Threading.Tasks;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
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
    /// Draws two views in one frame, each through a camera renderer whose render mask holds one render group, a white cube
    /// in each group and a red ambient and shadowed directional light in the first group, blue ones in the second; each
    /// view lights its cube with its own group's lights alone.
    /// </summary>
    public class TestLightRenderGroup : Graphics.Regression.GameTestBase
    {
        private const int Size = 64;

        private Texture redView;
        private Texture blueView;

        public TestLightRenderGroup()
        {
            GraphicsDeviceManager.PreferredBackBufferWidth = Size;
            GraphicsDeviceManager.PreferredBackBufferHeight = Size;
            GraphicsDeviceManager.PreferredDepthStencilFormat = PixelFormat.D24_UNorm_S8_UInt;
            GraphicsDeviceManager.PreferredGraphicsProfile = [GraphicsProfile.Level_10_0];
        }

        protected override async Task LoadContent()
        {
            await base.LoadContent();

            redView = Texture.New2D(GraphicsDevice, Size, Size, PixelFormat.R8G8B8A8_UNorm, TextureFlags.ShaderResource | TextureFlags.RenderTarget).DisposeBy(this);
            blueView = Texture.New2D(GraphicsDevice, Size, Size, PixelFormat.R8G8B8A8_UNorm, TextureFlags.ShaderResource | TextureFlags.RenderTarget).DisposeBy(this);

            var camera = new CameraComponent();
            var compositor = GraphicsCompositorHelper.CreateDefault(enablePostEffects: false, camera: camera, clearColor: Color.Black);
            var redForward = (ForwardRenderer)compositor.SingleView;
            var blueForward = new ForwardRenderer
            {
                Clear = { Color = Color.Black },
                OpaqueRenderStage = redForward.OpaqueRenderStage,
                TransparentRenderStage = redForward.TransparentRenderStage,
            };
            foreach (var stage in redForward.ShadowMapRenderStages)
                blueForward.ShadowMapRenderStages.Add(stage);

            var cameraSlot = compositor.Cameras[0];
            compositor.Game = new SceneRendererCollection
            {
                new SceneCameraRenderer { Camera = cameraSlot, RenderMask = RenderGroupMask.Group0, Child = new RenderTextureSceneRenderer { RenderTexture = redView, Child = redForward } },
                new SceneCameraRenderer { Camera = cameraSlot, RenderMask = RenderGroupMask.Group1, Child = new RenderTextureSceneRenderer { RenderTexture = blueView, Child = blueForward } },
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
            foreach (var group in new[] { RenderGroup.Group0, RenderGroup.Group1 })
            {
                var model = new CubeProceduralModel().Generate(Services);
                model.Materials.Add(material);
                scene.Entities.Add(new Entity { new ModelComponent { Model = model, RenderGroup = group } });
            }

            AddLights(scene, Color.Red, RenderGroup.Group0);
            AddLights(scene, Color.Blue, RenderGroup.Group1);

            SceneSystem.SceneInstance = new SceneInstance(Services, scene);
            SceneSystem.GraphicsCompositor = compositor;
        }

        private static void AddLights(Scene scene, Color color, RenderGroup group)
        {
            scene.Entities.Add(new Entity
            {
                new LightComponent { Type = new LightAmbient { Color = new ColorRgbProvider(color) }, Intensity = 0.25f, RenderGroup = group },
            });

            // A directional light shines down -Z, from the camera onto the cube's front face
            var directional = new LightDirectional { Color = new ColorRgbProvider(color) };
            directional.Shadow.Enabled = true;
            scene.Entities.Add(new Entity
            {
                new LightComponent { Type = directional, Intensity = 0.5f, RenderGroup = group },
            });
        }

        private Color CenterOf(Texture view)
        {
            var pixels = view.GetData<Color>(GraphicsContext.CommandList);
            return pixels[(Size / 2) * Size + Size / 2];
        }

        [SkippableFact]
        public void EachViewIsLitByTheLightsOfItsRenderGroups()
        {
            var game = new TestLightRenderGroup();
            Color red = default, blue = default;
            game.Script.AddTask(async () =>
            {
                game.ScreenShotAutomationEnabled = false;

                // Render several frames so the effects compile and the lights are drawn
                for (int i = 0; i < 10; i++)
                    await game.Script.NextFrame();

                red = game.CenterOf(game.redView);
                blue = game.CenterOf(game.blueView);
                game.Exit();
            });
            RunGameTest(game);

            Assert.True(red.R > 64 && red.G < 8 && red.B < 8, $"The Group0 view should be lit red alone, got {red}");
            Assert.True(blue.B > 64 && blue.R < 8 && blue.G < 8, $"The Group1 view should be lit blue alone, got {blue}");
        }
    }
}
