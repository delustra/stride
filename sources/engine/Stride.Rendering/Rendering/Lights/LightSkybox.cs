// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net) and Silicon Studio Corp. (https://www.siliconstudio.co.jp)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Graphics;
using Stride.Rendering.Skyboxes;

namespace Stride.Rendering.Lights
{
    /// <summary>
    /// A light coming from a skybox. The `BackgroundComponent` must be set on the entity in order to see a skybox.
    /// </summary>
    [DataContract("LightSkybox")]
    [Display("Skybox")]
    public class LightSkybox : IEnvironmentLight
    {
        /// <summary>
        /// Gets or sets the skybox.
        /// </summary>
        [DataMember(0)]
        public Skybox Skybox { get; set; }

        /// <summary>
        /// A cube map the light reflects in place of the one of its <see cref="Skybox"/>, or null.
        /// </summary>
        /// <remarks>
        /// For surroundings rendered at run time. The cube map is in the frame of the light and its mipmaps are
        /// prefiltered by roughness, as the specular cube map of a skybox is. It is not saved with the light.
        /// </remarks>
        [DataMemberIgnore]
        public Texture SpecularCubeMap { get; set; }

        [DataMemberIgnore]
        internal Quaternion Rotation;

        public bool Update(RenderLight light)
        {
            Rotation = Quaternion.RotationMatrix(light.WorldMatrix);
            return Skybox != null;
        }
    }
}
