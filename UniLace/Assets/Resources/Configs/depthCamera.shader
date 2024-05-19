// Reference:
// ML ImageSynthesis: https://bitbucket.org/Unity-Technologies/ml-imagesynthesis/src/master/Assets/ImageSynthesis/Shaders/UberReplacement.shader
// datatypes: https://docs.unity3d.com/Manual/SL-DataTypesAndPrecision.html
// shader fundamentals: https://docs.unity3d.com/Manual/SL-VertexFragmentShaderExamples.html
// UnityCG.cginc file: https://github.com/TwoTailsGames/Unity-Built-in-Shaders/blob/master/CGIncludes/UnityCG.cginc#L754
// Built-in shader variables: https://docs.unity3d.com/Manual/SL-UnityShaderVariables.html
// Built-in macros: https://docs.unity3d.com/Manual/SL-BuiltinMacros.html
// Shader semantics: https://docs.unity3d.com/Manual/SL-ShaderSemantics.html
// Generate simple nosie: https://docs.unity3d.com/Packages/com.unity.shadergraph@16.0/manual/Simple-Noise-Node.html

Shader "Hidden/DepthCamera" {
Properties {
	_MainTex ("", 2D) = "white" {}
	_NoiseVariance ("Gaussian Noise Variance", Range(0.0, 1.0)) = 0.0
}

SubShader {
	CGINCLUDE


	float _noiseVariance=0;
	float getNoiseVariance(float depth)
	{
		return _noiseVariance/2*depth;
	}

	float2 GenerateNoises(float2 uv)
	{
		float2 noise = float2(0, 0);
		noise.x = frac(sin(dot(uv, float2(12.9898, 78.233) + _Time.y * 1000)) * 43758.5453);
		noise.y = frac(sin(dot(uv, float2(12.9898, 78.233) * 2.0 + _Time.y * 1000)) * 43758.5453);
		return noise;
	}

	float GenerateGaussianNoise(float2 uv, float depth)
	{
		// Box-Muller method
		float2 noise = GenerateNoises(uv);
		return sqrt(-2 * log(noise.x)) * cos(2 * 3.14159265359 * noise.y)*depth;
		// random sign
		// float sign = noise.x > 0.5 ? 1 : -1;
		// return sign*sqrt(-2 * log(noise.x)) * cos(2 * 3.14159265359 * noise.y)*depth;
	}

	ENDCG

	// Support for different RenderTypes
		Tags { "RenderType"="Opaque" }
		Pass {
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			#include "UnityCG.cginc"
			struct v2f {
				float4 pos : SV_POSITION;
				float2 uv : TEXCOORD0;
				float depth : DEPTH;
			};
			// float _noiseVariance=0.02;

			v2f vert( appdata_base v ) {
				v2f o;
				UNITY_SETUP_INSTANCE_ID(v);
				o.pos = UnityObjectToClipPos(v.vertex);
				COMPUTE_EYEDEPTH(o.depth);
				o.uv = v.texcoord;
				// o.pos.y = 1 - o.pos.y; // flip the image
				return o;
			}

			float4 frag(v2f i) : SV_Target {
				float range = getNoiseVariance(i.depth);
				float noise = GenerateGaussianNoise(i.pos.xy, range);
				// restrict the noise to be between -2% and 2% of the depth
				noise = clamp(noise, -range, range);
				// add the noise to the depth
				i.depth += noise;
				return float4(i.depth/10,0,0,0);
			}
			ENDCG
		}
	}

Fallback Off
}
