Shader "FerramAerospaceResearch/Debug Voxel Mesh" {
    Properties{
        _Color("Color", Color) = (1,1,1,1)
        [NoScaleOffset] _MainTex("Albedo (RGB)", 2D) = "white" {}
        _Cutoff("Alpha Cutoff", Range(0.01,1)) = 0.5
    }
    SubShader
    {
        Tags{ "Queue" = "Transparent" "IgnoreProjector" = "True" "RenderType" = "Transparent" "PreviewType" = "Plane"}
        Blend SrcAlpha OneMinusSrcAlpha
        LOD 200
        ZWrite Off
        Cull Off

        // Simple AlphaTest pass to cut off transparent areas
        Pass{
            Tags{ "Queue" = "AlphaTest" "IgnoreProjector" = "True" "RenderType" = "TransparentCutout" }
            ColorMask 0
            ZWrite On

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct v2f {
                float4 vertex : SV_POSITION;
                float2 texcoord : TEXCOORD0;
            };

            sampler2D _MainTex;
            fixed _Cutoff;

            v2f vert(appdata_img v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.texcoord = v.texcoord;
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                fixed4 col = tex2D(_MainTex, i.texcoord);
                clip(col.a - _Cutoff);
                return 0;
            }
            ENDCG
        }

        CGPROGRAM
        #pragma surface surf Lambert alpha:blend decal:blend

        sampler2D _MainTex;

        struct Input {
            float2 uv_MainTex;
        };

        fixed4 _Color;
        fixed _Cutoff;

        void surf(Input IN, inout SurfaceOutput o) {
            // Albedo comes from a texture tinted by color
            fixed4 c = tex2D(_MainTex, IN.uv_MainTex) * _Color;
            o.Albedo = c.rgb;
            o.Alpha = c.a;

            // Need to set emmision to faded color for no lighting effect
            o.Emission = c.rgb * c.a;
        }
        ENDCG
    }
    FallBack "Diffuse"
}
