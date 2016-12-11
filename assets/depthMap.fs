#version 150

uniform sampler2D uTex0;

in vec2 TexCoord;

out vec4 oColor;

void main(void)
{ 
#if 1
    float d = texture(uTex0, TexCoord).r * 65536;
    oColor = vec4(d / 4000);
#else
    int d = int(texture(uTex0, TexCoord).r * 65536);
    int r = ((d>>6)&0xff)<<2;
    int g = ((d>>5)&0x3f)<<2;
    int b = (d&0xff)<<3;
    oColor = vec4(r / 255.0, g / 255.0, b / 255.0, 1);
#endif
}
