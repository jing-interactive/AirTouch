#version 120

uniform sampler2D image;

void main(void)
{ 
    // sample from the texture  
    float r = texture2D(image, gl_TexCoord[0].st).r;
    r = r * 65536 / 8000;
    gl_FragColor = vec4(1, 1, 1, 1);
}
