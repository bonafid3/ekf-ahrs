#version 400

// these are attributes, pointers to buffers
in vec3 qt_vertex;
in vec2 qt_textureCoords;

// input stuff from Qt
uniform mat4 qt_projectionMatrix;
uniform mat4 qt_modelMatrix;
uniform mat4 qt_viewMatrix;

uniform int qt_pointScale;

// this will contain the color
//out vec3 color;
out vec2 pass_textureCoords;

void main()
{
    vec3 v = vec3(qt_vertex);
    if(qt_pointScale > 0) {
        v *= qt_pointScale / 100.0;
    }

    pass_textureCoords = qt_textureCoords;
    vec4 worldPosition = qt_viewMatrix * qt_modelMatrix * vec4(v, 1);
    gl_Position = qt_projectionMatrix * worldPosition;
}
