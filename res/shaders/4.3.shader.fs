#version 430 core
out vec4 FragColor;
in vec3 ourColor;

float near = 0.015;
float far = 0.1;

float LinearizeDepth(float depth) 
{
    float z = depth * 2.0 - 1.0; // back to NDC
	return (2.0 * near * far) / (far + near - z * (far - near));
}

void main()
{
	float depth = LinearizeDepth(gl_FragCoord.z) / far; // divide by far for better depth differentieation in close areas
    //FragColor = vec4(ourColor, 1.0f);
	FragColor = vec4(ourColor, 1.0f)*vec4(vec3(1.0f-depth),1.0);
}