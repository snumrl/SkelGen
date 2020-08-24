
uniform mat4 depthMVP;
varying vec3 normal, lightDir, halfVector;
void main(){
	normal = normalize(gl_NormalMatrix*gl_Normal);
	lightDir = normalize(gl_LightSource[0].position.xyz);
	halfVector = normalize(gl_LightSource[0].halfVector.xyz);
	gl_Position =  depthMVP * gl_Vertex;
	//gl_Position =  gl_ModelViewProjectionMatrix * gl_Vertex;
}