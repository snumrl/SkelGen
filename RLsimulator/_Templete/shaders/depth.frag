varying vec3 normal, lightDir, halfVector;
uniform mat4 depthMVP;
//gl_FragCoord.z;
void main(){

	gl_FragColor = gl_FragCoord.z;
}
