/*
 *  phong.frag
 *  Chapter 13 - Example 01
 *
 *  Created by Graphics and Medea Lab (GML).
 *  Copyright 2010 Seoul National University. All rights reserved.
 *
 */
#version 130
varying vec3 normal, lightDir[3], halfVector[3];
varying vec3 v;
varying vec4 ShadowPos[3];

uniform mat4 LightDepthMVP[3];
uniform sampler2D LigthDepth[3];

void main() {
	gl_FragColor = vec4(0.0);
	for(int i=0;i<3;i++)
	{
		vec3 n, h, l;

		l = normalize(gl_LightSource[i].position.xyz - v);
		float NdotL, RdotE;
		vec4 color = gl_FrontMaterial.ambient*gl_LightSource[i].ambient ;//+ gl_LightModel.ambient*gl_FrontMaterial.ambient;
		n = normalize(normal);
		NdotL = max(dot(n,lightDir[i]),0.0);
		if (NdotL > 0.0) {
			color += gl_FrontMaterial.diffuse*gl_LightSource[i].diffuse*NdotL;
			h = normalize(halfVector[i]);
			RdotE = max(dot(normalize(-reflect(l,n)),normalize(-v)),0.0);
			color += gl_FrontMaterial.specular*gl_LightSource[i].specular*pow(RdotE, gl_FrontMaterial.shininess);//*(gl_FrontMaterial.shininess+1.0)/(8.0*3.14);
		}
		float bias = 0.005;
		float visiblity = 1.0;
		if(texture(LigthDepth[i],ShadowPos[i].xy).z < ShadowPos[i].z-bias)
			visiblity = 0.2;

		gl_FragColor += visiblity*color;
	}
	
}
