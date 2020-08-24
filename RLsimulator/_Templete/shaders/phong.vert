/*
 *  phong.vert
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
	normal = normalize(gl_NormalMatrix*gl_Normal);
	lightDir[0] = normalize(gl_LightSource[0].position.xyz);
	halfVector[0] = normalize(gl_LightSource[0].halfVector.xyz);
	lightDir[1] = normalize(gl_LightSource[1].position.xyz);
	halfVector[1] = normalize(gl_LightSource[1].halfVector.xyz);
	lightDir[2] = normalize(gl_LightSource[2].position.xyz);
	halfVector[2] = normalize(gl_LightSource[2].halfVector.xyz);
	v = (gl_ModelViewMatrix * gl_Vertex).xyz;

	ShadowPos[0] = LightDepthMVP[0]*gl_Vertex;
	ShadowPos[1] = LightDepthMVP[1]*gl_Vertex;
	ShadowPos[2] = LightDepthMVP[2]*gl_Vertex;

	ShadowPos[0] /= ShadowPos[0].w;
	ShadowPos[1] /= ShadowPos[1].w;
	ShadowPos[2] /= ShadowPos[2].w;

	gl_Position = gl_ModelViewProjectionMatrix*gl_Vertex;
}