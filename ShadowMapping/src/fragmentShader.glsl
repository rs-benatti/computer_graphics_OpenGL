#version 450 core            // minimal GL version support expected from the GPU

struct shadowMapClass{
  sampler2D shadowMapVal;
};

uniform shadowMapClass shadowMaps[3];
float shadowVal = 1.0;

struct LightSource {
  vec3 position;
  vec3 color;
  float intensity;
  int isActive;
};


int numberOfLights = 3;
uniform LightSource lightSources[3];


struct Material {
  vec3 albedo;

  sampler2D normalMap;
  bool hasNormalMap;

  sampler2D textureMap;
  bool hasTextureMap;
  // TODO: textures
};

uniform Material material;

uniform vec3 camPos;

in vec3 fPositionModel;
in vec3 fPosition;
in vec3 fNormal;
in vec2 fTexCoord;
in vec4 posLightSpace[3];

out vec4 colorOut; // shader output: the color response attached to this fragment

float pi = 3.1415927;

// TODO: shadows
float shadowCalculation(vec4 fragPosLightSpace, int i){
  float bias = 0.04;
  vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
  projCoords = projCoords * 0.5 + 0.5;
  float closestDepth =  texture(shadowMaps[i].shadowMapVal, projCoords.xy).r;
  float currentDepth = projCoords.z - bias;
  if (closestDepth < currentDepth){
    shadowVal -=0.3;
  }
  return shadowVal;
}


void main() {
  vec3 n = normalize(fNormal);
  if (material.hasNormalMap){
    n = texture(material.normalMap, fTexCoord).rgb;
    // transform normal vector to range [-1,1]
    n = normalize(n * 2.0 - 1.0); 
  } 

  vec3 wo = normalize(camPos - fPosition); // unit vector pointing to the camera

  vec3 radiance = vec3(0, 0, 0);
  for(int i=0; i<numberOfLights; ++i) {
    LightSource a_light = lightSources[i];
    vec4 fragPosLightSpace = posLightSpace[i];

    shadowVal = shadowCalculation(fragPosLightSpace, i);
    if(a_light.isActive == 1) { // consider active lights only
      vec3 wi = normalize(a_light.position - fPosition); // unit vector pointing to the light
      vec3 Li = a_light.color*a_light.intensity;
      vec3 albedo = material.albedo;
      if (material.hasTextureMap){
        albedo = texture(material.textureMap, fTexCoord).rgb;
      }

      radiance += Li*albedo*max(dot(n, wi), 0);
    }
    
  }

  colorOut = shadowVal*vec4(radiance, 1.0); // build an RGBA value from an RGB one
}
