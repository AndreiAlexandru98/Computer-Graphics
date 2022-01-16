#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <fstream>
#include <vector>
#include <glm/gtx/string_cast.hpp>
#include <CanvasPoint.h>
#include <Colour.h>
#include <cstring>
#include <map>
#include <iostream>
#include <string>
#include "SDL.h"
#include <math.h>
#include <cmath>
#include <RayTriangleIntersection.h>
#include <list>
#include <glm/ext.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <random>

//Global definitions
#define WIDTH  900
#define HEIGHT 900

#define ROTATION 0.1
#define TRANSLATION  0.5
typedef enum { RIGHT, LEFT, FORWARD, BACKWARD, UP, DOWN } Direction;
//Raytracing definitions
#define NUM_LIGHT_RAYS 1
#define GLASS_INDEX_OF_REFRACTION 1.512f
#define GLASS_MAGIC_NUMBER 1.15f
#define SOBEL_THRESHOLD 0.5
//Rasterising definitions
#define EDGE_THRESHOLD_MIN 0.0312
#define EDGE_THRESHOLD_MAX 0.125
#define SUBPIXEL_QUALITY 0.75

//Main functions
void draw();
void update(SDL_Event event);
void handleEvent(SDL_Event event);
void drawWireframe(CanvasTriangle triangle,Colour colour);
//Raytracing functions
void rayTracing(std::vector<ModelTriangle> triangles);
void GenAreaLight();
glm::vec3 traceRayFromCamera(float x, float y,std::vector<ModelTriangle> modelTriangles);
RayTriangleIntersection getClosestIntersection(std::vector<ModelTriangle> triangles,  glm::vec3 rayDirection,glm::vec3 start );
glm::vec3 intersection(ModelTriangle triangle, glm::vec3 rayDirection, glm::vec3 start);
Colour directLight(RayTriangleIntersection input, std::vector<ModelTriangle> modelTriangles, glm::vec3 direction);
glm::vec3 getReflectedDirection(const glm::vec3& incident, const glm::vec3& normal);
//Supersampling functions
void antiAliasing(std::vector<ModelTriangle> triangles) ;
void computePixelsIntensity();
float sobelOperator(int x, int y);
glm::vec3 supersamplingAA(int x, int y, std::vector<ModelTriangle> modelTriangles);
//Rasteriser functions
void initDepthBuffer();
std::vector<CanvasTriangle> convertModelToCanvas(std::vector<ModelTriangle> triangles);
CanvasPoint convertModelVertexToCanvasPoint(glm::vec3 modelVertex, glm::vec3 normal);
void rasterisation(CanvasTriangle triangle,Colour colour, bool texture);
void drawLine(CanvasPoint from, CanvasPoint to, Colour colour);
std::vector<CanvasPoint> interpolate(CanvasPoint from, CanvasPoint to);
CanvasPoint calculateExtra(CanvasTriangle triangle);
void fillTriangle(CanvasPoint v1, CanvasPoint v2, CanvasPoint v3, Colour colour);
//FXAA functions
void applyAntiAliasing();
float rgb2luma(glm::vec3 rgb);
//Utilities
std::map<std::string, Colour> loadMaterials(std::string img);
void loadTriangles (std::string imgName, std::map<std::string, Colour> m);
void loadSphere(std::string imgName, std::map<std::string, Colour> materials);
std::vector<ModelTriangle> loadLogo (std::string imgName);
std::vector<ModelTriangle> calculateNormals(std::vector<ModelTriangle> triangles);
glm::vec3 calculateVertexNormal(std::vector<ModelTriangle> modelTriangles, glm::vec3 modelVertex);
bool compareVectors(glm::vec3 v1, glm::vec3 v2);
std::vector<std::vector<uint32_t>> loadPPM(std::string imgName);
void saveImage();
void putPixels();
//Camera funtions
glm::vec3 GetAxis(Direction dir);
void UpdateYRotationMatrix();
void UpdateXRotationMatrix();
void lookAt();
float getDistance(glm::vec3 p1, glm::vec3 p2);
//Global variables
int mode = 1;
int scene = 0;
DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
glm::vec3 screen[WIDTH][HEIGHT];
float focalLength = WIDTH/2 + 700;

glm::vec3 cameraPos(0, 2, 10.001);
glm::mat3 RotationY = glm::mat3(0, 0, 0, 0, 0, 0, 0, 0, 0);
glm::mat3 RotationX = glm::mat3(0, 0, 0, 0, 0, 0, 0, 0, 0);
glm::mat3 orientationMatrix;
float yaw = 0;
float xaw = 0;
bool lock = 0;

glm::vec3 lightPos(0.25, 5,-3);
float directLightPower = 30.f;
float indirectLightPower = 0.3f;

bool animation = 0;

std::vector<ModelTriangle> modelTriangles;
std::map<std::string, Colour> materials;
std::vector<ModelTriangle> logoTriangles;
//Raytracing variables
std::vector<glm::vec3> lightPositionArr(NUM_LIGHT_RAYS);

glm::vec3 INTENSITY_WEIGHTS(0.2989, 0.5870, 0.1140);
float screenPixelsIntensity[HEIGHT][WIDTH];
std::list<std::pair<int, int>>* aliasedEdges = new std::list<std::pair<int, int>>();
bool isMirror = 0;
//Rasterising variables
float depthBuffer[WIDTH][HEIGHT];
float quality[12] = {1, 1, 1, 1, 1, 1.5, 2.0, 2.0, 2.0, 2.0, 4.0, 8.0};
bool isTexture = 0;
std::vector<std::vector<uint32_t>> ppmtex;
bool isLogo = 0;
float textureHeight;
float textureWidth;
////Start program
//////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[]) {

  RotationY[1][1] = 1;
  RotationX[0][0] = 1;
  UpdateYRotationMatrix();
  UpdateXRotationMatrix();

  materials = loadMaterials("cornell-box/cornell-box.mtl");
  loadTriangles("cornell-box/cornell-box.obj",materials);
  loadSphere("lowres-sphere.obj",materials);
  modelTriangles = calculateNormals(modelTriangles);
  isLogo = 1;
  logoTriangles = loadLogo("hackspace-logo/logo.obj");
  logoTriangles = calculateNormals(logoTriangles);
  ppmtex = loadPPM("hackspace-logo/texture.ppm");

  SDL_Event event;
  draw();
  window.renderFrame();
  while(true)
  {
    if(animation){
      cameraPos += GetAxis(Direction::BACKWARD);
      cameraPos += GetAxis(Direction::LEFT);
      draw();
      window.renderFrame();
    }
    // We MUST poll for events - otherwise the window will freeze !
    if(window.pollForInputEvents(&event)){
      if(event.type == SDL_KEYDOWN){
      update(event);
      draw();

    // Need to render the frame at the end, or nothing actually gets shown on the screen !
    window.renderFrame();}}
  }
  return 0;
}

//SDL draw funtion
void draw() {
  window.clearPixels();
  //Iterate though all triangles
  if(mode != 3){
    initDepthBuffer();
    std::vector<CanvasTriangle> canvasTriangles = convertModelToCanvas(modelTriangles);
    std::vector<CanvasTriangle> canvasLogo = convertModelToCanvas(logoTriangles);
    for(int i = 0; i < canvasTriangles.size(); i++){
      if(mode == 1)
        rasterisation(canvasTriangles[i], canvasTriangles[i].colour,0);
      if(mode == 2)
        drawWireframe(canvasTriangles[i], canvasTriangles[i].colour);
    }
    if(mode == 1)
      for(int i=0; i<canvasLogo.size(); i++)
        rasterisation(canvasLogo[i], Colour(255,255,255),1);
  }
  else
    rayTracing(modelTriangles);
  if(mode != 3) {
//  applyAntiAliasing();
  }
  else{
    //antiAliasing(modelTriangles);
  }
  putPixels();
  saveImage();
  std::cout<<"Done scene"<<std::endl;
}
////Utilities
//////////////////////////////////////////////////////////////////////////////////////////////////
//Load materials
std::map<std::string, Colour> loadMaterials(std::string img){

 std::ifstream file(img);
 std::string name;
 std::string values;
 std::string empty;

 std::string* nameVec;
 std::string* valuesVec;

 float red;
 float green;
 float blue;
 Colour colour;
 std::map<std::string, Colour> materials;

 while(file){
   std::getline(file, name);
   std::getline(file, values);
   std::getline(file, empty);

   nameVec = split(name, ' ');
   valuesVec = split(values, ' ');
   red = 255 * stof(valuesVec[1]);
   green = 255 * stof(valuesVec[2]);
   blue = 255 * stof(valuesVec[3]);

   colour = Colour(int(red),int(green),int(blue));

   materials[nameVec[1]] = colour;
 }
 return materials;
}

//Load triangles
void loadTriangles(std::string imgName, std::map<std::string, Colour> materials){
 std::ifstream file(imgName);
 std::string line;
 std::string* lineVal;

 std::vector<glm::vec3> vertices;
 ModelTriangle triangle;
 glm::vec3 vertex;
 Colour colour;

 while(file) {
    std::getline(file, line);
    lineVal = split(line, ' ');

    if(lineVal[0].compare("v") == 0) {
      vertex = glm::vec3(-stof(lineVal[1]), stof(lineVal[2]), stof(lineVal[3]) );
      vertices.push_back(vertex);
    }

    if(lineVal[0].compare("usemtl") == 0){
      colour = materials[lineVal[1]];
    }

    if(lineVal[0].compare("f") == 0){
      std::string v1 = lineVal[1].substr(0, lineVal[1].size()-1);
      std::string v2 = lineVal[2].substr(0, lineVal[2].size()-1);
      std::string v3 = lineVal[3].substr(0, lineVal[3].size()-1);

      triangle = ModelTriangle(vertices[std::stoi(v1) - 1], vertices[std::stoi(v2) - 1], vertices[std::stoi(v3) - 1], colour);
      modelTriangles.push_back(triangle);
    }
 }
}

//Load sphere
void loadSphere(std::string imgName, std::map<std::string, Colour> materials){
 std::ifstream file(imgName);
 std::string line;
 std::string* lineVal;

 std::vector<glm::vec3> vertices;
 ModelTriangle triangle;
 glm::vec3 vertex;
 Colour colour;

 while(file) {
    std::getline(file, line);
    lineVal = split(line, ' ');

    if(lineVal[0].compare("v") == 0) {
      vertex = glm::vec3(-stof(lineVal[1]) + 1, stof(lineVal[2]) + 3.8, stof(lineVal[3]) - 5);
      vertices.push_back(vertex);
    }

    if(lineVal[0].compare("usemtl") == 0){
      colour = materials[lineVal[1]];
    }

    if(lineVal[0].compare("f") == 0){
      std::string v1 = lineVal[1].substr(0, lineVal[1].size()-1);
      std::string v2 = lineVal[2].substr(0, lineVal[2].size()-1);
      std::string v3 = lineVal[3].substr(0, lineVal[3].size()-1);

      triangle = ModelTriangle(vertices[std::stoi(v1) - 1], vertices[std::stoi(v2) - 1], vertices[std::stoi(v3) - 1], colour);
      modelTriangles.push_back(triangle);
    }
 }
}

// Load logo
std::vector<ModelTriangle> loadLogo (std::string imgName){

  std::ifstream file(imgName);
  std::string line;
  std::string* lineVal;

  std::vector<glm::vec3> vertices;
  std::vector<TexturePoint> vts;
  std::vector<ModelTriangle> triangles;
  ModelTriangle triangleModel;
  CanvasTriangle triangleCanvas;
  TexturePoint vt;
  glm::vec3 vertex;
  Colour colour;

   while(file) {
      std::getline(file, line);
      lineVal = split(line, ' ');

     if(lineVal[0].compare("v") == 0) {
        vertex = glm::vec3(stof(lineVal[1])/200 - 2.5, stof(lineVal[2])/200 + 2, stof(lineVal[3])/200 - 5);
        vertices.push_back(vertex);
      }

     if(lineVal[0].compare("vt") == 0) {
       vt = TexturePoint(round(stof(lineVal[1])*299), round(stof(lineVal[2])*299) );
       vts.push_back(vt);
     }

     if(lineVal[0].compare("f") == 0) {

       std::string v1 = lineVal[1];
       std::string v2 = lineVal[2];
       std::string v3 = lineVal[3];

       std::string* v1_s = split(v1, '/');
       std::string* v2_s = split(v2, '/');
       std::string* v3_s = split(v3, '/');

       triangleModel = ModelTriangle(vertices[std::stoi(v1_s[0]) - 1], vertices[std::stoi(v2_s[0]) - 1], vertices[std::stoi(v3_s[0]) - 1], Colour(255, 255, 255));
       triangleModel.texturePoint[0] = vts[std::stoi(v1_s[1]) - 1];
       triangleModel.texturePoint[1] = vts[std::stoi(v2_s[1]) - 1];
       triangleModel.texturePoint[2] = vts[std::stoi(v3_s[1]) - 1];
       triangles.push_back(triangleModel);
     }
   }
   return triangles;
}

//Calculate model triangles normals
std::vector<ModelTriangle> calculateNormals(std::vector<ModelTriangle> triangles){
  for(int i = 0; i  < triangles.size(); i++){
      triangles[i].normals[0]= calculateVertexNormal(triangles, triangles[i].vertices[0]);
      triangles[i].normals[1]= calculateVertexNormal(triangles, triangles[i].vertices[1]);
      triangles[i].normals[2]= calculateVertexNormal(triangles, triangles[i].vertices[2]);

      glm::vec3 vec0 = triangles[i].vertices[0];
      glm::vec3 vec1 = triangles[i].vertices[1];
      glm::vec3 vec2 = triangles[i].vertices[2];
      glm::vec3 e1 = vec1 - vec0;
      glm::vec3 e2 = vec2 - vec0;
      glm::vec3 normal;
      if(isLogo)
      normal =glm::normalize(glm::cross(e1,e2));
      else
      normal =glm::normalize(glm::cross(e2,e1));
      triangles[i].triangleNormal = normal;
  }
  return triangles;
}

//Calculate vertex normal
glm::vec3 calculateVertexNormal(std::vector<ModelTriangle> modelTriangles, glm::vec3 modelVertex){
  std::vector<glm::vec3> normals;
  float xTotal,yTotal,zTotal;
  for(int i = 0; i  < modelTriangles.size(); i++){
    if(compareVectors(modelVertex,modelTriangles[i].vertices[0]) || compareVectors(modelVertex,modelTriangles[i].vertices[1]) || compareVectors(modelVertex,modelTriangles[i].vertices[2])){
      glm::vec3 vec0 = modelTriangles[i].vertices[0];
      glm::vec3 vec1 = modelTriangles[i].vertices[1];
      glm::vec3 vec2 = modelTriangles[i].vertices[2];
      glm::vec3 e1 = vec1 - vec0;
      glm::vec3 e2 = vec2 - vec0;
      glm::vec3 normal;
      if(isLogo)
      normal =glm::normalize(glm::cross(e1,e2));
      else
      normal =glm::normalize(glm::cross(e2,e1));
      normals.push_back(normal);
    }
  }

  glm::vec3 normal = glm::vec3(0,0,0);
  for(int i = 0; i  < normals.size(); i++){
    normal = normal + normals[i];
  }

  return glm::normalize(normal);
}

//Function for comparing vectors
bool compareVectors(glm::vec3 v1, glm::vec3 v2) {
  return(v1.x == v2.x && v1.y == v2.y && v1.z == v2.z);
}

// Load texture
std::vector<std::vector<uint32_t>> loadPPM(std::string imgName){

  std::ifstream file(imgName);

  std::string version;
  std::getline (file,version);
  std::string description;
  std::getline (file,description);
  std::string sizes;
  std::getline (file,sizes);
  std::string max;
  std::getline (file,max);
  int maxColourValue = std::stoi(max);

  int width, height;
  int split;

  split = sizes.find(" ");

  width = stoi(sizes.substr(0, split));
  height = stoi(sizes.substr(split + 1, sizes.length() - (split+1)));

  textureWidth = width;
  textureHeight = height;

  std::vector<std::vector<uint32_t>> image(height, std::vector<uint32_t>(width));
  std::string line;

  for(int i = 0; i < height; i++) {
    for(int j = 0; j < width; j++) {
      float red = file.get();
      float green = file.get();
      float blue = file.get();
      uint32_t colour = (255<<24) + (int(red)<<16) + (int(green)<<8) + int(blue);

      image[i][j] = colour;
    }
  }
  return image;
}

//Save to ppm
void saveImage(){
  std::ofstream outfile ("file"+std::to_string(scene)+".ppm");
  scene++;
  outfile<<"P6\n";
  outfile<<WIDTH<< " "<<HEIGHT<< "\n";
  outfile<<"255\n";
  for(int i = 0; i< HEIGHT; i++){
    for(int j = 0; j< WIDTH; j++){
      uint32_t colour = window.getPixelColour(j,i);
      float red = colour>>16 & 255;
      float green = colour>>8 & 255;
      float blue = int(colour & 255);
      outfile<<(unsigned char)red;
      outfile<<(unsigned char)green;
      outfile<<(unsigned char)blue;
    }
  }
  outfile.close();
}

//Put pixels on screen
void putPixels() {
  for(int i =0; i< WIDTH; i++)
      for(int j =0; j< HEIGHT; j++){
          uint32_t c = (255<<24) + (int(screen[i][j].x)<<16) + (int(screen[i][j].y)<<8) + int(screen[i][j].z);
          window.setPixelColour(i,j,c);
      }
}
////Camera
//////////////////////////////////////////////////////////////////////////////////////////////////
// Function for performing animation (shifting artifacts or moving the camera)
void update(SDL_Event event) {
  //If a key is pressed
  if(event.type == SDL_KEYDOWN) {
    std::cout<<"Key"<<std::endl;
    if(event.key.keysym.sym == SDLK_LEFT) {
		// Move camera to the left
		  cameraPos += GetAxis(Direction::LEFT);
	  }
    else if(event.key.keysym.sym == SDLK_RIGHT) {
		  // Move camera to the right
		  cameraPos += GetAxis(Direction::RIGHT);
	  }
    else if(event.key.keysym.sym == SDLK_UP) {
	  	// Move camera up
      cameraPos += GetAxis(Direction::UP);
  	}
    else if(event.key.keysym.sym == SDLK_DOWN) {
	  	// Move camera down
      cameraPos += GetAxis(Direction::DOWN);
  	}
    else if(event.key.keysym.sym == SDLK_w)	{
    	// Move camera forward
    	cameraPos += GetAxis(Direction::FORWARD);
    }
    else if(event.key.keysym.sym == SDLK_s) {
      // Move camera backwards
      cameraPos += GetAxis(Direction::BACKWARD);
    }
    else if(event.key.keysym.sym == SDLK_q) {
      //Tilt camera upwards
      xaw -= ROTATION;
      UpdateXRotationMatrix();
    }
    else if(event.key.keysym.sym == SDLK_e) {
      // Tilt camera downwards
      xaw += ROTATION;
      UpdateXRotationMatrix();
    }
   else if(event.key.keysym.sym == SDLK_a) {
       // Rotate camera to the left
      yaw -= ROTATION;
      UpdateYRotationMatrix();
    }
    else if(event.key.keysym.sym == SDLK_d) {
      // Move camera backwards
      yaw += ROTATION;
      UpdateYRotationMatrix();
    }
    else if(event.key.keysym.sym == SDLK_1) {
      mode = 1;
    }
    else if(event.key.keysym.sym == SDLK_2) {
      mode = 2;
    }
    else if(event.key.keysym.sym == SDLK_3) {
      mode = 3;
    }
    else if(event.key.keysym.sym == SDLK_l) {
      lock = !lock;
    }
    else if(event.key.keysym.sym == SDLK_f) {
      lock = 1;
      animation = !animation;
    }
  }
}

//Translate camera position
glm::vec3 GetAxis(Direction dir) {
	if (dir == Direction::UP) {
		return TRANSLATION * -glm::vec3(0, 1, 0);
	}
	if (dir == Direction::DOWN) {
		return TRANSLATION * glm::vec3(0, 1, 0);
	}
	if (dir == Direction::RIGHT) {
		return TRANSLATION * glm::vec3(1, 0, 0);
	}
	if (dir == Direction::LEFT) {
		return TRANSLATION * -glm::vec3(1, 0, 0);
	}
	if (dir == Direction::FORWARD) {
		return TRANSLATION * glm::vec3(0, 0, 1);
	}
	if (dir == Direction::BACKWARD) {
		return TRANSLATION * -glm::vec3(0, 0, 1);
	}
  return glm::vec3(0, 0, 0);
}

//Update RotationY
void UpdateYRotationMatrix() {
	RotationY[0][0] = cos(yaw);
	RotationY[0][2] = sin(yaw);
	RotationY[2][0] = -sin(yaw);
	RotationY[2][2] = cos(yaw);
}

//Update RotationX
void UpdateXRotationMatrix() {
	RotationX[1][1] = cos(xaw);
	RotationX[1][2] = -sin(xaw);
	RotationX[2][1] = sin(xaw);
	RotationX[2][2] = cos(xaw);
}

//Look at function
void lookAt(){
  glm::vec3 lookAtPoint = glm::vec3(0,2,-3);
  glm::vec3 cameraToLookAt = glm::normalize(cameraPos - lookAtPoint);
  glm::vec3 right = orientationMatrix[0];
  glm::vec3 up = orientationMatrix[1];
  glm::vec3 forward = orientationMatrix[2];
  orientationMatrix[2] = cameraToLookAt;
  orientationMatrix[0] = glm::cross(glm::vec3(0,1,0), cameraToLookAt);
  orientationMatrix[1] = glm::cross(orientationMatrix[2],orientationMatrix[0]);
}
////Rasteriser
//////////////////////////////////////////////////////////////////////////////////////////////////
//Initialize depth buffer
void initDepthBuffer() {

  for (int x = 0; x < WIDTH; ++x)
   	for (int y = 0; y < HEIGHT; ++y){
			 depthBuffer[x][y] = std::numeric_limits<float>::infinity();
       screen[x][y] = glm::vec3(0,0,0);}
}

//Convert triangles from 3D to 2D
std::vector<CanvasTriangle> convertModelToCanvas(std::vector<ModelTriangle> triangles) {
  std::vector<CanvasTriangle> canvasTriangles;
  for(int i = 0; i  < triangles.size(); i++) {

       CanvasPoint v1 = convertModelVertexToCanvasPoint(triangles[i].vertices[0],triangles[i].normals[0]);
       CanvasPoint v2 = convertModelVertexToCanvasPoint(triangles[i].vertices[1],triangles[i].normals[1]);
       CanvasPoint v3 = convertModelVertexToCanvasPoint(triangles[i].vertices[2],triangles[i].normals[2]);

       v1.texturePoint = triangles[i].texturePoint[0];
       v2.texturePoint = triangles[i].texturePoint[1];
       v3.texturePoint = triangles[i].texturePoint[2];

       // dividing by z for persepctive correctness
       v1.texturePoint.x /= triangles[i].vertices[0].z;
       v1.texturePoint.y /= triangles[i].vertices[0].z;
       v2.texturePoint.x /= triangles[i].vertices[1].z;
       v2.texturePoint.y /= triangles[i].vertices[1].z;
       v3.texturePoint.x /= triangles[i].vertices[2].z;
       v3.texturePoint.y /= triangles[i].vertices[2].z;

orientationMatrix = RotationX * RotationY;
       CanvasTriangle triangle = CanvasTriangle(v1,v2,v3,triangles[i].colour);
       //Back-face culling
       if(glm::dot((triangles[i].vertices[0] - cameraPos)*orientationMatrix,triangles[i].triangleNormal) < 0)
       //Far place clipping
       if(getDistance(cameraPos,triangles[i].vertices[0]) < 30 && getDistance(cameraPos,triangles[i].vertices[1]) < 30 && getDistance(cameraPos,triangles[i].vertices[2]) < 30)
       //Near place clipping
       if(getDistance(cameraPos,triangles[i].vertices[0]) > 8 && getDistance(cameraPos,triangles[i].vertices[1]) > 8 && getDistance(cameraPos,triangles[i].vertices[2]) > 8)
       canvasTriangles.push_back(triangle);
  }
  return canvasTriangles;
}

//Convert point from 3D to 2D
CanvasPoint convertModelVertexToCanvasPoint(glm::vec3 modelVertex, glm::vec3 normal) {

    glm::vec3 r = lightPos - modelVertex ;
    float distance = glm::length(r);
    float div = 4 * 3.14 * distance * distance;


    float max = fmax(glm::dot(r,normal),0);
    float brightness = (max*directLightPower/div ) + indirectLightPower ;

    orientationMatrix = RotationX * RotationY;
    glm::vec3 original = modelVertex;

    original.z = 1.0f/original.z;

    if(lock)
       lookAt();
    modelVertex = (modelVertex - cameraPos) * orientationMatrix;

    float canvasX = (modelVertex.x * focalLength) / modelVertex.z + WIDTH/2;
    float canvasY = (modelVertex.y * focalLength) / modelVertex.z + HEIGHT/2;
    float canvasZ = 1.0f/modelVertex.z;

    CanvasPoint point = CanvasPoint(canvasX, canvasY,canvasZ, original);
    if(brightness > 1)
      brightness = 1;
    else if(brightness < 0.2)
      brightness = 0.2;
    point.brightness = brightness;
    return point;
}

//RASTERISATION funtion
void rasterisation(CanvasTriangle triangle, Colour colour,bool texture){
  isTexture = texture;
  //Sort vertices
  if(triangle.vertices[0].y > triangle.vertices[1].y)
    std::swap(triangle.vertices[0], triangle.vertices[1]);
  if(triangle.vertices[0].y > triangle.vertices[2].y)
    std::swap(triangle.vertices[0], triangle.vertices[2]);
  if(triangle.vertices[1].y > triangle.vertices[2].y)
    std::swap(triangle.vertices[1], triangle.vertices[2]);

  //Draw edges
  drawLine(triangle.vertices[0], triangle.vertices[1], colour);
  drawLine(triangle.vertices[1], triangle.vertices[2], colour);
  drawLine(triangle.vertices[2], triangle.vertices[0], colour);


  //Split triangle
  CanvasPoint extra = calculateExtra(triangle);
  drawLine(triangle.vertices[1], extra, colour);

  //Fill top triangle
  fillTriangle(triangle.vertices[0], extra, triangle.vertices[1],colour);
  //Fill bottom triangle
  fillTriangle(triangle.vertices[2], extra, triangle.vertices[1],colour);
}

//Draw line
void drawLine(CanvasPoint from,CanvasPoint to, Colour colour){
  std::vector<CanvasPoint> canvasPoints = interpolate(from,to);

  for (int i=0; i<canvasPoints.size(); i++) {
       float x = canvasPoints[i].x;
       float y = canvasPoints[i].y;
       float z = canvasPoints[i].depth;

       float xt = canvasPoints[i].texturePoint.x;
       float yt = canvasPoints[i].texturePoint.y;

       float zInv = canvasPoints[i].pos3d.z;

       // perspective correctness
       // dividing by 1/z to obtain true coordinates
       xt /= zInv;
       yt /= zInv;

       // CASE: Pixel is on screen
       if(round(x) >= 0 && round(y) >= 0 && round(x) < WIDTH && round(y) < HEIGHT) {
          //CASE: Pixel is the closest to the camera
          if(z <= depthBuffer[int(round(x))][int(round(y))]) {
             float brightness = canvasPoints[i].brightness;
             //Normalize brightness
             if(brightness > 1)
                brightness = 1;
             else if(brightness < 0.2)
                brightness = 0.2;
             //Update buffer
             depthBuffer[int(round(x))][int(round(y))] = z;
             // Compute colour
             float red,green,blue;
             //CASE: Pixel has texture
             if(isTexture && xt >= 0 && yt >= 0 && xt < textureWidth && yt < textureHeight) {
                 if(xt > 299) xt = 299;
                 if(yt > 299) yt = 299;

                 uint32_t packetColour = ppmtex[round(xt)][round(yt)];
                 red = packetColour>>16 & 255;
                 green = packetColour>>8 & 255;
                 blue = int(packetColour & 255);

                 red = red*brightness;
                 green = green*brightness;
                 blue = blue*brightness;
             }
             // CASE: Pixel doesn't have texture
             else {
                red = colour.red*brightness;
                green = colour.green*brightness;
                blue = colour.blue*brightness;
             }

             glm::vec3 col = glm::vec3(red,green,blue);
             screen[int(round(x))][int(round(y))] = col;
          }
        }
   }
}

//Interpolation funtion
std::vector<CanvasPoint> interpolate(CanvasPoint from, CanvasPoint to){
  std::vector<CanvasPoint> canvasPoints;
  float diffX = to.x - from.x;
  float diffY = to.y - from.y;
  float diffZ = to.depth - from.depth;

  TexturePoint vtFrom = from.texturePoint;
  TexturePoint vtTo = to.texturePoint;

  float diffTextX = vtTo.x - vtFrom.x;
  float diffTextY = vtTo.y - vtFrom.y;

  float diff3DX = to.pos3d.x - from.pos3d.x;
  float diff3DY = to.pos3d.y - from.pos3d.y;
  float diff3DZ = to.pos3d.z - from.pos3d.z;

  float diffB = to.brightness - from.brightness;

  float numberOfSteps = std::max({abs(diffX), abs(diffY),abs(diffZ),abs(diff3DX),abs(diff3DY),abs(diff3DZ),abs(diffB),abs(diffTextX),abs(diffTextY)}) + 1;

  float xStepSize = diffX/numberOfSteps;
  float yStepSize = diffY/numberOfSteps;
  float zStepSize = diffZ/numberOfSteps;

  float x3DStepSize = diff3DX/numberOfSteps;
  float y3DStepSize = diff3DY/numberOfSteps;
  float z3DStepSize = diff3DZ/numberOfSteps;

  float xTextStepSize = diffTextX/numberOfSteps;
  float yTextStepSize = diffTextY/numberOfSteps;

  float bStepSize = diffB/numberOfSteps;

  for (float i=0.0; i<numberOfSteps+1; i++) {
       float x = from.x + (xStepSize*i) ;
       float y = from.y + (yStepSize*i) ;
       float z = from.depth + (zStepSize*i);

       float x3D = from.pos3d.x + (x3DStepSize*i) ;
       float y3D = from.pos3d.y + (y3DStepSize*i) ;
       float z3D = from.pos3d.z + (z3DStepSize*i);

       float brightness = from.brightness + (bStepSize*i);


       float xT = vtFrom.x + (xTextStepSize*i);
       float yT = vtFrom.y + (yTextStepSize*i);


       glm::vec3 pos3d = glm::vec3(x3D,y3D,z3D);
       CanvasPoint point = CanvasPoint(x,y,z,pos3d);
       point.texturePoint = TexturePoint(xT, yT);
       point.brightness = brightness;
       canvasPoints.push_back(point);
       }
  return canvasPoints;
}

// Split the triangle
CanvasPoint calculateExtra(CanvasTriangle triangle) {
  float extraX;
  float extraZ;
  glm::vec3 extraPos;
  float brightness;
  TexturePoint texture;

  std::vector<CanvasPoint> canvasPoints = interpolate(triangle.vertices[0],triangle.vertices[2]);

  for (int i=0; i< canvasPoints.size(); i++) {
       // CASE: Pixel is at the same height with the middle vertex
       if(round(canvasPoints[i].y) == round(triangle.vertices[1].y)){
        extraX = canvasPoints[i].x;
        extraZ = canvasPoints[i].depth;
        extraPos = canvasPoints[i].pos3d;
        brightness = canvasPoints[i].brightness;
        texture =canvasPoints[i].texturePoint;
       }
  }
  CanvasPoint point = CanvasPoint(extraX,triangle.vertices[1].y,extraZ, extraPos);
  point.brightness = brightness;
  point.texturePoint = texture;
  return point;
}

// Fill triangle
void fillTriangle(CanvasPoint v1, CanvasPoint v2, CanvasPoint v3, Colour colour){
  std::vector<CanvasPoint> canvasPoints1 = interpolate(v1,v2);
  std::vector<CanvasPoint> canvasPoints2 = interpolate(v1,v3);


  for (int i=0; i<canvasPoints1.size(); i++) {
       for (int j=0; j<canvasPoints2.size()  ; j++) {
            // CASE: Pixels are at the same height
            if(round(canvasPoints1[i].y) == round(canvasPoints2[j].y)){

              CanvasPoint from = CanvasPoint(round(canvasPoints1[i].x),round(canvasPoints1[i].y),canvasPoints1[i].depth,canvasPoints1[i].pos3d);
              CanvasPoint to = CanvasPoint(round(canvasPoints2[j].x),round(canvasPoints2[j].y),canvasPoints2[j].depth,canvasPoints2[j].pos3d);

              from.brightness = canvasPoints1[i].brightness;
              from.texturePoint = canvasPoints1[i].texturePoint;

              to.brightness = canvasPoints2[j].brightness;
              to.texturePoint = canvasPoints2[j].texturePoint;
              drawLine(from, to, colour);
            }
       }
   }
}

//FXAA
void applyAntiAliasing(){
  for(int row = 0; row < HEIGHT; row++){ // looping through the square
      for(int col = 0; col < WIDTH; col++){

        glm::vec3 colourCentre = glm::vec3(screen[col][row].x,screen[col][row].y,screen[col][row].z);
        glm::vec3 fragColor = glm::vec3(screen[col][row].x,screen[col][row].y,screen[col][row].z);
        float lumaCentre = rgb2luma(colourCentre);

        if (0 >= row || 0 >= col || HEIGHT-1 <= row || WIDTH-1 <= col)  continue;
        glm::vec2 inverseScreenSize(1.0f/WIDTH, 1.0f/HEIGHT);



       float lumaDown = rgb2luma(screen[row][col-1]);
       float lumaUp = rgb2luma(screen[row][col+1]);
       float lumaLeft = rgb2luma(screen[row-1][col]);
       float lumaRight = rgb2luma(screen[row+1][col]);

       float lumaMin = std::min(lumaCentre,std::min(std::min(lumaDown,lumaUp),std::min(lumaLeft,lumaRight)));
       float lumaMax = std::max(lumaCentre,std::max(std::max(lumaDown,lumaUp),std::max(lumaLeft,lumaRight)));
       float lumaRange = lumaMax - lumaMin;

       if(lumaRange < std::max(EDGE_THRESHOLD_MIN,lumaMax*EDGE_THRESHOLD_MAX)){
           fragColor = colourCentre;
           continue;
       }

       float lumaDownUp = lumaDown + lumaUp;
       float lumaLeftRight = lumaLeft + lumaRight;

       float lumaDownLeft = rgb2luma(screen[row-1][col-1]);
       float lumaUpRight = rgb2luma(screen[row+1][col+1]);
       float lumaUpLeft = rgb2luma(screen[row-1][col+1]);
       float lumaDownRight = rgb2luma(screen[row+1][col-1]);
       float lumaLeftCorners = lumaDownLeft + lumaUpLeft;
       float lumaDownCorners = lumaDownLeft + lumaDownRight;
       float lumaRightCorners = lumaDownRight + lumaUpRight;
       float lumaUpCorners = lumaUpRight + lumaUpLeft;

       bool isHorizontal = (abs(-2.0f*lumaLeft+lumaLeftCorners)+abs(-2.0*lumaCentre+lumaDownUp )*2.0+abs(-2.0*lumaRight+lumaRightCorners)
           >= abs((-2.0) * lumaUp+lumaUpCorners)+abs(-2.0*lumaCentre+lumaLeftRight)*2.0+abs(-2.0*lumaDown+lumaDownCorners));

       float luma1 = isHorizontal ? lumaDown : lumaLeft;
       float luma2 = isHorizontal ? lumaUp : lumaRight;
       float gradient1 = luma1 - lumaCentre;
       float gradient2 = luma2 - lumaCentre;

       float gradientScaled = 0.25f*std::max(abs(gradient1),abs(gradient2));
       float stepLength = isHorizontal ? inverseScreenSize.y : inverseScreenSize.x;
       float lumaLocalAverage = 0.0f;

       if(abs(gradient1) < abs(gradient2)) {
           lumaLocalAverage = 0.5*(luma2 + lumaCentre);
       } else {
           stepLength = - stepLength;
           lumaLocalAverage = 0.5*(luma1 + lumaCentre);
       }

       glm::vec2 currentUv(row, col);
       if(isHorizontal){
           currentUv.y += stepLength * 0.5;
       } else {
           currentUv.x += stepLength * 0.5;
       }

      glm:: vec2 offset = isHorizontal ? glm::vec2(inverseScreenSize.x,0.0) : glm::vec2(0.0,inverseScreenSize.y);
       glm::vec2 uv1 = currentUv - offset;
       glm::vec2 uv2 = currentUv + offset;

       float lumaEnd1 = rgb2luma(screen[(int)uv1.x][(int)uv1.y]);
       float lumaEnd2 = rgb2luma(screen[(int)uv2.x][(int)uv2.y]);
       lumaEnd1 -= lumaLocalAverage;
       lumaEnd2 -= lumaLocalAverage;

       bool reached1 = abs(lumaEnd1) >= gradientScaled;
       bool reached2 = abs(lumaEnd2) >= gradientScaled;
       bool reachedBoth = reached1 && reached2;

       if(!reached1) uv1 -= offset;
       if(!reached2) uv2 += offset;

       if(!reachedBoth) {
           for(int i = 2; i < 12; i++){
               if(!reached1){
                   lumaEnd1 = rgb2luma(screen[(int)uv1.x][(int)uv1.y]);
                   lumaEnd1 = lumaEnd1 - lumaLocalAverage;
               }
               if(!reached2){
                   lumaEnd2 = rgb2luma(screen[(int)uv2.x][(int)uv2.y]);
                   lumaEnd2 = lumaEnd2 - lumaLocalAverage;
               }
               reached1 = abs(lumaEnd1) >= gradientScaled;
               reached2 = abs(lumaEnd2) >= gradientScaled;
               reachedBoth = reached1 && reached2;

               if(!reached1) {
                   uv1 -= offset * quality[i];
               }
               if(!reached2) {
                   uv2 += offset * quality[i];
               }

               if(reachedBoth){
                   break;
               }
           }
       }

       float distance1 = isHorizontal ? (row - uv1.x) : (col - uv1.y);
       float distance2 = isHorizontal ? (uv2.x - row) : (uv2.y - col);
       bool dir = distance1 < distance2;
       float distanceFinal = std::min(distance1, distance2);
       float edgeThickness = (distance1 + distance2);
       float pixelOffset = - distanceFinal / edgeThickness + 0.5;
       bool iscenterSmaller = lumaCentre < lumaLocalAverage;
       bool var = ((dir ? lumaEnd1 : lumaEnd2) < 0.0) != iscenterSmaller;
       float finalOffset = var ? pixelOffset : 0.0;
       float avg = (1.0/12.0) * (2.0 * (lumaDownUp + lumaLeftRight) + lumaLeftCorners + lumaRightCorners);

       float tmp = 0.0;
       if (abs(avg - lumaCentre)/lumaRange < 0.0) {
           tmp = 0.0;
       } else if (abs(avg - lumaCentre)/lumaRange > 1.0) {
           tmp = 1.0;
       } else {
           tmp = abs(avg - lumaCentre)/lumaRange;
       }

       float subPixelOffset1 = tmp;
       float subPixelOffset2 = (-2.0 * subPixelOffset1 + 3.0) * subPixelOffset1 * subPixelOffset1;
       float subPixelOffsetFinal = subPixelOffset2 * subPixelOffset2 * SUBPIXEL_QUALITY;

       finalOffset = std::max(finalOffset,subPixelOffsetFinal);

       glm::vec2 finalUv(row, col);
       if(isHorizontal){
           finalUv.y += finalOffset * stepLength;
       } else {
           finalUv.x += finalOffset * stepLength;
       }

       screen[row][col] = (screen[(int)finalUv.x][(int)finalUv.y]+ 1.0f * screen[row][col]) / 2.0f;
    }
  }
}

//Convert rgb to luma
float rgb2luma(glm::vec3 rgb) {
    return sqrt(glm::dot(rgb, glm::vec3(0.299, 0.587, 0.114)));
}
////Wireframe
//////////////////////////////////////////////////////////////////////////////////////////////////
//Draw wireframe
void drawWireframe(CanvasTriangle triangle, Colour colour) {
  drawLine(triangle.vertices[0], triangle.vertices[1], colour);
  drawLine(triangle.vertices[1], triangle.vertices[2], colour);
  drawLine(triangle.vertices[2], triangle.vertices[0], colour);
}
////RAYTRACING
//////////////////////////////////////////////////////////////////////////////////////////////////
//Draw scene
void rayTracing(std::vector<ModelTriangle> triangles) {
  //Generate lights
  GenAreaLight();
  for(int i=0; i < WIDTH; i++) {
    for(int j=0; j < HEIGHT; j++) {
      screen[i][j] = traceRayFromCamera(i,j,triangles);
    }
  }
}

//Generate lights
void GenAreaLight() {
  int index = 0;
  float min = -0.11;
  float max = 0.11;
  for(int i = 0; i < NUM_LIGHT_RAYS; i++) {
    float offset_x = glm::linearRand(min, max);
    float offset_z = glm::linearRand(min, max);
    lightPositionArr[i] = glm::vec3(lightPos.x + offset_x, lightPos.y, lightPos.z + offset_z);
  }
}

//Trace ray from camera
glm::vec3 traceRayFromCamera(float x, float y,std::vector<ModelTriangle> triangles) {
	Colour colour ;

  //Compute ray direction
  glm::vec3 dir = glm::vec3(x - WIDTH/2, y - HEIGHT/2, focalLength);
  dir = glm::normalize(-dir);
  orientationMatrix = RotationX * RotationY;
  //Check for lookAt
  if(lock)
     lookAt();
  //Get closest intersection
  RayTriangleIntersection closestinter = getClosestIntersection(triangles, orientationMatrix*dir, cameraPos);
  float check = std::numeric_limits<float>::max();
  //CASE: Intersection found
  if(closestinter.distanceFromCamera < check) {
    colour = directLight(closestinter, triangles, dir);
  }
  //CASE: Intersection not found
  else {
    colour = Colour(0, 0, 0);
  }


   if(glm::dot((closestinter.intersectedTriangle.vertices[0] - cameraPos)*orientationMatrix,closestinter.intersectedTriangle.triangleNormal) > 0)
   colour = Colour(0, 0, 0);
  //Colour pixel
  float red = colour.red;
  float green = colour.green;
  float blue = colour.blue;


  glm::vec3 col = glm::vec3(red,green,blue);


  return col;
}

//Get closest intersection
RayTriangleIntersection getClosestIntersection(std::vector<ModelTriangle> triangles,  glm::vec3 rayDirection,glm::vec3 start ) {

  float minDist = std::numeric_limits<float>::max();
  RayTriangleIntersection result = RayTriangleIntersection();
  result.distanceFromCamera = minDist;

  for(int i = 0; i < triangles.size(); i++) {
    //Compute intersection
    glm::vec3 intersect = intersection(triangles[i], rayDirection, start);
    float t, u, v;
    t = intersect[0];
    u = intersect[1];
    v = intersect[2];
    //CASE:Intersection found
    if(t < minDist
        && 0.0 <= u && u <= 1.0
          && 0.0 <= v && v <= 1.0
            && (u + v) <= 1.0 && t >= 0.001) {
      minDist = t;
      //Save intersection data
      result.intersectionPoint = triangles[i].vertices[0] + u*(triangles[i].vertices[1] - triangles[i].vertices[0])+ v*(triangles[i].vertices[2] - triangles[i].vertices[0]);
      result.distanceFromCamera = minDist;
      result.intersectedTriangle = triangles[i];
      result.normal = triangles[i].normals[0] + u*(triangles[i].normals[1] - triangles[i].normals[0])+ v*(triangles[i].normals[2] - triangles[i].normals[0]);
      result.triangleIndex = i;
    }
  }
  return result;
}

//Compute intersection
glm::vec3 intersection(ModelTriangle triangle, glm::vec3 rayDirection, glm::vec3 start) {

  glm::vec3 e0 = triangle.vertices[1] - triangle.vertices[0];
  glm::vec3 e1 = triangle.vertices[2] - triangle.vertices[0];
  glm::vec3 SPVector = start-triangle.vertices[0];
  glm::mat3 DEMatrix(-rayDirection, e0, e1);
  glm::vec3 possibleSolution = glm::inverse(DEMatrix) * SPVector;

  return possibleSolution;
}

float getDistance(glm::vec3 p1, glm::vec3 p2) {
  float dx = p2.x - p1.x;
  float dy = p2.y - p1.y;
  float dz = p2.z - p1.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}
// Compute direct light
// Mirror, glass
// Ambient,diffuse,specular light
// Soft shadows
Colour directLight(RayTriangleIntersection input, std::vector<ModelTriangle> triangles, glm::vec3 rayDir) {
  float check = std::numeric_limits<float>::max();
  //CASE:Mirror found
  if(input.triangleIndex == 10 || input.triangleIndex == 11) {
      isMirror = 1;
      //Compute relfection's direction
      glm::vec3 next_ray= getReflectedDirection(rayDir, input.intersectedTriangle.triangleNormal);
      //Get reflection's intersection
      RayTriangleIntersection closest = getClosestIntersection(triangles, next_ray, input.intersectionPoint );
      //CASE:Intersection not found
      if(closest.distanceFromCamera == check)
        return  Colour(0, 0, 0);
      //CASE:Intersection found
      else
			  return directLight(closest, triangles, next_ray);
  }
  //CASE: Glass found
  if(input.triangleIndex == 12 || input.triangleIndex == 13 || input.triangleIndex == 14 || input.triangleIndex == 15 || input.triangleIndex == 16 || input.triangleIndex == 17 || input.triangleIndex == 18 || input.triangleIndex == 19 || input.triangleIndex == 20 || input.triangleIndex == 21){

      glm::vec3 surfaceNormal = input.intersectedTriangle.triangleNormal;
      glm::vec3 direction = rayDir;
      float currRefraction = GLASS_INDEX_OF_REFRACTION;
      // CASE: Inside glass object
      if(glm::dot(surfaceNormal, direction) > 0) {
        surfaceNormal *= -1.0f;
        currRefraction = 1 / currRefraction;
      }
      currRefraction = 1 / currRefraction;

      // Calculate cos theta
      float cost1 = std::max(glm::dot(surfaceNormal, direction) * -1.0f, 0.f);
      float cost2 = 1.0f - currRefraction * currRefraction * (1.0f - cost1 * cost1);

      // CASE: Refraction direction
      if (cost2 > 0) {
        // Snell's law vector
        direction = normalize((direction * currRefraction) + (surfaceNormal * (currRefraction * cost1 - sqrt(cost2))));
      }
      // CASE: Reflection direction
      else {
        // Snell's law vector
        direction = normalize(direction + surfaceNormal * (cost1 * 2.0f));
      }

     RayTriangleIntersection intersection = getClosestIntersection(triangles, direction, input.intersectionPoint );
     // CASE: Intersection found
     if(intersection.distanceFromCamera < check) {
        Colour colour =  directLight(intersection,triangles,direction);
        return Colour(colour.red*GLASS_MAGIC_NUMBER,colour.green*GLASS_MAGIC_NUMBER,colour.blue*GLASS_MAGIC_NUMBER);
     }
     // CASE: Intersection not found
     else {
        return  Colour(0, 0, 0);
     }
  }

  glm::vec3  D;
  glm::vec3 totalPixelIntensity = glm::vec3(0.f, 0.f, 0.f);
  glm::vec3 light;
  for(int i = 0; i < NUM_LIGHT_RAYS; i++) {
    //Calculate light's direction
    light = lightPositionArr[i];
    glm::vec3 lightDir = light - input.intersectionPoint;
    float dir_l = glm::length(lightDir);
    lightDir = glm::normalize(lightDir);
    //Get light's intersection
    RayTriangleIntersection closest = getClosestIntersection(triangles, lightDir, input.intersectionPoint + 0.001f*lightDir);
    //CASE:Intersection found -> do shadow
    if ( closest.distanceFromCamera <= dir_l  ) {
        D = glm::vec3 (0.1*input.intersectedTriangle.colour.red,
                       0.1*input.intersectedTriangle.colour.green,
                       0.1*input.intersectedTriangle.colour.blue );
    }
    //CASE:Intersection not found -> do light
    else {
       //Calculate diffuse light
       glm::vec3 normal = input.normal;
       float distance = getDistance(input.intersectionPoint,light);
       float divisor = 4 * 3.14 * distance * distance;
       float max = fmax(glm::dot(lightDir,normal),0);
       float diffuse = (directLightPower*max/divisor);
       //Calculate specular light
       float specular = 0;
       glm::vec3 reflection= getReflectedDirection(lightDir, normal);
       reflection = glm::normalize(reflection);
       double dot2 = glm::dot(reflection,rayDir);
       if(input.triangleIndex == 6 || input.triangleIndex == 7)
          if(dot2 > 0)
             specular = pow(dot2,256);

       //Calculate brightness
       float brightness = diffuse   + indirectLightPower + specular;

       //Normalize brightness
       if(brightness > 1)
         brightness = 1;
       else if(brightness < 0.2)
         brightness = 0.2;
       //Apply brightness
       D = glm::vec3 (brightness*input.intersectedTriangle.colour.red,
                      brightness*input.intersectedTriangle.colour.green,
                      brightness*input.intersectedTriangle.colour.blue );
       }

       totalPixelIntensity = totalPixelIntensity + D;
    }
   //Compute soft shadows
   glm::vec3 avgPixelIntensity = totalPixelIntensity / (float) NUM_LIGHT_RAYS;
   Colour col = Colour(avgPixelIntensity.x,avgPixelIntensity.y,avgPixelIntensity.z);
   return col;
}

//Compute reflection's direction
glm::vec3 getReflectedDirection(const glm::vec3& incident, const glm::vec3& normal) {
	glm::vec3 reflected = incident - normal * (2 * glm::dot(incident, normal));
  return reflected;
}

//Apply anti-aliasing
void antiAliasing(std::vector<ModelTriangle> triangles) {
	aliasedEdges->clear();
	computePixelsIntensity();

	// Compute edges
	for (int y = 1; y < HEIGHT - 1; ++y) {
	  for (int x = 1; x < WIDTH - 1; ++x) {
			//CASE: Pixel is on an edge
			if (sobelOperator(x, y) > SOBEL_THRESHOLD) {
				// Store pixel
				aliasedEdges->push_back(std::pair<int, int>(x, y));
			}
		}
 	}
  //Apply anti-aliasing on each edge
	for (const std::pair<int, int>& p : *aliasedEdges) {
    isMirror = 0;
    //Apply supersampling
    glm::vec3 color = supersamplingAA(p.first, p.second,triangles);
    //CASE: Pixel is not mirror
    if(!isMirror)
      screen[p.first][p.second] = color;
	}
}

//Compute the pixel's intensity
void computePixelsIntensity() {
	for (int y = 1; y < HEIGHT - 1; ++y) {
		for (int x = 1; x < WIDTH - 1; ++x) {
			screenPixelsIntensity[y][x] = glm::dot(screen[y][x], INTENSITY_WEIGHTS);
		}
	}
}

//Calculate sobel
float sobelOperator(int x, int y) {
	return abs((screenPixelsIntensity[y - 1][x - 1] + 2.0f * screenPixelsIntensity[y - 1][x] + screenPixelsIntensity[y - 1][x + 1])
		- (screenPixelsIntensity[y + 1][x - 1] + 2.0f * screenPixelsIntensity[y + 1][x] + screenPixelsIntensity[y + 1][x + 1]))
		+ abs((screenPixelsIntensity[y - 1][x + 1] + 2.0f * screenPixelsIntensity[y][x + 1] + screenPixelsIntensity[y + 1][x + 1])
		- (screenPixelsIntensity[y - 1][x - 1] + 2.0f * screenPixelsIntensity[y][x - 1] + screenPixelsIntensity[y + 1][x - 1]));
}

//SuperSampling algorithm
glm::vec3 supersamplingAA(int x, int y, std::vector<ModelTriangle> triangles) {
	glm::vec3 color = screen[x][y];
  for (float x1 = x - 0.5; x1 < x + 1; x1 += 0.5) {
    for (float y1 = y - 0.5; y1 < y + 1; y1 += 0.5) {
			if (x1 != x || y1 != y) {
				color += traceRayFromCamera(x1, y1, triangles);
			}
		}
	}
	return color / 9.0f;
}
