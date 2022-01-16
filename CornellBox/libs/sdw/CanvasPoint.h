#pragma once
#include "TexturePoint.h"
#include <iostream>

class CanvasPoint
{
  public:
    float x;
    float y;
    double depth;
    float brightness;
    TexturePoint texturePoint;
    glm::vec3 pos3d;

    CanvasPoint()
    {
        texturePoint = TexturePoint(-1,-1);
    }

    CanvasPoint(float xPos, float yPos)
    {
      x = xPos;
      y = yPos;
      depth = 0.0;
      brightness = 1.0;
      texturePoint = TexturePoint(-1,-1);
    }

    CanvasPoint(float xPos, float yPos, float pointDepth)
    {
      x = xPos;
      y = yPos;
      depth = pointDepth;
      brightness = 1.0;
      texturePoint = TexturePoint(-1,-1);
    }

    CanvasPoint(float xPos, float yPos, float pointDepth, glm::vec3 pos)
    {
      x = xPos;
      y = yPos;
      depth = pointDepth;
      pos3d = pos;
      brightness = 1.0;
      texturePoint = TexturePoint(-1,-1);
    }

};

std::ostream& operator<<(std::ostream& os, const CanvasPoint& point)
{
    os << "(" << point.x << ", " << point.y << ", " << point.depth << ") " << point.brightness << std::endl;
    return os;
}
