#include <Windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <stdlib.h>
#include <GL/glut.h>
#include "Robot.h"
#include <vector>
using namespace std;
class Circle
{
public:
	Circle():center(Vector2(0,0)),radius(0){}
	Circle(Vector2 c, float r):center(c) , radius(r)
	{}
public:
	Vector2 center;
	float radius;
public:
	virtual void DrawGraphics(){}
};

void draw_obstacles(vector<MyObstacle> obsts);
void draw_ground();
vector< pair<Vector2,Vector2> > draw_Circle(Circle circle);