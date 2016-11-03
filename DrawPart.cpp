#include "stdafx.h"
#include "DrawPart.h"

void draw_obstacles(vector<MyObstacle> obsts)
{
	
	for (int i = 0 ;i < obsts.size();i++)
	{
		glBegin(GL_LINE_LOOP);
		glColor3f(1,0,0);
		for (int j = 0;j < obsts[i].vtx.size();j++)
		{
			float x = obsts[i].vtx[j].x();
			float y = obsts[i].vtx[j].y();
			glVertex3f(x,  -0.01, y);
		}
		glEnd();
		glBegin(GL_POLYGON);
		glColor3f(0.5,0.5,0.5);
		for (int j = 0;j < obsts[i].vtx.size();j++)
		{
			float x = obsts[i].vtx[j].x();
			float y = obsts[i].vtx[j].y();
			glVertex3f(x,  -0.01, y);
		}
		glEnd();
	}

}

void draw_ground() 
{
	int count = 0;

	GLfloat white4[] = {.4, .4, .4, 1.};
	GLfloat white1[] = {.1, .1, .1, 1.};
	GLfloat green5[] = {0., .5, 0., 1.};
	GLfloat green2[] = {0., .2, 0., 1.};
	GLfloat black[] = {0., 0., 0., 1.};
	GLfloat red[] = {1, 0., 0., 1.};
	GLfloat mat_shininess[] = {7.};		/* Phong exponent */

	int numofQuad = 31;
	glLineWidth(1);
	glBegin(GL_LINES);
	glColor3f(0.6,0.6,0.6);
	for(int i=-numofQuad;i<=numofQuad;i+=1) {
		glVertex3f(100 + i*5.0, 0.1,100) ;
		glVertex3f(100 + i*5.0, 0.1, -100) ;
		glVertex3f(-100, 0.1, 100 + i*5) ;
		glVertex3f(100, 0.1, 100 + i*5) ;
	}
	glEnd();
	glLineWidth(0.1);
	glBegin(GL_LINES);
	glColor3f(0.1,0.1,0.1);

	for(int i=-numofQuad*10;i<=numofQuad*10;i+=1) {
		glVertex3f(100 + i*0.5, 0.1,100) ;
		glVertex3f(100 + i*0.5, 0.1, -100) ;
		glVertex3f(-100, 0.1, 100 + i*0.5) ;
		glVertex3f(100, 0.1, 100 + i*0.5) ;
	}
	glEnd();
}
vector< pair<Vector2,Vector2> > draw_Circle(Circle circle)
{
	vector< pair<Vector2,Vector2> > lines;
	const int sec = 20;
	float secAngle = PI*2/sec;
	float cx = circle.center.x();
	float cy = circle.center.y();
	for (int i = 0;i <= sec;i++){
		pair<Vector2,Vector2> line;
		float cFir = circle.radius * cos(i*secAngle);
		float sFir = circle.radius * sin(i*secAngle);

		float cSec = circle.radius * cos((i+1)*secAngle);
		float sSec = circle.radius * sin((i+1)*secAngle);

		float firX = cFir + cx;
		float firY = sFir + cy;

		float secX = cSec + cx;
		float secY = sSec + cy;
		line.first.x_ = firX;
		line.first.y_ = firY;

		line.second.x_ = secX;
		line.second.y_ = secY;

		lines.push_back(line);
	}
	return lines;
}