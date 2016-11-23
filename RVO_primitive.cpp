#include "stdafx.h"
#include "DrawPart.h"
#include "Robot.h"
#include "Definitions.h"
#include "ElementMovement.h"
//the agent initial

static GLubyte checkImage[ImageHeight][ImageWidth][4]; // image pixel
static GLuint texName;
vector<Vector2> predictedPosesOnSkelton;
struct Color{
	Color():r(1),g(1),b(1){
	}
	Color(float r_,float g_,float b_):r(r_),g(g_),b(b_){

	}
	float r,g,b;
};

RVOSimulator *sim = new RVOSimulator();//example
void move();
void draw_skeleton(){
	glLineWidth(1);
	glBegin(GL_LINES);
	glColor3f(0.0,0.0,1.0);
	for (size_t i = 0;i < sim->skeleton_.size();i++){
		Vector2 p0 = sim->skeleton_[i].begin()->position;
		Vector2 p1 = (sim->skeleton_[i].end()-1)->position;
		glVertex3f(p0.x(), yOffset, p0.y());
		glVertex3f(p1.x(), yOffset, p1.y());
	}
	glEnd();
}
void draw_Rectangle(int idex_clr,Vector2 position, Color clr)
{
	/*draw character*/
	float s = 0.01;
	glColor3f(clr.r,clr.g,clr.b); 
	/*draw circle*/
	Circle c( Vector2(position.x(), position.y()),RadiusOfRobot);
	vector<pair<Vector2,Vector2>> vL = draw_Circle(c);
	glLineWidth(1);
	glBegin(GL_LINES);
	for (int j = 0;j < vL.size();j++){
		glVertex3f(vL[j].first.x(), yOffset, vL[j].first.y()) ;
		glVertex3f(vL[j].second.x(), yOffset, vL[j].second.y()) ;
	}
	glVertex3f(vL[0].first.x(), yOffset, vL[0].first.y()) ;
	glEnd();
}
void draw_robot()
{
	//draw the sphere
	//set the agents' position
	int idex_clr = 1;
	for (int i = 0;i < sim->getNumAgents();i++){
		Vector2 position = sim->getAgentPosition(i);
		/*draw Agent*/
		Color clr;
		clr.r = 1;
		clr.g = 0;
		clr.b = 0;
		draw_Rectangle(i, position,clr);//agents
		/*draw mapping position on skeleton
		if (!predictedPosesOnSkelton.empty()){
			Vector2 mappingPos = predictedPosesOnSkelton[i];
			clr.r = 0;
			clr.g = 0;
			clr.b = 0;
			draw_Rectangle(i, mappingPos,clr);//predicting position
		}
		*/
	}	
};
void drawBackGround(IplImage* img);
void init()
{
	/* Set up the scenario. */
	setupScenario(sim);
	setPreferredVelocities(sim);
	IplImage* img = cvLoadImage( fileName.c_str() );

	drawBackGround(img);
	glClearColor(0, 0 , 0 , 0.0 );
	glShadeModel(GL_FLAT);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	glGenTextures(1, &texName);
	glBindTexture(GL_TEXTURE_2D, texName);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, 
		GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, 
		GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ImageWidth, 
		ImageHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
		checkImage);
}
extern vector<MyObstacle> obsts;
int numFrame = 0;
void display()
{
	glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);

	glColor3f(0.0,0.0,0.0);
	glLoadIdentity ();
	gluLookAt(600 ,1200 , 200 , 600 , 0.0 , 200 ,  0.0 ,0.0 , -1.0);
	glScalef(1.0 , 1.0 , 1.0);
	glEnable(GL_DEPTH_TEST);

	glLineWidth(1.2);
	//_sleep(1);
	draw_robot();
	//draw_skeleton();
	
	/*draw character*/
	const float s = 0.4;
	glPushMatrix();
	glTranslatef(-20, yOffset, -20);
	glRotatef(270,1,0,0);
	glScalef(s,s,s);
	int v_id = numFrame;	
	string s_vid = to_string((long long)v_id);
	for (int s = 0;s < s_vid.size();s++){
		glutStrokeCharacter(GLUT_STROKE_ROMAN,s_vid.c_str()[s]);
	}
	glPopMatrix();
	//draw_obstacles(obsts);
	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glBindTexture(GL_TEXTURE_2D, texName);
	glColor3f(0.25,0.25,0.25);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 0.0);glVertex3f(0, -0, 0.0);
	glTexCoord2f(1.0, 0.0);glVertex3f(ImageWidth, -0, 0.0);
	glTexCoord2f(1.0, 1.0);glVertex3f(ImageWidth, -0, ImageHeight);
	glTexCoord2f(0.0, 1.0);glVertex3f(0, -0, ImageHeight);
	glEnd();
	glFlush();
	glDisable(GL_TEXTURE_2D);
	glutSwapBuffers();
}

void reshape(int w, int h)
{
	glViewport(0, 0,(GLsizei)w,(GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45,(float)w/h,0.1,3000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity ();
}
int click = 0;

AgentsSequenceMovements movements;
void move()
{	
	if (1==click%2){
		if(false == reachedGoal(sim)){
			sim->doStep();
			predictedPosesOnSkelton = sim->calculateMappingPositionOnSkeleton();
			numFrame++;
		}
	}
	glutPostRedisplay();
}
void mouse(int button, int state, int x, int y)
{
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN){
			click++;
			glutIdleFunc(move);
		}
		break;
	default:
		break;
	}
}
int main(int argc ,char *argv[])
{
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(800,600);
	glutCreateWindow("meco_scale");
	init();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutMouseFunc(mouse);
	glutMainLoop();
	return 0;
}
void drawBackGround(IplImage* img){
	int i, j, c;
	/*background*/
	for (i = 0; i < ImageHeight; i++) {
		for (j = 0; j < ImageWidth; j++) {
			CvScalar pixel = cvGet2D(img,i,j);
			checkImage[i][j][0] = (GLubyte) pixel.val[0];
			checkImage[i][j][1] = (GLubyte) pixel.val[1];
			checkImage[i][j][2] = (GLubyte) pixel.val[2];
			checkImage[i][j][3] = (GLubyte) 255;
		}
	}
	//*skeleton*/
}