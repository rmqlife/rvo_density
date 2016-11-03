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
GLUquadricObj *quadratic[num_of_agents];
RVOSimulator *sim = new RVOSimulator();//example
void move();
void draw_skeleton(){
	glLineWidth(1);
	glBegin(GL_LINES);
	glColor3f(1.0,0.0,0.0);
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
	/*glPushMatrix();
	glTranslatef((position.x()-10), yOffset, (position.y()+10));
	glRotatef(270,1,0,0);
	glScalef(s,s,s);
	int v_id = idex_clr;
	string s_vid = to_string((long long)v_id);
	for (int s = 0;s < s_vid.size();s++){
		glutStrokeCharacter(GLUT_STROKE_ROMAN,s_vid.c_str()[s]);
	}
	glPopMatrix();*/

	/*draw circle*/
	Circle c ( Vector2(position.x(), position.y()),RadiusOfRobot);
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
		/*draw mapping position on skeleton*/
		if (!predictedPosesOnSkelton.empty()){
			Vector2 mappingPos = predictedPosesOnSkelton[i];
			clr.r = 0;
			clr.g = 0;
			clr.b = 0;
			draw_Rectangle(i, mappingPos,clr);//predicting position
		}
		
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
	draw_skeleton();
	
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
		/*	if(numFrame == 659){
				int ttt = 0;
			}*/
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
	//vector<Vector2> data;
	//data.push_back(Vector2(1,0));
	//data.push_back(Vector2(2,0));
	//data.push_back(Vector2(3,0));
	//data.push_back(Vector2(5,0));
	//Vector2 test = Vector2(4,0);
	//kdTreeGeneral tree;
	//tree.buildTree(data);
	//float dst = -1;
	//Vector2 result = tree.searchNearest(test,dst);
	//vector<Vector2> results = tree.searchNearestRange(test,2);
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
void testDijskla(){
	//vector< vector<int> > initialDst;
	//vector<int> dst0(9,INT_MAX),dst1(9,INT_MAX),dst2for (size_t c = 0;c < nPts;c++){
	
	//			,dst3(9,INT_MAX),dst4(9,INT_MAX),dst5(9,INT_MAX)
	//			,dst6(9,INT_MAX),dst7(9,INT_MAX),dst8(9,INT_MAX);
	//dst0[0] = 0;dst0[1] = 4;dst0[7] = 8;
	//dst1[1] = 0;dst1[0] = 4;dst1[2] = 8;dst1[7] = 11;
	//dst2[2] = 0;dst2[1] = 8;dst2[3] = 7;dst2[8] = 2;
	//dst3[3] = 0;dst3[2] = 7;dst3[4] = 9;dst3[5] = 14;
	//dst4[4] = 0;dst4[3] = 9;dst4[5] = 10;
	//dst5[5] = 0;dst5[2] = 4;dst5[3] = 14;dst5[4] = 10;dst5[6] = 2;
	//dst6[6] = 0;dst6[5] = 2;dst6[7] = 1;dst6[8] = 6;
	//dst7[7] = 0;dst7[0] = 8;dst7[1] = 11;dst7[6] = 1;dst7[8] = 7;
	//dst8[8] = 0;dst8[2] = 2;dst8[6] = 6;dst8[7] = 7;
	//initialDst.push_back(dst0);
	//initialDst.push_back(dst1);
	//initialDst.push_back(dst2);
	//initialDst.push_back(dst3);
	//initialDst.push_back(dst4);
	//initialDst.push_back(dst5);
	//initialDst.push_back(dst6);
	//initialDst.push_back(dst7);
	//initialDst.push_back(dst8);
	//vector<size_t> result = dijkstra(initialDst,  0);
}
