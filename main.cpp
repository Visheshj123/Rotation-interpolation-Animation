// standard
#include <assert.h>
#include <math.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <vector>
#include "quaternion.hpp"
#include <utility>
#include <iomanip>
#include <vector>
#include "euler.hpp"
#include "key.hpp"
using namespace std;

/* CONTROLS
 
 W(up), A(left), S(down), D(right) translation
 X,Y,Z + vertical mouse movement for rotation around X,Y, or Z axis
 B → Use B Spline basis matrix
 C → use catmull rom basis matrix
 Q → use quaternions
 E → use euler angles
 DEL → Clear all keys and stop interpolation
 SPACE → Set key
 I → interpolate over keys
 Up arrow → increase z-coordinate
 Down arrow → decrease z-coordinate


*/


// glut
#include <GLUT/GLUT.h>

//================================
// global variables
//================================
// screen size
int g_screenWidth  = 0;
int g_screenHeight = 0;

//toggle to use Quaternions, Euler angles, catmull-spline, spline
int useQ = 1;

//toggle to start interpolation/approximation
int interpolateQ = 0;

//toggle for which matrix to use
int catmull = 1;
int bspline = 0;

//interpolation index, keeps track of which keys to use for interpolation
int ii = 3;
GLfloat t = 0.00;

//basis matrix to catmull and bspline
Eigen::Matrix4f m;

Eigen::MatrixXf m1;
Quaternion q1; 


// frame index
int g_frameIndex = 0;

// angle for rotation
float eAngles[3] = {30,0,0}; //z,y,x angles to rotate



//points for translation
GLfloat x = 0.5;
GLfloat y = 0;
GLfloat z = -5.0;


//Angle to rotate with mouse
int angletoRotate = 0;

//vector to store all user generated keys
vector<key> vectorKeys;


//================================
// init
//================================
void init( void ) {
	// init something before main loop...
}

//================================
// update
//================================
void update( void ) {
    
    ::t+=0.01;

}



//sets global variable to determine to matrix for interpolation or approximation
void loadBasis(){
            
    if(bspline == 1){
        ::m <<  -1*0.166, 0.5, -1*0.5, 0.166,
                    0.5, -1, 0.5, 0,
                    -1*0.5, 0, 0.5, 0,
                0.166, 0.6666, 0.166, 0;
            }
    else if (catmull == 1){
            ::m << -0.5, 2-0.5, 0.5-2, 0.5,
                2*0.5, 0.5-3, 3-(2*0.5), -1*0.5,
                -1*0.5, 0, 0.5, 0,
                 0,1,0,0;
            }
    
    }

//converts eigen matrix to a 1D array to use with GlutLoadMatrix
void matrixToArray(Eigen::Matrix4f &m, GLfloat rotationMatrix[]) {
    int count = 0;
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            rotationMatrix[count++] = m.coeffRef(j, i);
        }
    }
}



void createKey(){
    
       key newKey;
       newKey.x = ::x;
       newKey.y = ::y;
       newKey.z = ::z;
       newKey.Angles[0] = eAngles[0];
       newKey.Angles[1] = eAngles[1];
       newKey.Angles[2] = eAngles[2];
    
         //newKey.z ::z;
         vectorKeys.push_back(newKey);
  
}

void displayKeys(){
    for(int i=0;i<vectorKeys.size(); i++){
        glPushMatrix();
        if(useQ != 1){
             rotateEuler(vectorKeys[i].Angles, vectorKeys[i].x, vectorKeys[i].y, vectorKeys[i].z);
        }else{
            rotateQuat(vectorKeys[i].Angles, vectorKeys[i].x, vectorKeys[i].y, vectorKeys[i].z);
        }
       
         glutWireCube(0.25);
    
        glPopMatrix();
        }

}



//================================
// render
//================================
void render( void ) {
  
	// clear buffer
	glClearColor (0, 0.0, 0.0, 0.0);
	glClearDepth (1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    

	// render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// light source attributes
	GLfloat LightAmbient[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightDiffuse[]	= { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat LightSpecular[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT , LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE , LightDiffuse );
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	// surface material attributes
	GLfloat material_Ka[]	= { 0.11f, 0.06f, 0.11f, 1.0f };
	GLfloat material_Kd[]	= { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[]	= { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[]	= { 0.1f , 0.0f , 0.1f , 1.0f };
	GLfloat material_Se		= 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT	, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE	, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR	, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION	, material_Ke);
	glMaterialf (GL_FRONT, GL_SHININESS	, material_Se);
    
    //get correct basis matrix for interpolation
    loadBasis();
    
	// modelview matrix
	glMatrixMode( GL_MODELVIEW ); //the current matrix is the one we want to interact with
    float m[16];
    //m is now the 4x4 identity matrix
    glGetFloatv(GL_MODELVIEW_MATRIX, m);
    
    
    

    glPushMatrix();
    
	glLoadIdentity(); //gets modelview matrix to original state
    
  
    if(useQ == 1){
         rotateQuat(eAngles, x,y,z);
    }else{
        rotateEuler(eAngles, x,y,z);
    }
    
    glutSolidTeapot(1);

       glPopMatrix();
    
    if(t > 1){
        t = 0;
        if(ii < (vectorKeys.size()-1)) ii++;
        else ii=3;
    }
       

    if(interpolateQ == 1 && useQ == 1){
        
        interpolateQuat();
       
    }else if(interpolateQ == 1 && useQ == 0){
        interpolateEuler();
    }

   
    displayKeys();
    
    

	

	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// swap back and front buffers
	glutSwapBuffers();
}

//================================
// keyboard input
//================================
void keyboard( unsigned char key, int x, int y ) {
    if(key == 27){
        exit(0);
    }
    

    if(key == 'w') ::y+=0.25;
    

    if(key == 'a') ::x-=0.25;
    if(key == 's') ::y-=0.25;
    if(key == 'd') ::x+=0.25;
   // if(key == 'z') ::z-=0.25;
    if(key == 32){
        createKey();
    }
    
    //start interpolation
    if(key == 'i'){
        //glutTimerFunc( 16, timer, 0 );
        ::t = 0;
        ::interpolateQ = 1;
        //cout << interpolateQ << endl;
    }
    
    //alternate which interpolation method is used
    if(key == 'c'){
        ::catmull = 1;
        ::bspline = 0;
    }
    
    if(key == 'b'){
        ::catmull = 0;
        ::bspline = 1;
    }
    
    //switch between quaternion and euler
    if(key == 'q') ::useQ = 1;
    if(key == 'e') ::useQ = 0;
    
    //move different axis with mouse
    if(key == 'z') ::angletoRotate = 0;
    if(key == 'y') ::angletoRotate = 1;
    if(key == 'x') ::angletoRotate = 2;
    
    //clear all keys and stop interpolation with DEL key
    if(key == 127){
        interpolateQ = 0;
        vectorKeys.clear();
        //TODO: clear out qvec
        
    }
    if(key == 'p'){
       
          cout << "object: " << "q.x " << q1.x << "q.y " << q1.y << "q.z " << q1.z << "q.w " << q1.w << endl;
         cout << "euler angles are: " << eAngles[2] << " " << eAngles[1] << " " << eAngles[0] << endl;
        cout << "coordinaes are " << ::x << "," << ::y << "," << ::z << endl;
    }
    if(key == 't'){
        ::t += 0.05;
    }
    
    
    
    
}

void specialKeyboard(int key, int x, int y){
    if(key == GLUT_KEY_UP) ::z+=0.25;
    if(key == GLUT_KEY_DOWN) ::z-=0.25;

    
}

void mouse(int x, int y){
    //cout << y << endl;
    GLfloat height = g_screenHeight;
    
    
   
    GLfloat anglePerCoor = 360/height;
    //cout << "angle per coordinate is: " << anglePerCoor << endl;
    GLfloat newAngle = y * anglePerCoor;
    if(newAngle > 360){
        newAngle -= 360;
    }
    eAngles[::angletoRotate] = newAngle;
    cout << "angle " << angletoRotate << "is now " << newAngle << endl;
    
    //cout << "The new angle is: " << newAngle << endl;
   
  
}






//================================
// reshape : update viewport and projection matrix when the window is resized
//================================
void reshape( int w, int h ) {
	// screen size
	g_screenWidth  = w;
	g_screenHeight = h;


	// viewport
	glViewport( 0, 0, (GLsizei)w, (GLsizei)h );
    //glViewport( 0, 0, w, h );

	// projection matrix
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective(45.0, (GLfloat)w/(GLfloat)h, 1.0, 2000.0);
}


//================================
// timer : triggered every 16ms ( about 60 frames per second )
//================================
void timer( int value ) {
	// increase frame index
	g_frameIndex++; //no functionality

	update();

	// render
	glutPostRedisplay(); //calls render function immadetely

	// reset timer
	// 16 ms per frame ( about 60 frames per second )
	//glutTimerFunc( 16, timer, 0 );
    glutTimerFunc( 16, timer, 0 );
}

//================================
// main
//================================
int main( int argc, char** argv ) {
	// create opengL window
	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH );
	glutInitWindowSize( 700, 700 );
	glutInitWindowPosition( 0, 0 );
	glutCreateWindow( argv[0] );
 //   glutSetKeyRepeat(GLUT_KEY_REPEAT_ON);

	// init
	init();

	// set callback functions
	glutDisplayFunc( render );
	glutReshapeFunc( reshape );
	glutKeyboardFunc( keyboard );
    glutSpecialFunc(specialKeyboard);
    glutMotionFunc(mouse);
    glutTimerFunc( 16, timer, 0 );
   
    
       

	// main loop
	glutMainLoop();
    
	return 0;
}
