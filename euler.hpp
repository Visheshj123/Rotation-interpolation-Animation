//
//  euler.hpp
//  SimpleGlut
//
//  Created by Vishesh Javangula on 9/27/20.
//  Copyright © 2020 lab. All rights reserved.
//

#ifndef euler_hpp
#define euler_hpp
#include <iomanip>
#include <stdio.h>
#include <GLUT/GLUT.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "key.hpp"


extern int ii;
extern GLfloat t;
extern Eigen::Matrix4f m;
extern std::vector<key> vectorKeys;
extern void matrixToArray(Eigen::Matrix4f &m, GLfloat rotationMatrix[]);
void rotateEuler(GLfloat eAngles[3], GLfloat x, GLfloat y, GLfloat z);
void interpolateEuler();


#endif /* euler_hpp */
