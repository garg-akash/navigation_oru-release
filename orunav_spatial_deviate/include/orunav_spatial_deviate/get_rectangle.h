#include <iostream>
#define _USE_MATH_DEFINES   
#include <math.h>
#include <algorithm>
#include <casadi/casadi.hpp>

using namespace std;
using namespace casadi;

MX** rectangle_plot_cas(float l,float b,float theta,MX x_centre,MX y_centre) {
  MX** rect2d = 0;
  rect2d = new MX*[4];
  for (int h = 0; h < 4; h++) {
    rect2d[h] = new MX[2];
  }
  MX X_A,Y_A,X_B,Y_B,X_C,Y_C,X_D,Y_D,angle_A,angle_B,angle_C,angle_D;
  MX X_AG,Y_AG,X_BG,Y_BG,X_CG,Y_CG,X_DG,Y_DG;
  // co-ordinates of A:
  X_A = b/2; Y_A = l/2;
  angle_A=atan2(Y_A,X_A);
  // co-ordinates of B:
  X_B=-b/2;Y_B=l/2;
  angle_B=atan2(Y_B,X_B);
  // co-ordinates of C:
  X_C=-b/2;Y_C=-l/2;
  angle_C=atan2(Y_C,X_C);
  // co-ordinates of D:
  X_D=b/2;Y_D=-l/2;
  angle_D=atan2(Y_D,X_D);
  // tansform the co-ordinates to the global frame:
  double r=sqrt( pow(l/2,2) + pow(b/2,2) );
  X_AG=x_centre+r*cos(angle_A+theta-M_PI/2);
  Y_AG=y_centre+r*sin(angle_A+theta-M_PI/2);

  X_BG=x_centre+r*cos(angle_B+theta-M_PI/2);
  Y_BG=y_centre+r*sin(angle_B+theta-M_PI/2);

  X_CG=x_centre+r*cos(angle_C+theta-M_PI/2);
  Y_CG=y_centre+r*sin(angle_C+theta-M_PI/2);

  X_DG=x_centre+r*cos(angle_D+theta-M_PI/2);
  Y_DG=y_centre+r*sin(angle_D+theta-M_PI/2);  

  //return in clockwise direction
  rect2d[0][0] = X_AG; rect2d[0][1] = Y_AG; 
  rect2d[1][0] = X_BG; rect2d[1][1] = Y_BG; 
  rect2d[2][0] = X_CG; rect2d[2][1] = Y_CG; 
  rect2d[3][0] = X_DG; rect2d[3][1] = Y_DG; 

  return rect2d;
}


double** rectangle_plot(float l,float b,float theta,float x_centre,float y_centre) {
  double** rect2d = 0;
  rect2d = new double*[4];
  for (int h = 0; h < 4; h++) {
    rect2d[h] = new double[2];
  }
  double X_A,Y_A,X_B,Y_B,X_C,Y_C,X_D,Y_D,angle_A,angle_B,angle_C,angle_D;
  double X_AG,Y_AG,X_BG,Y_BG,X_CG,Y_CG,X_DG,Y_DG;
  // co-ordinates of A:
  X_A = b/2; Y_A = l/2;
  angle_A=atan2(Y_A,X_A);
  // co-ordinates of B:
  X_B=-b/2;Y_B=l/2;
  angle_B=atan2(Y_B,X_B);
  // co-ordinates of C:
  X_C=-b/2;Y_C=-l/2;
  angle_C=atan2(Y_C,X_C);
  // co-ordinates of D:
  X_D=b/2;Y_D=-l/2;
  angle_D=atan2(Y_D,X_D);
  // tansform the co-ordinates to the global frame:
  double r=sqrt( pow(l/2,2) + pow(b/2,2) );
  X_AG=x_centre+r*cos(angle_A+theta-M_PI/2);
  Y_AG=y_centre+r*sin(angle_A+theta-M_PI/2);

  X_BG=x_centre+r*cos(angle_B+theta-M_PI/2);
  Y_BG=y_centre+r*sin(angle_B+theta-M_PI/2);

  X_CG=x_centre+r*cos(angle_C+theta-M_PI/2);
  Y_CG=y_centre+r*sin(angle_C+theta-M_PI/2);

  X_DG=x_centre+r*cos(angle_D+theta-M_PI/2);
  Y_DG=y_centre+r*sin(angle_D+theta-M_PI/2);  

  //return in clockwise direction
  rect2d[0][0] = X_AG; rect2d[0][1] = Y_AG; 
  rect2d[1][0] = X_BG; rect2d[1][1] = Y_BG; 
  rect2d[2][0] = X_CG; rect2d[2][1] = Y_CG; 
  rect2d[3][0] = X_DG; rect2d[3][1] = Y_DG; 

  return rect2d;
}
