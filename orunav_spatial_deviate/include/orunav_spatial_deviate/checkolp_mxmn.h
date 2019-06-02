#include <iostream>
#include <string.h>
#define _USE_MATH_DEFINES   
#include <math.h>
#include <algorithm>
#include <casadi/casadi.hpp>

using namespace casadi;
using namespace std;

void checkOLP_maxmin_1( vector<MX>& olp1_r, vector<MX>& d1_r, vector<MX>& d2_r, vector<MX>& d3_r, vector<MX>& d4_r 
                    , vector<MX>& mx1_r, vector<MX>& mn1_r, vector<MX>& mx2_r, vector<MX>& mn2_r
                    , vector<MX>& w_r, vector<double>& w_init_r, vector<double>& lbw_r, vector<double>& ubw_r
                    , vector<bool>& discrete_r, vector<MX>& g_r, vector<double>& lbg_r, vector<double>& ubg_r
                    , double** r1, MX** r2, MX x_rot, MX y_rot, MX x_ref, MX y_ref, int ns, int l, int j
                    , float l_1, float b_1, float l_2, float b_2) {

  int BIG = 10000;
  vector<MX> side1, side2;
  for(int i=0; i<ns; ++i) {
    MX inp1 = (x_rot * (r1[i][0] - x_ref) + y_rot * (r1[i][1] - y_ref))/sqrt( pow(x_rot,2) + pow(y_rot,2) );
    side1.push_back(inp1);

    MX inp2 = (x_rot * (r2[i][0] - x_ref) + y_rot * (r2[i][1] - y_ref))/sqrt( pow(x_rot,2) + pow(y_rot,2) );
    side2.push_back(inp2);
  } 

  MX d1 = MX::sym("d1" + str(l));
  MX d2 = MX::sym("d2" + str(l));
  MX d3 = MX::sym("d3" + str(l));
  MX d4 = MX::sym("d4" + str(l));
  double mx1, mn1;
    if (j==1) {
      mx1 = l_1;
      mn1 = 0;
    }
    else if (j==2) {
      mx1 = b_1;
      mn1 = 0;
    }
    for(int i=0; i<ns; ++i) {
      MX inp2 = (x_rot * (r2[i][0] - x_ref) + y_rot * (r2[i][1] - y_ref))/sqrt( pow(x_rot,2) + pow(y_rot,2) );
      side2.push_back(inp2);
    } 
    MX mx2 = MX::sym("mx2" + str(l));
    MX mn2 = MX::sym("mn2" + str(l));
    MX dmx2 = MX::sym("dmx2" + str(l),ns);
    MX dmn2 = MX::sym("dmn2" + str(l),ns);

      //To find max of side2
  w_r.push_back(mx2);
  lbw_r.push_back(-inf);
  ubw_r.push_back(inf);
  w_init_r.push_back(0);
  discrete_r.push_back(0);

  w_r.push_back(dmx2);
  for(int i=0; i<ns; ++i) {
    
    lbw_r.push_back(0);
    ubw_r.push_back(1);
    w_init_r.push_back(0);
    discrete_r.push_back(1);
  }

  for(int i=0; i<ns; ++i) {
    g_r.push_back( mx2 - side2.at(i) );
    lbg_r.push_back(0);
    ubg_r.push_back(inf);

    g_r.push_back( mx2 - side2.at(i) - BIG*(1-dmx2(i)) );
    lbg_r.push_back(-inf);
    ubg_r.push_back(0);
  }

  g_r.push_back( dmx2(0) + dmx2(1) + dmx2(2) + dmx2(3) );
  lbg_r.push_back(1);
  ubg_r.push_back(1);

  //To find min of side2
  w_r.push_back(mn2);
  lbw_r.push_back(-inf);
  ubw_r.push_back(inf);
  w_init_r.push_back(0);
  discrete_r.push_back(0);

  w_r.push_back(dmn2);
  for(int i=0; i<ns; ++i) {
    
    lbw_r.push_back(0);
    ubw_r.push_back(1);
    w_init_r.push_back(0);
    discrete_r.push_back(1);
  }

  for(int i=0; i<ns; ++i) {
    g_r.push_back( mn2 - side2.at(i) );
    lbg_r.push_back(-inf);
    ubg_r.push_back(0);

    g_r.push_back( mn2 - side2.at(i) + BIG*(1-dmn2(i)) );
    lbg_r.push_back(0);
    ubg_r.push_back(inf);
  }

  g_r.push_back( dmn2(0) + dmn2(1) + dmn2(2) + dmn2(3) );
  lbg_r.push_back(1);
  ubg_r.push_back(1);

  //Overlap calculation
  w_r.push_back(d1);
  lbw_r.push_back(0);
  ubw_r.push_back(1);
  w_init_r.push_back(0);
  discrete_r.push_back(1);
  g_r.push_back(mn2-mx1-BIG*d1);
  g_r.push_back(mn2-mx1+BIG*(1-d1));
  lbg_r.push_back(-inf);
  lbg_r.push_back(0);
  ubg_r.push_back(0);
  ubg_r.push_back(inf);

  w_r.push_back(d2);
  lbw_r.push_back(0);
  ubw_r.push_back(1);
  w_init_r.push_back(0);
  discrete_r.push_back(1);
  g_r.push_back(mn1-mx2-BIG*d2);
  g_r.push_back(mn1-mx2+BIG*(1-d2));
  lbg_r.push_back(-inf);
  lbg_r.push_back(0);
  ubg_r.push_back(0);
  ubg_r.push_back(inf);

  w_r.push_back(d3);
  lbw_r.push_back(0);
  ubw_r.push_back(1);
  w_init_r.push_back(0);
  discrete_r.push_back(1);
  g_r.push_back(mx1-mx2-BIG*d3);
  g_r.push_back(mx1-mx2+BIG*(1-d3));
  lbg_r.push_back(-inf);
  lbg_r.push_back(0);
  ubg_r.push_back(0);
  ubg_r.push_back(inf);

  w_r.push_back(d4);
  lbw_r.push_back(0);
  ubw_r.push_back(1);
  w_init_r.push_back(0);
  discrete_r.push_back(1);
  g_r.push_back(mn1-mn2-BIG*d4);
  g_r.push_back(mn1-mn2+BIG*(1-d4));
  lbg_r.push_back(-inf);
  lbg_r.push_back(0);
  ubg_r.push_back(0);
  ubg_r.push_back(inf);

  olp1_r.push_back( (((mx2 - mn2)*(1-d4) + (mx2-mn1)*d4)*d3 + 
                    ((mx1 - mn2)*(1-d4) + (mx1 - mn1)*d4)*(1-d3))*(1-d1)*(1-d2) );
}



void checkOLP_maxmin_2( vector<MX>& olp1_r, vector<MX>& d1_r, vector<MX>& d2_r, vector<MX>& d3_r, vector<MX>& d4_r 
                    , vector<MX>& mx1_r, vector<MX>& mn1_r, vector<MX>& mx2_r, vector<MX>& mn2_r
                    , vector<MX>& w_r, vector<double>& w_init_r, vector<double>& lbw_r, vector<double>& ubw_r
                    , vector<bool>& discrete_r, vector<MX>& g_r, vector<double>& lbg_r, vector<double>& ubg_r
                    , double** r1, MX** r2, MX x_rot, MX y_rot, MX x_ref, MX y_ref, int ns, int l, int j
                    , float l_1, float b_1, float l_2, float b_2) {

  int BIG = 10000;
  vector<MX> side1, side2;
  for(int i=0; i<ns; ++i) {
    MX inp1 = (x_rot * (r1[i][0] - x_ref) + y_rot * (r1[i][1] - y_ref))/sqrt( pow(x_rot,2) + pow(y_rot,2) );
    side1.push_back(inp1);

    MX inp2 = (x_rot * (r2[i][0] - x_ref) + y_rot * (r2[i][1] - y_ref))/sqrt( pow(x_rot,2) + pow(y_rot,2) );
    side2.push_back(inp2);
  } 

  MX d1 = MX::sym("d1" + str(l));
  MX d2 = MX::sym("d2" + str(l));
  MX d3 = MX::sym("d3" + str(l));
  MX d4 = MX::sym("d4" + str(l));
  double mx2, mn2;
    if (j==1) {
      mx2 = l_2;
      mn2 = 0;
    }
    else if (j==2) {
      mx2 = b_2;
      mn2 = 0;
    }
    for(int i=0; i<ns; ++i) {
      MX inp1 = (x_rot * (r1[i][0] - x_ref) + y_rot * (r1[i][1] - y_ref))/sqrt( pow(x_rot,2) + pow(y_rot,2) );
      side1.push_back(inp1);
    }
    MX mx1 = MX::sym("mx1" + str(l));
    MX mn1 = MX::sym("mn1" + str(l));
    MX dmx1 = MX::sym("dmx1" + str(l),ns);
    MX dmn1 = MX::sym("dmn1" + str(l),ns);

            //To find max of side1
  w_r.push_back(mx1);
  lbw_r.push_back(-inf);
  ubw_r.push_back(inf);
  w_init_r.push_back(0);
  discrete_r.push_back(0);

  w_r.push_back(dmx1);
  for(int i=0; i<ns; ++i) {
    
    lbw_r.push_back(0);
    ubw_r.push_back(1);
    w_init_r.push_back(0);
    discrete_r.push_back(1);
  }

  for(int i=0; i<ns; ++i) {
    g_r.push_back( mx1 - side1.at(i) );
    lbg_r.push_back(0);
    ubg_r.push_back(inf);

    g_r.push_back( mx1 - side1.at(i) - BIG*(1-dmx1(i)) );
    lbg_r.push_back(-inf);
    ubg_r.push_back(0);
  }

  g_r.push_back( dmx1(0) + dmx1(1) + dmx1(2) + dmx1(3) );
  lbg_r.push_back(1);
  ubg_r.push_back(1);

  //To find min of side1
  w_r.push_back(mn1);
  lbw_r.push_back(-inf);
  ubw_r.push_back(inf);
  w_init_r.push_back(0);
  discrete_r.push_back(0);

  w_r.push_back(dmn1);
  for(int i=0; i<ns; ++i) {
    
    lbw_r.push_back(0);
    ubw_r.push_back(1);
    w_init_r.push_back(0);
    discrete_r.push_back(1);
  }

  for(int i=0; i<ns; ++i) {
    g_r.push_back( mn1 - side1.at(i) );
    lbg_r.push_back(-inf);
    ubg_r.push_back(0);

    g_r.push_back( mn1 - side1.at(i) + BIG*(1-dmn1(i)) );
    lbg_r.push_back(0);
    ubg_r.push_back(inf);
  }

  g_r.push_back( dmn1(0) + dmn1(1) + dmn1(2) + dmn1(3) );
  lbg_r.push_back(1);
  ubg_r.push_back(1);

  //Overlap calculation
  w_r.push_back(d1);
  lbw_r.push_back(0);
  ubw_r.push_back(1);
  w_init_r.push_back(0);
  discrete_r.push_back(1);
  g_r.push_back(mn2-mx1-BIG*d1);
  g_r.push_back(mn2-mx1+BIG*(1-d1));
  lbg_r.push_back(-inf);
  lbg_r.push_back(0);
  ubg_r.push_back(0);
  ubg_r.push_back(inf);

  w_r.push_back(d2);
  lbw_r.push_back(0);
  ubw_r.push_back(1);
  w_init_r.push_back(0);
  discrete_r.push_back(1);
  g_r.push_back(mn1-mx2-BIG*d2);
  g_r.push_back(mn1-mx2+BIG*(1-d2));
  lbg_r.push_back(-inf);
  lbg_r.push_back(0);
  ubg_r.push_back(0);
  ubg_r.push_back(inf);

  w_r.push_back(d3);
  lbw_r.push_back(0);
  ubw_r.push_back(1);
  w_init_r.push_back(0);
  discrete_r.push_back(1);
  g_r.push_back(mx1-mx2-BIG*d3);
  g_r.push_back(mx1-mx2+BIG*(1-d3));
  lbg_r.push_back(-inf);
  lbg_r.push_back(0);
  ubg_r.push_back(0);
  ubg_r.push_back(inf);

  w_r.push_back(d4);
  lbw_r.push_back(0);
  ubw_r.push_back(1);
  w_init_r.push_back(0);
  discrete_r.push_back(1);
  g_r.push_back(mn1-mn2-BIG*d4);
  g_r.push_back(mn1-mn2+BIG*(1-d4));
  lbg_r.push_back(-inf);
  lbg_r.push_back(0);
  ubg_r.push_back(0);
  ubg_r.push_back(inf);

  olp1_r.push_back( (((mx2 - mn2)*(1-d4) + (mx2-mn1)*d4)*d3 + 
                    ((mx1 - mn2)*(1-d4) + (mx1 - mn1)*d4)*(1-d3))*(1-d1)*(1-d2) );
}