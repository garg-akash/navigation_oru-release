#include <iostream>
#define _USE_MATH_DEFINES   
#include <math.h>
#include <algorithm>
#include <casadi/casadi.hpp>

#include "checkolp_mxmn.h"

bool cvd = false;
void oru_splt_dev(vector<double>& px, vector<double>& py, vector<double>& v_opt, vector<double>& omg_opt, vector<double>& p_opt
                , vector<double>& th_opt, int k, int Nsim, int N, int n_area, int discard, double h, vector<double> robo_start
                , double x1_ref[], double y1_ref[], double th1_ref[], double x2_ref[], double y2_ref[]
                , double th2_ref[], double p2_ref[], double v2_ref[], double omg2_ref[], float l_1, float b_1, float l_2, float b_2
                , float L, float d_rear, int flag1[], int ind_ob, vector<int> ind_or, int ind_left, double cons_A0[][4]
                , double cons_A1[][4], double cons_b[][4], double cons_th[][2], MX vel_pichla, MX omg_pichla, int Nb) {


  int nx = 4; //no of state variables
  int nu = 2; //no of control variables
  int ns = 4; //sides of polygon

  float Q = 0.25;
  float R = 0.5;
  float T = 0.25;
  // Time horizon for integrator
  double t0 = 0;
  double tf = h;
  // To get indexes at which overlap will be calculated
  vector<int> n_ar;
  int haha = 0;
  for(int i=0; i<n_area; ++i) {
    //n_ar[i] = i*N/n_area;
    n_ar.push_back(haha);
    haha += round(N/n_area);
  }
  // Differential states
  SX X = SX::sym("X"), Y = SX::sym("Y"), Th = SX::sym("Th"), Ph = SX::sym("Ph");
  SX S = SX::vertcat({X, Y, Th, Ph});

  // Control
  SX V = SX::sym("V");
  SX Om = SX::sym("Om");
  SX U = SX::vertcat({V, Om});

  // Differential equation
  SX ode = SX::vertcat({U(0)*cos(S(2)), U(0)*sin(S(2)), U(0)*tan(S(3))/L, U(1)});

  // DAE
  SXDict dae = {{"x", S}, {"p", U}, {"ode", ode}};
  string plugin;
  if (cvd) 
    plugin = "cvodes";
  else
    plugin = "rk";
  // Create an integrator (CVodes)
  Function F = integrator("integrator", plugin, dae, {{"t0", t0}, {"tf", tf}});

  // Function f_("f_", {S,U}, {ode});
  // SX ss0 = SX::sym("ss0",ns);
  // //MX ss = MX::sym("ss",ns);
  // SX uu = SX::sym("uu",nu);
  // //vector<MX> arg1 = {ss0,uu};
  // SX k1 = f_(SX::vertcat({ss0,uu}));
  // vector<SX> arg2 = {ss0+0.5*h*k1,uu};
  // SX k2 = f_(arg2);
  // vector<SX> arg3 = {ss0+0.5*h*k2,uu};
  // SX k3 = f_(arg3);
  // vector<SX> arg4 = {ss0+h*k3,uu};
  // SX k4 = f_(arg4);
  // SX ss = ss0 + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
  // //Function F_("F_", {ss0,uu}, {ss});

  MX s=robo_start;
  MX J = 0;
  vector<MX> w;
  // Variable bounds
  vector<double> lbw, ubw;
  // Initial guess
  vector<double> w_init;
  // Constraints
  vector<MX> g;
  // Constraint bounds
  vector<double> lbg, ubg;
  // Discrete variables
  vector<bool> discrete;
  vector<MX> d1(4*n_area);
  vector<MX> d2(4*n_area);
  vector<MX> d3(4*n_area);
  vector<MX> d4(4*n_area);

  vector<MX> Mx1(4*n_area);
  vector<MX> Mn1(4*n_area);
  vector<MX> Mx2(4*n_area);
  vector<MX> Mn2(4*n_area);

  //vector<int> track_u(6*4*4*n_area + nu*N,0);
  vector<int> track_u;
  int temp_s;
  int l = 0;
  for(int i=0; i<N; ++i) {
    cout<<"N number: "<<i<<endl;
    MX u = MX::sym("u",nu);
    w.push_back(u);   
    w_init.push_back(v2_ref[k+i]);
    w_init.push_back(omg2_ref[k+i]);   
    lbw.push_back(-1);
    ubw.push_back(1);
    lbw.push_back(-1);
    ubw.push_back(1);
    discrete.push_back(0);
    discrete.push_back(0);
    g.push_back( u(0) - vel_pichla );
    lbg.push_back(-0.3);
    ubg.push_back(0.3);
    g.push_back( u(1) - omg_pichla );
    lbg.push_back(-0.25); 
    ubg.push_back(0.25); //rad2deg(0.25) ~ 14.3
    for(int j=0; j<nu; ++j){
      //track_u[(vertcat(w)).size1()-j-1] = 1;
      track_u.push_back(1);
    }

    // Integrate
    s = F(MXDict{{"x0", s}, {"p", u}}).at("xf");
    //cout<<"Ref got: "<<x2_ref[k+i+1]<<" "<<y2_ref[k+i+1]<<endl;
    //cout<<"Inits got: "<<v2_ref[k+i]<<" "<<omg2_ref[k+i]<<endl;
    J += Q*( pow( (s(0) - x2_ref[k+i+1]), 2) + pow( (s(1) - y2_ref[k+i+1]), 2));
    // J += Q*( pow((s(0) - x2_ref[Nb]), 2) + pow((s(1) - y2_ref[Nb]), 2) );
    //J += Q*( pow((s(0) - x2_ref.back()), 2) + pow((s(1) - y2_ref.back()), 2));
    //J += T*( pow(u(0), 2) + pow(u(1), 2) );
    J += T*( pow( (u(0)-v2_ref[k+i]), 2) + pow( (u(1)-omg2_ref[k+i]), 2) );

    //if ( (i==(N-1)) && ((Nsim-k) <= N) ) {
    if ( (i==(N-1)) && (Nsim/2 < k) ) {
      cout<<"Entered!!!!!!!"<<endl;
      g.push_back(s(0));
      lbg.push_back(x2_ref[Nb]);
      ubg.push_back(x2_ref[Nb]); 

      g.push_back(s(1));
      lbg.push_back(y2_ref[Nb]);
      ubg.push_back(y2_ref[Nb]);

      g.push_back(s(2));
      lbg.push_back(th2_ref[Nb]);
      ubg.push_back(th2_ref[Nb]);

      g.push_back(s(3));
      lbg.push_back(p2_ref[Nb]);
      ubg.push_back(p2_ref[Nb]); 
    }

    vector<int>::iterator p;
    p = find (n_ar.begin(), n_ar.end(), i);
    if (p != n_ar.end() && k<0.5*Nsim) {
      if ( (flag1[i] == 1) && (ind_left > 0) ) {
        double** rect_1 = rectangle_plot(l_1,b_1,th1_ref[ind_ob],x1_ref[ind_ob] + d_rear*cos(th1_ref[ind_ob])
                          ,y1_ref[ind_ob] + d_rear*sin(th1_ref[ind_ob]));

        MX** rect_2 = rectangle_plot_cas( l_2,b_2,th2_ref[k+i+1],s(0) + d_rear*cos(th2_ref[k+i+1])
                          ,s(1) + d_rear*sin(th2_ref[k+i+1]) );   
        vector<MX> olp1;

        // Produce projects about axis of obstacle 1  
        cout<<"Obstacle1 axis............."<<endl;  
        for(int j=1; j<3; ++j) {
          MX x_orig = rect_1[j][0] - rect_1[j-1][0];
          MX y_orig = rect_1[j][1] - rect_1[j-1][1];

          MX x_ref = rect_1[j-1][0];
          MX y_ref = rect_1[j-1][1];

          MX x_rot = -y_orig;
          MX y_rot = x_orig;

          cout<<"l : "<<l<<endl;

          //TODO checkOLP_maxmin function
          temp_s = discrete.size();
          checkOLP_maxmin_1(olp1, d1, d2, d3, d4, Mx1, Mn1, Mx2, Mn2, w, w_init, lbw, ubw, discrete, g, lbg, ubg, rect_1, rect_2
                          , x_rot, y_rot, x_ref, y_ref, ns, l, j, l_1, b_1, l_2, b_2);
          for(int kk = 0; kk<(discrete.size()-temp_s); ++kk) 
            track_u.push_back(0);
        
          l += 1;
        }
        // Produce projects about axis of robot  
        cout<<"Robot axis............."<<endl;  
        for(int j=1; j<3; ++j) {
          MX x_orig = rect_2[j][0] - rect_2[j-1][0];
          MX y_orig = rect_2[j][1] - rect_2[j-1][1];

          MX x_ref = rect_2[j-1][0];
          MX y_ref = rect_2[j-1][1];

          MX x_rot = -y_orig;
          MX y_rot = x_orig;

          cout<<"l : "<<l<<endl;

          //TODO checkOLP_maxmin function
          temp_s = discrete.size();
          checkOLP_maxmin_2(olp1, d1, d2, d3, d4, Mx1, Mn1, Mx2, Mn2, w, w_init, lbw, ubw, discrete, g, lbg, ubg, rect_1, rect_2
                          , x_rot, y_rot, x_ref, y_ref, ns, l, j, l_1, b_1, l_2, b_2);
          for(int kk = 0; kk<(discrete.size()-temp_s); ++kk) 
            track_u.push_back(0);
          l += 1;
        }

        ind_left -= round(N/n_area);
        ind_ob += round(N/n_area);
        J += R*olp1[0]*olp1[1]*olp1[2]*olp1[3];
      }
    }

    for(int j=0; j<4; ++j) {
      g.push_back(cons_A0[k+i+1][j]*s(0) + cons_A1[k+i+1][j]*s(1));
      lbg.push_back(-inf);
      ubg.push_back(cons_b[k+i+1][j]);
      //cout<<"Cons got: "<<cons_A0[k+i+1][j]<<" "<<cons_A1[k+i+1][j]<<endl;
    }
    g.push_back(s(2));
    lbg.push_back(cons_th[k+i+1][0]);
    ubg.push_back(cons_th[k+i+1][1]);

    vel_pichla = u(0);
    omg_pichla = u(1);
  }

  cout<<"Number of optimization variables : "<<(vertcat(w)).size()<<endl;
  cout<<"Number of discrete : "<<discrete.size()<<endl;
  cout<<"Number of w_init : "<<w_init.size()<<endl;
  cout<<"Number of track_u : "<<track_u.size()<<endl;
  // cout<<"Here is track_u: "<<endl;
  // for(int i = 0; i<track_u.size(); ++i) 
  //   cout<<i<<" "<<track_u[i]<<endl;
   
  // Create the NLP
  MXDict nlp = {{"x", vertcat(w)}, {"f", J}, {"g", vertcat(g)}};
  // NLP solver options
  Dict solver_opts;
  // solver_opts["ipopt.tol"] = 1e-5;
  // solver_opts["ipopt.max_iter"] = 500;
  solver_opts["bonmin.algorithm"] = "B-BB";
  solver_opts["discrete"] = discrete;
  //solver_opts = {{"discrete", discrete}};
  Function solver = nlpsol("nlpsol", "bonmin", nlp, solver_opts);
  std::map<std::string, DM> arg, res;

  // Bounds and initial guess
  arg["lbx"] = lbw;
  arg["ubx"] = ubw;
  arg["lbg"] = lbg;
  arg["ubg"] = ubg;
  arg["x0"] = w_init;
    
  // Solve the problem
  res = solver(arg);

  // Optimal solution of the NLP
  DM w_opt = res.at("x");
  vector<double> ww_opt(res.at("x"));
  DM u_opt;
  vector<DM> uu_opt;
  cout<<"Size of w_opt: "<<w_opt.size()<<endl;
  // Get the optimal state variables
  for(int i = 0; i<track_u.size(); ++i) {
    if (track_u[i] == 1) {
      v_opt.push_back(ww_opt[i]);
      omg_opt.push_back(ww_opt[i+1]);
      //u_opt = w_opt; //TODO******************************
      uu_opt.push_back(w_opt(Slice(i,i+nu)));
      ++i;
    }
  }

  if(k == 0) {
    ofstream file1;
    string filename = "trajectory_results_k1.m";
    file1.open(filename.c_str());
    file1 << "% Results file from " __FILE__ << endl;
    file1 << "% Generated " __DATE__ " at " __TIME__ << endl;
    file1 << endl;
    file1 << "ww_opt = " << "[";
    for(int lol = 0; lol<ww_opt.size(); ++lol) 
      file1 << ww_opt[lol] << endl;
    file1 << "]";
    file1.close();
  }

  cout<< "v_opt = " << endl;
  for(int i=0; i<N; ++i) {
    cout<< v_opt[i] << endl;    
  }
  cout<< "omg_opt = " << endl;
  for(int i=0; i<N; ++i) {
    cout<< omg_opt[i] << endl;    
  } 
  cout<<"Size of uu_opt: "<<uu_opt.size()<<endl; //uu_opt is a N*2 matrix
  //vector<double> x_opt, y_opt, th_opt, p_opt;
  vector<vector<double>> s_opt(1, robo_start);
  DMDict Fk = F(DMDict{{"x0", s_opt.back()}, {"p", uu_opt.at(0) }});
  //cout<<"Fk is here "<<Fk.at("xf")<<endl;
  for(int k = 0; k<N; ++k){
    //cout<<s_opt[k]<<endl;
    DMDict Fk = F(DMDict{{"x0", s_opt.back()}, {"p", uu_opt.at(k) }});
    s_opt.push_back(vector<double>(Fk.at("xf")));
  }
  //cout<<"till here"<<s_opt<<endl;
  for(int i = 0; i<=N ; ++i){
    px.push_back(s_opt[i][0]);
    py.push_back(s_opt[i][1]);
    th_opt.push_back(s_opt[i][2]);
    p_opt.push_back(s_opt[i][3]);
  }
}