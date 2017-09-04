#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "Context.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

template<class Vec>
void MapToCarSpace(Vec &c_x, Vec &c_y, const Vec &m_x, const Vec &m_y, double psi, double tx, double ty) {

    // To get to Car space : do the Inverse of counter-clockwise transformation from car to map space.
    // x += tx
    // y += ty
    // |x'| =| cos(a) sin(a)| |x|
    // |y'|=|-sin(a) cos(a)|*|y|

    assert(m_x.size() == m_y.size());

    c_x.resize(m_x.size());
    c_y.resize(m_y.size());

    const auto cosa = cos(psi);
    const auto sina = sin(psi);

    for (int i = 0; i < c_x.size(); i++)
    {
        const auto x = m_x[i] - tx;
        const auto y = m_y[i] - ty;
        c_x[i] = x * cosa + y * sina;
        c_y[i] = -x * sina + y * cosa;
    }
}


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
      } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
      }
    return "";
  }

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
      }
    return result;
  }

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
      }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
          }
      }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
  }

const double Lf = 2.67;

Eigen::VectorXd identifyCoeffs(vector<double>ptsx, vector<double> ptsy);

int main(int argc, const char *argv[]) {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;
    Context context ;

    if (argc == 10 ) {

        context.N = strtod (argv[1], NULL);
        context.dt = strtod (argv[2], NULL);
        context.ref_v = strtod (argv[3], NULL);

        context.ref_cte = strtod (argv[4], NULL);
        context.ref_epsi = strtod (argv[5], NULL);
        context.Lf = strtod (argv[6], NULL);

        //contstrain limits

        context.nactuators_limit = strtod (argv[7], NULL);
        context.delta_limit = strtod (argv[8], NULL);
        context.acc_limit = strtod (argv[9], NULL);

    } else {
        cout << "Usage ./mpc N dt ref_v ref_cte ref_epsi Lf" << endl;
        return -1;
    }
    h.onMessage([&mpc, context](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {

                    // j[1] is the data JSON object, specified in the Map space.
                    const vector<double> ms_ptsx = j[1]["ptsx"];  // Waypoints (x, y) in
                    const vector<double> ms_ptsy = j[1]["ptsy"];  //  the Map space.
                    const double ms_px  = j[1]["x"];  // position x of the car
                    const double ms_py  = j[1]["y"];  // position y of the car
                    const double ms_psi = j[1]["psi"]; // orientation of the car
                    const double v   = j[1]["speed"];  // velocity of the car

                    vector<double> ptsx;
                    vector<double> ptsy;
                    // Convert Map coordinate to Car space/coordinates
                    MapToCarSpace(ptsx, ptsy, ms_ptsx, ms_ptsy, ms_psi, ms_px, ms_py);
                    Eigen::VectorXd coeffs = identifyCoeffs (ptsx, ptsy);

                    // The distance between the car and the center of the road
                    double cte =  polyeval(coeffs, 0);
                    double epsi = -atan(coeffs[1]); // a simplification, since psi=0 and px=0

                    double delta = j[1]["steering_angle"];
                    double a = j[1]["throttle"];
                    double dt = 0.1; // accounting for 100ms of latency

                    double current_px = v*dt;
                    double current_py = 0.0;
                    double current_psi = v * (-delta)/Lf * dt;
                    double current_v = v + a*dt;
                    double current_cte = cte + v*sin(epsi) * dt;
                    double current_epsi = epsi + v * (-delta)/Lf * dt;

                    // save state
                    Eigen::VectorXd state(6);
                    state<< current_px, current_py, current_psi, 
                                current_v, current_cte, current_epsi;


                    auto vars = mpc.Solve(state, coeffs, context);


                    vector<double> next_x_vals;
                    vector<double> next_y_vals;


                    double poly_inc = 2.5;
                    int num_points = 25;
                    for (int i = 1; i < num_points; ++i)
                    {
                      next_x_vals.push_back(poly_inc*i);
                      next_y_vals.push_back(polyeval(coeffs, poly_inc*i));
                    }

                    //Display the MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;

                    for (unsigned int i = 2; i < vars.size(); ++i)
                    {
                      if (i%2 == 0)
                      {
                        mpc_x_vals.push_back(vars[i]);
                      }
                      else
                      {
                        mpc_y_vals.push_back(vars[i]);
                      }
                    }

                    json msgJson;
                    // Divide steering angle by deg2rad(25) to constrain between [-1, 1].
                    msgJson["steering_angle"] = -vars[0]/(deg2rad(25)*Lf);
                    msgJson["throttle"] = vars[1];

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;

                    this_thread::sleep_for(chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                  }
              } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              }
          }
      });


    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                    size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
          } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
          }
      });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
      });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                      char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
      });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
      } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
      }
    h.run();
  }

Eigen::VectorXd identifyCoeffs(vector<double>ptsx, vector<double> ptsy) {

    Eigen::VectorXd coeffs;
    Eigen::VectorXd x (ptsx.size());
    Eigen::VectorXd y (ptsy.size());
    for (size_t i = 0; i < ptsx.size(); i++) {
        x(i) = ptsx[i];
        y(i) = ptsy[i];
    }
    coeffs = polyfit(x, y, 3);
    // polyfit() to fit a third order polynomial to the (x, y) coordinates.
    return coeffs ;
}



