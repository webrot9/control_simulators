#include "bicycle.h"
#include <control_simulators/bicycle.h>
#include <control_simulators/utils/math_utils.h>

#include <cmath>

namespace {
    constexpr double inertial_ratio = 4.3333333333333333333; //=13.0 / 3.0;
    constexpr double g = 9.81;
}

// State has 9 variables:
//  {0,omega}: angle from vertical to the bicycle [rad]
//  {1,omegadot}: angular velocity, derivative of omega [rad/s]
//  {2,theta}: angle the handlebars are displaced from the normal [rad]
//  {3,thetadot}: angular velocity, derivative of theta [rad/s]
//  {4,psi}: angle formed by bicyle frame and x-axis [rad]
//  {5,xf}: x coordinate of front tire touching the ground
//  {6,yf}: y coordinate of front tire touching the ground
//  {7,xb}: x coordinate of back tire touching the ground
//  {8,yb}: y coordinate of back tire touching the ground
// 2-dimensional Controls:
//  {0,trq}: torque applied to the handlebar 
//  {1,disp}: displacement of the rider
ConsistentVector Bicycle::dynamics(double time,
        const ConsistentVector& state,
        const ConsistentVector &control) const {

    checkStateSize(state);
    checkControlSize(control);

    // These below could be cached
    const double Mc = param_[M_C]; const double Md = param_[M_D]; const double Mp = param_[M_P];
    const double L = param_[Bicycle::L]; const double Lsq = L*L;
    const double C = param_[Bicycle::C];
    const double Lc = L-C;
    const double Dcm = param_[D_CM];
    const double H = param_[Bicycle::H]; const double Hsq = H * H;
    const double R = param_[Bicycle::R]; const double Rsq = R * R;
    const double M = Mc + Md; // cycle + tire mass
    const double Idc = Md * Rsq;
    const double Idv = 1.5 * Idc;
    const double Idl = 0.5 * Idc;
    const double Itot = inertial_ratio * Mc * Hsq  + Mp * (H + Dcm) * (H + Dcm);
    const double V = param_[Bicycle::V]; const double Vsq = V*V;
    const double sigmadot = V/R;

    // extract parts of the control
    const double trq = control(0); // handlbar torque
    const double disp = control(1); // displacement rider

    // extract parts of the state
    const double omega = state(0);
    const double omegadot = state(1);
    const double theta = state(2);
    const double thetadot = state(3);
    const double psi = state(4);

    // Compute some intermediate values
    double rf  = 1e8; // handle divide by zero on theta
    double rb  = 1e8;
    double rCM = 1e8;
    if (std::abs(theta) > 1e-12) {
        const double tan_theta = tan(theta);
        rf = L / std::abs(sin(theta));
        rb = L / std::abs(tan_theta);
        rCM = sqrt(Lc * Lc + Lsq/(tan_theta*tan_theta));
    } 

    const double phi = omega + atan(disp/H);

    // Compute d/dt (omegadot) (second derivative)
    const double omegaddot = 1.0/Itot * (M * H * g * sin(phi) 
            - cos(phi) * (Idc * sigmadot * thetadot + sgn(theta) * Vsq 
                * (Md * R * (1.0 / rf + 1.0 / rb) + M * H / rCM)));

    const double thetaddot = (trq - Idv * sigmadot * omegadot) / Idl;


    // We don't do the asin(v*dt/2*rf) since this effect is handled by the changing theta during the
    // numerical integration step (omegaddot is a function of rf, omegadot is a function of
    // omegaddot integrated, thetaddot is a function of omegadot, and then theta is a function of thetaddot, 
    // psidot is also a function of rb)
    const double front_wheel_angle = psi + theta;
    const double xfdot = V*(-sin(front_wheel_angle)); 
    const double yfdot = V*cos(front_wheel_angle); 
    const double xbdot = V*(-sin(psi)); 
    const double ybdot = V*cos(psi); 
    
    //psi is the angle from the x-axis: psi = atan((xb-xf)/(yf-yb))
    const double psidot = sgn(theta)*V*(1.0/rb);

    ConsistentVector statedot(state.size());
    statedot(0) = omegadot; // new omegdot is old omegadot
    statedot(1) = omegaddot; 
    statedot(2) = thetadot; // new thetadot is old thetadot
    statedot(3) = thetaddot; 
    statedot(4) = psidot; 
    statedot(5) = xfdot; 
    statedot(6) = yfdot; 
    statedot(7) = xbdot; 
    statedot(8) = ybdot; 

    return statedot;
}


