#ifndef _BICYCLE_H_
#define _BICYCLE_H_

#include <control_simulators/interfaces/simulable.h>


// Implements the Randlov and Alstrom Bicycle model found on pages 8,9 of 
// Learning to Drive a Bicycle using Reinforcement Learning and Shaping 
// (http://robotics.usc.edu/~aatrash/COMP526/docs/randlv98learning.pdf)
//
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
class Bicycle: public Simulable {
 public:
  enum ParamIndex {
    C = 0, // Horizontal distance between front wheel ground contact and the center-of-mass (CM)
    D_CM,  // Vertical distance between CM and cyclist
    H,     // Height of CM over ground
    L,     // Distance between front and back tires at ground contact points
    M_C,   // Mass of cycle
    M_D,   // Mass of tire
    M_P,   // Mass of cyclist (person)
    R,     // Radius of tire
    V,     // Velocity of the bicyle
    END
  };

  // constructors
  explicit Bicycle(const ConsistentVector& state) : Simulable(state) {
    checkStateSize(state);
    // These are members of the parent class that must be set by the child
    param_.resize(END);
    param_[C]    = 0.66;  // Horizontal distance between front wheel ground contact and the center-of-mass (CM)
    param_[D_CM] = 0.30;  // Vertical distance between CM and cyclist
    param_[H]    = 0.94;  // Height of CM over ground
    param_[L]    = 1.11;  // Distance between front and back tires at ground contact points
    param_[M_C]  = 15;    // Mass of cycle
    param_[M_D]  = 1.7;   // Mass of tire
    param_[M_P]  = 60;    // Mass of cyclist (person)
    param_[R]    = 0.34;  // Radius of tire
    param_[V]    = 10 * 1000.0 / 3600.0;    // Velocity of the bicyle (10 km/h -> m/s)
    param_names_ = {{"c",0}, {"dcm",1}, {"h",2}, {"l",3}, {"mc",4}, {"md",5}, {"mp",6}, {"r",7}};
  }
  virtual ~Bicycle() {}

  // useful functions
  virtual ConsistentVector dynamics(double time,
				    const ConsistentVector& state,
				    const ConsistentVector &control) const override;
  
  // setters and getters
  virtual int stateSize() const override { return 9; }
  virtual int controlSize() const override { return 2; }
};

#endif // _BICYCLE_H_
