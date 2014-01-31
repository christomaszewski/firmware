#include "Platypus.h"

using namespace platypus;

Led::Led() 
: r_(0), g_(0), b_(0)
{  
  pinMode(board::LED.R, OUTPUT);
  digitalWrite(board::LED.R, HIGH);
  
  pinMode(board::LED.G, OUTPUT);
  digitalWrite(board::LED.G, HIGH);
  
  pinMode(board::LED.B, OUTPUT);
  digitalWrite(board::LED.B, HIGH);
}

Led::~Led()
{
  pinMode(board::LED.R, INPUT);
  pinMode(board::LED.G, INPUT);
  pinMode(board::LED.B, INPUT);
}

void Led::set(int red, int green, int blue)
{
  R(red);
  G(green);
  B(blue);
}

void Led::R(int red)
{
 r_ = red;
 digitalWrite(board::LED.R, !r_);
}

int Led::R()
{
 return r_;
}

void Led::G(int green)
{
 g_ = green; 
 digitalWrite(board::LED.G, !g_);
}

int Led::G()
{
 return g_;
}

void Led::B(int blue)
{
 b_ = blue; 
 digitalWrite(board::LED.B, !b_);
}

int Led::B()
{
 return b_;
}

Motor::Motor(int channel)
: enable_(board::MOTOR[channel].ENABLE), enabled_(false), velocity_(0)
{
  servo_.attach(board::MOTOR[channel].SERVO);
  pinMode(enable_, OUTPUT);
  digitalWrite(enable_, LOW);
}

Motor::~Motor()
{
  pinMode(enable_, INPUT);
  digitalWrite(enable_, LOW);
  servo_.detach();
}
    
void Motor::velocity(float velocity)
{
  if (velocity > 1.0) {
    velocity = 1.0;
  }
  if (velocity < -1.0) {
     velocity = -1.0; 
  }
  velocity_ = velocity;
  
  float command = (velocity * 500) + 1500;
  servo_.writeMicroseconds(command);
}

float Motor::velocity()
{
  return velocity_;
}

void Motor::enable(bool isOn)
{
  enabled_ = isOn;
  digitalWrite(enable_, enabled_);
}

bool Motor::enabled()
{
  return enabled_;
}
    
void Motor::enable()
{
  enable(true);
}

void Motor::disable()
{
  enable(false);  
}
    
float Motor::current()
{
 // TODO: fill me in. 
}

Sensor::Sensor(int channel)
{
  // TODO: fill me in
}

Sensor::~Sensor()
{
  // TODO: fill me in  
}

void VaporPro::arm()
{
  disable();
  delay(500);
  enable();
  
  velocity(1.0);
  delay(5500);

  velocity(-1.0);
  delay(3500);

  velocity(0.0);
  delay(8500);
}

void AnalogSensor::set(char* param, char* value)
{
  // TODO: fill me in
}

void ES2::set(char* param, char* value)
{
  // TODO: fill me in  
}
