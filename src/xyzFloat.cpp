/* defines the structure vector3_u which is used for gyroscopes, accelerometers and 
   magnetometers such as ICM20948, MPU9250, ADXL345, etc. */

   #include "xyzFloat.h"

   vector3_u::vector3_u()
       : vector3_u(0.f, 0.f, 0.f)
   {
       // intentionally empty
   }
   
   vector3_u::vector3_u(float const x, float const y, float const z)
       : x(x)
       , y(y)
       , z(z)
   {
       // intentionally empty
   }
   
   vector3_u vector3_u::operator+() const
   {
       return *this;
   }
   
   vector3_u vector3_u::operator-() const
   {
       return vector3_u{-x,
                       -y,
                       -z};
   }
   
   vector3_u vector3_u::operator+(vector3_u const & summand) const
   {
       return vector3_u{x + summand.x,
                       y + summand.y,
                       z + summand.z};
   }
   
   vector3_u vector3_u::operator-(vector3_u const & subtrahend) const
   {
       return vector3_u{x - subtrahend.x,
                       y - subtrahend.y,
                       z - subtrahend.z};
   }
   
   vector3_u vector3_u::operator*(float const operand) const
   {
       return vector3_u{x * operand,
                       y * operand,
                       z * operand};
   }
   
   vector3_u vector3_u::operator*(vector3_u const & operand) const
   {
       return vector3_u{x * operand.x,
                       y * operand.y,
                       z * operand.z};
   }
   
   vector3_u vector3_u::operator/(float const divisor) const
   {
       return vector3_u{x / divisor,
                       y / divisor,
                       z / divisor};
   }
   
   vector3_u & vector3_u::operator+=(vector3_u const & summand)
   {
       x += summand.x;
       y += summand.y;
       z += summand.z;
       return *this;
   }
   
   vector3_u & vector3_u::operator-=(vector3_u const & subtrahend)
   {
       x -= subtrahend.x;
       y -= subtrahend.y;
       z -= subtrahend.z;
       return *this;
   }
   
   vector3_u & vector3_u::operator*=(vector3_u const & operand)
   {
       x *= operand.x;
       y *= operand.y;
       z *= operand.z;
       return *this;
   }
   
   vector3_u & vector3_u::operator*=(float const operand)
   {
       x *= operand;
       y *= operand;
       z *= operand;
       return *this;
   }
   
   vector3_u & vector3_u::operator/=(float const divisor)
   {
       x /= divisor;
       y /= divisor;
       z /= divisor;
       return *this;
   }