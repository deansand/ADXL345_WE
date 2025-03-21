/* defines the structure vector3_u which is used for gyroscopes, accelerometers and 
   magnetometers such as ICM209468, MPU9250, ADXL345, etc. */

   #ifndef XYZ_FLOAT_H_
   #define XYZ_FLOAT_H_
   #include <xyzFloat.h>
   struct vector3_u {
       float x;
       float y;
       float z;
   
       vector3_u();
       vector3_u(float const x, float const y, float const z);
   
       vector3_u operator+() const;
       vector3_u operator-() const;
       vector3_u operator+(vector3_u const & summand) const;
       vector3_u operator-(vector3_u const & subtrahend) const;
       vector3_u operator*(float const operand) const;
       vector3_u operator*(vector3_u const & operand) const;
       vector3_u operator/(float const divisor) const;
       vector3_u & operator+=(vector3_u const & summand);
       vector3_u & operator-=(vector3_u const & subtrahend);
       vector3_u & operator*=(vector3_u const & operand);
       vector3_u & operator*=(float const operand);
       vector3_u & operator/=(float const divisor);
   };
   #endif