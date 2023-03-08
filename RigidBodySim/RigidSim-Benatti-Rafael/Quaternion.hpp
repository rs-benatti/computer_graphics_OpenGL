#include "Vector3.hpp"
#include "Matrix3x3.hpp"

class Quaternion{
    public:
        tReal _s;
        tReal _x;
        tReal _y;
        tReal _z;
        Quaternion(tReal s, tReal x, tReal y, tReal z){
           _s = s;
           _x = x;
           _y = y;
           _z = z; 
        }
        ~Quaternion(){}

        Quaternion operator*(Quaternion const &quatern) const{
            return Quaternion(
                this->_s*quatern._s - this->_x*quatern._x - this->_y*quatern._y - this->_z*quatern._z,
                this->_s*quatern._x + this->_x*quatern._s + this->_y*quatern._z - this->_z*quatern._y,
                this->_s*quatern._y - this->_x*quatern._z + this->_y*quatern._s + this->_z*quatern._x,
                this->_s*quatern._z + this->_x*quatern._y - this->_y*quatern._x + this->_z*quatern._s
            );
        }

        Quaternion operator*(tReal const &p) const{
            return Quaternion(p*this->_s, p*this->_x, p*this->_y, p*this->_z);
        }

        Quaternion operator+(Quaternion const &quatern)const{
            return Quaternion(this->_s+quatern._s, this->_x+quatern._x, this->_y+quatern._y, this->_z+quatern._z);
  }

        Quaternion normalize(){
            *this = *this * (1.0/std::sqrt(this->dotProduct(*this)));
            return Quaternion(*this);
        }

        float dotProduct(Quaternion &quatern) const{
            return this->_s*quatern._s + this->_x*quatern._x + this->_y*quatern._y + this->_z*quatern._z;
        }

        Mat3f getRotationMatrix(){
            return Mat3f(
                1.0-2.0*(this->_y*this->_y + this->_z*this->_z),
                2.0*  (this->_x*this->_y - this->_s*this->_z),
                2.0*  (this->_x*this->_z + this->_s*this->_y),
                2.0*  (this->_x*this->_y + this->_s*this->_z),
                1.0-2.0*(this->_x*this->_x + this->_z*this->_z),
                2.0*  (this->_y*this->_z - this->_s*this->_x),
                2.0*  (this->_x*this->_z - this->_s*this->_y),
                2.0*  (this->_y*this->_z + this->_s*this->_x),
                1.0-2.f*(this->_x*this->_x + this->_y*this->_y));
            }


};