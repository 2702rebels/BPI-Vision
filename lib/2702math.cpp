#include "2702math.h"
#include <stdio.h>

using namespace Rebels;

Point3d::Point3d(Point3d const &src) { *this = src; }

Point3d &Point3d::operator=(Point3d const &other) {
  m_fX = other.m_fX;
  m_fY = other.m_fY;
  m_fZ = other.m_fZ;

  return *this;
}

Point3d Point3d::operator+(Point3d const &other) const {
  float x = m_fX + other.x();
  float y = m_fY + other.y();
  float z = m_fZ + other.z();
  return Point3d(x, y, z);
}

Point3d &Point3d::operator+=(Point3d const &other) {
  m_fX += other.x();
  m_fY += other.y();
  m_fZ += other.z();

  return *this;
}

Point3d &Point3d::operator-=(Point3d const &other) {
  m_fX -= other.x();
  m_fY -= other.y();
  m_fZ -= other.z();

  return *this;
}

Point3d Point3d::operator-(Point3d const &other) const {
  float x = m_fX - other.x();
  float y = m_fY - other.y();
  float z = m_fZ - other.z();
  return Point3d(x, y, z);
}

Point3d Point3d::operator*(float const scalar) const {
  Point3d ret(m_fX * scalar, m_fY * scalar, m_fZ * scalar);
  return ret;
}

Point3d &Point3d::operator*=(float const scalar) {
  m_fX *= scalar;
  m_fY *= scalar;
  m_fZ *= scalar;
  return *this;
}

Point3d Point3d::operator/(float const scalar) const {
  // todo: catch divide by 0
  Point3d ret(m_fX / scalar, m_fY / scalar, m_fZ / scalar);
  return ret;
}

Point3d &Point3d::operator/=(float const scalar) {
  // todo: catch divide by 0
  m_fX /= scalar;
  m_fY /= scalar;
  m_fZ /= scalar;
  return *this;
}

float Point3d::length() const {
  return sqrt(m_fX * m_fX + m_fY * m_fY + m_fZ * m_fZ);
}

Point3d Point3d::CrossProduct(Point3d const &v2) const {
  float fI = m_fY * v2.m_fZ - v2.m_fY * m_fZ;
  float fJ = m_fZ * v2.m_fX - v2.m_fZ * m_fX;
  float fK = m_fX * v2.m_fY - v2.m_fX * m_fY;

  return Point3d(fI, fJ, fK);
}

float Point3d::DotProduct(Point3d const &v2) const {
  return m_fX * v2.m_fX + m_fY * v2.m_fY + m_fZ * v2.m_fZ;
}

void Point3d::Normalize() {
  float fMag = sqrt(m_fX * m_fX + m_fY * m_fY + m_fZ * m_fZ);
  if (fMag == 0.F) {
    printf("Error - Point normalize->length = 0.\n");
    m_fX = 1.F;
    m_fY = 0.F;
    m_fZ = 0.F;
  } else {
    m_fX /= fMag;
    m_fY /= fMag;
    m_fZ /= fMag;
  }
}

#if 0
#if 1
RotationMatrix &RotationMatrix::operator=( Euler const &src )
{
	float fCosYaw = cos( src.m_fYaw );
	float fSinYaw = sin( src.m_fYaw );
	float fCosPitch = cos( src.m_fPitch );
	float fSinPitch = sin( src.m_fPitch );
	float fCosRoll = cos( src.m_fRoll );
	float fSinRoll = sin( src.m_fRoll );
	float fSYSP = fSinYaw * fSinPitch;
	float fCYSP = fCosYaw * fSinPitch;
		
	m_grfMat[0][0] = fCosPitch * fCosRoll;
	m_grfMat[0][1] = -fCosPitch * fSinRoll;
	m_grfMat[0][2] = -fSinPitch;
	m_grfMat[1][0] = -fSYSP * fCosRoll + fCosYaw * fSinRoll;
	m_grfMat[1][1] =  fSYSP * fSinRoll + fCosYaw * fCosRoll;
	m_grfMat[1][2] = -fSinYaw * fCosPitch;
	m_grfMat[2][0] = fCYSP * fCosRoll + fSinYaw * fSinRoll;
	m_grfMat[2][1] = -fCYSP * fSinRoll + fSinYaw * fCosRoll;
	m_grfMat[2][2] = fCosYaw * fCosPitch;

	return *this;
}
#else
// Rx = yaw
// Ry = pitch
// Rz = roll
//       | 1     0     0 |      | cosy 0 -siny |      | cosz -sinz 0 |
//  Rx = | 0  cosx -sinx | Ry = |    0 1     0 | Rz = | sinz  cosz 0 |
//       | 0  sinx  cosx |      | siny 0  cosy |      |    0     0 1 |

RotationMatrix &RotationMatrix::operator=( Euler const &src )
{
	float fCosYaw = cos( src.m_fYaw );
	float fSinYaw = sin( src.m_fYaw );
	float fCosPitch = cos( src.m_fPitch );
	float fSinPitch = sin( src.m_fPitch );
	float fCosRoll = cos( src.m_fRoll );
	float fSinRoll = sin( src.m_fRoll );
	float fSYSP = fSinYaw * fSinPitch;
	float fCYSP = fCosYaw * fSinPitch;
		
	m_grfMat[0][0] = fCosPitch * fCosRoll;
	m_grfMat[0][1] = fSYSP * fCosRoll - fSinRoll * fCosYaw;
	m_grfMat[0][2] = fCYSP * fCosRoll + fSinRoll * fSinYaw;
	m_grfMat[1][0] = fCosPitch * fSinRoll;
	m_grfMat[1][1] = fSYSP * fSinRoll + fCosYaw * fCosRoll;
	m_grfMat[1][2] = fCYSP * fSinRoll - fSinYaw * fCosRoll;
	m_grfMat[2][0] = -fSinPitch;
	m_grfMat[2][1] = fSinYaw * fCosPitch;
	m_grfMat[2][2] = fCosYaw * fCosPitch;

	return *this;
}
#endif
#endif

RotationMatrix &RotationMatrix::operator=(RotationMatrix const &src) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      m_grfMat[i][j] = src.m_grfMat[i][j];
    }
  }
  return *this;
}

RotationMatrix RotationMatrix::operator*(RotationMatrix const &right) {
  RotationMatrix res;

  for (int c = 0; c < 3; c++) {
    for (int r = 0; r < 3; r++) {
      res.m_grfMat[r][c] = m_grfMat[r][0] * right.m_grfMat[0][c] +
                           m_grfMat[r][1] * right.m_grfMat[1][c] +
                           m_grfMat[r][2] * right.m_grfMat[2][c];
    }
  }
  return res;
}

RotationMatrix &RotationMatrix::operator=(Quaternion const &quat) {
  float fQ0X = quat.m_fQ0 * quat.m_fX;
  float fQ0Y = quat.m_fQ0 * quat.m_fY;
  float fQ0Z = quat.m_fQ0 * quat.m_fZ;

  float fXX = quat.m_fX * quat.m_fX;
  float fXY = quat.m_fX * quat.m_fY;
  float fXZ = quat.m_fX * quat.m_fZ;

  float fYY = quat.m_fY * quat.m_fY;
  float fYZ = quat.m_fY * quat.m_fZ;

  float fZZ = quat.m_fZ * quat.m_fZ;

  m_grfMat[0][0] = 1.F - 2.F * (fYY + fZZ);
  m_grfMat[0][1] = 2.F * (fXY - fQ0Z);
  m_grfMat[0][2] = 2.F * (fXZ + fQ0Y);

  m_grfMat[1][0] = 2.F * (fXY + fQ0Z);
  m_grfMat[1][1] = 1.F - 2.F * (fXX + fZZ);
  m_grfMat[1][2] = 2.F * (fYZ - fQ0X);

  m_grfMat[2][0] = 2.F * (fXZ - fQ0Y);
  m_grfMat[2][1] = 2.F * (fYZ + fQ0X);
  m_grfMat[2][2] = 1.F - 2.F * (fXX + fYY);

  return *this;
}

Point3d RotationMatrix::operator*(Point3d const &pt) const {
  float fX = m_grfMat[0][0] * pt.x() + m_grfMat[0][1] * pt.y() +
             m_grfMat[0][2] * pt.z();
  float fY = m_grfMat[1][0] * pt.x() + m_grfMat[1][1] * pt.y() +
             m_grfMat[1][2] * pt.z();
  float fZ = m_grfMat[2][0] * pt.x() + m_grfMat[2][1] * pt.y() +
             m_grfMat[2][2] * pt.z();

  return Point3d(fX, fY, fZ);
}

Quaternion &Quaternion::operator=(Quaternion const &src) {
  m_fQ0 = src.m_fQ0;
  m_fX = src.m_fX;
  m_fY = src.m_fY;
  m_fZ = src.m_fZ;

  return *this;
}

Quaternion::Quaternion(float fTheta, Point3d const &axis) {
  Point3d normalizedAxis(axis);
  normalizedAxis.Normalize();

  float fTheta2 = fTheta / 2.F;
  float fSinTheta2 = sin(fTheta2);
  m_fQ0 = cos(fTheta2);
  m_fX = normalizedAxis.x() * fSinTheta2;
  m_fY = normalizedAxis.y() * fSinTheta2;
  m_fZ = normalizedAxis.z() * fSinTheta2;
  Normalize();
}

Quaternion &Quaternion::operator=(RotationMatrix const &rm) {
  float fT = rm.m_grfMat[0][0] + rm.m_grfMat[1][1] + rm.m_grfMat[2][2] + 1;
  if (fT > 0.00000001F) {
    float fS = sqrt(fT) * 2.F;
    m_fQ0 = 0.25F * fS;
    m_fX = (rm.m_grfMat[2][1] - rm.m_grfMat[1][2]) / fS;
    m_fY = (rm.m_grfMat[0][2] - rm.m_grfMat[2][0]) / fS;
    m_fZ = (rm.m_grfMat[1][0] - rm.m_grfMat[0][1]) / fS;
  } else if (rm.m_grfMat[0][0] > rm.m_grfMat[1][1] &&
             rm.m_grfMat[0][0] > rm.m_grfMat[2][2]) {
    float fS =
        sqrt(1.0F + rm.m_grfMat[0][0] - rm.m_grfMat[1][1] - rm.m_grfMat[2][2]) *
        2.F;
    m_fX = 0.25F * fS;
    m_fY = (rm.m_grfMat[1][0] + rm.m_grfMat[0][1]) / fS;
    m_fZ = (rm.m_grfMat[0][2] + rm.m_grfMat[2][0]) / fS;
    m_fQ0 = (rm.m_grfMat[2][1] - rm.m_grfMat[1][2]) / fS;
  } else if (rm.m_grfMat[1][1] > rm.m_grfMat[2][2]) {
    float fS =
        sqrt(1.0F + rm.m_grfMat[1][1] - rm.m_grfMat[0][0] - rm.m_grfMat[2][2]) *
        2.F;
    m_fX = (rm.m_grfMat[1][0] + rm.m_grfMat[0][1]) / fS;
    m_fY = 0.25F * fS;
    m_fZ = (rm.m_grfMat[2][1] + rm.m_grfMat[1][2]) / fS;
    m_fQ0 = (rm.m_grfMat[0][2] - rm.m_grfMat[2][0]) / fS;
  } else {
    float fS =
        sqrt(1.0F + rm.m_grfMat[2][2] - rm.m_grfMat[0][0] - rm.m_grfMat[1][1]) *
        2.F;
    m_fX = (rm.m_grfMat[0][2] + rm.m_grfMat[2][0]) / fS;
    m_fY = (rm.m_grfMat[2][1] + rm.m_grfMat[1][2]) / fS;
    m_fZ = 0.25F * fS;
    m_fQ0 = (rm.m_grfMat[1][0] - rm.m_grfMat[0][1]) / fS;
  }
  return *this;
}

Quaternion Quaternion::operator*(Quaternion const &other) {
  float fQ0 = m_fQ0 * other.m_fQ0 - m_fX * other.m_fX - m_fY * other.m_fY -
              m_fZ * other.m_fZ;
  float fX = m_fQ0 * other.m_fX + m_fX * other.m_fQ0 + m_fY * other.m_fZ -
             m_fZ * other.m_fY;
  float fY = m_fQ0 * other.m_fY + m_fY * other.m_fQ0 + m_fZ * other.m_fX -
             m_fX * other.m_fZ;
  float fZ = m_fQ0 * other.m_fZ + m_fZ * other.m_fQ0 + m_fX * other.m_fY -
             m_fY * other.m_fX;
  Quaternion res(fQ0, fX, fY, fZ);
  res.Normalize();
  return res;
}

void Quaternion::Normalize() {
  float fMag = sqrt(m_fQ0 * m_fQ0 + m_fX * m_fX + m_fY * m_fY + m_fZ * m_fZ);
  if (fMag == 0.F) {
    m_fQ0 = 1.0F;
    m_fX = 0.F;
    m_fY = 0.F;
    m_fZ = 0.F;
  } else {
    m_fQ0 /= fMag;
    m_fX /= fMag;
    m_fY /= fMag;
    m_fZ /= fMag;
  }
}

Point3d Quaternion::operator*(Point3d const &pt) const {
  float UxV_x = m_fY * pt.z() - m_fZ * pt.y();
  float UxV_y = m_fZ * pt.x() - m_fX * pt.z();
  float UxV_z = m_fX * pt.y() - m_fY * pt.x();

  Point3d res;
  res.m_fX = pt.x() + 2.F * (m_fQ0 * UxV_x + m_fY * UxV_z - m_fZ * UxV_y);
  res.m_fY = pt.y() + 2.F * (m_fQ0 * UxV_y + m_fZ * UxV_x - m_fX * UxV_z);
  res.m_fZ = pt.z() + 2.F * (m_fQ0 * UxV_z + m_fX * UxV_y - m_fY * UxV_x);

  return res;
}

Point2d::Point2d() : m_fU(0.F), m_fV(0.F) {}

Point2d::Point2d(float fU, float fV) : m_fU(fU), m_fV(fV) {}

Point2d::Point2d(Point2d const &src) : m_fU(src.m_fU), m_fV(src.m_fV) {}

Point2d &Point2d::operator=(Point2d const &src) {
  m_fU = src.m_fU;
  m_fV = src.m_fV;
  return *this;
}
