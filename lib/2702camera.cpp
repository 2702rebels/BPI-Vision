#include "2702camera.h"
#include <stdio.h>
#include <string.h>
#include "2702math.h"

#include <fstream>

using namespace std;
using namespace Rebels;

CamParm::CamParm() {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      m_grfMat[i][j] = 0.F;
    }
  }
  m_nURes = 320;
  m_nVRes = 180;
  m_fU = 190.F;
  m_fV = 190.F;
  m_fU0 = 160.F;
  m_fV0 = 90.F;
  m_fk1 = 0.F;
  m_fk2 = 0.F;
  m_fk3 = 0.F;
  m_fp1 = 0.F;
  m_fp2 = 0.F;
}

bool CamParm::Load(char const *psz) {
  char szTmp[128];
  ifstream flCam;

  flCam.open(psz);
  if (!flCam.is_open()) {
    printf("Cam File '%s' not found\n", psz);
    return false;
  }
  if (flCam.getline(szTmp, sizeof(szTmp)).eof()) {
    printf("Camfile '%s' - unable to get sensor resolution\n", psz);
    return false;
  }
  if (sscanf(szTmp, "%d %d", &m_nURes, &m_nVRes) != 2) {
    printf("Camfile '%s' - unable to parse sensor resolution\n", psz);
    return false;
  }
  for (int i = 0; i < 3; i++) {
    if (flCam.getline(szTmp, sizeof(szTmp)).eof()) {
      printf("Camfile '%s' - invalid line\n", psz);
      return false;
    }
    if (sscanf(szTmp, "%f %f %f", &m_grfMat[i][0], &m_grfMat[i][1],
               &m_grfMat[i][2]) != 3) {
      printf("Camfile '%s' - incorrect line\n", psz);
      return false;
    }
  }

  m_fU = m_grfMat[0][0];
  m_fV = m_grfMat[1][1];
  m_fU0 = m_grfMat[0][2];
  m_fV0 = m_grfMat[1][2];

  if (!flCam.getline(szTmp, sizeof(szTmp)).eof()) {
    if (sscanf(szTmp, "%f %f %f %f", &m_fk1, &m_fk2, &m_fp1, &m_fp2) != 4) {
      printf("Camfile '%s' - invalid distortion terms\n", psz);
      return false;
    }
  }
  if (!flCam.getline(szTmp, sizeof(szTmp)).eof()) {
    float fQ0;
    float fX;
    float fY;
    float fZ;

    if (sscanf(szTmp, "%f %f %f %f", &fQ0, &fX, &fY, &fZ) != 4) {
      printf("Camfile '%s' - invalid rotation terms\n", psz);
      return false;
    }
    m_camRot = Quaternion(fQ0, fX, fY, fZ);
  }
  if (!flCam.getline(szTmp, sizeof(szTmp)).eof()) {
    float fX;
    float fY;
    float fZ;

    if (sscanf(szTmp, "%f %f %f", &fX, &fY, &fZ) != 3) {
      printf("Camfile '%s' - invalid rotation terms\n", psz);
      return false;
    }
    m_camPos = Point3d(fX, fY, fZ);
  }
  return true;
}

bool CamParm::Save(char const *psz) {
  ofstream of;
  of.open(psz);
  if (!of.is_open()) {
    printf("Unable to open '%s' for output\n", psz);
    return false;
  }

  char szTmp[1024];
  sprintf(szTmp, "%d %d\n", m_nURes, m_nVRes);
  of << szTmp;
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      sprintf(szTmp, "%.7f%c", m_grfMat[r][c], (c == 2) ? '\n' : ' ');
      of << szTmp;
    }
  }
  sprintf(szTmp, "%.7f %.7f %.7f %.7f\n", m_fk1, m_fk2, m_fp1, m_fp2);
  of << szTmp;
  sprintf(szTmp, "%.7f %.7f %.7f %.7f\n", m_camRot.q0(), m_camRot.x(),
          m_camRot.y(), m_camRot.z());
  of << szTmp;
  sprintf(szTmp, "%.3f %.3f %.3f\n", m_camPos.x(), m_camPos.y(), m_camPos.z());
  of << szTmp;

  return true;
}

void CamParm::Undistort(float fU, float fV, float &fUndistU,
                        float &fUndistV) const {
  fUndistU = fU;
  fUndistV = fV;
  for (int iter = 0; iter < 5; iter++) {
    float r2 = fUndistU * fUndistU + fUndistV * fUndistV;  // radius squared
    float icdist = 1.F / (1.F + ((m_fk3 * r2 + m_fk2) * r2 + m_fk1) * r2);
    float deltaX = 2.F * m_fp1 * fUndistU * fUndistV +
                   m_fp2 * (r2 + 2.F * fUndistU * fUndistU);
    float deltaY = m_fp1 * (r2 + 2.F * fUndistV * fUndistV) +
                   2.F * m_fp2 * fUndistU * fUndistV;
    fUndistU = (fU - deltaX) * icdist;
    fUndistV = (fV - deltaY) * icdist;
  }
}

void CamParm::UndistortPixels(float fU, float fV, float &fUndistU, float &fUndistV) const
{
	Undistort( (fU - m_fU0)/m_fU, (fV - m_fV0)/m_fV, fUndistU, fUndistV );
	fUndistU = fUndistU * m_fU + m_fU0;
	fUndistV = fUndistV * m_fV + m_fV0;
}

void CamParm::UpdateResolution(int nURes, int nVRes) {
  float fURatio = (float)m_nURes / (float)nURes;
  float fVRatio = (float)m_nVRes / (float)nVRes;

  m_fU /= fURatio;
  m_fV /= fVRatio;
  m_fU0 /= fURatio;
  m_fV0 /= fVRatio;

  m_grfMat[0][0] = m_fU;
  m_grfMat[1][1] = m_fV;
  m_grfMat[0][2] = m_fU0;
  m_grfMat[1][2] = m_fV0;

  m_nURes = nURes;
  m_nVRes = nVRes;
}

// Compute Line of Sight
Point3d CamParm::LOS(float fU, float fV) const {
  float fU1 = (fU - m_fU0) / m_fU;
  float fV1 = (fV - m_fV0) / m_fV;
  float fUUndist;
  float fVUndist;
  Undistort(fU1, fV1, fUUndist, fVUndist);
  Point3d lens(fUUndist, fVUndist, 1);
  lens.Normalize();
  Point3d res = m_camRot * lens;
  return res;
}

Point3d CamParm::CalcTargetPosFromHeight(float fU, float fV, float fZTarget) {
  //	float fTargetZ = 28.5F * 0.0254F;

  Point3d los = LOS(fU, fV);

  //						z = mt + z0; t = (z - z0)/m
  float fT = (fZTarget - m_camPos.z()) / los.z();
  float fX = los.x() * fT + m_camPos.x();
  float fY = los.y() * fT + m_camPos.y();
  float fZ = los.z() * fT + m_camPos.z();

  return Point3d(fX, fY, fZ);
}

CamParm &CamParm::operator=(CamParm const &src) {
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      m_grfMat[r][c] = src.m_grfMat[r][c];
    }
  }
  m_fU = src.m_fU;
  m_fV = src.m_fV;
  m_fU0 = src.m_fU0;
  m_fV0 = src.m_fV0;
  m_fk1 = src.m_fk1;
  m_fk2 = src.m_fk2;
  m_fk3 = src.m_fk3;
  m_fp1 = src.m_fp1;
  m_fp2 = src.m_fp2;
  m_nURes = src.m_nURes;
  m_nVRes = src.m_nVRes;

  m_camRot = src.m_camRot;
  m_camPos = src.m_camPos;
  return *this;
}

#include <ctype.h>
bool StereoCam::bLoad(char const *psz) {
  ifstream cfile;

  char szTmp[1024];
  sprintf(szTmp, "%s.sc", psz);
  cfile.open(szTmp);
  if (!cfile.is_open()) {
    printf("Unable to open Stereo Camera Config File '%s'\n", psz);
    return false;
  }
  for (int i = 0; i < 2; i++) {
    if (cfile.getline(szTmp, sizeof(szTmp)).eof()) {
      return false;
    }
    int nLen = strlen(szTmp);
    while (nLen > 0 && isspace(szTmp[nLen - 1])) nLen--;
    szTmp[nLen] = '\0';
    if (!m_grCams[i].Load(szTmp)) return false;
    if (cfile.getline(szTmp, sizeof(szTmp)).eof()) {
      return false;
    }
    float fQ0;
    float fX;
    float fY;
    float fZ;
    if (sscanf(szTmp, "%f,%f,%f,%f", &fQ0, &fX, &fY, &fZ) != 4) {
      return false;
    }
    Quaternion q(fQ0, fX, fY, fZ);
    m_grCams[i].SetRotation(q);
    if (cfile.getline(szTmp, sizeof(szTmp)).eof()) {
      return false;
    }
    if (sscanf(szTmp, "%f,%f,%f", &fX, &fY, &fZ) != 3) {
      return false;
    }
    Point3d t(fX, fY, fZ);
    m_grCams[i].SetOrigin(t);
  }

  if (cfile.getline(szTmp, sizeof(szTmp)).eof()) {
    return false;
  }
  float fQ0;
  float fX;
  float fY;
  float fZ;
  if (sscanf(szTmp, "%f,%f,%f,%f", &fQ0, &fX, &fY, &fZ) != 4) {
    return false;
  }
  m_CamToRobotRot = Quaternion(fQ0, fX, fY, fZ);
  if (cfile.getline(szTmp, sizeof(szTmp)).eof()) {
    return false;
  }
  if (sscanf(szTmp, "%f,%f,%f", &fX, &fY, &fZ) != 3) {
    return false;
  }
  m_CamToRobotXlat = Point3d(fX, fY, fZ);

  return true;
}

void StereoCam::SetCamXfrms(Quaternion const &q1, Point3d const &x1,
                            Quaternion const &q2, Point3d const &x2) {
  m_grCams[0].SetRotation(q1);
  m_grCams[0].SetOrigin(x1);
  m_grCams[1].SetRotation(q2);
  m_grCams[1].SetOrigin(x2);
}

bool StereoCam::bLoadCam(int nCam, char const *psz) {
  return m_grCams[nCam].Load(psz);
}

bool StereoCam::Save(char const *psz) {
  char szTmp[1024];

  ofstream of;

  sprintf(szTmp, "%s.sc", psz);
  of.open(szTmp);

  if (!of.is_open()) {
    return false;
  }
  for (int i = 0; i < 2; i++) {
    sprintf(szTmp, "%s-%d.cam", psz, i + 1);
    of << szTmp << endl;

    m_grCams[i].Save(szTmp);

    Quaternion const &r = m_grCams[i].getRotation();
    sprintf(szTmp, "%.7f,%.7f,%.7f,%.7f\n", r.q0(), r.x(), r.y(), r.z());
    of << szTmp;
    Point3d const &p = m_grCams[i].getOrigin();
    sprintf(szTmp, "%.7f,%.7f,%.7f\n", p.x(), p.y(), p.z());
    of << szTmp;
  }

  Quaternion const &r = m_CamToRobotRot;
  sprintf(szTmp, "%.7f,%.7f,%.7f,%.7f\n", r.q0(), r.x(), r.y(), r.z());
  of << szTmp;
  Point3d const &p = m_CamToRobotXlat;
  sprintf(szTmp, "%.7f,%.7f,%.7f\n", p.x(), p.y(), p.z());
  of << szTmp;

  return true;
}

void StereoCam::UpdateResolution(int nURes, int nVRes) {
  for (int i = 0; i < 2; i++) {
    m_grCams[i].UpdateResolution(nURes, nVRes);
  }
}

float StereoCam::fCalc3D(Point2d const &uv1, Point2d const &uv2, Point3d &res) {
  // from wikipedia - Skew Lines

  Point3d const &p1 = m_grCams[0].getOrigin();
  Point3d d1 = m_grCams[0].LOS(uv1.u(), uv1.v());

  Point3d const &p2 = m_grCams[1].getOrigin();
  Point3d d2 = m_grCams[1].LOS(uv2.u(), uv2.v());

  Point3d n2 = d2.CrossProduct(d1.CrossProduct(d2));
  Point3d c1 = p1 + d1 * ((p2 - p1).DotProduct(n2)) / (d1.DotProduct(n2));

  Point3d n1 = d1.CrossProduct(d2.CrossProduct(d1));
  Point3d c2 = p2 + d2 * ((p1 - p2).DotProduct(n1)) / (d2.DotProduct(n1));

  // c1 and c2 are the points on each line closest to the other line...
  // compute the vector between the two points.
  Point3d ls = c1 - c2;

  // the length of the vector is the line separation
  float fLineSep = ls.length();

  //  Take the mid point between the two lines as the 3D point
  Point3d mid = (c2 + c1) / 2;

  // rotate and translate into robot coordinate system.
//  res = m_CamToRobotRot * mid;
//  res += m_CamToRobotXlat;
  res = mid;

  return fLineSep;
}
