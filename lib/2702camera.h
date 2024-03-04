#pragma once

#include "2702math.h"

namespace Rebels {

class CamParm {
  float m_grfMat[3][3];
  float m_fU;
  float m_fV;
  float m_fU0;
  float m_fV0;
  float m_fk1;  // radial distortion terms
  float m_fk2;
  float m_fk3;
  float m_fp1;  // tangential distortion terms;
  float m_fp2;
  int m_nURes;
  int m_nVRes;

  Quaternion m_camRot;
  Point3d m_camPos;

 public:
  CamParm();
  ~CamParm() {}
  float pinhole(int r, int c) const { return m_grfMat[r][c]; }
  float dist(int i) const {
    switch (i) {
      case 0:
        return m_fk1;
      case 1:
        return m_fk2;
      case 2:
        return m_fp1;
      case 3:
        return m_fp2;
      case 4:
        return m_fk3;
    }
    return 0.;
  }
  bool Load(char const *psz);
  bool Save(char const *psz);
  void SetRotation(Quaternion const &q) { m_camRot = q; }
  void SetOrigin(Point3d const &p) { m_camPos = p; }
  Quaternion const &getRotation() const { return m_camRot; }
  Point3d const &getOrigin() const { return m_camPos; }

  void UpdateResolution(int nURes, int nVRes);
  void Undistort(float fU, float fV, float &fUndistU, float &fUnistV) const;			// normalized by focal length
  void UndistortPixels(float fU, float fV, float &fUndistU, float &fUnistV) const;		// in Pixel units
  Point3d LOS(float fU, float fV) const;

  Point3d CalcTargetPosFromHeight(float fU, float fV, float fZ);

  CamParm &operator=(CamParm const &src);
};

class StereoCam {
  CamParm m_grCams[2];
  Quaternion m_CamToRobotRot;
  Point3d m_CamToRobotXlat;

 public:
  StereoCam() {}
  StereoCam(CamParm const &c1, CamParm const &c2) {
    m_grCams[0] = c1;
    m_grCams[1] = c2;
  }
  ~StereoCam() {}

  void SetCam(int nCam, CamParm const &c) { m_grCams[nCam] = c; }
  void SetCamToRobot(Quaternion const &q, Point3d const &x) {
    m_CamToRobotRot = q;
    m_CamToRobotXlat = x;
  }
  bool bLoadCam(int nCam, char const *psz);
  void SetCamXfrms(Quaternion const &q1, Point3d const &x1,
                   Quaternion const &q2, Point3d const &x2);
  bool bLoad(char const *psz);
  bool Save(char const *psz);
  void UpdateResolution(int nURes, int nVRes);
  float fCalc3D(Point2d const &p1, Point2d const &p2,
                Point3d &res);  // returns line separation
};
};  // namespace Rebels
