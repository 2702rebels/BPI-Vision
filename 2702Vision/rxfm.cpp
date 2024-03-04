// taken from https://stackoverflow.com/questions/21206870/opencv-rigid-transformation-between-two-3d-point-clouds
#include <iostream>
#include <opencv2/core.hpp>

using namespace std;

static cv::Vec3f CalculateMean(const cv::Mat_<cv::Vec3f> &points)
{
    cv::Mat_<cv::Vec3f> result;
    cv::reduce(points, result, 0, cv::REDUCE_AVG);
    return result(0, 0);
}

cv::Mat_<float> FindRigidTransform(const cv::Mat_<cv::Vec3f> &points1, const cv::Mat_<cv::Vec3f> points2)
{
    /* Calculate centroids. */
    cv::Vec3f t1 = -CalculateMean(points1);
    cv::Vec3f t2 = -CalculateMean(points2);

    cv::Mat_<float> T1 = cv::Mat_<float>::eye(4, 4);
    T1(0, 3) = t1[0];
    T1(1, 3) = t1[1];
    T1(2, 3) = t1[2];

    cv::Mat_<float> T2 = cv::Mat_<float>::eye(4, 4);
    T2(0, 3) = -t2[0];
    T2(1, 3) = -t2[1];
    T2(2, 3) = -t2[2];

    /* Calculate covariance matrix for input points. Also calculate RMS deviation from centroid
     * which is used for scale calculation.
     */
    cv::Mat_<float> C(3, 3, 0.0);
    double p1Rms = 0, p2Rms = 0;
    for (int ptIdx = 0; ptIdx < points1.rows; ptIdx++) {
        cv::Vec3f p1 = points1(ptIdx, 0) + t1;
        cv::Vec3f p2 = points2(ptIdx, 0) + t2;
        p1Rms += p1.dot(p1);
        p2Rms += p2.dot(p2);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                C(i, j) += p2[i] * p1[j];
            }
        }
    }

    cv::Mat_<float> u, s, vh;
    cv::SVD::compute(C, s, u, vh);

    cv::Mat_<float> R = u * vh;

    if (cv::determinant(R) < 0) {
        R -= u.col(2) * (vh.row(2) * 2.0);
    }

    double scale = sqrt(p2Rms / p1Rms);
    R *= scale;

    cv::Mat_<float> M = cv::Mat_<float>::eye(4, 4);
    R.copyTo(M.colRange(0, 3).rowRange(0, 3));

    cv::Mat_<float> result = T2 * M * T1;
    result /= result(3, 3);

    return result.rowRange(0, 3);
}
#if 0
int main( int argc, char **argv ) {
	cv::Mat_<cv::Vec3f> pts1(4,1);
	cv::Mat_<cv::Vec3f> pts2(4,1);
	
	pts1.at<cv::Vec3f>(0) = cv::Vec3f(  0.f,  0.f,  0.f );
	pts1.at<cv::Vec3f>(1) = cv::Vec3f( 10.f,  0.f,  0.f );
	pts1.at<cv::Vec3f>(2) = cv::Vec3f( 10.f, 10.f,  0.f );
	pts1.at<cv::Vec3f>(3) = cv::Vec3f(  0.f, 10.f,  0.f );

	pts2.at<cv::Vec3f>(0) = cv::Vec3f(  0.f,  0.f, 10.f );
	pts2.at<cv::Vec3f>(1) = cv::Vec3f( 10.f,  0.f, 10.f );
	pts2.at<cv::Vec3f>(2) = cv::Vec3f( 10.f, 10.f, 10.f );
	pts2.at<cv::Vec3f>(3) = cv::Vec3f(  0.f, 10.f, 10.f );


	cv::Mat_<float> xfm = FindRigidTransform(pts1, pts2 );

	cout << xfm;
	return 1;
}
#endif
