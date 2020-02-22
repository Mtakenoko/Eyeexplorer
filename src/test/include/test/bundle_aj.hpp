#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
// Read a Bundle Adjustment in the Large dataset.
//<num_cameras> <num_points> <num_observations>
// <camera_index_1> <point_index_1> <x_1> <y_1>
// ...
// <camera_index_num_observations> <point_index_num_observations> <x_num_observations> <y_num_observations>
// <camera_1>
// ...
// <camera_num_cameras>
// <point_1>
// ...
// <point_num_points>

class BALProblem
{
public:
    ~BALProblem()
    {
        delete[] point_index_;
        delete[] camera_index_;
        delete[] observations_;
        delete[] parameters_;
    }
    int num_observations() const { return num_observations_; }
    const double *observations() const { return observations_; }
    double *mutable_cameras() { return parameters_; }
    double *mutable_points() { return parameters_ + 9 * num_cameras_; }
    double *mutable_camera_for_observation(int i)
    {
        return mutable_cameras() + camera_index_[i] * 9;
    }
    double *mutable_point_for_observation(int i)
    {
        return mutable_points() + point_index_[i] * 3;
    }
    bool LoadFile(const char *filename)
    {
        FILE *fptr = fopen(filename, "r");
        if (fptr == NULL)
        {
            return false;
        };
        //1行目<num_cameras> <num_points> <num_observations>
        FscanfOrDie(fptr, "%d", &num_cameras_);
        FscanfOrDie(fptr, "%d", &num_points_);
        FscanfOrDie(fptr, "%d", &num_observations_);
        point_index_ = new int[num_observations_];
        camera_index_ = new int[num_observations_];
        observations_ = new double[2 * num_observations_];
        num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
        parameters_ = new double[num_parameters_];
        //2行目〜　<camera_index_1> <point_index_1> <x_1> <y_1>
        for (int i = 0; i < num_observations_; ++i)
        {
            FscanfOrDie(fptr, "%d", camera_index_ + i);
            FscanfOrDie(fptr, "%d", point_index_ + i);
            for (int j = 0; j < 2; ++j)
            {
                FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
            }
        }
        //31845行目~　<> 
        for (int i = 0; i < num_parameters_; ++i)
        {
            FscanfOrDie(fptr, "%lf", parameters_ + i);
        }
        return true;
    }

private:
    template <typename T>
    void FscanfOrDie(FILE *fptr, const char *format, T *value)
    {
        int num_scanned = fscanf(fptr, format, value);
        if (num_scanned != 1)
        {
            LOG(FATAL) << "Invalid UW data file.";
        }
    }
    int num_cameras_;
    int num_points_;
    int num_observations_;
    int num_parameters_;
    int *point_index_;
    int *camera_index_;
    double *observations_;
    double *parameters_;
};
// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError
{
    SnavelyReprojectionError(double observed_x, double observed_y)
        : observed_x(observed_x), observed_y(observed_y) {}
    template <typename T>
    bool operator()(const T *const camera,
                    const T *const point,
                    T *residuals) const
    {
        // camera[0,1,2] are the angle-axis rotation.
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);
        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];
        // Apply second and fourth order radial distortion.
        const T &l1 = camera[7];
        const T &l2 = camera[8];
        T r2 = xp * xp + yp * yp;
        T distortion = 1.0 + r2 * (l1 + l2 * r2);
        // Compute final projected point position.
        const T &focal = camera[6];
        T predicted_x = focal * distortion * xp;
        T predicted_y = focal * distortion * yp;
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - observed_x;
        residuals[1] = predicted_y - observed_y;
        return true;
    }
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(const double observed_x,
                                       const double observed_y)
    {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
            new SnavelyReprojectionError(observed_x, observed_y)));
    }
    double observed_x;
    double observed_y;
};