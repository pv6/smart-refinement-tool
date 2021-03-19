#ifndef GENERALLSM_H
#define GENERALLSM_H

#include <array>
#include <algorithm>
#include <utility>
#include <memory>
#include <unordered_map>
#include <functional>
#include <vector>
#include <commontypes.hpp>
#include <lsmbase.hpp>

//! General Level-Set for evaluating contour expressed with density field
class GeneralLevelSet : public LevelSetMethodBase {
public:
    //! Constructor (allocates memory for inner matricies)
    GeneralLevelSet(const int sizeX, const int sizeY);

    /*!
    * \brief Run Level-Set contour evaluation
    * \param densityField Input density field - positive where there is need to grow, negative where to grow down
    * \param initial Initial contour mask
    * \param result Output
    */
    void run(const Matrix2D<double> &densityField, const Matrix2D<bool> &initial, Matrix2D<bool> &result);

    /*!
    * \brief Run region growing
    * \param image Input image / density field
    * \param initial Initial region
    */
    virtual Matrix2D<bool> run(const Matrix2D<double> &image, const Matrix2D<bool> &initial);

    /*!
    * \brief Set divider to reduce iterations number
    * \param div divide factor
    */
    void setIterationsDivider(int div) {
        iterationsDivider_ = div;
    }
    /*!
    * \brief Set curvature parameter
    * \param div divide factor
    */
    void setAlpha(const double alpha) {
        alpha_ = alpha;
    }
    //! Get curvature parameter
    double getAlpha() {
        return alpha_;
    }
protected:
    /*!
    * \brief Calculate curvature for each point of field
    * \param phi phi-surface
    * \param curvature Output curvature
    */
    void updateCurvature(const Matrix2D<double>& phi, const BBox2D &bbox, Matrix2D<double>& curvature);
    /*!
    * \brief Get iterations upper limit (may depend on size of area)
    * \param x width of area
    * \param y height of area
    */
    int calculateIterations(int x, int y);

    std::unique_ptr<Matrix2D<double> > phi_;
    std::unique_ptr<Matrix2D<double> > curvature_;
    std::unique_ptr<Matrix2D<double> > Fgradphi_;
private:
    //! curvEps_ is used to avoid division by zero
    const double curvEps_ = 1e-5;
    //! dx_ describes how fast contour grow
    const double dx_ = 0.1;
    //! alpha_ describes how much density field will affect phi changes
    double alpha_ = 0.9;
    //! divider to reduce iterations number
    int iterationsDivider_ = 1;
};

//! Downsampler runs algorithm several times on one region of an image with different resolutions
template<typename S>
class Downsampler {
public:
    /*!
    * \brief Constructor
    * \param div Divide factor
    */
    Downsampler(const int div) : div_(div) {}
    /*!
    * \brief Run S algorithm a few times with different resolutions
    * \param densityField Input density field - positive where there is need to grow, negative where to grow down
    * \param initial Initial contour mask
    */
    Matrix2D<bool> run(Matrix2D<double> &densityField, Matrix2D<bool> &initial);
    /*!
    * \brief Set curvature parameter
    * \param div divide factor
    */
    void setAlpha(const double alpha) {
        alpha_ = alpha;
    }
private:
    Matrix2D<double> downsampleDouble(const Matrix2D<double> &matrix);
    Matrix2D<bool> downsampleBool(const Matrix2D<bool> &matrix, int div);
    Matrix2D<bool> upscale(const Matrix2D<bool> &matrix, int mul);
protected:
    int div_;
    //! alpha_ describes how much density field will affect phi changes
    double alpha_ = 0.9;
};

template<typename S>
Matrix2D<double> Downsampler<S>::downsampleDouble(const Matrix2D<double> &matrix) {
    Matrix2D<double> result(matrix.sizeX / div_, matrix.sizeY / div_);
    for (int y = 0; y < result.sizeY; y++) {
        for (int x = 0; x < result.sizeX; x++) {
            double max = -std::numeric_limits<double>::infinity();
            for (int yy = y * div_; yy < std::min((y + 1) * div_, matrix.sizeY); yy++) {
                for (int xx = x * div_; xx < std::min((x + 1) * div_, matrix.sizeX); xx++) {
                    double val = matrix.get(xx, yy);
                    if (val > max)
                        max = val;
                }
            }
            result.set(x, y, max);
        }
    }
    return result;
}
template<typename S>
Matrix2D<bool> Downsampler<S>::downsampleBool(const Matrix2D<bool> &matrix, int div) {
    Matrix2D<bool> result(matrix.sizeX / div, matrix.sizeY / div);
    for (int y = 0; y < result.sizeY; y++) {
        for (int x = 0; x < result.sizeX; x++) {
            result.set(x, y, matrix.get(x * div, y * div));
        }
    }
    return result;
}
template<typename S>
Matrix2D<bool> Downsampler<S>::upscale(const Matrix2D<bool> &matrix, int mul) {
    Matrix2D<bool> result(matrix.sizeX * mul, matrix.sizeY * mul);
    for (int y = 0; y < result.sizeY; y++) {
        for (int x = 0; x < result.sizeX; x++) {
            result.set(x, y, matrix.get(x / mul, y / mul));
        }
    }
    return result;
}

template<typename S>
Matrix2D<bool> Downsampler<S>::run(Matrix2D<double> &densityField, Matrix2D<bool> &initial) {
    Matrix2D<double> downsampled = downsampleDouble(densityField);
    Matrix2D<bool> init = downsampleBool(initial, div_);
    S method(downsampled.sizeX, downsampled.sizeY);
    method.setIterationsDivider(3);
    method.setAlpha(alpha_);
    Matrix2D<bool> res = method.run(downsampled, init);
    S method2(densityField.sizeX, densityField.sizeY);
    method2.setIterationsDivider(3);
    method2.setAlpha(alpha_);
    return method2.run(densityField, upscale(res, div_));
}

#endif // GENERALLSM_H
