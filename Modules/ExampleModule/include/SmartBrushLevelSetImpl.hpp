#ifndef SMARTBRUSHLEVELSETIMPL_H
#define SMARTBRUSHLEVELSETIMPL_H

#include <commontypes.hpp>
#include <generallsm.hpp>

//! Position of a Voxel
typedef std::array<int, 3> VoxelPosition;
//! Bounding box in 3D - two corner VoxelPosition points
typedef std::array<VoxelPosition, 2> BBox3D;
//! Position of a Pixel in the slice
typedef std::array<int, 2> PixelPosition;
//! Bounding box in 2D - two corner PixelPosition points
typedef std::array<PixelPosition, 2> BBox2D;

//! Implements smart brush via level set
class SmartBrushLevelSetImpl
{
public:
    /*!
    * \brief Voxel labeling event handler. Each use case should have it implemented
    * \param voxelPos Voxel position in the image space
    */
    virtual void OnNewLabel2D(std::array<int, 3> &voxelPos);
    /*!
    * \brief Get median of vector
    * \param values Vector of doubles
    */
    static double getMedian(std::vector<double> &values);

    //! Start selection
    virtual void startSelection();

    void connect(std::function<void(VoxelPosition)> markVoxel,
        std::function<double(VoxelPosition)> getVoxelIntensity,
        std::function<int()> getSliceMinX,
        std::function<int()> getSliceMaxX,
        std::function<int()> getSliceMinY,
        std::function<int()> getSliceMaxY);
    
    /*!
    * \brief Reallocate memory for patterns on radius change
    * \param radius New radius
    */
    void changeRadius(const int radius);
    /*!
    * \brief Change sensitivity parameter
    * \param sensitivity New sensitivity
    */
    void changeSensitivity(const double sensitivity);
    /*!
    * \brief Change curvature parameter
    * \param alpha New curvature parameter
    */
    void changeCurvature(const double alpha);
    /*!
    * \brief Draw one brush circle with respect to borders
    * \param voxelPos Voxel position in the image space - center of new circle
    * \param radius Brush radius
    */
    void draw(const VoxelPosition &voxelPos, const int radius);
    /*!
    * \brief Draw circle on the center of mask
    * \param contour Initial contour to draw on
    * \param radius Radius of circle
    */
    void makeInitialCircle(Matrix2D<bool> &contour, const int radius);
    /*!
    * \brief Init density field matrix with expression: D = eps - abs(thresh - image)
    * \param bbox Bounding box of area
    * \param eps sensitivity, means acceptable image intensity bandwidth. In range of [0 .. 1]
    * \param thresh Threshold - mean value of acceptable intensity
    * \param dField Output density field matrix
    */
    void initDensityField(const BBox3D &bbox, const double eps, const double thresh, Matrix2D<double> &dField);
    /*!
    * \brief Calculate median threshold
    * \param bbox Bounding box of area
    * \param radius Brush radius
    */
    double calcThreshold(const BBox3D &bbox, const int radius);
    /*!
    * \brief Remove contour leaks and fit area to circle
    * \param mask Mask to remove leaks on
    * \param radius Radius of circle to fit by
    */
    Matrix2D<bool> removeLeaksInCircle(Matrix2D<bool> &mask, const int radius);
    /*!
    * \brief Mark voxels in image where mask has 'true'(bool) value
    * \param leftCorner Upper left corner position on the image - where to start marking voxels
    * \param mask Mask
    */
    void markMask(const VoxelPosition &leftCorner, const Matrix2D<bool> &mask);

    int getRadius();

private:

    std::unique_ptr<GeneralLevelSet> lsm_;
    
    int radius_ = 10;
    double sensitivity_ = 0.15;
    //! Downsampling is disabled if downsamplingRatio_ is zero
    int downsamplingRatio_ = 2;

    std::function<void(VoxelPosition)> markVoxel_;
    std::function<double(VoxelPosition)> getVoxelIntensity_;
    std::function<int()> getSliceMinX_;
    std::function<int()> getSliceMaxX_;
    std::function<int()> getSliceMinY_;
    std::function<int()> getSliceMaxY_;
};

#endif // SMARTBRUSHLEVELSETIMPL_H
