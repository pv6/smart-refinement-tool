#ifndef CHANVESELSM_H
#define CHANVESELSM_H

#include <LSMBase.hpp>

class ChanVeseSignedDistanceField : public SignedDistanceField
{
public:
    /*!
    * \brief Compute Chan-Vese specific Signed Distance Field (distance to mask) using image
    * \param image Input image
    * \param mask Input mask
    */
    Matrix2D<double> compute(const Matrix2D<double> &image, const Matrix2D<bool> &mask);

    //! Set base distance increase from pixel to pixel
    ChanVeseSignedDistanceField & setBaseIncrease(double newBaseIncrease);
    //! Set gradient magnitude based increase weight
    ChanVeseSignedDistanceField & setGradWeight(double newGradWeight);
    //! Get total increase in given pixels
    double getTotalIncrease(int x, int y);

private:
    //! Calculate image intensity gradient magnitutude
    void calculateGradMagnitude();
    //! Calculate total distance increase in a given pixel
    double calculateTotalIncrease(int x, int y);
    //! Calculate and set total distance increase in each pixel
    void calculateTotalIncrease();

    const Matrix2D<double> * image_;
    const Matrix2D<bool> * mask_;
    std::unique_ptr<Matrix2D<double>> gradMagnitude_;
    std::unique_ptr<Matrix2D<double>> totalIncrease_;

    double baseIncrease_ = 0.001;
    double gradWeight_ = 1000;
};

//! Chan-Vese Level-Set method for image segmentation based on intial region mask
class ChanVeseLevelSet : public LevelSetMethodBase
{
public:
    //! Constructor (allocates memory for inner matricies)
    ChanVeseLevelSet(const int sizeX, const int sizeY);

    //! Allocate memory for inner matrices
    void resize(const int sizeX, const int sizeY);

    /*!
      * \brief Run region growing
      * \param image Input image
      * \param initial Initial region
      * \param result Output mask
      */
    void run(const Matrix2D<double> &image, const Matrix2D<bool> &initial, Matrix2D<bool> &result);
    void run(const Matrix2D<double> &image, const Matrix2D<bool> &initial, Matrix2D<bool> &result, std::function<double(void)> selectC);

    /*!
    * \brief Set sensitivity coefficient
    */
    void setSensitivity(const double newValue);

    /*!
      * \brief Set Selective Segmentation term coefficient
      */
    void setR2Mu(const double newValue);

    /*!
    * \brief Set Data Fitting term coefficient
    */
    void setDF2SS(const double newValue);

    /*!
      * \brief Set dt (growth rate per iteration)
      */
    void setDt(const double newDt);

    /*!
    * \brief Set tolerance
    */
    void setTolerance(const double newTolerance);

    /*!
    * \brief Set maximum number of iterations
    */
    void setMaxIterations(const double newMaxIterations);

    /*!
    * \brief Set Eucledean distance coefficient in distance field
    */
    void setEuclIncrease(const double newEuclIncrease);

    /*!
      * \brief Set input slice to get average intensity from
      * \param newInputSlice New input slice
      */
    void setInputSlice(const Matrix2D<double> &newInputSlice);

    /*
      * \brief Calculate the mean intensity inside the mask
      * \param mask Input segmentation mask
      */
    double calculateC(const Matrix2D<bool> mask);

    void setNu(double newNu);
private:
    template<typename T>
    using pMatrix2D = std::unique_ptr<Matrix2D<T>>;

    //! Calculate gamma1_ and gamma2_ values
    void calculateGamma();
    void calculateGamma(const double c);

    /*!
      * \brief Perform one Chan-Vese iteration
      * \return phi average change
      */
      double updatePhi();
             
    /*!
      * \brief Calculate data fitting term foreground function
      * \param x, y Pixel coordinates
      */
    double f1(int x, int y);

    /*!
      * \brief Calculate data fitting term background function
      * \param x, y Pixel coordinates
      */
    double f2(int x, int y);

    double calculateA(int i, int j);
    double calculateB(int i, int j);

    /*!
      * \brief Calculate data fitting term
      * \param x, y Pixel coordinates
      * \param foreCoef foreground function weight coefficient
      * \param backCoef background function weight coefficient
      */
    double dataFittingTerm(int x, int y, double foreCoef = 1, double backCoef = 1);

    //! Calculate average mask intensity from input slice
    double calculateC();

    const Matrix2D<double> *inputSlice_ = nullptr;
    double dt_ = 0.1;
    double tolerance_ = 1.e-8;
    int maxIterations_ = 30;
    const double eps_ = 1.e-8; // small number to prevent division by zero
    const Matrix2D<bool> *mask_ = nullptr;
    const Matrix2D<double> *targetSlice_ = nullptr;
    pMatrix2D<double> phi_;
    pMatrix2D<double> a_;
    pMatrix2D<double> b_;
    pMatrix2D<double> dist_;
    pMatrix2D<double> r_;
    double gamma1_, gamma2_;
    double sensitivity_ = 0.5;
    double df2ss_ = 0.5;
    double r2mu_ = 0.5;
    double c_;
    double nu_ = 1;
}; // End of 'ChanVeseLevelSet' class

#endif // CHANVESELSM_H
