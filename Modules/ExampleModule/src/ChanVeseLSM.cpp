#include <ChanVeseLSM.hpp>

#include <algorithm>
#include <queue>
#include <unordered_set>

//! Calculate square function
template<typename T>
    inline T sqr(T x)
    {
        return x * x;
    } // End of 'sqr' function

//! Calculate delta function value
inline double delta(double t)
{
    return 1 / (3.141592653589793238463 * (1 + sqr(t)));
} // End of 'delta' function

/***
 * Chan-Vese Signed Distance Field implementation
 ***/

//! Calculate image intensity gradient magnitutude
void ChanVeseSignedDistanceField::calculateGradMagnitude()
{
    for (int i = 0; i < image_->sizeY; i++)
        for (int j = 0; j < image_->sizeX; j++)
        {
            const double dfdx = (image_->get(j, i) - image_->get(j + 1, i)) / 2;
            const double dfdy = (image_->get(j, i) - image_->get(j, i + 1)) / 2;
            gradMagnitude_->set(j, i, sqrt(sqr(dfdx) + sqr(dfdy)));
        }
} // End of 'ChanVeseSignedDistanceField::calculateGradMagnitude' function

//! Calculate total distance increase in a given pixel
double ChanVeseSignedDistanceField::calculateTotalIncrease(int x, int y)
{
    return baseIncrease_ + gradWeight_ * sqr(gradMagnitude_->get(x, y));
} // End of 'ChanVeseSignedDistanceField::calculateGradMagnitude' function

void NormalizeMatrix(Matrix2D<double> &matrix)
{
    // Find maximum value
    double max_val = 0;
    matrix.each([&max_val](double &x) -> void {
      if (x == std::numeric_limits<double>::max())
        x = 0;
      else
        if (max_val < x)
          max_val = x;
    });

    // Normalize matrix
    if (max_val != 0)
      matrix.each([max_val](double &x) -> void { x /= max_val; });
}

//! Calculate and set total distance increase in each pixel
void ChanVeseSignedDistanceField::calculateTotalIncrease()
{
    calculateGradMagnitude();
    // improve weak borders
    // apply sqrt
    //gradMagnitude_->each([](double &val) -> void {val = sqrt(val);});
    //
    //// calculate otsu and remove noise
    //NormalizedIntensityHistogram<256> histogram;
    //histogram.fromImage(*gradMagnitude_);
    //double border1, border2;
    //if (histogram.otsu2(border1, border2))
    //    gradMagnitude_->each([border1](double &val) -> void {if (val < border1) val = 0;});

    // calculate total increase
    for (int i = 0; i < totalIncrease_->sizeY; i++)
        for (int j = 0; j < totalIncrease_->sizeX; j++)
          //if (mask_->get(j, i))
          //  totalIncrease_->set(j, i, 0);
          //else
            totalIncrease_->set(j, i, calculateTotalIncrease(j, i));
} // End of 'ChanVeseSignedDistanceField::calculateTotalIncrease' function

//! Get total increase in given pixels
double ChanVeseSignedDistanceField::getTotalIncrease(int x, int y)
{
    return totalIncrease_->get(x, y);
} // End of 'ChanVeseSignedDistanceField::getTotalIncrease' function

 /*!
  * \brief Compute Chan-Vese specific Signed Distance Field (distance to mask) using image
  * \param image Input image
  * \param mask Input mask
  */
Matrix2D<double> ChanVeseSignedDistanceField::compute(const Matrix2D<double> &image, const Matrix2D<bool> &mask)
{
    Matrix2D<double> output(image.sizeX, image.sizeY);

    // Store image and mask pointers
    image_ = &image;
    mask_ = &mask;

    // Allocate necessary matrices
    gradMagnitude_ = std::make_unique<Matrix2D<double>>(image.sizeX, image.sizeY);
    totalIncrease_ = std::make_unique<Matrix2D<double>>(image.sizeX, image.sizeY);

    // Calulate matrices values
    calculateTotalIncrease();

    // Compute distance field
    computeGeodesicDistance(image, mask,
                            [this](int x, int y, int nX, int nY) { return this->getTotalIncrease(nX, nY); },
                            output);

    // Normalize distance
    NormalizeMatrix(output);

    return std::move(output);
} // End of 'ChanVeseSignedDistanceField::compute' function

//! Set base distance increase from pixel to pixel
ChanVeseSignedDistanceField & ChanVeseSignedDistanceField::setBaseIncrease(double newBaseIncrease)
{
    baseIncrease_ = newBaseIncrease;
    return *this;
} // End of 'ChanVeseSignedDistanceField::setBaseIncrease' function

//! Set gradient magnitude based increase weight
ChanVeseSignedDistanceField & ChanVeseSignedDistanceField::setGradWeight(double newGradWeight)
{
    gradWeight_ = newGradWeight;
    return *this;
} // End of 'ChanVeseSignedDistanceField::setGradWeight' function

/***
 * Chan-Vese Level Set impementation
 ***/

//! Constructor (allocates memory for inner matricies)
ChanVeseLevelSet::ChanVeseLevelSet(const int sizeX, const int sizeY)
{
    resize(sizeX, sizeY);
} // End of 'ChanVeseLevelSet::ChanVeseLevelSet' function

//! Allocate memory for inner matrices
void ChanVeseLevelSet::resize(const int sizeX, const int sizeY)
{
    phi_ = std::make_unique<Matrix2D<double>>(sizeX, sizeY);
    a_ = std::make_unique<Matrix2D<double>>(sizeX, sizeY);
    b_ = std::make_unique<Matrix2D<double>>(sizeX, sizeY);
    dist_ = std::make_unique<Matrix2D<double>>(sizeX, sizeY);
    r_ = std::make_unique<Matrix2D<double>>(sizeX, sizeY);
} // End of 'ChanVeseLevelSet::resize' function

/*!
 * \brief Calculate data fitting term foreground function
 * \param x, y Pixel coordinates
 */
double ChanVeseLevelSet::f1(int x, int y)
{
    return sqr(targetSlice_->get(x, y) - c_);
} // End of 'ChanVeseLevelSet::f1' function

/*!
 * \brief Calculate data fitting term background function
 * \param x, y Pixel coordinates
 */
double ChanVeseLevelSet::f2(int x, int y)
{
    const double z = targetSlice_->get(x, y);
    if (z < c_ - gamma1_ || z > c_ + gamma2_)
        return 0;
    else if (z < c_)
        return 1 + (z - c_) / gamma1_;
    return 1 - (z - c_) / gamma2_;
} // End of 'ChanVeseLevelSet::f2' function

/*!
 * \brief Calculate data fitting term
 * \param x, y Pixel coordinates
 * \param foreCoef foreground function weight coefficient
 * \param backCoef background function weight coefficient
 */
double ChanVeseLevelSet::dataFittingTerm(int x, int y, double foreCoef, double backCoef)
{
    const double z = targetSlice_->get(x, y);

    return foreCoef * f1(x, y) - backCoef * f2(x, y);
} // End of 'ChanVeseLevelSet::dataFittingTerm' function

double ChanVeseLevelSet::calculateA(int i, int j)
{
  return 1 / sqrt(sqr(eps_) +
    sqr(phi_->get(j, i + 1) - phi_->get(j, i)) +
    sqr((phi_->get(j + 1, i) - phi_->get(j - 1, i)) / 2));
} // End of 'ChanVeseLevelSet::calculateA' function

double ChanVeseLevelSet::calculateB(int i, int j)
{
  return 1 / sqrt(sqr(eps_) +
    sqr((phi_->get(j, i + 1) - phi_->get(j, i - 1)) / 2) +
    sqr(phi_->get(j, i) - phi_->get(j, i + 1)));
} // End of 'ChanVeseLevelSet::calculateB' function

/*!
 * \brief Perform one Chan-Vese iteration
 * \return phi average change
 */
double ChanVeseLevelSet::updatePhi()
{
    double dif = 0;

    // Update phi
    for (int i = 0; i < targetSlice_->sizeY; i++)
        for (int j = 0; j < targetSlice_->sizeX; j++)
        {

            // Update a_ and b_ 
            if (i == 0)
                a_->set(j, -1, calculateA(-1, j));
            if (j == 0)
                b_->set(-1, i, calculateB(i, -1));
            
            a_->set(j, i, calculateA(i, j));
            b_->set(j, i, calculateB(i, j));

            // Update phi in (j, i)
            const double old_phi = phi_->get(j, i);
            
            const double dt_delta = dt_ * delta(phi_->get(j, i));
            const double new_phi = (phi_->get(j, i)
                                    + dt_delta
                                      * (-nu_ + (1 - r2mu_) * (a_->get(j, i) * phi_->get(j, i + 1) + a_->get(j, i - 1) * phi_->get(j, i - 1)
                                                + b_->get(j, i) * phi_->get(j + 1, i) + b_->get(j - 1, i) * phi_->get(j - 1, i))
                                         - r2mu_ * r_->get(j, i)))
                                   / (1 + dt_delta * (1 - r2mu_) * (a_->get(j, i) + a_->get(j, i - 1) + b_->get(j, i) + b_->get(j - 1, i)));

            phi_->set(j, i, new_phi);

            dif += fabs(new_phi - old_phi);

            // Update padding values if necessary
            //if (i == 0)
            //    phi_->set(j, -1, new_phi);
            //else if (i == targetSlice_->sizeY - 1)
            //    phi_->set(j, targetSlice_->sizeY, new_phi);
            //
            //if (j == 0)
            //    phi_->set(-1, i, new_phi);
            //else if (j == targetSlice_->sizeX - 1)
            //    phi_->set(targetSlice_->sizeX, i, new_phi);
        }

    //dif = sqrt(dif / targetSlice_->sizeX / targetSlice_->sizeY);

    return dif;
} // End of 'ChanVeseLevelSet::updatePhi' function

/*!
 * \brief Set input slice to get average intensity from
 * \param newInputSlice New input slice
 */
void ChanVeseLevelSet::setInputSlice(const Matrix2D<double> &newInputSlice)
{
    inputSlice_ = &newInputSlice;
} // End of 'ChanVeseLevelSet::setInputSlice' function

/*!
 * \brief Set sensitivity coefficient
 */
void ChanVeseLevelSet::setSensitivity(const double newValue)
{
    printf("Sensitivity = %f\n", newValue);
    sensitivity_ = newValue;
} // End of 'ChanVeseLevelSet::setSensitivity' function

/*!
 * \brief Set Data Fitting to Selective Segmentation terms blending coefficient
 */
void ChanVeseLevelSet::setR2Mu(const double newValue)
{
    printf("r2mu = %f\n", newValue);
    r2mu_ = newValue;
} // End of 'ChanVeseLevelSet::setSSCoef' function

  /*!
  * \brief Set r term magnitude
  */
void ChanVeseLevelSet::setDF2SS(const double newValue)
{
    printf("df2ss = %f\n", newValue);
    df2ss_ = newValue;
} // End of 'ChanVeseLevelSet::setDFCoef' function

/*!
* \brief Set dt
*/
void ChanVeseLevelSet::setDt(const double newDt)
{
    printf("dt = %f\n", newDt);
    dt_ = newDt;
} // End of 'ChanVeseLevelSet::setDt' function

void ChanVeseLevelSet::setNu(double newNu)
{
  printf("nu = %f\n", newNu);
  nu_ = newNu;
} // End of 'ChanVeseLevelSet::setNu' function

/*!
* \brief Set tolerance
*/
void ChanVeseLevelSet::setTolerance(const double newTolerance)
{
    printf("tolerance = %f\n", newTolerance);
    tolerance_ = newTolerance;
} // End of 'ChanVeseLevelSet::setTolerance' function

/*!
 * \brief Set maximum number of iterations
 */
void ChanVeseLevelSet::setMaxIterations(const double newMaxIterations)
{
    printf("maxIterations = %f\n", newMaxIterations);
    maxIterations_ = (int)newMaxIterations;
} // End of 'ChanVeseLevelSet::setMaxIterations' function

//! Calculate average mask intensity from input slice
double ChanVeseLevelSet::calculateC()
{
    c_ = calculateC(*mask_);
    return c_;
}

double ChanVeseLevelSet::calculateC(const Matrix2D<bool> mask)
{
    double c = 0;

    // Calculate as mean intensity value
    int counter = 0;

    for (int y = 0; y < inputSlice_->sizeY; y++)
        for (int x = 0; x < inputSlice_->sizeX; x++)
            if (mask.get(x, y))
            {
                c += inputSlice_->get(x, y);
                counter++;
            }
    if (counter != 0)
        c /= counter;

    // Calculate as most observed intensity value
    //NormalizedIntensityHistogram<256> labelHistogram;
    //
    //for (int y = 0; y < inputSlice_->sizeY; y++)
    //    for (int x = 0; x < inputSlice_->sizeX; x++)
    //        if (mask.get(x, y))
    //            labelHistogram.add(inputSlice_->get(x, y));
    //
    //c = labelHistogram.mostObservedValue();

    return c;
} // End of 'ChanVeseLevelSet::calculateC' function

//! Calculate gamma1_ and gamma2_ values
void ChanVeseLevelSet::calculateGamma()
{
    calculateGamma(c_);
}

void ChanVeseLevelSet::calculateGamma(const double c)
{
    NormalizedIntensityHistogram<256> histogram;

    histogram.fromImage(*targetSlice_);
    double border1, border2;
    if (histogram.otsu2(border1, border2)) {
        if (c < border1)
        {
            gamma1_ = c;
            gamma2_ = border1 - c;
        }
        else
            if (c < border2)
            {
                gamma1_ = c - border1;
                gamma2_ = border2 - c;
            }
            else
            {
                gamma1_ = c - border2;
                gamma2_ = 1 - c;
            }
    }
    else
    {
        gamma1_ = gamma2_ = 0.1;
    }

    if (gamma1_ == 0)
        gamma1_ = 0.1;
    if (gamma2_ == 0)
        gamma2_ = 0.1;

    const double MIN_GAMMA_MULTIPLIER = 0.5;
    const double MAX_GAMMA_MULTIPLIER = 5;


    gamma1_ *= MIN_GAMMA_MULTIPLIER * sensitivity_ + MAX_GAMMA_MULTIPLIER * (1 - sensitivity_);
    gamma2_ *= MIN_GAMMA_MULTIPLIER * sensitivity_ + MAX_GAMMA_MULTIPLIER * (1 - sensitivity_);
} // En dof 'ChanVeseLevelSet::calculateGamma' function

/*!
 * \brief Run region growing
 * \param image Input image
 * \param initial Initial region
 * \param result Output mask
 */
void ChanVeseLevelSet::run(const Matrix2D<double> &image,
  const Matrix2D<bool> &initial, Matrix2D<bool> &result)
{
    run(image, initial, result, nullptr);
}

void ChanVeseLevelSet::run(const Matrix2D<double> &image,
  const Matrix2D<bool> &initial, Matrix2D<bool> &result,
  std::function<double(void)> selectC)
{
    targetSlice_ = &image;
    mask_ = &initial;

    // Calculate distance map
    *dist_ = ChanVeseSignedDistanceField().compute(image, initial);

    if (selectC)
        c_ = selectC();
    else
        c_ = calculateC();

    // Calculate gamma1_ and gamma2_
    calculateGamma();

    // Set initial phi values
    //*phi_ = SignedDistanceField().compute(initial);
    for (int i = 0; i < targetSlice_->sizeY; i++)
        for (int j = 0; j < targetSlice_->sizeX; j++)
            if (mask_->get(j, i))
                phi_->set(j, i, 0.30);
            else
                phi_->set(j, i, 0);
    // phi_->fill(0);
    phi_->fillPadding(-1);

    // Compute r term - normalized combination of Data Fitting term and Selective Segmentation term
    r_->fill(0);
    double r_max = 0;
    for (int i = 0; i < targetSlice_->sizeY; i++)
        for (int j = 0; j < targetSlice_->sizeX; j++)
        {
            if (dist_->get(j, i) < 0)
                printf("WARNING: negative distance in: (%d, %d)!!!!", j, i);
            r_->set(j, i, df2ss_ * dataFittingTerm(j, i) + (1 - df2ss_) * dist_->get(j, i));

            r_max = std::max(r_max, fabs(r_->get(j, i)));
        }

    printf("R MAX: %f\n", r_max);
    // normalize r
    if (r_max != 0)
        r_->each([r_max](double &x) -> void { x /= r_max; });

    // Update phi
    int iterations = 0;
    do
    {
        iterations++;
    } while (updatePhi() > tolerance_ && iterations < maxIterations_);

    printf("done iterations: %d\n", iterations);

    int size = 0;
    // Fill results inside bound box
    for (int i = 0; i < targetSlice_->sizeY; i++)
        for (int j = 0; j < targetSlice_->sizeX; j++)
        {
            size += phi_->get(j, i) >= 0;
            result.set(j, i, phi_->get(j, i) >= eps_);
        }
    printf("marked voxels: %d\n", size);
}  // End of 'ChanVeseLevelSet::run' function
