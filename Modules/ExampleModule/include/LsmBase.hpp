#ifndef LSMBASE_H
#define LSMBASE_H

#include <array>
#include <algorithm>
#include <utility>
#include <memory>
#include <unordered_map>
#include <functional>
#include <vector>
#include <CommonTypes.hpp>

//! Normalization parameters - min and max intensity values
struct NormalizationParameters {
    double min = std::numeric_limits<double>::max(), max = std::numeric_limits<double>::min();
};

//! Histogram of normalized intensities
template<size_t bucketsNumber>
class NormalizedIntensityHistogram {
public:
    //! C-tor
    NormalizedIntensityHistogram() {
        reset();
    }

    /*!
    * \brief Reset the histogram
    */
    void reset() {
        observationsNumber = 0;
        maxBucket_ = 0;
        std::fill(&buckets[0], &buckets[bucketsNumber - 1], 0);
    }

    /*!
    * \brief Add new value to the histogram
    * \param val Intensity value
    */
    void add(double val) {
        size_t idx = index(val);
        buckets[idx] += 1.0;
        observationsNumber++;
        if (buckets[idx] > buckets[maxBucket_])
            maxBucket_ = idx;
    }

    /*!
    * \brief Get frequency of value
    * \param val Intensity value
    * \return frequency
    */
    double get(double val) const {
        return buckets[index(val)];
    }

    /*!
    * \brief Normalize histogram with the given divider
    * \param denominator Divider
    */
    void normalize(double denominator) {
        for (size_t i = 0; i < bucketsNumber; i++) {
            buckets[i] /= denominator;
        }
    }

    /*!
    * \brief Normalize histogram
    * \param denominator Divider
    */
    void normalize() {
        if (observationsNumber != 0) {
            normalize(static_cast<double>(observationsNumber));
            observationsNumber = 0;
        }
        else {
            normalize(sum());
        }
    }

    /*!
    * \brief Calculate sum of all buckets
    * \return sum
    */
    double sum() {
        double res = 0.0;
        for (size_t i = 0; i < bucketsNumber; i++) {
            res += buckets[i];
        }
        return res;
    }

    /*!
    * \brief Smooth histogram using mean
    * \param windowSize Size of window in which mean is being calculated
    */
    void smooth(int windowSize = 5) {
        int halfWindowSize = windowSize / 2;

        for (size_t i = halfWindowSize; i < bucketsNumber - halfWindowSize; i++) {
            double mean = 0;
            for (size_t j = i - halfWindowSize; j <= (i + halfWindowSize); j++) {
                mean += buckets[j];
            }
            buckets[i] = mean / windowSize;
        }
    }

    /*!
    * \brief Compute intersection with another histogram
    * \param other The second histogram to compute intersection
    */
    double intersect(const NormalizedIntensityHistogram<bucketsNumber> &other) {
        double sum = 0.0;
        for (size_t i = 0; i < bucketsNumber; i++) {
            sum += std::min(buckets[i], other.buckets[i]);
        }
        return sum;
    }

    /*!
      * \brief Compute Otsu two level thresholding
      * \param border1 First class upper border
      * \param border2 Second class upper border
      * \return (bool) true if succesfully separated, false othrewise (image had only 2 colors)
      */
    bool otsu2(double &border1, double &border2)
    {
        int k1 = -1, k2 = -1;

        double sigma_b_max = -1;

        double mu_t = 0;
        for (int i = 0; i < bucketsNumber; i++)
            mu_t += i * buckets[i];

        double omega_0 = 0;
        double mu_0_tmp = 0;
        for (int i = 0; i < bucketsNumber - 1; i++)
        {
            omega_0 += buckets[i];
            mu_0_tmp += i * buckets[i];

            double mu_0 = mu_0_tmp;
            if (omega_0 != 0)
                mu_0 /= omega_0;

            double omega_1 = 0;
            double mu_1_tmp = 0;
            for (int j = i + 1; j < bucketsNumber; j++)
            {
                omega_1 += buckets[j];
                mu_1_tmp += j * buckets[j];
                double mu_1 = mu_1_tmp;
                if (omega_1 != 0)
                    mu_1 /= omega_1;

                double omega_2 = 0;
                double mu_2 = 0;
                for (int k = j + 1; k < bucketsNumber; k++)
                {
                    omega_2 += buckets[k];
                    mu_2 += k * buckets[k];
                } // end of for k

                if (omega_0 == 0 || omega_1 == 0 || omega_2 == 0)
                    continue;

                mu_2 /= omega_2;

                double sigma_b = omega_0 * (sqr(mu_0 - mu_t))
                                  + omega_1 * (sqr(mu_1 - mu_t))
                                  + omega_2 * (sqr(mu_2 - mu_t));

                if (sigma_b > sigma_b_max)
                {
                    k1 = i;
                    k2 = j;
                    sigma_b_max = sigma_b;
                }
            } // end of for j
        } // end of for i

        if (k1 == -1 || k2 == -1)
            return false;

        border1 = (double)k1 / bucketsNumber;
        border2 = (double)k2 / bucketsNumber;

        return true;
    } // End of 'otsu2' function

      /*!
      * \brief Compute image histogram
      * \param image Image to compute for
      */
    void fromImage(const Matrix2D<double> &image)
    {
        for (int i = 0; i < image.sizeY; i++)
            for (int j = 0; j < image.sizeX; j++)
                add(image.get(j, i));

        normalize();
    } // End of 'fromImage' function

    /*!
    * \brief Compute alpha-quantile
    * \param alpha Cumulative distribution threshold
    */
    double quantile(const double alpha) {
        double sum = 0.0;
        for (size_t i = 0; i < bucketsNumber; i++) {
            sum += buckets[i];
            if (sum >= alpha)
                return (double)i / (bucketsNumber - 1);
        }
        return 1.0;
    }

    //! Get most times observed value
    double mostObservedValue()
    {
        return (double)maxBucket_ / (bucketsNumber - 1);
    } // End of 'mostObservedValue' function

    std::array<double, bucketsNumber> buckets = { 0 };
    size_t observationsNumber = 0;
private:
    /*!
    * \brief Compute index of bucket
    * \param val Normalized intensity value
    */
    size_t index(double val) const {
        const size_t ind = static_cast<size_t>(std::ceil(val * (bucketsNumber - 1)));
        if (ind >= bucketsNumber)
            return bucketsNumber - 1;
        if (ind < 0)
            return 0;
        return ind;
    }

    //! Index of bucket with maximum number of observations
    size_t maxBucket_;
};

class SignedDistanceField {
public:
    /*!
      * \brief Compute Signed Distance Field (distance to mask)
      * \param mask Input mask
      */
    virtual Matrix2D<double> compute(const Matrix2D<bool> &mask);
    /*!
      * \brief Compute Signed Distance Field (distance to mask) using image
      * \param image Input image
      * \param mask Input mask
      */
    virtual Matrix2D<double> compute(const Matrix2D<double> &image, const Matrix2D<bool> &mask);
protected:
    void computeGeodesicDistance(const Matrix2D<double> &intensities,
                                  const Matrix2D<bool> &prevMask,
                                  const std::function<double(int, int, int, int)> geodesicImpact,
                                  Matrix2D<double> &result);
};

//! Penalty border finder class
class PenaltyBorder
{
public:
    /*!
      * \brief Class constructor
      * \param probeDistance Intensity probe distance in pixels
      * \param threshold Intensity deviation threshold
      * \param targetIntensity Intensity to compare to
      */
    PenaltyBorder(int probeDistance = 2, double threshold = 0.18, double targetIntensity = -1);

    /*!
      * \brief Remove penalty border pixels from mask
      * \param image Image to find penalty border on
      * \param initialMask Region to probe from
      * \param result Mask to remove pixels from
      */
    virtual void apply(const Matrix2D<double> &image,
        const Matrix2D<bool> &initialMask,
        Matrix2D<bool> &removeFrom);

    /*!
      * \brief Set intensity probe distance
      */
    virtual void setProbeDistance(const double newProbeDistance);

    /*!
      * \brief Set intensity probe threshold
      */
    virtual void setProbeThreshold(const double newProbeThreshold);

protected:
    /*!
      * \brief Find penalty border via intensity probe
      * \param image Image to find penalty border on
      * \param mask Region to probe from
      * \param result Final border penalty mask
      */
    virtual void find(const Matrix2D<double> &image, const Matrix2D<bool> &mask, Matrix2D<bool> &result);

private:
    //! Intensity probe distance in pixels
    int probeDistance_;
    //! Intensity deviation threshold
    double threshold_;
    //! Intensity to compare to
    double targetIntensity_;
    //! Annotated image region intensity histogram
    NormalizedIntensityHistogram<256> labelHistogram_;
};

//! Level Set Method Interface
class LevelSetMethodBase {
public:
    /*!
      * \brief Run region growing
      * \param image Input image / density field
      * \param initial Initial region
      * \param result Output mask
      */
    virtual void run(const Matrix2D<double> &image,
                     const Matrix2D<bool> &initial,
                     Matrix2D<bool> &result) = 0;
    /*!
      * \brief Run region growing
      * \param image Input image / density field
      * \param initial Initial region
      */
    virtual Matrix2D<bool> run(const Matrix2D<double> &image, const Matrix2D<bool> &initial);
protected:
    /*!
      * \brief Convert initial contour mask to rough Signed Distance Field
      * \param initial Initial contour mask
      * \param result Output SDF
      */
    void makeInitialApproximation(const Matrix2D<bool> &initial, Matrix2D<double> &result);
    /*!
      * \brief Convert initial contour mask to smooth Signed Distance Field
      * \param initial Initial contour mask
      * \param result Output SDF
      */
    void makeSmoothInitialApproximation(const Matrix2D<bool> &initial, Matrix2D<double> &result);
private:
    //! window size for level set smoothing
    int smoothWindowSize_ = 7;
};

#endif // LSMBASE_H
