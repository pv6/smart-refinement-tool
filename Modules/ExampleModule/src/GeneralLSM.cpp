#include <generallsm.hpp>

#include <algorithm>
#include <queue>
#include <unordered_set>

inline double sqr(double x) {
    return x * x;
}

void GeneralLevelSet::updateCurvature(const Matrix2D<double>& phi, const BBox2D &bbox, Matrix2D<double> &curvature) {
    for (int y = bbox[0][1]; y < bbox[1][1]; y++) {
        for (int x = bbox[0][0]; x < bbox[1][0]; x++) {
            const double value = phi.get(x, y);
#if defined(ONLYGROW) || defined(NOINNERCURVATURE)
            if (value >= 0)
                continue;
#endif
            const double
                right = phi.get(x + 1, y),
                left = phi.get(x - 1, y),
                up = phi.get(x, y - 1),
                down = phi.get(x, y + 1);

            double dx = (right - left) / 2.0;
            double dy = (up - down) / 2.0;
            double dxplus = right - value;
            double dyplus = up - value;
            double dxminus = value - left;
            double dyminus = value - down;

            double dxplusy = (phi.get(x + 1, y - 1) - phi.get(x - 1, y - 1)) / 2.0;
            double dxminusy = (phi.get(x + 1, y + 1) - phi.get(x - 1, y + 1)) / 2.0;
            double dyplusx = (phi.get(x + 1, y - 1) - phi.get(x + 1, y + 1)) / 2.0;
            double dyminusx = (phi.get(x - 1, y - 1) - phi.get(x - 1, y + 1)) / 2.0;

            double nplusx = dxplus / sqrt(curvEps_ + sqr(dxplus) + sqr((dyplusx + dy) / 2.0));
            double nplusy = dyplus / sqrt(curvEps_ + sqr(dyplus) + sqr((dxplusy + dx) / 2.0));
            double nminusx = dxminus / sqrt(curvEps_ + sqr(dxminus) + sqr((dyminusx + dy) / 2.0));
            double nminusy = dyminus / sqrt(curvEps_ + sqr(dyminus) + sqr((dxminusy + dx) / 2.0));

            curvature.set(x, y, ((nplusx - nminusx) + (nplusy - nminusy)) / 2.0);
        }
    }
}

int GeneralLevelSet::calculateIterations(int x, int y) {
    if (std::max(x, y) > 30) {
        return 260 / iterationsDivider_;
    }
    return 200 / iterationsDivider_;
}

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

GeneralLevelSet::GeneralLevelSet(const int sizeX, const int sizeY) {
    phi_ = make_unique<Matrix2D<double>>(sizeX, sizeY);
    curvature_ = make_unique<Matrix2D<double>>(phi_->sizeX, phi_->sizeY);
    Fgradphi_ = make_unique<Matrix2D<double>>(phi_->sizeX, phi_->sizeY);
}

Matrix2D<bool> GeneralLevelSet::run(const Matrix2D<double> &densityField, const Matrix2D<bool> &initial)
{
    Matrix2D<bool> result(densityField.sizeX, densityField.sizeY);
    run(densityField, initial, result);
    return result;
}

/*
* Position coordinates encoding/decoding to/from int
*/
struct Pos {
    Pos(int _x, int _y) : x(_x), y(_y) {}
    Pos(int hash) {
        x = hash % yShift;
        y = hash / yShift;
    }
    operator int() {
        return x + y * yShift;
    }
    int x;
    int y;
private:
    const int yShift = 2048;
};

void GeneralLevelSet::run(const Matrix2D<double> &densityField, const Matrix2D<bool> &initial, Matrix2D<bool> &result) {

    PixelPosition center = { densityField.sizeX / 2, densityField.sizeY / 2 };

    // important: size of phi-matrix shold not be less than size of initial
    makeInitialApproximation(initial, *phi_);

    int iterations = calculateIterations(densityField.sizeX, densityField.sizeY);
    const int iterationsWithoutNarrowBand = 30;
    const double zeroEps = 0.01;
    const double gradPhiEps = 0.02;
    const double narrowBandInitialRatio = 0.8;
    const int stride = 8;

    std::unordered_set<int> aFront;
    std::unordered_set<int> bFront;
    std::unordered_set<int> wholeMatrix;

    std::unordered_set<int> * front = &aFront;
    std::unordered_set<int> * newFront = &bFront;

    BBox2D bbox = {
        PixelPosition{ 0, 0 },
        PixelPosition{ densityField.sizeX, densityField.sizeY }
    };
    std::vector<BBox2D> boxes{ bbox };

    // make initial front:
    for (int y = bbox[0][1]; y < bbox[1][1]; y++) {
        for (int x = bbox[0][0]; x < bbox[1][0]; x++) {
            if (fabs(phi_->get(x, y)) <= zeroEps) {
                front->emplace(Pos(x / stride, y / stride));
            }
            wholeMatrix.emplace(Pos(x / stride, y / stride));
        }
    }
    for (int py = 0; py < bbox[1][1] / stride; py++) {
        for (int px = 0; px < bbox[1][0] / stride; px++) {
            const int yMAX = std::min(bbox[1][1], (py + 1) * stride);
            const int xMAX = std::min(bbox[1][0], (px + 1) * stride);
            double sum = 0.0;
            for (int y = py * stride; y < yMAX; y++) {
                for (int x = px * stride; x < xMAX; x++) {
                    sum += phi_->get(x, y);
                }
            }
            if (fabs(sum) <= (stride * stride) * narrowBandInitialRatio) {
                front->emplace(Pos(px, py));
            }
        }
    }


    for (int iter = 0; iter < iterations; iter++) {
        if (iter < iterationsWithoutNarrowBand) {
            front = &wholeMatrix;
        }

        double maxFgradphi = DBL_MIN;
        for (auto bbox : boxes) {
            if (alpha_ < 1.0) {
                updateCurvature(*phi_, bbox, *curvature_);
            }
            /*
            Two steps:
            - Calculate (F * gradient_of_phi) and find max
            - Calculate appendix and update phi
            */

            // find finite-difference approximation of phi gradient absolute value:

            newFront->clear();

            for (auto pt : *front) {
                const Pos p = pt;
                const int yMAX = std::min(bbox[1][1], (p.y + 1) * stride);
                const int xMAX = std::min(bbox[1][0], (p.x + 1) * stride);
                double currMaxFgradPhi = DBL_MIN;

                for (int y = p.y * stride; y < yMAX; y++) {
                    for (int x = p.x * stride; x < xMAX; x++) {

                        double value = phi_->get(x, y);

                        double F = densityField.get(x, y) * alpha_ + curvature_->get(x, y) * (1 - alpha_);

                        double dxplus = phi_->get(x + 1, y) - value;
                        double dyplus = phi_->get(x, y - 1) - value;
                        double dxminus = value - phi_->get(x - 1, y);
                        double dyminus = value - phi_->get(x, y + 1);

                        double gradphimax_x_sqr = (sqr(std::max(dxplus, 0.)) + sqr(std::max(-dxminus, 0.)));
                        double gradphimin_x_sqr = (sqr(std::min(dxplus, 0.)) + sqr(std::min(-dxminus, 0.)));
                        double gradphimax_y_sqr = (sqr(std::max(dyplus, 0.)) + sqr(std::max(-dyminus, 0.)));
                        double gradphimin_y_sqr = (sqr(std::min(dyplus, 0.)) + sqr(std::min(-dyminus, 0.)));

                        double gradphi;
                        if (F > 0) {
                            gradphi = sqrt(gradphimax_x_sqr + gradphimax_y_sqr);
                        }
                        else {
                            gradphi = sqrt(gradphimin_x_sqr + gradphimin_y_sqr);
                        }

                        const double FmulGradphi = fabs(F * gradphi);
                        if (FmulGradphi > currMaxFgradPhi) {
                            currMaxFgradPhi = FmulGradphi;
                        }

                        Fgradphi_->set(x, y, F * gradphi);
                    }
                }

                // add squares to the new front:
                if (currMaxFgradPhi > gradPhiEps) {
                    newFront->emplace(Pos(p.x, p.y));
                    newFront->emplace(Pos(std::max(0, p.x - 1), p.y));
                    newFront->emplace(Pos(p.x, std::max(0, p.y - 1)));
                    newFront->emplace(Pos(std::min(bbox[1][0] / stride, p.x + 1), p.y));
                    newFront->emplace(Pos(p.x, std::min(bbox[1][1] / stride, p.y + 1)));
                    newFront->emplace(Pos(std::max(0, p.x - 1), std::max(0, p.y - 1)));
                    newFront->emplace(Pos(std::min(bbox[1][0] / stride, p.x + 1), std::max(0, p.y - 1)));
                    newFront->emplace(Pos(std::min(bbox[1][0] / stride, p.x + 1), std::min(bbox[1][1] / stride, p.y + 1)));
                    newFront->emplace(Pos(std::max(0, p.x - 1), std::min(bbox[1][1] / stride, p.y + 1)));
                }
                if (currMaxFgradPhi > maxFgradphi) {
                    maxFgradphi = currMaxFgradPhi;
                }
            }
        }

        double dt = dx_ / maxFgradphi;
        double sumAppendix = 0;
        double realZeroEps = zeroEps;

        for (auto bbox : boxes) {
            for (auto pt : *front) {
                const Pos p = pt;
                const int yMAX = std::min(bbox[1][1], (p.y + 1) * stride);
                const int xMAX = std::min(bbox[1][0], (p.x + 1) * stride);

                for (int y = p.y * stride; y < yMAX; y++) {
                    for (int x = p.x * stride; x < xMAX; x++) {
                        double value = phi_->get(x, y);
                        double appendix = dt * Fgradphi_->get(x, y);
                        sumAppendix += fabs(appendix);
                        double newValue = value + appendix;

                        phi_->set(x, y, newValue);
                        if (fabs(phi_->get(x, y)) <= realZeroEps) {

                        }
                    }
                }
            }
        }
        if (iter >= 50 && sumAppendix < 2.0) { // 50 - to make phi function smoother
            break;
        }

        // swap fronts:
        front = (iter % 2 == 0) ? &bFront : &aFront;
        newFront = (iter % 2 != 0) ? &bFront : &aFront;
    }

    // find result contour as a non-negative part of phi matrix
    for (int y = 0; y < result.sizeY; y++) {
        for (int x = 0; x < result.sizeX; x++) {
            result.set(x, y, phi_->get(x, y) >= 0.0);
        }
    }
}
