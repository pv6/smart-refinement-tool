#include <smartrefinement.hpp>
#include <queue>
#include <iostream>
#include <vvt/algorithms/generallsm.hpp>
#include <vvt/algorithms/chanveselsm.hpp>

namespace vvt {

    namespace accessory {

        SmartRefinement::SmartRefinement(viewer::base *viewer) :
            PropagationAccessory(viewer),
          smartBrush_(new SmartBrushLevelSetImpl()),
          lsm_(1, 1)
        {
            smartBrush_->connect([this](VoxelPosition p) {
                markVoxel(p);
                smartBrushMarkedVoxels_.push_back(p);
            },
                [this](VoxelPosition p) {
                return getVoxelIntensity(p);
            },
                [this]() { return getSliceMinX(); },
                [this]() { return getSliceMaxX(); },
                [this]() { return getSliceMinY(); },
                [this]() { return getSliceMaxY(); }
            );
            smartBrush_->changeRadius(smartBrush_->getRadius() / 2);
            smartBrush_->startSelection();
            result_ = std::make_unique<Matrix2D<bool>>(1, 1);
        }


        void SmartRefinement::exposeUseCaseCommands(vvt::engine *engine)
        {
            engine->registerCommand(getUseCaseName() + "_setSensitivity", this, &SmartRefinement::changeSensitivity, args_in("sensitivity"));
            engine->registerCommand(getUseCaseName() + "_setBorderPenalty", this, &SmartRefinement::changeBorderPenalty, args_in("sensitivity"));
            engine->registerCommand(getUseCaseName() + "_setSmoothness", this, &SmartRefinement::changeSmoothness, args_in("sensitivity"));
            engine->registerCommand(getUseCaseName() + "_setBrushRadius", this, &SmartRefinement::changeBrushRadius, args_in("radius"));
            engine->registerCommand(getUseCaseName() + "_startNewAnnotation", this, &SmartRefinement::startNewAnnotation);
            engine->registerCommand(getUseCaseName() + "_saveDist", this, &SmartRefinement::saveDist, args_in("fileName"));
            engine->registerCommand(getUseCaseName() + "_saveMask", this, &SmartRefinement::saveMask, args_in("fileName"));
            engine->registerCommand(getUseCaseName() + "_savePhi", this, &SmartRefinement::savePhi, args_in("fileName"));
            engine->registerCommand(getUseCaseName() + "_clearMask", this, &SmartRefinement::onClearMask);
        }

        void SmartRefinement::clearState() {
            clicks_.clear();
            smartBrushMarkedVoxels_.clear();
        }

        void SmartRefinement::onClearMask() {
            clearState();
        }

        void SmartRefinement::startNewAnnotation()
        {
            convertTemporaryPatchToPrimary(); // save the latest patch
            clearState();
        }

        std::string SmartRefinement::getUseCaseName()
        {
            return "SmartRefinement";
        }

        void SmartRefinement::changeSensitivity(double sens) {
            sensitivity_ = sens;
            lsm_.setSensitivity(sens);
        }

        void SmartRefinement::changeSmoothness(double sens) {
            lsm_.setR2Mu(1 - sens);
        }

        void SmartRefinement::changeBrushRadius(int radius) {
            smartBrush_->changeRadius(radius);
        }

        void SmartRefinement::changeBorderPenalty(double sens)
        {
            lsm_.setDF2SS(1 - sens);
        } // End of 'SmartRefinement::changeSSCoef' function

        void SmartRefinement::updateClicksBBox(BBox3D &bbox, const int padding)
        {
            int minX = getSliceMaxX(),
                maxX = getSliceMinX(),
                minY = getSliceMaxY(),
                maxY = getSliceMinY();
            int z = clicks_[0][2];
            
            for (auto &p : clicks_) {
                int x = p[0];
                int y = p[1];

                minX = std::min(minX, x);
                minY = std::min(minY, y);
                maxX = std::max(maxX, x);
                maxY = std::max(maxY, y);
            }

            //if (minY > maxY) {
            //    minY = getSliceMinY();
            //    maxY = getSliceMaxY();
            //}

            bbox = {
                VoxelPosition({
                    std::max(minX - padding, getSliceMinX()),
                    std::max(minY - padding, getSliceMinY()),
                    z
                }),
                VoxelPosition({
                    std::min(maxX + padding, getSliceMaxX()),
                    std::min(maxY + padding, getSliceMaxY()),
                    z
                })
            };
        } // end of 'SmartRefinement::updateClicksBBox' function

        void SmartRefinement::wholeShabang()
        {
            if (clicks_.size() >= 3) {
                BBox3D bbox;
                std::cout << "I am here now\n";

                const int padding = smartBrush_->getRadius(); // 3;
                /*
                if (!updateSliceBBox(bbox, voxelPos[2], padding)) {
                std::cout << "Can't find at least one marked voxel" << std::endl;
                return;
                }
                */

                // find bbox using clicks_
                updateClicksBBox(bbox, padding);
                bbox_ = bbox;
                bbox_[0][0] -= 1;
                bbox_[0][1] += 1;
                bbox_[1][0] -= 1;
                bbox_[1][1] += 1;

                /*seedPoints_.clear();
                for (auto &seed : clicks_) {
                seedPoints_.push_back(seed);
                }*/
                seedPoints_ = clicks_;

                refine(bbox);

                time_t tmp;
                timer_ = time(&tmp) - timer_;
            }
        } // end of 'SmartRefinement::wholeShabang' function


        void SmartRefinement::OnNewLabel2D(std::array<int, 3> &voxelPos)
        {
            // seedPoints_.push_back(voxelPos);
            time(&timer_);

            std::cout << "I am here, clicks size = " << clicks_.size() << std::endl;

            clicks_.push_back(voxelPos);
            drawSmartBrush(voxelPos);
            wholeShabang();
        }

        void SmartRefinement::drawSmartBrush(std::array<int, 3> &voxelPos) {
            // startNewLabeling(); // PATCH_TYPE_TEMP);
            startNewLabeling(PATCH_TYPE_TEMP);
            setPrimaryLabel();
            for (auto &p : clicks_) {
                smartBrush_->draw(p, smartBrush_->getRadius());
            }
            // smartBrush_->draw(voxelPos, smartBrush_->getRadius());
            endNewLabeling();
        }

        void SmartRefinement::complementMask(Matrix2D<bool> &mask) {
            struct Vec {
                Vec() : x(0), y(0) {}
                Vec(const Vec &vec) : x(vec.x), y(vec.y) {}
                Vec(const int _x, const int _y) : x(_x), y(_y) {}
                Vec operator-(const Vec& rhs) {
                    return Vec(x - rhs.x, y - rhs.y);
                }
                Vec cross() {
                    return Vec(-y, x);
                }
                int operator*(const Vec& rhs) {
                    return x * rhs.x + y * rhs.y;
                }
                int x;
                int y;
            };
            std::vector<Vec> rectPoints;

            // Add border points
            /*
            for (int y = 0; y < mask.sizeY; y++) {
                for (int x = 0; x < mask.sizeX; x++) {
                    if (mask.get(x, y) && !mask.get(x - 1, y) && !mask.get(x - 1, y - 1) && !mask.get(x, y - 1)) {
                        rectPoints.push_back(Vec(x, y));
                    }
                }
            }
            */
            for (auto &p : seedPoints_) {
                rectPoints.push_back(Vec(p[0], p[1]));
            }

            if (rectPoints.size() <= 1) {
                return;
            }

            // Sort points by x coordinate
            std::sort(rectPoints.begin(), rectPoints.end(), [](auto a, auto b) {
                if (a.x == b.x)
                    return a.y < b.y;
                return a.x < b.x;
            });
            Vec normal = (rectPoints.back() - rectPoints.front()).cross();
            Vec left = rectPoints.front();
            std::vector<Vec> poly;
            // Copy points placed below the first->last line
            auto beg = std::copy_if(rectPoints.begin(), rectPoints.end(), std::back_inserter(poly), [&normal, &left](auto p) {
                return normal * (left - p) >= 0;
            });
            // Copy points placed above first->last line
            std::copy_if(rectPoints.rbegin(), rectPoints.rend(), beg, [&normal, &left](auto p) {
                return normal * (left - p) < 0;
            });

            // Even-odd rule for polygon rasterization
            std::function<bool(Vec&)> isPointInPath = [&poly](Vec& p) -> bool
            {
                size_t num = poly.size();
                size_t end = num - 1;
                bool res = false;
                for (size_t i = 0; i < num; i++)
                {
                    // p.y is between i and end points
                    bool c1 = (poly[i].y > p.y) != (poly[end].y > p.y);
                    Vec diff = poly[end] - poly[i];
                    if (c1 && 
                        ((poly[i].x * diff.y + diff.x * (p - poly[i]).y > p.x * diff.y) == (diff.y > 0)))
                    {
                        res = !res;
                    }
                    end = i;
                }

                return res;
            };

            // Rasterize polygon
            for (int y = 0; y < mask.sizeY; y++) {
                for (int x = 0; x < mask.sizeX; x++) {
                    mask.set(x, y, mask.get(x, y) || isPointInPath(Vec(x, y)));
                }
            }
        }

        void SmartRefinement::refine(BBox3D &bbox)
        {
            startNewLabeling(PATCH_TYPE_TEMP);
            setPrimaryLabel();

            // transform seed points
            for (VoxelPosition &seedPoint : seedPoints_)
                seedPoint =
                {
                    seedPoint[0] - bbox[0][0],
                    seedPoint[1] - bbox[0][1]
                };

            const int height = bbox[1][1] - bbox[0][1],
                      width = bbox[1][0] - bbox[0][0];
            const int z = bbox[0][2];

            Matrix2D<bool> mask(width, height);
            Matrix2D<bool> trustedMask(width, height);

            // load mask
            for (int y = 0; y <= height; y++)
                for (int x = 0; x <= width; x++)
                {
                    mask.set(
                        x,
                        y,
                        isLabeled(x + bbox[0][0], y + bbox[0][1], z)
                    );
                    trustedMask.set(
                        x,
                        y,
                        isLabeled(x + bbox[0][0], y + bbox[0][1], z)
                    );
                }

            for (const auto& p : smartBrushMarkedVoxels_)
            {
                const int x = p[0] - bbox[0][0];
                const int y = p[1] - bbox[0][1];
                if (x < 0 || y < 0 || x > width || y > height)
                    continue;
                mask.set(x, y, true);
                trustedMask.set(x, y, true);
            }

            complementMask(mask);
            Matrix2D<bool> result(width, height);

            Matrix2D<double> sliceImage(width, height);
            NormalizationParameters initialSliceNorm;
            NormalizeImage(bbox, sliceImage, initialSliceNorm);

            initLSM(sliceImage, mask);

            refineSlice(sliceImage, mask, trustedMask, result);

            mask.fill(false);
            removeLeaks(mask, result);

            result_ = std::make_unique<Matrix2D<bool>>(mask);

            markMask(bbox[0], mask);
            // markMask(bbox[0], result);
            endNewLabeling();
            //flushMarkedVoxels();
        }

        void SmartRefinement::initLSM(const Matrix2D<double> &sliceImage, const Matrix2D<bool> &mask)
        {
        } // End of 'SmartRefinement::initLSM' function

        void SmartRefinement::refineSlice(const Matrix2D<double> &sliceImage,
                                          const Matrix2D<bool> &mask,
                                          const Matrix2D<bool> &trustedMask,
                                          Matrix2D<bool> &result)
        {
            /*Matrix2D<double> densityField(sliceImage.sizeX, sliceImage.sizeY);
            const double threshold = calcThreshold(sliceImage, mask);
            const double eps = calcEps(sliceImage, threshold, mask);
            initDensityField(sliceImage, eps, threshold, densityField);
            GeneralLevelSet lsm_ = GeneralLevelSet(sliceImage.sizeX, sliceImage.sizeY);
            lsm_.setAlpha(0.75);
            lsm_.setIterationsDivider(1);
            std::cout << "threshold: " << threshold << std::endl;
            std::cout << "eps: " << eps << std::endl;
            lsm_.run(densityField, mask, result);*/
            lsm_.resize(sliceImage.sizeX, sliceImage.sizeY);

            // lsm_.setTheta(50);
            // lsm_.setEuclIncrease(10);

            std::cout << "I am being refined over here" << std::endl;

            //lsm_.setLambda(std::exp(sensitivity_ * 10.0));
            //lsm_.setEuclIncrease(std::exp(sensitivity_ * 2.0));
            lsm_.setInputSlice(sliceImage);
            lsm_.setMaxIterations(100);
            // If mask is passed instead of trustedMask, then area between clicks will be filled
            lsm_.run(sliceImage, trustedMask, result, [&trustedMask, &mask, this]() -> double {
                return lsm_.calculateC(trustedMask);
                //return c;
                //double c1 = lsm_.calculateC(trustedMask);
                //double c2 = lsm_.calculateC(mask);
                //const double alpha = sensitivity_;
                //return c1 * alpha + c2 * (1 - alpha);
            });
        }

        double SmartRefinement::calcEps(const Matrix2D<double> &sliceImage, const double threshold, const Matrix2D<bool> &mask)
        {
            NormalizedIntensityHistogram<255> hist;
            double max = std::numeric_limits<double>::min();
            for (int y = 0; y < sliceImage.sizeY; y++) {
                for (int x = 0; x < sliceImage.sizeX; x++) {
                    if (mask.get(x, y)) {
                        const double dist = fabs(sliceImage.get(x, y) - threshold);
                        hist.add(dist);
                        max = std::max(max, dist);
                    }
                }
            }
            hist.normalize();
            std::cout << "quantile: " << hist.quantile(0.85) << std::endl;
            std::cout << "max: " << max << std::endl;
            return hist.quantile(0.85);
        }

        double SmartRefinement::calcThreshold(const Matrix2D<double> &sliceImage, const Matrix2D<bool> &mask)
        {
            std::vector<double> intensities;
            for (int y = 0; y < sliceImage.sizeY; y++) {
                for (int x = 0; x < sliceImage.sizeX; x++) {
                    if (mask.get(x, y)) {
                        intensities.push_back(sliceImage.get(x, y));
                    }
                }
            }
            const auto median_it = intensities.begin() + intensities.size() / 2;
            std::nth_element(intensities.begin(), median_it, intensities.end());
            if (&(*median_it) == nullptr)
                return -1;
            double median = *median_it;
            return median;
        } // End of '' function

        void SmartRefinement::initDensityField(const Matrix2D<double> &sliceImage, const double eps, const double thresh, Matrix2D<double> &dField)
        {
            for (int y = 0; y < dField.sizeY; y++) {
                for (int x = 0; x < dField.sizeX; x++) {
                    double value = sliceImage.get(x, y);
                    const double density = eps - fabs(value - thresh);
                    dField.set(x, y, density);
                }
            }
        } // End of 'SmartRefinement::initDensityField' function

        void SmartRefinement::saveDist(const std::string &fileName)
        {
            printf("save distance\n");
            saveImage(fileName, *lsm_.dist_);
        } // End of 'SmartRefinement::saveDist' function

        void SmartRefinement::saveMask(const std::string &fileName)
        {
            printf("save mask\n");
            saveImage(fileName, *result_);
            startNewAnnotation();
        } // End of 'SmartRefinement::saveMask' function

        void SmartRefinement::savePhi(const std::string &fileName)
        {
            printf("save phi\n");
            saveImage(fileName, *lsm_.phi_);
        } // End of 'SmartRefinement::savePhi' function
    } // end of 'accessory' namespace
} // end of 'vvt' namespace
