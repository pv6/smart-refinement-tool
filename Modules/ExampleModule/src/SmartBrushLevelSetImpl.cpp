#include <smartbrushlevelsetimpl.hpp>
#include <queue>
#include <unordered_set>
#include <memory>
#include <cfloat>

void SmartBrushLevelSetImpl::connect(std::function<void(VoxelPosition)> markVoxel,
    std::function<double(VoxelPosition)> getVoxelIntensity,
    std::function<int()> getSliceMinX,
    std::function<int()> getSliceMaxX,
    std::function<int()> getSliceMinY,
    std::function<int()> getSliceMaxY)
{
    markVoxel_ = markVoxel;
    getVoxelIntensity_ = getVoxelIntensity;
    getSliceMinX_ = getSliceMinX;
    getSliceMaxX_ = getSliceMaxX;
    getSliceMinY_ = getSliceMinY;
    getSliceMaxY_ = getSliceMaxY;
}

void SmartBrushLevelSetImpl::startSelection()
{
    lsm_ = std::unique_ptr<GeneralLevelSet>(new GeneralLevelSet(radius_ * 2, radius_ * 2));
}

void SmartBrushLevelSetImpl::OnNewLabel2D(std::array<int, 3> &voxelPos)
{
    draw(voxelPos, radius_);
}

int SmartBrushLevelSetImpl::getRadius() {
    return radius_;
}

void SmartBrushLevelSetImpl::changeRadius(const int radius) {
    const int minRadius = 3;
    if (radius < minRadius)
        return;
    this->radius_ = radius;
    lsm_.reset(new GeneralLevelSet(radius * 2, radius * 2));
}

void SmartBrushLevelSetImpl::changeSensitivity(const double sensitivity) {
    sensitivity_ = sensitivity;
}

void SmartBrushLevelSetImpl::changeCurvature(const double alpha) {
    lsm_->setAlpha(alpha);
}

void SmartBrushLevelSetImpl::makeInitialCircle(Matrix2D<bool> &contour, const int radius) {
    std::array<int, 2> center = { contour.sizeX / 2, contour.sizeY / 2 };
    const int r2 = radius * radius;
    const int radiusX = std::min(contour.sizeX / 2, radius);
    const int radiusY = std::min(contour.sizeY / 2, radius);
    for (int i = center[0] - radiusX; i <= center[0] + radiusX; i++) {
        for (int j = center[1] - radiusY; j <= center[1] + radiusY; j++) {
            const int x = center[0] - i;
            const int y = center[1] - j;
            contour.set(i, j, x * x + y * y <= r2);
        }
    }
}

void SmartBrushLevelSetImpl::markMask(const VoxelPosition &leftCorner, const Matrix2D<bool> &mask) {
    for (int y = 0; y < mask.sizeY; y++) {
        for (int x = 0; x < mask.sizeX; x++) {
            const int i = leftCorner[0] + x;
            const int j = leftCorner[1] + y;

            if ((i >= getSliceMinX_() && i <= getSliceMaxX_() &&
                j >= getSliceMinY_() && j <= getSliceMaxY_())) {
                if (mask.get(x, y))
                    markVoxel_(std::array<int, 3>({ i, j, leftCorner[2] }));
            }
        }
    }
}

void SmartBrushLevelSetImpl::initDensityField(const BBox3D &bbox, const double eps, const double thresh, Matrix2D<double> &dField) {
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::min();

    for (int y = 0; y < dField.sizeY; y++) {
        for (int x = 0; x < dField.sizeX; x++) {
            const int xAbsPos = bbox[0][0] + x;
            const int yAbsPos = bbox[0][1] + y;
            
            if (xAbsPos < getSliceMinX_() || xAbsPos > getSliceMaxX_() ||
                yAbsPos < getSliceMinY_() || yAbsPos > getSliceMaxY_())
                continue; // ignore pixels outside actual image
            double value = getVoxelIntensity_(std::array<int, 3>({
                xAbsPos,
                yAbsPos,
                bbox[0][2]
            }));
            if (value > max)
                max = value;
            if (value < min)
                min = value;
        }
    }

    if (max == min) { // in case if region is completely homogeneous
        min = max - 1.0;
    }

    double threshold = (thresh - min) / (max - min); // normalize threshold

    for (int y = 0; y < dField.sizeY; y++) {
        for (int x = 0; x < dField.sizeX; x++) {
            const int xAbsPos = bbox[0][0] + x;
            const int yAbsPos = bbox[0][1] + y;

            if (xAbsPos < getSliceMinX_() || xAbsPos > getSliceMaxX_() ||
                yAbsPos < getSliceMinY_() || yAbsPos > getSliceMaxY_()) {
                dField.set(x, y, 0.0); // set zero density to ignore pixels outside actual image
                continue;
            }

            double value = getVoxelIntensity_(std::array<int, 3>({
                xAbsPos,
                yAbsPos,
                bbox[0][2]
            }));

            value = (value - min) / (max - min); // normalize value
            
            const double density = eps - fabs(value - threshold);
            dField.set(x, y, density);
        }
    }
}

double SmartBrushLevelSetImpl::getMedian(std::vector<double> &values) {
    const auto median_it = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), median_it, values.end());
    double median = *median_it;
    return median;
}

double SmartBrushLevelSetImpl::calcThreshold(const BBox3D &bbox, const int padding) {
    std::vector<double> intensities;
    std::array<int, 2> center = { (bbox[1][0] + bbox[0][0]) / 2, (bbox[1][1] + bbox[0][1]) / 2 };
    for (int i = center[0] - padding; i <= center[0] + padding; i++) {
        for (int j = center[1] - padding; j <= center[1] + padding; j++) {
            if (i >= getSliceMinX_() && i <= getSliceMaxX_() &&
                j >= getSliceMinY_() && j <= getSliceMaxY_()) {
                double value = getVoxelIntensity_(std::array<int, 3>({
                    i,
                    j,
                    bbox[0][2]
                }));
                intensities.push_back(value);
            }
        }
    }
    return getMedian(intensities);
}

void SmartBrushLevelSetImpl::draw(const VoxelPosition &voxelPos, const int radius)
{
    if (!lsm_)
        startSelection();

    // set bounding box that contains illegal pixel coordinates too
    BBox3D bbox = {
        VoxelPosition({
        voxelPos[0] - radius,
        voxelPos[1] - radius,
        voxelPos[2]
    }),
        VoxelPosition({
        voxelPos[0] + radius,
        voxelPos[1] + radius,
        voxelPos[2]
    })
    };

    Matrix2D<bool> mask(bbox[1][0] - bbox[0][0], bbox[1][1] - bbox[0][1]);

    const int initialCircleRadius = 5;
    makeInitialCircle(mask, initialCircleRadius);

    // get threshold from middle pixel intensity
    double threshold = getVoxelIntensity_(std::array<int, 3>({
        (bbox[0][0] + bbox[1][0]) / 2,
        (bbox[0][1] + bbox[1][1]) / 2,
        bbox[0][2]
    }));
    // double threshold = calcThreshold(bbox, 4); // calculate threshold using median intensity


    Matrix2D<double> densityField(bbox[1][0] - bbox[0][0], bbox[1][1] - bbox[0][1]);
    const double sens = sensitivity_ / 2.0 + 0.5;
    const double eps = 1 - sens;
    
    initDensityField(bbox, eps, threshold, densityField);

    const bool downsamplingEnabled = downsamplingRatio_ > 0;
    
    if (!downsamplingEnabled) {
        Matrix2D<bool> result = lsm_->run(densityField, mask);
        Matrix2D<bool> brushMask = removeLeaksInCircle(result, radius);
        markMask(bbox[0], brushMask);
    } else {
        Downsampler<GeneralLevelSet> d(downsamplingRatio_);
        d.setAlpha(lsm_->getAlpha());
        Matrix2D<bool> result = d.run(densityField, mask);
        Matrix2D<bool> brushMask = removeLeaksInCircle(result, radius);
        markMask(bbox[0], brushMask);
    }
}

inline double sqr(double x) {
    return x * x;
}

Matrix2D<bool> SmartBrushLevelSetImpl::removeLeaksInCircle(Matrix2D<bool> &mask, const int radius) {
    Matrix2D<bool> result(mask.sizeX, mask.sizeY);
    using Vertex = std::array<int, 2>;
    std::queue<Vertex> queue;
    const std::vector<Vertex> neighbors = { { 1, 0 },{ 0, 1 },{ -1, 0 },{ 0, -1 } };
    Vertex center = { mask.sizeX / 2, mask.sizeY / 2 };
    int radius2 = radius * radius;
    queue.push(center);

    while (!queue.empty()) {
        Vertex node = queue.front();
        queue.pop();
        if (result.get(node[0], node[1]))
            continue;
        result.set(node[0], node[1], true);
        for (auto n : neighbors) {
            Vertex coords = { node[0] + n[0], node[1] + n[1] };
            if (sqr(coords[0] - center[0]) + sqr(coords[1] - center[1]) >= radius2)
                continue;
            if (mask.get(coords[0], coords[1]) && !result.get(coords[0], coords[1]))
                queue.push(coords);
        }
    }

    return result;
}
