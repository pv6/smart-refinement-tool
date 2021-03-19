#include <LsmBase.hpp>

#include <algorithm>
#include <queue>

template<typename T>
    inline T sqr(T x)
    {
        return x * x;
    }

/*!
  * \brief Class constructor
  * \param probeDistance Intensity probe distance in pixels
  * \param threshold Intensity deviation threshold
  * \param targetIntensity Intensity to compare to
  */
PenaltyBorder::PenaltyBorder(int probeDistance, double threshold, double targetIntensity)
    : probeDistance_(probeDistance), threshold_(threshold), targetIntensity_(targetIntensity)
{
} // End of 'PenaltyBorder::PenaltyBorder' function

/*!
  * \brief Set intensity probe distance
  */
void PenaltyBorder::setProbeDistance(const double newProbeDistance)
{
    printf("set probe distance = %d\n", (int)newProbeDistance);
    probeDistance_ = (int) newProbeDistance;
} // End of 'PenaltyBorder::setProbeDistance' function

/*!
  * \brief Set intensity probe threshold
  */
void PenaltyBorder::setProbeThreshold(const double newProbeThreshold)
{
    printf("set probe distance = %f\n", newProbeThreshold);
    threshold_ = newProbeThreshold;
} // End of 'PenaltyBorder::setProbeThreshold' function

/*!
  * \brief Remove penalty border pixels from mask
  * \param image Image to find penalty border on
  * \param initialMask Region to probe from
  * \param result Mask to remove pixels from
  */
void PenaltyBorder::apply(const Matrix2D<double> &image,
                          const Matrix2D<bool> &initialMask,
                          Matrix2D<bool> &removeFrom)
{
    for (int y = 0; y < image.sizeY; y++)
        for (int x = 0; x < image.sizeX; x++)
            if (initialMask.get(x, y))
                labelHistogram_.add(image.get(x, y));

    labelHistogram_.normalize();

    if (targetIntensity_ == -1)
        targetIntensity_ = labelHistogram_.mostObservedValue();

    Matrix2D<bool> border(image.sizeX, image.sizeY);
    find(image, initialMask, border);


    int neighbour[][2] = { { 1, 0 },{ -1, 0 },{ 0, 1 },{ 0, -1 },
        { -1, -1 },{ 1, -1 },{ 1, 1 },{ -1, 1 } };

    for (int y = 0; y < removeFrom.sizeY; y++)
        for (int x = 0; x < removeFrom.sizeX; x++)
            removeFrom.set(x, y, !border.get(x, y) && removeFrom.get(x, y));
} // End of 'PenaltyBorder::apply' function

/*!
  * \brief Find penalty border via intensity probe
  * \param image Image to find penalty border on
  * \param mask Region to probe from
  * \param result Final border penalty mask
  */
void PenaltyBorder::find(const Matrix2D<double> &image, const Matrix2D<bool> &mask, Matrix2D<bool> &result)
{
    using Vertex = std::array<int, 2>;

    std::deque<Vertex> active;
    std::deque<Vertex> waiting;
    int h = image.sizeY, w = image.sizeX;

    Matrix2D<bool> visited(w, h);
    visited.fillPadding(true);

    const int neighbour[][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                          {-1, -1}, {1, -1}, {1, 1}, {-1, 1}};

    Matrix2D<int> borderMap(w, h);
    borderMap.fill(-1);
    borderMap.fillPadding(-1);

    auto get = [](const Matrix2D<bool> &mask, int x, int y, bool def) -> bool
    {
        if (x < 0 || x >= mask.sizeX || y < 0 || y >= mask.sizeY)
            return def;
        return mask.get(x, y);
    };

    // Find mask border and mark mask vertices as visited
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
        {
            if (mask.get(x, y))
            {
                visited.set(x, y, true);
                for (int i = 0; i < 4; i++)
                {
                    int x_n = x + neighbour[i][0];
                    int y_n = y + neighbour[i][1];

                    if (!visited.get(x_n, y_n) && !get(mask, x_n, y_n, true))
                    {
                        active.push_back({ x_n, y_n });
                        borderMap.set(x_n, y_n, 0);
                        visited.set(x_n, y_n, true);
                    }
                }
            }
        }

    // Perform intensity probe
    std::deque<Vertex> finalBorderQueue;

    double lowerQuantile = labelHistogram_.quantile(0.01);
    double upperQuantile = labelHistogram_.quantile(0.99);

    printf("lQ = %f\n", lowerQuantile);
    printf("uQ = %f\n", upperQuantile);

    int curLevel = 0;
    while (active.size() > 0 || waiting.size() > 0)
    {
        if (active.size() == 0)
        {
            active = std::move(waiting);
            curLevel += 1;
        }

        // get current active vertex
        Vertex vertex = active.front();
        active.pop_front();

        // mark it on border map
        borderMap.set(vertex[0], vertex[1], curLevel);
        // save it to final extended border if probing reached max distance
        if (curLevel == probeDistance_)
        {
            finalBorderQueue.push_back(vertex);
            continue;
        }

        // check if need to stop
        //if (fabs(image.get(vertex[0], vertex[1]) - targetIntensity_) > threshold_)
        double vertVal = image.get(vertex[0], vertex[1]);
        if (vertVal < lowerQuantile - threshold_ || vertVal > upperQuantile + threshold_)
            continue;

        // add not visited neighbours
        for (int i = 0; i < 8; i++)
        {
            int nX = vertex[0] + neighbour[i][0];
            int nY = vertex[1] + neighbour[i][1];
            if (!visited.get(nX, nY))
            {
                visited.set(nX, nY, true);
                waiting.push_back({ nX, nY });
            }
        } // end of for i
    } // end of while

    // Return back to level 0
    while (waiting.size() > 0)
        waiting.pop_front();
    active = std::move(finalBorderQueue);
    visited.fill(false);
    visited.fillPadding(true);
    for (int i = 0; i < active.size(); i++)
        visited.set(active[i][0], active[i][1], true);

    while (active.size() > 0 || waiting.size() > 0)
    {
        if (active.size() == 0)
            active = std::move(waiting);

        Vertex vertex = std::move(active.front());
        active.pop_front();

        if (!borderMap.get(vertex[0], vertex[1]))
        {
            finalBorderQueue.push_back(vertex);
            //continue;
        }

        for (int i = 0; i < 8; i++)
        {
            int nX = vertex[0] + neighbour[i][0];
            int nY = vertex[1] + neighbour[i][1];

            if (!visited.get(nX, nY)
                && borderMap.get(nX, nY) == borderMap.get(vertex[0], vertex[1]) - 1)
            {
                visited.set(nX, nY, true);
                waiting.push_back({ nX, nY });
            }
        } // end of for
    } // end of while

    result.fill(false);
    for (int i = 0; i < finalBorderQueue.size(); i++)
        result.set(finalBorderQueue[i][0], finalBorderQueue[i][1], true);

    // Fuse together close finalBorderQueue fragments
    std::function<bool(Vertex, int)> fuse;
    fuse = [&](Vertex curPixel, int distance)
    {
        result.set(curPixel[0], curPixel[1], true);
        bool isEdge = false;
        Vertex nextPixel;
        for (int i = 0; i < 8; i++)
        {
            int nX = curPixel[0] + neighbour[i][0];
            int nY = curPixel[1] + neighbour[i][1];
            if (!get(result, nX, nY, true)
                && borderMap.get(nX, nY) == 0)
            {
                isEdge = true;
                nextPixel = { nX, nY };
                break;
            }
        } // end of for i

        if (!isEdge)
            return true;

        if (distance == 0 || !fuse(nextPixel, distance - 1))
        {
            result.set(curPixel[0], curPixel[1], false);
            return false;
        }
            
        return true;
    }; // end of 'fuse' function

    for (int i = 0; i < finalBorderQueue.size(); i++)
    {
        // check if edge of fragment
        // (neighbour marked in borderMap but not result)
        bool isEdge = false;
        Vertex &vertex = finalBorderQueue[i];
        Vertex nextPixel;
        for (int j = 0; j < 8; j++)
        {
            int nX = vertex[0] + neighbour[j][0];
            int nY = vertex[1] + neighbour[j][1];
            if (!get(result, nX, nY, true)
                && borderMap.get(nX, nY) == 0)
            {
                isEdge = true;
                nextPixel = { nX, nY };
                break;
            }
        } // end of for j

        if (!isEdge)
            continue;

        int fuseDistance = 20;
        fuse(nextPixel, fuseDistance);
    } // end of for i
} // End of 'PenaltyBorder::find' function

Matrix2D<double> SignedDistanceField::compute(const Matrix2D<bool> &mask)
{
    Matrix2D<double> result(mask.sizeX, mask.sizeY);
    // result is passed as an input image because geodesic (image-dependent) impact is ignored:
    computeGeodesicDistance(result, mask, [](int x, int y, int nX, int nY) {
        return 1;
    }, result);
    return result;
}

Matrix2D<double> SignedDistanceField::compute(const Matrix2D<double> &image, const Matrix2D<bool> &mask)
{
    Matrix2D<double> result(image.sizeX, image.sizeY);
    computeGeodesicDistance(image, mask, [&image](int x, int y, int nX, int nY) {
        return 1 + std::abs(image.get(x, y) - image.get(nX, nY));
    }, result);
    return result;
}

struct PriorityVertex {
    int x, y;
    double value;
    PriorityVertex(int _x, int _y, double _value) : x(_x), y(_y), value(_value) {}
    PriorityVertex(std::array<int, 2> arr, double _value) : x(arr[0]), y(arr[1]), value(_value) {}
    operator std::array<int, 2>() const {
        return std::array<int, 2>{x, y};
    }
};

struct PriorityVertexCompare {
    bool operator()(const PriorityVertex& l, const PriorityVertex& r) {
        return l.value > r.value;
    }
};

void SignedDistanceField::computeGeodesicDistance(const Matrix2D<double> &intensities,
    const Matrix2D<bool> &prevMask,
    const std::function<double(int, int, int, int)> geodesicImpact,
    Matrix2D<double> &result)
{
    using Vertex = std::array<int, 2>;
    std::priority_queue<PriorityVertex, std::vector<PriorityVertex>, PriorityVertexCompare> active;
    std::priority_queue<PriorityVertex, std::vector<PriorityVertex>, PriorityVertexCompare> wait;
    const std::vector<Vertex> neighbors = { { 1, 0 },{ 0, 1 },{ -1, 0 },{ 0, -1 },
                                            { 1, 1 },{ -1, 1 },{ -1, -1 },{ 1, -1 } };

    result.each([](double &value) {
        value = std::numeric_limits<double>::max();
    });

    Matrix2D<bool> visited(result.sizeX, result.sizeY);
    // Add boundary points
    for (int y = 0; y < result.sizeY; y++) {
        for (int x = 0; x < result.sizeX; x++) {
            if (prevMask.get(x, y)) {
                const Vertex point = { x, y };
                for (int i = 0; i < 8; i++) {
                    if (!prevMask.get(x + neighbors[i][0], y + neighbors[i][1])) { // boundary point
                        result.set(x, y, 0);
                        visited.set(x, y, true);
                        active.push(PriorityVertex(point, 0));
                        break;
                    }
                }
            }
        }
    }

    int levels = 1;
    while (!active.empty() || !wait.empty()) {
        if (active.empty()) {
            active = wait;
            levels++;
            while (!wait.empty())
                wait.pop();
        }
        const PriorityVertex vertex = active.top();
        const Vertex point = vertex;
        active.pop();

        const int x = point[0], y = point[1];
        const double value = intensities.get(x, y);
        double updateValue = std::numeric_limits<double>::max();
        for (int i = 0; i < 8; i++) {
            const int nX = x + neighbors[i][0], nY = y + neighbors[i][1];
            const double neighbor = result.get(nX, nY);
            const double add = (i < 4) ? 1.0 : sqrt(2);
            const double geodesicDist = geodesicImpact(x, y, nX, nY);
            const double diff = add * geodesicDist;
            if (neighbor == std::numeric_limits<double>::max()) {
                // add unvisited point
                if (!visited.get(nX, nY)) {
                    if (0 <= nX && 0 <= nY && nX < result.sizeX && nY < result.sizeY) {
                        visited.set(nX, nY, true);
                        wait.push(PriorityVertex(nX, nY, vertex.value + diff));
                    }
                }
            }
            else
            {
                updateValue = std::min(updateValue, neighbor + diff);
            }
        }
        result.set(x, y, updateValue);
    }

}

Matrix2D<bool> LevelSetMethodBase::run(const Matrix2D<double> &image, const Matrix2D<bool> &initial)
{
    Matrix2D<bool> result(image.sizeX, image.sizeY);
    run(image, initial, result);
    return result;
}
    
void LevelSetMethodBase::makeInitialApproximation(const Matrix2D<bool> &initial, Matrix2D<double> &result)
{
    for (int y = 0; y < result.sizeY; y++) {
        for (int x = 0; x < result.sizeX; x++) {
            double value = -1.;
            if (initial.get(x, y)) {
                value = 1.;
            }
            result.set(x, y, value);
        }
    }
}

void LevelSetMethodBase::makeSmoothInitialApproximation(const Matrix2D<bool> &initial, Matrix2D<double> &result)
{
    const int halfWindowSize = smoothWindowSize_ / 2;
    const int pixelsInWindow = smoothWindowSize_ * smoothWindowSize_;

    for (int y = 0; y < result.sizeY; y++) {
        for (int x = 0; x < result.sizeX; x++) {
            double value = 0.0;
            for (int i = -halfWindowSize; i <= halfWindowSize; i++) {
                const int py = y + i;
                if (py < 0)
                    continue;
                if (py >= result.sizeY)
                    break;
                for (int j = -halfWindowSize; j <= halfWindowSize; j++) {
                    const int px = x + j;
                    if (px < 0)
                        continue;
                    if (px >= result.sizeX)
                        break;
                    value += initial.get(px, py) ? 1.0 : 0.0;
                }
            }
            result.set(x, y, 2.0 * (value / pixelsInWindow) - 1.0);
        }
    }
}
