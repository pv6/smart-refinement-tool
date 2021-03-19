#ifndef ANNOTATIONCOMMONTYPES_H
#define ANNOTATIONCOMMONTYPES_H

#include <array>
#include <algorithm>
#include <utility>
#include <memory>
#include <unordered_map>
#include <functional>
#include <vector>
#include <fstream>

static std::vector<std::string> UseCaseNames =
{
"ForegroundBackgroundClean",
"SmartBrushLevelSet",
"SmartBrushFastMarching",
"SmartBrushFastMarching3D",
"IntelligentScissors",
"MagicWand",
"Escultor",
"LSMPropagation",
"ChanVesePropagation",
"SmartRefinement"
};

enum AUXVOLUMERENDERING
{
GRADIENT,
RESAMPLED
};

enum Orientation
{
ZX,
ZY,
XY
};

enum TOOLS
{
FBCLEAN,
SMARTBRUSH_LS,
SMARTBRUSH_FM,
SMARTBRUSH_FM3D,
INTELLIGENT_SCISSORS,
MAGICWAND,
ESCULTOR,
PROPAGATION2D,
CHANVESE_PROPAGATION2D,
SMARTREFINEMENT,
TOOLS_COUNT
};

//! Types of filters
enum INTENSITY_FILTERTYPE
{
    NO_FILTER = 0,
    FILTER_GAUSS, 
    FILTER_MEDIAN, 
    FILTER_ANISO,

    FILTER_COUNT,
    FILTER_GRADIENT
};
//! Mesh modification modes
enum MESH_MODIF_MODE
{
    MESH_MODIF_MODE_EXPANSION = 0,
    MESH_MODIF_MODE_SHRINKING,

    MESH_MODIF_MODE_COUNT
};

//! Position of a Voxel
typedef std::array<int, 3> VoxelPosition;
//! Bounding box in 3D - two corner VoxelPosition points
typedef std::array<VoxelPosition, 2> BBox3D;
//! Position of a Pixel in the slice
typedef std::array<int, 2> PixelPosition;
//! Bounding box in 2D - two corner PixelPosition points
typedef std::array<PixelPosition, 2> BBox2D;

//! 2D-Matrix with padding
template<typename T>
struct Matrix2D {
    //! X-dimension
    int sizeX;
    //! Y-dimension
    int sizeY;
    //! Memory keeper
    T * data;
    //! Pointer to array that represents matrix
    T * arr;

    //! Zero-constructor
    Matrix2D() {
        sizeX = sizeY = 0;
        data = arr = nullptr;
    }
            
    //! Move constructor
    Matrix2D(Matrix2D<T> &&other) {
        sizeX = other.sizeX;
        sizeY = other.sizeY;
        data = other.data;
        arr = other.arr;
        other.arr = nullptr;
        other.data = nullptr;
    }

    //! Copy constructor - make a deep copy
    Matrix2D(const Matrix2D<T> &other) {
        sizeX = other.sizeX;
        sizeY = other.sizeY;
        const size_t allocSize = (sizeX + 2) * (sizeY + 2);
        data = new T[allocSize]{ 0 };
        if (!data) {
            data = nullptr;
            return;
        }
        //std::memcpy(data, other.data, allocSize * sizeof(T));
        std::copy(other.data, other.data + allocSize, data);
        arr = data + sizeX + 1;
    }

    /*!
    * \brief Constructor with memory allocation
    * \param x width of area
    * \param y height of area
    */
    Matrix2D(const int x, const int y) : sizeX(x), sizeY(y) {
        data = new T[(x + 2) * (y + 2)]{ 0 };
        std::fill(&data[0], &data[(x + 2) * (y + 2) - 1], static_cast<T>(0));
        arr = data + x + 1;
    }

    //! Move assignment
    void operator = (Matrix2D<T>&& other)
    {
        sizeX = other.sizeX;
        sizeY = other.sizeY;
        data = other.data;
        arr = other.arr;
        other.arr = nullptr;
        other.data = nullptr;
    }

    /*!
    * \brief Fill matrix elements with certain value
    * \param value Value to fill with
    */
    void fill(T value)
    {
        std::fill(&data[0], &data[(sizeX + 2) * (sizeY + 2) - 1], value);
    }

    /*!
    * \brief Fill matrix padding with certain value
    * \param Value to fill with
    */
    void fillPadding(T paddingValue)
    {
        // Fill top padding
        std::fill(data, data + sizeX + 1, paddingValue);

        // Fill side padding elements row by row
        int curIndex = sizeX + 2;
        for (int i = 0; i < sizeY; i++)
        {
            // Fill left-most element
            data[curIndex] = paddingValue;
            curIndex += sizeX + 1;
            // Fill right-most element
            data[curIndex] = paddingValue;
            curIndex += 1;
        }

        // Fill bottom padding
        std::fill(data + curIndex, data + curIndex + sizeX + 1, paddingValue);
    }

    /*!
    * \brief Replace current cell with a new value
    * \param x x-index
    * \param y y-index
    * \param value value to be set into cell
    */
    void set(const int x, const int y, const T &value) {
        arr[getIndex(x, y)] = value;
    }

    /*!
    * \brief Get value of a cell
    * \param x x-index
    * \param y y-index
    */
    T get(const int x, const int y) const {
        return arr[getIndex(x, y)];
    }

    /*!
    * \brief Iterate through every cell
    * \param func Functor
    */
    void each(std::function<void(T&)> func) {
        const int size = (sizeX + 2) * (sizeY + 2);
        for (int i = 0; i < size; i++) {
            func(data[i]);
        }
    }

    /*!
    * \brief Iterate through every cell
    * \param func Functor
    */
    void each(std::function<void(const T&)> func) const {
        const int size = (sizeX + 2) * (sizeY + 2);
        for (int i = 0; i < size; i++) {
            func(data[i]);
        }
    }

    /*
    * \brief Export matrix to file by fstream
    * \param file Output fstream
    */
    void toFile(std::fstream &file) const {
        file.write((char*)data, sizeof(T) * (sizeX + 2) * (sizeY + 2));
    }
            
    /*
    * \brief Export matrix to file by name
    * \param file Output file name
    */
    void toFile(const std::string &name) const {
        auto file = std::fstream(name, std::ios::out | std::ios::binary);
        toFile(file);
        file.close();
    }

    //! Destructor
    ~Matrix2D() {
        if (data != nullptr)
            delete[] data;
        data = nullptr;
    }

private:
    /*!
    * \brief Get index in data array according to padding
    * \param x x-index
    * \param y y-index
    */
    inline int getIndex(const int x, const int y) const {
        // return y * sizeX + x;
        const int pad = 2 * (y + 1);
        return y * sizeX + x + pad;
    }
};

#endif // SMARTBRUSHLEVELSET_H
