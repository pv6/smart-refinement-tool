#ifndef SMARTREFINEMENT_H
#define SMARTREFINEMENT_H

#include <vvt/algorithms/imagelabelingaccessory2d.hpp>
#include <vvt/algorithms/commontypes.hpp>
#include <vvt/accessory/propagationaccessory.hpp>
#include <vvt/algorithms/lsmcommon.hpp>
#include <vvt/algorithms/smartbrushlevelsetimpl.hpp>
#include <vvt/algorithms/chanveselsm.hpp>
#include <vector>
#include <windows.h>
#include <iostream>
#include <fstream>

namespace vvt {

    namespace accessory {

        //! Smart Refinement tool
        class SmartRefinement : public PropagationAccessory
        {
        public:
            //! C-tor must be with this signature
            SmartRefinement(viewer::base *viewer);

            //! Must return unique name of the use case. Reason: vvt commands must have unique names. It's a feature. =)
            virtual std::string getUseCaseName();

            /*!
            * \brief Expose use case-specific commands: start selection
            * \param engine Viewer's engine to link the interface with
            */
            virtual void exposeUseCaseCommands(vvt::engine *engine);

            /*!
            * \brief Voxel labeling event handler. Each use case should have it implemented
            * \param voxelPos Voxel position in the image space
            */
            virtual void OnNewLabel2D(std::array<int, 3> &voxelPos);

            //! Start selection
            virtual void startSelection() {};

            /*!
            * \brief Mouse move event handler. Each use case should have it implemented
            * \param voxelPos Voxel position in the image space
            */
            virtual void OnMouseMove2D(std::array<int, 3> &voxelPos) {}

            /*!
            * \brief Defines how a tool should react on a window/level change.
            * \param windowLevel New window and level that have already been SET to the image
            */
            virtual void onWindowLevelChanged(const std::array<double, 2>& windowLevel) {}

            virtual void onClearMask();

        protected:
            /*!
            * \brief Initialize method on annotated image
            * \param sliceImage Annotated image
            * \param mask Image annotation
            */
            virtual void initLSM(const Matrix2D<double> &sliceImage, const Matrix2D<bool> &mask);

            /*!
            * \brief Set the new sensitivity value
            * \param sens Sensitivity
            */
            void changeSensitivity(double sens);

            template<typename T>
            void saveImage(const std::string &fileName, const Matrix2D<T> &image)
            {
                static char view_num = 0;
                view_num %= 3;
                view_num++;
                std::string newFileName = fileName;
                newFileName.append(1, '_');
                newFileName.append(1, '0' + view_num);

                auto file = std::fstream(newFileName, std::ios::out | std::ios::binary);

                // write number of clicks
                int seedSize = seedPoints_.size();
                file.write((char *)&seedSize, sizeof(int));

                // write image size
                int size[] = {image.sizeX + 2, image.sizeY + 2};
                std::cout << size[0] << ", " << size[1] << std::endl;
                file.write((char *)size, sizeof(size));

                // write bbox info
                file.write((char *)bbox_[0].data(), sizeof(int) * 3);
                file.write((char *)bbox_[1].data(), sizeof(int) * 3);
                std::cout << bbox_[0][0] << ", " << bbox_[0][1] << std::endl;
                std::cout << bbox_[1][0] << ", " << bbox_[1][1] << std::endl;

                // write image
                image.toFile(file);                     

                // write manual border percentage
                double time_sec = (double)timer_ / CLOCKS_PER_SEC;
                file.write((char *)&time_sec, sizeof(double));

                file.close();
            } // End of 'saveImage' function

            void saveDist(const std::string &fileName);

            void saveMask(const std::string &fileName);

            void savePhi(const std::string &fileName);

            /*!
            * \brief Set the new sensitivity value
            * \param sens Sensitivity
            */
            void changeSmoothness(double sens);

            /*!
            * \brief Set the new sensitivity value
            * \param sens Sensitivity
            */
            void changeBrushRadius(int radius);

            /*!
            * \brief Set the new sensitivity value
            * \param sens Sensitivity
            */
            void changeBorderPenalty(double sens);

            /*!
            * \brief Fill region inside control points
            * \param mask Input & output mask
            */
            void complementMask(Matrix2D<bool> &mask);

            /*!
            * \brief Refine annotation
            * \param sliceImage New image
            * \param mask Initial approximation
            * \param trustedMask Mask of pixels definitely belonging to the ROI
            * \param result Refinement result - annotation mask
            */
            virtual void refineSlice(const Matrix2D<double> &sliceImage,
                                     const Matrix2D<bool> &mask,
                                     const Matrix2D<bool> &trustedMask,
                                     Matrix2D<bool> &result);

            void drawSmartBrush(std::array<int, 3> &voxelPos);

            void startNewAnnotation();

            void updateClicksBBox(BBox3D &bbox, const int padding);

            void propagateSlice(const Matrix2D<double> &, const Matrix2D<bool> &, Matrix2D<bool> &) {}

        private:
            //! Refine selection
            void refine(BBox3D &bbox);
            void wholeShabang();

            void clearState();

            double sensitivity_ = 0.1;
            time_t timer_;
            ChanVeseLevelSet lsm_;
            std::unique_ptr<SmartBrushLevelSetImpl> smartBrush_;
            std::vector<VoxelPosition> clicks_;
            std::vector<VoxelPosition> smartBrushMarkedVoxels_;
            std::unique_ptr<Matrix2D<bool>> result_;
            BBox3D bbox_;

            double calcEps(const Matrix2D<double> &sliceImage, const double threshold, const Matrix2D<bool> &mask);
            double calcThreshold(const Matrix2D<double> &sliceImage, const Matrix2D<bool> &mask);
            void initDensityField(const Matrix2D<double> &sliceImage, const double eps, const double thresh, Matrix2D<double> &dField);
        };

    } //namespace accessory
} //namespace vvt

#endif // SMARTREFINEMENT_H
