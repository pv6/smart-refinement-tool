/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef SmartRefinementSegTool2D_h
#define SmartRefinementSegTool2D_h

#include <mitkSegTool2D.h>
#include <mitkVtkImageOverwrite.h>
#include <mitkOverwriteSliceImageFilter.h>
#include <MitkExampleModuleExports.h>
#include <ChanVeseLSM.hpp>
#include <mitkMousePressEvent.h>
#include <SmartBrushLevelSetImpl.hpp>


namespace us
{
  class ModuleResource;
}

// This is an example on how to add a segmentation tool to the standard
// MITK Segmentation Views. Here's the crucial points to make it work:
//
//   * The name of the class MUST end in either SegTool2D or SegTool3D
//
//   * Derive directly or indirectly from mitk::Tool, for example
//     mitk::SegTool2D is a typical choice for 2d tools and
//     mitk::AutoSegmentationTool is a typical choice for 3d tools.
//
//   * Don't forget the MITK_TOOL_MACRO in the implementation file
//
//   * You don't have to provide your own state machine if the state
//     machines of the MitkSegmentation module are sufficient, but if
//     you do, you MUST provide both, the state machine and an event
//     config file, for example Paint.xml and PaintConfig.xml. The
//     file name of the event config file MUST be "equal" to the file
//     name of the state machine, suffixed by Config.xml instead of
//     .xml. The file name is passed to the base class constructor
//     without any extension, for example simply "Paint".
//
//   * Look into SmartRefinementSegTool2DGUI.h for an example of how to provide
//     a custom tool GUI. The naming is important and basically "equals"
//     the class name of the tool class, suffixed by GUI. Don't forget
//     the MITK_TOOL_GUI_MACRO in the implementation file of the tool GUI.
//
//   * Ensure that the module containing your tool is loaded at runtime
//     before the MITK Segmentation Views are opened. Otherwise it cannot
//     be found. One way is to call anything from that module in a plugin
//     with eager activation policy. Such plugins are loaded very early
//     in MITK. For an example of how this works, look into ExampleModule.h
//     and the org_mitk_exampleplugin_eageractivation plugin.

class MITKEXAMPLEMODULE_EXPORT SmartRefinementSegTool2D : public mitk::SegTool2D
{
public:
  mitkClassMacro(SmartRefinementSegTool2D, SegTool2D)
  itkFactorylessNewMacro(Self)

  us::ModuleResource GetIconResource() const override;
  const char *GetName() const override;
  const char **GetXPM() const override;

  virtual void ChangeSensitivity(double sens);
  virtual void ChangeSmoothness(double sens);
  virtual void ChangeBrushRadius(int radius);
  virtual void ChangeBorderPenalty(double sens);
  virtual void ChangeAreaPenalty(double areaPenalty);
  virtual void NewAnnotation();

  bool isSegment_ = true;
  virtual void Segment()
  {
    isSegment_ = true;
    WholeShabang();
  }
  virtual void WholeShabang();
  int CalculateBorderLength();
  void SaveMask(const std::string &fileName);

protected:
  ChanVeseLevelSet lsm_;
  std::unique_ptr<SmartBrushLevelSetImpl> smartBrush_;
  std::vector<VoxelPosition> clicks_;
  std::vector<VoxelPosition> smartBrushMarkedVoxels_;
  std::unique_ptr<Matrix2D<bool>> result_;
  BBox3D bbox_;
  std::vector<VoxelPosition> seedPoints_;
  mitk::Image::Pointer referenceSlice_;
  mitk::Image::Pointer oldWorkingSlice_;
  mitk::Image::Pointer newWorkingSlice_;
  std::vector<VoxelPosition> borderMask_;
  bool isBorderMasking_ = false;
  bool isStartBorderMasking_ = true;
  bool isCutStuff = true;

  SmartRefinementSegTool2D();
  ~SmartRefinementSegTool2D() override;

  double GetVoxelIntensity(const VoxelPosition &voxelPos);
  bool IsLabeled(const VoxelPosition &voxelPos);
  void MarkVoxel(const VoxelPosition &voxelPos);
  int GetSliceMinX();
  int GetSliceMaxX();
  int GetSliceMinY();
  int GetSliceMaxY();
  mitk::MousePressEvent::Pointer mouseEvent_;

  virtual void initLSM(const Matrix2D<double> &sliceImage, const Matrix2D<bool> &mask);

  /*!
  * \brief Refine annotation
  * \param sliceImage New image
  * \param trustedMask Mask of pixels definitely belonging to the ROI
  * \param result Refinement result - annotation mask
  */
  virtual void RefineSlice(const Matrix2D<double> &sliceImage,
                           const Matrix2D<bool> &trustedMask,
                           Matrix2D<bool> &result);

  /*!
   * \brief Remove contour leaks
   * \param result Result mask
   * \param mask Mask to remove leaks on
   */
  void RemoveLeaks(Matrix2D<bool> &result, Matrix2D<bool> &mask);
  void UpdateClicksBBox(BBox3D &bbox, const int padding);

  /*!
   * \brief Mark voxels in image where mask has 'true'(bool) value
   * \param leftCorner Upper left corner position on the image - where to start marking voxels
   * \param mask Mask
   */
  void MarkMask(const VoxelPosition &leftCorner, const Matrix2D<bool> &mask);

  void FlushVoxels();

  void DrawBorder(mitk::InteractionEvent* interactionEvent);
  void ApplyBorderMask(const BBox3D &bbox, Matrix2D<bool> &selection);

  //! Refine selection
  virtual void Refine(BBox3D &bbox);
  virtual void ClearState();
  virtual double CalcEps(const Matrix2D<double> &sliceImage, const double threshold, const Matrix2D<bool> &mask);
  virtual double CalcThreshold(const Matrix2D<double> &sliceImage, const Matrix2D<bool> &mask);
  virtual void InitDensityField(const Matrix2D<double> &sliceImage, const double eps, const double thresh, Matrix2D<double> &dField);
  virtual void Paint(mitk::StateMachineAction* action, mitk::InteractionEvent* event);
  virtual void OnInvertLogic(mitk::StateMachineAction *action, mitk::InteractionEvent *event);
  virtual void OnMousePressed(mitk::StateMachineAction *, mitk::InteractionEvent *);
  virtual void OnMouseMoved(mitk::StateMachineAction *, mitk::InteractionEvent *);
  virtual void OnPrimaryButtonPressedMoved(mitk::StateMachineAction *, mitk::InteractionEvent *);
  virtual void OnMouseReleased(mitk::StateMachineAction *, mitk::InteractionEvent *);


  /*!
   * \brief Find min and max intensities and normalize input image (inside bbox) - rescale to [0, 1]
   * \param bbox Bounding box
   * \param res Output matrix
   * \param normalizationParameters - If default values are used, parameters will be updated.
   *                                  Otherwise normalization is done with respect to given min and max values
   */
  virtual void NormalizeImage(const BBox3D &bbox, Matrix2D<double> &res,
                              NormalizationParameters &normalizationParameters);

  void DrawSmartBrush(bool flush);
  void StartNewLabeling();

private:
  template<typename T>
  void SaveImage(const std::string &fileName, const Matrix2D<T> &image)
  {
    auto file = std::fstream(fileName, std::ios::out | std::ios::binary);

    // write number of clicks
    int seedSize = seedPoints_.size();
    file.write((char *)&seedSize, sizeof(int));

    // write image size
    int size[] = { image.sizeX + 2, image.sizeY + 2 };
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
    double percentage = (double)borderMask_.size() / CalculateBorderLength();
    file.write((char *)&percentage, sizeof(double));

    MITK_INFO << "Percentage: " << percentage;
    MITK_INFO << borderMask_.size() << " / " << CalculateBorderLength();

    file.close();
  } // End of 'saveImage' function

  void ConnectActionsAndFunctions() override;
};

#endif
