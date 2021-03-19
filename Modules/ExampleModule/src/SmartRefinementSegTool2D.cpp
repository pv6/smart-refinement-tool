/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#include <SmartRefinementSegTool2D.h>

#include <usGetModuleContext.h>
#include <usModuleResource.h>
#include <queue>

#include <mitkImagePixelWriteAccessor.h>
#include <mitkImagePixelReadAccessor.h>
#include <mitkToolManager.h>
#include <vtkImageIterator.h>


MITK_TOOL_MACRO(MITKEXAMPLEMODULE_EXPORT, SmartRefinementSegTool2D, "Smart Refinement tool");


SmartRefinementSegTool2D::SmartRefinementSegTool2D()
  : SegTool2D("PressMoveReleaseWithCTRLInversionAllMouseMoves", us::GetModuleContext()->GetModule()), lsm_(1, 1),
  smartBrush_(new SmartBrushLevelSetImpl())
{
  smartBrush_->connect([this](VoxelPosition p) {
    MarkVoxel(p);
    smartBrushMarkedVoxels_.push_back(p);
  },
    [this](VoxelPosition p) {
    return GetVoxelIntensity(p);
  },
    [this]() { return GetSliceMinX(); },
    [this]() { return GetSliceMaxX(); },
    [this]() { return GetSliceMinY(); },
    [this]() { return GetSliceMaxY(); }
  );
  smartBrush_->changeRadius(smartBrush_->getRadius() / 2);
  smartBrush_->startSelection();

  result_ = std::make_unique<Matrix2D<bool>>(1, 1);
  mouseEvent_ = nullptr;
}

SmartRefinementSegTool2D::~SmartRefinementSegTool2D()
{
}

void SmartRefinementSegTool2D::OnMousePressed(mitk::StateMachineAction *action,
  mitk::InteractionEvent *interactionEvent)
{
  MITK_INFO << "mouse pressed";
  if (!isBorderMasking_)
    Paint(action, interactionEvent);
  else
  {
    DrawBorder(interactionEvent);
    isStartBorderMasking_ = false;
  }
}

void SmartRefinementSegTool2D::OnMouseMoved(mitk::StateMachineAction *, mitk::InteractionEvent *)
{
}

void SmartRefinementSegTool2D::OnPrimaryButtonPressedMoved(mitk::StateMachineAction *action,
  mitk::InteractionEvent *interactionEvent)
{
  if (isBorderMasking_)
    DrawBorder(interactionEvent);
}

void SmartRefinementSegTool2D::OnMouseReleased(mitk::StateMachineAction *, mitk::InteractionEvent *)
{
  MITK_INFO << "mouse released";
  if (isBorderMasking_)
  {
    WholeShabang();
    isStartBorderMasking_ = true;
  }
}

void SmartRefinementSegTool2D::ConnectActionsAndFunctions()
{
  CONNECT_FUNCTION("PrimaryButtonPressed", OnMousePressed);
  CONNECT_FUNCTION("Move", OnPrimaryButtonPressedMoved);
  CONNECT_FUNCTION("MouseMove", OnMouseMoved);
  CONNECT_FUNCTION("Release", OnMouseReleased);
  CONNECT_FUNCTION("InvertLogic", OnInvertLogic);
}

void SmartRefinementSegTool2D::OnInvertLogic(mitk::StateMachineAction *action,
  mitk::InteractionEvent *event)
{
  MITK_INFO << "invert logic";
  isBorderMasking_ = !isBorderMasking_;
} // end of 'SmartRefinementSegTool2D::OnInvertLogic' function

void SmartRefinementSegTool2D::UpdateClicksBBox(BBox3D &bbox, const int padding)
{
  int
    minX = GetSliceMaxX(),
    maxX = GetSliceMinX(),
    minY = GetSliceMaxY(),
    maxY = GetSliceMinY(),
    z = clicks_[0][2];

  for (auto &p : clicks_)
  {
    int x = p[0];
    int y = p[1];

    minX = std::min(minX, x);
    minY = std::min(minY, y);
    maxX = std::max(maxX, x);
    maxY = std::max(maxY, y);
  }

  if (minY > maxY)
  {
      minY = GetSliceMinY();
      maxY = GetSliceMaxY();
  }

  bbox =
  {
      VoxelPosition(
        {
          std::max(minX - padding, GetSliceMinX()),
          std::max(minY - padding, GetSliceMinY()),
          z
        }
      ),
      VoxelPosition(
        {
          std::min(maxX + padding, GetSliceMaxX()),
          std::min(maxY + padding, GetSliceMaxY()),
          z
        }
      )
  };
} // end of 'SmartRefinementSegTool2D::UpdateClicksBBox' function

void SmartRefinementSegTool2D::WholeShabang()
{
  if (mouseEvent_ == nullptr)
    return;

  DrawSmartBrush(clicks_.size() < 3);

  if (isSegment_ && clicks_.size() >= 3) {
    BBox3D bbox;
    //std::cout << "I am here now\n";

    const int padding = smartBrush_->getRadius(); // 3;

    // find bbox using clicks_
    UpdateClicksBBox(bbox, padding);
    bbox_ = bbox;

    seedPoints_ = clicks_;

    Refine(bbox);
  }
} // end of 'SmartRefinementSegTool2D::wholeShabang' function

void SmartRefinementSegTool2D::DrawBorder(mitk::InteractionEvent* interactionEvent)
{
  auto posEvent = dynamic_cast<mitk::InteractionPositionEvent *>(interactionEvent);

  if (posEvent == nullptr)
    return;

  VoxelPosition voxelPos = VoxelPosition();
  auto worldPos = posEvent->GetPositionInWorld();
  itk::Index<3> indx3d;
  GetAffectedReferenceSlice(posEvent)->GetGeometry()->WorldToIndex(worldPos, indx3d);
  for (int i = 0; i < 3; i++)
    voxelPos[i] = indx3d[i];

  if (!isStartBorderMasking_)
  {
    VoxelPosition cur = *(borderMask_.end() - 1);
    while (cur[0] != voxelPos[0] || cur[1] != voxelPos[1])
    {
      if (voxelPos[0] > cur[0])
        cur[0]++;
      else if (voxelPos[0] < cur[0])
        cur[0]--;

      if (voxelPos[1] > cur[1])
        cur[1]++;
      else if (voxelPos[1] < cur[1])
        cur[1]--;

      borderMask_.push_back(cur);
    }
  }

  borderMask_.push_back(voxelPos);
} // end of 'SmartRefinementSegTool2D::DrawBorder' function

void SmartRefinementSegTool2D::Paint(mitk::StateMachineAction*,
  mitk::InteractionEvent* interactionEvent)
{
  auto tmp = dynamic_cast<mitk::MousePressEvent *>(interactionEvent);

  if (tmp == nullptr)
    return;

  mouseEvent_ = tmp;

  referenceSlice_ = GetAffectedReferenceSlice(mouseEvent_);
  oldWorkingSlice_ = GetAffectedWorkingSlice(mouseEvent_);

  newWorkingSlice_ = oldWorkingSlice_->Clone();

  // save clicked voxel index
  VoxelPosition voxelPos = VoxelPosition();
  auto worldPos = mouseEvent_->GetPositionInWorld();

  MITK_INFO << "world pos: " << worldPos[0] << ", " << worldPos[1] << ", " << worldPos[2];

  itk::Index<3> indx3d;
  //referenceSlice_->GetGeometry()->WorldToIndex(worldPos, indx3d);
  mouseEvent_->GetSender()->GetCurrentWorldGeometry()->WorldToIndex(worldPos, indx3d);
  for (int i = 0; i < 3; i++)
    voxelPos[i] = indx3d[i];
  MITK_INFO << "index: " << voxelPos[0] << ", " << voxelPos[1] << ", " << voxelPos[2];

  clicks_.push_back(voxelPos);

  DrawSmartBrush(true);
  WholeShabang();
}

void SmartRefinementSegTool2D::FlushVoxels()
{
  newWorkingSlice_->Modified();
  WriteBackSegmentationResult(mouseEvent_, newWorkingSlice_);
}

void SmartRefinementSegTool2D::DrawSmartBrush(bool flush)
{
  StartNewLabeling();
  for (auto &p : clicks_)
    smartBrush_->draw(p, smartBrush_->getRadius());
  if (flush)
    FlushVoxels();
}

void SmartRefinementSegTool2D::SaveMask(const std::string &fileName)
{
  //isCutStuff = !isCutStuff;
  WholeShabang();
  return;

  SaveImage(fileName, *result_);
  ClearState();
} // End of 'SmartRefinement::saveMask' function

us::ModuleResource SmartRefinementSegTool2D::GetIconResource() const
{
  auto moduleContext = us::GetModuleContext();
  auto module = moduleContext->GetModule();
  auto resource = module->GetResource("ExampleIcon.svg");
  return resource;
}

void SmartRefinementSegTool2D::StartNewLabeling()
{
  auto iterator = vtkImageIterator<char>(newWorkingSlice_->GetVtkImageData(),
    newWorkingSlice_->GetVtkImageData()->GetExtent());
  while (!iterator.IsAtEnd())
  {
    auto begin = iterator.BeginSpan();
    auto end = iterator.EndSpan();
    while (begin != end)
      *begin++ = 0;
    iterator.NextSpan();
  }
  newWorkingSlice_->GetVtkImageData()->Modified();
  newWorkingSlice_->Modified();
}

void SmartRefinementSegTool2D::Refine(BBox3D &bbox)
{
  StartNewLabeling();

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

  Matrix2D<bool> trustedMask(width, height);

  // load mask
  for (int y = 0; y <= height; y++)
    for (int x = 0; x <= width; x++)
    {
      trustedMask.set(
        x,
        y,
        IsLabeled(VoxelPosition({ x + bbox[0][0], y + bbox[0][1], z }))
      );
    }

  for (const auto& p : smartBrushMarkedVoxels_)
  {
    const int x = p[0] - bbox[0][0];
    const int y = p[1] - bbox[0][1];
    if (x < 0 || y < 0 || x > width || y > height)
      continue;
    trustedMask.set(x, y, true);
  }

  Matrix2D<bool> result(width, height);

  Matrix2D<double> sliceImage(width, height);
  NormalizationParameters initialSliceNorm;
  NormalizeImage(bbox, sliceImage, initialSliceNorm);

  initLSM(sliceImage, trustedMask);

  RefineSlice(sliceImage, trustedMask, result);

  // here be border mask
  ApplyBorderMask(bbox, result);

  // here be remove leaks
  if (isCutStuff)
  {
    trustedMask.fill(false);
    RemoveLeaks(trustedMask, result);
  }
  else
  {
    trustedMask = std::move(result);
  }
  // ^^^ results now stored in mask!!

  result_ = std::make_unique<Matrix2D<bool>>(trustedMask);
  MarkMask(bbox[0], trustedMask);

  FlushVoxels();
} // end of 'SmartRefinementSegTool2D::Refine' function

void SmartRefinementSegTool2D::ApplyBorderMask(const BBox3D &bbox,
  Matrix2D<bool> &selection)
{
  const auto &corner = bbox[0];

  // remove duplicat points
  sort(borderMask_.begin(), borderMask_.end());
  borderMask_.erase(unique(borderMask_.begin(),
    borderMask_.end()), borderMask_.end());

  VoxelPosition bb_border;
  // iterate border mask vector
  for (auto i = borderMask_.begin(); i != borderMask_.end(); i++)
  {
    // subtract bbox corner and set selection to false
    bb_border[0] = (*i)[0] - corner[0];
    bb_border[1] = (*i)[1] - corner[1];

    if (bb_border[0] >= 0 && bb_border[0] < selection.sizeX &&
        bb_border[1] >= 0 && bb_border[1] < selection.sizeY)
      selection.set((*i)[0] - corner[0], (*i)[1] - corner[1], false);
  }
} // end of 'SmartRefinementSegTool2D::ApplyBorderMask' function

void SmartRefinementSegTool2D::MarkMask(const VoxelPosition &leftCorner, const Matrix2D<bool> &mask) {
  for (int y = 0; y < mask.sizeY; y++) {
    for (int x = 0; x < mask.sizeX; x++) {
      const int i = leftCorner[0] + x;
      const int j = leftCorner[1] + y;
      if (mask.get(x, y))
        MarkVoxel(VoxelPosition({ i, j, leftCorner[2] }));
    }
  }
}

void SmartRefinementSegTool2D::RemoveLeaks(Matrix2D<bool> &result,
                                           Matrix2D<bool> &mask)
{
  using Vertex = std::array<int, 2>;
  std::queue<Vertex> queue;
  const std::vector<Vertex> neighbors = { { 1, 0 },{ 0, 1 },{ -1, 0 },{ 0, -1 } };
  // Vertex center = { mask.sizeX / 2, mask.sizeY / 2 };
  for (VoxelPosition &seedPoint : seedPoints_) {
    Vertex center = { seedPoint[0], seedPoint[1] };

    if (!mask.get(center[0], center[1])) // if seed point is incorrect
      continue; // skip this point

    queue.push(center);

    while (!queue.empty()) {
      Vertex node = queue.front();
      queue.pop();
      if (result.get(node[0], node[1]))
        continue;
      result.set(node[0], node[1], true);
      for (auto n : neighbors) {
        Vertex coords = { node[0] + n[0], node[1] + n[1] };
        if (mask.get(coords[0], coords[1]) && !result.get(coords[0], coords[1]))
          queue.push(coords);
      }
    }
  }
} // End of 'SmartRefinementSegTool2D::RemoveLeaks' function

void SmartRefinementSegTool2D::NormalizeImage(const BBox3D &bbox,
                                              Matrix2D<double> &res,
                                              NormalizationParameters &params)
{
  if (params.min == std::numeric_limits<double>::max()) {
    for (int y = 0; y < res.sizeY; y++) {
      for (int x = 0; x < res.sizeX; x++) {
        double value = GetVoxelIntensity(VoxelPosition({
            bbox[0][0] + x,
            bbox[0][1] + y,
            bbox[0][2]
          }));
        if (value > params.max)
          params.max = value;
        if (value < params.min)
          params.min = value;
      }
    }
  }

  for (int y = 0; y < res.sizeY; y++) {
    for (int x = 0; x < res.sizeX; x++) {
      double value = GetVoxelIntensity(VoxelPosition({
          bbox[0][0] + x,
          bbox[0][1] + y,
          bbox[0][2]
        }));

      value = (value - params.min) / (params.max - params.min);
      if (value > 1.0)
        value = 1.0;
      if (value < 0)
        value = 0.0;
      res.set(x, y, value);
    }
  }
}

void SmartRefinementSegTool2D::initLSM(const Matrix2D<double> &sliceImage, const Matrix2D<bool> &mask)
{
} // End of 'SmartRefinementSegTool2D::initLSM' function

void SmartRefinementSegTool2D::ClearState()
{
  clicks_.clear();
  smartBrushMarkedVoxels_.clear();
  borderMask_.clear();
  //isSegment_ = false;
} // End of 'SmartRefinementSegTool2D::ClearState' function

void SmartRefinementSegTool2D::RefineSlice(const Matrix2D<double> &sliceImage,
  const Matrix2D<bool> &trustedMask,
  Matrix2D<bool> &result)
{
  lsm_.resize(sliceImage.sizeX, sliceImage.sizeY);
  //std::cout << "I am being refined over here" << std::endl;

  lsm_.setInputSlice(sliceImage);
  lsm_.setMaxIterations(100);
  lsm_.run(sliceImage, trustedMask, result, [&trustedMask, this]() -> double {
    return lsm_.calculateC(trustedMask);
  });
}

double SmartRefinementSegTool2D::CalcEps(const Matrix2D<double> &sliceImage, const double threshold, const Matrix2D<bool> &mask)
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

double SmartRefinementSegTool2D::CalcThreshold(const Matrix2D<double> &sliceImage, const Matrix2D<bool> &mask)
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
} // End of 'SmartRefinementSegTool2D::calcThreshold' function

void SmartRefinementSegTool2D::InitDensityField(const Matrix2D<double> &sliceImage,
  const double eps, const double thresh, Matrix2D<double> &dField)
{
  for (int y = 0; y < dField.sizeY; y++) {
    for (int x = 0; x < dField.sizeX; x++) {
      double value = sliceImage.get(x, y);
      const double density = eps - fabs(value - thresh);
      dField.set(x, y, density);
    }
  }
} // End of 'SmartRefinementSegTool2D::initDensityField' function

const char *SmartRefinementSegTool2D::GetName() const
{
  return "Smart Refinement";
}

const char **SmartRefinementSegTool2D::GetXPM() const
{
  return nullptr;
}

double SmartRefinementSegTool2D::GetVoxelIntensity(const VoxelPosition &voxelPos)
{
  itk::Index<2> indx;
  for (int i = 0; i < 2; i++)
    indx[i] = voxelPos[i];
  //indx[2] = 0;

  auto pixelType = referenceSlice_->GetPixelType();
  std::string typeName = pixelType.GetComponentTypeAsString();

  if (typeName == "int")
  {
      mitk::ImagePixelReadAccessor<int, 2> accessor(referenceSlice_);
      return (double)accessor.GetPixelByIndexSafe(indx);
  }
  if (typeName == "float")
  {
      mitk::ImagePixelReadAccessor<float, 2> accessor(referenceSlice_);
      return (double)accessor.GetPixelByIndexSafe(indx);
  }

  return 0;
} // end of 'SmartRefinementSegTool2D::GetVoxelIntensity' function

bool SmartRefinementSegTool2D::IsLabeled(const VoxelPosition &voxelPos)
{
  itk::Index<2> indx;
  for (int i = 0; i < 2; i++)
    indx[i] = voxelPos[i];
  //indx[2] = 0;

  if (newWorkingSlice_->GetPixelType().GetBitsPerComponent() == 16)
  {
    mitk::ImagePixelReadAccessor<unsigned short, 2> accessor(oldWorkingSlice_);
    return (bool)accessor.GetPixelByIndexSafe(indx);
  }
  else if (newWorkingSlice_->GetPixelType().GetBitsPerComponent() == 8)
  {
    mitk::ImagePixelReadAccessor<unsigned char, 2> accessor(oldWorkingSlice_);
    return (bool)accessor.GetPixelByIndexSafe(indx);
  }
  return false;
} // end of 'SmartRefinementSegTool2D::IsLabeled' function

void SetPixel(mitk::Image::Pointer &image, const VoxelPosition &voxelPos, int value)
{
  itk::Index<2> indx;
  for (int i = 0; i < 2; i++)
    indx[i] = voxelPos[i];

  if (image->GetPixelType().GetBitsPerComponent() == 16)
  {
    mitk::ImagePixelWriteAccessor<unsigned short, 2> accessor(image);
    accessor.SetPixelByIndexSafe(indx, value);
  }
  else if (image->GetPixelType().GetBitsPerComponent() == 8)
  {
    mitk::ImagePixelWriteAccessor<unsigned char, 2> accessor(image);
    accessor.SetPixelByIndexSafe(indx, value);
  }
} // end of 'SetPixel' function

void SmartRefinementSegTool2D::MarkVoxel(const VoxelPosition &voxelPos)
{
  SetPixel(newWorkingSlice_, voxelPos, 1);
} // end of 'SmartRefinementSegTool2D::MarkVoxels' function

int SmartRefinementSegTool2D::GetSliceMinX()
{
  return 0;
} // end of 'SmartRefinementSegTool2D::GetSliceMinX' function

int SmartRefinementSegTool2D::GetSliceMaxX()
{
  return oldWorkingSlice_->GetDimensions()[0];
} // end of 'SmartRefinementSegTool2D::GetSliceMaxX' function

int SmartRefinementSegTool2D::GetSliceMinY()
{
  return 0;
} // end of 'SmartRefinementSegTool2D::GetSliceMinY' function

int SmartRefinementSegTool2D::GetSliceMaxY()
{
  return oldWorkingSlice_->GetDimensions()[1];
} // end of 'SmartRefinementSegTool2D::GetSliceMaxY' function

void SmartRefinementSegTool2D::ChangeSensitivity(double sens) {
  lsm_.setSensitivity(sens);
  WholeShabang();
}

void SmartRefinementSegTool2D::ChangeAreaPenalty(double areaPenalty) {
  lsm_.setNu(areaPenalty);
  WholeShabang();
}

void SmartRefinementSegTool2D::ChangeSmoothness(double sens) {
  lsm_.setR2Mu(1 - sens);
  WholeShabang();
}

void SmartRefinementSegTool2D::ChangeBrushRadius(int radius) {
  smartBrush_->changeRadius(radius);
  WholeShabang();
}

void SmartRefinementSegTool2D::ChangeBorderPenalty(double sens)
{
  lsm_.setDF2SS(1 - sens);
  WholeShabang();
} // End of 'SmartRefinementSegTool2D::changeSSCoef' function

void SmartRefinementSegTool2D::NewAnnotation()
{
  if (newWorkingSlice_.IsNotNull())
  {
    StartNewLabeling();
    FlushVoxels();
  }

  ClearState();
} // end of 'SmartRefinementSegTool2D::NewAnnotation' function

int SmartRefinementSegTool2D::CalculateBorderLength()
{
  int sum = 0;

  result_->fillPadding(false);

  int delta_x[4] = { -1, 1, 0, 0 };
  int delta_y[4] = { 0, 0, -1, 1 };

  for (int y = 0; y < result_->sizeY; y++)
    for (int x = 0; x < result_->sizeX; x++)
    {
      // check if border pixel, e.g. is next to not mask, is mask.
      // check self first
      if (!result_->get(x, y))
        continue;

      // check neighbours
      for (int i = 0; i < 4; i++)
        if (!result_->get(x + delta_x[i], y + delta_y[i]))
        {
          sum++;
          break;
        }
    }

  return sum;
} // end of 'SmartRefinementSegTool2D::CalculateBorderLength' function
