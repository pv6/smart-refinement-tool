/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef ExampleImageFilter_h
#define ExampleImageFilter_h

#include <mitkImageToImageFilter.h>

// The following header file is generated by CMake and thus it's located in
// the build directory. It provides an export macro for classes and functions
// that you want to be part of the public interface of your module.
#include <MitkExampleModuleExports.h>

// While you are free to derive directly from ITK filter base classes,
// MITK filter base classes provide typed accessor methods for the inputs
// and outputs, which will save you and your clients lots of manual casting.
class MITKEXAMPLEMODULE_EXPORT ExampleImageFilter final : public mitk::ImageToImageFilter
{
public:
  // All classes that derive from an ITK-based MITK class need at least the
  // following two macros. Make sure you don't declare the constructor public
  // to force clients of your class to follow the ITK convention for
  // instantiating classes via the static New() method.
  mitkClassMacro(ExampleImageFilter, mitk::ImageToImageFilter)
  itkFactorylessNewMacro(Self)

  itkSetMacro(Offset, int)
  itkGetMacro(Offset, int)

private:
  ExampleImageFilter();
  ~ExampleImageFilter();

  void GenerateData() override;

  int m_Offset;
};

#endif