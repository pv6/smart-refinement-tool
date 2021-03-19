/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef SmartRefinementSegTool2DGUI_h
#define SmartRefinementSegTool2DGUI_h

#include <QmitkToolGUI.h>
#include <MitkExampleModuleExports.h>
#include <qlabel.h>
#include <qslider.h>
#include <qlayout.h>
#include <SmartRefinementSegTool2D.h>

// Look into SmartRefinementSegTool2D.h for more information.

class MITKEXAMPLEMODULE_EXPORT SmartRefinementSegTool2DGUI : public QmitkToolGUI
{
  Q_OBJECT

public:
  mitkClassMacro(SmartRefinementSegTool2DGUI, QmitkToolGUI)
  itkFactorylessNewMacro(Self)

protected slots:
  void OnNewToolAssociated(mitk::Tool *);
  void OnNewAnnotation();
  void OnBrushSizeChanged(int value);
  void OnSmoothnessChanged(int value);
  void OnSensitivityChanged(int value);
  void OnBorderPenaltyChanged(int value);
  void OnAreaPenaltyChanged(int value);
  void OnSetSavePath();
  void OnSaveMask();

protected:
  enum Parameter
  {
    BRUSH_SIZE,
    SMOOTHNESS,
    SENSITIVITY,
    BORDER_PENALTY,
    AREA_PENALTY,
    NOOF_PARAMETERS
  };

  SmartRefinementSegTool2DGUI();
  ~SmartRefinementSegTool2DGUI() override;

  void MakeSlider(Parameter param, const std::string &paramName,
    int minVal, int maxVal, int startVal, int step = 1);

  QBoxLayout *m_layout;
  QSlider *m_Slider[NOOF_PARAMETERS];
  QLabel *m_ValueLabel[NOOF_PARAMETERS];
  SmartRefinementSegTool2D::Pointer m_SmartRefinementTool;
  QString m_savePath;
};

#endif
