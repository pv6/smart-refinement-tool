/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#include <SmartRefinementSegTool2DGUI.h>
#include <ui_SmartRefinementSegTool2DGUI.h>

#include <QFileDialog>
#include <qpushbutton.h>

MITK_TOOL_GUI_MACRO(MITKEXAMPLEMODULE_EXPORT, SmartRefinementSegTool2DGUI, "")

void SmartRefinementSegTool2DGUI::MakeSlider(Parameter param,
  const std::string &paramName, int minVal, int maxVal, int startVal, int step)
{
  QVBoxLayout *sliderLayout = new QVBoxLayout();

  QHBoxLayout *labelLayout = new QHBoxLayout();

  QLabel *label = new QLabel(paramName.c_str(), this);
  QFont f = label->font();
  f.setBold(false);
  label->setFont(f);
  labelLayout->addWidget(label);

  m_ValueLabel[param] = new QLabel(" 10", this);
  f = m_ValueLabel[param]->font();
  f.setBold(false);
  m_ValueLabel[param]->setFont(f);
  labelLayout->addWidget(m_ValueLabel[param]);

  sliderLayout->addLayout(labelLayout);

  m_Slider[param] = new QSlider(Qt::Horizontal, this);
  m_Slider[param]->setMinimum(minVal);
  m_Slider[param]->setMaximum(maxVal);
  m_Slider[param]->setPageStep(step);
  m_Slider[param]->setValue(startVal);
  sliderLayout->addWidget(m_Slider[param]);

  m_layout->addLayout(sliderLayout);

  connect(this, SIGNAL(NewToolAssociated(mitk::Tool *)), this, SLOT(OnNewToolAssociated(mitk::Tool *)));
}

SmartRefinementSegTool2DGUI::SmartRefinementSegTool2DGUI()
{
  // create ui layout
  m_layout = new QVBoxLayout(this);
  this->setContentsMargins(0, 0, 0, 0);

  // new annotation button
  auto button = new QPushButton();
  button->setText("Start new annotation");
  connect(button, SIGNAL(released()), this, SLOT(OnNewAnnotation()));
  m_layout->addWidget(button);

  //// set save path button
  //button = new QPushButton();
  //button->setText("Set save path");
  //connect(button, SIGNAL(released()), this, SLOT(OnSetSavePath()));
  //m_layout->addWidget(button);
  //
  //// save mask button
  //button = new QPushButton();
  //button->setText("Save mask");
  //connect(button, SIGNAL(released()), this, SLOT(OnSaveMask()));
  //m_layout->addWidget(button);

  // brush size slider
  MakeSlider(BRUSH_SIZE, "Brush size", 1, 10, 3);
  connect(m_Slider[BRUSH_SIZE], SIGNAL(valueChanged(int)), this,
    SLOT(OnBrushSizeChanged(int)));
  
  //// smoothness slider
  //MakeSlider(SMOOTHNESS, "Smoothness", 1, 20, 10);
  //connect(m_Slider[SMOOTHNESS], SIGNAL(valueChanged(int)), this,
  //  SLOT(OnSmoothnessChanged(int)));
  //
  //// border penalty slider
  //MakeSlider(BORDER_PENALTY, "Border penalty", 1, 20, 10);
  //connect(m_Slider[BORDER_PENALTY], SIGNAL(valueChanged(int)), this,
  //  SLOT(OnBorderPenaltyChanged(int)));

  // area penalty slider
  MakeSlider(AREA_PENALTY, "Sensitivity", 1, 20, 10);
  connect(m_Slider[AREA_PENALTY], SIGNAL(valueChanged(int)), this,
    SLOT(OnAreaPenaltyChanged(int)));

  //// sensitivity slider
  //MakeSlider(SENSITIVITY, "Sensitivity", 1, 20, 10);
  //connect(m_Slider[SENSITIVITY], SIGNAL(valueChanged(int)), this,
  //  SLOT(OnSensitivityChanged(int)));
}

void SmartRefinementSegTool2DGUI::OnSetSavePath() {
  //m_savePath = QFileDialog::getSaveFileName();
  if (m_SmartRefinementTool.IsNotNull())
    m_SmartRefinementTool->Segment();
}

void SmartRefinementSegTool2DGUI::OnNewAnnotation()
{
  if (m_SmartRefinementTool.IsNotNull())
    m_SmartRefinementTool->NewAnnotation();
}

void SmartRefinementSegTool2DGUI::OnBrushSizeChanged(int value)
{
  if (m_SmartRefinementTool.IsNotNull())
    m_SmartRefinementTool->ChangeBrushRadius(value);
  m_ValueLabel[BRUSH_SIZE]->setText(QString("%1 ").arg(value));
}

void SmartRefinementSegTool2DGUI::OnSmoothnessChanged(int value)
{
  if (m_SmartRefinementTool.IsNotNull())
    m_SmartRefinementTool->ChangeSmoothness(value / 20.0);
  m_ValueLabel[SMOOTHNESS]->setText(QString("%1 ").arg(value / 20.0));
}
void SmartRefinementSegTool2DGUI::OnSensitivityChanged(int value)
{
  if (m_SmartRefinementTool.IsNotNull())
    m_SmartRefinementTool->ChangeSensitivity(value / 20.0);
  m_ValueLabel[SENSITIVITY]->setText(QString("%1 ").arg(value / 20.0));
}
void SmartRefinementSegTool2DGUI::OnBorderPenaltyChanged(int value)
{
    if (m_SmartRefinementTool.IsNotNull())
    {
        m_SmartRefinementTool->ChangeBorderPenalty(value / 20.0);
    }
  m_ValueLabel[BORDER_PENALTY]->setText(QString("%1 ").arg(value / 20.0));
}

void SmartRefinementSegTool2DGUI::OnAreaPenaltyChanged(int value)
{
    if (m_SmartRefinementTool.IsNotNull())
    {
        m_SmartRefinementTool->ChangeAreaPenalty(value / 20.0);
    }
  m_ValueLabel[AREA_PENALTY]->setText(QString("%1 ").arg(value / 20.0));
}

void SmartRefinementSegTool2DGUI::OnNewToolAssociated(mitk::Tool *tool)
{
  MITK_INFO << "tool set";
  m_SmartRefinementTool = dynamic_cast<SmartRefinementSegTool2D *>(tool);
  m_SmartRefinementTool->ChangeSensitivity(0.5);
  m_SmartRefinementTool->ChangeSmoothness(0.05);
  m_SmartRefinementTool->ChangeBorderPenalty(0.5);
}

SmartRefinementSegTool2DGUI::~SmartRefinementSegTool2DGUI()
{
}

void SmartRefinementSegTool2DGUI::OnSaveMask()
{
  static int i = 0;
  QString file_name_copy = m_savePath;
  file_name_copy.append("_");
  file_name_copy.append(std::to_string(i++).c_str());
  MITK_INFO << "saving " << file_name_copy.toStdString();
  if (m_SmartRefinementTool.IsNotNull() && !file_name_copy.isNull())
    m_SmartRefinementTool->SaveMask(file_name_copy.toStdString());
}

