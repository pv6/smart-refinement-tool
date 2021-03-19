set(CPP_FILES
  ExampleImageFilter.cpp
  ExampleImageInteractor.cpp
  ExampleModule.cpp
  SmartRefinementSegTool2D.cpp
  SmartRefinementSegTool2DGUI.cpp
  ChanVeseLSM.cpp
  GeneralLSM.cpp
  LsmBase.cpp
  SmartBrushLevelSetImpl.cpp
)

set(UI_FILES
  src/SmartRefinementSegTool2DGUI.ui
)

set(MOC_H_FILES
  include/SmartRefinementSegTool2DGUI.h
)

set(RESOURCE_FILES
  Interactions/Paint.xml
  Interactions/PaintConfig.xml
  ExampleIcon.svg
  
  Interactions/dummy.xml
  Interactions/LiveWireTool.xml
  Interactions/FastMarchingTool.xml
  Interactions/PressMoveRelease.xml
  Interactions/PressMoveReleaseAndPointSetting.xml
  Interactions/PressMoveReleaseWithCTRLInversion.xml
  Interactions/PressMoveReleaseWithCTRLInversionAllMouseMoves.xml
  Interactions/PressMoveReleaseWithCTRLInversionAllMouseMovesConfig.xml
 )
