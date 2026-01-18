# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'tab_vision.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_TabVision(object):
    def setupUi(self, TabVision):
        if not TabVision.objectName():
            TabVision.setObjectName(u"TabVision")
        TabVision.resize(1200, 800)
        self.horizontalLayoutVision = QHBoxLayout(TabVision)
        self.horizontalLayoutVision.setObjectName(u"horizontalLayoutVision")
        self.groupCameraStream = QGroupBox(TabVision)
        self.groupCameraStream.setObjectName(u"groupCameraStream")
        self.verticalLayoutCamera = QVBoxLayout(self.groupCameraStream)
        self.verticalLayoutCamera.setObjectName(u"verticalLayoutCamera")
        self.labelCameraView = QLabel(self.groupCameraStream)
        self.labelCameraView.setObjectName(u"labelCameraView")
        self.labelCameraView.setMinimumSize(QSize(640, 480))
        self.labelCameraView.setAlignment(Qt.AlignCenter)

        self.verticalLayoutCamera.addWidget(self.labelCameraView)

        self.horizontalLayoutCameraButtons = QHBoxLayout()
        self.horizontalLayoutCameraButtons.setObjectName(u"horizontalLayoutCameraButtons")
        self.btnStartCamera = QPushButton(self.groupCameraStream)
        self.btnStartCamera.setObjectName(u"btnStartCamera")

        self.horizontalLayoutCameraButtons.addWidget(self.btnStartCamera)

        self.btnStopCamera = QPushButton(self.groupCameraStream)
        self.btnStopCamera.setObjectName(u"btnStopCamera")

        self.horizontalLayoutCameraButtons.addWidget(self.btnStopCamera)

        self.btnSnapshot = QPushButton(self.groupCameraStream)
        self.btnSnapshot.setObjectName(u"btnSnapshot")

        self.horizontalLayoutCameraButtons.addWidget(self.btnSnapshot)


        self.verticalLayoutCamera.addLayout(self.horizontalLayoutCameraButtons)

        self.horizontalLayoutGamma = QHBoxLayout()
        self.horizontalLayoutGamma.setObjectName(u"horizontalLayoutGamma")
        self.labelGamma = QLabel(self.groupCameraStream)
        self.labelGamma.setObjectName(u"labelGamma")

        self.horizontalLayoutGamma.addWidget(self.labelGamma)

        self.sliderGamma = QSlider(self.groupCameraStream)
        self.sliderGamma.setObjectName(u"sliderGamma")
        self.sliderGamma.setMinimum(50)
        self.sliderGamma.setMaximum(200)
        self.sliderGamma.setValue(100)
        self.sliderGamma.setOrientation(Qt.Horizontal)

        self.horizontalLayoutGamma.addWidget(self.sliderGamma)

        self.labelGammaValue = QLabel(self.groupCameraStream)
        self.labelGammaValue.setObjectName(u"labelGammaValue")
        self.labelGammaValue.setMinimumSize(QSize(40, 0))

        self.horizontalLayoutGamma.addWidget(self.labelGammaValue)


        self.verticalLayoutCamera.addLayout(self.horizontalLayoutGamma)

        self.horizontalLayoutOverlay = QHBoxLayout()
        self.horizontalLayoutOverlay.setObjectName(u"horizontalLayoutOverlay")
        self.checkShowPoseAxes = QCheckBox(self.groupCameraStream)
        self.checkShowPoseAxes.setObjectName(u"checkShowPoseAxes")
        self.checkShowPoseAxes.setChecked(True)

        self.horizontalLayoutOverlay.addWidget(self.checkShowPoseAxes)

        self.checkShowBoundingBox = QCheckBox(self.groupCameraStream)
        self.checkShowBoundingBox.setObjectName(u"checkShowBoundingBox")
        self.checkShowBoundingBox.setChecked(True)

        self.horizontalLayoutOverlay.addWidget(self.checkShowBoundingBox)

        self.checkShowKeypoints = QCheckBox(self.groupCameraStream)
        self.checkShowKeypoints.setObjectName(u"checkShowKeypoints")
        self.checkShowKeypoints.setChecked(True)

        self.horizontalLayoutOverlay.addWidget(self.checkShowKeypoints)


        self.verticalLayoutCamera.addLayout(self.horizontalLayoutOverlay)


        self.horizontalLayoutVision.addWidget(self.groupCameraStream)

        self.groupDetectionInfo = QGroupBox(TabVision)
        self.groupDetectionInfo.setObjectName(u"groupDetectionInfo")
        self.groupDetectionInfo.setMinimumSize(QSize(350, 0))
        self.verticalLayoutDetection = QVBoxLayout(self.groupDetectionInfo)
        self.verticalLayoutDetection.setObjectName(u"verticalLayoutDetection")
        self.groupVisionProcessor = QGroupBox(self.groupDetectionInfo)
        self.groupVisionProcessor.setObjectName(u"groupVisionProcessor")
        self.verticalLayoutVP = QVBoxLayout(self.groupVisionProcessor)
        self.verticalLayoutVP.setObjectName(u"verticalLayoutVP")
        self.radioProcessorGun = QRadioButton(self.groupVisionProcessor)
        self.radioProcessorGun.setObjectName(u"radioProcessorGun")
        self.radioProcessorGun.setChecked(True)

        self.verticalLayoutVP.addWidget(self.radioProcessorGun)

        self.radioProcessorPort = QRadioButton(self.groupVisionProcessor)
        self.radioProcessorPort.setObjectName(u"radioProcessorPort")

        self.verticalLayoutVP.addWidget(self.radioProcessorPort)

        self.radioProcessorStandard = QRadioButton(self.groupVisionProcessor)
        self.radioProcessorStandard.setObjectName(u"radioProcessorStandard")

        self.verticalLayoutVP.addWidget(self.radioProcessorStandard)


        self.verticalLayoutDetection.addWidget(self.groupVisionProcessor)

        self.groupDetectionResult = QGroupBox(self.groupDetectionInfo)
        self.groupDetectionResult.setObjectName(u"groupDetectionResult")
        self.formLayoutDetection = QFormLayout(self.groupDetectionResult)
        self.formLayoutDetection.setObjectName(u"formLayoutDetection")
        self.labelArUcoID = QLabel(self.groupDetectionResult)
        self.labelArUcoID.setObjectName(u"labelArUcoID")

        self.formLayoutDetection.setWidget(0, QFormLayout.LabelRole, self.labelArUcoID)

        self.labelArUcoIDValue = QLabel(self.groupDetectionResult)
        self.labelArUcoIDValue.setObjectName(u"labelArUcoIDValue")

        self.formLayoutDetection.setWidget(0, QFormLayout.FieldRole, self.labelArUcoIDValue)

        self.labelConfidence = QLabel(self.groupDetectionResult)
        self.labelConfidence.setObjectName(u"labelConfidence")

        self.formLayoutDetection.setWidget(1, QFormLayout.LabelRole, self.labelConfidence)

        self.progressConfidence = QProgressBar(self.groupDetectionResult)
        self.progressConfidence.setObjectName(u"progressConfidence")
        self.progressConfidence.setValue(0)

        self.formLayoutDetection.setWidget(1, QFormLayout.FieldRole, self.progressConfidence)

        self.labelConvergence = QLabel(self.groupDetectionResult)
        self.labelConvergence.setObjectName(u"labelConvergence")

        self.formLayoutDetection.setWidget(2, QFormLayout.LabelRole, self.labelConvergence)

        self.progressConvergence = QProgressBar(self.groupDetectionResult)
        self.progressConvergence.setObjectName(u"progressConvergence")
        self.progressConvergence.setValue(0)

        self.formLayoutDetection.setWidget(2, QFormLayout.FieldRole, self.progressConvergence)


        self.verticalLayoutDetection.addWidget(self.groupDetectionResult)

        self.groupPoseCamera = QGroupBox(self.groupDetectionInfo)
        self.groupPoseCamera.setObjectName(u"groupPoseCamera")
        self.gridLayoutPoseCamera = QGridLayout(self.groupPoseCamera)
        self.gridLayoutPoseCamera.setObjectName(u"gridLayoutPoseCamera")
        self.labelCamX = QLabel(self.groupPoseCamera)
        self.labelCamX.setObjectName(u"labelCamX")

        self.gridLayoutPoseCamera.addWidget(self.labelCamX, 0, 0, 1, 1)

        self.editCamX = QLineEdit(self.groupPoseCamera)
        self.editCamX.setObjectName(u"editCamX")
        self.editCamX.setReadOnly(True)

        self.gridLayoutPoseCamera.addWidget(self.editCamX, 0, 1, 1, 1)

        self.labelCamRx = QLabel(self.groupPoseCamera)
        self.labelCamRx.setObjectName(u"labelCamRx")

        self.gridLayoutPoseCamera.addWidget(self.labelCamRx, 0, 2, 1, 1)

        self.editCamRx = QLineEdit(self.groupPoseCamera)
        self.editCamRx.setObjectName(u"editCamRx")
        self.editCamRx.setReadOnly(True)

        self.gridLayoutPoseCamera.addWidget(self.editCamRx, 0, 3, 1, 1)

        self.labelCamY = QLabel(self.groupPoseCamera)
        self.labelCamY.setObjectName(u"labelCamY")

        self.gridLayoutPoseCamera.addWidget(self.labelCamY, 1, 0, 1, 1)

        self.editCamY = QLineEdit(self.groupPoseCamera)
        self.editCamY.setObjectName(u"editCamY")
        self.editCamY.setReadOnly(True)

        self.gridLayoutPoseCamera.addWidget(self.editCamY, 1, 1, 1, 1)

        self.labelCamRy = QLabel(self.groupPoseCamera)
        self.labelCamRy.setObjectName(u"labelCamRy")

        self.gridLayoutPoseCamera.addWidget(self.labelCamRy, 1, 2, 1, 1)

        self.editCamRy = QLineEdit(self.groupPoseCamera)
        self.editCamRy.setObjectName(u"editCamRy")
        self.editCamRy.setReadOnly(True)

        self.gridLayoutPoseCamera.addWidget(self.editCamRy, 1, 3, 1, 1)

        self.labelCamZ = QLabel(self.groupPoseCamera)
        self.labelCamZ.setObjectName(u"labelCamZ")

        self.gridLayoutPoseCamera.addWidget(self.labelCamZ, 2, 0, 1, 1)

        self.editCamZ = QLineEdit(self.groupPoseCamera)
        self.editCamZ.setObjectName(u"editCamZ")
        self.editCamZ.setReadOnly(True)

        self.gridLayoutPoseCamera.addWidget(self.editCamZ, 2, 1, 1, 1)

        self.labelCamRz = QLabel(self.groupPoseCamera)
        self.labelCamRz.setObjectName(u"labelCamRz")

        self.gridLayoutPoseCamera.addWidget(self.labelCamRz, 2, 2, 1, 1)

        self.editCamRz = QLineEdit(self.groupPoseCamera)
        self.editCamRz.setObjectName(u"editCamRz")
        self.editCamRz.setReadOnly(True)

        self.gridLayoutPoseCamera.addWidget(self.editCamRz, 2, 3, 1, 1)


        self.verticalLayoutDetection.addWidget(self.groupPoseCamera)

        self.groupPoseWorld = QGroupBox(self.groupDetectionInfo)
        self.groupPoseWorld.setObjectName(u"groupPoseWorld")
        self.gridLayoutPoseWorld = QGridLayout(self.groupPoseWorld)
        self.gridLayoutPoseWorld.setObjectName(u"gridLayoutPoseWorld")
        self.labelWorldX = QLabel(self.groupPoseWorld)
        self.labelWorldX.setObjectName(u"labelWorldX")

        self.gridLayoutPoseWorld.addWidget(self.labelWorldX, 0, 0, 1, 1)

        self.editWorldX = QLineEdit(self.groupPoseWorld)
        self.editWorldX.setObjectName(u"editWorldX")
        self.editWorldX.setReadOnly(True)

        self.gridLayoutPoseWorld.addWidget(self.editWorldX, 0, 1, 1, 1)

        self.labelWorldRx = QLabel(self.groupPoseWorld)
        self.labelWorldRx.setObjectName(u"labelWorldRx")

        self.gridLayoutPoseWorld.addWidget(self.labelWorldRx, 0, 2, 1, 1)

        self.editWorldRx = QLineEdit(self.groupPoseWorld)
        self.editWorldRx.setObjectName(u"editWorldRx")
        self.editWorldRx.setReadOnly(True)

        self.gridLayoutPoseWorld.addWidget(self.editWorldRx, 0, 3, 1, 1)

        self.labelWorldY = QLabel(self.groupPoseWorld)
        self.labelWorldY.setObjectName(u"labelWorldY")

        self.gridLayoutPoseWorld.addWidget(self.labelWorldY, 1, 0, 1, 1)

        self.editWorldY = QLineEdit(self.groupPoseWorld)
        self.editWorldY.setObjectName(u"editWorldY")
        self.editWorldY.setReadOnly(True)

        self.gridLayoutPoseWorld.addWidget(self.editWorldY, 1, 1, 1, 1)

        self.labelWorldRy = QLabel(self.groupPoseWorld)
        self.labelWorldRy.setObjectName(u"labelWorldRy")

        self.gridLayoutPoseWorld.addWidget(self.labelWorldRy, 1, 2, 1, 1)

        self.editWorldRy = QLineEdit(self.groupPoseWorld)
        self.editWorldRy.setObjectName(u"editWorldRy")
        self.editWorldRy.setReadOnly(True)

        self.gridLayoutPoseWorld.addWidget(self.editWorldRy, 1, 3, 1, 1)

        self.labelWorldZ = QLabel(self.groupPoseWorld)
        self.labelWorldZ.setObjectName(u"labelWorldZ")

        self.gridLayoutPoseWorld.addWidget(self.labelWorldZ, 2, 0, 1, 1)

        self.editWorldZ = QLineEdit(self.groupPoseWorld)
        self.editWorldZ.setObjectName(u"editWorldZ")
        self.editWorldZ.setReadOnly(True)

        self.gridLayoutPoseWorld.addWidget(self.editWorldZ, 2, 1, 1, 1)

        self.labelWorldRz = QLabel(self.groupPoseWorld)
        self.labelWorldRz.setObjectName(u"labelWorldRz")

        self.gridLayoutPoseWorld.addWidget(self.labelWorldRz, 2, 2, 1, 1)

        self.editWorldRz = QLineEdit(self.groupPoseWorld)
        self.editWorldRz.setObjectName(u"editWorldRz")
        self.editWorldRz.setReadOnly(True)

        self.gridLayoutPoseWorld.addWidget(self.editWorldRz, 2, 3, 1, 1)


        self.verticalLayoutDetection.addWidget(self.groupPoseWorld)

        self.groupArucoAlignment = QGroupBox(self.groupDetectionInfo)
        self.groupArucoAlignment.setObjectName(u"groupArucoAlignment")
        self.layoutArucoAlignment = QVBoxLayout(self.groupArucoAlignment)
        self.layoutArucoAlignment.setObjectName(u"layoutArucoAlignment")
        self.hboxLayout = QHBoxLayout()
        self.hboxLayout.setObjectName(u"hboxLayout")
        self.labelTargetTagId = QLabel(self.groupArucoAlignment)
        self.labelTargetTagId.setObjectName(u"labelTargetTagId")

        self.hboxLayout.addWidget(self.labelTargetTagId)

        self.spinTargetTagId = QSpinBox(self.groupArucoAlignment)
        self.spinTargetTagId.setObjectName(u"spinTargetTagId")
        self.spinTargetTagId.setMinimum(0)
        self.spinTargetTagId.setMaximum(255)

        self.hboxLayout.addWidget(self.spinTargetTagId)

        self.labelNumSamples = QLabel(self.groupArucoAlignment)
        self.labelNumSamples.setObjectName(u"labelNumSamples")

        self.hboxLayout.addWidget(self.labelNumSamples)

        self.spinNumSamples = QSpinBox(self.groupArucoAlignment)
        self.spinNumSamples.setObjectName(u"spinNumSamples")
        self.spinNumSamples.setMinimum(1)
        self.spinNumSamples.setMaximum(100)
        self.spinNumSamples.setValue(10)

        self.hboxLayout.addWidget(self.spinNumSamples)


        self.layoutArucoAlignment.addLayout(self.hboxLayout)

        self.hboxLayout1 = QHBoxLayout()
        self.hboxLayout1.setObjectName(u"hboxLayout1")
        self.btnAlignCenter = QPushButton(self.groupArucoAlignment)
        self.btnAlignCenter.setObjectName(u"btnAlignCenter")

        self.hboxLayout1.addWidget(self.btnAlignCenter)

        self.btnAlignPose = QPushButton(self.groupArucoAlignment)
        self.btnAlignPose.setObjectName(u"btnAlignPose")

        self.hboxLayout1.addWidget(self.btnAlignPose)

        self.btnAlignFull = QPushButton(self.groupArucoAlignment)
        self.btnAlignFull.setObjectName(u"btnAlignFull")

        self.hboxLayout1.addWidget(self.btnAlignFull)


        self.layoutArucoAlignment.addLayout(self.hboxLayout1)

        self.hboxLayout2 = QHBoxLayout()
        self.hboxLayout2.setObjectName(u"hboxLayout2")
        self.labelAlignStatus = QLabel(self.groupArucoAlignment)
        self.labelAlignStatus.setObjectName(u"labelAlignStatus")

        self.hboxLayout2.addWidget(self.labelAlignStatus)


        self.layoutArucoAlignment.addLayout(self.hboxLayout2)


        self.verticalLayoutDetection.addWidget(self.groupArucoAlignment)

        self.groupDataCollection = QGroupBox(self.groupDetectionInfo)
        self.groupDataCollection.setObjectName(u"groupDataCollection")
        self.layoutDataCollection = QVBoxLayout(self.groupDataCollection)
        self.layoutDataCollection.setObjectName(u"layoutDataCollection")
        self.hboxLayout3 = QHBoxLayout()
        self.hboxLayout3.setObjectName(u"hboxLayout3")
        self.labelCollectTagId = QLabel(self.groupDataCollection)
        self.labelCollectTagId.setObjectName(u"labelCollectTagId")

        self.hboxLayout3.addWidget(self.labelCollectTagId)

        self.spinCollectTagId = QSpinBox(self.groupDataCollection)
        self.spinCollectTagId.setObjectName(u"spinCollectTagId")
        self.spinCollectTagId.setMinimum(0)
        self.spinCollectTagId.setMaximum(255)

        self.hboxLayout3.addWidget(self.spinCollectTagId)

        self.labelCollectCount = QLabel(self.groupDataCollection)
        self.labelCollectCount.setObjectName(u"labelCollectCount")

        self.hboxLayout3.addWidget(self.labelCollectCount)

        self.spinCollectCount = QSpinBox(self.groupDataCollection)
        self.spinCollectCount.setObjectName(u"spinCollectCount")
        self.spinCollectCount.setMinimum(10)
        self.spinCollectCount.setMaximum(1000)
        self.spinCollectCount.setValue(100)

        self.hboxLayout3.addWidget(self.spinCollectCount)


        self.layoutDataCollection.addLayout(self.hboxLayout3)

        self.hboxLayout4 = QHBoxLayout()
        self.hboxLayout4.setObjectName(u"hboxLayout4")
        self.btnStartCollect = QPushButton(self.groupDataCollection)
        self.btnStartCollect.setObjectName(u"btnStartCollect")

        self.hboxLayout4.addWidget(self.btnStartCollect)

        self.btnStopCollect = QPushButton(self.groupDataCollection)
        self.btnStopCollect.setObjectName(u"btnStopCollect")
        self.btnStopCollect.setEnabled(False)

        self.hboxLayout4.addWidget(self.btnStopCollect)

        self.btnSaveCollect = QPushButton(self.groupDataCollection)
        self.btnSaveCollect.setObjectName(u"btnSaveCollect")
        self.btnSaveCollect.setEnabled(False)

        self.hboxLayout4.addWidget(self.btnSaveCollect)


        self.layoutDataCollection.addLayout(self.hboxLayout4)

        self.hboxLayout5 = QHBoxLayout()
        self.hboxLayout5.setObjectName(u"hboxLayout5")
        self.labelCollectStatus = QLabel(self.groupDataCollection)
        self.labelCollectStatus.setObjectName(u"labelCollectStatus")

        self.hboxLayout5.addWidget(self.labelCollectStatus)

        self.progressCollect = QProgressBar(self.groupDataCollection)
        self.progressCollect.setObjectName(u"progressCollect")
        self.progressCollect.setValue(0)

        self.hboxLayout5.addWidget(self.progressCollect)


        self.layoutDataCollection.addLayout(self.hboxLayout5)


        self.verticalLayoutDetection.addWidget(self.groupDataCollection)

        self.verticalSpacerVision = QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayoutDetection.addItem(self.verticalSpacerVision)


        self.horizontalLayoutVision.addWidget(self.groupDetectionInfo)


        self.retranslateUi(TabVision)

        QMetaObject.connectSlotsByName(TabVision)
    # setupUi

    def retranslateUi(self, TabVision):
        self.groupCameraStream.setTitle(QCoreApplication.translate("TabVision", u"Camera Stream", None))
        self.labelCameraView.setStyleSheet(QCoreApplication.translate("TabVision", u"background-color: #333333; border: 1px solid #555555;", None))
        self.labelCameraView.setText(QCoreApplication.translate("TabVision", u"\uce74\uba54\ub77c \ubbf8\uc5f0\uacb0", None))
        self.btnStartCamera.setText(QCoreApplication.translate("TabVision", u"\uce74\uba54\ub77c \uc2dc\uc791", None))
        self.btnStopCamera.setText(QCoreApplication.translate("TabVision", u"\uce74\uba54\ub77c \uc815\uc9c0", None))
        self.btnSnapshot.setText(QCoreApplication.translate("TabVision", u"\uc2a4\ub0c5\uc0f7 \uc800\uc7a5", None))
        self.labelGamma.setText(QCoreApplication.translate("TabVision", u"\uac10\ub9c8 \ubcf4\uc815:", None))
        self.labelGammaValue.setText(QCoreApplication.translate("TabVision", u"1.0", None))
        self.checkShowPoseAxes.setText(QCoreApplication.translate("TabVision", u"\ud3ec\uc988 \ucd95 \ud45c\uc2dc", None))
        self.checkShowBoundingBox.setText(QCoreApplication.translate("TabVision", u"\ubc14\uc6b4\ub529 \ubc15\uc2a4", None))
        self.checkShowKeypoints.setText(QCoreApplication.translate("TabVision", u"\ud0a4\ud3ec\uc778\ud2b8", None))
        self.groupDetectionInfo.setTitle(QCoreApplication.translate("TabVision", u"Detection Info", None))
        self.groupVisionProcessor.setTitle(QCoreApplication.translate("TabVision", u"Vision Processor", None))
        self.radioProcessorGun.setText(QCoreApplication.translate("TabVision", u"Gun (\ucda9\uc804\uac74)", None))
        self.radioProcessorPort.setText(QCoreApplication.translate("TabVision", u"Port (\ucda9\uc804 \ud3ec\ud2b8)", None))
        self.radioProcessorStandard.setText(QCoreApplication.translate("TabVision", u"Standard (\ucc28\ub7c9 \ucda9\uc804\ud3ec\ud2b8)", None))
        self.groupDetectionResult.setTitle(QCoreApplication.translate("TabVision", u"\uac10\uc9c0 \uacb0\uacfc", None))
        self.labelArUcoID.setText(QCoreApplication.translate("TabVision", u"ArUco ID:", None))
        self.labelArUcoIDValue.setText(QCoreApplication.translate("TabVision", u"-", None))
        self.labelConfidence.setText(QCoreApplication.translate("TabVision", u"\uc2e0\ub8b0\ub3c4:", None))
        self.labelConvergence.setText(QCoreApplication.translate("TabVision", u"\uc218\ub834\ub960:", None))
        self.groupPoseCamera.setTitle(QCoreApplication.translate("TabVision", u"Pose (Camera \uc88c\ud45c)", None))
        self.labelCamX.setText(QCoreApplication.translate("TabVision", u"X:", None))
        self.labelCamRx.setText(QCoreApplication.translate("TabVision", u"Rx:", None))
        self.labelCamY.setText(QCoreApplication.translate("TabVision", u"Y:", None))
        self.labelCamRy.setText(QCoreApplication.translate("TabVision", u"Ry:", None))
        self.labelCamZ.setText(QCoreApplication.translate("TabVision", u"Z:", None))
        self.labelCamRz.setText(QCoreApplication.translate("TabVision", u"Rz:", None))
        self.groupPoseWorld.setTitle(QCoreApplication.translate("TabVision", u"Pose (World \uc88c\ud45c)", None))
        self.labelWorldX.setText(QCoreApplication.translate("TabVision", u"X:", None))
        self.labelWorldRx.setText(QCoreApplication.translate("TabVision", u"Rx:", None))
        self.labelWorldY.setText(QCoreApplication.translate("TabVision", u"Y:", None))
        self.labelWorldRy.setText(QCoreApplication.translate("TabVision", u"Ry:", None))
        self.labelWorldZ.setText(QCoreApplication.translate("TabVision", u"Z:", None))
        self.labelWorldRz.setText(QCoreApplication.translate("TabVision", u"Rz:", None))
        self.groupArucoAlignment.setTitle(QCoreApplication.translate("TabVision", u"Aruco \uc815\ub82c \ud14c\uc2a4\ud2b8", None))
        self.labelTargetTagId.setText(QCoreApplication.translate("TabVision", u"Tag ID:", None))
        self.labelNumSamples.setText(QCoreApplication.translate("TabVision", u"\uc0d8\ud50c \uc218:", None))
        self.btnAlignCenter.setText(QCoreApplication.translate("TabVision", u"\uc911\uc2ec \uc815\ub82c", None))
        self.btnAlignPose.setText(QCoreApplication.translate("TabVision", u"\uc790\uc138 \uc815\ub82c", None))
        self.btnAlignFull.setText(QCoreApplication.translate("TabVision", u"\uc804\uccb4 \uc815\ub82c", None))
        self.labelAlignStatus.setText(QCoreApplication.translate("TabVision", u"\uc0c1\ud0dc: \ub300\uae30", None))
        self.groupDataCollection.setTitle(QCoreApplication.translate("TabVision", u"\ub370\uc774\ud130 \uc218\uc9d1 (\ub178\uc774\uc988 \ubd84\uc11d\uc6a9)", None))
        self.labelCollectTagId.setText(QCoreApplication.translate("TabVision", u"Tag ID:", None))
        self.labelCollectCount.setText(QCoreApplication.translate("TabVision", u"\uc218\uc9d1 \ud69f\uc218:", None))
        self.btnStartCollect.setText(QCoreApplication.translate("TabVision", u"\uc218\uc9d1 \uc2dc\uc791", None))
        self.btnStopCollect.setText(QCoreApplication.translate("TabVision", u"\uc218\uc9d1 \uc911\uc9c0", None))
        self.btnSaveCollect.setText(QCoreApplication.translate("TabVision", u"CSV \uc800\uc7a5", None))
        self.labelCollectStatus.setText(QCoreApplication.translate("TabVision", u"\uc218\uc9d1: 0 / 100", None))
        pass
    # retranslateUi

