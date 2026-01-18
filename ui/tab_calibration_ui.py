# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'tab_calibration.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_TabCalibration(object):
    def setupUi(self, TabCalibration):
        if not TabCalibration.objectName():
            TabCalibration.setObjectName(u"TabCalibration")
        TabCalibration.resize(1200, 800)
        self.horizontalLayoutCalibration = QHBoxLayout(TabCalibration)
        self.horizontalLayoutCalibration.setObjectName(u"horizontalLayoutCalibration")
        self.widgetLeft = QWidget(TabCalibration)
        self.widgetLeft.setObjectName(u"widgetLeft")
        self.verticalLayoutLeft = QVBoxLayout(self.widgetLeft)
        self.verticalLayoutLeft.setObjectName(u"verticalLayoutLeft")
        self.verticalLayoutLeft.setContentsMargins(0, 0, 0, 0)
        self.groupCalibCameraStream = QGroupBox(self.widgetLeft)
        self.groupCalibCameraStream.setObjectName(u"groupCalibCameraStream")
        self.verticalLayoutCalibCamera = QVBoxLayout(self.groupCalibCameraStream)
        self.verticalLayoutCalibCamera.setObjectName(u"verticalLayoutCalibCamera")
        self.labelCalibCameraView = QLabel(self.groupCalibCameraStream)
        self.labelCalibCameraView.setObjectName(u"labelCalibCameraView")
        sizePolicy = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.labelCalibCameraView.sizePolicy().hasHeightForWidth())
        self.labelCalibCameraView.setSizePolicy(sizePolicy)
        self.labelCalibCameraView.setMinimumSize(QSize(640, 480))
        self.labelCalibCameraView.setAlignment(Qt.AlignCenter)

        self.verticalLayoutCalibCamera.addWidget(self.labelCalibCameraView)

        self.horizontalLayoutCalibCameraButtons = QHBoxLayout()
        self.horizontalLayoutCalibCameraButtons.setObjectName(u"horizontalLayoutCalibCameraButtons")
        self.btnCalibStartCamera = QPushButton(self.groupCalibCameraStream)
        self.btnCalibStartCamera.setObjectName(u"btnCalibStartCamera")

        self.horizontalLayoutCalibCameraButtons.addWidget(self.btnCalibStartCamera)

        self.btnCalibStopCamera = QPushButton(self.groupCalibCameraStream)
        self.btnCalibStopCamera.setObjectName(u"btnCalibStopCamera")

        self.horizontalLayoutCalibCameraButtons.addWidget(self.btnCalibStopCamera)

        self.btnCalibSnapshot = QPushButton(self.groupCalibCameraStream)
        self.btnCalibSnapshot.setObjectName(u"btnCalibSnapshot")

        self.horizontalLayoutCalibCameraButtons.addWidget(self.btnCalibSnapshot)


        self.verticalLayoutCalibCamera.addLayout(self.horizontalLayoutCalibCameraButtons)


        self.verticalLayoutLeft.addWidget(self.groupCalibCameraStream)

        self.tabWidgetCalibSub = QTabWidget(self.widgetLeft)
        self.tabWidgetCalibSub.setObjectName(u"tabWidgetCalibSub")
        self.tabChessboard = QWidget()
        self.tabChessboard.setObjectName(u"tabChessboard")
        self.verticalLayoutChessboardTab = QVBoxLayout(self.tabChessboard)
        self.verticalLayoutChessboardTab.setObjectName(u"verticalLayoutChessboardTab")
        self.horizontalLayoutChessboardSettings = QHBoxLayout()
        self.horizontalLayoutChessboardSettings.setObjectName(u"horizontalLayoutChessboardSettings")
        self.checkChessboardDetect = QCheckBox(self.tabChessboard)
        self.checkChessboardDetect.setObjectName(u"checkChessboardDetect")

        self.horizontalLayoutChessboardSettings.addWidget(self.checkChessboardDetect)

        self.labelChessboardCols = QLabel(self.tabChessboard)
        self.labelChessboardCols.setObjectName(u"labelChessboardCols")

        self.horizontalLayoutChessboardSettings.addWidget(self.labelChessboardCols)

        self.spinChessboardCols = QSpinBox(self.tabChessboard)
        self.spinChessboardCols.setObjectName(u"spinChessboardCols")
        self.spinChessboardCols.setMinimum(3)
        self.spinChessboardCols.setMaximum(20)
        self.spinChessboardCols.setValue(10)

        self.horizontalLayoutChessboardSettings.addWidget(self.spinChessboardCols)

        self.labelChessboardRows = QLabel(self.tabChessboard)
        self.labelChessboardRows.setObjectName(u"labelChessboardRows")

        self.horizontalLayoutChessboardSettings.addWidget(self.labelChessboardRows)

        self.spinChessboardRows = QSpinBox(self.tabChessboard)
        self.spinChessboardRows.setObjectName(u"spinChessboardRows")
        self.spinChessboardRows.setMinimum(3)
        self.spinChessboardRows.setMaximum(20)
        self.spinChessboardRows.setValue(7)

        self.horizontalLayoutChessboardSettings.addWidget(self.spinChessboardRows)

        self.labelSquareSize = QLabel(self.tabChessboard)
        self.labelSquareSize.setObjectName(u"labelSquareSize")

        self.horizontalLayoutChessboardSettings.addWidget(self.labelSquareSize)

        self.spinSquareSize = QDoubleSpinBox(self.tabChessboard)
        self.spinSquareSize.setObjectName(u"spinSquareSize")
        self.spinSquareSize.setMinimum(1.000000000000000)
        self.spinSquareSize.setMaximum(100.000000000000000)
        self.spinSquareSize.setSingleStep(0.500000000000000)
        self.spinSquareSize.setValue(25.000000000000000)

        self.horizontalLayoutChessboardSettings.addWidget(self.spinSquareSize)

        self.horizontalSpacerChessboard = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayoutChessboardSettings.addItem(self.horizontalSpacerChessboard)

        self.btnFindChessboardPose = QPushButton(self.tabChessboard)
        self.btnFindChessboardPose.setObjectName(u"btnFindChessboardPose")

        self.horizontalLayoutChessboardSettings.addWidget(self.btnFindChessboardPose)


        self.verticalLayoutChessboardTab.addLayout(self.horizontalLayoutChessboardSettings)

        self.horizontalLayoutChessboardPose = QHBoxLayout()
        self.horizontalLayoutChessboardPose.setObjectName(u"horizontalLayoutChessboardPose")
        self.labelChessboardCenter = QLabel(self.tabChessboard)
        self.labelChessboardCenter.setObjectName(u"labelChessboardCenter")

        self.horizontalLayoutChessboardPose.addWidget(self.labelChessboardCenter)

        self.labelChessboardCenterValue = QLabel(self.tabChessboard)
        self.labelChessboardCenterValue.setObjectName(u"labelChessboardCenterValue")

        self.horizontalLayoutChessboardPose.addWidget(self.labelChessboardCenterValue)

        self.labelChessboardAngle = QLabel(self.tabChessboard)
        self.labelChessboardAngle.setObjectName(u"labelChessboardAngle")

        self.horizontalLayoutChessboardPose.addWidget(self.labelChessboardAngle)

        self.labelChessboardAngleValue = QLabel(self.tabChessboard)
        self.labelChessboardAngleValue.setObjectName(u"labelChessboardAngleValue")

        self.horizontalLayoutChessboardPose.addWidget(self.labelChessboardAngleValue)

        self.horizontalSpacerPose = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayoutChessboardPose.addItem(self.horizontalSpacerPose)


        self.verticalLayoutChessboardTab.addLayout(self.horizontalLayoutChessboardPose)

        self.horizontalLayoutCalibCapture = QHBoxLayout()
        self.horizontalLayoutCalibCapture.setObjectName(u"horizontalLayoutCalibCapture")
        self.labelCapturedImages = QLabel(self.tabChessboard)
        self.labelCapturedImages.setObjectName(u"labelCapturedImages")

        self.horizontalLayoutCalibCapture.addWidget(self.labelCapturedImages)

        self.labelCapturedCount = QLabel(self.tabChessboard)
        self.labelCapturedCount.setObjectName(u"labelCapturedCount")

        self.horizontalLayoutCalibCapture.addWidget(self.labelCapturedCount)

        self.horizontalSpacerCaptureCount = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayoutCalibCapture.addItem(self.horizontalSpacerCaptureCount)

        self.btnCaptureCalibImage = QPushButton(self.tabChessboard)
        self.btnCaptureCalibImage.setObjectName(u"btnCaptureCalibImage")

        self.horizontalLayoutCalibCapture.addWidget(self.btnCaptureCalibImage)

        self.btnClearCalibImages = QPushButton(self.tabChessboard)
        self.btnClearCalibImages.setObjectName(u"btnClearCalibImages")

        self.horizontalLayoutCalibCapture.addWidget(self.btnClearCalibImages)

        self.btnRunCalibration = QPushButton(self.tabChessboard)
        self.btnRunCalibration.setObjectName(u"btnRunCalibration")

        self.horizontalLayoutCalibCapture.addWidget(self.btnRunCalibration)


        self.verticalLayoutChessboardTab.addLayout(self.horizontalLayoutCalibCapture)

        self.groupChessboardAlign = QGroupBox(self.tabChessboard)
        self.groupChessboardAlign.setObjectName(u"groupChessboardAlign")
        self.verticalLayoutChessboardAlign = QVBoxLayout(self.groupChessboardAlign)
        self.verticalLayoutChessboardAlign.setObjectName(u"verticalLayoutChessboardAlign")
        self.horizontalLayoutAlignButtons = QHBoxLayout()
        self.horizontalLayoutAlignButtons.setObjectName(u"horizontalLayoutAlignButtons")
        self.btnAlignCenter = QPushButton(self.groupChessboardAlign)
        self.btnAlignCenter.setObjectName(u"btnAlignCenter")

        self.horizontalLayoutAlignButtons.addWidget(self.btnAlignCenter)

        self.btnAlignAngle = QPushButton(self.groupChessboardAlign)
        self.btnAlignAngle.setObjectName(u"btnAlignAngle")

        self.horizontalLayoutAlignButtons.addWidget(self.btnAlignAngle)

        self.btnAlignBoth = QPushButton(self.groupChessboardAlign)
        self.btnAlignBoth.setObjectName(u"btnAlignBoth")

        self.horizontalLayoutAlignButtons.addWidget(self.btnAlignBoth)

        self.horizontalSpacerAlign = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayoutAlignButtons.addItem(self.horizontalSpacerAlign)


        self.verticalLayoutChessboardAlign.addLayout(self.horizontalLayoutAlignButtons)


        self.verticalLayoutChessboardTab.addWidget(self.groupChessboardAlign)

        self.tabWidgetCalibSub.addTab(self.tabChessboard, "")
        self.tabAutoCalib = QWidget()
        self.tabAutoCalib.setObjectName(u"tabAutoCalib")
        self.horizontalLayoutAutoCalibTab = QHBoxLayout(self.tabAutoCalib)
        self.horizontalLayoutAutoCalibTab.setObjectName(u"horizontalLayoutAutoCalibTab")
        self.labelNumPositions = QLabel(self.tabAutoCalib)
        self.labelNumPositions.setObjectName(u"labelNumPositions")

        self.horizontalLayoutAutoCalibTab.addWidget(self.labelNumPositions)

        self.spinNumPositions = QSpinBox(self.tabAutoCalib)
        self.spinNumPositions.setObjectName(u"spinNumPositions")
        self.spinNumPositions.setMinimum(5)
        self.spinNumPositions.setMaximum(30)
        self.spinNumPositions.setValue(10)

        self.horizontalLayoutAutoCalibTab.addWidget(self.spinNumPositions)

        self.btnStartAutoCalib = QPushButton(self.tabAutoCalib)
        self.btnStartAutoCalib.setObjectName(u"btnStartAutoCalib")

        self.horizontalLayoutAutoCalibTab.addWidget(self.btnStartAutoCalib)

        self.btnStopAutoCalib = QPushButton(self.tabAutoCalib)
        self.btnStopAutoCalib.setObjectName(u"btnStopAutoCalib")
        self.btnStopAutoCalib.setEnabled(False)

        self.horizontalLayoutAutoCalibTab.addWidget(self.btnStopAutoCalib)

        self.labelAutoCalibStatus = QLabel(self.tabAutoCalib)
        self.labelAutoCalibStatus.setObjectName(u"labelAutoCalibStatus")

        self.horizontalLayoutAutoCalibTab.addWidget(self.labelAutoCalibStatus)

        self.horizontalSpacerAutoCalib = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayoutAutoCalibTab.addItem(self.horizontalSpacerAutoCalib)

        self.tabWidgetCalibSub.addTab(self.tabAutoCalib, "")

        self.verticalLayoutLeft.addWidget(self.tabWidgetCalibSub)


        self.horizontalLayoutCalibration.addWidget(self.widgetLeft)

        self.groupCalibControls = QGroupBox(TabCalibration)
        self.groupCalibControls.setObjectName(u"groupCalibControls")
        self.groupCalibControls.setMinimumSize(QSize(300, 0))
        self.verticalLayoutCalibControls = QVBoxLayout(self.groupCalibControls)
        self.verticalLayoutCalibControls.setObjectName(u"verticalLayoutCalibControls")
        self.groupRobotPosition = QGroupBox(self.groupCalibControls)
        self.groupRobotPosition.setObjectName(u"groupRobotPosition")
        self.gridLayoutRobotPos = QGridLayout(self.groupRobotPosition)
        self.gridLayoutRobotPos.setObjectName(u"gridLayoutRobotPos")
        self.labelCalibX = QLabel(self.groupRobotPosition)
        self.labelCalibX.setObjectName(u"labelCalibX")

        self.gridLayoutRobotPos.addWidget(self.labelCalibX, 0, 0, 1, 1)

        self.editCalibX = QLineEdit(self.groupRobotPosition)
        self.editCalibX.setObjectName(u"editCalibX")
        self.editCalibX.setReadOnly(True)
        self.editCalibX.setAlignment(Qt.AlignRight)

        self.gridLayoutRobotPos.addWidget(self.editCalibX, 0, 1, 1, 1)

        self.labelCalibRx = QLabel(self.groupRobotPosition)
        self.labelCalibRx.setObjectName(u"labelCalibRx")

        self.gridLayoutRobotPos.addWidget(self.labelCalibRx, 0, 2, 1, 1)

        self.editCalibRx = QLineEdit(self.groupRobotPosition)
        self.editCalibRx.setObjectName(u"editCalibRx")
        self.editCalibRx.setReadOnly(True)
        self.editCalibRx.setAlignment(Qt.AlignRight)

        self.gridLayoutRobotPos.addWidget(self.editCalibRx, 0, 3, 1, 1)

        self.labelCalibY = QLabel(self.groupRobotPosition)
        self.labelCalibY.setObjectName(u"labelCalibY")

        self.gridLayoutRobotPos.addWidget(self.labelCalibY, 1, 0, 1, 1)

        self.editCalibY = QLineEdit(self.groupRobotPosition)
        self.editCalibY.setObjectName(u"editCalibY")
        self.editCalibY.setReadOnly(True)
        self.editCalibY.setAlignment(Qt.AlignRight)

        self.gridLayoutRobotPos.addWidget(self.editCalibY, 1, 1, 1, 1)

        self.labelCalibRy = QLabel(self.groupRobotPosition)
        self.labelCalibRy.setObjectName(u"labelCalibRy")

        self.gridLayoutRobotPos.addWidget(self.labelCalibRy, 1, 2, 1, 1)

        self.editCalibRy = QLineEdit(self.groupRobotPosition)
        self.editCalibRy.setObjectName(u"editCalibRy")
        self.editCalibRy.setReadOnly(True)
        self.editCalibRy.setAlignment(Qt.AlignRight)

        self.gridLayoutRobotPos.addWidget(self.editCalibRy, 1, 3, 1, 1)

        self.labelCalibZ = QLabel(self.groupRobotPosition)
        self.labelCalibZ.setObjectName(u"labelCalibZ")

        self.gridLayoutRobotPos.addWidget(self.labelCalibZ, 2, 0, 1, 1)

        self.editCalibZ = QLineEdit(self.groupRobotPosition)
        self.editCalibZ.setObjectName(u"editCalibZ")
        self.editCalibZ.setReadOnly(True)
        self.editCalibZ.setAlignment(Qt.AlignRight)

        self.gridLayoutRobotPos.addWidget(self.editCalibZ, 2, 1, 1, 1)

        self.labelCalibRz = QLabel(self.groupRobotPosition)
        self.labelCalibRz.setObjectName(u"labelCalibRz")

        self.gridLayoutRobotPos.addWidget(self.labelCalibRz, 2, 2, 1, 1)

        self.editCalibRz = QLineEdit(self.groupRobotPosition)
        self.editCalibRz.setObjectName(u"editCalibRz")
        self.editCalibRz.setReadOnly(True)
        self.editCalibRz.setAlignment(Qt.AlignRight)

        self.gridLayoutRobotPos.addWidget(self.editCalibRz, 2, 3, 1, 1)


        self.verticalLayoutCalibControls.addWidget(self.groupRobotPosition)

        self.groupCalibResult = QGroupBox(self.groupCalibControls)
        self.groupCalibResult.setObjectName(u"groupCalibResult")
        self.verticalLayoutCalibResult = QVBoxLayout(self.groupCalibResult)
        self.verticalLayoutCalibResult.setObjectName(u"verticalLayoutCalibResult")
        self.formLayoutCalibResult = QFormLayout()
        self.formLayoutCalibResult.setObjectName(u"formLayoutCalibResult")
        self.labelRMSError = QLabel(self.groupCalibResult)
        self.labelRMSError.setObjectName(u"labelRMSError")

        self.formLayoutCalibResult.setWidget(0, QFormLayout.LabelRole, self.labelRMSError)

        self.labelRMSErrorValue = QLabel(self.groupCalibResult)
        self.labelRMSErrorValue.setObjectName(u"labelRMSErrorValue")

        self.formLayoutCalibResult.setWidget(0, QFormLayout.FieldRole, self.labelRMSErrorValue)

        self.labelFx = QLabel(self.groupCalibResult)
        self.labelFx.setObjectName(u"labelFx")

        self.formLayoutCalibResult.setWidget(1, QFormLayout.LabelRole, self.labelFx)

        self.labelFxValue = QLabel(self.groupCalibResult)
        self.labelFxValue.setObjectName(u"labelFxValue")

        self.formLayoutCalibResult.setWidget(1, QFormLayout.FieldRole, self.labelFxValue)

        self.labelFy = QLabel(self.groupCalibResult)
        self.labelFy.setObjectName(u"labelFy")

        self.formLayoutCalibResult.setWidget(2, QFormLayout.LabelRole, self.labelFy)

        self.labelFyValue = QLabel(self.groupCalibResult)
        self.labelFyValue.setObjectName(u"labelFyValue")

        self.formLayoutCalibResult.setWidget(2, QFormLayout.FieldRole, self.labelFyValue)

        self.labelCx = QLabel(self.groupCalibResult)
        self.labelCx.setObjectName(u"labelCx")

        self.formLayoutCalibResult.setWidget(3, QFormLayout.LabelRole, self.labelCx)

        self.labelCxValue = QLabel(self.groupCalibResult)
        self.labelCxValue.setObjectName(u"labelCxValue")

        self.formLayoutCalibResult.setWidget(3, QFormLayout.FieldRole, self.labelCxValue)

        self.labelCy = QLabel(self.groupCalibResult)
        self.labelCy.setObjectName(u"labelCy")

        self.formLayoutCalibResult.setWidget(4, QFormLayout.LabelRole, self.labelCy)

        self.labelCyValue = QLabel(self.groupCalibResult)
        self.labelCyValue.setObjectName(u"labelCyValue")

        self.formLayoutCalibResult.setWidget(4, QFormLayout.FieldRole, self.labelCyValue)


        self.verticalLayoutCalibResult.addLayout(self.formLayoutCalibResult)

        self.btnSaveCalibration = QPushButton(self.groupCalibResult)
        self.btnSaveCalibration.setObjectName(u"btnSaveCalibration")

        self.verticalLayoutCalibResult.addWidget(self.btnSaveCalibration)

        self.btnLoadCalibration = QPushButton(self.groupCalibResult)
        self.btnLoadCalibration.setObjectName(u"btnLoadCalibration")

        self.verticalLayoutCalibResult.addWidget(self.btnLoadCalibration)


        self.verticalLayoutCalibControls.addWidget(self.groupCalibResult)

        self.verticalSpacerCalib = QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayoutCalibControls.addItem(self.verticalSpacerCalib)


        self.horizontalLayoutCalibration.addWidget(self.groupCalibControls)


        self.retranslateUi(TabCalibration)

        self.tabWidgetCalibSub.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(TabCalibration)
    # setupUi

    def retranslateUi(self, TabCalibration):
        self.groupCalibCameraStream.setTitle(QCoreApplication.translate("TabCalibration", u"Camera Stream", None))
        self.labelCalibCameraView.setStyleSheet(QCoreApplication.translate("TabCalibration", u"background-color: #333333; border: 1px solid #555555;", None))
        self.labelCalibCameraView.setText(QCoreApplication.translate("TabCalibration", u"\uce74\uba54\ub77c \ubbf8\uc5f0\uacb0", None))
        self.btnCalibStartCamera.setText(QCoreApplication.translate("TabCalibration", u"\uce74\uba54\ub77c \uc2dc\uc791", None))
        self.btnCalibStopCamera.setText(QCoreApplication.translate("TabCalibration", u"\uce74\uba54\ub77c \uc815\uc9c0", None))
        self.btnCalibSnapshot.setText(QCoreApplication.translate("TabCalibration", u"\uc2a4\ub0c5\uc0f7 \uc800\uc7a5", None))
        self.checkChessboardDetect.setText(QCoreApplication.translate("TabCalibration", u"\uac10\uc9c0", None))
        self.labelChessboardCols.setText(QCoreApplication.translate("TabCalibration", u"\uac00\ub85c:", None))
        self.labelChessboardRows.setText(QCoreApplication.translate("TabCalibration", u"\uc138\ub85c:", None))
        self.labelSquareSize.setText(QCoreApplication.translate("TabCalibration", u"\ud06c\uae30(mm):", None))
        self.btnFindChessboardPose.setText(QCoreApplication.translate("TabCalibration", u"Find Pose", None))
        self.btnFindChessboardPose.setStyleSheet(QCoreApplication.translate("TabCalibration", u"background-color: #FF9800; color: white;", None))
        self.labelChessboardCenter.setText(QCoreApplication.translate("TabCalibration", u"\uc911\uc2ec:", None))
        self.labelChessboardCenterValue.setText(QCoreApplication.translate("TabCalibration", u"-", None))
        self.labelChessboardCenterValue.setStyleSheet(QCoreApplication.translate("TabCalibration", u"font-weight: bold;", None))
        self.labelChessboardAngle.setText(QCoreApplication.translate("TabCalibration", u"\uac01\ub3c4:", None))
        self.labelChessboardAngleValue.setText(QCoreApplication.translate("TabCalibration", u"-", None))
        self.labelChessboardAngleValue.setStyleSheet(QCoreApplication.translate("TabCalibration", u"font-weight: bold;", None))
        self.labelCapturedImages.setText(QCoreApplication.translate("TabCalibration", u"\ucea1\ucc98\ub41c \uc774\ubbf8\uc9c0:", None))
        self.labelCapturedCount.setText(QCoreApplication.translate("TabCalibration", u"0", None))
        self.labelCapturedCount.setStyleSheet(QCoreApplication.translate("TabCalibration", u"font-weight: bold; color: #0066cc;", None))
        self.btnCaptureCalibImage.setText(QCoreApplication.translate("TabCalibration", u"\uc774\ubbf8\uc9c0 \ucea1\ucc98", None))
        self.btnClearCalibImages.setText(QCoreApplication.translate("TabCalibration", u"\ucd08\uae30\ud654", None))
        self.btnRunCalibration.setText(QCoreApplication.translate("TabCalibration", u"\uce98\ub9ac\ube0c\ub808\uc774\uc158 \uc2e4\ud589", None))
        self.btnRunCalibration.setStyleSheet(QCoreApplication.translate("TabCalibration", u"background-color: #4CAF50; color: white;", None))
        self.groupChessboardAlign.setTitle(QCoreApplication.translate("TabCalibration", u"Chessboard Robot Align", None))
        self.btnAlignCenter.setText(QCoreApplication.translate("TabCalibration", u"\uc911\uc2ec \uc815\ub82c", None))
        self.btnAlignAngle.setText(QCoreApplication.translate("TabCalibration", u"\uac01\ub3c4 \uc815\ub82c", None))
        self.btnAlignBoth.setText(QCoreApplication.translate("TabCalibration", u"\uc804\uccb4 \uc815\ub82c", None))
        self.btnAlignBoth.setStyleSheet(QCoreApplication.translate("TabCalibration", u"background-color: #9C27B0; color: white;", None))
        self.tabWidgetCalibSub.setTabText(self.tabWidgetCalibSub.indexOf(self.tabChessboard), QCoreApplication.translate("TabCalibration", u"Chessboard", None))
        self.labelNumPositions.setText(QCoreApplication.translate("TabCalibration", u"\uc704\uce58 \uc218:", None))
        self.btnStartAutoCalib.setText(QCoreApplication.translate("TabCalibration", u"\uc2dc\uc791", None))
        self.btnStartAutoCalib.setStyleSheet(QCoreApplication.translate("TabCalibration", u"background-color: #2196F3; color: white;", None))
        self.btnStopAutoCalib.setText(QCoreApplication.translate("TabCalibration", u"\uc911\uc9c0", None))
        self.labelAutoCalibStatus.setText(QCoreApplication.translate("TabCalibration", u"\uc0c1\ud0dc: \ub300\uae30", None))
        self.tabWidgetCalibSub.setTabText(self.tabWidgetCalibSub.indexOf(self.tabAutoCalib), QCoreApplication.translate("TabCalibration", u"\uc790\ub3d9 \uce98\ub9ac\ube0c\ub808\uc774\uc158", None))
        self.groupCalibControls.setTitle(QCoreApplication.translate("TabCalibration", u"\uce98\ub9ac\ube0c\ub808\uc774\uc158", None))
        self.groupRobotPosition.setTitle(QCoreApplication.translate("TabCalibration", u"\ub85c\ubd07 \uc88c\ud45c", None))
        self.labelCalibX.setText(QCoreApplication.translate("TabCalibration", u"X:", None))
        self.labelCalibRx.setText(QCoreApplication.translate("TabCalibration", u"Rx:", None))
        self.labelCalibY.setText(QCoreApplication.translate("TabCalibration", u"Y:", None))
        self.labelCalibRy.setText(QCoreApplication.translate("TabCalibration", u"Ry:", None))
        self.labelCalibZ.setText(QCoreApplication.translate("TabCalibration", u"Z:", None))
        self.labelCalibRz.setText(QCoreApplication.translate("TabCalibration", u"Rz:", None))
        self.groupCalibResult.setTitle(QCoreApplication.translate("TabCalibration", u"\uce98\ub9ac\ube0c\ub808\uc774\uc158 \uacb0\uacfc", None))
        self.labelRMSError.setText(QCoreApplication.translate("TabCalibration", u"RMS \uc624\ucc28:", None))
        self.labelRMSErrorValue.setText(QCoreApplication.translate("TabCalibration", u"-", None))
        self.labelFx.setText(QCoreApplication.translate("TabCalibration", u"fx:", None))
        self.labelFxValue.setText(QCoreApplication.translate("TabCalibration", u"-", None))
        self.labelFy.setText(QCoreApplication.translate("TabCalibration", u"fy:", None))
        self.labelFyValue.setText(QCoreApplication.translate("TabCalibration", u"-", None))
        self.labelCx.setText(QCoreApplication.translate("TabCalibration", u"cx:", None))
        self.labelCxValue.setText(QCoreApplication.translate("TabCalibration", u"-", None))
        self.labelCy.setText(QCoreApplication.translate("TabCalibration", u"cy:", None))
        self.labelCyValue.setText(QCoreApplication.translate("TabCalibration", u"-", None))
        self.btnSaveCalibration.setText(QCoreApplication.translate("TabCalibration", u"\uce98\ub9ac\ube0c\ub808\uc774\uc158 \uc800\uc7a5", None))
        self.btnLoadCalibration.setText(QCoreApplication.translate("TabCalibration", u"\uce98\ub9ac\ube0c\ub808\uc774\uc158 \ub85c\ub4dc", None))
        pass
    # retranslateUi

