# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'tab_eye_in_hand.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_TabEyeInHand(object):
    def setupUi(self, TabEyeInHand):
        if not TabEyeInHand.objectName():
            TabEyeInHand.setObjectName(u"TabEyeInHand")
        TabEyeInHand.resize(1200, 800)
        self.horizontalLayoutEyeInHand = QHBoxLayout(TabEyeInHand)
        self.horizontalLayoutEyeInHand.setObjectName(u"horizontalLayoutEyeInHand")
        self.groupEyeInHandCameraStream = QGroupBox(TabEyeInHand)
        self.groupEyeInHandCameraStream.setObjectName(u"groupEyeInHandCameraStream")
        self.verticalLayoutEyeInHandCamera = QVBoxLayout(self.groupEyeInHandCameraStream)
        self.verticalLayoutEyeInHandCamera.setObjectName(u"verticalLayoutEyeInHandCamera")
        self.labelEyeInHandCameraView = QLabel(self.groupEyeInHandCameraStream)
        self.labelEyeInHandCameraView.setObjectName(u"labelEyeInHandCameraView")
        sizePolicy = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.labelEyeInHandCameraView.sizePolicy().hasHeightForWidth())
        self.labelEyeInHandCameraView.setSizePolicy(sizePolicy)
        self.labelEyeInHandCameraView.setMinimumSize(QSize(640, 480))
        self.labelEyeInHandCameraView.setAlignment(Qt.AlignCenter)

        self.verticalLayoutEyeInHandCamera.addWidget(self.labelEyeInHandCameraView)

        self.horizontalLayoutEyeInHandCameraButtons = QHBoxLayout()
        self.horizontalLayoutEyeInHandCameraButtons.setObjectName(u"horizontalLayoutEyeInHandCameraButtons")
        self.btnEyeInHandStartCamera = QPushButton(self.groupEyeInHandCameraStream)
        self.btnEyeInHandStartCamera.setObjectName(u"btnEyeInHandStartCamera")

        self.horizontalLayoutEyeInHandCameraButtons.addWidget(self.btnEyeInHandStartCamera)

        self.btnEyeInHandStopCamera = QPushButton(self.groupEyeInHandCameraStream)
        self.btnEyeInHandStopCamera.setObjectName(u"btnEyeInHandStopCamera")

        self.horizontalLayoutEyeInHandCameraButtons.addWidget(self.btnEyeInHandStopCamera)

        self.btnEyeInHandSnapshot = QPushButton(self.groupEyeInHandCameraStream)
        self.btnEyeInHandSnapshot.setObjectName(u"btnEyeInHandSnapshot")

        self.horizontalLayoutEyeInHandCameraButtons.addWidget(self.btnEyeInHandSnapshot)


        self.verticalLayoutEyeInHandCamera.addLayout(self.horizontalLayoutEyeInHandCameraButtons)


        self.horizontalLayoutEyeInHand.addWidget(self.groupEyeInHandCameraStream)

        self.groupEyeInHandControls = QGroupBox(TabEyeInHand)
        self.groupEyeInHandControls.setObjectName(u"groupEyeInHandControls")
        self.groupEyeInHandControls.setMinimumSize(QSize(350, 0))
        self.verticalLayoutEyeInHandControls = QVBoxLayout(self.groupEyeInHandControls)
        self.verticalLayoutEyeInHandControls.setObjectName(u"verticalLayoutEyeInHandControls")
        self.labelEyeInHandPlaceholder = QLabel(self.groupEyeInHandControls)
        self.labelEyeInHandPlaceholder.setObjectName(u"labelEyeInHandPlaceholder")
        self.labelEyeInHandPlaceholder.setAlignment(Qt.AlignCenter)

        self.verticalLayoutEyeInHandControls.addWidget(self.labelEyeInHandPlaceholder)

        self.verticalSpacerEyeInHand = QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayoutEyeInHandControls.addItem(self.verticalSpacerEyeInHand)


        self.horizontalLayoutEyeInHand.addWidget(self.groupEyeInHandControls)


        self.retranslateUi(TabEyeInHand)

        QMetaObject.connectSlotsByName(TabEyeInHand)
    # setupUi

    def retranslateUi(self, TabEyeInHand):
        self.groupEyeInHandCameraStream.setTitle(QCoreApplication.translate("TabEyeInHand", u"Camera Stream", None))
        self.labelEyeInHandCameraView.setStyleSheet(QCoreApplication.translate("TabEyeInHand", u"background-color: #333333; border: 1px solid #555555;", None))
        self.labelEyeInHandCameraView.setText(QCoreApplication.translate("TabEyeInHand", u"\uce74\uba54\ub77c \ubbf8\uc5f0\uacb0", None))
        self.btnEyeInHandStartCamera.setText(QCoreApplication.translate("TabEyeInHand", u"\uce74\uba54\ub77c \uc2dc\uc791", None))
        self.btnEyeInHandStopCamera.setText(QCoreApplication.translate("TabEyeInHand", u"\uce74\uba54\ub77c \uc815\uc9c0", None))
        self.btnEyeInHandSnapshot.setText(QCoreApplication.translate("TabEyeInHand", u"\uc2a4\ub0c5\uc0f7 \uc800\uc7a5", None))
        self.groupEyeInHandControls.setTitle(QCoreApplication.translate("TabEyeInHand", u"Eye in Hand \ucee8\ud2b8\ub864", None))
        self.labelEyeInHandPlaceholder.setText(QCoreApplication.translate("TabEyeInHand", u"Eye in Hand \ucee8\ud2b8\ub864 (\ucd94\ud6c4 \uad6c\ud604 \uc608\uc815)", None))
        pass
    # retranslateUi

