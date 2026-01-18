# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'tab_task_edit.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_TabTaskEdit(object):
    def setupUi(self, TabTaskEdit):
        if not TabTaskEdit.objectName():
            TabTaskEdit.setObjectName(u"TabTaskEdit")
        TabTaskEdit.resize(1200, 800)
        self.horizontalLayoutTask = QHBoxLayout(TabTaskEdit)
        self.horizontalLayoutTask.setObjectName(u"horizontalLayoutTask")
        self.groupTaskSequence = QGroupBox(TabTaskEdit)
        self.groupTaskSequence.setObjectName(u"groupTaskSequence")
        self.groupTaskSequence.setMinimumSize(QSize(280, 0))
        self.verticalLayoutSequence = QVBoxLayout(self.groupTaskSequence)
        self.verticalLayoutSequence.setObjectName(u"verticalLayoutSequence")
        self.listTaskSequence = QListWidget(self.groupTaskSequence)
        self.listTaskSequence.setObjectName(u"listTaskSequence")
        self.listTaskSequence.setAlternatingRowColors(True)

        self.verticalLayoutSequence.addWidget(self.listTaskSequence)

        self.horizontalLayoutSeqButtons = QHBoxLayout()
        self.horizontalLayoutSeqButtons.setObjectName(u"horizontalLayoutSeqButtons")
        self.btnMoveUp = QPushButton(self.groupTaskSequence)
        self.btnMoveUp.setObjectName(u"btnMoveUp")
        self.btnMoveUp.setMaximumSize(QSize(40, 16777215))

        self.horizontalLayoutSeqButtons.addWidget(self.btnMoveUp)

        self.btnMoveDown = QPushButton(self.groupTaskSequence)
        self.btnMoveDown.setObjectName(u"btnMoveDown")
        self.btnMoveDown.setMaximumSize(QSize(40, 16777215))

        self.horizontalLayoutSeqButtons.addWidget(self.btnMoveDown)

        self.horizontalSpacer = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayoutSeqButtons.addItem(self.horizontalSpacer)

        self.btnDeleteTask = QPushButton(self.groupTaskSequence)
        self.btnDeleteTask.setObjectName(u"btnDeleteTask")

        self.horizontalLayoutSeqButtons.addWidget(self.btnDeleteTask)


        self.verticalLayoutSequence.addLayout(self.horizontalLayoutSeqButtons)


        self.horizontalLayoutTask.addWidget(self.groupTaskSequence)

        self.groupAvailableTasks = QGroupBox(TabTaskEdit)
        self.groupAvailableTasks.setObjectName(u"groupAvailableTasks")
        self.groupAvailableTasks.setMinimumSize(QSize(280, 0))
        self.verticalLayoutAvailable = QVBoxLayout(self.groupAvailableTasks)
        self.verticalLayoutAvailable.setObjectName(u"verticalLayoutAvailable")
        self.treeAvailableTasks = QTreeWidget(self.groupAvailableTasks)
        self.treeAvailableTasks.setObjectName(u"treeAvailableTasks")
        self.treeAvailableTasks.setAlternatingRowColors(True)

        self.verticalLayoutAvailable.addWidget(self.treeAvailableTasks)

        self.btnAddTask = QPushButton(self.groupAvailableTasks)
        self.btnAddTask.setObjectName(u"btnAddTask")

        self.verticalLayoutAvailable.addWidget(self.btnAddTask)


        self.horizontalLayoutTask.addWidget(self.groupAvailableTasks)

        self.groupTaskParams = QGroupBox(TabTaskEdit)
        self.groupTaskParams.setObjectName(u"groupTaskParams")
        self.groupTaskParams.setMinimumSize(QSize(300, 0))
        self.verticalLayoutParams = QVBoxLayout(self.groupTaskParams)
        self.verticalLayoutParams.setObjectName(u"verticalLayoutParams")
        self.labelSelectedTask = QLabel(self.groupTaskParams)
        self.labelSelectedTask.setObjectName(u"labelSelectedTask")
        font = QFont()
        font.setBold(True)
        font.setWeight(75)
        self.labelSelectedTask.setFont(font)

        self.verticalLayoutParams.addWidget(self.labelSelectedTask)

        self.labelTaskType = QLabel(self.groupTaskParams)
        self.labelTaskType.setObjectName(u"labelTaskType")

        self.verticalLayoutParams.addWidget(self.labelTaskType)

        self.line = QFrame(self.groupTaskParams)
        self.line.setObjectName(u"line")
        self.line.setFrameShape(QFrame.HLine)
        self.line.setFrameShadow(QFrame.Sunken)

        self.verticalLayoutParams.addWidget(self.line)

        self.scrollAreaParams = QScrollArea(self.groupTaskParams)
        self.scrollAreaParams.setObjectName(u"scrollAreaParams")
        self.scrollAreaParams.setWidgetResizable(True)
        self.scrollAreaWidgetContents = QWidget()
        self.scrollAreaWidgetContents.setObjectName(u"scrollAreaWidgetContents")
        self.scrollAreaWidgetContents.setGeometry(QRect(0, 0, 278, 400))
        self.formLayoutParams = QFormLayout(self.scrollAreaWidgetContents)
        self.formLayoutParams.setObjectName(u"formLayoutParams")
        self.scrollAreaParams.setWidget(self.scrollAreaWidgetContents)

        self.verticalLayoutParams.addWidget(self.scrollAreaParams)

        self.horizontalLayoutParamButtons = QHBoxLayout()
        self.horizontalLayoutParamButtons.setObjectName(u"horizontalLayoutParamButtons")
        self.btnApplyParams = QPushButton(self.groupTaskParams)
        self.btnApplyParams.setObjectName(u"btnApplyParams")

        self.horizontalLayoutParamButtons.addWidget(self.btnApplyParams)

        self.btnTeachPosition = QPushButton(self.groupTaskParams)
        self.btnTeachPosition.setObjectName(u"btnTeachPosition")

        self.horizontalLayoutParamButtons.addWidget(self.btnTeachPosition)


        self.verticalLayoutParams.addLayout(self.horizontalLayoutParamButtons)


        self.horizontalLayoutTask.addWidget(self.groupTaskParams)

        self.groupRobotStatus = QGroupBox(TabTaskEdit)
        self.groupRobotStatus.setObjectName(u"groupRobotStatus")
        self.groupRobotStatus.setMinimumSize(QSize(280, 0))
        self.verticalLayoutRobot = QVBoxLayout(self.groupRobotStatus)
        self.verticalLayoutRobot.setObjectName(u"verticalLayoutRobot")
        self.groupNetwork = QGroupBox(self.groupRobotStatus)
        self.groupNetwork.setObjectName(u"groupNetwork")
        self.formLayoutNetwork = QFormLayout(self.groupNetwork)
        self.formLayoutNetwork.setObjectName(u"formLayoutNetwork")
        self.labelPCIP = QLabel(self.groupNetwork)
        self.labelPCIP.setObjectName(u"labelPCIP")

        self.formLayoutNetwork.setWidget(0, QFormLayout.LabelRole, self.labelPCIP)

        self.comboPCIP = QComboBox(self.groupNetwork)
        self.comboPCIP.setObjectName(u"comboPCIP")
        self.comboPCIP.setMinimumSize(QSize(150, 0))

        self.formLayoutNetwork.setWidget(0, QFormLayout.FieldRole, self.comboPCIP)

        self.labelRobotIP = QLabel(self.groupNetwork)
        self.labelRobotIP.setObjectName(u"labelRobotIP")

        self.formLayoutNetwork.setWidget(1, QFormLayout.LabelRole, self.labelRobotIP)

        self.horizontalLayoutIP = QHBoxLayout()
        self.horizontalLayoutIP.setObjectName(u"horizontalLayoutIP")
        self.editRobotIP = QLineEdit(self.groupNetwork)
        self.editRobotIP.setObjectName(u"editRobotIP")

        self.horizontalLayoutIP.addWidget(self.editRobotIP)

        self.btnConnect = QPushButton(self.groupNetwork)
        self.btnConnect.setObjectName(u"btnConnect")
        self.btnConnect.setMaximumSize(QSize(60, 16777215))

        self.horizontalLayoutIP.addWidget(self.btnConnect)


        self.formLayoutNetwork.setLayout(1, QFormLayout.FieldRole, self.horizontalLayoutIP)

        self.labelModbusPort = QLabel(self.groupNetwork)
        self.labelModbusPort.setObjectName(u"labelModbusPort")

        self.formLayoutNetwork.setWidget(2, QFormLayout.LabelRole, self.labelModbusPort)

        self.spinModbusPort = QSpinBox(self.groupNetwork)
        self.spinModbusPort.setObjectName(u"spinModbusPort")
        self.spinModbusPort.setMinimum(1)
        self.spinModbusPort.setMaximum(65535)
        self.spinModbusPort.setValue(1502)

        self.formLayoutNetwork.setWidget(2, QFormLayout.FieldRole, self.spinModbusPort)

        self.labelConnectionStatus = QLabel(self.groupNetwork)
        self.labelConnectionStatus.setObjectName(u"labelConnectionStatus")

        self.formLayoutNetwork.setWidget(3, QFormLayout.LabelRole, self.labelConnectionStatus)

        self.labelConnectionStatusValue = QLabel(self.groupNetwork)
        self.labelConnectionStatusValue.setObjectName(u"labelConnectionStatusValue")

        self.formLayoutNetwork.setWidget(3, QFormLayout.FieldRole, self.labelConnectionStatusValue)


        self.verticalLayoutRobot.addWidget(self.groupNetwork)

        self.groupJointPosition = QGroupBox(self.groupRobotStatus)
        self.groupJointPosition.setObjectName(u"groupJointPosition")
        self.gridLayoutJoint = QGridLayout(self.groupJointPosition)
        self.gridLayoutJoint.setObjectName(u"gridLayoutJoint")
        self.labelJ1 = QLabel(self.groupJointPosition)
        self.labelJ1.setObjectName(u"labelJ1")

        self.gridLayoutJoint.addWidget(self.labelJ1, 0, 0, 1, 1)

        self.editJ1 = QLineEdit(self.groupJointPosition)
        self.editJ1.setObjectName(u"editJ1")
        self.editJ1.setReadOnly(True)

        self.gridLayoutJoint.addWidget(self.editJ1, 0, 1, 1, 1)

        self.labelJ2 = QLabel(self.groupJointPosition)
        self.labelJ2.setObjectName(u"labelJ2")

        self.gridLayoutJoint.addWidget(self.labelJ2, 0, 2, 1, 1)

        self.editJ2 = QLineEdit(self.groupJointPosition)
        self.editJ2.setObjectName(u"editJ2")
        self.editJ2.setReadOnly(True)

        self.gridLayoutJoint.addWidget(self.editJ2, 0, 3, 1, 1)

        self.labelJ3 = QLabel(self.groupJointPosition)
        self.labelJ3.setObjectName(u"labelJ3")

        self.gridLayoutJoint.addWidget(self.labelJ3, 1, 0, 1, 1)

        self.editJ3 = QLineEdit(self.groupJointPosition)
        self.editJ3.setObjectName(u"editJ3")
        self.editJ3.setReadOnly(True)

        self.gridLayoutJoint.addWidget(self.editJ3, 1, 1, 1, 1)

        self.labelJ4 = QLabel(self.groupJointPosition)
        self.labelJ4.setObjectName(u"labelJ4")

        self.gridLayoutJoint.addWidget(self.labelJ4, 1, 2, 1, 1)

        self.editJ4 = QLineEdit(self.groupJointPosition)
        self.editJ4.setObjectName(u"editJ4")
        self.editJ4.setReadOnly(True)

        self.gridLayoutJoint.addWidget(self.editJ4, 1, 3, 1, 1)

        self.labelJ5 = QLabel(self.groupJointPosition)
        self.labelJ5.setObjectName(u"labelJ5")

        self.gridLayoutJoint.addWidget(self.labelJ5, 2, 0, 1, 1)

        self.editJ5 = QLineEdit(self.groupJointPosition)
        self.editJ5.setObjectName(u"editJ5")
        self.editJ5.setReadOnly(True)

        self.gridLayoutJoint.addWidget(self.editJ5, 2, 1, 1, 1)

        self.labelJ6 = QLabel(self.groupJointPosition)
        self.labelJ6.setObjectName(u"labelJ6")

        self.gridLayoutJoint.addWidget(self.labelJ6, 2, 2, 1, 1)

        self.editJ6 = QLineEdit(self.groupJointPosition)
        self.editJ6.setObjectName(u"editJ6")
        self.editJ6.setReadOnly(True)

        self.gridLayoutJoint.addWidget(self.editJ6, 2, 3, 1, 1)


        self.verticalLayoutRobot.addWidget(self.groupJointPosition)

        self.groupTCPPosition = QGroupBox(self.groupRobotStatus)
        self.groupTCPPosition.setObjectName(u"groupTCPPosition")
        self.gridLayoutTCP = QGridLayout(self.groupTCPPosition)
        self.gridLayoutTCP.setObjectName(u"gridLayoutTCP")
        self.labelX = QLabel(self.groupTCPPosition)
        self.labelX.setObjectName(u"labelX")

        self.gridLayoutTCP.addWidget(self.labelX, 0, 0, 1, 1)

        self.editX = QLineEdit(self.groupTCPPosition)
        self.editX.setObjectName(u"editX")
        self.editX.setReadOnly(True)

        self.gridLayoutTCP.addWidget(self.editX, 0, 1, 1, 1)

        self.labelUnitX = QLabel(self.groupTCPPosition)
        self.labelUnitX.setObjectName(u"labelUnitX")

        self.gridLayoutTCP.addWidget(self.labelUnitX, 0, 2, 1, 1)

        self.labelRx = QLabel(self.groupTCPPosition)
        self.labelRx.setObjectName(u"labelRx")

        self.gridLayoutTCP.addWidget(self.labelRx, 0, 3, 1, 1)

        self.editRx = QLineEdit(self.groupTCPPosition)
        self.editRx.setObjectName(u"editRx")
        self.editRx.setReadOnly(True)

        self.gridLayoutTCP.addWidget(self.editRx, 0, 4, 1, 1)

        self.labelUnitRx = QLabel(self.groupTCPPosition)
        self.labelUnitRx.setObjectName(u"labelUnitRx")

        self.gridLayoutTCP.addWidget(self.labelUnitRx, 0, 5, 1, 1)

        self.labelY = QLabel(self.groupTCPPosition)
        self.labelY.setObjectName(u"labelY")

        self.gridLayoutTCP.addWidget(self.labelY, 1, 0, 1, 1)

        self.editY = QLineEdit(self.groupTCPPosition)
        self.editY.setObjectName(u"editY")
        self.editY.setReadOnly(True)

        self.gridLayoutTCP.addWidget(self.editY, 1, 1, 1, 1)

        self.labelUnitY = QLabel(self.groupTCPPosition)
        self.labelUnitY.setObjectName(u"labelUnitY")

        self.gridLayoutTCP.addWidget(self.labelUnitY, 1, 2, 1, 1)

        self.labelRy = QLabel(self.groupTCPPosition)
        self.labelRy.setObjectName(u"labelRy")

        self.gridLayoutTCP.addWidget(self.labelRy, 1, 3, 1, 1)

        self.editRy = QLineEdit(self.groupTCPPosition)
        self.editRy.setObjectName(u"editRy")
        self.editRy.setReadOnly(True)

        self.gridLayoutTCP.addWidget(self.editRy, 1, 4, 1, 1)

        self.labelUnitRy = QLabel(self.groupTCPPosition)
        self.labelUnitRy.setObjectName(u"labelUnitRy")

        self.gridLayoutTCP.addWidget(self.labelUnitRy, 1, 5, 1, 1)

        self.labelZ = QLabel(self.groupTCPPosition)
        self.labelZ.setObjectName(u"labelZ")

        self.gridLayoutTCP.addWidget(self.labelZ, 2, 0, 1, 1)

        self.editZ = QLineEdit(self.groupTCPPosition)
        self.editZ.setObjectName(u"editZ")
        self.editZ.setReadOnly(True)

        self.gridLayoutTCP.addWidget(self.editZ, 2, 1, 1, 1)

        self.labelUnitZ = QLabel(self.groupTCPPosition)
        self.labelUnitZ.setObjectName(u"labelUnitZ")

        self.gridLayoutTCP.addWidget(self.labelUnitZ, 2, 2, 1, 1)

        self.labelRz = QLabel(self.groupTCPPosition)
        self.labelRz.setObjectName(u"labelRz")

        self.gridLayoutTCP.addWidget(self.labelRz, 2, 3, 1, 1)

        self.editRz = QLineEdit(self.groupTCPPosition)
        self.editRz.setObjectName(u"editRz")
        self.editRz.setReadOnly(True)

        self.gridLayoutTCP.addWidget(self.editRz, 2, 4, 1, 1)

        self.labelUnitRz = QLabel(self.groupTCPPosition)
        self.labelUnitRz.setObjectName(u"labelUnitRz")

        self.gridLayoutTCP.addWidget(self.labelUnitRz, 2, 5, 1, 1)


        self.verticalLayoutRobot.addWidget(self.groupTCPPosition)

        self.horizontalLayoutHomeButtons = QHBoxLayout()
        self.horizontalLayoutHomeButtons.setObjectName(u"horizontalLayoutHomeButtons")
        self.btnGoHome = QPushButton(self.groupRobotStatus)
        self.btnGoHome.setObjectName(u"btnGoHome")

        self.horizontalLayoutHomeButtons.addWidget(self.btnGoHome)

        self.btnSetHome = QPushButton(self.groupRobotStatus)
        self.btnSetHome.setObjectName(u"btnSetHome")

        self.horizontalLayoutHomeButtons.addWidget(self.btnSetHome)


        self.verticalLayoutRobot.addLayout(self.horizontalLayoutHomeButtons)

        self.groupSavedPoses = QGroupBox(self.groupRobotStatus)
        self.groupSavedPoses.setObjectName(u"groupSavedPoses")
        self.verticalLayoutSavedPoses = QVBoxLayout(self.groupSavedPoses)
        self.verticalLayoutSavedPoses.setObjectName(u"verticalLayoutSavedPoses")
        self.listSavedPoses = QListWidget(self.groupSavedPoses)
        self.listSavedPoses.setObjectName(u"listSavedPoses")
        self.listSavedPoses.setMaximumSize(QSize(16777215, 150))
        self.listSavedPoses.setAlternatingRowColors(True)

        self.verticalLayoutSavedPoses.addWidget(self.listSavedPoses)

        self.gridLayoutPoseButtons = QGridLayout()
        self.gridLayoutPoseButtons.setObjectName(u"gridLayoutPoseButtons")
        self.btnSavePose = QPushButton(self.groupSavedPoses)
        self.btnSavePose.setObjectName(u"btnSavePose")

        self.gridLayoutPoseButtons.addWidget(self.btnSavePose, 0, 0, 1, 1)

        self.btnDeletePose = QPushButton(self.groupSavedPoses)
        self.btnDeletePose.setObjectName(u"btnDeletePose")

        self.gridLayoutPoseButtons.addWidget(self.btnDeletePose, 0, 1, 1, 1)

        self.btnMoveToPose = QPushButton(self.groupSavedPoses)
        self.btnMoveToPose.setObjectName(u"btnMoveToPose")

        self.gridLayoutPoseButtons.addWidget(self.btnMoveToPose, 1, 0, 1, 1)

        self.btnApproachPose = QPushButton(self.groupSavedPoses)
        self.btnApproachPose.setObjectName(u"btnApproachPose")

        self.gridLayoutPoseButtons.addWidget(self.btnApproachPose, 1, 1, 1, 1)


        self.verticalLayoutSavedPoses.addLayout(self.gridLayoutPoseButtons)


        self.verticalLayoutRobot.addWidget(self.groupSavedPoses)

        self.verticalSpacerRobot = QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayoutRobot.addItem(self.verticalSpacerRobot)


        self.horizontalLayoutTask.addWidget(self.groupRobotStatus)


        self.retranslateUi(TabTaskEdit)

        QMetaObject.connectSlotsByName(TabTaskEdit)
    # setupUi

    def retranslateUi(self, TabTaskEdit):
        self.groupTaskSequence.setTitle(QCoreApplication.translate("TabTaskEdit", u"Task Sequence", None))
        self.btnMoveUp.setText(QCoreApplication.translate("TabTaskEdit", u"\u25b2", None))
        self.btnMoveDown.setText(QCoreApplication.translate("TabTaskEdit", u"\u25bc", None))
        self.btnDeleteTask.setText(QCoreApplication.translate("TabTaskEdit", u"\uc0ad\uc81c", None))
        self.groupAvailableTasks.setTitle(QCoreApplication.translate("TabTaskEdit", u"Available Tasks", None))
        ___qtreewidgetitem = self.treeAvailableTasks.headerItem()
        ___qtreewidgetitem.setText(0, QCoreApplication.translate("TabTaskEdit", u"Task", None));
        self.btnAddTask.setText(QCoreApplication.translate("TabTaskEdit", u"\u2190 Sequence\uc5d0 \ucd94\uac00", None))
        self.groupTaskParams.setTitle(QCoreApplication.translate("TabTaskEdit", u"Task Parameters", None))
        self.labelSelectedTask.setText(QCoreApplication.translate("TabTaskEdit", u"\uc120\ud0dd\ub41c Task \uc5c6\uc74c", None))
        self.labelTaskType.setText("")
        self.labelTaskType.setStyleSheet(QCoreApplication.translate("TabTaskEdit", u"color: gray;", None))
        self.btnApplyParams.setText(QCoreApplication.translate("TabTaskEdit", u"\ud30c\ub77c\ubbf8\ud130 \uc801\uc6a9", None))
        self.btnTeachPosition.setText(QCoreApplication.translate("TabTaskEdit", u"\ud604\uc7ac\uc704\uce58 \uc785\ub825", None))
        self.groupRobotStatus.setTitle(QCoreApplication.translate("TabTaskEdit", u"Robot Status", None))
        self.groupNetwork.setTitle(QCoreApplication.translate("TabTaskEdit", u"Network", None))
        self.labelPCIP.setText(QCoreApplication.translate("TabTaskEdit", u"PC IP:", None))
        self.labelRobotIP.setText(QCoreApplication.translate("TabTaskEdit", u"Robot IP:", None))
        self.editRobotIP.setText(QCoreApplication.translate("TabTaskEdit", u"192.168.0.29", None))
        self.btnConnect.setText(QCoreApplication.translate("TabTaskEdit", u"\uc5f0\uacb0", None))
        self.labelModbusPort.setText(QCoreApplication.translate("TabTaskEdit", u"Port:", None))
        self.labelConnectionStatus.setText(QCoreApplication.translate("TabTaskEdit", u"\uc0c1\ud0dc:", None))
        self.labelConnectionStatusValue.setText(QCoreApplication.translate("TabTaskEdit", u"\uc5f0\uacb0 \uc548\ub428", None))
        self.labelConnectionStatusValue.setStyleSheet(QCoreApplication.translate("TabTaskEdit", u"color: red;", None))
        self.groupJointPosition.setTitle(QCoreApplication.translate("TabTaskEdit", u"Joint Position (deg)", None))
        self.labelJ1.setText(QCoreApplication.translate("TabTaskEdit", u"J1:", None))
        self.labelJ2.setText(QCoreApplication.translate("TabTaskEdit", u"J2:", None))
        self.labelJ3.setText(QCoreApplication.translate("TabTaskEdit", u"J3:", None))
        self.labelJ4.setText(QCoreApplication.translate("TabTaskEdit", u"J4:", None))
        self.labelJ5.setText(QCoreApplication.translate("TabTaskEdit", u"J5:", None))
        self.labelJ6.setText(QCoreApplication.translate("TabTaskEdit", u"J6:", None))
        self.groupTCPPosition.setTitle(QCoreApplication.translate("TabTaskEdit", u"TCP Position", None))
        self.labelX.setText(QCoreApplication.translate("TabTaskEdit", u"X:", None))
        self.labelUnitX.setText(QCoreApplication.translate("TabTaskEdit", u"mm", None))
        self.labelRx.setText(QCoreApplication.translate("TabTaskEdit", u"Rx:", None))
        self.labelUnitRx.setText(QCoreApplication.translate("TabTaskEdit", u"\u00b0", None))
        self.labelY.setText(QCoreApplication.translate("TabTaskEdit", u"Y:", None))
        self.labelUnitY.setText(QCoreApplication.translate("TabTaskEdit", u"mm", None))
        self.labelRy.setText(QCoreApplication.translate("TabTaskEdit", u"Ry:", None))
        self.labelUnitRy.setText(QCoreApplication.translate("TabTaskEdit", u"\u00b0", None))
        self.labelZ.setText(QCoreApplication.translate("TabTaskEdit", u"Z:", None))
        self.labelUnitZ.setText(QCoreApplication.translate("TabTaskEdit", u"mm", None))
        self.labelRz.setText(QCoreApplication.translate("TabTaskEdit", u"Rz:", None))
        self.labelUnitRz.setText(QCoreApplication.translate("TabTaskEdit", u"\u00b0", None))
        self.btnGoHome.setText(QCoreApplication.translate("TabTaskEdit", u"HOME \uc774\ub3d9", None))
        self.btnSetHome.setText(QCoreApplication.translate("TabTaskEdit", u"\ud604\uc7ac\uc704\uce58\u2192HOME", None))
        self.groupSavedPoses.setTitle(QCoreApplication.translate("TabTaskEdit", u"Saved Poses", None))
        self.btnSavePose.setText(QCoreApplication.translate("TabTaskEdit", u"\ud604\uc7ac\uc704\uce58 \uc800\uc7a5", None))
        self.btnDeletePose.setText(QCoreApplication.translate("TabTaskEdit", u"\uc0ad\uc81c", None))
        self.btnMoveToPose.setText(QCoreApplication.translate("TabTaskEdit", u"\uc120\ud0dd\uc704\uce58 \uc774\ub3d9", None))
        self.btnMoveToPose.setStyleSheet(QCoreApplication.translate("TabTaskEdit", u"background-color: #4CAF50; color: white;", None))
        self.btnApproachPose.setText(QCoreApplication.translate("TabTaskEdit", u"\uc5b4\ud504\ub85c\uce58", None))
        self.btnApproachPose.setStyleSheet(QCoreApplication.translate("TabTaskEdit", u"background-color: #2196F3; color: white;", None))
        pass
    # retranslateUi

