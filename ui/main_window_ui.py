# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'main_window.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1400, 900)
        self.actionNew = QAction(MainWindow)
        self.actionNew.setObjectName(u"actionNew")
        self.actionOpen = QAction(MainWindow)
        self.actionOpen.setObjectName(u"actionOpen")
        self.actionSave = QAction(MainWindow)
        self.actionSave.setObjectName(u"actionSave")
        self.actionSaveAs = QAction(MainWindow)
        self.actionSaveAs.setObjectName(u"actionSaveAs")
        self.actionExit = QAction(MainWindow)
        self.actionExit.setObjectName(u"actionExit")
        self.actionConnect = QAction(MainWindow)
        self.actionConnect.setObjectName(u"actionConnect")
        self.actionDisconnect = QAction(MainWindow)
        self.actionDisconnect.setObjectName(u"actionDisconnect")
        self.actionGoHome = QAction(MainWindow)
        self.actionGoHome.setObjectName(u"actionGoHome")
        self.actionStartCamera = QAction(MainWindow)
        self.actionStartCamera.setObjectName(u"actionStartCamera")
        self.actionStopCamera = QAction(MainWindow)
        self.actionStopCamera.setObjectName(u"actionStopCamera")
        self.actionEmergencyStop = QAction(MainWindow)
        self.actionEmergencyStop.setObjectName(u"actionEmergencyStop")
        self.actionAbout = QAction(MainWindow)
        self.actionAbout.setObjectName(u"actionAbout")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tabMonitor = QWidget()
        self.tabMonitor.setObjectName(u"tabMonitor")
        self.verticalLayoutMonitor = QVBoxLayout(self.tabMonitor)
        self.verticalLayoutMonitor.setObjectName(u"verticalLayoutMonitor")
        self.groupExecutionControl = QGroupBox(self.tabMonitor)
        self.groupExecutionControl.setObjectName(u"groupExecutionControl")
        self.horizontalLayoutExecControl = QHBoxLayout(self.groupExecutionControl)
        self.horizontalLayoutExecControl.setObjectName(u"horizontalLayoutExecControl")
        self.btnRun = QPushButton(self.groupExecutionControl)
        self.btnRun.setObjectName(u"btnRun")
        self.btnRun.setMinimumSize(QSize(100, 40))

        self.horizontalLayoutExecControl.addWidget(self.btnRun)

        self.btnPause = QPushButton(self.groupExecutionControl)
        self.btnPause.setObjectName(u"btnPause")
        self.btnPause.setMinimumSize(QSize(100, 40))

        self.horizontalLayoutExecControl.addWidget(self.btnPause)

        self.btnStop = QPushButton(self.groupExecutionControl)
        self.btnStop.setObjectName(u"btnStop")
        self.btnStop.setMinimumSize(QSize(100, 40))

        self.horizontalLayoutExecControl.addWidget(self.btnStop)

        self.btnStep = QPushButton(self.groupExecutionControl)
        self.btnStep.setObjectName(u"btnStep")
        self.btnStep.setMinimumSize(QSize(100, 40))

        self.horizontalLayoutExecControl.addWidget(self.btnStep)

        self.horizontalSpacerExec = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayoutExecControl.addItem(self.horizontalSpacerExec)

        self.btnEmergencyStop = QPushButton(self.groupExecutionControl)
        self.btnEmergencyStop.setObjectName(u"btnEmergencyStop")
        self.btnEmergencyStop.setMinimumSize(QSize(120, 40))

        self.horizontalLayoutExecControl.addWidget(self.btnEmergencyStop)


        self.verticalLayoutMonitor.addWidget(self.groupExecutionControl)

        self.groupCurrentStatus = QGroupBox(self.tabMonitor)
        self.groupCurrentStatus.setObjectName(u"groupCurrentStatus")
        self.verticalLayoutStatus = QVBoxLayout(self.groupCurrentStatus)
        self.verticalLayoutStatus.setObjectName(u"verticalLayoutStatus")
        self.horizontalLayoutCurrentTask = QHBoxLayout()
        self.horizontalLayoutCurrentTask.setObjectName(u"horizontalLayoutCurrentTask")
        self.labelCurrentTask = QLabel(self.groupCurrentStatus)
        self.labelCurrentTask.setObjectName(u"labelCurrentTask")

        self.horizontalLayoutCurrentTask.addWidget(self.labelCurrentTask)

        self.labelCurrentTaskValue = QLabel(self.groupCurrentStatus)
        self.labelCurrentTaskValue.setObjectName(u"labelCurrentTaskValue")
        font = QFont()
        font.setBold(True)
        font.setWeight(75)
        self.labelCurrentTaskValue.setFont(font)

        self.horizontalLayoutCurrentTask.addWidget(self.labelCurrentTaskValue)

        self.horizontalSpacerTask = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayoutCurrentTask.addItem(self.horizontalSpacerTask)

        self.labelExecutionState = QLabel(self.groupCurrentStatus)
        self.labelExecutionState.setObjectName(u"labelExecutionState")

        self.horizontalLayoutCurrentTask.addWidget(self.labelExecutionState)


        self.verticalLayoutStatus.addLayout(self.horizontalLayoutCurrentTask)

        self.progressExecution = QProgressBar(self.groupCurrentStatus)
        self.progressExecution.setObjectName(u"progressExecution")
        self.progressExecution.setValue(0)

        self.verticalLayoutStatus.addWidget(self.progressExecution)


        self.verticalLayoutMonitor.addWidget(self.groupCurrentStatus)

        self.horizontalLayoutLogModbus = QHBoxLayout()
        self.horizontalLayoutLogModbus.setObjectName(u"horizontalLayoutLogModbus")
        self.groupExecutionLog = QGroupBox(self.tabMonitor)
        self.groupExecutionLog.setObjectName(u"groupExecutionLog")
        self.verticalLayoutLog = QVBoxLayout(self.groupExecutionLog)
        self.verticalLayoutLog.setObjectName(u"verticalLayoutLog")
        self.textExecutionLog = QTextEdit(self.groupExecutionLog)
        self.textExecutionLog.setObjectName(u"textExecutionLog")
        self.textExecutionLog.setReadOnly(True)

        self.verticalLayoutLog.addWidget(self.textExecutionLog)

        self.horizontalLayoutLogButtons = QHBoxLayout()
        self.horizontalLayoutLogButtons.setObjectName(u"horizontalLayoutLogButtons")
        self.btnClearLog = QPushButton(self.groupExecutionLog)
        self.btnClearLog.setObjectName(u"btnClearLog")

        self.horizontalLayoutLogButtons.addWidget(self.btnClearLog)

        self.btnSaveLog = QPushButton(self.groupExecutionLog)
        self.btnSaveLog.setObjectName(u"btnSaveLog")

        self.horizontalLayoutLogButtons.addWidget(self.btnSaveLog)

        self.horizontalSpacerLog = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayoutLogButtons.addItem(self.horizontalSpacerLog)


        self.verticalLayoutLog.addLayout(self.horizontalLayoutLogButtons)


        self.horizontalLayoutLogModbus.addWidget(self.groupExecutionLog)

        self.groupModbusMonitor = QGroupBox(self.tabMonitor)
        self.groupModbusMonitor.setObjectName(u"groupModbusMonitor")
        self.groupModbusMonitor.setMinimumSize(QSize(400, 0))
        self.verticalLayoutModbus = QVBoxLayout(self.groupModbusMonitor)
        self.verticalLayoutModbus.setObjectName(u"verticalLayoutModbus")
        self.tableModbusRegisters = QTableWidget(self.groupModbusMonitor)
        if (self.tableModbusRegisters.columnCount() < 7):
            self.tableModbusRegisters.setColumnCount(7)
        __qtablewidgetitem = QTableWidgetItem()
        self.tableModbusRegisters.setHorizontalHeaderItem(0, __qtablewidgetitem)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.tableModbusRegisters.setHorizontalHeaderItem(1, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        self.tableModbusRegisters.setHorizontalHeaderItem(2, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        self.tableModbusRegisters.setHorizontalHeaderItem(3, __qtablewidgetitem3)
        __qtablewidgetitem4 = QTableWidgetItem()
        self.tableModbusRegisters.setHorizontalHeaderItem(4, __qtablewidgetitem4)
        __qtablewidgetitem5 = QTableWidgetItem()
        self.tableModbusRegisters.setHorizontalHeaderItem(5, __qtablewidgetitem5)
        __qtablewidgetitem6 = QTableWidgetItem()
        self.tableModbusRegisters.setHorizontalHeaderItem(6, __qtablewidgetitem6)
        if (self.tableModbusRegisters.rowCount() < 4):
            self.tableModbusRegisters.setRowCount(4)
        __qtablewidgetitem7 = QTableWidgetItem()
        self.tableModbusRegisters.setVerticalHeaderItem(0, __qtablewidgetitem7)
        __qtablewidgetitem8 = QTableWidgetItem()
        self.tableModbusRegisters.setVerticalHeaderItem(1, __qtablewidgetitem8)
        __qtablewidgetitem9 = QTableWidgetItem()
        self.tableModbusRegisters.setVerticalHeaderItem(2, __qtablewidgetitem9)
        __qtablewidgetitem10 = QTableWidgetItem()
        self.tableModbusRegisters.setVerticalHeaderItem(3, __qtablewidgetitem10)
        self.tableModbusRegisters.setObjectName(u"tableModbusRegisters")
        self.tableModbusRegisters.setRowCount(4)
        self.tableModbusRegisters.setColumnCount(7)

        self.verticalLayoutModbus.addWidget(self.tableModbusRegisters)

        self.horizontalLayoutModbusButtons = QHBoxLayout()
        self.horizontalLayoutModbusButtons.setObjectName(u"horizontalLayoutModbusButtons")
        self.btnRefreshModbus = QPushButton(self.groupModbusMonitor)
        self.btnRefreshModbus.setObjectName(u"btnRefreshModbus")

        self.horizontalLayoutModbusButtons.addWidget(self.btnRefreshModbus)

        self.checkAutoRefresh = QCheckBox(self.groupModbusMonitor)
        self.checkAutoRefresh.setObjectName(u"checkAutoRefresh")
        self.checkAutoRefresh.setChecked(True)

        self.horizontalLayoutModbusButtons.addWidget(self.checkAutoRefresh)

        self.horizontalSpacerModbus = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayoutModbusButtons.addItem(self.horizontalSpacerModbus)


        self.verticalLayoutModbus.addLayout(self.horizontalLayoutModbusButtons)


        self.horizontalLayoutLogModbus.addWidget(self.groupModbusMonitor)


        self.verticalLayoutMonitor.addLayout(self.horizontalLayoutLogModbus)

        self.tabWidget.addTab(self.tabMonitor, "")
        self.tabTest = QWidget()
        self.tabTest.setObjectName(u"tabTest")
        self.horizontalLayoutTest = QHBoxLayout(self.tabTest)
        self.horizontalLayoutTest.setObjectName(u"horizontalLayoutTest")
        self.groupTestSettings = QGroupBox(self.tabTest)
        self.groupTestSettings.setObjectName(u"groupTestSettings")
        self.groupTestSettings.setMaximumSize(QSize(300, 16777215))
        self.verticalLayoutTestSettings = QVBoxLayout(self.groupTestSettings)
        self.verticalLayoutTestSettings.setObjectName(u"verticalLayoutTestSettings")
        self.groupTestType = QGroupBox(self.groupTestSettings)
        self.groupTestType.setObjectName(u"groupTestType")
        self.verticalLayoutTestType = QVBoxLayout(self.groupTestType)
        self.verticalLayoutTestType.setObjectName(u"verticalLayoutTestType")
        self.radioSingleDetect = QRadioButton(self.groupTestType)
        self.radioSingleDetect.setObjectName(u"radioSingleDetect")
        self.radioSingleDetect.setChecked(True)

        self.verticalLayoutTestType.addWidget(self.radioSingleDetect)

        self.radioRepeatTest = QRadioButton(self.groupTestType)
        self.radioRepeatTest.setObjectName(u"radioRepeatTest")

        self.verticalLayoutTestType.addWidget(self.radioRepeatTest)

        self.radioPoseTracking = QRadioButton(self.groupTestType)
        self.radioPoseTracking.setObjectName(u"radioPoseTracking")

        self.verticalLayoutTestType.addWidget(self.radioPoseTracking)


        self.verticalLayoutTestSettings.addWidget(self.groupTestType)

        self.groupRepeatSettings = QGroupBox(self.groupTestSettings)
        self.groupRepeatSettings.setObjectName(u"groupRepeatSettings")
        self.formLayoutRepeat = QFormLayout(self.groupRepeatSettings)
        self.formLayoutRepeat.setObjectName(u"formLayoutRepeat")
        self.labelRepeatCount = QLabel(self.groupRepeatSettings)
        self.labelRepeatCount.setObjectName(u"labelRepeatCount")

        self.formLayoutRepeat.setWidget(0, QFormLayout.LabelRole, self.labelRepeatCount)

        self.spinRepeatCount = QSpinBox(self.groupRepeatSettings)
        self.spinRepeatCount.setObjectName(u"spinRepeatCount")
        self.spinRepeatCount.setMinimum(1)
        self.spinRepeatCount.setMaximum(1000)
        self.spinRepeatCount.setValue(50)

        self.formLayoutRepeat.setWidget(0, QFormLayout.FieldRole, self.spinRepeatCount)

        self.labelInterval = QLabel(self.groupRepeatSettings)
        self.labelInterval.setObjectName(u"labelInterval")

        self.formLayoutRepeat.setWidget(1, QFormLayout.LabelRole, self.labelInterval)

        self.spinInterval = QSpinBox(self.groupRepeatSettings)
        self.spinInterval.setObjectName(u"spinInterval")
        self.spinInterval.setMinimum(100)
        self.spinInterval.setMaximum(10000)
        self.spinInterval.setValue(500)

        self.formLayoutRepeat.setWidget(1, QFormLayout.FieldRole, self.spinInterval)


        self.verticalLayoutTestSettings.addWidget(self.groupRepeatSettings)

        self.horizontalLayoutTestButtons = QHBoxLayout()
        self.horizontalLayoutTestButtons.setObjectName(u"horizontalLayoutTestButtons")
        self.btnStartTest = QPushButton(self.groupTestSettings)
        self.btnStartTest.setObjectName(u"btnStartTest")

        self.horizontalLayoutTestButtons.addWidget(self.btnStartTest)

        self.btnStopTest = QPushButton(self.groupTestSettings)
        self.btnStopTest.setObjectName(u"btnStopTest")

        self.horizontalLayoutTestButtons.addWidget(self.btnStopTest)


        self.verticalLayoutTestSettings.addLayout(self.horizontalLayoutTestButtons)

        self.progressTest = QProgressBar(self.groupTestSettings)
        self.progressTest.setObjectName(u"progressTest")
        self.progressTest.setValue(0)

        self.verticalLayoutTestSettings.addWidget(self.progressTest)

        self.verticalSpacerTest = QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayoutTestSettings.addItem(self.verticalSpacerTest)


        self.horizontalLayoutTest.addWidget(self.groupTestSettings)

        self.groupTestGraph = QGroupBox(self.tabTest)
        self.groupTestGraph.setObjectName(u"groupTestGraph")
        self.verticalLayoutGraph = QVBoxLayout(self.groupTestGraph)
        self.verticalLayoutGraph.setObjectName(u"verticalLayoutGraph")
        self.widgetGraph = QWidget(self.groupTestGraph)
        self.widgetGraph.setObjectName(u"widgetGraph")
        self.widgetGraph.setMinimumSize(QSize(400, 400))

        self.verticalLayoutGraph.addWidget(self.widgetGraph)

        self.horizontalLayoutGraphOptions = QHBoxLayout()
        self.horizontalLayoutGraphOptions.setObjectName(u"horizontalLayoutGraphOptions")
        self.checkShowX = QCheckBox(self.groupTestGraph)
        self.checkShowX.setObjectName(u"checkShowX")
        self.checkShowX.setChecked(True)

        self.horizontalLayoutGraphOptions.addWidget(self.checkShowX)

        self.checkShowY = QCheckBox(self.groupTestGraph)
        self.checkShowY.setObjectName(u"checkShowY")
        self.checkShowY.setChecked(True)

        self.horizontalLayoutGraphOptions.addWidget(self.checkShowY)

        self.checkShowZ = QCheckBox(self.groupTestGraph)
        self.checkShowZ.setObjectName(u"checkShowZ")
        self.checkShowZ.setChecked(True)

        self.horizontalLayoutGraphOptions.addWidget(self.checkShowZ)

        self.checkShowMean = QCheckBox(self.groupTestGraph)
        self.checkShowMean.setObjectName(u"checkShowMean")
        self.checkShowMean.setChecked(True)

        self.horizontalLayoutGraphOptions.addWidget(self.checkShowMean)

        self.horizontalSpacerGraph = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayoutGraphOptions.addItem(self.horizontalSpacerGraph)


        self.verticalLayoutGraph.addLayout(self.horizontalLayoutGraphOptions)


        self.horizontalLayoutTest.addWidget(self.groupTestGraph)

        self.groupStatistics = QGroupBox(self.tabTest)
        self.groupStatistics.setObjectName(u"groupStatistics")
        self.groupStatistics.setMinimumSize(QSize(250, 0))
        self.verticalLayoutStats = QVBoxLayout(self.groupStatistics)
        self.verticalLayoutStats.setObjectName(u"verticalLayoutStats")
        self.tableStatistics = QTableWidget(self.groupStatistics)
        if (self.tableStatistics.columnCount() < 2):
            self.tableStatistics.setColumnCount(2)
        __qtablewidgetitem11 = QTableWidgetItem()
        self.tableStatistics.setHorizontalHeaderItem(0, __qtablewidgetitem11)
        __qtablewidgetitem12 = QTableWidgetItem()
        self.tableStatistics.setHorizontalHeaderItem(1, __qtablewidgetitem12)
        if (self.tableStatistics.rowCount() < 6):
            self.tableStatistics.setRowCount(6)
        __qtablewidgetitem13 = QTableWidgetItem()
        self.tableStatistics.setVerticalHeaderItem(0, __qtablewidgetitem13)
        __qtablewidgetitem14 = QTableWidgetItem()
        self.tableStatistics.setVerticalHeaderItem(1, __qtablewidgetitem14)
        __qtablewidgetitem15 = QTableWidgetItem()
        self.tableStatistics.setVerticalHeaderItem(2, __qtablewidgetitem15)
        __qtablewidgetitem16 = QTableWidgetItem()
        self.tableStatistics.setVerticalHeaderItem(3, __qtablewidgetitem16)
        __qtablewidgetitem17 = QTableWidgetItem()
        self.tableStatistics.setVerticalHeaderItem(4, __qtablewidgetitem17)
        __qtablewidgetitem18 = QTableWidgetItem()
        self.tableStatistics.setVerticalHeaderItem(5, __qtablewidgetitem18)
        self.tableStatistics.setObjectName(u"tableStatistics")
        self.tableStatistics.setRowCount(6)
        self.tableStatistics.setColumnCount(2)

        self.verticalLayoutStats.addWidget(self.tableStatistics)

        self.btnExportCSV = QPushButton(self.groupStatistics)
        self.btnExportCSV.setObjectName(u"btnExportCSV")

        self.verticalLayoutStats.addWidget(self.btnExportCSV)


        self.horizontalLayoutTest.addWidget(self.groupStatistics)

        self.tabWidget.addTab(self.tabTest, "")
        self.tabSettings = QWidget()
        self.tabSettings.setObjectName(u"tabSettings")
        self.horizontalLayoutSettings = QHBoxLayout(self.tabSettings)
        self.horizontalLayoutSettings.setObjectName(u"horizontalLayoutSettings")
        self.groupConnectionSettings = QGroupBox(self.tabSettings)
        self.groupConnectionSettings.setObjectName(u"groupConnectionSettings")
        self.verticalLayoutConnection = QVBoxLayout(self.groupConnectionSettings)
        self.verticalLayoutConnection.setObjectName(u"verticalLayoutConnection")
        self.groupModbusSettings = QGroupBox(self.groupConnectionSettings)
        self.groupModbusSettings.setObjectName(u"groupModbusSettings")
        self.verticalLayoutModbusInner = QVBoxLayout(self.groupModbusSettings)
        self.verticalLayoutModbusInner.setObjectName(u"verticalLayoutModbusInner")
        self.formLayoutModbusSettings = QFormLayout()
        self.formLayoutModbusSettings.setObjectName(u"formLayoutModbusSettings")
        self.labelSettingsPCIP = QLabel(self.groupModbusSettings)
        self.labelSettingsPCIP.setObjectName(u"labelSettingsPCIP")

        self.formLayoutModbusSettings.setWidget(0, QFormLayout.LabelRole, self.labelSettingsPCIP)

        self.labelPCIPValue = QLabel(self.groupModbusSettings)
        self.labelPCIPValue.setObjectName(u"labelPCIPValue")

        self.formLayoutModbusSettings.setWidget(0, QFormLayout.FieldRole, self.labelPCIPValue)

        self.labelSettingsRobotIP = QLabel(self.groupModbusSettings)
        self.labelSettingsRobotIP.setObjectName(u"labelSettingsRobotIP")

        self.formLayoutModbusSettings.setWidget(1, QFormLayout.LabelRole, self.labelSettingsRobotIP)

        self.editSettingsRobotIP = QLineEdit(self.groupModbusSettings)
        self.editSettingsRobotIP.setObjectName(u"editSettingsRobotIP")

        self.formLayoutModbusSettings.setWidget(1, QFormLayout.FieldRole, self.editSettingsRobotIP)

        self.labelSettingsPort = QLabel(self.groupModbusSettings)
        self.labelSettingsPort.setObjectName(u"labelSettingsPort")

        self.formLayoutModbusSettings.setWidget(2, QFormLayout.LabelRole, self.labelSettingsPort)

        self.spinSettingsPort = QSpinBox(self.groupModbusSettings)
        self.spinSettingsPort.setObjectName(u"spinSettingsPort")
        self.spinSettingsPort.setMinimum(1)
        self.spinSettingsPort.setMaximum(65535)
        self.spinSettingsPort.setValue(1502)

        self.formLayoutModbusSettings.setWidget(2, QFormLayout.FieldRole, self.spinSettingsPort)

        self.labelTimeout = QLabel(self.groupModbusSettings)
        self.labelTimeout.setObjectName(u"labelTimeout")

        self.formLayoutModbusSettings.setWidget(3, QFormLayout.LabelRole, self.labelTimeout)

        self.spinTimeout = QDoubleSpinBox(self.groupModbusSettings)
        self.spinTimeout.setObjectName(u"spinTimeout")
        self.spinTimeout.setMinimum(0.100000000000000)
        self.spinTimeout.setMaximum(10.000000000000000)
        self.spinTimeout.setSingleStep(0.100000000000000)
        self.spinTimeout.setValue(1.000000000000000)

        self.formLayoutModbusSettings.setWidget(3, QFormLayout.FieldRole, self.spinTimeout)


        self.verticalLayoutModbusInner.addLayout(self.formLayoutModbusSettings)

        self.horizontalLayoutConnectBtn = QHBoxLayout()
        self.horizontalLayoutConnectBtn.setObjectName(u"horizontalLayoutConnectBtn")
        self.btnTestConnection = QPushButton(self.groupModbusSettings)
        self.btnTestConnection.setObjectName(u"btnTestConnection")
        self.btnTestConnection.setMinimumSize(QSize(0, 35))

        self.horizontalLayoutConnectBtn.addWidget(self.btnTestConnection)

        self.labelSettingsConnStatus = QLabel(self.groupModbusSettings)
        self.labelSettingsConnStatus.setObjectName(u"labelSettingsConnStatus")

        self.horizontalLayoutConnectBtn.addWidget(self.labelSettingsConnStatus)


        self.verticalLayoutModbusInner.addLayout(self.horizontalLayoutConnectBtn)


        self.verticalLayoutConnection.addWidget(self.groupModbusSettings)

        self.checkSimulationMode = QCheckBox(self.groupConnectionSettings)
        self.checkSimulationMode.setObjectName(u"checkSimulationMode")

        self.verticalLayoutConnection.addWidget(self.checkSimulationMode)

        self.groupDebugLog = QGroupBox(self.groupConnectionSettings)
        self.groupDebugLog.setObjectName(u"groupDebugLog")
        self.verticalLayoutDebugLog = QVBoxLayout(self.groupDebugLog)
        self.verticalLayoutDebugLog.setObjectName(u"verticalLayoutDebugLog")
        self.textDebugLog = QTextEdit(self.groupDebugLog)
        self.textDebugLog.setObjectName(u"textDebugLog")
        self.textDebugLog.setReadOnly(True)

        self.verticalLayoutDebugLog.addWidget(self.textDebugLog)

        self.btnClearDebugLog = QPushButton(self.groupDebugLog)
        self.btnClearDebugLog.setObjectName(u"btnClearDebugLog")

        self.verticalLayoutDebugLog.addWidget(self.btnClearDebugLog)


        self.verticalLayoutConnection.addWidget(self.groupDebugLog)


        self.horizontalLayoutSettings.addWidget(self.groupConnectionSettings)

        self.groupCameraSettings = QGroupBox(self.tabSettings)
        self.groupCameraSettings.setObjectName(u"groupCameraSettings")
        self.verticalLayoutCameraSettings = QVBoxLayout(self.groupCameraSettings)
        self.verticalLayoutCameraSettings.setObjectName(u"verticalLayoutCameraSettings")
        self.groupResolution = QGroupBox(self.groupCameraSettings)
        self.groupResolution.setObjectName(u"groupResolution")
        self.verticalLayoutResolution = QVBoxLayout(self.groupResolution)
        self.verticalLayoutResolution.setObjectName(u"verticalLayoutResolution")
        self.radioRes1080 = QRadioButton(self.groupResolution)
        self.radioRes1080.setObjectName(u"radioRes1080")
        self.radioRes1080.setChecked(True)

        self.verticalLayoutResolution.addWidget(self.radioRes1080)

        self.radioRes720 = QRadioButton(self.groupResolution)
        self.radioRes720.setObjectName(u"radioRes720")

        self.verticalLayoutResolution.addWidget(self.radioRes720)

        self.radioRes480 = QRadioButton(self.groupResolution)
        self.radioRes480.setObjectName(u"radioRes480")

        self.verticalLayoutResolution.addWidget(self.radioRes480)


        self.verticalLayoutCameraSettings.addWidget(self.groupResolution)

        self.formLayoutCameraSettings = QFormLayout()
        self.formLayoutCameraSettings.setObjectName(u"formLayoutCameraSettings")
        self.labelFPS = QLabel(self.groupCameraSettings)
        self.labelFPS.setObjectName(u"labelFPS")

        self.formLayoutCameraSettings.setWidget(0, QFormLayout.LabelRole, self.labelFPS)

        self.spinFPS = QSpinBox(self.groupCameraSettings)
        self.spinFPS.setObjectName(u"spinFPS")
        self.spinFPS.setMinimum(5)
        self.spinFPS.setMaximum(60)
        self.spinFPS.setValue(15)

        self.formLayoutCameraSettings.setWidget(0, QFormLayout.FieldRole, self.spinFPS)


        self.verticalLayoutCameraSettings.addLayout(self.formLayoutCameraSettings)

        self.groupCalibration = QGroupBox(self.groupCameraSettings)
        self.groupCalibration.setObjectName(u"groupCalibration")
        self.verticalLayoutCalibration = QVBoxLayout(self.groupCalibration)
        self.verticalLayoutCalibration.setObjectName(u"verticalLayoutCalibration")
        self.labelCalibStatus = QLabel(self.groupCalibration)
        self.labelCalibStatus.setObjectName(u"labelCalibStatus")

        self.verticalLayoutCalibration.addWidget(self.labelCalibStatus)

        self.horizontalLayoutCalibButtons = QHBoxLayout()
        self.horizontalLayoutCalibButtons.setObjectName(u"horizontalLayoutCalibButtons")
        self.btnLoadCalib = QPushButton(self.groupCalibration)
        self.btnLoadCalib.setObjectName(u"btnLoadCalib")

        self.horizontalLayoutCalibButtons.addWidget(self.btnLoadCalib)

        self.btnNewCalib = QPushButton(self.groupCalibration)
        self.btnNewCalib.setObjectName(u"btnNewCalib")

        self.horizontalLayoutCalibButtons.addWidget(self.btnNewCalib)


        self.verticalLayoutCalibration.addLayout(self.horizontalLayoutCalibButtons)


        self.verticalLayoutCameraSettings.addWidget(self.groupCalibration)

        self.verticalSpacerCamera = QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayoutCameraSettings.addItem(self.verticalSpacerCamera)


        self.horizontalLayoutSettings.addWidget(self.groupCameraSettings)

        self.groupOffsetSettings = QGroupBox(self.tabSettings)
        self.groupOffsetSettings.setObjectName(u"groupOffsetSettings")
        self.verticalLayoutOffset = QVBoxLayout(self.groupOffsetSettings)
        self.verticalLayoutOffset.setObjectName(u"verticalLayoutOffset")
        self.groupApproachOffset = QGroupBox(self.groupOffsetSettings)
        self.groupApproachOffset.setObjectName(u"groupApproachOffset")
        self.formLayoutApproachOffset = QFormLayout(self.groupApproachOffset)
        self.formLayoutApproachOffset.setObjectName(u"formLayoutApproachOffset")
        self.labelOffsetX = QLabel(self.groupApproachOffset)
        self.labelOffsetX.setObjectName(u"labelOffsetX")

        self.formLayoutApproachOffset.setWidget(0, QFormLayout.LabelRole, self.labelOffsetX)

        self.spinOffsetX = QDoubleSpinBox(self.groupApproachOffset)
        self.spinOffsetX.setObjectName(u"spinOffsetX")
        self.spinOffsetX.setMinimum(-1000.000000000000000)
        self.spinOffsetX.setMaximum(1000.000000000000000)

        self.formLayoutApproachOffset.setWidget(0, QFormLayout.FieldRole, self.spinOffsetX)

        self.labelOffsetY = QLabel(self.groupApproachOffset)
        self.labelOffsetY.setObjectName(u"labelOffsetY")

        self.formLayoutApproachOffset.setWidget(1, QFormLayout.LabelRole, self.labelOffsetY)

        self.spinOffsetY = QDoubleSpinBox(self.groupApproachOffset)
        self.spinOffsetY.setObjectName(u"spinOffsetY")
        self.spinOffsetY.setMinimum(-1000.000000000000000)
        self.spinOffsetY.setMaximum(1000.000000000000000)

        self.formLayoutApproachOffset.setWidget(1, QFormLayout.FieldRole, self.spinOffsetY)

        self.labelOffsetZ = QLabel(self.groupApproachOffset)
        self.labelOffsetZ.setObjectName(u"labelOffsetZ")

        self.formLayoutApproachOffset.setWidget(2, QFormLayout.LabelRole, self.labelOffsetZ)

        self.spinOffsetZ = QDoubleSpinBox(self.groupApproachOffset)
        self.spinOffsetZ.setObjectName(u"spinOffsetZ")
        self.spinOffsetZ.setMinimum(-1000.000000000000000)
        self.spinOffsetZ.setMaximum(1000.000000000000000)
        self.spinOffsetZ.setValue(200.000000000000000)

        self.formLayoutApproachOffset.setWidget(2, QFormLayout.FieldRole, self.spinOffsetZ)


        self.verticalLayoutOffset.addWidget(self.groupApproachOffset)

        self.groupArUcoSettings = QGroupBox(self.groupOffsetSettings)
        self.groupArUcoSettings.setObjectName(u"groupArUcoSettings")
        self.formLayoutArUco = QFormLayout(self.groupArUcoSettings)
        self.formLayoutArUco.setObjectName(u"formLayoutArUco")
        self.labelMarkerSize = QLabel(self.groupArUcoSettings)
        self.labelMarkerSize.setObjectName(u"labelMarkerSize")

        self.formLayoutArUco.setWidget(0, QFormLayout.LabelRole, self.labelMarkerSize)

        self.spinMarkerSize = QDoubleSpinBox(self.groupArUcoSettings)
        self.spinMarkerSize.setObjectName(u"spinMarkerSize")
        self.spinMarkerSize.setMinimum(1.000000000000000)
        self.spinMarkerSize.setMaximum(500.000000000000000)
        self.spinMarkerSize.setValue(18.199999999999999)

        self.formLayoutArUco.setWidget(0, QFormLayout.FieldRole, self.spinMarkerSize)

        self.labelDictionary = QLabel(self.groupArUcoSettings)
        self.labelDictionary.setObjectName(u"labelDictionary")

        self.formLayoutArUco.setWidget(1, QFormLayout.LabelRole, self.labelDictionary)

        self.comboDictionary = QComboBox(self.groupArUcoSettings)
        self.comboDictionary.addItem("")
        self.comboDictionary.addItem("")
        self.comboDictionary.addItem("")
        self.comboDictionary.addItem("")
        self.comboDictionary.addItem("")
        self.comboDictionary.setObjectName(u"comboDictionary")

        self.formLayoutArUco.setWidget(1, QFormLayout.FieldRole, self.comboDictionary)


        self.verticalLayoutOffset.addWidget(self.groupArUcoSettings)

        self.horizontalLayoutSaveSettings = QHBoxLayout()
        self.horizontalLayoutSaveSettings.setObjectName(u"horizontalLayoutSaveSettings")
        self.btnSaveSettings = QPushButton(self.groupOffsetSettings)
        self.btnSaveSettings.setObjectName(u"btnSaveSettings")

        self.horizontalLayoutSaveSettings.addWidget(self.btnSaveSettings)

        self.btnLoadSettings = QPushButton(self.groupOffsetSettings)
        self.btnLoadSettings.setObjectName(u"btnLoadSettings")

        self.horizontalLayoutSaveSettings.addWidget(self.btnLoadSettings)


        self.verticalLayoutOffset.addLayout(self.horizontalLayoutSaveSettings)

        self.verticalSpacerOffset = QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayoutOffset.addItem(self.verticalSpacerOffset)


        self.horizontalLayoutSettings.addWidget(self.groupOffsetSettings)

        self.tabWidget.addTab(self.tabSettings, "")

        self.verticalLayout.addWidget(self.tabWidget)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1400, 22))
        self.menuFile = QMenu(self.menubar)
        self.menuFile.setObjectName(u"menuFile")
        self.menuRobot = QMenu(self.menubar)
        self.menuRobot.setObjectName(u"menuRobot")
        self.menuVision = QMenu(self.menubar)
        self.menuVision.setObjectName(u"menuVision")
        self.menuHelp = QMenu(self.menubar)
        self.menuHelp.setObjectName(u"menuHelp")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.toolBar = QToolBar(MainWindow)
        self.toolBar.setObjectName(u"toolBar")
        MainWindow.addToolBar(Qt.TopToolBarArea, self.toolBar)

        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuRobot.menuAction())
        self.menubar.addAction(self.menuVision.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())
        self.menuFile.addAction(self.actionNew)
        self.menuFile.addAction(self.actionOpen)
        self.menuFile.addAction(self.actionSave)
        self.menuFile.addAction(self.actionSaveAs)
        self.menuFile.addSeparator()
        self.menuFile.addAction(self.actionExit)
        self.menuRobot.addAction(self.actionConnect)
        self.menuRobot.addAction(self.actionDisconnect)
        self.menuRobot.addSeparator()
        self.menuRobot.addAction(self.actionGoHome)
        self.menuVision.addAction(self.actionStartCamera)
        self.menuVision.addAction(self.actionStopCamera)
        self.menuHelp.addAction(self.actionAbout)
        self.toolBar.addAction(self.actionConnect)
        self.toolBar.addSeparator()
        self.toolBar.addAction(self.actionGoHome)
        self.toolBar.addSeparator()
        self.toolBar.addAction(self.actionEmergencyStop)

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Charging Robot Task Manager", None))
        self.actionNew.setText(QCoreApplication.translate("MainWindow", u"\uc0c8\ub85c \ub9cc\ub4e4\uae30", None))
#if QT_CONFIG(shortcut)
        self.actionNew.setShortcut(QCoreApplication.translate("MainWindow", u"Ctrl+N", None))
#endif // QT_CONFIG(shortcut)
        self.actionOpen.setText(QCoreApplication.translate("MainWindow", u"\uc5f4\uae30", None))
#if QT_CONFIG(shortcut)
        self.actionOpen.setShortcut(QCoreApplication.translate("MainWindow", u"Ctrl+O", None))
#endif // QT_CONFIG(shortcut)
        self.actionSave.setText(QCoreApplication.translate("MainWindow", u"\uc800\uc7a5", None))
#if QT_CONFIG(shortcut)
        self.actionSave.setShortcut(QCoreApplication.translate("MainWindow", u"Ctrl+S", None))
#endif // QT_CONFIG(shortcut)
        self.actionSaveAs.setText(QCoreApplication.translate("MainWindow", u"\ub2e4\ub978 \uc774\ub984\uc73c\ub85c \uc800\uc7a5", None))
#if QT_CONFIG(shortcut)
        self.actionSaveAs.setShortcut(QCoreApplication.translate("MainWindow", u"Ctrl+Shift+S", None))
#endif // QT_CONFIG(shortcut)
        self.actionExit.setText(QCoreApplication.translate("MainWindow", u"\uc885\ub8cc", None))
#if QT_CONFIG(shortcut)
        self.actionExit.setShortcut(QCoreApplication.translate("MainWindow", u"Ctrl+Q", None))
#endif // QT_CONFIG(shortcut)
        self.actionConnect.setText(QCoreApplication.translate("MainWindow", u"\uc5f0\uacb0", None))
        self.actionDisconnect.setText(QCoreApplication.translate("MainWindow", u"\uc5f0\uacb0 \ud574\uc81c", None))
        self.actionGoHome.setText(QCoreApplication.translate("MainWindow", u"HOME", None))
        self.actionStartCamera.setText(QCoreApplication.translate("MainWindow", u"\uce74\uba54\ub77c \uc2dc\uc791", None))
        self.actionStopCamera.setText(QCoreApplication.translate("MainWindow", u"\uce74\uba54\ub77c \uc815\uc9c0", None))
        self.actionEmergencyStop.setText(QCoreApplication.translate("MainWindow", u"\ube44\uc0c1 \uc815\uc9c0", None))
        self.actionAbout.setText(QCoreApplication.translate("MainWindow", u"\uc815\ubcf4", None))
        self.groupExecutionControl.setTitle(QCoreApplication.translate("MainWindow", u"\uc2e4\ud589 \uc81c\uc5b4", None))
        self.btnRun.setText(QCoreApplication.translate("MainWindow", u"\u25b6 \uc2e4\ud589", None))
        self.btnPause.setText(QCoreApplication.translate("MainWindow", u"\u23f8 \uc77c\uc2dc\uc815\uc9c0", None))
        self.btnStop.setText(QCoreApplication.translate("MainWindow", u"\u23f9 \uc815\uc9c0", None))
        self.btnStep.setText(QCoreApplication.translate("MainWindow", u"\u23ed \uc2a4\ud15d", None))
        self.btnEmergencyStop.setText(QCoreApplication.translate("MainWindow", u"\ube44\uc0c1 \uc815\uc9c0", None))
        self.btnEmergencyStop.setStyleSheet(QCoreApplication.translate("MainWindow", u"background-color: #ff4444; color: white; font-weight: bold;", None))
        self.groupCurrentStatus.setTitle(QCoreApplication.translate("MainWindow", u"\ud604\uc7ac \uc0c1\ud0dc", None))
        self.labelCurrentTask.setText(QCoreApplication.translate("MainWindow", u"\ud604\uc7ac Task:", None))
        self.labelCurrentTaskValue.setText(QCoreApplication.translate("MainWindow", u"-", None))
        self.labelExecutionState.setText(QCoreApplication.translate("MainWindow", u"\ub300\uae30 \uc911", None))
        self.labelExecutionState.setStyleSheet(QCoreApplication.translate("MainWindow", u"color: gray; font-weight: bold;", None))
        self.groupExecutionLog.setTitle(QCoreApplication.translate("MainWindow", u"\uc2e4\ud589 \ub85c\uadf8", None))
        self.textExecutionLog.setStyleSheet(QCoreApplication.translate("MainWindow", u"font-family: 'Consolas', 'Monaco', monospace;", None))
        self.btnClearLog.setText(QCoreApplication.translate("MainWindow", u"\ub85c\uadf8 \uc9c0\uc6b0\uae30", None))
        self.btnSaveLog.setText(QCoreApplication.translate("MainWindow", u"\ub85c\uadf8 \uc800\uc7a5", None))
        self.groupModbusMonitor.setTitle(QCoreApplication.translate("MainWindow", u"Modbus \ubaa8\ub2c8\ud130", None))
        ___qtablewidgetitem = self.tableModbusRegisters.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("MainWindow", u"Name", None));
        ___qtablewidgetitem1 = self.tableModbusRegisters.horizontalHeaderItem(1)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("MainWindow", u"X", None));
        ___qtablewidgetitem2 = self.tableModbusRegisters.horizontalHeaderItem(2)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("MainWindow", u"Y", None));
        ___qtablewidgetitem3 = self.tableModbusRegisters.horizontalHeaderItem(3)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("MainWindow", u"Z", None));
        ___qtablewidgetitem4 = self.tableModbusRegisters.horizontalHeaderItem(4)
        ___qtablewidgetitem4.setText(QCoreApplication.translate("MainWindow", u"Rx", None));
        ___qtablewidgetitem5 = self.tableModbusRegisters.horizontalHeaderItem(5)
        ___qtablewidgetitem5.setText(QCoreApplication.translate("MainWindow", u"Ry", None));
        ___qtablewidgetitem6 = self.tableModbusRegisters.horizontalHeaderItem(6)
        ___qtablewidgetitem6.setText(QCoreApplication.translate("MainWindow", u"Rz", None));
        ___qtablewidgetitem7 = self.tableModbusRegisters.verticalHeaderItem(0)
        ___qtablewidgetitem7.setText(QCoreApplication.translate("MainWindow", u"Pose Main (301-306)", None));
        ___qtablewidgetitem8 = self.tableModbusRegisters.verticalHeaderItem(1)
        ___qtablewidgetitem8.setText(QCoreApplication.translate("MainWindow", u"Pose Back (307-312)", None));
        ___qtablewidgetitem9 = self.tableModbusRegisters.verticalHeaderItem(2)
        ___qtablewidgetitem9.setText(QCoreApplication.translate("MainWindow", u"Command (351)", None));
        ___qtablewidgetitem10 = self.tableModbusRegisters.verticalHeaderItem(3)
        ___qtablewidgetitem10.setText(QCoreApplication.translate("MainWindow", u"Response (352)", None));
        self.btnRefreshModbus.setText(QCoreApplication.translate("MainWindow", u"\uc0c8\ub85c\uace0\uce68", None))
        self.checkAutoRefresh.setText(QCoreApplication.translate("MainWindow", u"\uc790\ub3d9 \uac31\uc2e0", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tabMonitor), QCoreApplication.translate("MainWindow", u"\uc2e4\ud589 \ubaa8\ub2c8\ud130", None))
        self.groupTestSettings.setTitle(QCoreApplication.translate("MainWindow", u"\ud14c\uc2a4\ud2b8 \uc124\uc815", None))
        self.groupTestType.setTitle(QCoreApplication.translate("MainWindow", u"\ud14c\uc2a4\ud2b8 \uc720\ud615", None))
        self.radioSingleDetect.setText(QCoreApplication.translate("MainWindow", u"\ub2e8\uc77c \uac10\uc9c0 \ud14c\uc2a4\ud2b8", None))
        self.radioRepeatTest.setText(QCoreApplication.translate("MainWindow", u"\ubc18\ubcf5 \uc815\ubc00\ub3c4 \ud14c\uc2a4\ud2b8", None))
        self.radioPoseTracking.setText(QCoreApplication.translate("MainWindow", u"\ud3ec\uc988 \ucd94\uc801 \ud14c\uc2a4\ud2b8", None))
        self.groupRepeatSettings.setTitle(QCoreApplication.translate("MainWindow", u"\ubc18\ubcf5 \uc124\uc815", None))
        self.labelRepeatCount.setText(QCoreApplication.translate("MainWindow", u"\ubc18\ubcf5 \ud69f\uc218:", None))
        self.labelInterval.setText(QCoreApplication.translate("MainWindow", u"\uac04\uaca9 (ms):", None))
        self.btnStartTest.setText(QCoreApplication.translate("MainWindow", u"\ud14c\uc2a4\ud2b8 \uc2dc\uc791", None))
        self.btnStopTest.setText(QCoreApplication.translate("MainWindow", u"\ud14c\uc2a4\ud2b8 \uc911\uc9c0", None))
        self.groupTestGraph.setTitle(QCoreApplication.translate("MainWindow", u"\uacb0\uacfc \uadf8\ub798\ud504", None))
        self.widgetGraph.setStyleSheet(QCoreApplication.translate("MainWindow", u"background-color: white; border: 1px solid #cccccc;", None))
        self.checkShowX.setText(QCoreApplication.translate("MainWindow", u"X\ucd95", None))
        self.checkShowY.setText(QCoreApplication.translate("MainWindow", u"Y\ucd95", None))
        self.checkShowZ.setText(QCoreApplication.translate("MainWindow", u"Z\ucd95", None))
        self.checkShowMean.setText(QCoreApplication.translate("MainWindow", u"\ud3c9\uade0\uc810 \ud45c\uc2dc", None))
        self.groupStatistics.setTitle(QCoreApplication.translate("MainWindow", u"\ud1b5\uacc4", None))
        ___qtablewidgetitem11 = self.tableStatistics.horizontalHeaderItem(0)
        ___qtablewidgetitem11.setText(QCoreApplication.translate("MainWindow", u"\ud56d\ubaa9", None));
        ___qtablewidgetitem12 = self.tableStatistics.horizontalHeaderItem(1)
        ___qtablewidgetitem12.setText(QCoreApplication.translate("MainWindow", u"\uac12", None));
        ___qtablewidgetitem13 = self.tableStatistics.verticalHeaderItem(0)
        ___qtablewidgetitem13.setText(QCoreApplication.translate("MainWindow", u"\uac10\uc9c0 \ud69f\uc218", None));
        ___qtablewidgetitem14 = self.tableStatistics.verticalHeaderItem(1)
        ___qtablewidgetitem14.setText(QCoreApplication.translate("MainWindow", u"\uc131\uacf5\ub960", None));
        ___qtablewidgetitem15 = self.tableStatistics.verticalHeaderItem(2)
        ___qtablewidgetitem15.setText(QCoreApplication.translate("MainWindow", u"X \ud45c\uc900\ud3b8\ucc28", None));
        ___qtablewidgetitem16 = self.tableStatistics.verticalHeaderItem(3)
        ___qtablewidgetitem16.setText(QCoreApplication.translate("MainWindow", u"Y \ud45c\uc900\ud3b8\ucc28", None));
        ___qtablewidgetitem17 = self.tableStatistics.verticalHeaderItem(4)
        ___qtablewidgetitem17.setText(QCoreApplication.translate("MainWindow", u"Z \ud45c\uc900\ud3b8\ucc28", None));
        ___qtablewidgetitem18 = self.tableStatistics.verticalHeaderItem(5)
        ___qtablewidgetitem18.setText(QCoreApplication.translate("MainWindow", u"\ud3c9\uade0 \uc218\ub834\uc2dc\uac04", None));
        self.btnExportCSV.setText(QCoreApplication.translate("MainWindow", u"CSV \ub0b4\ubcf4\ub0b4\uae30", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tabTest), QCoreApplication.translate("MainWindow", u"\ud14c\uc2a4\ud2b8", None))
        self.groupConnectionSettings.setTitle(QCoreApplication.translate("MainWindow", u"\uc5f0\uacb0 \uc124\uc815", None))
        self.groupModbusSettings.setTitle(QCoreApplication.translate("MainWindow", u"Modbus TCP", None))
        self.labelSettingsPCIP.setText(QCoreApplication.translate("MainWindow", u"PC IP:", None))
        self.labelPCIPValue.setText(QCoreApplication.translate("MainWindow", u"-", None))
        self.labelPCIPValue.setStyleSheet(QCoreApplication.translate("MainWindow", u"color: #0066cc; font-weight: bold;", None))
        self.labelSettingsRobotIP.setText(QCoreApplication.translate("MainWindow", u"Robot IP:", None))
        self.editSettingsRobotIP.setText(QCoreApplication.translate("MainWindow", u"192.168.0.29", None))
        self.labelSettingsPort.setText(QCoreApplication.translate("MainWindow", u"Port:", None))
        self.labelTimeout.setText(QCoreApplication.translate("MainWindow", u"Timeout (s):", None))
        self.btnTestConnection.setText(QCoreApplication.translate("MainWindow", u"\uc5f0\uacb0 \ud14c\uc2a4\ud2b8", None))
        self.labelSettingsConnStatus.setText(QCoreApplication.translate("MainWindow", u"\uc5f0\uacb0 \uc548\ub428", None))
        self.labelSettingsConnStatus.setStyleSheet(QCoreApplication.translate("MainWindow", u"color: red; font-weight: bold;", None))
        self.checkSimulationMode.setText(QCoreApplication.translate("MainWindow", u"\uc2dc\ubbac\ub808\uc774\uc158 \ubaa8\ub4dc (Modbus \uc5c6\uc774 \uc2e4\ud589)", None))
        self.groupDebugLog.setTitle(QCoreApplication.translate("MainWindow", u"Debug Log", None))
        self.textDebugLog.setStyleSheet(QCoreApplication.translate("MainWindow", u"font-family: 'Consolas', 'Monaco', monospace; font-size: 10px; background-color: #1e1e1e; color: #00ff00;", None))
        self.btnClearDebugLog.setText(QCoreApplication.translate("MainWindow", u"\ub85c\uadf8 \uc9c0\uc6b0\uae30", None))
        self.groupCameraSettings.setTitle(QCoreApplication.translate("MainWindow", u"\uce74\uba54\ub77c \uc124\uc815", None))
        self.groupResolution.setTitle(QCoreApplication.translate("MainWindow", u"\ud574\uc0c1\ub3c4", None))
        self.radioRes1080.setText(QCoreApplication.translate("MainWindow", u"1920 x 1080", None))
        self.radioRes720.setText(QCoreApplication.translate("MainWindow", u"1280 x 720", None))
        self.radioRes480.setText(QCoreApplication.translate("MainWindow", u"640 x 480", None))
        self.labelFPS.setText(QCoreApplication.translate("MainWindow", u"FPS:", None))
        self.groupCalibration.setTitle(QCoreApplication.translate("MainWindow", u"\uce98\ub9ac\ube0c\ub808\uc774\uc158", None))
        self.labelCalibStatus.setText(QCoreApplication.translate("MainWindow", u"\uc0c1\ud0dc: \uae30\ubcf8\uac12 \uc0ac\uc6a9 \uc911", None))
        self.btnLoadCalib.setText(QCoreApplication.translate("MainWindow", u"\ub85c\ub4dc", None))
        self.btnNewCalib.setText(QCoreApplication.translate("MainWindow", u"\uc0c8 \uce98\ub9ac\ube0c\ub808\uc774\uc158", None))
        self.groupOffsetSettings.setTitle(QCoreApplication.translate("MainWindow", u"\uc624\ud504\uc14b \uc124\uc815", None))
        self.groupApproachOffset.setTitle(QCoreApplication.translate("MainWindow", u"\uc811\uadfc \uc624\ud504\uc14b (mm)", None))
        self.labelOffsetX.setText(QCoreApplication.translate("MainWindow", u"X:", None))
        self.labelOffsetY.setText(QCoreApplication.translate("MainWindow", u"Y:", None))
        self.labelOffsetZ.setText(QCoreApplication.translate("MainWindow", u"Z:", None))
        self.groupArUcoSettings.setTitle(QCoreApplication.translate("MainWindow", u"ArUco \ub9c8\ucee4 \uc124\uc815", None))
        self.labelMarkerSize.setText(QCoreApplication.translate("MainWindow", u"\ub9c8\ucee4 \ud06c\uae30 (mm):", None))
        self.labelDictionary.setText(QCoreApplication.translate("MainWindow", u"Dictionary:", None))
        self.comboDictionary.setItemText(0, QCoreApplication.translate("MainWindow", u"DICT_4X4_50", None))
        self.comboDictionary.setItemText(1, QCoreApplication.translate("MainWindow", u"DICT_4X4_100", None))
        self.comboDictionary.setItemText(2, QCoreApplication.translate("MainWindow", u"DICT_4X4_250", None))
        self.comboDictionary.setItemText(3, QCoreApplication.translate("MainWindow", u"DICT_5X5_50", None))
        self.comboDictionary.setItemText(4, QCoreApplication.translate("MainWindow", u"DICT_6X6_50", None))

        self.btnSaveSettings.setText(QCoreApplication.translate("MainWindow", u"\uc124\uc815 \uc800\uc7a5", None))
        self.btnLoadSettings.setText(QCoreApplication.translate("MainWindow", u"\uc124\uc815 \ubd88\ub7ec\uc624\uae30", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tabSettings), QCoreApplication.translate("MainWindow", u"\uc124\uc815", None))
        self.menuFile.setTitle(QCoreApplication.translate("MainWindow", u"\ud30c\uc77c", None))
        self.menuRobot.setTitle(QCoreApplication.translate("MainWindow", u"\ub85c\ubd07", None))
        self.menuVision.setTitle(QCoreApplication.translate("MainWindow", u"\ube44\uc804", None))
        self.menuHelp.setTitle(QCoreApplication.translate("MainWindow", u"\ub3c4\uc6c0\ub9d0", None))
        self.toolBar.setWindowTitle(QCoreApplication.translate("MainWindow", u"toolBar", None))
    # retranslateUi

