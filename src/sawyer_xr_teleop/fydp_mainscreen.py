# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'fydp_mainscreen.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1803, 1150)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.bottom_bar = QtWidgets.QFrame(self.centralwidget)
        self.bottom_bar.setGeometry(QtCore.QRect(60, 945, 1700, 15))
        self.bottom_bar.setAutoFillBackground(False)
        self.bottom_bar.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.bottom_bar.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.bottom_bar.setFrameShadow(QtWidgets.QFrame.Raised)
        self.bottom_bar.setObjectName("bottom_bar")
        self.left_bar = QtWidgets.QFrame(self.centralwidget)
        self.left_bar.setGeometry(QtCore.QRect(60, 95, 15, 850))
        self.left_bar.setAutoFillBackground(False)
        self.left_bar.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.left_bar.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.left_bar.setFrameShadow(QtWidgets.QFrame.Raised)
        self.left_bar.setObjectName("left_bar")
        self.right_bar = QtWidgets.QFrame(self.centralwidget)
        self.right_bar.setGeometry(QtCore.QRect(1745, 95, 15, 850))
        self.right_bar.setAutoFillBackground(False)
        self.right_bar.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.right_bar.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.right_bar.setFrameShadow(QtWidgets.QFrame.Raised)
        self.right_bar.setObjectName("right_bar")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(60, 20, 81, 31))
        self.label.setAutoFillBackground(True)
        self.label.setTextFormat(QtCore.Qt.AutoText)
        self.label.setObjectName("label")
        self.Errors = QtWidgets.QScrollArea(self.centralwidget)
        self.Errors.setGeometry(QtCore.QRect(1420, 840, 325, 105))
        self.Errors.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.Errors.setWidgetResizable(True)
        self.Errors.setObjectName("Errors")
        self.scrollAreaWidgetContents = QtWidgets.QWidget()
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 323, 103))
        self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.scrollAreaWidgetContents)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(0, 0, 319, 101))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.Box_title = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.Box_title.setAlignment(QtCore.Qt.AlignCenter)
        self.Box_title.setObjectName("Box_title")
        self.verticalLayout.addWidget(self.Box_title)
        self.error_1 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.error_1.setMinimumSize(QtCore.QSize(0, 30))
        self.error_1.setAutoFillBackground(True)
        self.error_1.setText("")
        self.error_1.setObjectName("error_1")
        self.verticalLayout.addWidget(self.error_1)
        self.Errors.setWidget(self.scrollAreaWidgetContents)
        self.z_middle_frame = QtWidgets.QFrame(self.centralwidget)
        self.z_middle_frame.setGeometry(QtCore.QRect(64, 85, 1681, 871))
        self.z_middle_frame.setAutoFillBackground(False)
        self.z_middle_frame.setStyleSheet("background-color: rgba(0, 0, 0, 0);")
        self.z_middle_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.z_middle_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.z_middle_frame.setObjectName("z_middle_frame")
        self.top_bar = QtWidgets.QFrame(self.centralwidget)
        self.top_bar.setGeometry(QtCore.QRect(60, 80, 1700, 15))
        self.top_bar.setAutoFillBackground(False)
        self.top_bar.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.top_bar.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.top_bar.setFrameShadow(QtWidgets.QFrame.Raised)
        self.top_bar.setObjectName("top_bar")
        self.z_middle_frame.raise_()
        self.bottom_bar.raise_()
        self.left_bar.raise_()
        self.right_bar.raise_()
        self.label.raise_()
        self.Errors.raise_()
        self.top_bar.raise_()
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1803, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "iControbot"))
        self.Box_title.setText(_translate("MainWindow", "Message"))


