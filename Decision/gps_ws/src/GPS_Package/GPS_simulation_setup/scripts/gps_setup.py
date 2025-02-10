
import sys
from PyQt5.QtCore import Qt
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QDialog, QFileDialog, QTableWidgetItem
from Tkinter import *
import numpy as np

from pyproj import Proj


form_class = uic.loadUiType('gps_setup.ui')[0]

print(form_class)

m_lat  = 37.338443  
m_long = 127.899031

m_lat_2  = 37.338443  
m_long_2 = 127.899031

UTM_x  = 402474.39110413427
UTM_y  = 4132986.3800136615

myProj = Proj("+proj=utm +zone=52S, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")

# WGS1984
proj_WGS84 = Proj(init='epsg:4326') # Wgs84


class WindowClass(QDialog, form_class):
	def __init__(self):
		super(QDialog,self).__init__() #super().__init__()
		self.setupUi(self)
		
		self.textEdit_latitude.setText(str(m_lat))
		self.textEdit_longitude.setText(str(m_long))
		
		self.pushButton_gpsgen.clicked.connect(self.test1)
		
		
	def test1(self):
		
		print("clicked")
		
		text_lat = self.textEdit_latitude.toPlainText()
		text_long = self.textEdit_longitude.toPlainText()
		
		m_lat  = float(text_lat)
		m_long = float(text_long)
		
		UTM_x, UTM_y = myProj(m_long, m_lat)
		print(m_lat, m_long)
		print(UTM_x, UTM_y)
		
		m_long_1, m_lat_1 = myProj(UTM_x,UTM_y + 0.0 , inverse=True)
		m_long_2, m_lat_2 = myProj(UTM_x,UTM_y + 0.3, inverse=True)
		
		print(m_lat_1, m_long_1)
		print(m_lat_2, m_long_2)
		#QMessageBox.about(self, "message", "clicked")
		
			


if __name__ == "__main__":
	app = QApplication(sys.argv)
	myWindow = WindowClass()
	myWindow.show()
	app.exec_()
