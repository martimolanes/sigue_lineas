import sys
import numpy as np

from PyQt5 import QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QBrush, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

class Car(QGraphicsRectItem):
    def __init__(self):
        self.width = 10
        self.height = 50
        super().__init__(0, 0, self.width, self.height)
        self.setPos(50, 20)
        brush = QBrush(Qt.red)
        self.setBrush(brush)
        self.setTransformOriginPoint(self.boundingRect().center())

    def update(self):
        self.setRotation(-90)
        #self.setRotation(self.rotation() + 1)
        self.move_forward(1, self.get_corners())

    def get_corners(self) -> list[tuple[float, float]]:
        x, y, width, height = self.boundingRect().getRect()
        
        theta_rad = np.deg2rad(self.rotation())
       
        # Calculate the movement vector
        top_left = self.x(), self.y()
        top_right = self.x() + width * np.cos(theta_rad), self.y() + width * np.sin(theta_rad)
        bottom_left = self.x() + height * np.sin(theta_rad), self.y() +height * np.cos(theta_rad)
        bottom_right = self.x() + (width * np.cos(theta_rad) - height * np.sin(theta_rad)), self.y() + width * np.sin(theta_rad) + height * np.cos(theta_rad)
        print(top_left, top_right, bottom_right, bottom_left)

        return [top_left, top_right, bottom_left, bottom_right]

    def move_forward(self, dist, corners):
        # Convert the orientation angle to radians
        theta_rad = np.deg2rad(self.rotation())
        # Calculate the movement vector
        dx, dy = dist * np.cos(theta_rad), dist * np.sin(theta_rad)
        
        # Update the rectangle's position
        new_x, new_y = self.x() + dx, self.y() + dy
        
        print(min_x(corners))
        
        if min_x(corners) < 0:
            new_x = 0
            self.setRotation(180 - self.rotation())
        if max_x(corners) > 400:
            new_x = 400
            self.setRotation(180 - self.rotation())
        if min_y(corners) < 0:
            new_y = 0
            self.setRotation(180 - self.rotation())
        if max_y(corners) > 200:
            new_y = 180
            self.setRotation(360 - self.rotation())
        self.setX(new_x) 
        self.setY(new_y)

def min_x(corners):
    min = np.Inf
    for corner in corners:
        if corner[0] < min:
            min = corner[0]
    return min
        
def max_x(corners):
    max = -np.Inf
    for corner in corners:
        if corner[0] > max:
            max = corner[0]
    return max

def min_y(corners):
    min = np.Inf
    for corner in corners:
        if corner[1] < min:
            min = corner[1]
    return min

def max_y(corners):
    max = -np.Inf
    for corner in corners:
        if corner[1] > max:
            max = corner[1]
    return max


class CarTrack(QWidget):
    def __init__(self):
        super().__init__()

        # Defining a scene rect of 400x200
        self.scene = QGraphicsScene(0, 0, 400, 200)

        # Draw a Car
        car = Car()

        # Add the items to the scene. Items are stacked in the order they are added.
        self.scene.addItem(car)

        # Set all items as moveable and selectable.
        for item in self.scene.items():
            item.setFlag(QGraphicsItem.ItemIsMovable)
            item.setFlag(QGraphicsItem.ItemIsSelectable)

        view = QGraphicsView(self.scene)
        view.setRenderHint(QPainter.Antialiasing)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(17)

        hbox = QHBoxLayout(self)
        hbox.addWidget(view)

        self.setLayout(hbox)


    def update(self):
        for item in self.scene.items():
            item.update()

    def up(self):
        """ Iterate all selected items in the view, moving them forward. """
        items = self.scene.selectedItems()
        for item in items:
            z = item.zValue()
            item.setZValue(z + 1)

    def down(self):
        """ Iterate all selected items in the view, moving them backward. """
        items = self.scene.selectedItems()
        for item in items:
            z = item.zValue()
            item.setZValue(z - 1)

    def rotate(self):
        """ Rotate the object by the received number of degrees """
        items = self.scene.items()
        for item in items:
            value = item.rotation() + 10
            print(value)
            item.setTransformOriginPoint(item.boundingRect().center())
            item.setRotation(value)


def display():
    print("Hello World")

app = QApplication(sys.argv)

w = CarTrack()
w.show()

app.exec()
