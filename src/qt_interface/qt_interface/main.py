import sys
import numpy as np
import random as rd

from PyQt5 import QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QBrush, QPainter, QPen, QColor
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
    def __init__(self, width, height):
        super().__init__(0, 0, width, height)
        self.setPos(0, 0)
        brush = QBrush(Qt.red)
        self.setBrush(brush)
        self.setTransformOriginPoint(self.boundingRect().center())
        self.setRotation(60)

    def update(self):
        # self.setRotation(self.rotation() + 3)
        self.move_forward(5, self.get_corners())

    def get_corners(self) -> list[tuple[float, float]]:
        x, y, width, height = self.boundingRect().getRect()
        
        theta_rad = np.deg2rad(self.rotation())
       
        # Calculate the movement vector
        top_left = self.x(), self.y()
        top_right = self.x() + width * np.cos(theta_rad), self.y() + width * np.sin(theta_rad)
        bottom_left = self.x() + height * np.sin(theta_rad), self.y() +height * np.cos(theta_rad)
        bottom_right = self.x() + (width * np.cos(theta_rad) - height * np.sin(theta_rad)), self.y() + width * np.sin(theta_rad) + height * np.cos(theta_rad)

        return [top_left, top_right, bottom_left, bottom_right]

    def move_forward(self, dist, corners):
        if min_x(corners) < -30:
            self.setRotation((180 - self.rotation()) % 360)
            self.setX(max_x(corners))
        if max_x(corners) > 800:
            self.setRotation((180 - self.rotation()) % 360)
            self.setX(min_x(corners))
        if min_y(corners) < -30:
            self.setRotation((360 - self.rotation()) % 360)
            self.setY(max_y(corners))
        if max_y(corners) > 500:
            self.setRotation((360 - self.rotation()) % 360)
            self.setY(min_y(corners))
        
        # Convert the orientation angle to radians
        theta_rad = np.deg2rad(self.rotation())
        # Calculate the movement vector
        dx, dy = dist * np.cos(theta_rad), dist * np.sin(theta_rad)
        
        # Update the rectangle's position
        new_x, new_y = self.x() + dx, self.y() + dy
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
        self.scene = QGraphicsScene(0, 0, 800, 500)

        # Draw a Car
        car = Car(30, 30)

        pen = QPen(QColor("black"), 4)

        coordinates = []

        # create a small ellipse item to simulate a point
        for i in range(12):
            random_x = rd.randint(25, 775)
            random_y = rd.randint(25, 475)
            point = self.scene.addEllipse(random_x, random_y, 1, 1, pen)
            coordinates.append((random_x, random_y))

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

def display():
    print("Hello World")


def main(args=None):
    app = QApplication(sys.argv)
    w = CarTrack()
    w.show()
    app.exec()

