from PyQt5.QtWidgets import (QWidget, QHBoxLayout, QPushButton, QLabel, QMainWindow, 
                             QGroupBox)
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont
from PyQt5.QtCore import Qt, QPoint, pyqtSignal
from PyQt5.QtGui import QPalette

from measurement.measurement import Contacts


class ContactsView(QMainWindow):

    selection_changed = pyqtSignal()

    def __init__(self, parent = None):
        super().__init__(parent)
        self.setGeometry(0, 0, 240, 240)

        self.setBackgroundRole(QPalette.Background)
        #self.setAutoFillBackground(True)

        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)

        self._contacts = [  dict(position=(5, 95), sector = 1, contact=7, selected=False),
                            dict(position=(5, 65), sector = 1, contact=8, selected=False),
                            dict(position=(5, 35), sector = 1, contact=2, selected=False),
                            dict(position=(35, 5), sector = 1, contact=4, selected=False),
                            dict(position=(65, 5), sector = 1, contact=3, selected=False),
                            dict(position=(95, 5), sector = 1, contact=1, selected=False),
                            dict(position=(125, 5), sector = 2, contact=1, selected=False),
                            dict(position=(155, 5), sector = 2, contact=3, selected=False),
                            dict(position=(185, 5), sector = 2, contact=4, selected=False),
                            dict(position=(215, 35), sector = 2, contact=2, selected=False),
                            dict(position=(215, 65), sector = 2, contact=8, selected=False),
                            dict(position=(215, 95), sector = 2, contact=7, selected=False),
                            dict(position=(215, 125), sector = 3, contact=7, selected=False),
                            dict(position=(215, 155), sector = 3, contact=8, selected=False),
                            dict(position=(215, 185), sector = 3, contact=2, selected=False),
                            dict(position=(185, 215), sector = 3, contact=4, selected=False),
                            dict(position=(155, 215), sector = 3, contact=3, selected=False),
                            dict(position=(125, 215), sector = 3, contact=1, selected=False),
                            dict(position=(95, 215), sector = 4, contact=1, selected=False),
                            dict(position=(65, 215), sector = 4, contact=3, selected=False),
                            dict(position=(35, 215), sector = 4, contact=4, selected=False),
                            dict(position=(5, 185), sector = 4, contact=2, selected=False),
                            dict(position=(5, 155), sector = 4, contact=8, selected=False),
                            dict(position=(5, 125), sector = 4, contact=7, selected=False),
        ]

        self.number_of_contacts = 0

        self._last_selection = []

    @property
    def number_of_contacts(self):
        return self._number_of_active_contacts

    @number_of_contacts.setter 
    def number_of_contacts(self, value: int):
        self.deselect()
        self._last_selection = []
        if value <= 0:
            self._number_of_active_contacts = -1
        else:
            self._number_of_active_contacts = value

    @property
    def selection(self):
        sectors = ['', 'I', 'II', 'III', 'IV']
        return ['{:s}{:1d}'.format(sectors[contact['sector']], contact['contact'])
                for contact in self._contacts if contact['selected']]

    def leaveEvent(self, event):
        self.hide()

    def deselect(self):
        self._last_selection = []
        for contact in self._contacts:
            contact['selected'] = False
        self.selection_changed.emit()

    def _add_last_selection(self, contact):
        if contact in self._last_selection:
            self._last_selection.remove(contact)

        if len(self._last_selection) < self._number_of_active_contacts:
            self._last_selection.append(contact)
        

    def select(self, contact_string: str):
        if len(self._last_selection) >= self._number_of_active_contacts:
            return

        sector = 0
        contact = 0
        if contact_string.startswith('IV'):
            sector = 4
            contact = int(contact_string[2:])
        elif contact_string.startswith('III'):
            sector = 3
            contact = int(contact_string[3:])
        elif contact_string.startswith('II'):
            sector = 2
            contact = int(contact_string[2:])
        elif contact_string.startswith('I'):
            sector = 1
            contact = int(contact_string[1:])

        for contact_item in self._contacts:
            if contact_item['sector'] == sector and contact_item['contact'] == contact:
                contact_item['selected'] = True
                self._add_last_selection(contact_item)
                self.selection_changed.emit()
                break


    def select_list(self, contact_list):
        self.deselect()
        for item in contact_list:
            self.select(item)


    def mousePressEvent(self, event):
        if event.button() == Qt.RightButton:
            self.next()
        else:
            x = event.x() - 10
            y = event.y() - 10
            radius = 10
            radius_sq = radius**2

            changed = False
            for contact in self._contacts:
                contact_x, contact_y = contact['position']
                dist = (x - contact_x)**2 + (y - contact_y)**2
                if dist <= radius_sq:
                    #self._deselect()
                    contact['selected'] = ~contact['selected']
                    changed = True 

                    if contact['selected'] and self._number_of_active_contacts > 0 and contact not in self._last_selection:
                        if len(self._last_selection) >= self._number_of_active_contacts:
                            self._last_selection[0]['selected'] = False
                            self._last_selection = self._last_selection[1:]

                        self._last_selection.append(contact)
                    elif not contact['selected'] and self._number_of_active_contacts > 0:
                        if contact in self._last_selection:
                            self._last_selection.remove(contact)

                    break

            if changed:
                self.selection_changed.emit()
                self.update()

    def next(self):
        selection = [contact['selected'] for contact in self._contacts]
        new_selection = [selection[-1]] + selection[:-1]

        for index, contact in enumerate(self._contacts):
            contact['selected'] = new_selection[index]
        
        self.selection_changed.emit()

        self.update()
            
        

    def paintEvent(self, e):
        painter = QPainter()
        painter.begin(self)
        self.draw_things(painter)
        painter.end()

    def draw_things(self, painter: QPainter):
        painter.setRenderHint(painter.Antialiasing)


        background_color = QColor(QPalette.Background)

        painter.setPen(background_color)
        painter.drawRoundedRect(self.rect(), 4, 4)


        col = QColor(20, 20, 20)
        color_green = QColor(100, 200, 100)
        color_dark_green = QColor(50, 100, 50)

        color_gray = QColor(200, 200, 200)
        color_dark_gray = QColor(100, 100, 100)

        green_brush = QBrush(color_dark_green)
        green_pen = QPen(green_brush, 3.0)

        gray_brush = QBrush(color_dark_gray)
        gray_pen = QPen(gray_brush, 3.0)

        painter.setPen(col)

        painter.drawLine(120, 5, 120, 235)
        painter.drawLine(5, 120, 235, 120)

        painter.save()

        font = painter.font()
        font.setPointSize(20)
        painter.setFont(font)

        painter.drawText(90, 90, 30, 30, Qt.AlignCenter, 'I')
        painter.drawText(120,90,30,30, Qt.AlignCenter, 'II')
        painter.drawText(120,120,30,30, Qt.AlignCenter, 'III')
        painter.drawText(90,120,30,30, Qt.AlignCenter, 'IV')

        painter.restore()

        for contact in self._contacts:
            x, y = contact['position']

            pen, brush_color = (green_pen, color_green) if contact['selected'] else (gray_pen, color_gray)


            painter.setPen(pen)
            painter.setBrush(brush_color)
            painter.drawEllipse(x, y, 20, 20)
            painter.setPen(col)
            painter.drawText(x, y, 20, 20, Qt.AlignCenter, str(contact['contact']))


class ContactsSelector(QWidget):

    save_triggered = pyqtSignal()

    def __init__(self, parent = None):
        super().__init__(parent)

        self.setContentsMargins(0,0,0,0)

        layout = QHBoxLayout()
        layout.setContentsMargins(0,0,0,0)
        self._description_label = QLabel('Contacts')
        self._description_label.hide()
        layout.addWidget(self._description_label)

        self._sample_dialog = ContactsView()
        self._sample_dialog.hide()
        self._sample_dialog.selection_changed.connect(self._on_selection_changed)

        self._contact_label = QLabel('No Contacts')
        self._button = QPushButton('\u25bc')
        self._button.hide()
        
        self._add_button = QPushButton('+')
        self._add_button.hide()

        layout.addWidget(self._contact_label)
        layout.addStretch()
        layout.addWidget(self._button)
        layout.addWidget(self._add_button)

        self.setLayout(layout)

        self._button.clicked.connect(self._clicked)
        self._add_button.clicked.connect(self._add)

    def _add(self, event):
        if len(self.contacts) > 0:
            self.save_triggered.emit()

    @property
    def contacts(self):
        return self._sample_dialog.selection

    @property
    def contacts_string(self):
        return '-'.join(self._sample_dialog.selection)

    def set_number_of_contacts(self, contacts: Contacts):
        self._sample_dialog.deselect()
        if contacts == Contacts.NONE:
            self._deactivate_buttons()
            self._sample_dialog.number_of_contacts = 0
        else:
            self._activate_buttons()
            self._sample_dialog.number_of_contacts = contacts.value

    def _deactivate_buttons(self):
        self._button.hide()
        self._add_button.hide()
        self._description_label.hide()
        self._contact_label.setText('No Contacts')

    def _activate_buttons(self):
        self._button.show()
        self._add_button.show()
        self._description_label.show()
        self._contact_label.setText('')

    def _on_selection_changed(self):
        string = self.contacts_string
        self._contact_label.setText(string)
        if len(string) > 0:
            self._description_label.hide()
        else:
            self._description_label.show()

    def select_list(self, contact_list):
        self._sample_dialog.select_list(contact_list)

    def next(self):
        self._sample_dialog.next()

    def _clicked(self):
        if self._sample_dialog.isVisible():
            self._sample_dialog.hide()
        else:
            button_pos = self._button.mapToGlobal(QPoint(0, 0))

            x = button_pos.x()
            y = button_pos.y()

            new_x = x + self._button.width() / 2 - self._sample_dialog.width()/2
            new_y = y + self._button.height() / 2 - self._sample_dialog.height()/2

            self._sample_dialog.move(new_x, new_y)
            self._sample_dialog.raise_()
            self._sample_dialog.show()



if __name__=='__main__':
    import sys
    from PyQt5.QtWidgets import QApplication, QMainWindow

    class MainWindow(QMainWindow):
        def __init__(self):
            super().__init__()

            self.setGeometry(0,0,250,250)

            self.setWindowTitle('ContactsSelector')

            widget = ContactsSelector(self)
            self.setCentralWidget(widget)

            self.show()

    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())
