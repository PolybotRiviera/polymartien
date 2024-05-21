import json
import time

from PySide6.QtCore import Slot, Signal, Qt, QCoreApplication, QTimer
from PySide6.QtWidgets import (
    QFileDialog,
    QHBoxLayout,
    QMainWindow,
    QMenu,
    QMenuBar,
    QVBoxLayout,
    QWidget,
    QTableView,
    QProgressDialog,
)
from arena import IArena, Bot
from coords import Coord


class IMainWindow(QMainWindow):

    closed_signal = Signal()

    def __init__(self) -> None:
        """Creates the main window."""
        QMainWindow.__init__(self)
        self.bot1 = Bot("blue")
        self.bot2 = Bot("yellow")
        self.bot1.move(Coord(225, 225))
        self.bot2.move(Coord(2775, 225))
        self.arena = IArena(self.bot1, self.bot2)
        self.setCentralWidget(self.arena)
        self.setWindowTitle("Simul Strategy")
        self.resize(3000, 2000)
        self.closed_signal.connect(self.arena.close)
        self.closed_signal.connect(self.close)
        self.timer_ticks = QTimer(self)
        self.timer_ticks.timeout.connect(self.arena.update_bots)
        self.timer_ticks.start(1000 / 60)

    @Slot()
    def close_window(self) -> None:
        """Close the window."""
        self.close()

    def closeEvent(self, event) -> None:
        """
        Method called on window closed. Used for stopping all the timers.

        Overloaded from QMainWindow.
        """
        print(f"closing event: {event}")
        self.closed_signal.emit()
        time.sleep(0.2)
        print("closing")
        event.accept()

    @Slot()
    def save_all(self) -> None:
        """Save the data from all views in a JSON file."""
        _file_name: tuple[str, str] = QFileDialog.getSaveFileName(
            self, "Save File", "", "Json Files (*.json)"
        )
        if _file_name[0]:
            _dict = {}
            for view in self.views:
                _dict[view.title()] = view.get_data()
            with open(_file_name[0], "w", encoding="utf-8") as _file:
                _file.write(json.dumps(_dict, indent=4))

    @Slot()
    def open_file(self) -> None:
        """Open a JSON file and load the data in the views."""
        file_name: tuple[str, str] = QFileDialog.getOpenFileName(
            self, "Open File", "", filter="Json Files (*.json)"
        )
        if file_name[0]:
            with open(file_name[0], "r", encoding="utf-8") as file:
                _dict = json.loads(file.read())
                for view in self.views:
                    if view.title() in _dict:
                        view.open_data(_dict[view.title()])