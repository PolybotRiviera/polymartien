"""Main module."""
import sys
import time
from typing import NoReturn

from PySide6.QtWidgets import QApplication

import main_window


def main() -> NoReturn:
    """Main function."""
    app = QApplication(sys.argv)

    window = main_window.IMainWindow()
    window.show()

    print("launching")
    exit_code: int = app.exec()
    print(f"Quitted, code = {exit_code}")
    time.sleep(0.5)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()