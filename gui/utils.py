from PySide6.QtGui import QImage, QPixmap


def convert_cv_qt(cv_img):
    """Convert a BGR opencv image to QPixmap without an intermediate copy."""
    h, w, ch = cv_img.shape
    bytes_per_line = ch * w
    qt_img = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_BGR888)
    return QPixmap.fromImage(qt_img)
