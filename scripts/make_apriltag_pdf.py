from pathlib import Path
import cv2
import numpy as np
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.utils import ImageReader

out = Path('/mnt/d/RB 2/output/pdf/apriltags_14cm_a4.pdf')
out.parent.mkdir(parents=True, exist_ok=True)
W, H = A4
side = 140 * 72 / 25.4
dict_ = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
c = canvas.Canvas(str(out), pagesize=A4)
c.setTitle('AprilTags 1, 2, 3 - 140 mm black square on A4')
for tag_id in (1, 2, 3):
    pixels = 1400
    image = np.zeros((pixels, pixels), dtype=np.uint8)
    cv2.aruco.drawMarker(dict_, tag_id, pixels, image, borderBits=1)
    # Keep the printed black square exactly 140 mm edge-to-edge.
    tag_png = Path(f'/tmp/apriltag_{tag_id}.png')
    cv2.imwrite(str(tag_png), image)
    x = (W - side) / 2
    y = (H - side) / 2 + 10
    c.drawImage(str(tag_png), x, y, width=side, height=side, preserveAspectRatio=True, mask='auto')
    c.setFont('Helvetica-Bold', 12)
    c.drawCentredString(W / 2, y - 24, f'AprilTag {tag_id} - DICT_APRILTAG_36h11 - black square: 140 mm x 140 mm')
    c.setFont('Helvetica', 9)
    c.drawCentredString(W / 2, 24, 'Print at 100% / Actual size. Do not use Fit to page.')
    c.showPage()
c.save()
print(out)
