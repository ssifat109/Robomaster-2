"""Create exact-size AprilTags and a camera calibration checkerboard."""
from pathlib import Path
import cv2
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from PIL import Image

root = Path(__file__).resolve().parents[1]
out = root/'output/pdf'
out.mkdir(parents=True, exist_ok=True)
assets = root/'output/tags'
assets.mkdir(parents=True, exist_ok=True)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
c = canvas.Canvas(str(out/'apriltags_40mm.pdf'), pagesize=(210*mm, 297*mm))
c.setTitle('AprilTags 1, 2, 3 - 40 mm black square')
c.setFont('Helvetica-Bold', 16)
c.drawString(20*mm, 277*mm, 'RoboMaster targets: AprilTag 36h11')
c.setFont('Helvetica', 10)
c.drawString(20*mm, 268*mm, 'Print Actual size / 100%. Disable Fit to page. Measure before use.')
for i in (1, 2, 3):
    y = (220-(i-1)*68)*mm
    marker = cv2.aruco.drawMarker(dictionary, i, 800)
    Image.fromarray(marker).save(assets/f'tag{i}_black_40mm.png')
    c.drawImage(ImageReader(Image.fromarray(marker)), 30*mm, y, 40*mm, 40*mm)
    c.setDash(2, 2)
    c.rect(25*mm, y-5*mm, 50*mm, 50*mm)
    c.setDash()
    c.setFont('Helvetica-Bold', 12)
    c.drawString(85*mm, y+25*mm, f'Tag {i} (ID {i})')
    c.setFont('Helvetica', 10)
    c.drawString(85*mm, y+17*mm, 'Black outer square: 40 x 40 mm')
    c.drawString(85*mm, y+10*mm, 'Cut line: 50 x 50 mm')
    c.drawString(85*mm, y+3*mm, 'Keep the white margin visible.')
c.line(25*mm, 20*mm, 125*mm, 20*mm)
c.drawString(25*mm, 13*mm, 'This reference line must measure 100 mm.')
c.save()
c = canvas.Canvas(str(out/'camera_checkerboard_20mm.pdf'), pagesize=(210*mm, 297*mm))
c.setTitle('Camera calibration - 9 by 6 inner corners - 20 mm squares')
c.setFont('Helvetica-Bold', 13)
c.drawString(10*mm, 275*mm, 'Camera calibration: 9 x 6 inner corners')
c.setFont('Helvetica', 10)
c.drawString(10*mm, 265*mm, '20 mm squares. Print at 100%. Mount on a flat rigid surface.')
for row in range(7):
    for col in range(10):
        if (row+col)%2 == 0:
            c.rect((5+col*20)*mm, (70+row*20)*mm, 20*mm, 20*mm, fill=1, stroke=0)
c.save()
print(out)
