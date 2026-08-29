"""
Regenerates icon.ico from icon.png. Run automatically by BUILD_EXE.bat before
every build, so a new icon.png just works next build without a manual step.

Pads to a square canvas first -- icon.png isn't necessarily square, and
resizing a non-square source directly into square .ico frames would squash
it. Padding with transparency keeps the actual artwork's proportions intact.
"""
import os
from PIL import Image

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'icon.png')
DST = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'icon.ico')
SIZES = [(16, 16), (24, 24), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)]

if os.path.isfile(SRC):
    img = Image.open(SRC).convert('RGBA')
    w, h = img.size
    side = max(w, h)
    square = Image.new('RGBA', (side, side), (0, 0, 0, 0))
    square.paste(img, ((side - w) // 2, (side - h) // 2), img)
    square.save(DST, format='ICO', sizes=SIZES)
    print(f'icon.ico regenerated from icon.png ({w}x{h} -> {side}x{side} padded)')
else:
    print('icon.png not found -- skipping icon regeneration')
