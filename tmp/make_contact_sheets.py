from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

root = Path(r"C:\Users\mateb\Desktop\ccs_project\ccs-project\tmp\docx_render")
pages = sorted(root.glob("page-*.png"))

for sheet_index in range(0, len(pages), 4):
    group = pages[sheet_index:sheet_index + 4]
    thumbs = []
    for path in group:
        img = Image.open(path).convert("RGB")
        img.thumbnail((620, 880))
        canvas = Image.new("RGB", (650, 940), "white")
        x = (650 - img.width) // 2
        canvas.paste(img, (x, 38))
        draw = ImageDraw.Draw(canvas)
        draw.text((20, 12), path.stem, fill="black")
        thumbs.append(canvas)
    while len(thumbs) < 4:
        thumbs.append(Image.new("RGB", (650, 940), "white"))
    sheet = Image.new("RGB", (1300, 1880), "#D9D9D9")
    for idx, thumb in enumerate(thumbs):
        sheet.paste(thumb, ((idx % 2) * 650, (idx // 2) * 940))
    sheet.save(root / f"contact-{sheet_index // 4 + 1}.png", quality=92)
