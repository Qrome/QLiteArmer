from PIL import Image, ImageDraw, ImageFont

# --- INPUT IMAGE ---
src = Image.open("DJI-Font.png")

# --- GRID PARAMETERS ---
cols = 16
rows = 32

cell_w = 13   # 12px content + 1px right border
cell_h = 19   # 18px content + 1px bottom border

content_w = 12
content_h = 18

border = 1

# --- SCALE CONTENT UNIFORMLY (3×) ---
scaled_w = content_w * 3   # 36
scaled_h = content_h * 3   # 54

# --- NEW CELL GEOMETRY ---
total_cell_w = scaled_w * 3      # 108
number_area_w = total_cell_w * 2 // 3   # 72
content_area_w = total_cell_w // 3      # 36

# --- BORDER ---
cell_border = 2
final_cell_w = total_cell_w + cell_border * 2   # 112
final_cell_h = scaled_h + cell_border * 2       # 58

# --- OUTPUT IMAGE ---
out = Image.new("RGBA", (cols * final_cell_w, rows * final_cell_h), (0, 0, 0, 0))
draw = ImageDraw.Draw(out)

# Choose a font
try:
    font = ImageFont.truetype("arial.ttf", 28)
except:
    font = ImageFont.load_default()

index = 0

for r in range(rows):
    for c in range(cols):

        # --- Extract original content region ---
        x0 = c * cell_w + border
        y0 = r * cell_h + border
        x1 = x0 + content_w
        y1 = y0 + content_h

        cell_content = src.crop((x0, y0, x1, y1))

        # --- Scale content uniformly ---
        cell_content = cell_content.resize((scaled_w, scaled_h), Image.NEAREST)

        # --- Output placement ---
        out_x = c * final_cell_w
        out_y = r * final_cell_h

        # --- Draw border ---
        draw.rectangle(
            [out_x, out_y, out_x + final_cell_w - 1, out_y + final_cell_h - 1],
            outline=(0, 0, 0),
            width=cell_border
        )

        # --- Paste content on RIGHT 1/3 of cell ---
        content_x = out_x + cell_border + number_area_w
        content_y = out_y + cell_border
        out.paste(cell_content, (content_x, content_y))

        # --- Draw number in LEFT 2/3 of cell ---
        num_x = out_x + cell_border + 10
        num_y = out_y + cell_border + 10
        draw.text((num_x +1, num_y +1), str(index%256), fill=(0, 0, 0), font=font)
        draw.text((num_x, num_y), str(index%256), fill=(255, 255, 255), font=font)

        index += 1

# --- SAVE ---
out.save("dji_fontmap.png")
print("Done. Saved as dji_fontmap.png")
