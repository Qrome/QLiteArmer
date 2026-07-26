from PIL import Image, ImageDraw, ImageFont

# -----------------------------
# CONFIGURATION
# -----------------------------
GLYPH_W = 36
GLYPH_H = 54

ROWS = 256
COLS_PER_OUTPUT = 16   # 32 glyphs per column in the output image

INPUT_PNG = "WS_BFx4_Europa_36.png"
OUTPUT_PNG = "first_column_ascii_map.png"

# Font for ASCII labels (fallback to default if not found)
try:
    label_font = ImageFont.truetype("arial.ttf", 16)
except:
    label_font = ImageFont.load_default()

# -----------------------------
# LOAD FONT PNG
# -----------------------------
font_img = Image.open(INPUT_PNG)

# -----------------------------
# OUTPUT IMAGE SIZE
# -----------------------------
# Each glyph row becomes: [ASCII text][glyph]
LABEL_W = 30
CELL_W = LABEL_W + GLYPH_W
CELL_H = GLYPH_H

NUM_OUTPUT_COLS = (ROWS + COLS_PER_OUTPUT - 1) // COLS_PER_OUTPUT

OUT_W = NUM_OUTPUT_COLS * CELL_W
OUT_H = COLS_PER_OUTPUT * CELL_H

output = Image.new("RGBA", (OUT_W, OUT_H), (0, 0, 0, 0))
draw = ImageDraw.Draw(output)

# -----------------------------
# PROCESS EACH ROW
# -----------------------------
for row in range(ROWS):
    ascii_value = row  # the "character code" you want to display

    # Extract glyph from first column (col 0)
    src_x = 0
    src_y = row * GLYPH_H
    glyph = font_img.crop((src_x, src_y, src_x + GLYPH_W, src_y + GLYPH_H))

    # Determine output position
    out_col = row // COLS_PER_OUTPUT
    out_row = row % COLS_PER_OUTPUT

    dst_x = out_col * CELL_W
    dst_y = out_row * CELL_H

    # Draw ASCII label
    label = f"{ascii_value:03d}"
    draw.text((dst_x + 3, dst_y + 5), label, fill="black", font=label_font)
    draw.text((dst_x + 2, dst_y + 4), label, fill="white", font=label_font)

    if(ascii_value > 32 and ascii_value < 126):
        draw.text((dst_x+11,dst_y+23), f"{chr(ascii_value)}", fill="black", font=label_font)
        draw.text((dst_x+10,dst_y+22), f"{chr(ascii_value)}", fill="white", font=label_font)


    # Paste glyph
    output.paste(glyph, (dst_x + LABEL_W, dst_y))

# -----------------------------
# SAVE RESULT
# -----------------------------
output.save(OUTPUT_PNG)
print(f"Created: {OUTPUT_PNG}")
