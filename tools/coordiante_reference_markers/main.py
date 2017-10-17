from PIL import Image, ImageDraw, ImageFont
from fpdf import FPDF
import time
import random

def make_ref_image(i, tag_img):
    canvas = Image.new("RGB", (280 * 2, 200 * 4), (255, 255, 255))  # 4 px = 1 mm

    draw = ImageDraw.Draw(canvas)
    font = ImageFont.truetype('Pillow/Tests/fonts/FreeMono.ttf', 40)
    font_small = ImageFont.truetype('Pillow/Tests/fonts/FreeMono.ttf', 18)
    font_large = ImageFont.truetype('Pillow/Tests/fonts/FreeMono.ttf', 100)

    # Reference length
    draw.line([(280 - 200, 770), (280 + 200, 770)], width=5, fill=0)
    draw.text((225, 730), "100mm", font=font, fill=(0, 0, 0))

    # April tags
    canvas.paste(tag_img.resize((9 * 40, 9 * 40)).copy(), box=(280 - 9 * 20, 40))

    # Date
    draw.text((200, 780), "Date: " + time.strftime("%Y-%m-%d"), font=font_small, fill=(0, 0, 0))

    # Coordinates
    x = i % 4
    y = int(i / 4) % 4
    z = int(i / 16)
    draw.text((40 * 2, 40 * 10 + 20), ("x y z\n%d %d %d" % (x, y, z)), font=font, fill=(0, 0, 0))

    # Tag ID
    draw.text((40 * 8, 40 * 10), "#" + str(i), font=font_large, fill=(0, 0, 0))

    # Coordinate arrow z
    arr_x = 200
    arr_y = 40 * 16 + 20
    r = 20
    draw.ellipse((arr_x - r, arr_y - r, arr_x + r, arr_y + r), fill=0)
    r = 15
    draw.ellipse((arr_x - r, arr_y - r, arr_x + r, arr_y + r), fill=(255, 255, 255))
    r = 5
    draw.ellipse((arr_x - r, arr_y - r, arr_x + r, arr_y + r), fill=0)
    draw.text((arr_x - 20, arr_y + 20), "z", font=font, fill=(0, 0, 0))

    # Coordinate arrow x
    draw.line((arr_x + 20, arr_y, arr_x + 100, arr_y), fill=0, width=5)
    draw.polygon(((arr_x + 100, arr_y - 10, arr_x + 100, arr_y + 10, arr_x + 100 + 30, arr_y)), fill=0)
    draw.text((arr_x + 120, arr_y - 50), "x", font=font, fill=(0, 0, 0))

    # Coordinate arrow y
    draw.line((arr_x, arr_y - 20, arr_x, arr_y - 100), fill=0, width=5)
    draw.polygon(((arr_x - 10, arr_y - 100, arr_x + 10, arr_y - 100, arr_x, arr_y - 100 - 30)), fill=0)
    draw.text((arr_x + 20, arr_y - 150), "y", font=font, fill=(0, 0, 0))
    return canvas

def main():

    aprtiltags = [Image.open('tag25h9/tag25_09_' + format(i, '05') + '.png')  for i in range(35)]
    pdf = FPDF(orientation='L', unit='mm', format='A4')

    for i in range(0,32,2):

        img_left = make_ref_image(i, aprtiltags[i])
        img_right = make_ref_image(i+1, aprtiltags[i+1])

        canvas = Image.new("RGB", (img_left.width + img_right.width + 2, img_left.height), (255, 255, 255))
        canvas.paste(img_left, box=(0,0))
        canvas.paste(img_right, box=(img_left.width+2,0))
        draw = ImageDraw.Draw(canvas)
        draw.line((img_left.width+1,0,img_left.width+1,img_left.height), fill=0,width=2)
        canvas.save("/tmp/tmp"+str(i)+".png","PNG")

        pdf.add_page()
        pdf.image("/tmp/tmp"+str(i)+".png", x= 8.5, y=5 ,w = canvas.width/4, h = canvas.height/4)

    pdf.output("markers.pdf", "F")



if __name__ == "__main__":
    main()