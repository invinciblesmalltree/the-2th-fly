from pyzbar.pyzbar import decode
from PIL import Image


def decode_barcode(image_path):
    image = Image.open(image_path)
    barcodes = decode(image)

    for barcode in barcodes:
        barcode_type = barcode.type
        barcode_data = barcode.data.decode('utf-8')
        print(f"发现条形码：类型：{barcode_type}，数据：{barcode_data}")


decode_barcode('D:\\Temp\\download.png')
