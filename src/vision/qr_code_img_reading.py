# import the necessary packages
from pyzbar import pyzbar
import argparse
import cv2
import os

class QrCode:

    def test(self, qr_code_filename):
        image_path = os.path.join(os.getcwd(), 'res', 'qr-codes', qr_code_filename)
        frame = cv2.imread(image_path)
        self.get_qr_code_data(frame)

    def read_qr_from_camera(self, cam_id, debug = False):
        cam = cv2.VideoCapture(cam_id)
        _, frame = cam.read()
        cam.release()
        
        barcodeData, _ = self.get_qr_code_data(frame)

        if debug:
            cv2.imwrite("qr-code.png", frame)

        return barcodeData


    def get_qr_code_data(self, image, show_plot=False):
        """Retrieve qr code data from image

        Args:
            image (cv2 bgr image): 
            show_plot (bool, optional): If a plot should be shown. Defaults to False.

        Returns:
            _type_: _description_
        """
        # find the barcodes in the image and decode each of the barcodes
        barcodes = pyzbar.decode(image)
        barcodeData = []
        barcodeType = []

        # loop over the detected barcodes
        for barcode in barcodes:
            # the barcode data is a bytes object so if we want to draw it on
            # our output image we need to convert it to a string first
            barcodeData.append(barcode.data.decode("utf-8"))
            barcodeType.append(barcode.type)
            # print the barcode type and data to the terminal
            print("[INFO] Found {}: {}".format(barcodeType, barcodeData))

            if show_plot is True:
                # extract the bounding box location of the barcode and draw the
                # bounding box surrounding the barcode on the image
                (x, y, w, h) = barcode.rect
                cv2.rectangle(self.image, (x, y),
                              (x + w, y + h), (0, 0, 255), 2)
                # draw the barcode data and barcode type on the image
                text = "{} ({})".format(barcode.data.decode("utf-8"), barcode.type)
                cv2.putText(self.image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 255), 2)

        if show_plot is True:
            # show the output image
            cv2.imwrite("QR.png", self.image)
            cv2.waitKey(0)

        return barcodeData, barcodeType
