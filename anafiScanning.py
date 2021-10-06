import cv2
from pyzbar import pyzbar
# from anafiController import AnafiConnection #just added


class Anafi_Scanning():

    # start the scanning of the QR code
    # accept cv2Frame, focalLength and known_width parameter
    # return back barcodeData which contain the data of the QR code
    def startScanning(self, cv2frame, focalLength, known_width):
        # call decodeDrame function to decode the QR code in the frame. will receive back in list
        decodedData = self.decodeFrame(cv2frame)

        # loop over the list of barcode and call drawBoxAndData function
        for barcode in decodedData:
            barcodeData, info = self.drawBoxAndData(cv2frame, barcode, focalLength, known_width)
            # return barcodeData, known_width
            return barcodeData, info

    # decode QR code that contain the frame
    # accept cv2Frame
    # return back decoded which is in string that contain information of the QR code
    def decodeFrame(self, cv2frame):
        decoded = pyzbar.decode(cv2frame)
        return decoded

    # draw box and data around the QR code
    # accept cv2Frame, barcode, focalLength and known_width
    # return barcodeData
    def drawBoxAndData(self, cv2frame, barcode, focalLength, known_width):

        myFaceListC = []
        myFaceListArea = []

        # extract the bounding box location of the barcode and draw
        # the bounding box surrounding the barcode on the image
        (x, y, w, h) = barcode.rect
        cv2.rectangle(cv2frame, (x, y), (x + w, y + h), (0, 255, 0), 5)
        
        cx = x + w // 2
        cy = x + h // 2
        area = w * h
        cv2.circle(cv2frame, (cx, cy), 5, (0, 255, 0), cv2.FILLED)

        myFaceListC.append([cx, cy])
        myFaceListArea.append(area)


        # call find_data_type_and_decode function
        barcodeData, barcodeType = self.find_data_type_and_decode(barcode)

        # draw the barcode data and barcode type on the image
        text = "{} ({})".format(barcodeData, barcodeType)
        cv2.putText(cv2frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        # call distance_to_camera and draw_distance function
        inches = self.distance_to_camera(known_width, focalLength, w)
        distance = self.distance_to_move(inches) #add by aina
        self.draw_distance(cv2frame, inches)

        if len(myFaceListArea) != 0:
            i = myFaceListArea.index(max(myFaceListArea))
            return barcodeData, [myFaceListC[i], myFaceListArea[i]]

        else:
            return barcodeData, [[0, 0], 0]

       

    def distance_to_scan(self, frame):
        myFaceListC = []
        myFaceListArea = []



    def distance_to_move(self, distance):
        if distance < 65: 
            # print("drone too close")
            return distance

        else:
            return 
        

    # find the data type and decode the QR code
    # accept barcode
    # return barcodeData and barcodeType
    def find_data_type_and_decode(self, barcode):
        # the barcode data is a bytes object so if we want to draw it
        # on output image we need to convert it to a string first
        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type
        return barcodeData, barcodeType

    # find distance to camera and return the calculated distance
    # def distance_to_camera(self, knownWidth, focalLength, perWidth):
    #     # compute and return the distance from the maker to the camera
    #     return (knownWidth * focalLength) / perWidth

    # draw the distance in the frame but putting the text at the most bottom right frame, green colour
    def draw_distance(self, frame, inches):
        cv2.putText(frame, "%.2fcm" % inches,
                    (frame.shape[1] - 200, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
                    1.3, (0, 255, 0), 3)

        
