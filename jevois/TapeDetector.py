import libjevois as jevois
import numpy as np
import cv2
from pipeline import TapePipeline
import math


class TapeDetector:
    TARGET_STRIP_WIDTH = 2.0  # in
    TARGET_STRIP_LENGTH = 5.5  # in

    TARGET_STRIP_CORNER_OFFSET = 4.0  # in

    TARGET_STRIP_ROT = math.radians(14.5)

    def __init__(self):
        self.pipeline = TapePipeline()
        self.width = 640
        self.fov = 65
        self.outimg = None
        # Extra samples to improve PnP
        avgL = [sum(x) / 2.0 for x in zip(self.left_strip[2], self.left_strip[1])]
        avgR = [sum(x) / 2.0 for x in zip(self.right_strip[2], self.right_strip[1])]
        avgHigh = [sum(x) / 2.0 for x in zip(self.left_strip[1], self.right_strip[1])]
        avgLow = [sum(x) / 2.0 for x in zip(self.left_strip[2], self.right_strip[2])]
        self.outsideTargetCoords = np.array([self.left_strip[2], self.left_strip[1], avgL, avgHigh,
                                             avgLow, avgR, self.right_strip[1], self.right_strip[2]])

    @classmethod
    def init_class_variables(cls):
        cosa = math.cos(cls.TARGET_STRIP_ROT)
        sina = math.sin(cls.TARGET_STRIP_ROT)

        pt = [cls.TARGET_STRIP_CORNER_OFFSET, 0.0, 0.0]
        strip = [tuple(pt), ]  # This makes a copy of pt
        pt[0] += cls.TARGET_STRIP_WIDTH * cosa
        pt[1] += cls.TARGET_STRIP_WIDTH * sina
        strip.append(tuple(pt))
        pt[0] += cls.TARGET_STRIP_LENGTH * sina
        pt[1] -= cls.TARGET_STRIP_LENGTH * cosa
        strip.append(tuple(pt))
        pt[0] -= cls.TARGET_STRIP_WIDTH * cosa
        pt[1] -= cls.TARGET_STRIP_WIDTH * sina
        strip.append(tuple(pt))

        cls.right_strip = strip
        cls.left_strip = [(-p[0], p[1], p[2]) for p in cls.right_strip]

    def generate_bitmask(self, hsvImg):
        return cv2.morphologyEx(hsvImg, cv2.MORPH_CLOSE, np.ones((3, 3)))

    def loadCameraCalibration(self, w, h):
        """Loads the camera calibration from disk for the given width and height of the image"""
        calibFile = '/jevois/share/camera/calibration{}x{}.yaml'.format(w, h)
        fs = cv2.FileStorage(calibFile, cv2.FILE_STORAGE_READ)
        if fs.isOpened():
            self.camMatrix = fs.getNode("camera_matrix").mat()
            self.distCoeffs = fs.getNode("distortion_coefficients").mat()
            jevois.LINFO(f"Loaded camera calibration from {calibFile}")
        else:
            jevois.LFATAL(f"Failed to read camera calibration parameters from {calibFile}")

    def process(self, inframe, outframe):
        inimg = inframe.getCvBGR()
        self.pipeline.process(inimg)
        contours = self.pipeline.filter_contours_output

        # Transform pipeline contours into Rect, sort by x centre
        contourRects = [cv2.minAreaRect(cnt) for cnt in contours]
        contourRects.sort(key=sortContours)

        self.outimg = inimg

        # Mark all seen contours as such for drivers
        for rect in contourRects:
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(self.outimg,
                             [box],
                             0,
                             (0, 255, 0),
                             2)

        tapeIndices = []
        for i in range(0, len(contourRects) - 1):
            (_, _), (_, _), angle = contourRects[i]
            (_, _), (_, _), nextAngle = contourRects[i + 1]

            # If the angle of the rectangle is within the angle of the tape
            jevois.LINFO(f"First angle is {int(angle)}. Next is {int(nextAngle)}")
            if -80 < int(angle) <= -40:
                jevois.LINFO("Past first check")
                if -40 < int(nextAngle) <= -10:
                    jevois.LINFO(f"Appending index {i}")

                    tapeIndices.append(i)

        distances = []
        for idx, button in zip(tapeIndices, ['A', 'B', 'X', 'Y']): # these are just examples, change to whatever
            idx = tapeIndices[0]
            meanTapeXPos = (contourRects[idx][0][0] + contourRects[idx + 1][0][0]) / 2.0
            meanTapeYPos = (contourRects[idx][0][1] + contourRects[idx + 1][0][1]) / 2.0

            # jevois.LINFO("Trying to draw the circle now")
            # cv2.circle(self.outimg, (int(meanTapeXPos), int(contourRects[idx][0][1])), 5, (0, 255, 0), -1)
            # cv2.putText(self.outimg, button, cv2.FONT_HERSHEY_PLAIN, 20, (0, 255, 0), bottomLeftOrigin=(int(meanTapeXPos), int(meanTapeYPos)))

            ### BEGIN BLACK MAGIC FUCKERY
            h, w, _ = inimg.shape
            if not hasattr(self, 'camMatrix'):
                self.loadCameraCalibration(w, h)

            bitmask = self.generate_bitmask(self.pipeline.hsv_threshold_output)

            tapeContours = np.array([contours[idx], contours[idx + 1]])
            corners = self.getCorners(tapeContours, bitmask)

            cv2.circle(self.outimg, (int(corners[0][0][0]), int(corners[0][0][1])), 5, (0, 0, 255), -1)
            cv2.circle(self.outimg, (int(corners[2][0][0]), int(corners[2][0][1])), 5, (0, 0, 255), -1)

            cv2.circle(self.outimg, (int(corners[1][0][0]), int(corners[1][0][1])), 5, (0, 0, 255), -1)
            cv2.circle(self.outimg, (int(corners[3][0][0]), int(corners[3][0][1])), 5, (0, 0, 255), -1)

            left = [corners[0], corners[2]]
            right = [corners[1], corners[3]]

            # Get extra samples similarly to in the constructor
            avgL = [sum(x) / 2.0 for x in zip(left[0], left[1])]
            avgR = [sum(x) / 2.0 for x in zip(right[0], right[1])]
            avgHigh = [sum(x) / 2.0 for x in zip(left[0], right[0])]
            avgLow = [sum(x) / 2.0 for x in zip(left[1], right[1])]

            # Turn all the contours into an array and feed it into solvePnP
            outerCorners = np.float32(np.array((left[1], left[0], avgL, avgHigh, avgLow, avgR, right[0], right[1])))

            retval, rvec, tvec = cv2.solvePnP(self.outsideTargetCoords, outerCorners, self.camMatrix, self.distCoeffs)

            if retval:
                dist, x, z = self.computeOutputValues(rvec, tvec)
                distances.append([dist, meanTapeXPos, meanTapeYPos])
                # if idx == 0:
                jevois.sendSerial(f"RDIST {dist},{x},{z}") # Send the 3 legs of the tvec triangle to the RIO
                # jevois.sendSerial(f"OPTION {dist},{button}")
        outframe.sendCv(self.outimg)

    def computeOutputValues(self, rvec, tvec):
        """Computes the output values for solvePnP based on the returned rvec and tvec"""
        x = tvec[0][0]
        z = tvec[2][0]

        dist = math.sqrt(x ** 2 + z ** 2)

        euler_angles = self._rodrigues_to_euler(rvec)

        return dist, x, z

    def _rodrigues_to_euler(self, rvec):
        """Convert the rodrigues rvec into component euler angles in radians"""
        mat, jac = cv2.Rodrigues(rvec)
        sy = np.sqrt(mat[0, 0] * mat[0, 0] + mat[1, 0] * mat[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = np.math.atan2(mat[2, 1], mat[2, 2])
            y = np.math.atan2(-mat[2, 0], sy)
            z = np.math.atan2(mat[1, 0], mat[0, 0])
        else:
            x = np.math.atan2(-mat[1, 2], mat[1, 1])
            y = np.math.atan2(-mat[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def getCorners(self, contours, bitmask):
        contours = [x.reshape(-1, 2) for x in contours[:2]]

        tops = [min(contour, key=lambda x: x[1]) for contour in contours]
        lefts = [min(contour, key=lambda x: x[0]) for contour in contours]
        rights = [max(contour, key=lambda x: x[0]) for contour in contours]

        tops.sort(key=lambda x: x[0])
        lefts.sort(key=lambda x: x[0])
        rights.sort(key=lambda x: -x[0])

        pixel_corners = np.array(tops + lefts[:1] + rights[:1], dtype=np.float32).reshape(-1, 1, 2)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        corners_subpixel = cv2.cornerSubPix(bitmask,
                                            pixel_corners,
                                            (5, 5), (-1, -1),
                                            criteria)
        return corners_subpixel


TapeDetector.init_class_variables()

def sortContours(cnt):
    return cnt[0]
