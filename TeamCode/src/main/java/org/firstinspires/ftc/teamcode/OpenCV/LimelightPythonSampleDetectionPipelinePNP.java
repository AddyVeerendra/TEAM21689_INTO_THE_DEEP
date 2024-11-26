/*
import cv2
import numpy as np

# Constants
YELLOW_MASK_THRESHOLD = 57
BLUE_MASK_THRESHOLD = 150
RED_MASK_THRESHOLD = 198
CONTOUR_LINE_THICKNESS = 2

# Colors
RED = (0, 0, 255)  # OpenCV uses BGR
BLUE = (255, 0, 0)
YELLOW = (0, 255, 255)

# Camera Calibration Parameters (CHANGE FOR LIMELIGHT)
cameraMatrix = np.array([
    [822.317, 0, 319.495],
    [0, 822.317, 242.502],
    [0, 0, 1]
], dtype=np.float64)

distCoeffs = np.array([0, 0, 0, 0, 0], dtype=np.float64)

# Morphological operation elements
erodeElement = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
dilateElement = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))

def morphMask(input_mask):
    output = cv2.erode(input_mask, erodeElement)
    output = cv2.erode(output, erodeElement)
    output = cv2.dilate(output, dilateElement)
    output = cv2.dilate(output, dilateElement)
    return output

def drawRotatedRect(img, rect, color):
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(img, [box], 0, color, CONTOUR_LINE_THICKNESS)

def drawTagText(img, rect, text, color):
    font = cv2.FONT_HERSHEY_PLAIN
    cv2.putText(img, text,
                (int(rect[0][0] - 50), int(rect[0][1] + 25)),
                font, 1, color, 1, cv2.LINE_AA)

def drawAxis(img, rvec, tvec):
    axisLength = 50
    imgpts, _ = cv2.projectPoints(np.float32([[0,0,0], [axisLength,0,0], [0,axisLength,0], [0,0,-axisLength]]),
                                  rvec, tvec, cameraMatrix, distCoeffs)
    imgpts = imgpts.astype(int)
    img = cv2.line(img, tuple(imgpts[0].ravel()), tuple(imgpts[1].ravel()), (0,0,255), 3)
    img = cv2.line(img, tuple(imgpts[0].ravel()), tuple(imgpts[2].ravel()), (0,255,0), 3)
    img = cv2.line(img, tuple(imgpts[0].ravel()), tuple(imgpts[3].ravel()), (255,0,0), 3)
    return img

def analyzeContour(contour, img, color):
    rect = cv2.minAreaRect(contour)
    drawRotatedRect(img, rect, getColorScalar(color))

    angle = rect[2]
    if rect[1][0] < rect[1][1]:
        angle += 90

    angle = -(angle - 180)
    drawTagText(img, rect, f"{int(round(angle))} deg", getColorScalar(color))

    # Prepare for solvePnP
    objectPoints = np.array([[-5, -2.5, 0], [5, -2.5, 0], [5, 2.5, 0], [-5, 2.5, 0]], dtype=np.float32)
    imagePoints = cv2.boxPoints(rect)

    success, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs)

    if success:
        img = drawAxis(img, rvec, tvec)

    return angle, rvec, tvec if success else None

def getColorScalar(color):
    return {
        "Blue": BLUE,
        "Yellow": YELLOW,
        "Red": RED
    }.get(color, RED)

def runPipeline(image, llrobot):
    # Resize image for faster processing
    height, width = image.shape[:2]
    resized = cv2.resize(image, (width // 2, height // 2))

    # Convert to YCrCb color space
    ycrcb = cv2.cvtColor(resized, cv2.COLOR_BGR2YCrCb)

    # Extract Cr and Cb channels
    _, cr, cb = cv2.split(ycrcb)

    # Apply Gaussian blur
    cb = cv2.GaussianBlur(cb, (5, 5), 0)
    cr = cv2.GaussianBlur(cr, (5, 5), 0)

    # Thresholding
    _, blue_mask = cv2.threshold(cb, BLUE_MASK_THRESHOLD, 255, cv2.THRESH_BINARY)
    _, red_mask = cv2.threshold(cr, RED_MASK_THRESHOLD, 255, cv2.THRESH_BINARY)
    _, yellow_mask = cv2.threshold(cb, YELLOW_MASK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

    # Apply morphological operations
    blue_mask = morphMask(blue_mask)
    red_mask = morphMask(red_mask)
    yellow_mask = morphMask(yellow_mask)

    # Find contours
    contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_yellow, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    analyzed_stones = []

    # Analyze contours
    for contour in contours_blue + contours_red + contours_yellow:
        if cv2.contourArea(contour) > 100:  # Filter small contours
            color = "Blue" if contour in contours_blue else "Red" if contour in contours_red else "Yellow"
            angle, rvec, tvec = analyzeContour(contour, resized, color)
            if tvec is not None:
                analyzed_stones.append({"angle": angle, "color": color, "rvec": rvec, "tvec": tvec})

    # Resize the processed image back to original size for output
    output = cv2.resize(resized, (width, height))

    # Prepare llpython output (up to 8 values)
    llpython = [len(analyzed_stones)] + [0] * 7

    return np.array([]), output, llpython
 */