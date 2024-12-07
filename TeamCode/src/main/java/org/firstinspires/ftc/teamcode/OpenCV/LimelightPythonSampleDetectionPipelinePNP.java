/*
def analyzeContour(contour, img, color):
    rect = cv2.minAreaRect(contour)
    width, height = rect[1]

    # Filter based on aspect ratio
    aspect_ratio = max(width, height) / (min(width, height) + 1e-5)  # Avoid division by zero
    if aspect_ratio > 5.0 or aspect_ratio < 0.2:  # Reject extreme aspect ratios
        return None, None, None

    # Draw and tag the rectangle
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

    # Analyze contours with filtering
    for contour in contours_blue + contours_red + contours_yellow:
        area = cv2.contourArea(contour)
        if area > 100:  # Filter small contours
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