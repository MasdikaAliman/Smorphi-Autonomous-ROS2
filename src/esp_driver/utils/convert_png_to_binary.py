import cv2  # Import OpenCV

# read the image file
img = cv2.imread('../map/floor_plans/floor.png')

ret, bw_img = cv2.threshold(img, 220, 255, cv2.THRESH_BINARY)

# converting to its binary form
bw = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)

cv2.imwrite("../map/floor_plans/floor_binary.png", bw_img)
