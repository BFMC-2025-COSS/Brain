import numpy as np
import cv2

def flatten_perspective(image):
    """
    Warps the image from the vehicle front-facing camera mapping hte road to a bird view perspective.

    Parameters
    ----------
    image       : Image from the vehicle front-facing camera.

    Returns
    -------
    Warped image.
    """
    # Get image dimensions
    (h, w) = (image.shape[0], image.shape[1])
    # Define source points
    # source = np.float32([[w // 2 - 80 , h * .25], [w // 2 + 80, h * .25], [w // 2 - 200, h * .85], [w // 2 + 200, h * .85]])
    source = np.float32([[w // 2 - 75, h * .45], [w // 2 + 75, h * .45], [w // 2 - 145, h * .85], [w // 2 + 145, h * .85]])
    ## 3rd report
    # source = np.float32([[w // 2 - 45 , h * .3], [w // 2 + 45, h * .3], [w // 2 - 145, h * .85], [w // 2 + 145, h * .85]])

    #source = np.float32([[330-70 , 340], [670+70, 340], [0, 540], [960, 540]])
    # Define corresponding destination points
    destination = np.float32([[100, 0], [w - 100, 0], [100, h], [w - 100, h]])
    transform_matrix = cv2.getPerspectiveTransform(source, destination)
    unwarp_matrix = cv2.getPerspectiveTransform(destination, source)
    return (cv2.warpPerspective(image, transform_matrix, (w, h)), unwarp_matrix)

# Test the function
# img = "/home/seame/mask1.jpg"
# frame = cv2.imread(img)
# print("Ready for BEV")
# flattened = flatten_perspective(frame)
# print("BEV done")
# cv2.imshow("flattened", flattened[0])
# cv2.waitKey(0)
# cv2.destroyAllWindows()
