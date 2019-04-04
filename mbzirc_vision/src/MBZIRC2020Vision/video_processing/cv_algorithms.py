import cv2


###############################
#           COLOR             #
###############################


def gaussian_blur(frame, ksize=(5,5), sigma=0):
    """ Blurs an image using a Gaussian filter

    Convolution between image and Gaussian kernel.
    Parameters
    ----------
    ksize : (int, int) odd
        Kernel size
    sigma : int, optional
        Gaussian standard deviation
    """
    return cv2.GaussianBlur(frame, ksize, sigma)


def bilateral_filter(frame, d=9, sigmacolor=75, sigmaspace=10):
    """ Applies the bilateral filter to an image

    Smoothing with edge-preservation.
    Parameters
    ----------
    d : int
        Neighborhood diameter.
        Note: flattens color, but also makes transparent. More than 10 is heavy computationally.
    sigmacolor : int
        Filter sigma in color space. Higher value mixes farther colors together
    sigmaspace: int
        Filter sigma in coord space. Higher values make farther points interact more.
        Note: not so relevant in practice
    """

    return cv2.bilateralFilter(frame, d, sigmacolor, sigmaspace)


def pyramid_filter(frame, sp=10, sr=30):
    """ Meanshift segmentation

    Parameters:
    sp : int
        Spatial activation radius
    sr : int
        Color activation threshold
    """

    return cv2.pyrMeanShiftFiltering(frame, sp, sr)


###############################
#           COLOR             #
###############################


def bgr_to_hsv(frame):
    """ Convert a BGR frame into HSV """

    return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


def color_filter(frame, lb1, ub1, lb2=None, ub2=None):
    """ Filter by HSV color based on the specified thresholds.
    
    Input frame should be already in HSV format.
    """

    if lb2 is None or ub2 is None:
        return cv2.inRange(frame, lb1, ub1)
    else:
        mask1 = cv2.inRange(frame, lb1, ub1)
        mask2 = cv2.inRange(frame, lb2, ub2)
        mask = cv2.bitwise_or(mask1, mask2)
        return mask


def find_contours(mask):
    """ Detect contours on a binary image """

    _, cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return cnts