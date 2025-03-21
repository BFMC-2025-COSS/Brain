import numpy as np
import cv2


def gradient_abs_value_mask(image, sobel_kernel=3, axis='x', threshold=(0, 255)):
    """
    Masks the image based on gradient absolute value.

    Parameters
    ----------
    image           : Image to mask.
    sobel_kernel    : Kernel of the Sobel gradient operation.
    axis            : Axis of the gradient, 'x' or 'y'.
    threshold       : Value threshold for it to make it to appear in the mask.

    Returns
    -------
    Image mask with 1s in activations and 0 in other pixels.
    """
    # Take the absolute value of derivative in x or y given orient = 'x' or 'y'
    if axis == 'x':
        sobel = np.absolute(cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=sobel_kernel))
    if axis == 'y':
        sobel = np.absolute(cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=sobel_kernel))
    # Scale to 8-bit (0 - 255) then convert to type = np.uint8
    sobel = np.uint8(255 * sobel / np.max(sobel))
    # Create a mask of 1's where the scaled gradient magnitude is > thresh_min and < thresh_max
    mask = np.zeros_like(sobel)
    # Return this mask as your binary_output image
    mask[(sobel >= threshold[0]) & (sobel <= threshold[1])] = 1
    return mask

def gradient_magnitude_mask(image, sobel_kernel=3, threshold=(0, 255)):
    """
    Masks the image based on gradient magnitude.

    Parameters
    ----------
    image           : Image to mask.
    sobel_kernel    : Kernel of the Sobel gradient operation.
    threshold       : Magnitude threshold for it to make it to appear in the mask.

    Returns
    -------
    Image mask with 1s in activations and 0 in other pixels.
    """
    # Take the gradient in x and y separately
    sobel_x = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobel_y = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # Calculate the magnitude
    magnitude = np.sqrt(sobel_x ** 2 + sobel_y ** 2)
    # Scale to 8-bit (0 - 255) and convert to type = np.uint8
    magnitude = (magnitude * 255 / np.max(magnitude)).astype(np.uint8)
    # Create a binary mask where mag thresholds are met
    mask = np.zeros_like(magnitude)
    mask[(magnitude >= threshold[0]) & (magnitude <= threshold[1])] = 1
    # Return this mask as your binary_output image
    return mask

def gradient_direction_mask(image, sobel_kernel=3, threshold=(0, np.pi / 2)):
    """
    Masks the image based on gradient direction.

    Parameters
    ----------
    image           : Image to mask.
    sobel_kernel    : Kernel of the Sobel gradient operation.
    threshold       : Direction threshold for it to make it to appear in the mask.

    Returns
    -------
    Image mask with 1s in activations and 0 in other pixels.
    """
    # Take the gradient in x and y separately
    sobel_x = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobel_y = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # Take the absolute value of the x and y gradients and calculate the direction of the gradient
    direction = np.arctan2(np.absolute(sobel_y), np.absolute(sobel_x))
    # Create a binary mask where direction thresholds are met
    mask = np.zeros_like(direction)
    # Return this mask as your binary_output image
    mask[(direction >= threshold[0]) & (direction <= threshold[1])] = 1
    return mask

def color_threshold_mask(image, threshold=(0, 255)):
    """
    Masks the image based on color intensity.

    Parameters
    ----------
    image           : Image to mask.
    threshold       : Color intensity threshold.

    Returns
    -------
    Image mask with 1s in activations and 0 in other pixels.
    """
    mask = np.zeros_like(image)
    mask[(image > threshold[0]) & (image <= threshold[1])] = 1
    return mask

def get_edges(image, separate_channels=False):
    """
    Masks the image based on a composition of edge detectors: gradient value,
    gradient magnitude, gradient direction and color.

    Parameters
    ----------
    image               : Image to mask.
    separate_channels   : Flag indicating if we need to put masks in different color channels.

    Returns
    -------
    Image mask with 1s in activations and 0 in other pixels.
    """
    # Convert to HLS color space and separate required channel
    hls = cv2.cvtColor(np.copy(image), cv2.COLOR_RGB2HLS).astype(np.float32)
    s_channel = hls[:, :, 1]
    # Get a combination of all gradient thresholding masks
    gradient_x = gradient_abs_value_mask(s_channel, axis='x', sobel_kernel=3, threshold=(20, 100))
    gradient_y = gradient_abs_value_mask(s_channel, axis='y', sobel_kernel=3, threshold=(20, 100))
    magnitude = gradient_magnitude_mask(s_channel, sobel_kernel=3, threshold=(20, 100))
    direction = gradient_direction_mask(s_channel, sobel_kernel=3, threshold=(0.7, 1.3))
    gradient_mask = np.zeros_like(s_channel)

    #More Clean
    gradient_mask[((gradient_x == 1) & (magnitude == 1) & (direction == 1)) | ((gradient_x == 1) & (gradient_y == 1))] = 1
    
    #gradient_mask[((gradient_x == 1) & (gradient_y == 1)) | ((magnitude == 1) & (direction == 1))] = 1
    
    # Get a color thresholding mask
    color_mask = color_threshold_mask(s_channel, threshold=(200, 255))

    if separate_channels:
        return np.dstack((np.zeros_like(s_channel), gradient_mask, color_mask))
    else:
        mask = np.zeros_like(gradient_mask)
        mask[(gradient_mask == 1) | (color_mask == 1)] = 1
        return mask

def optimized_get_edges(image):
    # Convert to HLS color space and separate the S channel
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS).astype(np.float32)
    s_channel = hls[:, :, 1]

    # Downsample the S channel to reduce computation
    s_channel_downsampled = cv2.pyrDown(s_channel)

    # Apply Sobel operations only once and reuse results
    sobel_x = cv2.Sobel(s_channel_downsampled, cv2.CV_64F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(s_channel_downsampled, cv2.CV_64F, 0, 1, ksize=3)

    # Compute gradient magnitude and direction
    gradient_magnitude = np.sqrt(sobel_x**2 + sobel_y**2).astype(np.uint8)
    gradient_direction = cv2.phase(sobel_x, sobel_y, angleInDegrees=False)

    # Create binary masks based on thresholds
    gradient_x_mask = (np.abs(sobel_x) > 20).astype(np.uint8)
    gradient_y_mask = (np.abs(sobel_y) > 20).astype(np.uint8)
    magnitude_mask = (gradient_magnitude > 20).astype(np.uint8)
    direction_mask = ((gradient_direction > 0.7) & (gradient_direction < 1.3)).astype(np.uint8)

    # Combine all masks
    combined_gradient_mask = cv2.bitwise_or(
        cv2.bitwise_and(gradient_x_mask, gradient_y_mask),
        cv2.bitwise_and(gradient_x_mask, magnitude_mask, direction_mask)
    )

    # Color threshold mask
    color_mask = (s_channel_downsampled > 200).astype(np.uint8)

    # Combine gradient and color masks
    final_mask = cv2.bitwise_or(combined_gradient_mask, color_mask)

    # Upsample back to original resolution
    final_mask_upsampled = cv2.pyrUp(final_mask).astype(np.uint8)

    return final_mask_upsampled
