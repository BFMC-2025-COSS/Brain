import numpy as np
import cv2
from concurrent.futures import ThreadPoolExecutor
from src.utils.lantracker_pi.window import Window
from src.utils.lantracker_pi.line import Line
from src.utils.lantracker_pi.gradients import get_edges, optimized_get_edges
from src.utils.lantracker_pi.perspective import flatten_perspective
import time
# from window import Window
# from line import Line
# from gradients import get_edges
# from perspective import flatten_perspective

class LaneTracker(object):
    """
    Tracks the lane in a series of consecutive frames.
    """

    def __init__(self, first_frame, mask, n_windows=15):
        """
        Initializes a tracker object.

        Parameters
        ----------
        first_frame     : First frame of the frame series. We use it to get dimensions and initialise values.
        n_windows       : Number of windows we use to track each lane edge.
        """
        (self.h, self.w, _) = first_frame.shape
        self.win_n = n_windows
        self.left = None
        self.right = None
        self.l_windows = []
        self.r_windows = []
        self.initialize_lines(mask)

    def initialize_lines(self, flat_edges):
        """
        Finds starting points for left and right lines (e.g. lane edges) and initialises Window and Line objects.

        Parameters
        ----------
        frame   : Frame to scan for lane edges.
        """
        # Take a histogram of the bottom half of the image
        # edges = optimized_get_edges(frame)
        # (flat_edges, _) = flatten_perspective(edges)

        histogram = np.sum(flat_edges[int(self.h / 2):, :], axis=0)

        nonzero = flat_edges.nonzero()
        # Create empty lists to receive left and right lane pixel indices
        l_indices = np.empty([0], dtype=np.int32)
        r_indices = np.empty([0], dtype=np.int32)
        window_height = int(self.h / self.win_n)

        l_median_centers = []
        r_median_centers = []

        for i in range(self.win_n):
            l_window = Window(
                y1=self.h - (i + 1) * window_height,
                y2=self.h - i * window_height,
                m = 40 if len(self.l_windows) > 0 else 70,
                x=self.l_windows[-1].x if len(self.l_windows) > 0 else np.argmax(histogram[:self.w // 2])
            )
            r_window = Window(
                y1=self.h - (i + 1) * window_height,
                y2=self.h - i * window_height,
                m = 40 if len(self.l_windows) > 0 else 70,
                x=self.r_windows[-1].x if len(self.r_windows) > 0 else np.argmax(histogram[self.w // 2:]) + self.w // 2
            )

            l_pixel_indices = l_window.pixels_in(nonzero)
            r_pixel_indices = r_window.pixels_in(nonzero)

            l_x_pixels = nonzero[1][l_pixel_indices]
            r_x_pixels = nonzero[1][r_pixel_indices]
            l_y_pixels = nonzero[0][l_pixel_indices]
            r_y_pixels = nonzero[0][r_pixel_indices]

            if len(l_x_pixels) > 0 and len(l_y_pixels) > 0:
                l_median_x = np.mean(l_x_pixels)
                l_median_y = np.mean(l_y_pixels)
                l_median_centers.append((l_median_x, l_median_y))

            if len(r_x_pixels) > 0 and len(r_y_pixels) > 0:
                r_median_x = np.mean(r_x_pixels)
                r_median_y = np.mean(r_y_pixels)
                r_median_centers.append((r_median_x, r_median_y))

            l_indices = np.append(l_indices, l_pixel_indices, axis=0)
            r_indices = np.append(r_indices, r_pixel_indices, axis=0)

            self.l_windows.append(l_window)
            self.r_windows.append(r_window)

        l_median_xs = np.array([pt[0] for pt in l_median_centers])
        l_median_ys = np.array([pt[1] for pt in l_median_centers])
        r_median_xs = np.array([pt[0] for pt in r_median_centers])
        r_median_ys = np.array([pt[1] for pt in r_median_centers])

        print("Left Line")
        self.left = Line(x=l_median_xs, y=l_median_ys, h=self.h, w=self.w)
        print("Right Line")
        self.right = Line(x=r_median_xs, y=r_median_ys, h=self.h, w=self.w)

        # for i in range(self.win_n):
        #     l_window = Window(
        #         y1=self.h - (i + 1) * window_height,
        #         y2=self.h - i * window_height,
        #         m = 40 if len(self.l_windows) > 0 else 70,
        #         x=self.l_windows[-1].x if len(self.l_windows) > 0 else np.argmax(histogram[:self.w // 2])
        #     )
        #     r_window = Window(
        #         y1=self.h - (i + 1) * window_height,
        #         y2=self.h - i * window_height,
        #         m = 40 if len(self.l_windows) > 0 else 70,
        #         x=self.r_windows[-1].x if len(self.r_windows) > 0 else np.argmax(histogram[self.w // 2:]) + self.w // 2
        #     )
        #     # Append nonzero indices in the window boundary to the lists
        #     l_indices = np.append(l_indices, l_window.pixels_in(nonzero), axis=0)
        #     r_indices = np.append(r_indices, r_window.pixels_in(nonzero), axis=0)
        #     self.l_windows.append(l_window)
        #     self.r_windows.append(r_window)
        # self.left = Line(x=nonzero[1][l_indices], y=nonzero[0][l_indices], h=self.h, w = self.w)
        # self.right = Line(x=nonzero[1][r_indices], y=nonzero[0][r_indices], h=self.h, w = self.w)

    def scan_frame_with_windows(self, frame, windows):
        """
        Scans a frame using initialised windows in an attempt to track the lane edges.

        Parameters
        ----------
        frame   : New frame
        windows : Array of windows to use for scanning the frame.

        Returns
        -------
        A tuple of arrays containing coordinates of points found in the specified windows.
        """
        indices = np.empty([0], dtype=np.int32)
        nonzero = frame.nonzero()
        window_x = None
        for window in windows:
            indices = np.append(indices, window.pixels_in(nonzero, window_x), axis=0)
            window_x = window.mean_x
        return (nonzero[1][indices], nonzero[0][indices])

    def predict_missing_lane(self, visible_lane, lane_width, direction="right"):
        """
        Predicts the missing lane using the visible lane and the lane width.

        Parameters
        ----------
        visible_lane : Line
            The visible lane (left or right).
        lane_width : int
            The distance (in pixels) between the two lanes.
        direction : str
            "left" or "right", indicating which lane is missing.

        Returns
        -------
        np.ndarray
            Predicted points for the missing lane.
        """
        visible_points = visible_lane.get_points()  
        if direction == "right":
            predicted_points = visible_points.copy()
            predicted_points[:, 0] += lane_width  
        elif direction == "left":
            predicted_points = visible_points.copy()
            predicted_points[:, 0] -= lane_width  
        return predicted_points

    def update_missing_windows(self, missing_windows, reference_windows, direction, lane_width=300):
        for missing_window, reference_window in zip(missing_windows, reference_windows):
            if direction == "left":
                missing_window.x = reference_window.x - lane_width
            elif direction == "right":
                missing_window.x = reference_window.x + lane_width


    def process(self, frame, flat_edges, unwarp_matrix, draw_lane=True, draw_statistics=True):
        """
        Performs a full lane tracking pipeline on a frame.

        Parameters
        ----------
        frame               : New frame to process.
        draw_lane           : Flag indicating if we need to draw the lane on top of the frame.
        draw_statistics     : Flag indicating if we need to render the debug information on top of the frame.

        Returns
        -------
        Resulting frame.
        """

        (l_x, l_y) = self.scan_frame_with_windows(flat_edges, self.l_windows)
        (r_x, r_y) = self.scan_frame_with_windows(flat_edges, self.r_windows)


        # print("l: ",len(l_x),"r: ", len(r_x))
        left_visible = len(l_x) > 3500
        right_visible = len(r_x) > 3500

        if not left_visible and right_visible:
            self.update_missing_windows(self.l_windows, self.r_windows, direction="left", lane_width=300)

            predicted_left_points = self.predict_missing_lane(self.right, lane_width=300, direction="left")
            self.left.process_points(predicted_left_points[:, 0], predicted_left_points[:, 1])
            self.right.process_points(r_x, r_y)

        elif not right_visible and left_visible:
            self.update_missing_windows(self.r_windows, self.l_windows, direction="right", lane_width=300)

            predicted_right_points = self.predict_missing_lane(self.left, lane_width=300, direction="right")
            self.right.process_points(predicted_right_points[:, 0], predicted_right_points[:, 1])
            self.left.process_points(l_x, l_y)

        else:
            self.left.process_points(l_x, l_y)
            self.right.process_points(r_x, r_y)

        offset, curvature = self.calculate_metrics(flat_edges.shape)
        if draw_statistics:
            debug_overlay = self.draw_debug_overlay(flat_edges)
            cv2.imshow("Debug",debug_overlay)
            key = cv2.waitKey(1)

            # top_overlay = self.draw_lane_overlay(flat_edges)
            # lane_center = int((np.mean(self.left.get_points()[:,0]) + np.mean(self.right.get_points()[:,0]))/2)
            # frame_center = top_overlay.shape[1] // 2
            # cv2.circle(top_overlay, (lane_center, 0), 3, (0, 0, 255), -1)
            # cv2.circle(top_overlay, (frame_center, 0), 3, (255, 0, 0), -1)
            # cv2.imshow("Top", top_overlay)
            # key = cv2.waitKey(1)

        if draw_lane:
            frame = self.draw_lane_overlay(frame, unwarp_matrix)

        return frame, offset, curvature

    
    def calculate_metrics(self, frame_shape):
        # Calculate lane curvature
        curvature = self.radius_of_curvature()

        # Calculate lane center offset
        # Using only last window x pixels
        #lane_center = (self.left.get_points()[0][0] + self.right.get_points()[0][0]) / 2
        
        # Using all window's x pixels
        lane_center = (np.mean(self.left.get_points()[:, 0]) + np.mean(self.right.get_points()[:, 0])) / 2
        
        # Using weights average(more weights on near lane)
        #weights = np.linspace(1, 0, len(self.left.get_points()))
        #lane_center = (np.average(self.left.get_points()[:, 0], weights=weights) + np.average(self.right.get_points()[:, 0], weights=weights)) / 2
        
        frame_center = frame_shape[1] // 2
        offset = (frame_center - lane_center) * 35 / 352  # Convert to cm in BFMC Format(35cm, 960x540=352pixels)
        return offset, curvature



    def draw_text(self, frame, text, x, y):
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 2)

    def draw_debug_overlay(self, binary, lines=True, windows=True):
        """
        Draws an overlay with debugging information on a bird's-eye view of the road (e.g. after applying perspective
        transform).

        Parameters
        ----------
        binary  : Frame to overlay.
        lines   : Flag indicating if we need to draw lines.
        windows : Flag indicating if we need to draw windows.

        Returns
        -------
        Frame with an debug information overlay.
        """
        if len(binary.shape) == 2:
            image = np.dstack((binary, binary, binary))
        else:
            image = binary
        if windows:
            for window in self.l_windows:
                coordinates = window.coordinates()
                cv2.rectangle(image, coordinates[0], coordinates[1], (1., 1., 0), 2)
            for window in self.r_windows:
                coordinates = window.coordinates()
                cv2.rectangle(image, coordinates[0], coordinates[1], (1., 1., 0), 2)
        if lines:
            cv2.polylines(image, [self.left.get_points()], False, (1., 0, 0), 2)
            cv2.polylines(image, [self.right.get_points()], False, (1., 0, 0), 2)
        return image * 255

    def draw_lane_overlay(self, image, unwarp_matrix=None):
        """
        Draws an overlay with tracked lane applying perspective unwarp to project it on the original frame.

        Parameters
        ----------
        image           : Original frame.
        unwarp_matrix   : Transformation matrix to unwarp the bird's eye view to initial frame. Defaults to `None` (in
        which case no unwarping is applied).

        Returns
        -------
        Frame with a lane overlay.
        """
        # Create an image to draw the lines on
        overlay = np.zeros_like(image).astype(np.uint8)
        points = np.vstack((self.left.get_points(), np.flipud(self.right.get_points())))
        # Draw the lane onto the warped blank image
        cv2.fillPoly(overlay, [points], (0, 255, 0))
        if unwarp_matrix is not None:
            # Warp the blank back to original image space using inverse perspective matrix (Minv)
            overlay = cv2.warpPerspective(overlay, unwarp_matrix, (image.shape[1], image.shape[0]))
        # Combine the result with the original image
        return cv2.addWeighted(image, 1, overlay, 0.3, 0)

    def radius_of_curvature(self):
        """
        Calculates radius of the lane curvature by averaging curvature of the edge lines.

        Returns
        -------
        Radius of the lane curvature in meters.
        """
        return int(np.average([self.left.radius_of_curvature(), self.right.radius_of_curvature()]))
