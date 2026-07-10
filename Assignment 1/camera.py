import sensor, time, math

class Cam(object):
    """
    The Cam class manages the camera sensor for image capturing, processing,
    and color tracking. It initializes the camera parameters and sets the color
    thresholds for blob detection.
    """

    def __init__(self, thresholds, gain=25):
        """
        Initialise the Cam object by setting up camera parameters and
        configuring color thresholds.

        Args:
            thresholds (list): List of LAB colour range tuples for blob detection.
            gain (float): Camera gain value. Defaults to 25.
        """
        # Configure camera settings
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)        # Set frame to VGA 640x480, or QVGA for higher fps
        sensor.skip_frames(time=2000)           # Allow the camera to adjust to light levels

        # Both must be turned off for color tracking
        sensor.set_auto_gain(False, gain_db=gain)
        sensor.set_auto_whitebal(False)

        # Initialise sensor properties
        self.w_centre = sensor.width() / 2
        self.h_centre = sensor.height() / 2
        self.h_fov = 31.5
        self.v_fov = 21
        self.camera_elevation_angle = -11.5     # Can measure and adjust this value
        self.clock = time.clock()

        # Define color tracking thresholds for blob detection
        # Thresholds are in the order of (L Min, L Max, A Min, A Max, B Min, B Max)
        self.thresholds = thresholds


    def rotate_blob_coords(self, cx, cy, angle_deg):
        """
        Rotate blob centre coordinates around the image centre.
        This avoids rotating the entire image — much more efficient.

        Args:
            cx (float): Blob centre x coordinate.
            cy (float): Blob centre y coordinate.
            angle_deg (float): Rotation angle in degrees (z/pan rotation).

        Returns:
            (rx, ry): Rotated (x, y) coordinates as a tuple of floats.
        """
        angle_rad = math.radians(angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        # Translate to origin (relative to image centre)
        dx = cx - self.w_centre
        dy = cy - self.h_centre

        # Apply 2D rotation matrix
        rx = cos_a * dx - sin_a * dy + self.w_centre
        ry = sin_a * dx + cos_a * dy + self.h_centre

        return rx, ry


    def get_blobs(self, angle=0) -> tuple:
        """
        Capture an image and detect color blobs based on predefined thresholds.
        Blob coordinates are corrected for periscope rotation without rotating
        the entire image, which is significantly more efficient.

        Args:
            angle (float): Pan angle for blob coordinate rotation correction. Defaults to 0.
        Returns:
            blobs (list): List of detected blobs (raw, unrotated).
            img (image): Captured image (never rotated — for robot operation only).
            rotated_centres (list): List of (rx, ry) corrected blob centres.
        """
        img = sensor.snapshot()

        # Find blobs on raw unrotated image — always fast
        blobs = img.find_blobs(
            self.thresholds,
            pixels_threshold=60,
            area_threshold=60
        )

        # Rotate only the blob coordinates — never the image
        rotated_centres = [
            self.rotate_blob_coords(b.cx, b.cy, angle) for b in blobs
        ]

        return blobs, img, rotated_centres


    def get_blobs_bottom(self, angle=0) -> tuple:
        """
        Capture an image and detect colour blobs based on predefined thresholds.
        Region of interest is set to the bottom 2/3 of the image.
        Blob coordinates are corrected for periscope rotation without rotating
        the entire image.

        Args:
            angle (float): Pan angle for blob coordinate rotation correction. Defaults to 0.
        Returns:
            blobs (list): List of detected blobs (raw, unrotated).
            img (image): Captured image (never rotated — for robot operation only).
            rotated_centres (list): List of (rx, ry) corrected blob centres.
        """
        img = sensor.snapshot()

        # Find blobs on raw unrotated image — always fast
        blobs = img.find_blobs(
            self.thresholds,
            pixels_threshold=150,
            area_threshold=150,
            roi=(
                1,
                int(sensor.height() / 3),
                int(sensor.width()),
                int(2 * sensor.height() / 3)
            )
        )

        # Rotate only the blob coordinates — never the image
        rotated_centres = [
            self.rotate_blob_coords(b.cx, b.cy, angle) for b in blobs
        ]

        return blobs, img, rotated_centres


    def get_biggest_blob(self, blobs):
        """
        Identify and return the largest blob from a list of detected blobs.

        Args:
            blobs (list): List of detected blobs.

        Returns:
            big_blob (blob): The biggest blob from list - see OpenMV docs for blob class.
        """
        max_pixels = 0
        big_blob = None

        for blob in blobs:
            # Update the big blob if the current blob has more pixels
            if blob.pixels > max_pixels:
                max_pixels = blob.pixels
                big_blob = blob

        return big_blob


    def get_blob_colours(self, blobs) -> list:
        """
        Returns the binary code (as int) of thresholds met by each blob.

        Args:
            blobs (list): List of detected blobs.

        Returns:
            colours (list): List of binary codes (as int) of thresholds met by each element in blobs.
        """
        colours = []

        for blob in blobs:
            colours.append(blob[8])

        return colours

    def find_blob(self, blobs, threshold_idx: int):
        """
        Finds the first blob in blobs that was detected using a specified threshold.

        Args:
            blobs (list): List of detected blobs.
            threshold_idx (int): Index along self.thresholds.

        Returns:
            found_idx (int): Index along blobs for the first blob that was detected
                             using self.thresholds[threshold_idx]. Returns None if not found.
        """
        colours = self.get_blob_colours(blobs)

        for found_idx, colour in enumerate(colours):
            if colour == pow(2, threshold_idx):
                return found_idx

        return None


if __name__ == "__main__":
    #
    # Blob threshold tester / visual sanity check
    #
    # Run this file directly in OpenMV IDE to test colour thresholds and
    # verify blob coordinate rotation without affecting robot operation.
    #

    import sensor, math

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA)
    sensor.skip_frames(time=2000)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_gain(False, gain_db=20)

    # Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
    thresholds = [
        (30, 80, -28, 0, 0, 30),  # Orange
    ]

    angle = 0                   # Set pan angle for rotation correction
    rotate_image = False        # Toggle for visual sanity check:
                        #   True  — rotates image so you can verify blob crosses align
                        #   False — raw image displayed, blob crosses still at corrected positions

    w_centre = sensor.width() / 2
    h_centre = sensor.height() / 2

    while True:
        img = sensor.snapshot()

        # Find blobs on raw unrotated image first
        blobs = img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150)

        angle_rad = math.radians(angle)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        for blob in blobs:
            # Rotate only the blob centre coordinates
            dx = blob.cx - w_centre
            dy = blob.cy - h_centre
            rx = int(cos_a * dx - sin_a * dy + w_centre)
            ry = int(sin_a * dx + cos_a * dy + h_centre)

            img.draw_rectangle(blob.rect)     # Raw bounding box (pre-rotation reference)
            img.draw_cross((rx, ry))            # Corrected centre after coordinate rotation

        # Rotate image AFTER blob detection and coordinate correction
        # Purely for visual verification — confirm rx,ry cross aligns with
        # the blob in the rotated image. Has no effect on robot operation.
        if rotate_image:
            img.rotation_corr(z_rotation=angle)
