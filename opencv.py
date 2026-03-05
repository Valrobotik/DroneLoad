import cv2

class OpenCV:
    def __init__(slef):

    def Aruco(self, frame, fps):
        self.frame = frame
        self.fps = fps

        cv2.putText(frame, f"Pi4 - Detection Active - FPS {fps}",
                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)