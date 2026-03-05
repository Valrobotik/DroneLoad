import cv2
import sys

class VideoGStreamer:
    def __init__(self, width=1280, height=720, fps_input=30, colors_input=False,stream='N', fps_output=30, colors_output=True):
        self.width = width
        self.height = height
        self.fps_input = fps_input
        self.colors_input = colors_input
        self.stream = stream
        self.fps_output = fps_output
        self.colors_output = colors_output

        self.port = 5000

        if self.stream == 'N' or self.stream=='n':
            self.ip_dest = "0.0.0.0"
            self.streaming_active = False
        else:
            self.streaming_active = True
            self.ip_dest = input("Type IP (or 1 for 192.168.2.9) : ")
            if self.ip_dest == "1":
                self.ip_dest = "192.168.2.9"


        #Camera raspberry :
        f"""
        gst_in = (
            f"libcamerasrc ! "
            f"video/x-raw,width={self.width},height={self.height},framerate={self.fps_input}/1 ! " #format=GRAY8 #niveaux de gris (ici si camera isc)
            f"videoconvert ! " # ou v4l2convert
            f"video/x-raw,format=BGR ! "        #format=GRAY8 #niveaux de gris
            f"appsink drop=true"
        )
        """
        gst_in = (
            f"v4l2src device=/dev/video0 ! "
            f"image/jpeg,width={self.width},height={self.height},framerate={self.fps_input}/1 ! "
            f"jpegdec ! "        #v4l2jpegdec #à tester, version matérielle
            f"videoconvert ! video/x-raw,format=GRAY8 ! appsink drop=true"
        )
        self.out = None
        if self.streaming_active:
            gst_out = (
                f"appsrc ! "
                f"video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps_output}/1 ! "  # format=GRAY8 #niveaux de gris (réduire le bitrate)
                f"videoconvert ! "  #v4l2convert encodeur matériel (moins cpu mais moins fps)
                f"video/x-raw,format=I420 ! "
                f"v4l2h264enc extra-controls=\"controls,h264_profile=4,h264_level=13,video_bitrate=4000000,h264_i_frame_period=15\" ! "
                f"video/x-h264,level=(string)4,profile=high,stream-format=byte-stream ! "  # On force le caps-filter qui marche dans ton terminal
                f"h264parse ! "  # Indispensable pour stabiliser le flux matériel
                f"rtph264pay config-interval=1 pt=96 aggregate-mode=none ! "
                f"udpsink host={self.ip_dest} port={self.port} sync=false async=false"
            )
            self.out = cv2.VideoWriter(gst_out, cv2.CAP_GSTREAMER, 0, self.fps_output, (self.width, self.height), True) #False : niveaux de gris

            if not self.out.isOpened():
                print("Erreur : Impossible d'ouvrir le pipeline de sortie")
                sys.exit()
            else:
                print(f"Streaming vers {self.ip_dest}:{self.port} en {self.height}p... \n\nctrl+C to stop")

        self.cap = cv2.VideoCapture(gst_in, cv2.CAP_GSTREAMER)
        """
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2) #non usb : cap = cv2.VideoCapture(0)    (pas sur de fonctionnner)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))    #Supprimer en non usb
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        """

        if not self.cap.isOpened():
            print("Erreur : Impossible d'accéder à la caméra / d'ouvrir le pipeline d'entrée")
            sys.exit()


    def read(self):
        """Lit une frame de la caméra"""
        return self.cap.read()

    def send(self, frame):
        """Envoie la frame traitée sur le réseau"""
        if self.out is not None and self.out.isOpened():
            self.out.write(frame)

    def release(self):
        """Libère proprement les ressources"""
        self.cap.release()
        if self.out is not None and self.out.isOpened():
            self.out.release()
        print("Pipelines fermées.")