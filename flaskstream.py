import cv2
import sys
import threading
import time
from flask import Flask, Response

class VideoFStreamer:
    def __init__(self, width=1280, height=720, fps=30, stream='N'):
        self.width = width
        self.height = height
        self.fps = fps
        self.port = 5000
        self.IP = '192.168.2.3'

        self.app = Flask(__name__)
        self.last_frame = None
        self.lock = threading.Lock()  # Pour éviter les conflits d'accès à la frame

        # Configuration des routes
        self.app.add_url_rule('/', 'index', self.index)
        self.app.add_url_rule('/video_feed', 'video_feed', self.video_feed)

        # Lancement de Flask dans un thread démon (s'arrête avec le programme)
        self.thread = threading.Thread(target=self._run_server, daemon=True)
        self.thread.start()
        print(f"Serveur Flask lancé sur http://{self.IP}:{self.port}")

        #Camera raspberry :
        """
        gst_in = (
            f"libcamerasrc ! "
            f"video/x-raw,width={WIDTH},height={HEIGHT},framerate={FPS}/1 ! " #format=GRAY8 #niveaux de gris (ici si camera isc)
            f"videoconvert ! " # ou v4l2convert
            f"video/x-raw,format=BGR ! "        #format=GRAY8 #niveaux de gris
            f"appsink drop=true"
        )
        """
        gst_in = (
            "v4l2src device=/dev/video0 ! "
            "image/jpeg,width=1920,height=1080,framerate=30/1 ! "
            "jpegdec ! "        #v4l2jpegdec #à tester, version matérielle
            "videoconvert ! video/x-raw,format=BGR ! appsink drop=true"
        )

        #cap = cv2.VideoCapture(gst_in, cv2.CAP_GSTREAMER)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2) #non usb : cap = cv2.VideoCapture(0)    (pas sur de fonctionnner)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))    #Supprimer en non usb
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            print("Erreur : Impossible d'accéder à la caméra / d'ouvrir le pipeline d'entrée")
            sys.exit()

    def _run_server(self):
        # On désactive le reloader car on est déjà dans un thread
        self.app.run(host='0.0.0.0', port=self.port, debug=False, use_reloader=False)

    def index(self):
        return '<img src="/video_feed" style="width:100%">'

    def video_feed(self):
        return Response(self._generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def _generate(self):
        print(f"Streaming en {self.height}p... \n\nctrl+C to stop")
        while True:
            time.sleep(0.03)
            with self.lock:
                if self.last_frame is None:
                    continue
                # Encodage en JPEG pour le navigateur
                ret, buffer = cv2.imencode('.jpg', self.last_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


    def read(self):
        """Lit une frame de la caméra"""
        return self.cap.read()

    def send(self, frame):
        """Met à jour la frame qui sera diffusée par Flask"""
        with self.lock:
            #small_frame = cv2.resize(frame, (640, 360))
            #self.last_frame = small_frame
            self.last_frame = frame.copy()

    def release(self):
        # Flask n'a pas de méthode simple 'stop', le mode daemon=True suffit
        print("Serveur Flask prêt à fermer.")
        self.cap.release()
