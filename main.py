import cv2
import time
import sys
from gstream import VideoGStreamer
from flaskstream import VideoFStreamer

Width=1280
Height=720
FPS_input=30
Colors_input = False

FPS_output=30
Colors_output = True

Stream = 'G'

if input("Default : Def_in: {1}x{2}, FPS_in: {3}, Colors_in: {4} \n Stream: {5}, FPS_out: {6}, Colors_out: {7} \n (y/n): ")=='n':
    Width = input("Width: ")
    Height = input("Height: ")
    FPS_input = input("FPS_in: ")
    Colors_input = input("Colors_in (y/n): ")=='y'

    FPS_output = input("FPS_output: ")
    Colors_output = input("Colors_output (y/n) : ")=='y'

    Stream = input("Stream (G for gstreamer / F for flask / N for none): ")


if Stream=="F" or Stream=="f":
    streamer = VideoFStreamer(width=Width, height=Height, fps_input=FPS_input, colors_input=Colors_input,stream=Stream, fps_output=FPS_output, colors_output=Colors_output)
else:
    streamer = VideoGStreamer(width=Width, height=Height, fps_input=FPS_input, colors_input=Colors_input,stream=Stream, fps_output=FPS_output, colors_output=Colors_output)

print("Traitement démarré... ctrl+C pour stopper")

prev_time = time.time()
frame_count = 0
fps=0

try:
    while True:
        ret, frame = streamer.read()
        if not ret:
            break

        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)        #conversion niveaux de gris
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # --- DÉBUT DU TRAITEMENT OPENCV ---
        # Exemple simple : Détection de visages ou dessin
        cv2.putText(frame, f"Pi4 - Detection Active - FPS {fps}",
                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Insérez votre modèle de détection ici (YOLO, Haar, etc.)
        # --- FIN DU TRAITEMENT OPENCV ---

        # Envoi vers le pipeline de streaming
        if Stream!="N" or "n":
            streamer.send(frame)

            frame_count += 1
            current_time = time.time()
            if current_time - prev_time >= 1.0:
                print(f"FPS Réels : {frame_count}")
                fps = frame_count
                frame_count = 0
                prev_time = current_time

except KeyboardInterrupt:
    print("\nArrêt du programme...")
    sys.exit(1)

finally:
    streamer.release()
