#!/usr/bin/env python3
"""Capture RAW RGB at the validated resolution; never resize or rectify."""
import sys
from pathlib import Path
import cv2

def capture(filename):
    path = Path(filename)
    if path.exists():
        raise RuntimeError('Arquivo já existe: ' + str(path))
    cam = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    try:
        if not cam.isOpened():
            raise RuntimeError('Não foi possível abrir /dev/video0')
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        for _ in range(30):
            ok, frame = cam.read()
            if not ok:
                raise RuntimeError('Falha na leitura da câmera')
        if frame.shape != (480, 640, 3):
            raise RuntimeError('Resolução/formato diferente de 640x480 BGR')
        if not cv2.imwrite(str(path), frame):
            raise RuntimeError('Falha ao salvar PNG')
    finally:
        cam.release()

if __name__ == '__main__':
    capture(sys.argv[1])
