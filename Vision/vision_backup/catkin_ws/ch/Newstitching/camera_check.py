# -*- coding: utf-8 -*-

import cv2

# 각 카메라 인덱스를 시도하여 확인
for i in range(6):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"카메라 {i} 열기 성공")
    else:
        print(f"카메라 {i} 열기 실패")

