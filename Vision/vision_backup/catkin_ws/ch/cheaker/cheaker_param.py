import cv2
import numpy as np
import glob
import os

# 체커보드 내부 코너 개수
CHECKERBOARD = (7, 5)
IMAGE_FOLDER = "camera4_images/"  # 카메라1 이미지 폴더
SAVE_PATH = "camera4_intrinsic.npz"  # 저장할 내부 파라미터 파일

objpoints = []  # 3D 좌표 리스트
imgpoints = []  # 2D 좌표 리스트

# 3D 체커보드 좌표 생성
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# 이미지 불러오기
images = glob.glob(f"{IMAGE_FOLDER}/*.jpg")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 체커보드 코너 찾기
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

# 내부 파라미터 계산
ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# 내부 파라미터 저장
np.savez(SAVE_PATH, mtx=mtx, dist=dist)

print(f"✅ 카메라1 내부 파라미터 저장 완료: {SAVE_PATH}")
print(f"🔹 카메라 행렬 (mtx):\n{mtx}")
print(f"🔹 왜곡 계수 (dist):\n{dist}")

