import cv2
import numpy as np
import glob
import os

# 체커보드 내부 코너 개수 (너희 체커보드는 9x6 칸 → 코너는 (8, 5))
CHECKERBOARD = (8, 5)

IMAGE_FOLDER = "camera1_images/"  # 캘리브레이션할 이미지 폴더
FAILED_FOLDER = "camera1_failed/"  # 감지 실패 이미지 저장 폴더
SAVE_PATH = "camera1_intrinsic.npz"  # 내부 파라미터 저장 파일

os.makedirs(FAILED_FOLDER, exist_ok=True)  # 실패한 이미지 폴더 생성

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
        # 코너 검출 정밀도 향상 (서브픽셀 보정)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        objpoints.append(objp)
        imgpoints.append(corners_refined)
    else:
        # 체커보드 감지 실패한 이미지 저장
        failed_path = os.path.join(FAILED_FOLDER, os.path.basename(fname))
        cv2.imwrite(failed_path, img)
        print(f"❌ 체커보드 감지 실패: {failed_path}")

# 최소 20장 이상 감지된 경우에만 캘리브레이션 수행
if len(objpoints) >= 20:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # 내부 파라미터 저장
    np.savez(SAVE_PATH, mtx=mtx, dist=dist)

    print(f"✅ 카메라1 내부 파라미터 저장 완료: {SAVE_PATH}")
    print(f"🔹 카메라 행렬 (mtx):\n{mtx}")
    print(f"🔹 왜곡 계수 (dist):\n{dist}")

    # 📌 재투영 오차 확인
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error

    reprojection_error = total_error / len(objpoints)
    print(f"📌 전체 reprojection error: {reprojection_error}")

    if reprojection_error > 1.0:
        print("⚠️ 경고: 캘리브레이션 오차가 큼. 추가 데이터 확보 필요.")
else:
    print("❌ 감지된 체커보드 개수가 부족하여 캘리브레이션을 수행할 수 없음. 더 많은 이미지를 확보하세요.")

