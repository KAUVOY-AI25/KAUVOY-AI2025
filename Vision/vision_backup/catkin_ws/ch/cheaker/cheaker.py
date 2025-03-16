import cv2
import os

# 저장할 폴더 설정
IMAGE_FOLDER = "camera4_images/"
os.makedirs(IMAGE_FOLDER, exist_ok=True)  # 폴더 생성

cap = cv2.VideoCapture(2)  # 카메라 1번 (장치 번호 확인 필요)
img_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    cv2.imshow("Camera 1 - Capture Mode", frame)

    # 스페이스바(␣)를 누르면 이미지 저장
    key = cv2.waitKey(1) & 0xFF
    if key == ord(' '):  # 스페이스바(␣) 감지
        filename = f"{IMAGE_FOLDER}/image_{img_count:03d}.jpg"
        cv2.imwrite(filename, frame)
        print(f"✅ 저장됨: {filename}")
        img_count += 1

    # ESC(⎋)를 누르면 종료
    if key == 27:  # ESC(⎋) 감지
        break

cap.release()
cv2.destroyAllWindows()

