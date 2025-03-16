import cv2
import numpy as np
import os

# 저장할 디렉토리 설정
SAVE_DIR = "camera1"
os.makedirs(SAVE_DIR, exist_ok=True)

# 체커보드 패턴의 내부 코너 수 (예: 9x6 체커보드의 내부 코너는 8x5)
CHECKERBOARD = (7, 5)

# 비디오 캡처 시작
cap = cv2.VideoCapture(2)
if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

# 캡처된 이미지 저장 카운터
img_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 가져올 수 없습니다.")
        break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 체커보드 패턴 감지
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    
    if ret:
        # 체커보드가 화면의 80% 이상을 차지할 경우에만 캡처
        corner_x_min = np.min(corners[:, 0, 0])
        corner_x_max = np.max(corners[:, 0, 0])
        corner_y_min = np.min(corners[:, 0, 1])
        corner_y_max = np.max(corners[:, 0, 1])
        
        frame_height, frame_width = frame.shape[:2]
        board_width = corner_x_max - corner_x_min
        board_height = corner_y_max - corner_y_min
        
        if board_width / frame_width > 0.8 and board_height / frame_height > 0.8:
            # 체커보드 코너 그리기
            cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret)
            
            # 이미지 저장 (자동 캡처)
            img_filename = os.path.join(SAVE_DIR, f'checkerboard_capture_{img_count}.jpg')
            cv2.imwrite(img_filename, frame)
            print(f"유효한 체커보드 감지됨! 이미지 저장: {img_filename}")
            img_count += 1
    
    # 화면에 프레임 표시
    cv2.imshow('Camera Feed', frame)
    
    # ESC 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()

print(f"총 {img_count}개의 이미지가 {SAVE_DIR} 디렉토리에 저장되었습니다.")

