import cv2
import subprocess

def get_video_devices():
    """v4l2를 사용해 현재 시스템의 모든 비디오 장치를 가져옵니다."""
    devices = []
    try:
        output = subprocess.check_output("v4l2-ctl --list-devices", shell=True).decode()
        lines = output.split("\n")
        for i, line in enumerate(lines):
            if "/dev/video" in line:
                devices.append(line.strip())
    except Exception as e:
        print(f"Error getting video devices: {e}")
    return devices

def match_cameras():
    """VideoCapture 번호와 장치 경로 매칭"""
    devices = get_video_devices()
    camera_matches = {}

    print("\nMatching VideoCapture numbers with device paths...\n")
    for i in range(len(devices)):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                camera_matches[i] = devices[i]
                print(f"VideoCapture {i}: {devices[i]} (Resolution: {frame.shape[1]}x{frame.shape[0]})")
            else:
                print(f"VideoCapture {i}: Unable to capture frame.")
            cap.release()
        else:
            print(f"VideoCapture {i}: Unable to open camera.")

    if not camera_matches:
        print("No cameras matched.")
    return camera_matches

if __name__ == "__main__":
    match_cameras()

