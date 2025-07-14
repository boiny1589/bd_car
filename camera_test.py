# -*- coding: utf-8 -*-
import cv2
import time

# ��������ͷ����ָ���豸��
front_cam = cv2.VideoCapture(1)  # ǰ����ͷ (�豸��1)
side_cam = cv2.VideoCapture(2)   # ������ͷ (�豸��2)

# ��������ͷ�������������
for cam in [front_cam, side_cam]:
    if cam.isOpened():
        # ���÷ֱ��ʣ����ͷֱ��ʿ�����ٶȣ�
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # ����֡��
        cam.set(cv2.CAP_PROP_FPS, 30)
        # ���û�������С
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# �������ͷ�Ƿ�ɹ���
if not front_cam.isOpened():
    print("? �޷���ǰ����ͷ (�豸��1)")
if not side_cam.isOpened():
    print("? �޷��򿪲�����ͷ (�豸��2)")

# ����������ʾ����
cv2.namedWindow("Front Camera - Device 1", cv2.WINDOW_NORMAL)
cv2.namedWindow("Side Camera - Device 2", cv2.WINDOW_NORMAL)

# ���ô��ڴ�С
cv2.resizeWindow("Front Camera - Device 1", 640, 480)
cv2.resizeWindow("Side Camera - Device 2", 640, 480)

print("?? ����ͷ���Կ�ʼ���� 'q' ���˳�...")
start_time = time.time()
frame_count = 0

while True:
    # ��ȡǰ����ͷ֡
    ret_front, frame_front = front_cam.read()
    if ret_front:
        # �ڻ���������豸�ű�ʶ��FPS��Ϣ
        cv2.putText(frame_front, "Device 1", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Front Camera - Device 1", frame_front)
    
    # ��ȡ������ͷ֡
    ret_side, frame_side = side_cam.read()
    if ret_side:
        # �ڻ���������豸�ű�ʶ��FPS��Ϣ
        cv2.putText(frame_side, "Device 2", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Side Camera - Device 2", frame_side)
    
    frame_count += 1
    
    # ÿ100֡��ʾһ��FPS
    if frame_count % 100 == 0:
        elapsed_time = time.time() - start_time
        fps = frame_count / elapsed_time
        print(f"?? ��ǰFPS: {fps:.2f}")
    
    # ��'q'���˳�����
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):  # ��'s'�����浱ǰ֡
        if ret_front:
            cv2.imwrite(f"front_cam_{int(time.time())}.jpg", frame_front)
        if ret_side:
            cv2.imwrite(f"side_cam_{int(time.time())}.jpg", frame_side)
        print("?? �ѱ��浱ǰ֡")

# ��������FPS
total_time = time.time() - start_time
final_fps = frame_count / total_time
print(f"?? ƽ��FPS: {final_fps:.2f}")
print(f"??  ������ʱ��: {total_time:.2f}��")

# �ͷ���Դ
front_cam.release()
side_cam.release()
cv2.destroyAllWindows()