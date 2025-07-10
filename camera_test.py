import cv2

# 创建摄像头对象并指定设备号
front_cam = cv2.VideoCapture(2)  # 前摄像头 (设备号0)
side_cam = cv2.VideoCapture(1)   # 侧摄像头 (设备号2)

# 检查摄像头是否成功打开
if not front_cam.isOpened():
    print("❌ 无法打开前摄像头 (设备号qian)")
if not side_cam.isOpened():
    print("❌ 无法打开侧摄像头 (设备号ce)")

# 创建两个显示窗口
cv2.namedWindow("Front Camera - Device 0", cv2.WINDOW_NORMAL)
cv2.namedWindow("Side Camera - Device 2", cv2.WINDOW_NORMAL)

while True:
    # 读取前摄像头帧
    ret_front, frame_front = front_cam.read()
    if ret_front:
        # 在画面上添加设备号标识
        cv2.putText(frame_front, "Device 0", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Front Camera - Device 0", frame_front)
    
    # 读取侧摄像头帧
    ret_side, frame_side = side_cam.read()
    if ret_side:
        # 在画面上添加设备号标识
        cv2.putText(frame_side, "Device 2", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Side Camera - Device 2", frame_side)
    
    # 按'q'键退出程序
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
front_cam.release()
side_cam.release()
cv2.destroyAllWindows()