import cv2
import time

# 创建摄像头对象并指定设备号
front_cam = cv2.VideoCapture(1)  # 前摄像头 (设备号1)
side_cam = cv2.VideoCapture(2)   # 侧摄像头 (设备号2)

# 设置摄像头参数以提高性能
for cam in [front_cam, side_cam]:
    if cam.isOpened():
        # 设置分辨率（降低分辨率可提高速度）
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # 设置帧率
        cam.set(cv2.CAP_PROP_FPS, 30)
        # 设置缓冲区大小
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# 检查摄像头是否成功打开
if not front_cam.isOpened():
    print("❌ 无法打开前摄像头 (设备号1)")
if not side_cam.isOpened():
    print("❌ 无法打开侧摄像头 (设备号2)")

# 创建两个显示窗口
cv2.namedWindow("Front Camera - Device 1", cv2.WINDOW_NORMAL)
cv2.namedWindow("Side Camera - Device 2", cv2.WINDOW_NORMAL)

# 设置窗口大小
cv2.resizeWindow("Front Camera - Device 1", 640, 480)
cv2.resizeWindow("Side Camera - Device 2", 640, 480)

print("🚀 摄像头测试开始，按 'q' 键退出...")
start_time = time.time()
frame_count = 0

while True:
    # 读取前摄像头帧
    ret_front, frame_front = front_cam.read()
    if ret_front:
        # 在画面上添加设备号标识和FPS信息
        cv2.putText(frame_front, "Device 1", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Front Camera - Device 1", frame_front)
    
    # 读取侧摄像头帧
    ret_side, frame_side = side_cam.read()
    if ret_side:
        # 在画面上添加设备号标识和FPS信息
        cv2.putText(frame_side, "Device 2", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Side Camera - Device 2", frame_side)
    
    frame_count += 1
    
    # 每100帧显示一次FPS
    if frame_count % 100 == 0:
        elapsed_time = time.time() - start_time
        fps = frame_count / elapsed_time
        print(f"📊 当前FPS: {fps:.2f}")
    
    # 按'q'键退出程序
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):  # 按's'键保存当前帧
        if ret_front:
            cv2.imwrite(f"front_cam_{int(time.time())}.jpg", frame_front)
        if ret_side:
            cv2.imwrite(f"side_cam_{int(time.time())}.jpg", frame_side)
        print("💾 已保存当前帧")

# 计算最终FPS
total_time = time.time() - start_time
final_fps = frame_count / total_time
print(f"📈 平均FPS: {final_fps:.2f}")
print(f"⏱️  总运行时间: {total_time:.2f}秒")

# 释放资源
front_cam.release()
side_cam.release()
cv2.destroyAllWindows()