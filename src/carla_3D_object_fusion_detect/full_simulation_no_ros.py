import carla
import time
import random
import os
import cv2
import numpy as np

# 1. 创建保存目录
os.makedirs("./output/images", exist_ok=True)
os.makedirs("./output/lidar", exist_ok=True)

# 2. 连接 Carla 服务器
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# 3. 设置雨天天气
weather = carla.WeatherParameters(
    cloudiness=90.0,
    precipitation=90.0,
    precipitation_deposits=90.0,
    wind_intensity=20.0,
    wetness=90.0
)
world.set_weather(weather)
print("✅ 雨天天气已设置成功!")

# 4. 生成车辆并开启 Carla 原生自动驾驶
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('model3')[0]
spawn_point = random.choice(world.get_map().get_spawn_points())
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
print("✅ 车辆生成成功!")

# 开启自动驾驶，不依赖任何第三方库
vehicle.set_autopilot(True)
print("✅ 车辆已开启 Carla 原生自动驾驶!")

# 5. 挂载相机和激光雷达（和项目多传感器逻辑对齐）
# RGB 相机
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '800')
camera_bp.set_attribute('image_size_y', '600')
camera_bp.set_attribute('fov', '110')
camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
print("✅ RGB 相机挂载成功!")

# 激光雷达
lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('range', '100')
lidar_bp.set_attribute('points_per_second', '100000')
lidar_bp.set_attribute('rotation_frequency', '10')
lidar_transform = carla.Transform(carla.Location(x=0, z=2.5))
lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
print("✅ 激光雷达挂载成功!")

# 6. 定义传感器回调，保存数据并简单可视化
def process_camera_image(image):
    # 转换为OpenCV格式
    img = np.array(image.raw_data)
    img = img.reshape((image.height, image.width, 4))
    img = img[:, :, :3]
    # 保存图片
    cv2.imwrite(f'./output/images/camera_{image.frame:06d}.png', img)
    # 显示画面
    cv2.imshow("Camera", img)
    cv2.waitKey(1)

def process_lidar_data(point_cloud):
    # 保存点云数据
    point_cloud.save_to_disk(f'./output/lidar/lidar_{point_cloud.frame:06d}.ply')

# 监听传感器数据
camera.listen(process_camera_image)
lidar.listen(process_lidar_data)

# 7. 保持运行，按 Ctrl+C 退出
print("\n🚗 仿真已启动，车辆正在雨天自动行驶，传感器数据将保存到 ./output 文件夹...")
print("按 Ctrl+C 停止仿真")

try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n⏹️ 收到停止信号，正在清理资源...")
finally:
    cv2.destroyAllWindows()
    camera.destroy()
    lidar.destroy()
    vehicle.destroy()
    print("✅ 资源已释放，仿真结束！")