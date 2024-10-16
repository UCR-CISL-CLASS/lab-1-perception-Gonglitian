import carla
import pygame
import numpy as np
from detector import Detector
# 初始化Pygame
pygame.init()

# 设置窗口大小
display_width = 800
display_height = 600
camera_display = pygame.display.set_mode((display_width, display_height))
pygame.display.set_caption("CARLA Camera View")

# 连接到CARLA服务器
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# 获取蓝图库
blueprint_library = world.get_blueprint_library()

# 生成车辆
vehicle_bp = blueprint_library.filter('vehicle.*')[0]
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
#设置车辆ID
vehicle.set_attribute('role_name', 'hero')


# 设置车辆为自动驾驶
vehicle.set_autopilot(True)

# 定义传感器
sensors = Detector().sensors()

# 安装传感器
sensor_actors = []
for sensor in sensors:
    print(sensor)
    sensor_bp = blueprint_library.find(sensor['type'])
    # for attr_name, attr_value in sensor.items():
    #     if attr_name != 'type' and attr_name != 'id':
    #         sensor_bp.set_attribute(attr_name, str(attr_value))
    sensor_transform = carla.Transform(carla.Location(x=sensor['x'], y=sensor['y'], z=sensor['z']),
                                       )
    sensor_actor = world.spawn_actor(
        sensor_bp, sensor_transform, attach_to=vehicle)
    sensor_actors.append(sensor_actor)

# 摄像头回调函数


def camera_callback(image, display):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    display.blit(surface, (0, 0))
    pygame.display.flip()


# 绑定摄像头回调函数
camera_sensor = sensor_actors[0]  # 使用第一个摄像头
camera_sensor.listen(lambda image: camera_callback(image, camera_display))

# 主循环
try:
    while True:
        world.tick()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
except KeyboardInterrupt:
    print("退出程序")

# 清理
for sensor_actor in sensor_actors:
    sensor_actor.stop()
    sensor_actor.destroy()
vehicle.destroy()
pygame.quit()
