import carla
import random
import time

# 连接到Carla服务器
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# 获取蓝图库
blueprint_library = world.get_blueprint_library()

# 选择车辆蓝图
vehicle_bp = blueprint_library.filter('vehicle.*')[0]

# 选择生成位置
spawn_points = world.get_map().get_spawn_points()
spawn_point = random.choice(spawn_points)

# 生成车辆
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
vehicle_id = vehicle.id
print(f"Generated vehicle with ID: {vehicle_id}")
# 选择camera蓝图
camera_bp = blueprint_library.find('sensor.camera.rgb')

# 设置camera属性
camera_bp.set_attribute('image_size_x', '800')
camera_bp.set_attribute('image_size_y', '600')
camera_bp.set_attribute('fov', '90')

# 选择camera生成位置（相对于车辆）
camera_transform = carla.Transform(carla.Location(x=-5, z=2.5))

# 生成camera并附加到车辆上
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

# 定义图像处理回调函数


def process_image(image):
    image.save_to_disk('_out/%06d.png' % image.frame)


# 绑定图像处理回调函数
camera.listen(process_image)

# 运行一段时间以捕获图像
time.sleep(10)

# 销毁actor
camera.destroy()
vehicle.destroy()
