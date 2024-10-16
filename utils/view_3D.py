import carla
import pygame
import numpy as np
import argparse
# 设置窗口大小
display_width = 800
display_height = 600
camera_display = pygame.display.set_mode((display_width, display_height))
pygame.display.set_caption("Vehicle View")


def get_vehicle_by_id(world, vehicle_id):
    # 获取世界中的所有actor
    actors = world.get_actors()

    # 遍历所有actor，找到指定ID的车辆
    for actor in actors:
        # print(actor.id, actor.type_id)
        if actor.id == vehicle_id and 'vehicle' in actor.type_id:
            return actor
    return None


def camera_callback(image, display):
    # 摄像头回调函数
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    display.blit(surface, (0, 0))
    pygame.display.flip()


def main(vehicle_id):
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # 获取车辆
    vehicle = get_vehicle_by_id(world, vehicle_id)

    if vehicle:
        print(f"Found vehicle with ID {vehicle_id}")
        # 在这里可以对车辆进行进一步操作，例如附加传感器
        # 第三人称camera
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=-5, z=2.5))
        camera = world.spawn_actor(
            camera_bp, camera_transform, attach_to=vehicle)
        # pygame 显示camera图像

        # 绑定摄像头回调函数
        camera.listen(lambda image: camera_callback(image, camera_display))

        # 主循环
        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        raise KeyboardInterrupt
        except KeyboardInterrupt:
            print("退出程序")

        # 清理
        camera.destroy()
        pygame.quit()

    else:
        print(f"Vehicle with ID {vehicle_id} not found")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--id', type=int, required=True, help='Vehicle ID')
    args = parser.parse_args()
    main(args.id)
