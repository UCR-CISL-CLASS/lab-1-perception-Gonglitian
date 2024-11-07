from mmdet3d.apis import LidarDet3DInferencer

model = ".\\PointPillars_model\\pointpillars_hv_secfpn_8xb6-160e_kitti-3d-car.py"
weights = ".\\PointPillars_model\\hv_pointpillars_secfpn_6x8_160e_kitti-3d-car_20220331_134606-d42d15ed.pth"
device = "cuda:0"

inferencer = LidarDet3DInferencer(model= model, weights = weights, device= device)