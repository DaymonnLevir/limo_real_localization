#!/usr/bin/env python3
"""Native ROS1 acquisition and official FAST-Calib invocation, without source patches."""
import argparse
import math
from pathlib import Path
import signal
import subprocess
import time
import yaml
import rosbag
import rospy
from sensor_msgs.msg import PointCloud2
from capture_rgb import capture

ROOT = Path('/calibration')
p = argparse.ArgumentParser()
s = p.add_subparsers(dest='action', required=True)
c = s.add_parser('capture')
c.add_argument('scene', choices=['scene_01', 'scene_02', 'scene_03'])
r = s.add_parser('single')
r.add_argument('scene', choices=['scene_01', 'scene_02', 'scene_03'])
r.add_argument('--roi', nargs=6, type=float, required=True,
               metavar=('XMIN','XMAX','YMIN','YMAX','ZMIN','ZMAX'))
s.add_parser('multi')
a = p.parse_args()
config = yaml.safe_load((ROOT/'config/target.yaml').read_text())

def verify_bag(path):
    with rosbag.Bag(str(path)) as bag:
        info = bag.get_type_and_topic_info().topics.get('/livox/lidar')
        if not info or info.msg_type != 'sensor_msgs/PointCloud2' or not info.message_count:
            raise RuntimeError('Bag sem PointCloud2 /livox/lidar')

if a.action == 'capture':
    rospy.init_node('stage3_capture', anonymous=True)
    msg = rospy.wait_for_message('/livox/lidar', PointCloud2, timeout=15)
    if not {'x','y','z'}.issubset({f.name for f in msg.fields}):
        raise RuntimeError('PointCloud2 sem campos x/y/z')
    scene = ROOT/'datasets'/a.scene
    scene.mkdir(exist_ok=False)
    # All cloud messages are recorded in ROS1 while the same static image is captured.
    proc = subprocess.Popen(['rosbag','record','--duration=8','-O',str(scene/'lidar.bag'),'/livox/lidar'])
    try:
        time.sleep(1)
        capture(scene/'image.png')
        if proc.poll() is not None:
            raise RuntimeError('Captura RGB não terminou durante a gravação da bag; descarte esta cena')
        proc.wait(timeout=30)
        if proc.returncode:
            raise RuntimeError('rosbag record falhou')
        verify_bag(scene/'lidar.bag')
        (scene/'COMPLETE').write_text('ROS1 lidar.bag + RAW image.png; sensores e alvo estáticos.\n')
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=10)
    print(scene)
elif a.action == 'single':
    if not all(math.isfinite(v) for v in a.roi) or any(a.roi[i] >= a.roi[i+1] for i in (0,2,4)):
        p.error('ROI deve conter mínimos < máximos finitos, em metros no frame LiDAR')
    scene = ROOT/'datasets'/a.scene
    if not (scene/'COMPLETE').is_file():
        raise RuntimeError('Aquisição incompleta')
    verify_bag(scene/'lidar.bag')
    output = ROOT/'results'/a.scene
    output.mkdir(exist_ok=False)
    config.update(dict(zip(['x_min','x_max','y_min','y_max','z_min','z_max'],a.roi)))
    config.update(bag_path=str(scene/'lidar.bag'), image_path=str(scene/'image.png'), output_path=str(output)+'/')
    params = output/'params.yaml'
    params.write_text(yaml.safe_dump(config))
    subprocess.run(['rosparam','load',str(params)],check=True)
    # Upstream keeps publishing after saving. Stop only our child once all outputs exist.
    proc = subprocess.Popen(['rosrun','fast_calib','fast_calib'])
    expected = ['single_calib_result.txt','colored_cloud.pcd','qr_detect.png','circle_center_record.txt']
    deadline = time.monotonic() + 600
    try:
        while proc.poll() is None:
            if all((output/f).is_file() and (output/f).stat().st_size for f in expected):
                proc.send_signal(signal.SIGINT)
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('FAST-Calib excedeu 600 s; inspecione os resultados')
            time.sleep(0.5)
        code = proc.wait(timeout=10)
        if code or not all((output/f).is_file() for f in expected):
            raise RuntimeError('FAST-Calib não terminou com todos os resultados; código ' + str(code))
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
    print('Resultado individual salvo; inspecione a projeção e os quatro pares antes de combinar.')

else:
    records=[]
    for name in ['scene_01','scene_02','scene_03']:
        folder=ROOT/'results'/name
        if not (folder/'single_calib_result.txt').is_file():
            raise RuntimeError('Resultado individual ausente: '+name)
        record = (folder/'circle_center_record.txt').read_text()
        if len([line for line in record.splitlines() if line.startswith('time:')]) != 1:
            raise RuntimeError('Esperado exatamente um registro por cena: '+name)
        records.append(record)
    output=ROOT/'results'/'multi'
    output.mkdir(exist_ok=False)
    (output/'circle_center_record.txt').write_text('\n'.join(records))
    subprocess.run(['rosparam','set','/output_path',str(output)+'/'],check=True)
    subprocess.run(['rosrun','fast_calib','multi_fast_calib'],check=True)
