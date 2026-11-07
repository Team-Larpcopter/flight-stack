"""Read-only native-image quality probe; no camera controls or vehicle publishers."""
import argparse,json,time
from pathlib import Path
import numpy as np


def metrics(image):
    import cv2
    corners=cv2.goodFeaturesToTrack(image,maxCorners=300,qualityLevel=.01,minDistance=15)
    tiles=np.zeros((4,6),bool)
    if corners is not None:
        for x,y in corners[:,0,:]:
            tiles[min(3,int(4*y/image.shape[0])),min(5,int(6*x/image.shape[1]))]=True
    return {'mean':float(image.mean()),'p05_p50_p95':np.percentile(image,[5,50,95]).tolist(),
            'dark_fraction':float(np.mean(image<=5)),'saturated_fraction':float(np.mean(image>=250)),
            'laplacian_variance':float(cv2.Laplacian(image,cv2.CV_32F).var()),
            'corners':0 if corners is None else len(corners),'feature_tile_fraction':float(tiles.mean())}


def main():
    import cv2,rclpy
    from sensor_msgs.msg import Image
    from rclpy.qos import qos_profile_sensor_data
    ap=argparse.ArgumentParser();ap.add_argument('--out',required=True);ap.add_argument('--seconds',type=float,default=20.)
    args=ap.parse_args();cv2.setNumThreads(1);rclpy.init();node=rclpy.create_node('autonomy_image_quality')
    rows={s:[] for s in ('left','right')};last={s:-float('inf') for s in rows}
    def receive(m,side):
        if m.encoding!='mono8' or time.monotonic()-last[side]<.5:return
        last[side]=time.monotonic()
        im=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width]
        data=metrics(im);data.update(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,width=m.width,height=m.height)
        rows[side].append(data)
    for side in rows:
        node.create_subscription(Image,f'/oak/{side}/image_undistorted',lambda m,s=side:receive(m,s),qos_profile_sensor_data)
    end=time.monotonic()+args.seconds
    try:
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    finally:node.destroy_node();rclpy.try_shutdown()
    out=Path(args.out);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps({'frames':rows,'note':'Stationary texture/brightness diagnostic. Laplacian variance alone cannot qualify moving blur; exposure metadata and motion testing are required.'},indent=2)+'\n')


if __name__=='__main__':main()
