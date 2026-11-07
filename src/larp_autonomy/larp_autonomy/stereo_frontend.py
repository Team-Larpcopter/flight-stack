"""Pair native-rate cameras; preserve capture time and factory calibration."""
from collections import deque
from copy import deepcopy
import json,array
import numpy as np


def scaled_calibration(k,p,width,height,target_width,target_height):
    sx,sy=target_width/width,target_height/height
    if not 0<sx<=1 or not 0<sy<=1:raise ValueError('Only nonempty downsampling is supported')
    # cv2.resize maps pixel centres, not image corners.
    transform=np.array([[sx,0,(sx-1)/2],[0,sy,(sy-1)/2],[0,0,1]])
    return (transform@np.asarray(k).reshape(3,3)).ravel().tolist(),(transform@np.asarray(p).reshape(3,4)).ravel().tolist()


def main():
    import cv2,rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo
    from std_msgs.msg import String
    cv2.setNumThreads(1)
    rclpy.init();node=Node('autonomy_stereo_frontend')
    width=int(node.declare_parameter('width',0).value);height=int(node.declare_parameter('height',0).value)
    skew=float(node.declare_parameter('max_pair_skew_s',.0005).value)
    if not 0<=skew<=.002 or width<0 or height<0 or (width==0)!=(height==0):raise ValueError('Invalid stereo limits')
    queues=[deque(maxlen=8),deque(maxlen=8)];infos=[None,None];counts={'pairs':0,'unpaired':0,'stale':0}
    pubs=[];ipubs=[]
    for side in ('left','right'):
        pubs.append(node.create_publisher(Image,f'/autonomy/stereo/{side}/image',4))
        ipubs.append(node.create_publisher(CameraInfo,f'/autonomy/stereo/{side}/camera_info',4))
    status=node.create_publisher(String,'/autonomy/stereo/status',4)
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def receive(m,i):
        if m.encoding!='mono8' or m.is_bigendian:return
        if queues[i] and stamp(m)<=stamp(queues[i][-1]):return
        queues[i].append(m)
        while queues[0] and queues[1]:
            ts=[stamp(q[0]) for q in queues]
            if abs(ts[0]-ts[1])>skew:
                queues[int(ts[1]<ts[0])].popleft();counts['unpaired']+=1;continue
            pair=[q.popleft() for q in queues];capture=(ts[0]+ts[1])/2
            age=node.get_clock().now().nanoseconds*1e-9-capture
            if age<-.005 or age>.2:counts['stale']+=1;continue
            if any(info is None for info in infos):continue
            if any((im.width,im.height,im.header.frame_id)!=(ci.width,ci.height,ci.header.frame_id) for im,ci in zip(pair,infos)):continue
            if any(ci.distortion_model not in ('plumb_bob','') or any(abs(d)>1e-8 for d in ci.d) for ci in infos):
                node.get_logger().error('Expected genuinely undistorted pinhole images');continue
            ns=round(capture*1e9)
            for j,(im,ci) in enumerate(zip(pair,infos)):
                a=np.frombuffer(im.data,np.uint8).reshape(im.height,im.step)[:,:im.width]
                w,h=(width,height) if width else (im.width,im.height)
                if (w,h)!=(im.width,im.height):
                    if w>im.width or h>im.height:raise ValueError('Upsampling is not supported')
                    a=cv2.resize(a,(w,h),interpolation=cv2.INTER_AREA)
                out=Image();out.header=deepcopy(im.header);out.header.stamp.sec=ns//10**9;out.header.stamp.nanosec=ns%10**9
                out.width=w;out.height=h;out.encoding='mono8';out.step=w;out.data=im.data if (w,h)==(im.width,im.height) and im.step==w else array.array('B',a.tobytes())
                info=deepcopy(ci);info.header=deepcopy(out.header)
                info.k,info.p=scaled_calibration(ci.k,ci.p,ci.width,ci.height,w,h);info.width=w;info.height=h
                pubs[j].publish(out);ipubs[j].publish(info)
            counts['pairs']+=1
    for i,side in enumerate(('left','right')):
        topic=node.declare_parameter(f'{side}_topic',f'/oak/{side}/image_undistorted').value
        ci=node.declare_parameter(f'{side}_info_topic',topic+'/camera_info').value
        node.create_subscription(Image,topic,lambda m,i=i:receive(m,i),qos_profile_sensor_data)
        node.create_subscription(CameraInfo,ci,lambda m,i=i:infos.__setitem__(i,m),qos_profile_sensor_data)
    node.create_timer(1.,lambda:status.publish(String(data=json.dumps(counts))))
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
