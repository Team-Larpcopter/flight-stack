#!/usr/bin/env python3
"""Bounded diagnostic acquisition, no ROS or vehicle interface.

Uses the same Camera/IMU configuration as the native driver. This diagnoses
firmware timestamps; Python delivery timings do not qualify the C++ pipeline.
"""
import argparse,csv,json,time
from pathlib import Path
import depthai as dai

p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--seconds',type=float,default=20)
p.add_argument('--accel-hz',type=int,default=250);p.add_argument('--split-imu',action='store_true')
p.add_argument('--gyro-hz',type=int,default=200);p.add_argument('--max-exposure-us',type=int,default=0)
a=p.parse_args()
if not 0<a.seconds<=120:raise ValueError('capture duration must be in (0,120]')
out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
with dai.Pipeline() as pipeline:
    device=pipeline.getDefaultDevice();calib=device.readCalibration()
    calib.eepromToJsonFile(str(out/'factory.json'))
    rig={'sdk':dai.__version__,'usb_speed':str(device.getUsbSpeed()),'camera_to_imu_cm':{}}
    queues=[]
    for side,socket in [('left',dai.CameraBoardSocket.CAM_B),('right',dai.CameraBoardSocket.CAM_C)]:
        rig['camera_to_imu_cm'][side]=calib.getCameraToImuExtrinsics(socket)
        cam=pipeline.create(dai.node.Camera).build(socket,sensorFps=30)
        if a.max_exposure_us:cam.initialControl.setAutoExposureLimit(a.max_exposure_us)
        image=cam.requestOutput((1280,800),type=dai.ImgFrame.Type.GRAY8,resizeMode=dai.ImgResizeMode.CROP,fps=30,enableUndistortion=True)
        queues.append((side,image.createOutputQueue(maxSize=8,blocking=False)))
    imu=pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW,a.accel_hz)
    gyro=pipeline.create(dai.node.IMU) if a.split_imu else imu
    gyro.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW,a.gyro_hz)
    gyro.setBatchReportThreshold(1);gyro.setMaxBatchReports(10)
    imu.setBatchReportThreshold(1);imu.setMaxBatchReports(10)
    iq=imu.out.createOutputQueue(maxSize=128,blocking=False)
    iqg=gyro.out.createOutputQueue(maxSize=128,blocking=False) if a.split_imu else None
    (out/'rig-sdk.json').write_text(json.dumps(rig,indent=2))
    pipeline.start();last={};saved=set()
    with (out/'timestamps.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['stream','capture_ns','device_ns','arrival_ns','sequence','exposure_us','iso','x','y','z'])
        end=time.monotonic()+a.seconds
        while time.monotonic()<end:
            packets=[iq.tryGet()]
            if iqg:packets.append(iqg.tryGet())
            for packet in packets:
                if packet is None:continue
                for v in packet.packets:
                    for side,r in [('accel',v.acceleroMeter),('gyro',v.gyroscope)]:
                        stamp=round(r.getTimestamp().total_seconds()*1e9)
                        if stamp<=last.get(side,0):continue
                        last[side]=stamp
                        writer.writerow([side,stamp,round(r.getTimestampDevice().total_seconds()*1e9),time.monotonic_ns(),r.sequence,0,0,r.x,r.y,r.z])
            for side,q in queues:
                image=q.tryGet()
                if image is None:continue
                writer.writerow([side,round(image.getTimestamp().total_seconds()*1e9),round(image.getTimestampDevice().total_seconds()*1e9),time.monotonic_ns(),image.getSequenceNum(),image.getExposureTime().total_seconds()*1e6,image.getSensitivity(),0,0,0])
                if side not in saved:
                    t=image.getTransformation()
                    meta={'width':image.getWidth(),'height':image.getHeight(),'K':t.getIntrinsicMatrix(),'D':t.getDistortionCoefficients()}
                    (out/(side+'.json')).write_text(json.dumps(meta,indent=2))
                    (out/(side+'.raw')).write_bytes(bytes(image.getData()));saved.add(side)
            time.sleep(.001)
print('Capture complete:',out,flush=True)
