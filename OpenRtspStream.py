import ffmpeg
import numpy as np
import cv2
#ffmpeg -f video4linux2 -list_formats all -i /dev/video0

def main(source):
    args = {
        "rtsp_transport": "tcp",
        "fflags": "nobuffer",
        "flags": "low_delay"
    }    # 添加参数
    probe = ffmpeg.probe(source)
    cap_info = next(x for x in probe['streams'] if x['codec_type'] == 'video')
    print("fps: {}".format(cap_info['r_frame_rate']))
    width = cap_info['width']           # 获取视频流的宽度
    height = cap_info['height']         # 获取视频流的高度
    up, down = str(cap_info['r_frame_rate']).split('/')
    fps = eval(up) / eval(down)
    print("fps: {}".format(fps))    # 读取可能会出错错误
    process1 = (
        ffmpeg
        .input(source, **args)
        .output('pipe:', format='rawvideo', pix_fmt='rgb24')
        .overwrite_output()
        .run_async(pipe_stdout=True)
    )
    while True:
        in_bytes = process1.stdout.read(width * height * 3)     # 读取图片
        if not in_bytes:
            break
        in_frame = np.frombuffer(in_bytes, np.uint8).reshape([height, width, 3])
        in_frame = cv2.resize(in_frame, None, fx=0.8, fy=0.8)   # 改变图片尺寸
        frame = cv2.cvtColor(in_frame, cv2.COLOR_RGB2BGR)  # 转成BGR
        cv2.imshow("ffmpeg", frame)
        if cv2.waitKey(1) == ord('q'):
            break
    process1.kill()             # 关闭


if __name__ == "__main__":
    alhua_rtsp = "rtsp://127.0.0.1:8554/video_stream"
    main(alhua_rtsp)