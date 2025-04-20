import sensor, image, time

sensor.reset()
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((320, 240))
sensor.skip_frames(time=2000)
sensor.set_framerate(10)
clock = time.clock()

KEYPOINTS_SIZE = 8


def draw_keypoints(img, keypoints):
    if keypoints:
        print(keypoints)
        img.draw_keypoints(keypoints, size=KEYPOINTS_SIZE)

kpoints = None

frame_width = sensor.width()
frame_height = sensor.height()
half_height = frame_height // 2
sensor.set_framebuffers(10)
frame = sensor.alloc_extra_fb(frame_width, frame_height, sensor.GRAYSCALE)

while True:
    clock.tick()

    last_frame = sensor.get_fb()
#    time.sleep_us(500000)
    current_frame = sensor.snapshot()


    frame_similarity= current_frame.get_similarity(last_frame)
    print(frame_similarity.stdev())

    displayed_last_frame = image.Image(frame_width, half_height, sensor.GRAYSCALE)
    displayed_last_frame.replace(last_frame, roi=(0, half_height, frame_width, half_height))
    current_frame.draw_image(displayed_last_frame,0,0)
    kpoints = current_frame.find_keypoints(max_keypoints=100, threshold=20, scale_factor=1.2)
    if kpoints:
        draw_keypoints(current_frame, kpoints)

    sensor.flush()

    print("FPS:", clock.fps())

