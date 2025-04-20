import sensor
import image
import time

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
max_keypoints = 5

def draw_keypoints(img, keypoints):
    if keypoints:
        print(keypoints)
        img.draw_keypoints(keypoints, size=KEYPOINTS_SIZE)

frame_width = sensor.width()
frame_height = sensor.height()
half_height = frame_height // 2

sensor.set_framebuffers(10)
frame = sensor.alloc_extra_fb(frame_width, frame_height, sensor.GRAYSCALE)

last_frame = sensor.snapshot()
lkpoints = last_frame.find_keypoints(max_keypoints=150, threshold=10, scale_factor=1.2)

while True:
    clock.tick()
    current_frame = sensor.snapshot()
    ckpoints = current_frame.find_keypoints(max_keypoints=150, threshold=10, normalized=True)

    frame_similarity = current_frame.get_similarity(last_frame)
#    print("Frame Similarity StdDev:", frame_similarity.stdev())

    displayed_last_frame = image.Image(frame_width, half_height, sensor.GRAYSCALE)
    displayed_last_frame.replace(last_frame, roi=(0, half_height, frame_width, half_height))

    draw_keypoints(displayed_last_frame, lkpoints)

    draw_keypoints(current_frame, ckpoints)


    current_frame.draw_image(displayed_last_frame, 0, 0)

    xframe = image.Image(frame_width, frame_height, sensor.GRAYSCALE)
    xframe.replace(current_frame)
    if ckpoints:
        if lkpoints:
            matches = image.match_descriptor(lkpoints, ckpoints, threshold=85)
            print(matches)
            if matches:
                for match in matches.match():

                    if 0 <= match[0] < len(lkpoints) and 0 <= match[1] < len(ckpoints):
                        lpoint = (lkpoints[match[0]][0], lkpoints[match[0]][1])
                        cpoint = (ckpoints[match[1]][0], ckpoints[match[1]][1])
                        if lpoint[1] <= half_height and cpoint[1] <= half_height:
                            print(lpoint)
                            print(cpoint)
                            print(cpoint[1] + half_height)

                            current_frame.draw_line(
                                lpoint[0], lpoint[1],
                                cpoint[0], cpoint[1] + half_height,
                                color=(255, 0, 0)
                            )
                            current_frame.draw_circle(cpoint[0], cpoint[1] , 2)
                            current_frame.draw_circle(lpoint[0], lpoint[1] + half_height, 2)

    lkpoints = ckpoints
    last_frame.replace(xframe)

    print("FPS:", clock.fps())
