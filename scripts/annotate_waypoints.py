
import rospy
import cv2
import os
import glob
import tempfile
import subprocess
import shutil


def extract_images(tmp_dir, args):
    rospy.loginfo('Using {} as working directory.'.format(tmp_dir))
    cmd = ["rosrun", "bag_tools", "extract_images", tmp_dir, "jpg", args.topic] + args.inbag
    subprocess.call(cmd)
    images = glob.glob(tmp_dir + '/*.jpg')
    images.sort()
    return images

    # for i in range(0, len(images)):
    #     shutil.move(images[i], tmp_dir + '/img-' + str(i) + '.jpg')


def annotate_waypoints(tmp_dir, args):
    images = self.extract_images(tmp_dir, args)
    waypoints = []
    i = 0
    while i < len(images):
        cv2.imshow(images[i])
        key = cv2.waitkey(1/args.fps)
        if key == 8: # backspace
            i -= 1
        elif key == 32: # space
            waypoints.append(i)
        else:
            i += 1

    print "Waypoints: {}".format(waypoint))


if __name__ == "__main__":
    rospy.init_node('annotate_waypoints')
    import argparse
    parser=argparse.ArgumentParser(description='Annotate the given bag file')
    parser.add_argument('topic', help='topic of the images to use')
    parser.add_argument('--fps', help='frames per second of the recording', type=int, default=30)
    parser.add_argument('inbag', help='input bagfile(s)', nargs='+')
    args=parser.parse_args()
    tmp_dir=tempfile.mkdtemp()
    try:
        annotate_waypoints(create_video(tmp_dir, args)
    except Exception, e:
        import traceback
        traceback.print_exc()
    rospy.loginfo('Cleaning up temp files...')
    shutil.rmtree(tmp_dir)
