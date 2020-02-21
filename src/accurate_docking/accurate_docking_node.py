#!/usr/bin/env python

import rospy

from accurate_docking import AccurateDocking


def main():

    rospy.init_node("accurate_docking_node")

    rc_node = AccurateDocking()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
