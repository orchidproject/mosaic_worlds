#!/usr/bin/env python
"""
Based on sdf2tfstatic_node.py from gazebo2rviz package
Publish all model world poses from SDF file.
"""

import argparse

import rospy
import tf
import tf_conversions.posemath as pm
from tf.transformations import *

import pysdf

submodelsToBeIgnored = []
tfBroadcaster = None
world = None
tfs = []


def calculate_tfs(prefix):
  world.for_all_submodels(calculate_tf)
  for tf in tfs:
    tf[0] = prefix + pysdf.sdf2tfname(tf[0])
    tf[1] = prefix + pysdf.sdf2tfname(tf[1])


def calculate_tf(model, full_name):
  translation, quaternion = pysdf.homogeneous2translation_quaternion(model.pose_world)
  tfs.append(['map', full_name, translation, quaternion])
  rospy.loginfo("Added tf for %s" % full_name)


def publish_tf():
  for tf in tfs:
    #print(tf)
    tfBroadcaster.sendTransform(tf[2], tf[3], rospy.get_rostime(), tf[1], tf[0])


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-f', '--freq', type=float, default=10, help='Frequency TFs are published (default: 10 Hz)')
  parser.add_argument('-p', '--prefix', default='', help='Publish with prefix')
  parser.add_argument('sdf', help='SDF model to publish (e.g. coke_can)')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('sdf2tfstatic')

  global submodelsToBeIgnored
  submodelsToBeIgnored = rospy.get_param('~ignore_submodels_of', '').split(';')
  rospy.loginfo('Ignoring submodels of: ' + str(submodelsToBeIgnored))

  global tfBroadcaster
  tfBroadcaster = tf.TransformBroadcaster()

  global world
  sdf = pysdf.SDF(file=args.sdf)
  world = sdf.world

  calculate_tfs(args.prefix)

  rospy.loginfo('Spinning')
  r = rospy.Rate(args.freq)
  while not rospy.is_shutdown():
    publish_tf();
    r.sleep()


if __name__ == '__main__':
  main()
