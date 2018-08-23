#! /usr/bin/env python
# encoding: utf-8
#
# Credits to: Res Andy 

import os, re, sys, time, socket, urllib
import rospy
import datetime
from settings import camaddr
from settings import camport

def take_photo():
  srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  srv.settimeout(5)
  rospy.loginfo('Trying to connect with yiCam @ ' + str(camaddr) + ':' + str(camport))
  srv.connect((camaddr, camport))

  srv.send('{"msg_id":257,"token":0}')

  data = srv.recv(512)
  rospy.loginfo('Got first data from yiCam, looking for "rval": ' + str(data))
  if "rval" in data:
  	token = re.findall('"param":\s*(.+)\s*}',data)[0]	
  else:
  	data = srv.recv(512)
	rospy.loginfo('Got second data from yiCam, looking for "rval": ' + str(data))
  	if "rval" in data:
  		token = re.findall('"param":\s*(.+)\s*}',data)[0]	

  rospy.loginfo('Got data from yiCam: ' + str(data))

  tosend = '{"msg_id":769,"token":%s}' %token
  srv.send(tosend)
  data = srv.recv(512)
  rospy.loginfo('Got data from yiCam: ' + str(data))


  findCollection = list()

  maxTries = 3
  i = 0

  while len(findCollection) == 0 and i < maxTries:
    data = srv.recv(512)
    rospy.loginfo('Got data from yiCam: ' + str(data))
    findCollection = re.findall('"param":"/tmp/fuse_d/(.+)"}', data)
    ++i

  if len(findCollection) == 0:
    rospy.logwarn("Yi cam photo ERROR")
    return ''

  yiCamFileName = findCollection[0]

  
  url = "http://" + str(camaddr) + "/" + yiCamFileName

  fileName = '$BAMBI_OWNCLOUD_HOME/yi-cam-pic-' + datetime.datetime.now().strftime('%Y-%m-%d-%H:%M:%S') + '.jpg'
  fileName = os.path.expandvars(fileName)

  rospy.loginfo(url)
  urllib.urlretrieve(url, filename=fileName)
  return fileName
