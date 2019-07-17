#!/usr/bin/python
#encoding=utf8

import json
import urllib2

username = "你的MQTT用户名"
password = "你的MQTT密码"
topic_name = "你的MQTT主题"
host = "api.mqtt.iot.bj.baidubce.com"
url = "https://" + host+"/v1/proxy?qos=0&topic="+topic_name
print url
method = "POST"


header = {
    'auth.username': username,
    'auth.password': password,
    'content-type': 'application/octet-stream',
}
data = {
  "reported": {
    "key": "value"
    }
}
data = json.dumps(data)

request = urllib2.Request(url, data, header)
response = None
try:
    response = urllib2.urlopen(request)
    post_res_str = response.read()
    print (post_res_str)
except urllib2.URLError, e:
    print "URLError"
    print e.code, e.reason
    print e.read()
except urllib2.HTTPError, e:
    print "HTTPError"
    print e.code, e.reason
    print e.read()
