#!/usr/bin/python

import argparse
from ws4py.client.threadedclient import WebSocketClient
import time
import threading
import sys
import urllib
import Queue
import json
import time
import os
import sys
import pyaudio

def rate_limited(maxPerSecond):
    minInterval = 1.0 / float(maxPerSecond)
    def decorate(func):
        lastTimeCalled = [0.0]
        def rate_limited_function(*args,**kargs):
            elapsed = time.clock() - lastTimeCalled[0]
            leftToWait = minInterval - elapsed
            if leftToWait>0:
                time.sleep(leftToWait)
            ret = func(*args,**kargs)
            lastTimeCalled[0] = time.clock()
            return ret
        return rate_limited_function
    return decorate


class KaldiClient(WebSocketClient):
    def __init__(self, url, rate, protocols=None, extensions=None, heartbeat_freq=None):
        super(KaldiClient, self).__init__(url, protocols, extensions, heartbeat_freq)
        self.audio = pyaudio.PyAudio()
        self.audio_stream = self.audio.open(format=pyaudio.paInt16, 
                channels=1, rate=rate, 
                input=True, frames_per_buffer=rate/4)
        self.audio_stream.start_stream()
        self.rate = rate


    @rate_limited(4)
    def send_data(self, data):
        self.send(data, binary=True)

    def opened(self):
        print "Socket opened!"
        def send_data_to_ws():
            while True:
                try:
                    block = self.audio_stream.read(self.rate / 4)
                    self.send(block, binary=True)
                except Exception as e:
                    print e
            self.send("EOS")

        t = threading.Thread(target=send_data_to_ws)
        t.start()


    def received_message(self, m):
        response = json.loads(str(m))
        #print >> sys.stderr, "RESPONSE:", response
        #print >> sys.stderr, "JSON was:", m
        if response['status'] == 0:
            if 'result' in response:
                trans = response['result']['hypotheses'][0]['transcript']
                print trans
                if response['result']['final']:
                    print '[Final]: ' + trans
                    #print >> sys.stderr, trans,
                    print >> sys.stderr, '\r%s' % trans.replace("\n", "\\n")
        else:
            print >> sys.stderr, "Received error from server (status %d)" % response['status']
            if 'message' in response:
                print >> sys.stderr, "Error message:",  response['message']

    def closed(self, code, reason=None):
        print 'closed!'


def main():
    uri = 'ws://localhost:8080/client/ws/speech?content-type=audio/x-raw,+layout=(string)interleaved,+rate=(int)16000,+format=(string)S16LE,+channels=(int)1'
    sample_rate = 16000
    print uri
    ws = KaldiClient(uri, rate=sample_rate)
    ws.connect()
    while(True):
        pass
    #result = ws.get_full_hyp()
    #print result.encode('utf-8')
        

if __name__ == "__main__":
    main()

