import './index.less';
import React, { useState, useEffect } from 'react';
import * as ROSLIB from 'roslib'

const WS_URL = "ws://192.168.20.7";
const PORT = 9090;

const IMAGE_TOPIC_CFG = {
  name: '/camera/raw/compressed',
  messageType: 'sensor_msgs/CompressedImage'
}
const DETECTION_TOPIC_CFG = {
  name: '/detection',
  messageType: '/detection'
}

const registerRos = (wsUrl: string, port: number) => {
  const ros = new ROSLIB.Ros({
    url: `${wsUrl}:${port}`
  });
  ros.on('connection', () => {
    console.log('Connected to websocket server.');
  });
  ros.on('error', (error: any) => {
    console.log('Error connecting to websocket server: ', error);
  });
  ros.on('close', () => {
    console.log('Connection to websocket server closed.');
  });
  return ros;
}

const createListener = (ros: any, { name, messageType }: {name: string, messageType: string}) => {
  const listener = new ROSLIB.Topic({
    ros, name, messageType
  });
  return listener;
}

export default function IndexPage() {
  const [imageListener, setImageListener] = useState(null);
  const [detsListener, setDetsListener] = useState(null);
  let current_results = null;

  useEffect(() => {
    const ros = registerRos(WS_URL, PORT); // init ros
    const imageListener = createListener(ros, IMAGE_TOPIC_CFG); // subscribe to image
    const detsListener = createListener(ros, DETECTION_TOPIC_CFG); // subscribe to detection results
    setImageListener(imageListener);
    setDetsListener(detsListener);

    // unsubscribe
    return () => {
      imageListener.unsubscribe();
      detsListener.unsubscribe();
    };
  }, []);

  useEffect(() => {
    if (imageListener) {
      imageListener.subscribe((message: any) => {
        const current_frame = `data:image/jpg;base64,${message.data}`;
        document.getElementById('ros-image').src = current_frame;
      });
    }
    if (detsListener) {
      detsListener.subscribe((message: any) => {
        const jsonString = JSON.stringify(message)
        const data = JSON.parse(jsonString)['data'];
        const det = JSON.parse(data);
        const results = det['results'];
        const timestamp = det['timestamp']; // timestamp
        current_results = results;
      });
    }
  });

  return (
    <div style={{ padding: 24, minHeight: 500 }}>
        <img id="ros-image" src={null} />
    </div>
  );
}


