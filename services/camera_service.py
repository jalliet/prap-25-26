import depthai as dai
import cv2
import numpy as np
import time

class CameraService:
    def __init__(self):
        self.running = False
        self.device = None
        self.queue = None
        self.stream_name = "preview"
        
        # Configuration
        self.resolution = dai.ColorCameraProperties.SensorResolution.THE_1080_P
        self.preview_size = (1280, 720)
        self.fps = 30

    def _create_pipeline(self):
        pipeline = dai.Pipeline()

        # Define sources and outputs
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        xout_rgb = pipeline.create(dai.node.XLinkOut)

        xout_rgb.setStreamName(self.stream_name)

        # Properties
        cam_rgb.setPreviewSize(self.preview_size)
        cam_rgb.setResolution(self.resolution)
        cam_rgb.setInterleaved(True)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(self.fps)

        # Linking
        cam_rgb.preview.link(xout_rgb.input)
        
        return pipeline

    def start(self):
        if self.running:
            return

        pipeline = self._create_pipeline()
        
        # Connect to device and start pipeline
        try:
            self.device = dai.Device(pipeline)
            self.queue = self.device.getOutputQueue(name=self.stream_name, maxSize=4, blocking=False)
            self.running = True
            print("Camera Service started successfully.")
        except Exception as e:
            print(f"Failed to start Camera Service: {e}")
            self.stop()

    def stop(self):
        self.running = False
        if self.device:
            self.device.close()
            self.device = None
        self.queue = None
        print("Camera Service stopped.")

    def get_frame(self):
        if not self.running or self.queue is None:
            return None

        try:
            in_rgb = self.queue.tryGet()
            if in_rgb is not None:
                return in_rgb.getCvFrame()
        except Exception as e:
            print(f"Error retrieving frame: {e}")
        
        return None

if __name__ == "__main__":
    service = CameraService()
    service.start()
    
    print("Press 'q' to quit.")
    
    try:
        while True:
            frame = service.get_frame()
            if frame is not None:
                cv2.imshow("Camera Preview", frame)
            
            if cv2.waitKey(1) == ord('q'):
                break
            
            # Small sleep to prevent busy loop if no frame
            if frame is None:
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        pass
    finally:
        service.stop()
        cv2.destroyAllWindows()
