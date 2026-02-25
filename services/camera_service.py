import depthai as dai
import cv2
import numpy as np
import time
from dataclasses import dataclass, field
from typing import Tuple, Optional

@dataclass
class CameraConfig:
    """
    Configuration artifact for the Camera Service.
    
    This data structure defines the hardware parameters for the OAK-D camera,
    serving as the explicit interface for camera initialisation.
    
    Attributes:
        resolution: The sensor resolution (e.g., THE_1080_P).
        preview_size: Tuple (width, height) for the preview stream.
        fps: Frames per second for the video stream.
        stream_name: Identifier for the XLink output stream.
    """
    resolution: dai.ColorCameraProperties.SensorResolution = dai.ColorCameraProperties.SensorResolution.THE_1080_P
    preview_size: Tuple[int, int] = (1280, 720)
    fps: int = 30
    stream_name: str = "preview"


class CameraService:
    """
    Manages the interface with the OAK-D camera hardware.
    
    This service handles the lifecycle of the DepthAI pipeline, establishing
    the connection to the device and managing the data queues for frame retrieval.
    """
    
    def __init__(self, config: CameraConfig = None):
        """
        Initialise the Camera Service.
        
        Args:
            config: Configuration object defining hardware parameters.
                    If None, default configuration is used.
        """
        self.config = config if config else CameraConfig()
        self.running = False
        self.device: Optional[dai.Device] = None
        self.queue: Optional[dai.DataOutputQueue] = None
        self.control_queue: Optional[dai.DataInputQueue] = None
        
    def _create_pipeline(self) -> dai.Pipeline:
        """
        Constructs the DepthAI processing pipeline.
        
        The pipeline consists of nodes linked together to define the flow of data
        on the OAK-D device.
        
        Nodes:
            ColorCamera: Captures RGB frames from the central sensor.
            XLinkOut: Transmits the captured frames from the device to the host via USB.
            
        Returns:
            A fully configured dai.Pipeline object.
        """
        pipeline = dai.Pipeline()

        # Define sources (Camera) and outputs (XLink)
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        xout_rgb = pipeline.create(dai.node.XLinkOut)

        # Set the stream name for the XLink output
        # This name is used by the host to tap into the specific data stream
        xout_rgb.setStreamName(self.config.stream_name)

        # Configure Camera Properties
        cam_rgb.setPreviewSize(self.config.preview_size)
        cam_rgb.setResolution(self.config.resolution)
        cam_rgb.setInterleaved(True) # Planar vs Interleaved layout for CV2 compatibility
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR) # OpenCV uses BGR
        cam_rgb.setFps(self.config.fps)

        # Input control stream for dynamic CameraControl messages (e.g., setFrameRate)
        input_control = pipeline.create(dai.node.XLinkIn)
        input_control.setStreamName("inputControl")
        # Link host input to camera's inputControl port so we can send CameraControl messages
        input_control.out.link(cam_rgb.inputControl)

        # Link the Camera's preview output to the XLink input
        # Flow: Sensor -> ColorCamera Node -> Preview Output -> XLinkOut Node -> USB -> Host
        cam_rgb.preview.link(xout_rgb.input)
        
        return pipeline

    def start(self):
        """
        Starts the camera pipeline and establishes connection to the device.
        """
        if self.running:
            return

        pipeline = self._create_pipeline()
        
        # Connect to device, start pipeline
        try:
            self.device = dai.Device(pipeline)
            
            # Get the output queue for the specific stream
            # maxSize=4: Buffer size to prevent dropped frames if host is slow
            # blocking=False: Do not block the device if the queue is full (overwrite old frames)
            self.queue = self.device.getOutputQueue(
                name=self.config.stream_name, 
                maxSize=4, 
                blocking=False
            )
            # Expose input queue for control messages
            try:
                self.control_queue = self.device.getInputQueue(name="inputControl")
            except Exception:
                # If device doesn't expose input queue, keep control_queue as None
                self.control_queue = None
            
            self.running = True
            print("Camera Service started successfully.")
        except Exception as e:
            print(f"Failed to start Camera Service: {e}")
            self.stop()

    def stop(self):
        """
        Stops the pipeline and releases the device connection.
        """
        self.running = False
        if self.device:
            self.device.close()
            self.device = None
        self.queue = None
        print("Camera Service stopped.")

    def get_frame(self) -> Optional[np.ndarray]:
        """
        Retrieves the latest available frame from the device queue.
        
        Returns:
            A numpy array representing the image (BGR format), or None if no frame is available.
        """
        if not self.running or self.queue is None:
            return None

        try:
            # tryGet() retrieves the latest message from the queue without blocking
            in_rgb = self.queue.tryGet()
            
            if in_rgb is not None:
                # Convert the DepthAI ImgFrame to an OpenCV-compatible numpy array
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
            
            # Sleep prevents busy loop if no frame is available immediately
            if frame is None:
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        pass
    finally:
        service.stop()
        cv2.destroyAllWindows()
