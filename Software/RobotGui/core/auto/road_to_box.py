def road_to_box(color):
    from RobotGui.gui.camera_display import camera_device
    from RobotGui.gui.window import subscriber,movement_publisher
    from time import sleep
    import numpy as np
    import cv2
    from RobotGui.core.auto.delivery import delivery_flag

    if delivery_flag:
        box_flag=False
        sensor=False
        def upper_frame():
            frame=camera_device.frame
            mask=np.zeros(frame.shape[:2],dtype=np.uint8)
            mask=frame[frame.shape[0]//2:,:]
            return cv2.bitwise_and(frame,frame,mask=mask)

        movement_publisher.publish_arm_movement(1)
        movement_publisher.publish_gripper_state(1)

        camera_device.start_color_thread(upper_frame(),color)
        angle_cmd=90
        while not camera_device.detected:
            
            _,_,angle=subscriber.coordinates
            if angle > 270:
                angle_cmd = 270
            elif angle < 90:
                angle_cmd = 90
            movement_publisher.publish_body_movement(1,angle_cmd)
            sleep(0.5)
            movement_publisher.publish_body_movement(0,0)
        while not sensor:
            movement_publisher.publish_body_movement(1,0)
        camera_device.stop_color_thread()
        movement_publisher.publish_arm_movement(-1)
        sleep(0.2)
        movement_publisher.publish_gripper_state(0)
        box_flag=True
        return box_flag