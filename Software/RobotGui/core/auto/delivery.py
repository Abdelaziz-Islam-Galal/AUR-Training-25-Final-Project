def delivery(color):
    from RobotGui.gui.camera_display import camera_device
    from RobotGui.gui.window import subscriber,movement_publisher
    from time import sleep
    from RobotGui.core.auto.road_to_box import box_flag
    if box_flag:
        delivery_flag=False
        _,_,angle=subscriber.coordinates
        while angle < 90 or angle > 95:
            if angle < 90:
                movement_publisher.publish_body_movement(1,90)
                sleep(0.5)
                movement_publisher.publish_body_movement(0,0)
                
            elif angle > 95:
                movement_publisher.publish_body_movement(1,270)
                sleep(0.5)
                movement_publisher.publish_body_movement(0,0)
            _,_,angle=subscriber.coordinates

        #start scanning zone
        camera_device.start_color_thread(camera_device.frame,color)
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
        while not camera_device.detector.inzone:
            movement_publisher.publish_body_movement(1,0)
            sleep(0.5)
            movement_publisher.publish_body_movement(0,0)
        camera_device.stop_color_thread()
        movement_publisher.publish_body_movement(1,0)
        sleep(1)
        movement_publisher.publish_body_movement(0,0)
        movement_publisher.publish_arm_movement(-1)
        sleep(0.2)
        movement_publisher.publish_gripper_state(0)
        _,_,angle=subscriber.coordinates
        angle_cmd=90
        while angle > 5 or angle < -5:
            if angle > 5:
                angle_cmd=270
                movement_publisher.publish_body_movement(1,angle_cmd)
                sleep(0.5)
                movement_publisher.publish_body_movement(0,0)
            elif angle < -5:
                angle_cmd=90
                movement_publisher.publish_body_movement(1,angle_cmd)
                sleep(0.5)
                movement_publisher.publish_body_movement(0,0)
        delivery_flag=True
        return delivery_flag