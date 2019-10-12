class Inspecting(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['inspection_complete'],
                            input_keys=['drone', 'rust_score_dict', 'rust_reports', 'current_windmill'],
                            output_keys=['drone', 'rust_score_dict', 'rust_reports', 'current_windmill'])

    def execute(self, userdata):
        sub_path = points_around_windmill(userdata.drone, userdata.current_windmill)
        images = []

        #  HER
        picture_target = []
        picture_target.append(subpath[-1])
        picture_target.append(subpath[int(len(subpath)/2)])

        for target in sub_path:

            # Fly to target
            userdata.drone.set_target(target.x, target.y, OPERATING_HEIGHT, yaw=target.yaw)

            #  HER
            while not is_almost_at_target(userdata.drone):
                continue

            '''
            print("is_at_yaw:")
            yaw_diff = abs(userdata.drone.yaw - userdata.drone._setpoint_msg.yaw)
            print(userdata.drone.yaw)
            print(userdata.drone._setpoint_msg.yaw)
            print(yaw_diff)
            while not is_at_yaw(userdata.drone):
                continue
            '''

            #  HER: Hvilken point er target, og går det ann å sammenligne points på denna måten?
            for i in range(len(picture_target))
                if target == picture_target[i]:
                     # Take and analyse photo
                    images.append(userdata.drone.camera.image)


        # Evaluate if/how much rust on windmill
        RUST_THRESHOLD = 50

        has_rust = False
        rust_images = []
        photo_id = 0
        for img in images:
            score = rust_score(img)

            # Lagre bilde og score
            windmill_position = userdata.current_windmill
            save_photos(windmill_position, photo_id, img, score)
            photo_id = photo_id + 1

            if score > RUST_THRESHOLD:
                has_rust = True
                rust_images.append(img)

        # Save rust score for later sorting
        #userdata.rust_score_dict[current_windmill] = rust_score

        # Save report for current windmill
        #userdata.rust_reports.append(build_rust_report_message(userdata.current_windmill, has_rust, rust_images))

        rust_report = build_rust_report_message(userdata.current_windmill, has_rust, rust_images)
        rospy.loginfo("Sending rust report for task 2")
        send_single_inspection_report(rust_report)

        return 'inspection_complete'
