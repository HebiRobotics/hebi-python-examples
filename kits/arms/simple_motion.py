import hebi

example_config = hebi.config.load_config("config/T25-7-dof.cfg.yaml")
arm = hebi.arm.create_from_config(example_config)
goal = hebi.arm.Goal(arm.size)
goal.add_waypoint(t=2, position=example_config.user_data['pose_1'])
goal.add_waypoint(t=4, position=example_config.user_data['pose_2'])
arm.update()
arm.set_goal(goal)

while True:
    arm.update()
    arm.send()
    if arm.at_goal:
        arm.set_goal(goal)