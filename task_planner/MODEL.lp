\ Model Task_Planning&Scheduling
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 same_agent[('pick_orange_box_1',_'pick_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'pick_blue_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'pick_blue_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'pick_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'pick_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'place_orange_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'pick_blue_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'pick_blue_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'pick_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'pick_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'place_orange_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'place_orange_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_2',_'place_orange_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_2',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_2',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_2',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_3',_'place_orange_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_3',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_3',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_4',_'place_orange_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_4',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_4',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_4',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('place_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'pick_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'pick_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'place_blue_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'pick_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'pick_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'place_blue_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'pick_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'pick_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'place_blue_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'pick_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'pick_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'place_blue_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_white_box_1',_'pick_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_white_box_1',_'place_blue_box_1'),human_right_arm]
   + 0 same_agent[('pick_white_box_1',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_white_box_1',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_white_box_1',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_white_box_2',_'place_blue_box_1'),human_right_arm]
   + 0 same_agent[('pick_white_box_2',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_white_box_2',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_white_box_2',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('place_blue_box_1',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('place_blue_box_1',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('place_blue_box_1',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('place_blue_box_2',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('place_blue_box_2',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('place_white_box_1',_'place_white_box_2'),human_right_arm]
   + 0 both_pick_orange_box_1_pick_orange_box_2_to_ur5_on_guide
   + 0 both_pick_orange_box_1_pick_blue_box_1_to_ur5_on_guide
   + 0 both_pick_orange_box_1_pick_blue_box_2_to_ur5_on_guide
   + 0 both_pick_orange_box_1_pick_blue_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_1_pick_blue_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_1_place_orange_box_1_to_ur5_on_guide
   + 0 both_pick_orange_box_1_place_orange_box_2_to_ur5_on_guide
   + 0 both_pick_orange_box_1_place_blue_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_1_place_blue_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_2_pick_blue_box_1_to_ur5_on_guide
   + 0 both_pick_orange_box_2_pick_blue_box_2_to_ur5_on_guide
   + 0 both_pick_orange_box_2_pick_blue_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_2_pick_blue_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_2_place_orange_box_1_to_ur5_on_guide
   + 0 both_pick_orange_box_2_place_orange_box_2_to_ur5_on_guide
   + 0 both_pick_orange_box_2_place_blue_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_2_place_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_1_pick_blue_box_2_to_ur5_on_guide
   + 0 both_pick_blue_box_1_pick_blue_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_1_pick_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_1_place_orange_box_1_to_ur5_on_guide
   + 0 both_pick_blue_box_1_place_orange_box_2_to_ur5_on_guide
   + 0 both_pick_blue_box_1_place_blue_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_1_place_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_2_pick_blue_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_2_pick_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_2_place_orange_box_1_to_ur5_on_guide
   + 0 both_pick_blue_box_2_place_orange_box_2_to_ur5_on_guide
   + 0 both_pick_blue_box_2_place_blue_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_2_place_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_3_pick_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_3_place_orange_box_1_to_ur5_on_guide
   + 0 both_pick_blue_box_3_place_orange_box_2_to_ur5_on_guide
   + 0 both_pick_blue_box_3_place_blue_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_3_place_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_4_place_orange_box_1_to_ur5_on_guide
   + 0 both_pick_blue_box_4_place_orange_box_2_to_ur5_on_guide
   + 0 both_pick_blue_box_4_place_blue_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_4_place_blue_box_4_to_ur5_on_guide
   + 0 both_place_orange_box_1_place_orange_box_2_to_ur5_on_guide
   + 0 both_place_orange_box_1_place_blue_box_3_to_ur5_on_guide
   + 0 both_place_orange_box_1_place_blue_box_4_to_ur5_on_guide
   + 0 both_place_orange_box_2_place_blue_box_3_to_ur5_on_guide
   + 0 both_place_orange_box_2_place_blue_box_4_to_ur5_on_guide
   + 0 both_place_blue_box_3_place_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_1_pick_blue_box_2_to_human_right_arm
   + 0 both_pick_blue_box_1_pick_blue_box_3_to_human_right_arm
   + 0 both_pick_blue_box_1_pick_blue_box_4_to_human_right_arm
   + 0 both_pick_blue_box_1_pick_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_1_pick_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_1_place_blue_box_1_to_human_right_arm
   + 0 both_pick_blue_box_1_place_blue_box_2_to_human_right_arm
   + 0 both_pick_blue_box_1_place_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_1_place_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_2_pick_blue_box_3_to_human_right_arm
   + 0 both_pick_blue_box_2_pick_blue_box_4_to_human_right_arm
   + 0 both_pick_blue_box_2_pick_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_2_pick_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_2_place_blue_box_1_to_human_right_arm
   + 0 both_pick_blue_box_2_place_blue_box_2_to_human_right_arm
   + 0 both_pick_blue_box_2_place_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_2_place_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_3_pick_blue_box_4_to_human_right_arm
   + 0 both_pick_blue_box_3_pick_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_3_pick_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_3_place_blue_box_1_to_human_right_arm
   + 0 both_pick_blue_box_3_place_blue_box_2_to_human_right_arm
   + 0 both_pick_blue_box_3_place_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_3_place_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_4_pick_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_4_pick_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_4_place_blue_box_1_to_human_right_arm
   + 0 both_pick_blue_box_4_place_blue_box_2_to_human_right_arm
   + 0 both_pick_blue_box_4_place_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_4_place_white_box_2_to_human_right_arm
   + 0 both_pick_white_box_1_pick_white_box_2_to_human_right_arm
   + 0 both_pick_white_box_1_place_blue_box_1_to_human_right_arm
   + 0 both_pick_white_box_1_place_blue_box_2_to_human_right_arm
   + 0 both_pick_white_box_1_place_white_box_1_to_human_right_arm
   + 0 both_pick_white_box_1_place_white_box_2_to_human_right_arm
   + 0 both_pick_white_box_2_place_blue_box_1_to_human_right_arm
   + 0 both_pick_white_box_2_place_blue_box_2_to_human_right_arm
   + 0 both_pick_white_box_2_place_white_box_1_to_human_right_arm
   + 0 both_pick_white_box_2_place_white_box_2_to_human_right_arm
   + 0 both_place_blue_box_1_place_blue_box_2_to_human_right_arm
   + 0 both_place_blue_box_1_place_white_box_1_to_human_right_arm
   + 0 both_place_blue_box_1_place_white_box_2_to_human_right_arm
   + 0 both_place_blue_box_2_place_white_box_1_to_human_right_arm
   + 0 both_place_blue_box_2_place_white_box_2_to_human_right_arm
   + 0 both_place_white_box_1_place_white_box_2_to_human_right_arm
   + makespan
Subject To
 R0: - 10.40928127480102 assignment[ur5_on_guide,pick_orange_box_1]
   - t_start[pick_orange_box_1] + t_end[pick_orange_box_1] = 0
 R1: - 10.40928127480102 assignment[ur5_on_guide,pick_orange_box_2]
   - t_start[pick_orange_box_2] + t_end[pick_orange_box_2] = 0
 R2: - 8.6753970655 assignment[ur5_on_guide,pick_blue_box_1]
   - 6.195686122460001 assignment[human_right_arm,pick_blue_box_1]
   - t_start[pick_blue_box_1] + t_end[pick_blue_box_1] = 0
 R3: - 8.6753970655 assignment[ur5_on_guide,pick_blue_box_2]
   - 6.195686122460001 assignment[human_right_arm,pick_blue_box_2]
   - t_start[pick_blue_box_2] + t_end[pick_blue_box_2] = 0
 R4: - 8.6753970655 assignment[ur5_on_guide,pick_blue_box_3]
   - 6.195686122460001 assignment[human_right_arm,pick_blue_box_3]
   - t_start[pick_blue_box_3] + t_end[pick_blue_box_3] = 0
 R5: - 8.6753970655 assignment[ur5_on_guide,pick_blue_box_4]
   - 6.195686122460001 assignment[human_right_arm,pick_blue_box_4]
   - t_start[pick_blue_box_4] + t_end[pick_blue_box_4] = 0
 R6: - 8.544268380495 assignment[human_right_arm,pick_white_box_1]
   - t_start[pick_white_box_1] + t_end[pick_white_box_1] = 0
 R7: - 8.544268380495 assignment[human_right_arm,pick_white_box_2]
   - t_start[pick_white_box_2] + t_end[pick_white_box_2] = 0
 R8: - 6.715740734934011 assignment[ur5_on_guide,place_orange_box_1]
   - t_start[place_orange_box_1] + t_end[place_orange_box_1] = 0
 R9: - 6.715740734934011 assignment[ur5_on_guide,place_orange_box_2]
   - t_start[place_orange_box_2] + t_end[place_orange_box_2] = 0
 R10: - 4.032899474640001 assignment[human_right_arm,place_blue_box_1]
   - t_start[place_blue_box_1] + t_end[place_blue_box_1] = 0
 R11: - 4.032899474640001 assignment[human_right_arm,place_blue_box_2]
   - t_start[place_blue_box_2] + t_end[place_blue_box_2] = 0
 R12: - 4.859784660183673 assignment[ur5_on_guide,place_blue_box_3]
   - t_start[place_blue_box_3] + t_end[place_blue_box_3] = 0
 R13: - 4.859784660183673 assignment[ur5_on_guide,place_blue_box_4]
   - t_start[place_blue_box_4] + t_end[place_blue_box_4] = 0
 R14: - 6.01988483553 assignment[human_right_arm,place_white_box_1]
   - t_start[place_white_box_1] + t_end[place_white_box_1] = 0
 R15: - 6.01988483553 assignment[human_right_arm,place_white_box_2]
   - t_start[place_white_box_2] + t_end[place_white_box_2] = 0
 unique_assignment[pick_orange_box_1]:
   assignment[ur5_on_guide,pick_orange_box_1] = 1
 unique_assignment[pick_orange_box_2]:
   assignment[ur5_on_guide,pick_orange_box_2] = 1
 unique_assignment[pick_blue_box_1]:
   assignment[ur5_on_guide,pick_blue_box_1]
   + assignment[human_right_arm,pick_blue_box_1] = 1
 unique_assignment[pick_blue_box_2]:
   assignment[ur5_on_guide,pick_blue_box_2]
   + assignment[human_right_arm,pick_blue_box_2] = 1
 unique_assignment[pick_blue_box_3]:
   assignment[ur5_on_guide,pick_blue_box_3]
   + assignment[human_right_arm,pick_blue_box_3] = 1
 unique_assignment[pick_blue_box_4]:
   assignment[ur5_on_guide,pick_blue_box_4]
   + assignment[human_right_arm,pick_blue_box_4] = 1
 unique_assignment[pick_white_box_1]:
   assignment[human_right_arm,pick_white_box_1] = 1
 unique_assignment[pick_white_box_2]:
   assignment[human_right_arm,pick_white_box_2] = 1
 unique_assignment[place_orange_box_1]:
   assignment[ur5_on_guide,place_orange_box_1] = 1
 unique_assignment[place_orange_box_2]:
   assignment[ur5_on_guide,place_orange_box_2] = 1
 unique_assignment[place_blue_box_1]:
   assignment[human_right_arm,place_blue_box_1] = 1
 unique_assignment[place_blue_box_2]:
   assignment[human_right_arm,place_blue_box_2] = 1
 unique_assignment[place_blue_box_3]:
   assignment[ur5_on_guide,place_blue_box_3] = 1
 unique_assignment[place_blue_box_4]:
   assignment[ur5_on_guide,place_blue_box_4] = 1
 unique_assignment[place_white_box_1]:
   assignment[human_right_arm,place_white_box_1] = 1
 unique_assignment[place_white_box_2]:
   assignment[human_right_arm,place_white_box_2] = 1
 not_enabled_assignment_pick_blue_box_1:
   assignment[ur5_on_guide,pick_blue_box_1] = 0
 not_enabled_assignment_pick_blue_box_2:
   assignment[ur5_on_guide,pick_blue_box_2] = 0
 not_enabled_assignment_pick_blue_box_3:
   assignment[human_right_arm,pick_blue_box_3] = 0
 not_enabled_assignment_pick_blue_box_4:
   assignment[human_right_arm,pick_blue_box_4] = 0
 R36: t_start[place_orange_box_1] - t_end[pick_orange_box_1] = 0
 R37: t_start[place_orange_box_2] - t_end[pick_orange_box_2] = 0
 R38: t_start[place_blue_box_1] - t_end[pick_blue_box_1] = 0
 R39: t_start[place_blue_box_2] - t_end[pick_blue_box_2] = 0
 R40: t_start[place_blue_box_3] - t_end[pick_blue_box_3] = 0
 R41: t_start[place_blue_box_4] - t_end[pick_blue_box_4] = 0
 R42: t_start[place_white_box_1] - t_end[pick_white_box_1] = 0
 R43: t_start[place_white_box_2] - t_end[pick_white_box_2] = 0
 GC1: both_pick_orange_box_1_pick_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC2: both_pick_orange_box_1_pick_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_2'),ur5_on_guide]
   <= 0
 GC3: both_pick_orange_box_1_pick_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[pick_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC4: both_pick_orange_box_1_pick_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_2'),ur5_on_guide]
   >= 0
 GC6: both_pick_orange_box_1_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC7: both_pick_orange_box_1_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_1'),ur5_on_guide]
   <= 0
 GC8: both_pick_orange_box_1_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_1'),ur5_on_guide]
   >= -1e+07
 GC9: both_pick_orange_box_1_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_1'),ur5_on_guide]
   >= 0
 GC11: both_pick_orange_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC12: both_pick_orange_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_2'),ur5_on_guide]
   <= 0
 GC13: both_pick_orange_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_2'),ur5_on_guide]
   >= -1e+07
 GC14: both_pick_orange_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_2'),ur5_on_guide]
   >= 0
 GC16: both_pick_orange_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC17: both_pick_orange_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_3'),ur5_on_guide]
   <= 0
 GC18: both_pick_orange_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC19: both_pick_orange_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_3'),ur5_on_guide]
   >= 0
 GC21: both_pick_orange_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC22: both_pick_orange_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_4'),ur5_on_guide]
   <= 0
 GC23: both_pick_orange_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC24: both_pick_orange_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_4'),ur5_on_guide]
   >= 0
 GC26: both_pick_orange_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC27: both_pick_orange_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_1'),ur5_on_guide]
   <= 0
 GC28: both_pick_orange_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_1'),ur5_on_guide]
   >= -1e+07
 GC29: both_pick_orange_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_1'),ur5_on_guide]
   >= 0
 GC31: both_pick_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC32: both_pick_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC33: both_pick_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC34: both_pick_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC36: both_pick_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC37: both_pick_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC38: both_pick_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC39: both_pick_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC41: both_pick_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC42: both_pick_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC43: both_pick_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC44: both_pick_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC46: both_pick_orange_box_2_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC47: both_pick_orange_box_2_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_1'),ur5_on_guide]
   <= 0
 GC48: both_pick_orange_box_2_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_1'),ur5_on_guide]
   >= -1e+07
 GC49: both_pick_orange_box_2_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_1'),ur5_on_guide]
   >= 0
 GC51: both_pick_orange_box_2_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC52: both_pick_orange_box_2_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_2'),ur5_on_guide]
   <= 0
 GC53: both_pick_orange_box_2_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_2'),ur5_on_guide]
   >= -1e+07
 GC54: both_pick_orange_box_2_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_2'),ur5_on_guide]
   >= 0
 GC56: both_pick_orange_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC57: both_pick_orange_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_3'),ur5_on_guide]
   <= 0
 GC58: both_pick_orange_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC59: both_pick_orange_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_3'),ur5_on_guide]
   >= 0
 GC61: both_pick_orange_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC62: both_pick_orange_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_4'),ur5_on_guide]
   <= 0
 GC63: both_pick_orange_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC64: both_pick_orange_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_4'),ur5_on_guide]
   >= 0
 GC66: both_pick_orange_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC67: both_pick_orange_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_1'),ur5_on_guide]
   <= 0
 GC68: both_pick_orange_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_1'),ur5_on_guide]
   >= -1e+07
 GC69: both_pick_orange_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_1'),ur5_on_guide]
   >= 0
 GC71: both_pick_orange_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC72: both_pick_orange_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC73: both_pick_orange_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC74: both_pick_orange_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC76: both_pick_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC77: both_pick_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC78: both_pick_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC79: both_pick_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC81: both_pick_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC82: both_pick_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC83: both_pick_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC84: both_pick_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC86: both_pick_blue_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC87: both_pick_blue_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),ur5_on_guide]
   <= 0
 GC88: both_pick_blue_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),ur5_on_guide]
   >= -1e+07
 GC89: both_pick_blue_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),ur5_on_guide]
   >= 0
 GC91: both_pick_blue_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC92: both_pick_blue_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),ur5_on_guide]
   <= 0
 GC93: both_pick_blue_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC94: both_pick_blue_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),ur5_on_guide]
   >= 0
 GC96: both_pick_blue_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC97: both_pick_blue_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),ur5_on_guide]
   <= 0
 GC98: both_pick_blue_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC99: both_pick_blue_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),ur5_on_guide]
   >= 0
 GC101: both_pick_blue_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC102: both_pick_blue_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_1'),ur5_on_guide]
   <= 0
 GC103: both_pick_blue_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_1'),ur5_on_guide]
   >= -1e+07
 GC104: both_pick_blue_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_1'),ur5_on_guide]
   >= 0
 GC106: both_pick_blue_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC107: both_pick_blue_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC108: both_pick_blue_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC109: both_pick_blue_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC111: both_pick_blue_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC112: both_pick_blue_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC113: both_pick_blue_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC114: both_pick_blue_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC116: both_pick_blue_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC117: both_pick_blue_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC118: both_pick_blue_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC119: both_pick_blue_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC121: both_pick_blue_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC122: both_pick_blue_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),ur5_on_guide]
   <= 0
 GC123: both_pick_blue_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC124: both_pick_blue_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),ur5_on_guide]
   >= 0
 GC126: both_pick_blue_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC127: both_pick_blue_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),ur5_on_guide]
   <= 0
 GC128: both_pick_blue_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC129: both_pick_blue_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),ur5_on_guide]
   >= 0
 GC131: both_pick_blue_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC132: both_pick_blue_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_1'),ur5_on_guide]
   <= 0
 GC133: both_pick_blue_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_1'),ur5_on_guide]
   >= -1e+07
 GC134: both_pick_blue_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_1'),ur5_on_guide]
   >= 0
 GC136: both_pick_blue_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC137: both_pick_blue_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC138: both_pick_blue_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC139: both_pick_blue_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC141: both_pick_blue_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC142: both_pick_blue_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC143: both_pick_blue_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC144: both_pick_blue_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC146: both_pick_blue_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC147: both_pick_blue_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC148: both_pick_blue_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC149: both_pick_blue_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC151: both_pick_blue_box_3_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC152: both_pick_blue_box_3_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),ur5_on_guide]
   <= 0
 GC153: both_pick_blue_box_3_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC154: both_pick_blue_box_3_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),ur5_on_guide]
   >= 0
 GC156: both_pick_blue_box_3_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC157: both_pick_blue_box_3_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_1'),ur5_on_guide]
   <= 0
 GC158: both_pick_blue_box_3_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_1'),ur5_on_guide]
   >= -1e+07
 GC159: both_pick_blue_box_3_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_1'),ur5_on_guide]
   >= 0
 GC161: both_pick_blue_box_3_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC162: both_pick_blue_box_3_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC163: both_pick_blue_box_3_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC164: both_pick_blue_box_3_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC166: both_pick_blue_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC167: both_pick_blue_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC168: both_pick_blue_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC169: both_pick_blue_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC171: both_pick_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC172: both_pick_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC173: both_pick_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC174: both_pick_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC176: both_pick_blue_box_4_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC177: both_pick_blue_box_4_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_1'),ur5_on_guide]
   <= 0
 GC178: both_pick_blue_box_4_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_1'),ur5_on_guide]
   >= -1e+07
 GC179: both_pick_blue_box_4_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_1'),ur5_on_guide]
   >= 0
 GC181: both_pick_blue_box_4_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC182: both_pick_blue_box_4_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC183: both_pick_blue_box_4_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC184: both_pick_blue_box_4_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC186: both_pick_blue_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC187: both_pick_blue_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC188: both_pick_blue_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC189: both_pick_blue_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC191: both_pick_blue_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC192: both_pick_blue_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC193: both_pick_blue_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC194: both_pick_blue_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC196: both_place_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC197: both_place_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC198: both_place_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC199: both_place_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[place_orange_box_1]
   + 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC201: both_place_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC202: both_place_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC203: both_place_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC204: both_place_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[place_orange_box_1]
   + 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC206: both_place_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC207: both_place_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC208: both_place_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC209: both_place_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[place_orange_box_1]
   + 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC211: both_place_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC212: both_place_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC213: both_place_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC214: both_place_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[place_orange_box_2]
   + 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC216: both_place_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC217: both_place_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC218: both_place_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC219: both_place_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[place_orange_box_2]
   + 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC221: both_place_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC222: both_place_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC223: both_place_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('place_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC224: both_place_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[place_blue_box_3]
   + 1e+07 same_agent[('place_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC226: both_pick_blue_box_1_pick_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC227: both_pick_blue_box_1_pick_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),human_right_arm]
   <= 0
 GC228: both_pick_blue_box_1_pick_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),human_right_arm]
   >= -1e+07
 GC229: both_pick_blue_box_1_pick_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),human_right_arm]
   >= 0
 GC231: both_pick_blue_box_1_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC232: both_pick_blue_box_1_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),human_right_arm]
   <= 0
 GC233: both_pick_blue_box_1_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),human_right_arm]
   >= -1e+07
 GC234: both_pick_blue_box_1_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),human_right_arm]
   >= 0
 GC236: both_pick_blue_box_1_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC237: both_pick_blue_box_1_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),human_right_arm]
   <= 0
 GC238: both_pick_blue_box_1_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),human_right_arm]
   >= -1e+07
 GC239: both_pick_blue_box_1_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),human_right_arm]
   >= 0
 GC241: both_pick_blue_box_1_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC242: both_pick_blue_box_1_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_1'),human_right_arm]
   <= 0
 GC243: both_pick_blue_box_1_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_1'),human_right_arm]
   >= -1e+07
 GC244: both_pick_blue_box_1_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_1'),human_right_arm]
   >= 0
 GC246: both_pick_blue_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC247: both_pick_blue_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_2'),human_right_arm]
   <= 0
 GC248: both_pick_blue_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_2'),human_right_arm]
   >= -1e+07
 GC249: both_pick_blue_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_2'),human_right_arm]
   >= 0
 GC251: both_pick_blue_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC252: both_pick_blue_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_1'),human_right_arm]
   <= 0
 GC253: both_pick_blue_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_1'),human_right_arm]
   >= -1e+07
 GC254: both_pick_blue_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_1'),human_right_arm]
   >= 0
 GC256: both_pick_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC257: both_pick_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC258: both_pick_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC259: both_pick_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC261: both_pick_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC262: both_pick_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_1'),human_right_arm]
   <= 0
 GC263: both_pick_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC264: both_pick_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_1'),human_right_arm]
   >= 0
 GC266: both_pick_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC267: both_pick_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_2'),human_right_arm]
   <= 0
 GC268: both_pick_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC269: both_pick_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_2'),human_right_arm]
   >= 0
 GC271: both_pick_blue_box_2_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC272: both_pick_blue_box_2_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),human_right_arm]
   <= 0
 GC273: both_pick_blue_box_2_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),human_right_arm]
   >= -1e+07
 GC274: both_pick_blue_box_2_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),human_right_arm]
   >= 0
 GC276: both_pick_blue_box_2_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC277: both_pick_blue_box_2_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),human_right_arm]
   <= 0
 GC278: both_pick_blue_box_2_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),human_right_arm]
   >= -1e+07
 GC279: both_pick_blue_box_2_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),human_right_arm]
   >= 0
 GC281: both_pick_blue_box_2_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC282: both_pick_blue_box_2_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_1'),human_right_arm]
   <= 0
 GC283: both_pick_blue_box_2_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_1'),human_right_arm]
   >= -1e+07
 GC284: both_pick_blue_box_2_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_1'),human_right_arm]
   >= 0
 GC286: both_pick_blue_box_2_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC287: both_pick_blue_box_2_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_2'),human_right_arm]
   <= 0
 GC288: both_pick_blue_box_2_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_2'),human_right_arm]
   >= -1e+07
 GC289: both_pick_blue_box_2_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_2'),human_right_arm]
   >= 0
 GC291: both_pick_blue_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC292: both_pick_blue_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_1'),human_right_arm]
   <= 0
 GC293: both_pick_blue_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_1'),human_right_arm]
   >= -1e+07
 GC294: both_pick_blue_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_1'),human_right_arm]
   >= 0
 GC296: both_pick_blue_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC297: both_pick_blue_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC298: both_pick_blue_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC299: both_pick_blue_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC301: both_pick_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC302: both_pick_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_1'),human_right_arm]
   <= 0
 GC303: both_pick_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC304: both_pick_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_1'),human_right_arm]
   >= 0
 GC306: both_pick_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC307: both_pick_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_2'),human_right_arm]
   <= 0
 GC308: both_pick_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC309: both_pick_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_2'),human_right_arm]
   >= 0
 GC311: both_pick_blue_box_3_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC312: both_pick_blue_box_3_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),human_right_arm]
   <= 0
 GC313: both_pick_blue_box_3_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),human_right_arm]
   >= -1e+07
 GC314: both_pick_blue_box_3_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),human_right_arm]
   >= 0
 GC316: both_pick_blue_box_3_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC317: both_pick_blue_box_3_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_1'),human_right_arm]
   <= 0
 GC318: both_pick_blue_box_3_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_1'),human_right_arm]
   >= -1e+07
 GC319: both_pick_blue_box_3_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_1'),human_right_arm]
   >= 0
 GC321: both_pick_blue_box_3_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC322: both_pick_blue_box_3_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_2'),human_right_arm]
   <= 0
 GC323: both_pick_blue_box_3_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_2'),human_right_arm]
   >= -1e+07
 GC324: both_pick_blue_box_3_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_2'),human_right_arm]
   >= 0
 GC326: both_pick_blue_box_3_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC327: both_pick_blue_box_3_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_1'),human_right_arm]
   <= 0
 GC328: both_pick_blue_box_3_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_1'),human_right_arm]
   >= -1e+07
 GC329: both_pick_blue_box_3_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_1'),human_right_arm]
   >= 0
 GC331: both_pick_blue_box_3_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC332: both_pick_blue_box_3_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC333: both_pick_blue_box_3_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC334: both_pick_blue_box_3_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC336: both_pick_blue_box_3_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC337: both_pick_blue_box_3_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_1'),human_right_arm]
   <= 0
 GC338: both_pick_blue_box_3_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC339: both_pick_blue_box_3_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_1'),human_right_arm]
   >= 0
 GC341: both_pick_blue_box_3_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC342: both_pick_blue_box_3_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_2'),human_right_arm]
   <= 0
 GC343: both_pick_blue_box_3_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC344: both_pick_blue_box_3_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_2'),human_right_arm]
   >= 0
 GC346: both_pick_blue_box_4_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC347: both_pick_blue_box_4_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_1'),human_right_arm]
   <= 0
 GC348: both_pick_blue_box_4_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_1'),human_right_arm]
   >= -1e+07
 GC349: both_pick_blue_box_4_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_1'),human_right_arm]
   >= 0
 GC351: both_pick_blue_box_4_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC352: both_pick_blue_box_4_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_2'),human_right_arm]
   <= 0
 GC353: both_pick_blue_box_4_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_2'),human_right_arm]
   >= -1e+07
 GC354: both_pick_blue_box_4_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_2'),human_right_arm]
   >= 0
 GC356: both_pick_blue_box_4_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC357: both_pick_blue_box_4_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_1'),human_right_arm]
   <= 0
 GC358: both_pick_blue_box_4_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_1'),human_right_arm]
   >= -1e+07
 GC359: both_pick_blue_box_4_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_1'),human_right_arm]
   >= 0
 GC361: both_pick_blue_box_4_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC362: both_pick_blue_box_4_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC363: both_pick_blue_box_4_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC364: both_pick_blue_box_4_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC366: both_pick_blue_box_4_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC367: both_pick_blue_box_4_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_1'),human_right_arm]
   <= 0
 GC368: both_pick_blue_box_4_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC369: both_pick_blue_box_4_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_1'),human_right_arm]
   >= 0
 GC371: both_pick_blue_box_4_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC372: both_pick_blue_box_4_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_2'),human_right_arm]
   <= 0
 GC373: both_pick_blue_box_4_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC374: both_pick_blue_box_4_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_2'),human_right_arm]
   >= 0
 GC376: both_pick_white_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC377: both_pick_white_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_2'),human_right_arm]
   <= 0
 GC378: both_pick_white_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[pick_white_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_2'),human_right_arm]
   >= -1e+07
 GC379: both_pick_white_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[pick_white_box_1]
   + 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_2'),human_right_arm]
   >= 0
 GC381: both_pick_white_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC382: both_pick_white_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_1'),human_right_arm]
   <= 0
 GC383: both_pick_white_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_1'),human_right_arm]
   >= -1e+07
 GC384: both_pick_white_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[pick_white_box_1]
   + 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_1'),human_right_arm]
   >= 0
 GC386: both_pick_white_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC387: both_pick_white_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC388: both_pick_white_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC389: both_pick_white_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[pick_white_box_1]
   + 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC391: both_pick_white_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC392: both_pick_white_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_1'),human_right_arm]
   <= 0
 GC393: both_pick_white_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC394: both_pick_white_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[pick_white_box_1]
   + 1e+07 same_agent[('pick_white_box_1',_'place_white_box_1'),human_right_arm]
   >= 0
 GC396: both_pick_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC397: both_pick_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_2'),human_right_arm]
   <= 0
 GC398: both_pick_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC399: both_pick_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[pick_white_box_1]
   + 1e+07 same_agent[('pick_white_box_1',_'place_white_box_2'),human_right_arm]
   >= 0
 GC401: both_pick_white_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC402: both_pick_white_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_1'),human_right_arm]
   <= 0
 GC403: both_pick_white_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_1'),human_right_arm]
   >= -1e+07
 GC404: both_pick_white_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[pick_white_box_2]
   + 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_1'),human_right_arm]
   >= 0
 GC406: both_pick_white_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC407: both_pick_white_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC408: both_pick_white_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC409: both_pick_white_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[pick_white_box_2]
   + 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC411: both_pick_white_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC412: both_pick_white_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_1'),human_right_arm]
   <= 0
 GC413: both_pick_white_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC414: both_pick_white_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[pick_white_box_2]
   + 1e+07 same_agent[('pick_white_box_2',_'place_white_box_1'),human_right_arm]
   >= 0
 GC416: both_pick_white_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC417: both_pick_white_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_2'),human_right_arm]
   <= 0
 GC418: both_pick_white_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC419: both_pick_white_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[pick_white_box_2]
   + 1e+07 same_agent[('pick_white_box_2',_'place_white_box_2'),human_right_arm]
   >= 0
 GC421: both_place_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('place_blue_box_1',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC422: both_place_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('place_blue_box_1',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC423: both_place_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('place_blue_box_1',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC424: both_place_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[place_blue_box_1]
   + 1e+07 same_agent[('place_blue_box_1',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC426: both_place_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_white_box_1]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC427: both_place_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_white_box_1]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_1'),human_right_arm]
   <= 0
 GC428: both_place_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[place_white_box_1]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC429: both_place_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[place_blue_box_1]
   + 1e+07 same_agent[('place_blue_box_1',_'place_white_box_1'),human_right_arm]
   >= 0
 GC431: both_place_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC432: both_place_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_2'),human_right_arm]
   <= 0
 GC433: both_place_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[place_white_box_2]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC434: both_place_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[place_blue_box_1]
   + 1e+07 same_agent[('place_blue_box_1',_'place_white_box_2'),human_right_arm]
   >= 0
 GC436: both_place_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_start[place_white_box_1]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC437: both_place_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_start[place_white_box_1]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_1'),human_right_arm]
   <= 0
 GC438: both_place_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[place_white_box_1]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC439: both_place_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[place_blue_box_2]
   + 1e+07 same_agent[('place_blue_box_2',_'place_white_box_1'),human_right_arm]
   >= 0
 GC441: both_place_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_start[place_white_box_2]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC442: both_place_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_start[place_white_box_2]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_2'),human_right_arm]
   <= 0
 GC443: both_place_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[place_white_box_2]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC444: both_place_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[place_blue_box_2]
   + 1e+07 same_agent[('place_blue_box_2',_'place_white_box_2'),human_right_arm]
   >= 0
 GC446: both_place_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('place_white_box_1',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC447: both_place_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('place_white_box_1',_'place_white_box_2'),human_right_arm]
   <= 0
 GC448: both_place_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[place_white_box_2]
   - 1e+07 same_agent[('place_white_box_1',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC449: both_place_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[place_white_box_1]
   + 1e+07 same_agent[('place_white_box_1',_'place_white_box_2'),human_right_arm]
   >= 0
Bounds
 t_start[pick_orange_box_1] <= 1158.653069831674
 t_start[pick_orange_box_2] <= 1158.653069831674
 t_start[pick_blue_box_1] <= 1158.653069831674
 t_start[pick_blue_box_2] <= 1158.653069831674
 t_start[pick_blue_box_3] <= 1158.653069831674
 t_start[pick_blue_box_4] <= 1158.653069831674
 t_start[pick_white_box_1] <= 1158.653069831674
 t_start[pick_white_box_2] <= 1158.653069831674
 t_start[place_orange_box_1] <= 1158.653069831674
 t_start[place_orange_box_2] <= 1158.653069831674
 t_start[place_blue_box_1] <= 1158.653069831674
 t_start[place_blue_box_2] <= 1158.653069831674
 t_start[place_blue_box_3] <= 1158.653069831674
 t_start[place_blue_box_4] <= 1158.653069831674
 t_start[place_white_box_1] <= 1158.653069831674
 t_start[place_white_box_2] <= 1158.653069831674
 t_end[pick_orange_box_1] <= 1158.653069831674
 t_end[pick_orange_box_2] <= 1158.653069831674
 t_end[pick_blue_box_1] <= 1158.653069831674
 t_end[pick_blue_box_2] <= 1158.653069831674
 t_end[pick_blue_box_3] <= 1158.653069831674
 t_end[pick_blue_box_4] <= 1158.653069831674
 t_end[pick_white_box_1] <= 1158.653069831674
 t_end[pick_white_box_2] <= 1158.653069831674
 t_end[place_orange_box_1] <= 1158.653069831674
 t_end[place_orange_box_2] <= 1158.653069831674
 t_end[place_blue_box_1] <= 1158.653069831674
 t_end[place_blue_box_2] <= 1158.653069831674
 t_end[place_blue_box_3] <= 1158.653069831674
 t_end[place_blue_box_4] <= 1158.653069831674
 t_end[place_white_box_1] <= 1158.653069831674
 t_end[place_white_box_2] <= 1158.653069831674
Binaries
 assignment[ur5_on_guide,pick_orange_box_1]
 assignment[ur5_on_guide,pick_orange_box_2]
 assignment[ur5_on_guide,pick_blue_box_1]
 assignment[human_right_arm,pick_blue_box_1]
 assignment[ur5_on_guide,pick_blue_box_2]
 assignment[human_right_arm,pick_blue_box_2]
 assignment[ur5_on_guide,pick_blue_box_3]
 assignment[human_right_arm,pick_blue_box_3]
 assignment[ur5_on_guide,pick_blue_box_4]
 assignment[human_right_arm,pick_blue_box_4]
 assignment[human_right_arm,pick_white_box_1]
 assignment[human_right_arm,pick_white_box_2]
 assignment[ur5_on_guide,place_orange_box_1]
 assignment[ur5_on_guide,place_orange_box_2]
 assignment[human_right_arm,place_blue_box_1]
 assignment[human_right_arm,place_blue_box_2]
 assignment[ur5_on_guide,place_blue_box_3]
 assignment[ur5_on_guide,place_blue_box_4]
 assignment[human_right_arm,place_white_box_1]
 assignment[human_right_arm,place_white_box_2]
 same_agent[('pick_orange_box_1',_'pick_orange_box_2'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'pick_blue_box_1'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'pick_blue_box_2'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'pick_blue_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'pick_blue_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'place_orange_box_1'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'pick_blue_box_1'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'pick_blue_box_2'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'pick_blue_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'pick_blue_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'place_orange_box_1'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'place_orange_box_1'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_2',_'place_orange_box_1'),ur5_on_guide]
 same_agent[('pick_blue_box_2',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('pick_blue_box_2',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_2',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_3',_'place_orange_box_1'),ur5_on_guide]
 same_agent[('pick_blue_box_3',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('pick_blue_box_3',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_4',_'place_orange_box_1'),ur5_on_guide]
 same_agent[('pick_blue_box_4',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('pick_blue_box_4',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_4',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('place_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('place_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('place_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('place_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('place_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('place_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),human_right_arm]
 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),human_right_arm]
 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),human_right_arm]
 same_agent[('pick_blue_box_1',_'pick_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_1',_'pick_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_1',_'place_blue_box_1'),human_right_arm]
 same_agent[('pick_blue_box_1',_'place_blue_box_2'),human_right_arm]
 same_agent[('pick_blue_box_1',_'place_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_1',_'place_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),human_right_arm]
 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),human_right_arm]
 same_agent[('pick_blue_box_2',_'pick_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_2',_'pick_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_2',_'place_blue_box_1'),human_right_arm]
 same_agent[('pick_blue_box_2',_'place_blue_box_2'),human_right_arm]
 same_agent[('pick_blue_box_2',_'place_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_2',_'place_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),human_right_arm]
 same_agent[('pick_blue_box_3',_'pick_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_3',_'pick_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_3',_'place_blue_box_1'),human_right_arm]
 same_agent[('pick_blue_box_3',_'place_blue_box_2'),human_right_arm]
 same_agent[('pick_blue_box_3',_'place_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_3',_'place_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_4',_'pick_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_4',_'pick_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_4',_'place_blue_box_1'),human_right_arm]
 same_agent[('pick_blue_box_4',_'place_blue_box_2'),human_right_arm]
 same_agent[('pick_blue_box_4',_'place_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_4',_'place_white_box_2'),human_right_arm]
 same_agent[('pick_white_box_1',_'pick_white_box_2'),human_right_arm]
 same_agent[('pick_white_box_1',_'place_blue_box_1'),human_right_arm]
 same_agent[('pick_white_box_1',_'place_blue_box_2'),human_right_arm]
 same_agent[('pick_white_box_1',_'place_white_box_1'),human_right_arm]
 same_agent[('pick_white_box_1',_'place_white_box_2'),human_right_arm]
 same_agent[('pick_white_box_2',_'place_blue_box_1'),human_right_arm]
 same_agent[('pick_white_box_2',_'place_blue_box_2'),human_right_arm]
 same_agent[('pick_white_box_2',_'place_white_box_1'),human_right_arm]
 same_agent[('pick_white_box_2',_'place_white_box_2'),human_right_arm]
 same_agent[('place_blue_box_1',_'place_blue_box_2'),human_right_arm]
 same_agent[('place_blue_box_1',_'place_white_box_1'),human_right_arm]
 same_agent[('place_blue_box_1',_'place_white_box_2'),human_right_arm]
 same_agent[('place_blue_box_2',_'place_white_box_1'),human_right_arm]
 same_agent[('place_blue_box_2',_'place_white_box_2'),human_right_arm]
 same_agent[('place_white_box_1',_'place_white_box_2'),human_right_arm]
 both_pick_orange_box_1_pick_orange_box_2_to_ur5_on_guide
 both_pick_orange_box_1_pick_blue_box_1_to_ur5_on_guide
 both_pick_orange_box_1_pick_blue_box_2_to_ur5_on_guide
 both_pick_orange_box_1_pick_blue_box_3_to_ur5_on_guide
 both_pick_orange_box_1_pick_blue_box_4_to_ur5_on_guide
 both_pick_orange_box_1_place_orange_box_1_to_ur5_on_guide
 both_pick_orange_box_1_place_orange_box_2_to_ur5_on_guide
 both_pick_orange_box_1_place_blue_box_3_to_ur5_on_guide
 both_pick_orange_box_1_place_blue_box_4_to_ur5_on_guide
 both_pick_orange_box_2_pick_blue_box_1_to_ur5_on_guide
 both_pick_orange_box_2_pick_blue_box_2_to_ur5_on_guide
 both_pick_orange_box_2_pick_blue_box_3_to_ur5_on_guide
 both_pick_orange_box_2_pick_blue_box_4_to_ur5_on_guide
 both_pick_orange_box_2_place_orange_box_1_to_ur5_on_guide
 both_pick_orange_box_2_place_orange_box_2_to_ur5_on_guide
 both_pick_orange_box_2_place_blue_box_3_to_ur5_on_guide
 both_pick_orange_box_2_place_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_1_pick_blue_box_2_to_ur5_on_guide
 both_pick_blue_box_1_pick_blue_box_3_to_ur5_on_guide
 both_pick_blue_box_1_pick_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_1_place_orange_box_1_to_ur5_on_guide
 both_pick_blue_box_1_place_orange_box_2_to_ur5_on_guide
 both_pick_blue_box_1_place_blue_box_3_to_ur5_on_guide
 both_pick_blue_box_1_place_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_2_pick_blue_box_3_to_ur5_on_guide
 both_pick_blue_box_2_pick_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_2_place_orange_box_1_to_ur5_on_guide
 both_pick_blue_box_2_place_orange_box_2_to_ur5_on_guide
 both_pick_blue_box_2_place_blue_box_3_to_ur5_on_guide
 both_pick_blue_box_2_place_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_3_pick_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_3_place_orange_box_1_to_ur5_on_guide
 both_pick_blue_box_3_place_orange_box_2_to_ur5_on_guide
 both_pick_blue_box_3_place_blue_box_3_to_ur5_on_guide
 both_pick_blue_box_3_place_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_4_place_orange_box_1_to_ur5_on_guide
 both_pick_blue_box_4_place_orange_box_2_to_ur5_on_guide
 both_pick_blue_box_4_place_blue_box_3_to_ur5_on_guide
 both_pick_blue_box_4_place_blue_box_4_to_ur5_on_guide
 both_place_orange_box_1_place_orange_box_2_to_ur5_on_guide
 both_place_orange_box_1_place_blue_box_3_to_ur5_on_guide
 both_place_orange_box_1_place_blue_box_4_to_ur5_on_guide
 both_place_orange_box_2_place_blue_box_3_to_ur5_on_guide
 both_place_orange_box_2_place_blue_box_4_to_ur5_on_guide
 both_place_blue_box_3_place_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_1_pick_blue_box_2_to_human_right_arm
 both_pick_blue_box_1_pick_blue_box_3_to_human_right_arm
 both_pick_blue_box_1_pick_blue_box_4_to_human_right_arm
 both_pick_blue_box_1_pick_white_box_1_to_human_right_arm
 both_pick_blue_box_1_pick_white_box_2_to_human_right_arm
 both_pick_blue_box_1_place_blue_box_1_to_human_right_arm
 both_pick_blue_box_1_place_blue_box_2_to_human_right_arm
 both_pick_blue_box_1_place_white_box_1_to_human_right_arm
 both_pick_blue_box_1_place_white_box_2_to_human_right_arm
 both_pick_blue_box_2_pick_blue_box_3_to_human_right_arm
 both_pick_blue_box_2_pick_blue_box_4_to_human_right_arm
 both_pick_blue_box_2_pick_white_box_1_to_human_right_arm
 both_pick_blue_box_2_pick_white_box_2_to_human_right_arm
 both_pick_blue_box_2_place_blue_box_1_to_human_right_arm
 both_pick_blue_box_2_place_blue_box_2_to_human_right_arm
 both_pick_blue_box_2_place_white_box_1_to_human_right_arm
 both_pick_blue_box_2_place_white_box_2_to_human_right_arm
 both_pick_blue_box_3_pick_blue_box_4_to_human_right_arm
 both_pick_blue_box_3_pick_white_box_1_to_human_right_arm
 both_pick_blue_box_3_pick_white_box_2_to_human_right_arm
 both_pick_blue_box_3_place_blue_box_1_to_human_right_arm
 both_pick_blue_box_3_place_blue_box_2_to_human_right_arm
 both_pick_blue_box_3_place_white_box_1_to_human_right_arm
 both_pick_blue_box_3_place_white_box_2_to_human_right_arm
 both_pick_blue_box_4_pick_white_box_1_to_human_right_arm
 both_pick_blue_box_4_pick_white_box_2_to_human_right_arm
 both_pick_blue_box_4_place_blue_box_1_to_human_right_arm
 both_pick_blue_box_4_place_blue_box_2_to_human_right_arm
 both_pick_blue_box_4_place_white_box_1_to_human_right_arm
 both_pick_blue_box_4_place_white_box_2_to_human_right_arm
 both_pick_white_box_1_pick_white_box_2_to_human_right_arm
 both_pick_white_box_1_place_blue_box_1_to_human_right_arm
 both_pick_white_box_1_place_blue_box_2_to_human_right_arm
 both_pick_white_box_1_place_white_box_1_to_human_right_arm
 both_pick_white_box_1_place_white_box_2_to_human_right_arm
 both_pick_white_box_2_place_blue_box_1_to_human_right_arm
 both_pick_white_box_2_place_blue_box_2_to_human_right_arm
 both_pick_white_box_2_place_white_box_1_to_human_right_arm
 both_pick_white_box_2_place_white_box_2_to_human_right_arm
 both_place_blue_box_1_place_blue_box_2_to_human_right_arm
 both_place_blue_box_1_place_white_box_1_to_human_right_arm
 both_place_blue_box_1_place_white_box_2_to_human_right_arm
 both_place_blue_box_2_place_white_box_1_to_human_right_arm
 both_place_blue_box_2_place_white_box_2_to_human_right_arm
 both_place_white_box_1_place_white_box_2_to_human_right_arm
General Constraints
 GC0: both_pick_orange_box_1_pick_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,pick_orange_box_2] )
 GC5: both_pick_orange_box_1_pick_blue_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,pick_blue_box_1] )
 GC10: both_pick_orange_box_1_pick_blue_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,pick_blue_box_2] )
 GC15: both_pick_orange_box_1_pick_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,pick_blue_box_3] )
 GC20: both_pick_orange_box_1_pick_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,pick_blue_box_4] )
 GC25: both_pick_orange_box_1_place_orange_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,place_orange_box_1] )
 GC30: both_pick_orange_box_1_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC35: both_pick_orange_box_1_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC40: both_pick_orange_box_1_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC45: both_pick_orange_box_2_pick_blue_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,pick_blue_box_1] )
 GC50: both_pick_orange_box_2_pick_blue_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,pick_blue_box_2] )
 GC55: both_pick_orange_box_2_pick_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,pick_blue_box_3] )
 GC60: both_pick_orange_box_2_pick_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,pick_blue_box_4] )
 GC65: both_pick_orange_box_2_place_orange_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,place_orange_box_1] )
 GC70: both_pick_orange_box_2_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC75: both_pick_orange_box_2_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC80: both_pick_orange_box_2_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC85: both_pick_blue_box_1_pick_blue_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,pick_blue_box_2] )
 GC90: both_pick_blue_box_1_pick_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,pick_blue_box_3] )
 GC95: both_pick_blue_box_1_pick_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,pick_blue_box_4] )
 GC100: both_pick_blue_box_1_place_orange_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,place_orange_box_1] )
 GC105: both_pick_blue_box_1_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC110: both_pick_blue_box_1_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC115: both_pick_blue_box_1_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC120: both_pick_blue_box_2_pick_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_2] ,
   assignment[ur5_on_guide,pick_blue_box_3] )
 GC125: both_pick_blue_box_2_pick_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_2] ,
   assignment[ur5_on_guide,pick_blue_box_4] )
 GC130: both_pick_blue_box_2_place_orange_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_2] ,
   assignment[ur5_on_guide,place_orange_box_1] )
 GC135: both_pick_blue_box_2_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_2] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC140: both_pick_blue_box_2_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_2] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC145: both_pick_blue_box_2_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_2] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC150: both_pick_blue_box_3_pick_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_3] ,
   assignment[ur5_on_guide,pick_blue_box_4] )
 GC155: both_pick_blue_box_3_place_orange_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_3] ,
   assignment[ur5_on_guide,place_orange_box_1] )
 GC160: both_pick_blue_box_3_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_3] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC165: both_pick_blue_box_3_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_3] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC170: both_pick_blue_box_3_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_3] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC175: both_pick_blue_box_4_place_orange_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_4] ,
   assignment[ur5_on_guide,place_orange_box_1] )
 GC180: both_pick_blue_box_4_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_4] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC185: both_pick_blue_box_4_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_4] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC190: both_pick_blue_box_4_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_4] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC195: both_place_orange_box_1_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_1] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC200: both_place_orange_box_1_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_1] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC205: both_place_orange_box_1_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_1] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC210: both_place_orange_box_2_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_2] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC215: both_place_orange_box_2_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_2] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC220: both_place_blue_box_3_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_blue_box_3] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC225: both_pick_blue_box_1_pick_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,pick_blue_box_2] )
 GC230: both_pick_blue_box_1_pick_blue_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,pick_blue_box_3] )
 GC235: both_pick_blue_box_1_pick_blue_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,pick_blue_box_4] )
 GC240: both_pick_blue_box_1_pick_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,pick_white_box_1] )
 GC245: both_pick_blue_box_1_pick_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,pick_white_box_2] )
 GC250: both_pick_blue_box_1_place_blue_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,place_blue_box_1] )
 GC255: both_pick_blue_box_1_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC260: both_pick_blue_box_1_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,place_white_box_1] )
 GC265: both_pick_blue_box_1_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,place_white_box_2] )
 GC270: both_pick_blue_box_2_pick_blue_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,pick_blue_box_3] )
 GC275: both_pick_blue_box_2_pick_blue_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,pick_blue_box_4] )
 GC280: both_pick_blue_box_2_pick_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,pick_white_box_1] )
 GC285: both_pick_blue_box_2_pick_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,pick_white_box_2] )
 GC290: both_pick_blue_box_2_place_blue_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,place_blue_box_1] )
 GC295: both_pick_blue_box_2_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC300: both_pick_blue_box_2_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,place_white_box_1] )
 GC305: both_pick_blue_box_2_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,place_white_box_2] )
 GC310: both_pick_blue_box_3_pick_blue_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,pick_blue_box_4] )
 GC315: both_pick_blue_box_3_pick_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,pick_white_box_1] )
 GC320: both_pick_blue_box_3_pick_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,pick_white_box_2] )
 GC325: both_pick_blue_box_3_place_blue_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,place_blue_box_1] )
 GC330: both_pick_blue_box_3_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC335: both_pick_blue_box_3_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,place_white_box_1] )
 GC340: both_pick_blue_box_3_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,place_white_box_2] )
 GC345: both_pick_blue_box_4_pick_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,pick_white_box_1] )
 GC350: both_pick_blue_box_4_pick_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,pick_white_box_2] )
 GC355: both_pick_blue_box_4_place_blue_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,place_blue_box_1] )
 GC360: both_pick_blue_box_4_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC365: both_pick_blue_box_4_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,place_white_box_1] )
 GC370: both_pick_blue_box_4_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,place_white_box_2] )
 GC375: both_pick_white_box_1_pick_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_1] ,
   assignment[human_right_arm,pick_white_box_2] )
 GC380: both_pick_white_box_1_place_blue_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_1] ,
   assignment[human_right_arm,place_blue_box_1] )
 GC385: both_pick_white_box_1_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_1] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC390: both_pick_white_box_1_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_1] ,
   assignment[human_right_arm,place_white_box_1] )
 GC395: both_pick_white_box_1_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_1] ,
   assignment[human_right_arm,place_white_box_2] )
 GC400: both_pick_white_box_2_place_blue_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_2] ,
   assignment[human_right_arm,place_blue_box_1] )
 GC405: both_pick_white_box_2_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_2] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC410: both_pick_white_box_2_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_2] ,
   assignment[human_right_arm,place_white_box_1] )
 GC415: both_pick_white_box_2_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_2] ,
   assignment[human_right_arm,place_white_box_2] )
 GC420: both_place_blue_box_1_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,place_blue_box_1] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC425: both_place_blue_box_1_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,place_blue_box_1] ,
   assignment[human_right_arm,place_white_box_1] )
 GC430: both_place_blue_box_1_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,place_blue_box_1] ,
   assignment[human_right_arm,place_white_box_2] )
 GC435: both_place_blue_box_2_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,place_blue_box_2] ,
   assignment[human_right_arm,place_white_box_1] )
 GC440: both_place_blue_box_2_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,place_blue_box_2] ,
   assignment[human_right_arm,place_white_box_2] )
 GC445: both_place_white_box_1_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,place_white_box_1] ,
   assignment[human_right_arm,place_white_box_2] )
 GC450: makespan = MAX ( t_end[pick_orange_box_1] ,
   t_end[pick_orange_box_2] , t_end[pick_blue_box_1] ,
   t_end[pick_blue_box_2] , t_end[pick_blue_box_3] ,
   t_end[pick_blue_box_4] , t_end[pick_white_box_1] ,
   t_end[pick_white_box_2] , t_end[place_orange_box_1] ,
   t_end[place_orange_box_2] , t_end[place_blue_box_1] ,
   t_end[place_blue_box_2] , t_end[place_blue_box_3] ,
   t_end[place_blue_box_4] , t_end[place_white_box_1] ,
   t_end[place_white_box_2] )
End
