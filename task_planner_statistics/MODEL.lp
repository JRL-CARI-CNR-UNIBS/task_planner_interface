\ Model Task_Planning&Scheduling
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 same_agent[('pick_orange_box_1',_'pick_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'pick_orange_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'pick_orange_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'pick_blue_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'pick_blue_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'pick_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'pick_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'place_orange_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'place_orange_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'place_orange_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'pick_orange_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'pick_orange_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'pick_blue_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'pick_blue_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'pick_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'pick_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'place_orange_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'place_orange_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'place_orange_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_3',_'pick_orange_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_3',_'pick_blue_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_3',_'pick_blue_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_3',_'pick_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_3',_'pick_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_3',_'place_orange_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_3',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_3',_'place_orange_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_3',_'place_orange_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_3',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_3',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_4',_'pick_blue_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_4',_'pick_blue_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_4',_'pick_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_4',_'pick_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_4',_'place_orange_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_4',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_4',_'place_orange_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_4',_'place_orange_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_4',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_orange_box_4',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'place_orange_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'place_orange_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'place_orange_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_2',_'place_orange_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_2',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_2',_'place_orange_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_2',_'place_orange_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_2',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_2',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_3',_'place_orange_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_3',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_3',_'place_orange_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_3',_'place_orange_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_3',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_4',_'place_orange_box_1'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_4',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_4',_'place_orange_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_4',_'place_orange_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_4',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_4',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_1',_'place_orange_box_3'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_1',_'place_orange_box_4'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_2',_'place_orange_box_3'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_2',_'place_orange_box_4'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_3',_'place_orange_box_4'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_3',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_3',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_4',_'place_blue_box_3'),ur5_on_guide]
   + 0 same_agent[('place_orange_box_4',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('place_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   + 0 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'pick_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'pick_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'pick_white_box_3'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'pick_white_box_4'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'place_blue_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'place_white_box_3'),human_right_arm]
   + 0 same_agent[('pick_blue_box_1',_'place_white_box_4'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'pick_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'pick_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'pick_white_box_3'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'pick_white_box_4'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'place_blue_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'place_white_box_3'),human_right_arm]
   + 0 same_agent[('pick_blue_box_2',_'place_white_box_4'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'pick_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'pick_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'pick_white_box_3'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'pick_white_box_4'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'place_blue_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'place_white_box_3'),human_right_arm]
   + 0 same_agent[('pick_blue_box_3',_'place_white_box_4'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'pick_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'pick_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'pick_white_box_3'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'pick_white_box_4'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'place_blue_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'place_white_box_3'),human_right_arm]
   + 0 same_agent[('pick_blue_box_4',_'place_white_box_4'),human_right_arm]
   + 0 same_agent[('pick_white_box_1',_'pick_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_white_box_1',_'pick_white_box_3'),human_right_arm]
   + 0 same_agent[('pick_white_box_1',_'pick_white_box_4'),human_right_arm]
   + 0 same_agent[('pick_white_box_1',_'place_blue_box_1'),human_right_arm]
   + 0 same_agent[('pick_white_box_1',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_white_box_1',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_white_box_1',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_white_box_1',_'place_white_box_3'),human_right_arm]
   + 0 same_agent[('pick_white_box_1',_'place_white_box_4'),human_right_arm]
   + 0 same_agent[('pick_white_box_2',_'pick_white_box_3'),human_right_arm]
   + 0 same_agent[('pick_white_box_2',_'pick_white_box_4'),human_right_arm]
   + 0 same_agent[('pick_white_box_2',_'place_blue_box_1'),human_right_arm]
   + 0 same_agent[('pick_white_box_2',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_white_box_2',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_white_box_2',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_white_box_2',_'place_white_box_3'),human_right_arm]
   + 0 same_agent[('pick_white_box_2',_'place_white_box_4'),human_right_arm]
   + 0 same_agent[('pick_white_box_3',_'pick_white_box_4'),human_right_arm]
   + 0 same_agent[('pick_white_box_3',_'place_blue_box_1'),human_right_arm]
   + 0 same_agent[('pick_white_box_3',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_white_box_3',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_white_box_3',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_white_box_3',_'place_white_box_3'),human_right_arm]
   + 0 same_agent[('pick_white_box_3',_'place_white_box_4'),human_right_arm]
   + 0 same_agent[('pick_white_box_4',_'place_blue_box_1'),human_right_arm]
   + 0 same_agent[('pick_white_box_4',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('pick_white_box_4',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('pick_white_box_4',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('pick_white_box_4',_'place_white_box_3'),human_right_arm]
   + 0 same_agent[('pick_white_box_4',_'place_white_box_4'),human_right_arm]
   + 0 same_agent[('place_blue_box_1',_'place_blue_box_2'),human_right_arm]
   + 0 same_agent[('place_blue_box_1',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('place_blue_box_1',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('place_blue_box_1',_'place_white_box_3'),human_right_arm]
   + 0 same_agent[('place_blue_box_1',_'place_white_box_4'),human_right_arm]
   + 0 same_agent[('place_blue_box_2',_'place_white_box_1'),human_right_arm]
   + 0 same_agent[('place_blue_box_2',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('place_blue_box_2',_'place_white_box_3'),human_right_arm]
   + 0 same_agent[('place_blue_box_2',_'place_white_box_4'),human_right_arm]
   + 0 same_agent[('place_white_box_1',_'place_white_box_2'),human_right_arm]
   + 0 same_agent[('place_white_box_1',_'place_white_box_3'),human_right_arm]
   + 0 same_agent[('place_white_box_1',_'place_white_box_4'),human_right_arm]
   + 0 same_agent[('place_white_box_2',_'place_white_box_3'),human_right_arm]
   + 0 same_agent[('place_white_box_2',_'place_white_box_4'),human_right_arm]
   + 0 same_agent[('place_white_box_3',_'place_white_box_4'),human_right_arm]
   + 0 both_pick_orange_box_1_pick_orange_box_2_to_ur5_on_guide
   + 0 both_pick_orange_box_1_pick_orange_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_1_pick_orange_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_1_pick_blue_box_1_to_ur5_on_guide
   + 0 both_pick_orange_box_1_pick_blue_box_2_to_ur5_on_guide
   + 0 both_pick_orange_box_1_pick_blue_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_1_pick_blue_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_1_place_orange_box_1_to_ur5_on_guide
   + 0 both_pick_orange_box_1_place_orange_box_2_to_ur5_on_guide
   + 0 both_pick_orange_box_1_place_orange_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_1_place_orange_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_1_place_blue_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_1_place_blue_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_2_pick_orange_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_2_pick_orange_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_2_pick_blue_box_1_to_ur5_on_guide
   + 0 both_pick_orange_box_2_pick_blue_box_2_to_ur5_on_guide
   + 0 both_pick_orange_box_2_pick_blue_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_2_pick_blue_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_2_place_orange_box_1_to_ur5_on_guide
   + 0 both_pick_orange_box_2_place_orange_box_2_to_ur5_on_guide
   + 0 both_pick_orange_box_2_place_orange_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_2_place_orange_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_2_place_blue_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_2_place_blue_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_3_pick_orange_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_3_pick_blue_box_1_to_ur5_on_guide
   + 0 both_pick_orange_box_3_pick_blue_box_2_to_ur5_on_guide
   + 0 both_pick_orange_box_3_pick_blue_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_3_pick_blue_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_3_place_orange_box_1_to_ur5_on_guide
   + 0 both_pick_orange_box_3_place_orange_box_2_to_ur5_on_guide
   + 0 both_pick_orange_box_3_place_orange_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_3_place_orange_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_3_place_blue_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_3_place_blue_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_4_pick_blue_box_1_to_ur5_on_guide
   + 0 both_pick_orange_box_4_pick_blue_box_2_to_ur5_on_guide
   + 0 both_pick_orange_box_4_pick_blue_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_4_pick_blue_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_4_place_orange_box_1_to_ur5_on_guide
   + 0 both_pick_orange_box_4_place_orange_box_2_to_ur5_on_guide
   + 0 both_pick_orange_box_4_place_orange_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_4_place_orange_box_4_to_ur5_on_guide
   + 0 both_pick_orange_box_4_place_blue_box_3_to_ur5_on_guide
   + 0 both_pick_orange_box_4_place_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_1_pick_blue_box_2_to_ur5_on_guide
   + 0 both_pick_blue_box_1_pick_blue_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_1_pick_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_1_place_orange_box_1_to_ur5_on_guide
   + 0 both_pick_blue_box_1_place_orange_box_2_to_ur5_on_guide
   + 0 both_pick_blue_box_1_place_orange_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_1_place_orange_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_1_place_blue_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_1_place_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_2_pick_blue_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_2_pick_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_2_place_orange_box_1_to_ur5_on_guide
   + 0 both_pick_blue_box_2_place_orange_box_2_to_ur5_on_guide
   + 0 both_pick_blue_box_2_place_orange_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_2_place_orange_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_2_place_blue_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_2_place_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_3_pick_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_3_place_orange_box_1_to_ur5_on_guide
   + 0 both_pick_blue_box_3_place_orange_box_2_to_ur5_on_guide
   + 0 both_pick_blue_box_3_place_orange_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_3_place_orange_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_3_place_blue_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_3_place_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_4_place_orange_box_1_to_ur5_on_guide
   + 0 both_pick_blue_box_4_place_orange_box_2_to_ur5_on_guide
   + 0 both_pick_blue_box_4_place_orange_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_4_place_orange_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_4_place_blue_box_3_to_ur5_on_guide
   + 0 both_pick_blue_box_4_place_blue_box_4_to_ur5_on_guide
   + 0 both_place_orange_box_1_place_orange_box_2_to_ur5_on_guide
   + 0 both_place_orange_box_1_place_orange_box_3_to_ur5_on_guide
   + 0 both_place_orange_box_1_place_orange_box_4_to_ur5_on_guide
   + 0 both_place_orange_box_1_place_blue_box_3_to_ur5_on_guide
   + 0 both_place_orange_box_1_place_blue_box_4_to_ur5_on_guide
   + 0 both_place_orange_box_2_place_orange_box_3_to_ur5_on_guide
   + 0 both_place_orange_box_2_place_orange_box_4_to_ur5_on_guide
   + 0 both_place_orange_box_2_place_blue_box_3_to_ur5_on_guide
   + 0 both_place_orange_box_2_place_blue_box_4_to_ur5_on_guide
   + 0 both_place_orange_box_3_place_orange_box_4_to_ur5_on_guide
   + 0 both_place_orange_box_3_place_blue_box_3_to_ur5_on_guide
   + 0 both_place_orange_box_3_place_blue_box_4_to_ur5_on_guide
   + 0 both_place_orange_box_4_place_blue_box_3_to_ur5_on_guide
   + 0 both_place_orange_box_4_place_blue_box_4_to_ur5_on_guide
   + 0 both_place_blue_box_3_place_blue_box_4_to_ur5_on_guide
   + 0 both_pick_blue_box_1_pick_blue_box_2_to_human_right_arm
   + 0 both_pick_blue_box_1_pick_blue_box_3_to_human_right_arm
   + 0 both_pick_blue_box_1_pick_blue_box_4_to_human_right_arm
   + 0 both_pick_blue_box_1_pick_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_1_pick_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_1_pick_white_box_3_to_human_right_arm
   + 0 both_pick_blue_box_1_pick_white_box_4_to_human_right_arm
   + 0 both_pick_blue_box_1_place_blue_box_1_to_human_right_arm
   + 0 both_pick_blue_box_1_place_blue_box_2_to_human_right_arm
   + 0 both_pick_blue_box_1_place_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_1_place_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_1_place_white_box_3_to_human_right_arm
   + 0 both_pick_blue_box_1_place_white_box_4_to_human_right_arm
   + 0 both_pick_blue_box_2_pick_blue_box_3_to_human_right_arm
   + 0 both_pick_blue_box_2_pick_blue_box_4_to_human_right_arm
   + 0 both_pick_blue_box_2_pick_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_2_pick_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_2_pick_white_box_3_to_human_right_arm
   + 0 both_pick_blue_box_2_pick_white_box_4_to_human_right_arm
   + 0 both_pick_blue_box_2_place_blue_box_1_to_human_right_arm
   + 0 both_pick_blue_box_2_place_blue_box_2_to_human_right_arm
   + 0 both_pick_blue_box_2_place_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_2_place_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_2_place_white_box_3_to_human_right_arm
   + 0 both_pick_blue_box_2_place_white_box_4_to_human_right_arm
   + 0 both_pick_blue_box_3_pick_blue_box_4_to_human_right_arm
   + 0 both_pick_blue_box_3_pick_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_3_pick_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_3_pick_white_box_3_to_human_right_arm
   + 0 both_pick_blue_box_3_pick_white_box_4_to_human_right_arm
   + 0 both_pick_blue_box_3_place_blue_box_1_to_human_right_arm
   + 0 both_pick_blue_box_3_place_blue_box_2_to_human_right_arm
   + 0 both_pick_blue_box_3_place_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_3_place_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_3_place_white_box_3_to_human_right_arm
   + 0 both_pick_blue_box_3_place_white_box_4_to_human_right_arm
   + 0 both_pick_blue_box_4_pick_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_4_pick_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_4_pick_white_box_3_to_human_right_arm
   + 0 both_pick_blue_box_4_pick_white_box_4_to_human_right_arm
   + 0 both_pick_blue_box_4_place_blue_box_1_to_human_right_arm
   + 0 both_pick_blue_box_4_place_blue_box_2_to_human_right_arm
   + 0 both_pick_blue_box_4_place_white_box_1_to_human_right_arm
   + 0 both_pick_blue_box_4_place_white_box_2_to_human_right_arm
   + 0 both_pick_blue_box_4_place_white_box_3_to_human_right_arm
   + 0 both_pick_blue_box_4_place_white_box_4_to_human_right_arm
   + 0 both_pick_white_box_1_pick_white_box_2_to_human_right_arm
   + 0 both_pick_white_box_1_pick_white_box_3_to_human_right_arm
   + 0 both_pick_white_box_1_pick_white_box_4_to_human_right_arm
   + 0 both_pick_white_box_1_place_blue_box_1_to_human_right_arm
   + 0 both_pick_white_box_1_place_blue_box_2_to_human_right_arm
   + 0 both_pick_white_box_1_place_white_box_1_to_human_right_arm
   + 0 both_pick_white_box_1_place_white_box_2_to_human_right_arm
   + 0 both_pick_white_box_1_place_white_box_3_to_human_right_arm
   + 0 both_pick_white_box_1_place_white_box_4_to_human_right_arm
   + 0 both_pick_white_box_2_pick_white_box_3_to_human_right_arm
   + 0 both_pick_white_box_2_pick_white_box_4_to_human_right_arm
   + 0 both_pick_white_box_2_place_blue_box_1_to_human_right_arm
   + 0 both_pick_white_box_2_place_blue_box_2_to_human_right_arm
   + 0 both_pick_white_box_2_place_white_box_1_to_human_right_arm
   + 0 both_pick_white_box_2_place_white_box_2_to_human_right_arm
   + 0 both_pick_white_box_2_place_white_box_3_to_human_right_arm
   + 0 both_pick_white_box_2_place_white_box_4_to_human_right_arm
   + 0 both_pick_white_box_3_pick_white_box_4_to_human_right_arm
   + 0 both_pick_white_box_3_place_blue_box_1_to_human_right_arm
   + 0 both_pick_white_box_3_place_blue_box_2_to_human_right_arm
   + 0 both_pick_white_box_3_place_white_box_1_to_human_right_arm
   + 0 both_pick_white_box_3_place_white_box_2_to_human_right_arm
   + 0 both_pick_white_box_3_place_white_box_3_to_human_right_arm
   + 0 both_pick_white_box_3_place_white_box_4_to_human_right_arm
   + 0 both_pick_white_box_4_place_blue_box_1_to_human_right_arm
   + 0 both_pick_white_box_4_place_blue_box_2_to_human_right_arm
   + 0 both_pick_white_box_4_place_white_box_1_to_human_right_arm
   + 0 both_pick_white_box_4_place_white_box_2_to_human_right_arm
   + 0 both_pick_white_box_4_place_white_box_3_to_human_right_arm
   + 0 both_pick_white_box_4_place_white_box_4_to_human_right_arm
   + 0 both_place_blue_box_1_place_blue_box_2_to_human_right_arm
   + 0 both_place_blue_box_1_place_white_box_1_to_human_right_arm
   + 0 both_place_blue_box_1_place_white_box_2_to_human_right_arm
   + 0 both_place_blue_box_1_place_white_box_3_to_human_right_arm
   + 0 both_place_blue_box_1_place_white_box_4_to_human_right_arm
   + 0 both_place_blue_box_2_place_white_box_1_to_human_right_arm
   + 0 both_place_blue_box_2_place_white_box_2_to_human_right_arm
   + 0 both_place_blue_box_2_place_white_box_3_to_human_right_arm
   + 0 both_place_blue_box_2_place_white_box_4_to_human_right_arm
   + 0 both_place_white_box_1_place_white_box_2_to_human_right_arm
   + 0 both_place_white_box_1_place_white_box_3_to_human_right_arm
   + 0 both_place_white_box_1_place_white_box_4_to_human_right_arm
   + 0 both_place_white_box_2_place_white_box_3_to_human_right_arm
   + 0 both_place_white_box_2_place_white_box_4_to_human_right_arm
   + 0 both_place_white_box_3_place_white_box_4_to_human_right_arm
   + makespan
Subject To
 t_end(pick_white_box_1):
   - 8.666432213783263 assignment[human_right_arm,pick_white_box_1]
   - t_start[pick_white_box_1] + t_end[pick_white_box_1] = 0
 t_end(pick_white_box_2):
   - 8.666432213783263 assignment[human_right_arm,pick_white_box_2]
   - t_start[pick_white_box_2] + t_end[pick_white_box_2] = 0
 t_end(pick_white_box_3):
   - 8.666432213783263 assignment[human_right_arm,pick_white_box_3]
   - t_start[pick_white_box_3] + t_end[pick_white_box_3] = 0
 t_end(pick_white_box_4):
   - 8.666432213783263 assignment[human_right_arm,pick_white_box_4]
   - t_start[pick_white_box_4] + t_end[pick_white_box_4] = 0
 t_end(place_blue_box_1):
   - 4.1574129986763 assignment[human_right_arm,place_blue_box_1]
   - t_start[place_blue_box_1] + t_end[place_blue_box_1] = 0
 t_end(place_blue_box_2):
   - 4.1574129986763 assignment[human_right_arm,place_blue_box_2]
   - t_start[place_blue_box_2] + t_end[place_blue_box_2] = 0
 t_end(place_white_box_1):
   - 6.268468438386917 assignment[human_right_arm,place_white_box_1]
   - t_start[place_white_box_1] + t_end[place_white_box_1] = 0
 t_end(place_white_box_2):
   - 6.268468438386917 assignment[human_right_arm,place_white_box_2]
   - t_start[place_white_box_2] + t_end[place_white_box_2] = 0
 t_end(place_white_box_3):
   - 6.268468438386917 assignment[human_right_arm,place_white_box_3]
   - t_start[place_white_box_3] + t_end[place_white_box_3] = 0
 t_end(place_white_box_4):
   - 6.268468438386917 assignment[human_right_arm,place_white_box_4]
   - t_start[place_white_box_4] + t_end[place_white_box_4] = 0
 unique_assignment[pick_orange_box_1]:
   assignment[ur5_on_guide,pick_orange_box_1] = 1
 unique_assignment[pick_orange_box_2]:
   assignment[ur5_on_guide,pick_orange_box_2] = 1
 unique_assignment[pick_orange_box_3]:
   assignment[ur5_on_guide,pick_orange_box_3] = 1
 unique_assignment[pick_orange_box_4]:
   assignment[ur5_on_guide,pick_orange_box_4] = 1
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
 unique_assignment[pick_white_box_3]:
   assignment[human_right_arm,pick_white_box_3] = 1
 unique_assignment[pick_white_box_4]:
   assignment[human_right_arm,pick_white_box_4] = 1
 unique_assignment[place_orange_box_1]:
   assignment[ur5_on_guide,place_orange_box_1] = 1
 unique_assignment[place_orange_box_2]:
   assignment[ur5_on_guide,place_orange_box_2] = 1
 unique_assignment[place_orange_box_3]:
   assignment[ur5_on_guide,place_orange_box_3] = 1
 unique_assignment[place_orange_box_4]:
   assignment[ur5_on_guide,place_orange_box_4] = 1
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
 unique_assignment[place_white_box_3]:
   assignment[human_right_arm,place_white_box_3] = 1
 unique_assignment[place_white_box_4]:
   assignment[human_right_arm,place_white_box_4] = 1
 not_enabled_assignment_pick_blue_box_1:
   assignment[ur5_on_guide,pick_blue_box_1] = 0
 not_enabled_assignment_pick_blue_box_2:
   assignment[ur5_on_guide,pick_blue_box_2] = 0
 not_enabled_assignment_pick_blue_box_3:
   assignment[human_right_arm,pick_blue_box_3] = 0
 not_enabled_assignment_pick_blue_box_4:
   assignment[human_right_arm,pick_blue_box_4] = 0
 R38: raw_overlapping(pick_orange_box_1,pick_orange_box_2)
   - min_t_end(pick_orange_box_1,_pick_orange_box_2)
   + max_t_start(pick_orange_box_1,_pick_orange_box_2) = 0
 R39: raw_overlapping(pick_orange_box_1,pick_orange_box_2)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_orange_box_2)
   >= -1e+12
 R40: raw_overlapping(pick_orange_box_1,pick_orange_box_2)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_orange_box_2)
   <= -1e-12
 R41: raw_overlapping(pick_orange_box_1,pick_orange_box_3)
   - min_t_end(pick_orange_box_1,_pick_orange_box_3)
   + max_t_start(pick_orange_box_1,_pick_orange_box_3) = 0
 R42: raw_overlapping(pick_orange_box_1,pick_orange_box_3)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_orange_box_3)
   >= -1e+12
 R43: raw_overlapping(pick_orange_box_1,pick_orange_box_3)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_orange_box_3)
   <= -1e-12
 R44: raw_overlapping(pick_orange_box_1,pick_orange_box_4)
   - min_t_end(pick_orange_box_1,_pick_orange_box_4)
   + max_t_start(pick_orange_box_1,_pick_orange_box_4) = 0
 R45: raw_overlapping(pick_orange_box_1,pick_orange_box_4)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_orange_box_4)
   >= -1e+12
 R46: raw_overlapping(pick_orange_box_1,pick_orange_box_4)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_orange_box_4)
   <= -1e-12
 R47: raw_overlapping(pick_orange_box_1,pick_blue_box_1)
   - min_t_end(pick_orange_box_1,_pick_blue_box_1)
   + max_t_start(pick_orange_box_1,_pick_blue_box_1) = 0
 R48: raw_overlapping(pick_orange_box_1,pick_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_blue_box_1) >= -1e+12
 R49: raw_overlapping(pick_orange_box_1,pick_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_blue_box_1) <= -1e-12
 R50: raw_overlapping(pick_orange_box_1,pick_blue_box_2)
   - min_t_end(pick_orange_box_1,_pick_blue_box_2)
   + max_t_start(pick_orange_box_1,_pick_blue_box_2) = 0
 R51: raw_overlapping(pick_orange_box_1,pick_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_blue_box_2) >= -1e+12
 R52: raw_overlapping(pick_orange_box_1,pick_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_blue_box_2) <= -1e-12
 R53: raw_overlapping(pick_orange_box_1,pick_blue_box_3)
   - min_t_end(pick_orange_box_1,_pick_blue_box_3)
   + max_t_start(pick_orange_box_1,_pick_blue_box_3) = 0
 R54: raw_overlapping(pick_orange_box_1,pick_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_blue_box_3) >= -1e+12
 R55: raw_overlapping(pick_orange_box_1,pick_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_blue_box_3) <= -1e-12
 R56: raw_overlapping(pick_orange_box_1,pick_blue_box_4)
   - min_t_end(pick_orange_box_1,_pick_blue_box_4)
   + max_t_start(pick_orange_box_1,_pick_blue_box_4) = 0
 R57: raw_overlapping(pick_orange_box_1,pick_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_blue_box_4) >= -1e+12
 R58: raw_overlapping(pick_orange_box_1,pick_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_blue_box_4) <= -1e-12
 R59: raw_overlapping(pick_orange_box_1,pick_white_box_1)
   - min_t_end(pick_orange_box_1,_pick_white_box_1)
   + max_t_start(pick_orange_box_1,_pick_white_box_1) = 0
 R60: raw_overlapping(pick_orange_box_1,pick_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_white_box_1) >= -1e+12
 R61: raw_overlapping(pick_orange_box_1,pick_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_white_box_1) <= -1e-12
 R62: raw_overlapping(pick_orange_box_1,pick_white_box_2)
   - min_t_end(pick_orange_box_1,_pick_white_box_2)
   + max_t_start(pick_orange_box_1,_pick_white_box_2) = 0
 R63: raw_overlapping(pick_orange_box_1,pick_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_white_box_2) >= -1e+12
 R64: raw_overlapping(pick_orange_box_1,pick_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_white_box_2) <= -1e-12
 R65: raw_overlapping(pick_orange_box_1,pick_white_box_3)
   - min_t_end(pick_orange_box_1,_pick_white_box_3)
   + max_t_start(pick_orange_box_1,_pick_white_box_3) = 0
 R66: raw_overlapping(pick_orange_box_1,pick_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_white_box_3) >= -1e+12
 R67: raw_overlapping(pick_orange_box_1,pick_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_white_box_3) <= -1e-12
 R68: raw_overlapping(pick_orange_box_1,pick_white_box_4)
   - min_t_end(pick_orange_box_1,_pick_white_box_4)
   + max_t_start(pick_orange_box_1,_pick_white_box_4) = 0
 R69: raw_overlapping(pick_orange_box_1,pick_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_white_box_4) >= -1e+12
 R70: raw_overlapping(pick_orange_box_1,pick_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_1,pick_white_box_4) <= -1e-12
 R71: raw_overlapping(pick_orange_box_1,place_orange_box_1)
   - min_t_end(pick_orange_box_1,_place_orange_box_1)
   + max_t_start(pick_orange_box_1,_place_orange_box_1) = 0
 R72: raw_overlapping(pick_orange_box_1,place_orange_box_1)
   - 1e+12 check_parallelism(pick_orange_box_1,place_orange_box_1)
   >= -1e+12
 R73: raw_overlapping(pick_orange_box_1,place_orange_box_1)
   - 1e+12 check_parallelism(pick_orange_box_1,place_orange_box_1)
   <= -1e-12
 R74: raw_overlapping(pick_orange_box_1,place_orange_box_2)
   - min_t_end(pick_orange_box_1,_place_orange_box_2)
   + max_t_start(pick_orange_box_1,_place_orange_box_2) = 0
 R75: raw_overlapping(pick_orange_box_1,place_orange_box_2)
   - 1e+12 check_parallelism(pick_orange_box_1,place_orange_box_2)
   >= -1e+12
 R76: raw_overlapping(pick_orange_box_1,place_orange_box_2)
   - 1e+12 check_parallelism(pick_orange_box_1,place_orange_box_2)
   <= -1e-12
 R77: raw_overlapping(pick_orange_box_1,place_orange_box_3)
   - min_t_end(pick_orange_box_1,_place_orange_box_3)
   + max_t_start(pick_orange_box_1,_place_orange_box_3) = 0
 R78: raw_overlapping(pick_orange_box_1,place_orange_box_3)
   - 1e+12 check_parallelism(pick_orange_box_1,place_orange_box_3)
   >= -1e+12
 R79: raw_overlapping(pick_orange_box_1,place_orange_box_3)
   - 1e+12 check_parallelism(pick_orange_box_1,place_orange_box_3)
   <= -1e-12
 R80: raw_overlapping(pick_orange_box_1,place_orange_box_4)
   - min_t_end(pick_orange_box_1,_place_orange_box_4)
   + max_t_start(pick_orange_box_1,_place_orange_box_4) = 0
 R81: raw_overlapping(pick_orange_box_1,place_orange_box_4)
   - 1e+12 check_parallelism(pick_orange_box_1,place_orange_box_4)
   >= -1e+12
 R82: raw_overlapping(pick_orange_box_1,place_orange_box_4)
   - 1e+12 check_parallelism(pick_orange_box_1,place_orange_box_4)
   <= -1e-12
 R83: raw_overlapping(pick_orange_box_1,place_blue_box_1)
   - min_t_end(pick_orange_box_1,_place_blue_box_1)
   + max_t_start(pick_orange_box_1,_place_blue_box_1) = 0
 R84: raw_overlapping(pick_orange_box_1,place_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_1,place_blue_box_1) >= -1e+12
 R85: raw_overlapping(pick_orange_box_1,place_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_1,place_blue_box_1) <= -1e-12
 R86: raw_overlapping(pick_orange_box_1,place_blue_box_2)
   - min_t_end(pick_orange_box_1,_place_blue_box_2)
   + max_t_start(pick_orange_box_1,_place_blue_box_2) = 0
 R87: raw_overlapping(pick_orange_box_1,place_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_1,place_blue_box_2) >= -1e+12
 R88: raw_overlapping(pick_orange_box_1,place_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_1,place_blue_box_2) <= -1e-12
 R89: raw_overlapping(pick_orange_box_1,place_blue_box_3)
   - min_t_end(pick_orange_box_1,_place_blue_box_3)
   + max_t_start(pick_orange_box_1,_place_blue_box_3) = 0
 R90: raw_overlapping(pick_orange_box_1,place_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_1,place_blue_box_3) >= -1e+12
 R91: raw_overlapping(pick_orange_box_1,place_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_1,place_blue_box_3) <= -1e-12
 R92: raw_overlapping(pick_orange_box_1,place_blue_box_4)
   - min_t_end(pick_orange_box_1,_place_blue_box_4)
   + max_t_start(pick_orange_box_1,_place_blue_box_4) = 0
 R93: raw_overlapping(pick_orange_box_1,place_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_1,place_blue_box_4) >= -1e+12
 R94: raw_overlapping(pick_orange_box_1,place_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_1,place_blue_box_4) <= -1e-12
 R95: raw_overlapping(pick_orange_box_1,place_white_box_1)
   - min_t_end(pick_orange_box_1,_place_white_box_1)
   + max_t_start(pick_orange_box_1,_place_white_box_1) = 0
 R96: raw_overlapping(pick_orange_box_1,place_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_1,place_white_box_1)
   >= -1e+12
 R97: raw_overlapping(pick_orange_box_1,place_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_1,place_white_box_1)
   <= -1e-12
 R98: raw_overlapping(pick_orange_box_1,place_white_box_2)
   - min_t_end(pick_orange_box_1,_place_white_box_2)
   + max_t_start(pick_orange_box_1,_place_white_box_2) = 0
 R99: raw_overlapping(pick_orange_box_1,place_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_1,place_white_box_2)
   >= -1e+12
 R100: raw_overlapping(pick_orange_box_1,place_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_1,place_white_box_2)
   <= -1e-12
 R101: raw_overlapping(pick_orange_box_1,place_white_box_3)
   - min_t_end(pick_orange_box_1,_place_white_box_3)
   + max_t_start(pick_orange_box_1,_place_white_box_3) = 0
 R102: raw_overlapping(pick_orange_box_1,place_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_1,place_white_box_3)
   >= -1e+12
 R103: raw_overlapping(pick_orange_box_1,place_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_1,place_white_box_3)
   <= -1e-12
 R104: raw_overlapping(pick_orange_box_1,place_white_box_4)
   - min_t_end(pick_orange_box_1,_place_white_box_4)
   + max_t_start(pick_orange_box_1,_place_white_box_4) = 0
 R105: raw_overlapping(pick_orange_box_1,place_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_1,place_white_box_4)
   >= -1e+12
 R106: raw_overlapping(pick_orange_box_1,place_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_1,place_white_box_4)
   <= -1e-12
 R107: raw_overlapping(pick_orange_box_2,pick_orange_box_3)
   - min_t_end(pick_orange_box_2,_pick_orange_box_3)
   + max_t_start(pick_orange_box_2,_pick_orange_box_3) = 0
 R108: raw_overlapping(pick_orange_box_2,pick_orange_box_3)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_orange_box_3)
   >= -1e+12
 R109: raw_overlapping(pick_orange_box_2,pick_orange_box_3)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_orange_box_3)
   <= -1e-12
 R110: raw_overlapping(pick_orange_box_2,pick_orange_box_4)
   - min_t_end(pick_orange_box_2,_pick_orange_box_4)
   + max_t_start(pick_orange_box_2,_pick_orange_box_4) = 0
 R111: raw_overlapping(pick_orange_box_2,pick_orange_box_4)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_orange_box_4)
   >= -1e+12
 R112: raw_overlapping(pick_orange_box_2,pick_orange_box_4)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_orange_box_4)
   <= -1e-12
 R113: raw_overlapping(pick_orange_box_2,pick_blue_box_1)
   - min_t_end(pick_orange_box_2,_pick_blue_box_1)
   + max_t_start(pick_orange_box_2,_pick_blue_box_1) = 0
 R114: raw_overlapping(pick_orange_box_2,pick_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_blue_box_1) >= -1e+12
 R115: raw_overlapping(pick_orange_box_2,pick_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_blue_box_1) <= -1e-12
 R116: raw_overlapping(pick_orange_box_2,pick_blue_box_2)
   - min_t_end(pick_orange_box_2,_pick_blue_box_2)
   + max_t_start(pick_orange_box_2,_pick_blue_box_2) = 0
 R117: raw_overlapping(pick_orange_box_2,pick_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_blue_box_2) >= -1e+12
 R118: raw_overlapping(pick_orange_box_2,pick_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_blue_box_2) <= -1e-12
 R119: raw_overlapping(pick_orange_box_2,pick_blue_box_3)
   - min_t_end(pick_orange_box_2,_pick_blue_box_3)
   + max_t_start(pick_orange_box_2,_pick_blue_box_3) = 0
 R120: raw_overlapping(pick_orange_box_2,pick_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_blue_box_3) >= -1e+12
 R121: raw_overlapping(pick_orange_box_2,pick_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_blue_box_3) <= -1e-12
 R122: raw_overlapping(pick_orange_box_2,pick_blue_box_4)
   - min_t_end(pick_orange_box_2,_pick_blue_box_4)
   + max_t_start(pick_orange_box_2,_pick_blue_box_4) = 0
 R123: raw_overlapping(pick_orange_box_2,pick_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_blue_box_4) >= -1e+12
 R124: raw_overlapping(pick_orange_box_2,pick_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_blue_box_4) <= -1e-12
 R125: raw_overlapping(pick_orange_box_2,pick_white_box_1)
   - min_t_end(pick_orange_box_2,_pick_white_box_1)
   + max_t_start(pick_orange_box_2,_pick_white_box_1) = 0
 R126: raw_overlapping(pick_orange_box_2,pick_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_white_box_1) >= -1e+12
 R127: raw_overlapping(pick_orange_box_2,pick_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_white_box_1) <= -1e-12
 R128: raw_overlapping(pick_orange_box_2,pick_white_box_2)
   - min_t_end(pick_orange_box_2,_pick_white_box_2)
   + max_t_start(pick_orange_box_2,_pick_white_box_2) = 0
 R129: raw_overlapping(pick_orange_box_2,pick_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_white_box_2) >= -1e+12
 R130: raw_overlapping(pick_orange_box_2,pick_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_white_box_2) <= -1e-12
 R131: raw_overlapping(pick_orange_box_2,pick_white_box_3)
   - min_t_end(pick_orange_box_2,_pick_white_box_3)
   + max_t_start(pick_orange_box_2,_pick_white_box_3) = 0
 R132: raw_overlapping(pick_orange_box_2,pick_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_white_box_3) >= -1e+12
 R133: raw_overlapping(pick_orange_box_2,pick_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_white_box_3) <= -1e-12
 R134: raw_overlapping(pick_orange_box_2,pick_white_box_4)
   - min_t_end(pick_orange_box_2,_pick_white_box_4)
   + max_t_start(pick_orange_box_2,_pick_white_box_4) = 0
 R135: raw_overlapping(pick_orange_box_2,pick_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_white_box_4) >= -1e+12
 R136: raw_overlapping(pick_orange_box_2,pick_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_2,pick_white_box_4) <= -1e-12
 R137: raw_overlapping(pick_orange_box_2,place_orange_box_1)
   - min_t_end(pick_orange_box_2,_place_orange_box_1)
   + max_t_start(pick_orange_box_2,_place_orange_box_1) = 0
 R138: raw_overlapping(pick_orange_box_2,place_orange_box_1)
   - 1e+12 check_parallelism(pick_orange_box_2,place_orange_box_1)
   >= -1e+12
 R139: raw_overlapping(pick_orange_box_2,place_orange_box_1)
   - 1e+12 check_parallelism(pick_orange_box_2,place_orange_box_1)
   <= -1e-12
 R140: raw_overlapping(pick_orange_box_2,place_orange_box_2)
   - min_t_end(pick_orange_box_2,_place_orange_box_2)
   + max_t_start(pick_orange_box_2,_place_orange_box_2) = 0
 R141: raw_overlapping(pick_orange_box_2,place_orange_box_2)
   - 1e+12 check_parallelism(pick_orange_box_2,place_orange_box_2)
   >= -1e+12
 R142: raw_overlapping(pick_orange_box_2,place_orange_box_2)
   - 1e+12 check_parallelism(pick_orange_box_2,place_orange_box_2)
   <= -1e-12
 R143: raw_overlapping(pick_orange_box_2,place_orange_box_3)
   - min_t_end(pick_orange_box_2,_place_orange_box_3)
   + max_t_start(pick_orange_box_2,_place_orange_box_3) = 0
 R144: raw_overlapping(pick_orange_box_2,place_orange_box_3)
   - 1e+12 check_parallelism(pick_orange_box_2,place_orange_box_3)
   >= -1e+12
 R145: raw_overlapping(pick_orange_box_2,place_orange_box_3)
   - 1e+12 check_parallelism(pick_orange_box_2,place_orange_box_3)
   <= -1e-12
 R146: raw_overlapping(pick_orange_box_2,place_orange_box_4)
   - min_t_end(pick_orange_box_2,_place_orange_box_4)
   + max_t_start(pick_orange_box_2,_place_orange_box_4) = 0
 R147: raw_overlapping(pick_orange_box_2,place_orange_box_4)
   - 1e+12 check_parallelism(pick_orange_box_2,place_orange_box_4)
   >= -1e+12
 R148: raw_overlapping(pick_orange_box_2,place_orange_box_4)
   - 1e+12 check_parallelism(pick_orange_box_2,place_orange_box_4)
   <= -1e-12
 R149: raw_overlapping(pick_orange_box_2,place_blue_box_1)
   - min_t_end(pick_orange_box_2,_place_blue_box_1)
   + max_t_start(pick_orange_box_2,_place_blue_box_1) = 0
 R150: raw_overlapping(pick_orange_box_2,place_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_2,place_blue_box_1) >= -1e+12
 R151: raw_overlapping(pick_orange_box_2,place_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_2,place_blue_box_1) <= -1e-12
 R152: raw_overlapping(pick_orange_box_2,place_blue_box_2)
   - min_t_end(pick_orange_box_2,_place_blue_box_2)
   + max_t_start(pick_orange_box_2,_place_blue_box_2) = 0
 R153: raw_overlapping(pick_orange_box_2,place_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_2,place_blue_box_2) >= -1e+12
 R154: raw_overlapping(pick_orange_box_2,place_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_2,place_blue_box_2) <= -1e-12
 R155: raw_overlapping(pick_orange_box_2,place_blue_box_3)
   - min_t_end(pick_orange_box_2,_place_blue_box_3)
   + max_t_start(pick_orange_box_2,_place_blue_box_3) = 0
 R156: raw_overlapping(pick_orange_box_2,place_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_2,place_blue_box_3) >= -1e+12
 R157: raw_overlapping(pick_orange_box_2,place_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_2,place_blue_box_3) <= -1e-12
 R158: raw_overlapping(pick_orange_box_2,place_blue_box_4)
   - min_t_end(pick_orange_box_2,_place_blue_box_4)
   + max_t_start(pick_orange_box_2,_place_blue_box_4) = 0
 R159: raw_overlapping(pick_orange_box_2,place_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_2,place_blue_box_4) >= -1e+12
 R160: raw_overlapping(pick_orange_box_2,place_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_2,place_blue_box_4) <= -1e-12
 R161: raw_overlapping(pick_orange_box_2,place_white_box_1)
   - min_t_end(pick_orange_box_2,_place_white_box_1)
   + max_t_start(pick_orange_box_2,_place_white_box_1) = 0
 R162: raw_overlapping(pick_orange_box_2,place_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_2,place_white_box_1)
   >= -1e+12
 R163: raw_overlapping(pick_orange_box_2,place_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_2,place_white_box_1)
   <= -1e-12
 R164: raw_overlapping(pick_orange_box_2,place_white_box_2)
   - min_t_end(pick_orange_box_2,_place_white_box_2)
   + max_t_start(pick_orange_box_2,_place_white_box_2) = 0
 R165: raw_overlapping(pick_orange_box_2,place_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_2,place_white_box_2)
   >= -1e+12
 R166: raw_overlapping(pick_orange_box_2,place_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_2,place_white_box_2)
   <= -1e-12
 R167: raw_overlapping(pick_orange_box_2,place_white_box_3)
   - min_t_end(pick_orange_box_2,_place_white_box_3)
   + max_t_start(pick_orange_box_2,_place_white_box_3) = 0
 R168: raw_overlapping(pick_orange_box_2,place_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_2,place_white_box_3)
   >= -1e+12
 R169: raw_overlapping(pick_orange_box_2,place_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_2,place_white_box_3)
   <= -1e-12
 R170: raw_overlapping(pick_orange_box_2,place_white_box_4)
   - min_t_end(pick_orange_box_2,_place_white_box_4)
   + max_t_start(pick_orange_box_2,_place_white_box_4) = 0
 R171: raw_overlapping(pick_orange_box_2,place_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_2,place_white_box_4)
   >= -1e+12
 R172: raw_overlapping(pick_orange_box_2,place_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_2,place_white_box_4)
   <= -1e-12
 R173: raw_overlapping(pick_orange_box_3,pick_orange_box_4)
   - min_t_end(pick_orange_box_3,_pick_orange_box_4)
   + max_t_start(pick_orange_box_3,_pick_orange_box_4) = 0
 R174: raw_overlapping(pick_orange_box_3,pick_orange_box_4)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_orange_box_4)
   >= -1e+12
 R175: raw_overlapping(pick_orange_box_3,pick_orange_box_4)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_orange_box_4)
   <= -1e-12
 R176: raw_overlapping(pick_orange_box_3,pick_blue_box_1)
   - min_t_end(pick_orange_box_3,_pick_blue_box_1)
   + max_t_start(pick_orange_box_3,_pick_blue_box_1) = 0
 R177: raw_overlapping(pick_orange_box_3,pick_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_blue_box_1) >= -1e+12
 R178: raw_overlapping(pick_orange_box_3,pick_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_blue_box_1) <= -1e-12
 R179: raw_overlapping(pick_orange_box_3,pick_blue_box_2)
   - min_t_end(pick_orange_box_3,_pick_blue_box_2)
   + max_t_start(pick_orange_box_3,_pick_blue_box_2) = 0
 R180: raw_overlapping(pick_orange_box_3,pick_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_blue_box_2) >= -1e+12
 R181: raw_overlapping(pick_orange_box_3,pick_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_blue_box_2) <= -1e-12
 R182: raw_overlapping(pick_orange_box_3,pick_blue_box_3)
   - min_t_end(pick_orange_box_3,_pick_blue_box_3)
   + max_t_start(pick_orange_box_3,_pick_blue_box_3) = 0
 R183: raw_overlapping(pick_orange_box_3,pick_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_blue_box_3) >= -1e+12
 R184: raw_overlapping(pick_orange_box_3,pick_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_blue_box_3) <= -1e-12
 R185: raw_overlapping(pick_orange_box_3,pick_blue_box_4)
   - min_t_end(pick_orange_box_3,_pick_blue_box_4)
   + max_t_start(pick_orange_box_3,_pick_blue_box_4) = 0
 R186: raw_overlapping(pick_orange_box_3,pick_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_blue_box_4) >= -1e+12
 R187: raw_overlapping(pick_orange_box_3,pick_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_blue_box_4) <= -1e-12
 R188: raw_overlapping(pick_orange_box_3,pick_white_box_1)
   - min_t_end(pick_orange_box_3,_pick_white_box_1)
   + max_t_start(pick_orange_box_3,_pick_white_box_1) = 0
 R189: raw_overlapping(pick_orange_box_3,pick_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_white_box_1) >= -1e+12
 R190: raw_overlapping(pick_orange_box_3,pick_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_white_box_1) <= -1e-12
 R191: raw_overlapping(pick_orange_box_3,pick_white_box_2)
   - min_t_end(pick_orange_box_3,_pick_white_box_2)
   + max_t_start(pick_orange_box_3,_pick_white_box_2) = 0
 R192: raw_overlapping(pick_orange_box_3,pick_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_white_box_2) >= -1e+12
 R193: raw_overlapping(pick_orange_box_3,pick_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_white_box_2) <= -1e-12
 R194: raw_overlapping(pick_orange_box_3,pick_white_box_3)
   - min_t_end(pick_orange_box_3,_pick_white_box_3)
   + max_t_start(pick_orange_box_3,_pick_white_box_3) = 0
 R195: raw_overlapping(pick_orange_box_3,pick_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_white_box_3) >= -1e+12
 R196: raw_overlapping(pick_orange_box_3,pick_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_white_box_3) <= -1e-12
 R197: raw_overlapping(pick_orange_box_3,pick_white_box_4)
   - min_t_end(pick_orange_box_3,_pick_white_box_4)
   + max_t_start(pick_orange_box_3,_pick_white_box_4) = 0
 R198: raw_overlapping(pick_orange_box_3,pick_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_white_box_4) >= -1e+12
 R199: raw_overlapping(pick_orange_box_3,pick_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_3,pick_white_box_4) <= -1e-12
 R200: raw_overlapping(pick_orange_box_3,place_orange_box_1)
   - min_t_end(pick_orange_box_3,_place_orange_box_1)
   + max_t_start(pick_orange_box_3,_place_orange_box_1) = 0
 R201: raw_overlapping(pick_orange_box_3,place_orange_box_1)
   - 1e+12 check_parallelism(pick_orange_box_3,place_orange_box_1)
   >= -1e+12
 R202: raw_overlapping(pick_orange_box_3,place_orange_box_1)
   - 1e+12 check_parallelism(pick_orange_box_3,place_orange_box_1)
   <= -1e-12
 R203: raw_overlapping(pick_orange_box_3,place_orange_box_2)
   - min_t_end(pick_orange_box_3,_place_orange_box_2)
   + max_t_start(pick_orange_box_3,_place_orange_box_2) = 0
 R204: raw_overlapping(pick_orange_box_3,place_orange_box_2)
   - 1e+12 check_parallelism(pick_orange_box_3,place_orange_box_2)
   >= -1e+12
 R205: raw_overlapping(pick_orange_box_3,place_orange_box_2)
   - 1e+12 check_parallelism(pick_orange_box_3,place_orange_box_2)
   <= -1e-12
 R206: raw_overlapping(pick_orange_box_3,place_orange_box_3)
   - min_t_end(pick_orange_box_3,_place_orange_box_3)
   + max_t_start(pick_orange_box_3,_place_orange_box_3) = 0
 R207: raw_overlapping(pick_orange_box_3,place_orange_box_3)
   - 1e+12 check_parallelism(pick_orange_box_3,place_orange_box_3)
   >= -1e+12
 R208: raw_overlapping(pick_orange_box_3,place_orange_box_3)
   - 1e+12 check_parallelism(pick_orange_box_3,place_orange_box_3)
   <= -1e-12
 R209: raw_overlapping(pick_orange_box_3,place_orange_box_4)
   - min_t_end(pick_orange_box_3,_place_orange_box_4)
   + max_t_start(pick_orange_box_3,_place_orange_box_4) = 0
 R210: raw_overlapping(pick_orange_box_3,place_orange_box_4)
   - 1e+12 check_parallelism(pick_orange_box_3,place_orange_box_4)
   >= -1e+12
 R211: raw_overlapping(pick_orange_box_3,place_orange_box_4)
   - 1e+12 check_parallelism(pick_orange_box_3,place_orange_box_4)
   <= -1e-12
 R212: raw_overlapping(pick_orange_box_3,place_blue_box_1)
   - min_t_end(pick_orange_box_3,_place_blue_box_1)
   + max_t_start(pick_orange_box_3,_place_blue_box_1) = 0
 R213: raw_overlapping(pick_orange_box_3,place_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_3,place_blue_box_1) >= -1e+12
 R214: raw_overlapping(pick_orange_box_3,place_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_3,place_blue_box_1) <= -1e-12
 R215: raw_overlapping(pick_orange_box_3,place_blue_box_2)
   - min_t_end(pick_orange_box_3,_place_blue_box_2)
   + max_t_start(pick_orange_box_3,_place_blue_box_2) = 0
 R216: raw_overlapping(pick_orange_box_3,place_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_3,place_blue_box_2) >= -1e+12
 R217: raw_overlapping(pick_orange_box_3,place_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_3,place_blue_box_2) <= -1e-12
 R218: raw_overlapping(pick_orange_box_3,place_blue_box_3)
   - min_t_end(pick_orange_box_3,_place_blue_box_3)
   + max_t_start(pick_orange_box_3,_place_blue_box_3) = 0
 R219: raw_overlapping(pick_orange_box_3,place_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_3,place_blue_box_3) >= -1e+12
 R220: raw_overlapping(pick_orange_box_3,place_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_3,place_blue_box_3) <= -1e-12
 R221: raw_overlapping(pick_orange_box_3,place_blue_box_4)
   - min_t_end(pick_orange_box_3,_place_blue_box_4)
   + max_t_start(pick_orange_box_3,_place_blue_box_4) = 0
 R222: raw_overlapping(pick_orange_box_3,place_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_3,place_blue_box_4) >= -1e+12
 R223: raw_overlapping(pick_orange_box_3,place_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_3,place_blue_box_4) <= -1e-12
 R224: raw_overlapping(pick_orange_box_3,place_white_box_1)
   - min_t_end(pick_orange_box_3,_place_white_box_1)
   + max_t_start(pick_orange_box_3,_place_white_box_1) = 0
 R225: raw_overlapping(pick_orange_box_3,place_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_3,place_white_box_1)
   >= -1e+12
 R226: raw_overlapping(pick_orange_box_3,place_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_3,place_white_box_1)
   <= -1e-12
 R227: raw_overlapping(pick_orange_box_3,place_white_box_2)
   - min_t_end(pick_orange_box_3,_place_white_box_2)
   + max_t_start(pick_orange_box_3,_place_white_box_2) = 0
 R228: raw_overlapping(pick_orange_box_3,place_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_3,place_white_box_2)
   >= -1e+12
 R229: raw_overlapping(pick_orange_box_3,place_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_3,place_white_box_2)
   <= -1e-12
 R230: raw_overlapping(pick_orange_box_3,place_white_box_3)
   - min_t_end(pick_orange_box_3,_place_white_box_3)
   + max_t_start(pick_orange_box_3,_place_white_box_3) = 0
 R231: raw_overlapping(pick_orange_box_3,place_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_3,place_white_box_3)
   >= -1e+12
 R232: raw_overlapping(pick_orange_box_3,place_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_3,place_white_box_3)
   <= -1e-12
 R233: raw_overlapping(pick_orange_box_3,place_white_box_4)
   - min_t_end(pick_orange_box_3,_place_white_box_4)
   + max_t_start(pick_orange_box_3,_place_white_box_4) = 0
 R234: raw_overlapping(pick_orange_box_3,place_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_3,place_white_box_4)
   >= -1e+12
 R235: raw_overlapping(pick_orange_box_3,place_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_3,place_white_box_4)
   <= -1e-12
 R236: raw_overlapping(pick_orange_box_4,pick_blue_box_1)
   - min_t_end(pick_orange_box_4,_pick_blue_box_1)
   + max_t_start(pick_orange_box_4,_pick_blue_box_1) = 0
 R237: raw_overlapping(pick_orange_box_4,pick_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_blue_box_1) >= -1e+12
 R238: raw_overlapping(pick_orange_box_4,pick_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_blue_box_1) <= -1e-12
 R239: raw_overlapping(pick_orange_box_4,pick_blue_box_2)
   - min_t_end(pick_orange_box_4,_pick_blue_box_2)
   + max_t_start(pick_orange_box_4,_pick_blue_box_2) = 0
 R240: raw_overlapping(pick_orange_box_4,pick_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_blue_box_2) >= -1e+12
 R241: raw_overlapping(pick_orange_box_4,pick_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_blue_box_2) <= -1e-12
 R242: raw_overlapping(pick_orange_box_4,pick_blue_box_3)
   - min_t_end(pick_orange_box_4,_pick_blue_box_3)
   + max_t_start(pick_orange_box_4,_pick_blue_box_3) = 0
 R243: raw_overlapping(pick_orange_box_4,pick_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_blue_box_3) >= -1e+12
 R244: raw_overlapping(pick_orange_box_4,pick_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_blue_box_3) <= -1e-12
 R245: raw_overlapping(pick_orange_box_4,pick_blue_box_4)
   - min_t_end(pick_orange_box_4,_pick_blue_box_4)
   + max_t_start(pick_orange_box_4,_pick_blue_box_4) = 0
 R246: raw_overlapping(pick_orange_box_4,pick_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_blue_box_4) >= -1e+12
 R247: raw_overlapping(pick_orange_box_4,pick_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_blue_box_4) <= -1e-12
 R248: raw_overlapping(pick_orange_box_4,pick_white_box_1)
   - min_t_end(pick_orange_box_4,_pick_white_box_1)
   + max_t_start(pick_orange_box_4,_pick_white_box_1) = 0
 R249: raw_overlapping(pick_orange_box_4,pick_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_white_box_1) >= -1e+12
 R250: raw_overlapping(pick_orange_box_4,pick_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_white_box_1) <= -1e-12
 R251: raw_overlapping(pick_orange_box_4,pick_white_box_2)
   - min_t_end(pick_orange_box_4,_pick_white_box_2)
   + max_t_start(pick_orange_box_4,_pick_white_box_2) = 0
 R252: raw_overlapping(pick_orange_box_4,pick_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_white_box_2) >= -1e+12
 R253: raw_overlapping(pick_orange_box_4,pick_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_white_box_2) <= -1e-12
 R254: raw_overlapping(pick_orange_box_4,pick_white_box_3)
   - min_t_end(pick_orange_box_4,_pick_white_box_3)
   + max_t_start(pick_orange_box_4,_pick_white_box_3) = 0
 R255: raw_overlapping(pick_orange_box_4,pick_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_white_box_3) >= -1e+12
 R256: raw_overlapping(pick_orange_box_4,pick_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_white_box_3) <= -1e-12
 R257: raw_overlapping(pick_orange_box_4,pick_white_box_4)
   - min_t_end(pick_orange_box_4,_pick_white_box_4)
   + max_t_start(pick_orange_box_4,_pick_white_box_4) = 0
 R258: raw_overlapping(pick_orange_box_4,pick_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_white_box_4) >= -1e+12
 R259: raw_overlapping(pick_orange_box_4,pick_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_4,pick_white_box_4) <= -1e-12
 R260: raw_overlapping(pick_orange_box_4,place_orange_box_1)
   - min_t_end(pick_orange_box_4,_place_orange_box_1)
   + max_t_start(pick_orange_box_4,_place_orange_box_1) = 0
 R261: raw_overlapping(pick_orange_box_4,place_orange_box_1)
   - 1e+12 check_parallelism(pick_orange_box_4,place_orange_box_1)
   >= -1e+12
 R262: raw_overlapping(pick_orange_box_4,place_orange_box_1)
   - 1e+12 check_parallelism(pick_orange_box_4,place_orange_box_1)
   <= -1e-12
 R263: raw_overlapping(pick_orange_box_4,place_orange_box_2)
   - min_t_end(pick_orange_box_4,_place_orange_box_2)
   + max_t_start(pick_orange_box_4,_place_orange_box_2) = 0
 R264: raw_overlapping(pick_orange_box_4,place_orange_box_2)
   - 1e+12 check_parallelism(pick_orange_box_4,place_orange_box_2)
   >= -1e+12
 R265: raw_overlapping(pick_orange_box_4,place_orange_box_2)
   - 1e+12 check_parallelism(pick_orange_box_4,place_orange_box_2)
   <= -1e-12
 R266: raw_overlapping(pick_orange_box_4,place_orange_box_3)
   - min_t_end(pick_orange_box_4,_place_orange_box_3)
   + max_t_start(pick_orange_box_4,_place_orange_box_3) = 0
 R267: raw_overlapping(pick_orange_box_4,place_orange_box_3)
   - 1e+12 check_parallelism(pick_orange_box_4,place_orange_box_3)
   >= -1e+12
 R268: raw_overlapping(pick_orange_box_4,place_orange_box_3)
   - 1e+12 check_parallelism(pick_orange_box_4,place_orange_box_3)
   <= -1e-12
 R269: raw_overlapping(pick_orange_box_4,place_orange_box_4)
   - min_t_end(pick_orange_box_4,_place_orange_box_4)
   + max_t_start(pick_orange_box_4,_place_orange_box_4) = 0
 R270: raw_overlapping(pick_orange_box_4,place_orange_box_4)
   - 1e+12 check_parallelism(pick_orange_box_4,place_orange_box_4)
   >= -1e+12
 R271: raw_overlapping(pick_orange_box_4,place_orange_box_4)
   - 1e+12 check_parallelism(pick_orange_box_4,place_orange_box_4)
   <= -1e-12
 R272: raw_overlapping(pick_orange_box_4,place_blue_box_1)
   - min_t_end(pick_orange_box_4,_place_blue_box_1)
   + max_t_start(pick_orange_box_4,_place_blue_box_1) = 0
 R273: raw_overlapping(pick_orange_box_4,place_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_4,place_blue_box_1) >= -1e+12
 R274: raw_overlapping(pick_orange_box_4,place_blue_box_1)
   - 1e+12 check_parallelism(pick_orange_box_4,place_blue_box_1) <= -1e-12
 R275: raw_overlapping(pick_orange_box_4,place_blue_box_2)
   - min_t_end(pick_orange_box_4,_place_blue_box_2)
   + max_t_start(pick_orange_box_4,_place_blue_box_2) = 0
 R276: raw_overlapping(pick_orange_box_4,place_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_4,place_blue_box_2) >= -1e+12
 R277: raw_overlapping(pick_orange_box_4,place_blue_box_2)
   - 1e+12 check_parallelism(pick_orange_box_4,place_blue_box_2) <= -1e-12
 R278: raw_overlapping(pick_orange_box_4,place_blue_box_3)
   - min_t_end(pick_orange_box_4,_place_blue_box_3)
   + max_t_start(pick_orange_box_4,_place_blue_box_3) = 0
 R279: raw_overlapping(pick_orange_box_4,place_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_4,place_blue_box_3) >= -1e+12
 R280: raw_overlapping(pick_orange_box_4,place_blue_box_3)
   - 1e+12 check_parallelism(pick_orange_box_4,place_blue_box_3) <= -1e-12
 R281: raw_overlapping(pick_orange_box_4,place_blue_box_4)
   - min_t_end(pick_orange_box_4,_place_blue_box_4)
   + max_t_start(pick_orange_box_4,_place_blue_box_4) = 0
 R282: raw_overlapping(pick_orange_box_4,place_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_4,place_blue_box_4) >= -1e+12
 R283: raw_overlapping(pick_orange_box_4,place_blue_box_4)
   - 1e+12 check_parallelism(pick_orange_box_4,place_blue_box_4) <= -1e-12
 R284: raw_overlapping(pick_orange_box_4,place_white_box_1)
   - min_t_end(pick_orange_box_4,_place_white_box_1)
   + max_t_start(pick_orange_box_4,_place_white_box_1) = 0
 R285: raw_overlapping(pick_orange_box_4,place_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_4,place_white_box_1)
   >= -1e+12
 R286: raw_overlapping(pick_orange_box_4,place_white_box_1)
   - 1e+12 check_parallelism(pick_orange_box_4,place_white_box_1)
   <= -1e-12
 R287: raw_overlapping(pick_orange_box_4,place_white_box_2)
   - min_t_end(pick_orange_box_4,_place_white_box_2)
   + max_t_start(pick_orange_box_4,_place_white_box_2) = 0
 R288: raw_overlapping(pick_orange_box_4,place_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_4,place_white_box_2)
   >= -1e+12
 R289: raw_overlapping(pick_orange_box_4,place_white_box_2)
   - 1e+12 check_parallelism(pick_orange_box_4,place_white_box_2)
   <= -1e-12
 R290: raw_overlapping(pick_orange_box_4,place_white_box_3)
   - min_t_end(pick_orange_box_4,_place_white_box_3)
   + max_t_start(pick_orange_box_4,_place_white_box_3) = 0
 R291: raw_overlapping(pick_orange_box_4,place_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_4,place_white_box_3)
   >= -1e+12
 R292: raw_overlapping(pick_orange_box_4,place_white_box_3)
   - 1e+12 check_parallelism(pick_orange_box_4,place_white_box_3)
   <= -1e-12
 R293: raw_overlapping(pick_orange_box_4,place_white_box_4)
   - min_t_end(pick_orange_box_4,_place_white_box_4)
   + max_t_start(pick_orange_box_4,_place_white_box_4) = 0
 R294: raw_overlapping(pick_orange_box_4,place_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_4,place_white_box_4)
   >= -1e+12
 R295: raw_overlapping(pick_orange_box_4,place_white_box_4)
   - 1e+12 check_parallelism(pick_orange_box_4,place_white_box_4)
   <= -1e-12
 R296: raw_overlapping(pick_blue_box_1,pick_blue_box_2)
   - min_t_end(pick_blue_box_1,_pick_blue_box_2)
   + max_t_start(pick_blue_box_1,_pick_blue_box_2) = 0
 R297: raw_overlapping(pick_blue_box_1,pick_blue_box_2)
   - 1e+12 check_parallelism(pick_blue_box_1,pick_blue_box_2) >= -1e+12
 R298: raw_overlapping(pick_blue_box_1,pick_blue_box_2)
   - 1e+12 check_parallelism(pick_blue_box_1,pick_blue_box_2) <= -1e-12
 R299: raw_overlapping(pick_blue_box_1,pick_blue_box_3)
   - min_t_end(pick_blue_box_1,_pick_blue_box_3)
   + max_t_start(pick_blue_box_1,_pick_blue_box_3) = 0
 R300: raw_overlapping(pick_blue_box_1,pick_blue_box_3)
   - 1e+12 check_parallelism(pick_blue_box_1,pick_blue_box_3) >= -1e+12
 R301: raw_overlapping(pick_blue_box_1,pick_blue_box_3)
   - 1e+12 check_parallelism(pick_blue_box_1,pick_blue_box_3) <= -1e-12
 R302: raw_overlapping(pick_blue_box_1,pick_blue_box_4)
   - min_t_end(pick_blue_box_1,_pick_blue_box_4)
   + max_t_start(pick_blue_box_1,_pick_blue_box_4) = 0
 R303: raw_overlapping(pick_blue_box_1,pick_blue_box_4)
   - 1e+12 check_parallelism(pick_blue_box_1,pick_blue_box_4) >= -1e+12
 R304: raw_overlapping(pick_blue_box_1,pick_blue_box_4)
   - 1e+12 check_parallelism(pick_blue_box_1,pick_blue_box_4) <= -1e-12
 R305: raw_overlapping(pick_blue_box_1,pick_white_box_1)
   - min_t_end(pick_blue_box_1,_pick_white_box_1)
   + max_t_start(pick_blue_box_1,_pick_white_box_1) = 0
 R306: raw_overlapping(pick_blue_box_1,pick_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_1,pick_white_box_1) >= -1e+12
 R307: raw_overlapping(pick_blue_box_1,pick_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_1,pick_white_box_1) <= -1e-12
 R308: raw_overlapping(pick_blue_box_1,pick_white_box_2)
   - min_t_end(pick_blue_box_1,_pick_white_box_2)
   + max_t_start(pick_blue_box_1,_pick_white_box_2) = 0
 R309: raw_overlapping(pick_blue_box_1,pick_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_1,pick_white_box_2) >= -1e+12
 R310: raw_overlapping(pick_blue_box_1,pick_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_1,pick_white_box_2) <= -1e-12
 R311: raw_overlapping(pick_blue_box_1,pick_white_box_3)
   - min_t_end(pick_blue_box_1,_pick_white_box_3)
   + max_t_start(pick_blue_box_1,_pick_white_box_3) = 0
 R312: raw_overlapping(pick_blue_box_1,pick_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_1,pick_white_box_3) >= -1e+12
 R313: raw_overlapping(pick_blue_box_1,pick_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_1,pick_white_box_3) <= -1e-12
 R314: raw_overlapping(pick_blue_box_1,pick_white_box_4)
   - min_t_end(pick_blue_box_1,_pick_white_box_4)
   + max_t_start(pick_blue_box_1,_pick_white_box_4) = 0
 R315: raw_overlapping(pick_blue_box_1,pick_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_1,pick_white_box_4) >= -1e+12
 R316: raw_overlapping(pick_blue_box_1,pick_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_1,pick_white_box_4) <= -1e-12
 R317: raw_overlapping(pick_blue_box_1,place_orange_box_1)
   - min_t_end(pick_blue_box_1,_place_orange_box_1)
   + max_t_start(pick_blue_box_1,_place_orange_box_1) = 0
 R318: raw_overlapping(pick_blue_box_1,place_orange_box_1)
   - 1e+12 check_parallelism(pick_blue_box_1,place_orange_box_1) >= -1e+12
 R319: raw_overlapping(pick_blue_box_1,place_orange_box_1)
   - 1e+12 check_parallelism(pick_blue_box_1,place_orange_box_1) <= -1e-12
 R320: raw_overlapping(pick_blue_box_1,place_orange_box_2)
   - min_t_end(pick_blue_box_1,_place_orange_box_2)
   + max_t_start(pick_blue_box_1,_place_orange_box_2) = 0
 R321: raw_overlapping(pick_blue_box_1,place_orange_box_2)
   - 1e+12 check_parallelism(pick_blue_box_1,place_orange_box_2) >= -1e+12
 R322: raw_overlapping(pick_blue_box_1,place_orange_box_2)
   - 1e+12 check_parallelism(pick_blue_box_1,place_orange_box_2) <= -1e-12
 R323: raw_overlapping(pick_blue_box_1,place_orange_box_3)
   - min_t_end(pick_blue_box_1,_place_orange_box_3)
   + max_t_start(pick_blue_box_1,_place_orange_box_3) = 0
 R324: raw_overlapping(pick_blue_box_1,place_orange_box_3)
   - 1e+12 check_parallelism(pick_blue_box_1,place_orange_box_3) >= -1e+12
 R325: raw_overlapping(pick_blue_box_1,place_orange_box_3)
   - 1e+12 check_parallelism(pick_blue_box_1,place_orange_box_3) <= -1e-12
 R326: raw_overlapping(pick_blue_box_1,place_orange_box_4)
   - min_t_end(pick_blue_box_1,_place_orange_box_4)
   + max_t_start(pick_blue_box_1,_place_orange_box_4) = 0
 R327: raw_overlapping(pick_blue_box_1,place_orange_box_4)
   - 1e+12 check_parallelism(pick_blue_box_1,place_orange_box_4) >= -1e+12
 R328: raw_overlapping(pick_blue_box_1,place_orange_box_4)
   - 1e+12 check_parallelism(pick_blue_box_1,place_orange_box_4) <= -1e-12
 R329: raw_overlapping(pick_blue_box_1,place_blue_box_1)
   - min_t_end(pick_blue_box_1,_place_blue_box_1)
   + max_t_start(pick_blue_box_1,_place_blue_box_1) = 0
 R330: raw_overlapping(pick_blue_box_1,place_blue_box_1)
   - 1e+12 check_parallelism(pick_blue_box_1,place_blue_box_1) >= -1e+12
 R331: raw_overlapping(pick_blue_box_1,place_blue_box_1)
   - 1e+12 check_parallelism(pick_blue_box_1,place_blue_box_1) <= -1e-12
 R332: raw_overlapping(pick_blue_box_1,place_blue_box_2)
   - min_t_end(pick_blue_box_1,_place_blue_box_2)
   + max_t_start(pick_blue_box_1,_place_blue_box_2) = 0
 R333: raw_overlapping(pick_blue_box_1,place_blue_box_2)
   - 1e+12 check_parallelism(pick_blue_box_1,place_blue_box_2) >= -1e+12
 R334: raw_overlapping(pick_blue_box_1,place_blue_box_2)
   - 1e+12 check_parallelism(pick_blue_box_1,place_blue_box_2) <= -1e-12
 R335: raw_overlapping(pick_blue_box_1,place_blue_box_3)
   - min_t_end(pick_blue_box_1,_place_blue_box_3)
   + max_t_start(pick_blue_box_1,_place_blue_box_3) = 0
 R336: raw_overlapping(pick_blue_box_1,place_blue_box_3)
   - 1e+12 check_parallelism(pick_blue_box_1,place_blue_box_3) >= -1e+12
 R337: raw_overlapping(pick_blue_box_1,place_blue_box_3)
   - 1e+12 check_parallelism(pick_blue_box_1,place_blue_box_3) <= -1e-12
 R338: raw_overlapping(pick_blue_box_1,place_blue_box_4)
   - min_t_end(pick_blue_box_1,_place_blue_box_4)
   + max_t_start(pick_blue_box_1,_place_blue_box_4) = 0
 R339: raw_overlapping(pick_blue_box_1,place_blue_box_4)
   - 1e+12 check_parallelism(pick_blue_box_1,place_blue_box_4) >= -1e+12
 R340: raw_overlapping(pick_blue_box_1,place_blue_box_4)
   - 1e+12 check_parallelism(pick_blue_box_1,place_blue_box_4) <= -1e-12
 R341: raw_overlapping(pick_blue_box_1,place_white_box_1)
   - min_t_end(pick_blue_box_1,_place_white_box_1)
   + max_t_start(pick_blue_box_1,_place_white_box_1) = 0
 R342: raw_overlapping(pick_blue_box_1,place_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_1,place_white_box_1) >= -1e+12
 R343: raw_overlapping(pick_blue_box_1,place_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_1,place_white_box_1) <= -1e-12
 R344: raw_overlapping(pick_blue_box_1,place_white_box_2)
   - min_t_end(pick_blue_box_1,_place_white_box_2)
   + max_t_start(pick_blue_box_1,_place_white_box_2) = 0
 R345: raw_overlapping(pick_blue_box_1,place_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_1,place_white_box_2) >= -1e+12
 R346: raw_overlapping(pick_blue_box_1,place_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_1,place_white_box_2) <= -1e-12
 R347: raw_overlapping(pick_blue_box_1,place_white_box_3)
   - min_t_end(pick_blue_box_1,_place_white_box_3)
   + max_t_start(pick_blue_box_1,_place_white_box_3) = 0
 R348: raw_overlapping(pick_blue_box_1,place_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_1,place_white_box_3) >= -1e+12
 R349: raw_overlapping(pick_blue_box_1,place_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_1,place_white_box_3) <= -1e-12
 R350: raw_overlapping(pick_blue_box_1,place_white_box_4)
   - min_t_end(pick_blue_box_1,_place_white_box_4)
   + max_t_start(pick_blue_box_1,_place_white_box_4) = 0
 R351: raw_overlapping(pick_blue_box_1,place_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_1,place_white_box_4) >= -1e+12
 R352: raw_overlapping(pick_blue_box_1,place_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_1,place_white_box_4) <= -1e-12
 R353: raw_overlapping(pick_blue_box_2,pick_blue_box_3)
   - min_t_end(pick_blue_box_2,_pick_blue_box_3)
   + max_t_start(pick_blue_box_2,_pick_blue_box_3) = 0
 R354: raw_overlapping(pick_blue_box_2,pick_blue_box_3)
   - 1e+12 check_parallelism(pick_blue_box_2,pick_blue_box_3) >= -1e+12
 R355: raw_overlapping(pick_blue_box_2,pick_blue_box_3)
   - 1e+12 check_parallelism(pick_blue_box_2,pick_blue_box_3) <= -1e-12
 R356: raw_overlapping(pick_blue_box_2,pick_blue_box_4)
   - min_t_end(pick_blue_box_2,_pick_blue_box_4)
   + max_t_start(pick_blue_box_2,_pick_blue_box_4) = 0
 R357: raw_overlapping(pick_blue_box_2,pick_blue_box_4)
   - 1e+12 check_parallelism(pick_blue_box_2,pick_blue_box_4) >= -1e+12
 R358: raw_overlapping(pick_blue_box_2,pick_blue_box_4)
   - 1e+12 check_parallelism(pick_blue_box_2,pick_blue_box_4) <= -1e-12
 R359: raw_overlapping(pick_blue_box_2,pick_white_box_1)
   - min_t_end(pick_blue_box_2,_pick_white_box_1)
   + max_t_start(pick_blue_box_2,_pick_white_box_1) = 0
 R360: raw_overlapping(pick_blue_box_2,pick_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_2,pick_white_box_1) >= -1e+12
 R361: raw_overlapping(pick_blue_box_2,pick_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_2,pick_white_box_1) <= -1e-12
 R362: raw_overlapping(pick_blue_box_2,pick_white_box_2)
   - min_t_end(pick_blue_box_2,_pick_white_box_2)
   + max_t_start(pick_blue_box_2,_pick_white_box_2) = 0
 R363: raw_overlapping(pick_blue_box_2,pick_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_2,pick_white_box_2) >= -1e+12
 R364: raw_overlapping(pick_blue_box_2,pick_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_2,pick_white_box_2) <= -1e-12
 R365: raw_overlapping(pick_blue_box_2,pick_white_box_3)
   - min_t_end(pick_blue_box_2,_pick_white_box_3)
   + max_t_start(pick_blue_box_2,_pick_white_box_3) = 0
 R366: raw_overlapping(pick_blue_box_2,pick_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_2,pick_white_box_3) >= -1e+12
 R367: raw_overlapping(pick_blue_box_2,pick_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_2,pick_white_box_3) <= -1e-12
 R368: raw_overlapping(pick_blue_box_2,pick_white_box_4)
   - min_t_end(pick_blue_box_2,_pick_white_box_4)
   + max_t_start(pick_blue_box_2,_pick_white_box_4) = 0
 R369: raw_overlapping(pick_blue_box_2,pick_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_2,pick_white_box_4) >= -1e+12
 R370: raw_overlapping(pick_blue_box_2,pick_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_2,pick_white_box_4) <= -1e-12
 R371: raw_overlapping(pick_blue_box_2,place_orange_box_1)
   - min_t_end(pick_blue_box_2,_place_orange_box_1)
   + max_t_start(pick_blue_box_2,_place_orange_box_1) = 0
 R372: raw_overlapping(pick_blue_box_2,place_orange_box_1)
   - 1e+12 check_parallelism(pick_blue_box_2,place_orange_box_1) >= -1e+12
 R373: raw_overlapping(pick_blue_box_2,place_orange_box_1)
   - 1e+12 check_parallelism(pick_blue_box_2,place_orange_box_1) <= -1e-12
 R374: raw_overlapping(pick_blue_box_2,place_orange_box_2)
   - min_t_end(pick_blue_box_2,_place_orange_box_2)
   + max_t_start(pick_blue_box_2,_place_orange_box_2) = 0
 R375: raw_overlapping(pick_blue_box_2,place_orange_box_2)
   - 1e+12 check_parallelism(pick_blue_box_2,place_orange_box_2) >= -1e+12
 R376: raw_overlapping(pick_blue_box_2,place_orange_box_2)
   - 1e+12 check_parallelism(pick_blue_box_2,place_orange_box_2) <= -1e-12
 R377: raw_overlapping(pick_blue_box_2,place_orange_box_3)
   - min_t_end(pick_blue_box_2,_place_orange_box_3)
   + max_t_start(pick_blue_box_2,_place_orange_box_3) = 0
 R378: raw_overlapping(pick_blue_box_2,place_orange_box_3)
   - 1e+12 check_parallelism(pick_blue_box_2,place_orange_box_3) >= -1e+12
 R379: raw_overlapping(pick_blue_box_2,place_orange_box_3)
   - 1e+12 check_parallelism(pick_blue_box_2,place_orange_box_3) <= -1e-12
 R380: raw_overlapping(pick_blue_box_2,place_orange_box_4)
   - min_t_end(pick_blue_box_2,_place_orange_box_4)
   + max_t_start(pick_blue_box_2,_place_orange_box_4) = 0
 R381: raw_overlapping(pick_blue_box_2,place_orange_box_4)
   - 1e+12 check_parallelism(pick_blue_box_2,place_orange_box_4) >= -1e+12
 R382: raw_overlapping(pick_blue_box_2,place_orange_box_4)
   - 1e+12 check_parallelism(pick_blue_box_2,place_orange_box_4) <= -1e-12
 R383: raw_overlapping(pick_blue_box_2,place_blue_box_1)
   - min_t_end(pick_blue_box_2,_place_blue_box_1)
   + max_t_start(pick_blue_box_2,_place_blue_box_1) = 0
 R384: raw_overlapping(pick_blue_box_2,place_blue_box_1)
   - 1e+12 check_parallelism(pick_blue_box_2,place_blue_box_1) >= -1e+12
 R385: raw_overlapping(pick_blue_box_2,place_blue_box_1)
   - 1e+12 check_parallelism(pick_blue_box_2,place_blue_box_1) <= -1e-12
 R386: raw_overlapping(pick_blue_box_2,place_blue_box_2)
   - min_t_end(pick_blue_box_2,_place_blue_box_2)
   + max_t_start(pick_blue_box_2,_place_blue_box_2) = 0
 R387: raw_overlapping(pick_blue_box_2,place_blue_box_2)
   - 1e+12 check_parallelism(pick_blue_box_2,place_blue_box_2) >= -1e+12
 R388: raw_overlapping(pick_blue_box_2,place_blue_box_2)
   - 1e+12 check_parallelism(pick_blue_box_2,place_blue_box_2) <= -1e-12
 R389: raw_overlapping(pick_blue_box_2,place_blue_box_3)
   - min_t_end(pick_blue_box_2,_place_blue_box_3)
   + max_t_start(pick_blue_box_2,_place_blue_box_3) = 0
 R390: raw_overlapping(pick_blue_box_2,place_blue_box_3)
   - 1e+12 check_parallelism(pick_blue_box_2,place_blue_box_3) >= -1e+12
 R391: raw_overlapping(pick_blue_box_2,place_blue_box_3)
   - 1e+12 check_parallelism(pick_blue_box_2,place_blue_box_3) <= -1e-12
 R392: raw_overlapping(pick_blue_box_2,place_blue_box_4)
   - min_t_end(pick_blue_box_2,_place_blue_box_4)
   + max_t_start(pick_blue_box_2,_place_blue_box_4) = 0
 R393: raw_overlapping(pick_blue_box_2,place_blue_box_4)
   - 1e+12 check_parallelism(pick_blue_box_2,place_blue_box_4) >= -1e+12
 R394: raw_overlapping(pick_blue_box_2,place_blue_box_4)
   - 1e+12 check_parallelism(pick_blue_box_2,place_blue_box_4) <= -1e-12
 R395: raw_overlapping(pick_blue_box_2,place_white_box_1)
   - min_t_end(pick_blue_box_2,_place_white_box_1)
   + max_t_start(pick_blue_box_2,_place_white_box_1) = 0
 R396: raw_overlapping(pick_blue_box_2,place_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_2,place_white_box_1) >= -1e+12
 R397: raw_overlapping(pick_blue_box_2,place_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_2,place_white_box_1) <= -1e-12
 R398: raw_overlapping(pick_blue_box_2,place_white_box_2)
   - min_t_end(pick_blue_box_2,_place_white_box_2)
   + max_t_start(pick_blue_box_2,_place_white_box_2) = 0
 R399: raw_overlapping(pick_blue_box_2,place_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_2,place_white_box_2) >= -1e+12
 R400: raw_overlapping(pick_blue_box_2,place_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_2,place_white_box_2) <= -1e-12
 R401: raw_overlapping(pick_blue_box_2,place_white_box_3)
   - min_t_end(pick_blue_box_2,_place_white_box_3)
   + max_t_start(pick_blue_box_2,_place_white_box_3) = 0
 R402: raw_overlapping(pick_blue_box_2,place_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_2,place_white_box_3) >= -1e+12
 R403: raw_overlapping(pick_blue_box_2,place_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_2,place_white_box_3) <= -1e-12
 R404: raw_overlapping(pick_blue_box_2,place_white_box_4)
   - min_t_end(pick_blue_box_2,_place_white_box_4)
   + max_t_start(pick_blue_box_2,_place_white_box_4) = 0
 R405: raw_overlapping(pick_blue_box_2,place_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_2,place_white_box_4) >= -1e+12
 R406: raw_overlapping(pick_blue_box_2,place_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_2,place_white_box_4) <= -1e-12
 R407: raw_overlapping(pick_blue_box_3,pick_blue_box_4)
   - min_t_end(pick_blue_box_3,_pick_blue_box_4)
   + max_t_start(pick_blue_box_3,_pick_blue_box_4) = 0
 R408: raw_overlapping(pick_blue_box_3,pick_blue_box_4)
   - 1e+12 check_parallelism(pick_blue_box_3,pick_blue_box_4) >= -1e+12
 R409: raw_overlapping(pick_blue_box_3,pick_blue_box_4)
   - 1e+12 check_parallelism(pick_blue_box_3,pick_blue_box_4) <= -1e-12
 R410: raw_overlapping(pick_blue_box_3,pick_white_box_1)
   - min_t_end(pick_blue_box_3,_pick_white_box_1)
   + max_t_start(pick_blue_box_3,_pick_white_box_1) = 0
 R411: raw_overlapping(pick_blue_box_3,pick_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_3,pick_white_box_1) >= -1e+12
 R412: raw_overlapping(pick_blue_box_3,pick_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_3,pick_white_box_1) <= -1e-12
 R413: raw_overlapping(pick_blue_box_3,pick_white_box_2)
   - min_t_end(pick_blue_box_3,_pick_white_box_2)
   + max_t_start(pick_blue_box_3,_pick_white_box_2) = 0
 R414: raw_overlapping(pick_blue_box_3,pick_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_3,pick_white_box_2) >= -1e+12
 R415: raw_overlapping(pick_blue_box_3,pick_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_3,pick_white_box_2) <= -1e-12
 R416: raw_overlapping(pick_blue_box_3,pick_white_box_3)
   - min_t_end(pick_blue_box_3,_pick_white_box_3)
   + max_t_start(pick_blue_box_3,_pick_white_box_3) = 0
 R417: raw_overlapping(pick_blue_box_3,pick_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_3,pick_white_box_3) >= -1e+12
 R418: raw_overlapping(pick_blue_box_3,pick_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_3,pick_white_box_3) <= -1e-12
 R419: raw_overlapping(pick_blue_box_3,pick_white_box_4)
   - min_t_end(pick_blue_box_3,_pick_white_box_4)
   + max_t_start(pick_blue_box_3,_pick_white_box_4) = 0
 R420: raw_overlapping(pick_blue_box_3,pick_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_3,pick_white_box_4) >= -1e+12
 R421: raw_overlapping(pick_blue_box_3,pick_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_3,pick_white_box_4) <= -1e-12
 R422: raw_overlapping(pick_blue_box_3,place_orange_box_1)
   - min_t_end(pick_blue_box_3,_place_orange_box_1)
   + max_t_start(pick_blue_box_3,_place_orange_box_1) = 0
 R423: raw_overlapping(pick_blue_box_3,place_orange_box_1)
   - 1e+12 check_parallelism(pick_blue_box_3,place_orange_box_1) >= -1e+12
 R424: raw_overlapping(pick_blue_box_3,place_orange_box_1)
   - 1e+12 check_parallelism(pick_blue_box_3,place_orange_box_1) <= -1e-12
 R425: raw_overlapping(pick_blue_box_3,place_orange_box_2)
   - min_t_end(pick_blue_box_3,_place_orange_box_2)
   + max_t_start(pick_blue_box_3,_place_orange_box_2) = 0
 R426: raw_overlapping(pick_blue_box_3,place_orange_box_2)
   - 1e+12 check_parallelism(pick_blue_box_3,place_orange_box_2) >= -1e+12
 R427: raw_overlapping(pick_blue_box_3,place_orange_box_2)
   - 1e+12 check_parallelism(pick_blue_box_3,place_orange_box_2) <= -1e-12
 R428: raw_overlapping(pick_blue_box_3,place_orange_box_3)
   - min_t_end(pick_blue_box_3,_place_orange_box_3)
   + max_t_start(pick_blue_box_3,_place_orange_box_3) = 0
 R429: raw_overlapping(pick_blue_box_3,place_orange_box_3)
   - 1e+12 check_parallelism(pick_blue_box_3,place_orange_box_3) >= -1e+12
 R430: raw_overlapping(pick_blue_box_3,place_orange_box_3)
   - 1e+12 check_parallelism(pick_blue_box_3,place_orange_box_3) <= -1e-12
 R431: raw_overlapping(pick_blue_box_3,place_orange_box_4)
   - min_t_end(pick_blue_box_3,_place_orange_box_4)
   + max_t_start(pick_blue_box_3,_place_orange_box_4) = 0
 R432: raw_overlapping(pick_blue_box_3,place_orange_box_4)
   - 1e+12 check_parallelism(pick_blue_box_3,place_orange_box_4) >= -1e+12
 R433: raw_overlapping(pick_blue_box_3,place_orange_box_4)
   - 1e+12 check_parallelism(pick_blue_box_3,place_orange_box_4) <= -1e-12
 R434: raw_overlapping(pick_blue_box_3,place_blue_box_1)
   - min_t_end(pick_blue_box_3,_place_blue_box_1)
   + max_t_start(pick_blue_box_3,_place_blue_box_1) = 0
 R435: raw_overlapping(pick_blue_box_3,place_blue_box_1)
   - 1e+12 check_parallelism(pick_blue_box_3,place_blue_box_1) >= -1e+12
 R436: raw_overlapping(pick_blue_box_3,place_blue_box_1)
   - 1e+12 check_parallelism(pick_blue_box_3,place_blue_box_1) <= -1e-12
 R437: raw_overlapping(pick_blue_box_3,place_blue_box_2)
   - min_t_end(pick_blue_box_3,_place_blue_box_2)
   + max_t_start(pick_blue_box_3,_place_blue_box_2) = 0
 R438: raw_overlapping(pick_blue_box_3,place_blue_box_2)
   - 1e+12 check_parallelism(pick_blue_box_3,place_blue_box_2) >= -1e+12
 R439: raw_overlapping(pick_blue_box_3,place_blue_box_2)
   - 1e+12 check_parallelism(pick_blue_box_3,place_blue_box_2) <= -1e-12
 R440: raw_overlapping(pick_blue_box_3,place_blue_box_3)
   - min_t_end(pick_blue_box_3,_place_blue_box_3)
   + max_t_start(pick_blue_box_3,_place_blue_box_3) = 0
 R441: raw_overlapping(pick_blue_box_3,place_blue_box_3)
   - 1e+12 check_parallelism(pick_blue_box_3,place_blue_box_3) >= -1e+12
 R442: raw_overlapping(pick_blue_box_3,place_blue_box_3)
   - 1e+12 check_parallelism(pick_blue_box_3,place_blue_box_3) <= -1e-12
 R443: raw_overlapping(pick_blue_box_3,place_blue_box_4)
   - min_t_end(pick_blue_box_3,_place_blue_box_4)
   + max_t_start(pick_blue_box_3,_place_blue_box_4) = 0
 R444: raw_overlapping(pick_blue_box_3,place_blue_box_4)
   - 1e+12 check_parallelism(pick_blue_box_3,place_blue_box_4) >= -1e+12
 R445: raw_overlapping(pick_blue_box_3,place_blue_box_4)
   - 1e+12 check_parallelism(pick_blue_box_3,place_blue_box_4) <= -1e-12
 R446: raw_overlapping(pick_blue_box_3,place_white_box_1)
   - min_t_end(pick_blue_box_3,_place_white_box_1)
   + max_t_start(pick_blue_box_3,_place_white_box_1) = 0
 R447: raw_overlapping(pick_blue_box_3,place_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_3,place_white_box_1) >= -1e+12
 R448: raw_overlapping(pick_blue_box_3,place_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_3,place_white_box_1) <= -1e-12
 R449: raw_overlapping(pick_blue_box_3,place_white_box_2)
   - min_t_end(pick_blue_box_3,_place_white_box_2)
   + max_t_start(pick_blue_box_3,_place_white_box_2) = 0
 R450: raw_overlapping(pick_blue_box_3,place_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_3,place_white_box_2) >= -1e+12
 R451: raw_overlapping(pick_blue_box_3,place_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_3,place_white_box_2) <= -1e-12
 R452: raw_overlapping(pick_blue_box_3,place_white_box_3)
   - min_t_end(pick_blue_box_3,_place_white_box_3)
   + max_t_start(pick_blue_box_3,_place_white_box_3) = 0
 R453: raw_overlapping(pick_blue_box_3,place_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_3,place_white_box_3) >= -1e+12
 R454: raw_overlapping(pick_blue_box_3,place_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_3,place_white_box_3) <= -1e-12
 R455: raw_overlapping(pick_blue_box_3,place_white_box_4)
   - min_t_end(pick_blue_box_3,_place_white_box_4)
   + max_t_start(pick_blue_box_3,_place_white_box_4) = 0
 R456: raw_overlapping(pick_blue_box_3,place_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_3,place_white_box_4) >= -1e+12
 R457: raw_overlapping(pick_blue_box_3,place_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_3,place_white_box_4) <= -1e-12
 R458: raw_overlapping(pick_blue_box_4,pick_white_box_1)
   - min_t_end(pick_blue_box_4,_pick_white_box_1)
   + max_t_start(pick_blue_box_4,_pick_white_box_1) = 0
 R459: raw_overlapping(pick_blue_box_4,pick_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_4,pick_white_box_1) >= -1e+12
 R460: raw_overlapping(pick_blue_box_4,pick_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_4,pick_white_box_1) <= -1e-12
 R461: raw_overlapping(pick_blue_box_4,pick_white_box_2)
   - min_t_end(pick_blue_box_4,_pick_white_box_2)
   + max_t_start(pick_blue_box_4,_pick_white_box_2) = 0
 R462: raw_overlapping(pick_blue_box_4,pick_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_4,pick_white_box_2) >= -1e+12
 R463: raw_overlapping(pick_blue_box_4,pick_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_4,pick_white_box_2) <= -1e-12
 R464: raw_overlapping(pick_blue_box_4,pick_white_box_3)
   - min_t_end(pick_blue_box_4,_pick_white_box_3)
   + max_t_start(pick_blue_box_4,_pick_white_box_3) = 0
 R465: raw_overlapping(pick_blue_box_4,pick_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_4,pick_white_box_3) >= -1e+12
 R466: raw_overlapping(pick_blue_box_4,pick_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_4,pick_white_box_3) <= -1e-12
 R467: raw_overlapping(pick_blue_box_4,pick_white_box_4)
   - min_t_end(pick_blue_box_4,_pick_white_box_4)
   + max_t_start(pick_blue_box_4,_pick_white_box_4) = 0
 R468: raw_overlapping(pick_blue_box_4,pick_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_4,pick_white_box_4) >= -1e+12
 R469: raw_overlapping(pick_blue_box_4,pick_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_4,pick_white_box_4) <= -1e-12
 R470: raw_overlapping(pick_blue_box_4,place_orange_box_1)
   - min_t_end(pick_blue_box_4,_place_orange_box_1)
   + max_t_start(pick_blue_box_4,_place_orange_box_1) = 0
 R471: raw_overlapping(pick_blue_box_4,place_orange_box_1)
   - 1e+12 check_parallelism(pick_blue_box_4,place_orange_box_1) >= -1e+12
 R472: raw_overlapping(pick_blue_box_4,place_orange_box_1)
   - 1e+12 check_parallelism(pick_blue_box_4,place_orange_box_1) <= -1e-12
 R473: raw_overlapping(pick_blue_box_4,place_orange_box_2)
   - min_t_end(pick_blue_box_4,_place_orange_box_2)
   + max_t_start(pick_blue_box_4,_place_orange_box_2) = 0
 R474: raw_overlapping(pick_blue_box_4,place_orange_box_2)
   - 1e+12 check_parallelism(pick_blue_box_4,place_orange_box_2) >= -1e+12
 R475: raw_overlapping(pick_blue_box_4,place_orange_box_2)
   - 1e+12 check_parallelism(pick_blue_box_4,place_orange_box_2) <= -1e-12
 R476: raw_overlapping(pick_blue_box_4,place_orange_box_3)
   - min_t_end(pick_blue_box_4,_place_orange_box_3)
   + max_t_start(pick_blue_box_4,_place_orange_box_3) = 0
 R477: raw_overlapping(pick_blue_box_4,place_orange_box_3)
   - 1e+12 check_parallelism(pick_blue_box_4,place_orange_box_3) >= -1e+12
 R478: raw_overlapping(pick_blue_box_4,place_orange_box_3)
   - 1e+12 check_parallelism(pick_blue_box_4,place_orange_box_3) <= -1e-12
 R479: raw_overlapping(pick_blue_box_4,place_orange_box_4)
   - min_t_end(pick_blue_box_4,_place_orange_box_4)
   + max_t_start(pick_blue_box_4,_place_orange_box_4) = 0
 R480: raw_overlapping(pick_blue_box_4,place_orange_box_4)
   - 1e+12 check_parallelism(pick_blue_box_4,place_orange_box_4) >= -1e+12
 R481: raw_overlapping(pick_blue_box_4,place_orange_box_4)
   - 1e+12 check_parallelism(pick_blue_box_4,place_orange_box_4) <= -1e-12
 R482: raw_overlapping(pick_blue_box_4,place_blue_box_1)
   - min_t_end(pick_blue_box_4,_place_blue_box_1)
   + max_t_start(pick_blue_box_4,_place_blue_box_1) = 0
 R483: raw_overlapping(pick_blue_box_4,place_blue_box_1)
   - 1e+12 check_parallelism(pick_blue_box_4,place_blue_box_1) >= -1e+12
 R484: raw_overlapping(pick_blue_box_4,place_blue_box_1)
   - 1e+12 check_parallelism(pick_blue_box_4,place_blue_box_1) <= -1e-12
 R485: raw_overlapping(pick_blue_box_4,place_blue_box_2)
   - min_t_end(pick_blue_box_4,_place_blue_box_2)
   + max_t_start(pick_blue_box_4,_place_blue_box_2) = 0
 R486: raw_overlapping(pick_blue_box_4,place_blue_box_2)
   - 1e+12 check_parallelism(pick_blue_box_4,place_blue_box_2) >= -1e+12
 R487: raw_overlapping(pick_blue_box_4,place_blue_box_2)
   - 1e+12 check_parallelism(pick_blue_box_4,place_blue_box_2) <= -1e-12
 R488: raw_overlapping(pick_blue_box_4,place_blue_box_3)
   - min_t_end(pick_blue_box_4,_place_blue_box_3)
   + max_t_start(pick_blue_box_4,_place_blue_box_3) = 0
 R489: raw_overlapping(pick_blue_box_4,place_blue_box_3)
   - 1e+12 check_parallelism(pick_blue_box_4,place_blue_box_3) >= -1e+12
 R490: raw_overlapping(pick_blue_box_4,place_blue_box_3)
   - 1e+12 check_parallelism(pick_blue_box_4,place_blue_box_3) <= -1e-12
 R491: raw_overlapping(pick_blue_box_4,place_blue_box_4)
   - min_t_end(pick_blue_box_4,_place_blue_box_4)
   + max_t_start(pick_blue_box_4,_place_blue_box_4) = 0
 R492: raw_overlapping(pick_blue_box_4,place_blue_box_4)
   - 1e+12 check_parallelism(pick_blue_box_4,place_blue_box_4) >= -1e+12
 R493: raw_overlapping(pick_blue_box_4,place_blue_box_4)
   - 1e+12 check_parallelism(pick_blue_box_4,place_blue_box_4) <= -1e-12
 R494: raw_overlapping(pick_blue_box_4,place_white_box_1)
   - min_t_end(pick_blue_box_4,_place_white_box_1)
   + max_t_start(pick_blue_box_4,_place_white_box_1) = 0
 R495: raw_overlapping(pick_blue_box_4,place_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_4,place_white_box_1) >= -1e+12
 R496: raw_overlapping(pick_blue_box_4,place_white_box_1)
   - 1e+12 check_parallelism(pick_blue_box_4,place_white_box_1) <= -1e-12
 R497: raw_overlapping(pick_blue_box_4,place_white_box_2)
   - min_t_end(pick_blue_box_4,_place_white_box_2)
   + max_t_start(pick_blue_box_4,_place_white_box_2) = 0
 R498: raw_overlapping(pick_blue_box_4,place_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_4,place_white_box_2) >= -1e+12
 R499: raw_overlapping(pick_blue_box_4,place_white_box_2)
   - 1e+12 check_parallelism(pick_blue_box_4,place_white_box_2) <= -1e-12
 R500: raw_overlapping(pick_blue_box_4,place_white_box_3)
   - min_t_end(pick_blue_box_4,_place_white_box_3)
   + max_t_start(pick_blue_box_4,_place_white_box_3) = 0
 R501: raw_overlapping(pick_blue_box_4,place_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_4,place_white_box_3) >= -1e+12
 R502: raw_overlapping(pick_blue_box_4,place_white_box_3)
   - 1e+12 check_parallelism(pick_blue_box_4,place_white_box_3) <= -1e-12
 R503: raw_overlapping(pick_blue_box_4,place_white_box_4)
   - min_t_end(pick_blue_box_4,_place_white_box_4)
   + max_t_start(pick_blue_box_4,_place_white_box_4) = 0
 R504: raw_overlapping(pick_blue_box_4,place_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_4,place_white_box_4) >= -1e+12
 R505: raw_overlapping(pick_blue_box_4,place_white_box_4)
   - 1e+12 check_parallelism(pick_blue_box_4,place_white_box_4) <= -1e-12
 R506: raw_overlapping(pick_white_box_1,pick_white_box_2)
   - min_t_end(pick_white_box_1,_pick_white_box_2)
   + max_t_start(pick_white_box_1,_pick_white_box_2) = 0
 R507: raw_overlapping(pick_white_box_1,pick_white_box_2)
   - 1e+12 check_parallelism(pick_white_box_1,pick_white_box_2) >= -1e+12
 R508: raw_overlapping(pick_white_box_1,pick_white_box_2)
   - 1e+12 check_parallelism(pick_white_box_1,pick_white_box_2) <= -1e-12
 R509: raw_overlapping(pick_white_box_1,pick_white_box_3)
   - min_t_end(pick_white_box_1,_pick_white_box_3)
   + max_t_start(pick_white_box_1,_pick_white_box_3) = 0
 R510: raw_overlapping(pick_white_box_1,pick_white_box_3)
   - 1e+12 check_parallelism(pick_white_box_1,pick_white_box_3) >= -1e+12
 R511: raw_overlapping(pick_white_box_1,pick_white_box_3)
   - 1e+12 check_parallelism(pick_white_box_1,pick_white_box_3) <= -1e-12
 R512: raw_overlapping(pick_white_box_1,pick_white_box_4)
   - min_t_end(pick_white_box_1,_pick_white_box_4)
   + max_t_start(pick_white_box_1,_pick_white_box_4) = 0
 R513: raw_overlapping(pick_white_box_1,pick_white_box_4)
   - 1e+12 check_parallelism(pick_white_box_1,pick_white_box_4) >= -1e+12
 R514: raw_overlapping(pick_white_box_1,pick_white_box_4)
   - 1e+12 check_parallelism(pick_white_box_1,pick_white_box_4) <= -1e-12
 R515: raw_overlapping(pick_white_box_1,place_orange_box_1)
   - min_t_end(pick_white_box_1,_place_orange_box_1)
   + max_t_start(pick_white_box_1,_place_orange_box_1) = 0
 R516: raw_overlapping(pick_white_box_1,place_orange_box_1)
   - 1e+12 check_parallelism(pick_white_box_1,place_orange_box_1)
   >= -1e+12
 R517: raw_overlapping(pick_white_box_1,place_orange_box_1)
   - 1e+12 check_parallelism(pick_white_box_1,place_orange_box_1)
   <= -1e-12
 R518: raw_overlapping(pick_white_box_1,place_orange_box_2)
   - min_t_end(pick_white_box_1,_place_orange_box_2)
   + max_t_start(pick_white_box_1,_place_orange_box_2) = 0
 R519: raw_overlapping(pick_white_box_1,place_orange_box_2)
   - 1e+12 check_parallelism(pick_white_box_1,place_orange_box_2)
   >= -1e+12
 R520: raw_overlapping(pick_white_box_1,place_orange_box_2)
   - 1e+12 check_parallelism(pick_white_box_1,place_orange_box_2)
   <= -1e-12
 R521: raw_overlapping(pick_white_box_1,place_orange_box_3)
   - min_t_end(pick_white_box_1,_place_orange_box_3)
   + max_t_start(pick_white_box_1,_place_orange_box_3) = 0
 R522: raw_overlapping(pick_white_box_1,place_orange_box_3)
   - 1e+12 check_parallelism(pick_white_box_1,place_orange_box_3)
   >= -1e+12
 R523: raw_overlapping(pick_white_box_1,place_orange_box_3)
   - 1e+12 check_parallelism(pick_white_box_1,place_orange_box_3)
   <= -1e-12
 R524: raw_overlapping(pick_white_box_1,place_orange_box_4)
   - min_t_end(pick_white_box_1,_place_orange_box_4)
   + max_t_start(pick_white_box_1,_place_orange_box_4) = 0
 R525: raw_overlapping(pick_white_box_1,place_orange_box_4)
   - 1e+12 check_parallelism(pick_white_box_1,place_orange_box_4)
   >= -1e+12
 R526: raw_overlapping(pick_white_box_1,place_orange_box_4)
   - 1e+12 check_parallelism(pick_white_box_1,place_orange_box_4)
   <= -1e-12
 R527: raw_overlapping(pick_white_box_1,place_blue_box_1)
   - min_t_end(pick_white_box_1,_place_blue_box_1)
   + max_t_start(pick_white_box_1,_place_blue_box_1) = 0
 R528: raw_overlapping(pick_white_box_1,place_blue_box_1)
   - 1e+12 check_parallelism(pick_white_box_1,place_blue_box_1) >= -1e+12
 R529: raw_overlapping(pick_white_box_1,place_blue_box_1)
   - 1e+12 check_parallelism(pick_white_box_1,place_blue_box_1) <= -1e-12
 R530: raw_overlapping(pick_white_box_1,place_blue_box_2)
   - min_t_end(pick_white_box_1,_place_blue_box_2)
   + max_t_start(pick_white_box_1,_place_blue_box_2) = 0
 R531: raw_overlapping(pick_white_box_1,place_blue_box_2)
   - 1e+12 check_parallelism(pick_white_box_1,place_blue_box_2) >= -1e+12
 R532: raw_overlapping(pick_white_box_1,place_blue_box_2)
   - 1e+12 check_parallelism(pick_white_box_1,place_blue_box_2) <= -1e-12
 R533: raw_overlapping(pick_white_box_1,place_blue_box_3)
   - min_t_end(pick_white_box_1,_place_blue_box_3)
   + max_t_start(pick_white_box_1,_place_blue_box_3) = 0
 R534: raw_overlapping(pick_white_box_1,place_blue_box_3)
   - 1e+12 check_parallelism(pick_white_box_1,place_blue_box_3) >= -1e+12
 R535: raw_overlapping(pick_white_box_1,place_blue_box_3)
   - 1e+12 check_parallelism(pick_white_box_1,place_blue_box_3) <= -1e-12
 R536: raw_overlapping(pick_white_box_1,place_blue_box_4)
   - min_t_end(pick_white_box_1,_place_blue_box_4)
   + max_t_start(pick_white_box_1,_place_blue_box_4) = 0
 R537: raw_overlapping(pick_white_box_1,place_blue_box_4)
   - 1e+12 check_parallelism(pick_white_box_1,place_blue_box_4) >= -1e+12
 R538: raw_overlapping(pick_white_box_1,place_blue_box_4)
   - 1e+12 check_parallelism(pick_white_box_1,place_blue_box_4) <= -1e-12
 R539: raw_overlapping(pick_white_box_1,place_white_box_1)
   - min_t_end(pick_white_box_1,_place_white_box_1)
   + max_t_start(pick_white_box_1,_place_white_box_1) = 0
 R540: raw_overlapping(pick_white_box_1,place_white_box_1)
   - 1e+12 check_parallelism(pick_white_box_1,place_white_box_1) >= -1e+12
 R541: raw_overlapping(pick_white_box_1,place_white_box_1)
   - 1e+12 check_parallelism(pick_white_box_1,place_white_box_1) <= -1e-12
 R542: raw_overlapping(pick_white_box_1,place_white_box_2)
   - min_t_end(pick_white_box_1,_place_white_box_2)
   + max_t_start(pick_white_box_1,_place_white_box_2) = 0
 R543: raw_overlapping(pick_white_box_1,place_white_box_2)
   - 1e+12 check_parallelism(pick_white_box_1,place_white_box_2) >= -1e+12
 R544: raw_overlapping(pick_white_box_1,place_white_box_2)
   - 1e+12 check_parallelism(pick_white_box_1,place_white_box_2) <= -1e-12
 R545: raw_overlapping(pick_white_box_1,place_white_box_3)
   - min_t_end(pick_white_box_1,_place_white_box_3)
   + max_t_start(pick_white_box_1,_place_white_box_3) = 0
 R546: raw_overlapping(pick_white_box_1,place_white_box_3)
   - 1e+12 check_parallelism(pick_white_box_1,place_white_box_3) >= -1e+12
 R547: raw_overlapping(pick_white_box_1,place_white_box_3)
   - 1e+12 check_parallelism(pick_white_box_1,place_white_box_3) <= -1e-12
 R548: raw_overlapping(pick_white_box_1,place_white_box_4)
   - min_t_end(pick_white_box_1,_place_white_box_4)
   + max_t_start(pick_white_box_1,_place_white_box_4) = 0
 R549: raw_overlapping(pick_white_box_1,place_white_box_4)
   - 1e+12 check_parallelism(pick_white_box_1,place_white_box_4) >= -1e+12
 R550: raw_overlapping(pick_white_box_1,place_white_box_4)
   - 1e+12 check_parallelism(pick_white_box_1,place_white_box_4) <= -1e-12
 R551: raw_overlapping(pick_white_box_2,pick_white_box_3)
   - min_t_end(pick_white_box_2,_pick_white_box_3)
   + max_t_start(pick_white_box_2,_pick_white_box_3) = 0
 R552: raw_overlapping(pick_white_box_2,pick_white_box_3)
   - 1e+12 check_parallelism(pick_white_box_2,pick_white_box_3) >= -1e+12
 R553: raw_overlapping(pick_white_box_2,pick_white_box_3)
   - 1e+12 check_parallelism(pick_white_box_2,pick_white_box_3) <= -1e-12
 R554: raw_overlapping(pick_white_box_2,pick_white_box_4)
   - min_t_end(pick_white_box_2,_pick_white_box_4)
   + max_t_start(pick_white_box_2,_pick_white_box_4) = 0
 R555: raw_overlapping(pick_white_box_2,pick_white_box_4)
   - 1e+12 check_parallelism(pick_white_box_2,pick_white_box_4) >= -1e+12
 R556: raw_overlapping(pick_white_box_2,pick_white_box_4)
   - 1e+12 check_parallelism(pick_white_box_2,pick_white_box_4) <= -1e-12
 R557: raw_overlapping(pick_white_box_2,place_orange_box_1)
   - min_t_end(pick_white_box_2,_place_orange_box_1)
   + max_t_start(pick_white_box_2,_place_orange_box_1) = 0
 R558: raw_overlapping(pick_white_box_2,place_orange_box_1)
   - 1e+12 check_parallelism(pick_white_box_2,place_orange_box_1)
   >= -1e+12
 R559: raw_overlapping(pick_white_box_2,place_orange_box_1)
   - 1e+12 check_parallelism(pick_white_box_2,place_orange_box_1)
   <= -1e-12
 R560: raw_overlapping(pick_white_box_2,place_orange_box_2)
   - min_t_end(pick_white_box_2,_place_orange_box_2)
   + max_t_start(pick_white_box_2,_place_orange_box_2) = 0
 R561: raw_overlapping(pick_white_box_2,place_orange_box_2)
   - 1e+12 check_parallelism(pick_white_box_2,place_orange_box_2)
   >= -1e+12
 R562: raw_overlapping(pick_white_box_2,place_orange_box_2)
   - 1e+12 check_parallelism(pick_white_box_2,place_orange_box_2)
   <= -1e-12
 R563: raw_overlapping(pick_white_box_2,place_orange_box_3)
   - min_t_end(pick_white_box_2,_place_orange_box_3)
   + max_t_start(pick_white_box_2,_place_orange_box_3) = 0
 R564: raw_overlapping(pick_white_box_2,place_orange_box_3)
   - 1e+12 check_parallelism(pick_white_box_2,place_orange_box_3)
   >= -1e+12
 R565: raw_overlapping(pick_white_box_2,place_orange_box_3)
   - 1e+12 check_parallelism(pick_white_box_2,place_orange_box_3)
   <= -1e-12
 R566: raw_overlapping(pick_white_box_2,place_orange_box_4)
   - min_t_end(pick_white_box_2,_place_orange_box_4)
   + max_t_start(pick_white_box_2,_place_orange_box_4) = 0
 R567: raw_overlapping(pick_white_box_2,place_orange_box_4)
   - 1e+12 check_parallelism(pick_white_box_2,place_orange_box_4)
   >= -1e+12
 R568: raw_overlapping(pick_white_box_2,place_orange_box_4)
   - 1e+12 check_parallelism(pick_white_box_2,place_orange_box_4)
   <= -1e-12
 R569: raw_overlapping(pick_white_box_2,place_blue_box_1)
   - min_t_end(pick_white_box_2,_place_blue_box_1)
   + max_t_start(pick_white_box_2,_place_blue_box_1) = 0
 R570: raw_overlapping(pick_white_box_2,place_blue_box_1)
   - 1e+12 check_parallelism(pick_white_box_2,place_blue_box_1) >= -1e+12
 R571: raw_overlapping(pick_white_box_2,place_blue_box_1)
   - 1e+12 check_parallelism(pick_white_box_2,place_blue_box_1) <= -1e-12
 R572: raw_overlapping(pick_white_box_2,place_blue_box_2)
   - min_t_end(pick_white_box_2,_place_blue_box_2)
   + max_t_start(pick_white_box_2,_place_blue_box_2) = 0
 R573: raw_overlapping(pick_white_box_2,place_blue_box_2)
   - 1e+12 check_parallelism(pick_white_box_2,place_blue_box_2) >= -1e+12
 R574: raw_overlapping(pick_white_box_2,place_blue_box_2)
   - 1e+12 check_parallelism(pick_white_box_2,place_blue_box_2) <= -1e-12
 R575: raw_overlapping(pick_white_box_2,place_blue_box_3)
   - min_t_end(pick_white_box_2,_place_blue_box_3)
   + max_t_start(pick_white_box_2,_place_blue_box_3) = 0
 R576: raw_overlapping(pick_white_box_2,place_blue_box_3)
   - 1e+12 check_parallelism(pick_white_box_2,place_blue_box_3) >= -1e+12
 R577: raw_overlapping(pick_white_box_2,place_blue_box_3)
   - 1e+12 check_parallelism(pick_white_box_2,place_blue_box_3) <= -1e-12
 R578: raw_overlapping(pick_white_box_2,place_blue_box_4)
   - min_t_end(pick_white_box_2,_place_blue_box_4)
   + max_t_start(pick_white_box_2,_place_blue_box_4) = 0
 R579: raw_overlapping(pick_white_box_2,place_blue_box_4)
   - 1e+12 check_parallelism(pick_white_box_2,place_blue_box_4) >= -1e+12
 R580: raw_overlapping(pick_white_box_2,place_blue_box_4)
   - 1e+12 check_parallelism(pick_white_box_2,place_blue_box_4) <= -1e-12
 R581: raw_overlapping(pick_white_box_2,place_white_box_1)
   - min_t_end(pick_white_box_2,_place_white_box_1)
   + max_t_start(pick_white_box_2,_place_white_box_1) = 0
 R582: raw_overlapping(pick_white_box_2,place_white_box_1)
   - 1e+12 check_parallelism(pick_white_box_2,place_white_box_1) >= -1e+12
 R583: raw_overlapping(pick_white_box_2,place_white_box_1)
   - 1e+12 check_parallelism(pick_white_box_2,place_white_box_1) <= -1e-12
 R584: raw_overlapping(pick_white_box_2,place_white_box_2)
   - min_t_end(pick_white_box_2,_place_white_box_2)
   + max_t_start(pick_white_box_2,_place_white_box_2) = 0
 R585: raw_overlapping(pick_white_box_2,place_white_box_2)
   - 1e+12 check_parallelism(pick_white_box_2,place_white_box_2) >= -1e+12
 R586: raw_overlapping(pick_white_box_2,place_white_box_2)
   - 1e+12 check_parallelism(pick_white_box_2,place_white_box_2) <= -1e-12
 R587: raw_overlapping(pick_white_box_2,place_white_box_3)
   - min_t_end(pick_white_box_2,_place_white_box_3)
   + max_t_start(pick_white_box_2,_place_white_box_3) = 0
 R588: raw_overlapping(pick_white_box_2,place_white_box_3)
   - 1e+12 check_parallelism(pick_white_box_2,place_white_box_3) >= -1e+12
 R589: raw_overlapping(pick_white_box_2,place_white_box_3)
   - 1e+12 check_parallelism(pick_white_box_2,place_white_box_3) <= -1e-12
 R590: raw_overlapping(pick_white_box_2,place_white_box_4)
   - min_t_end(pick_white_box_2,_place_white_box_4)
   + max_t_start(pick_white_box_2,_place_white_box_4) = 0
 R591: raw_overlapping(pick_white_box_2,place_white_box_4)
   - 1e+12 check_parallelism(pick_white_box_2,place_white_box_4) >= -1e+12
 R592: raw_overlapping(pick_white_box_2,place_white_box_4)
   - 1e+12 check_parallelism(pick_white_box_2,place_white_box_4) <= -1e-12
 R593: raw_overlapping(pick_white_box_3,pick_white_box_4)
   - min_t_end(pick_white_box_3,_pick_white_box_4)
   + max_t_start(pick_white_box_3,_pick_white_box_4) = 0
 R594: raw_overlapping(pick_white_box_3,pick_white_box_4)
   - 1e+12 check_parallelism(pick_white_box_3,pick_white_box_4) >= -1e+12
 R595: raw_overlapping(pick_white_box_3,pick_white_box_4)
   - 1e+12 check_parallelism(pick_white_box_3,pick_white_box_4) <= -1e-12
 R596: raw_overlapping(pick_white_box_3,place_orange_box_1)
   - min_t_end(pick_white_box_3,_place_orange_box_1)
   + max_t_start(pick_white_box_3,_place_orange_box_1) = 0
 R597: raw_overlapping(pick_white_box_3,place_orange_box_1)
   - 1e+12 check_parallelism(pick_white_box_3,place_orange_box_1)
   >= -1e+12
 R598: raw_overlapping(pick_white_box_3,place_orange_box_1)
   - 1e+12 check_parallelism(pick_white_box_3,place_orange_box_1)
   <= -1e-12
 R599: raw_overlapping(pick_white_box_3,place_orange_box_2)
   - min_t_end(pick_white_box_3,_place_orange_box_2)
   + max_t_start(pick_white_box_3,_place_orange_box_2) = 0
 R600: raw_overlapping(pick_white_box_3,place_orange_box_2)
   - 1e+12 check_parallelism(pick_white_box_3,place_orange_box_2)
   >= -1e+12
 R601: raw_overlapping(pick_white_box_3,place_orange_box_2)
   - 1e+12 check_parallelism(pick_white_box_3,place_orange_box_2)
   <= -1e-12
 R602: raw_overlapping(pick_white_box_3,place_orange_box_3)
   - min_t_end(pick_white_box_3,_place_orange_box_3)
   + max_t_start(pick_white_box_3,_place_orange_box_3) = 0
 R603: raw_overlapping(pick_white_box_3,place_orange_box_3)
   - 1e+12 check_parallelism(pick_white_box_3,place_orange_box_3)
   >= -1e+12
 R604: raw_overlapping(pick_white_box_3,place_orange_box_3)
   - 1e+12 check_parallelism(pick_white_box_3,place_orange_box_3)
   <= -1e-12
 R605: raw_overlapping(pick_white_box_3,place_orange_box_4)
   - min_t_end(pick_white_box_3,_place_orange_box_4)
   + max_t_start(pick_white_box_3,_place_orange_box_4) = 0
 R606: raw_overlapping(pick_white_box_3,place_orange_box_4)
   - 1e+12 check_parallelism(pick_white_box_3,place_orange_box_4)
   >= -1e+12
 R607: raw_overlapping(pick_white_box_3,place_orange_box_4)
   - 1e+12 check_parallelism(pick_white_box_3,place_orange_box_4)
   <= -1e-12
 R608: raw_overlapping(pick_white_box_3,place_blue_box_1)
   - min_t_end(pick_white_box_3,_place_blue_box_1)
   + max_t_start(pick_white_box_3,_place_blue_box_1) = 0
 R609: raw_overlapping(pick_white_box_3,place_blue_box_1)
   - 1e+12 check_parallelism(pick_white_box_3,place_blue_box_1) >= -1e+12
 R610: raw_overlapping(pick_white_box_3,place_blue_box_1)
   - 1e+12 check_parallelism(pick_white_box_3,place_blue_box_1) <= -1e-12
 R611: raw_overlapping(pick_white_box_3,place_blue_box_2)
   - min_t_end(pick_white_box_3,_place_blue_box_2)
   + max_t_start(pick_white_box_3,_place_blue_box_2) = 0
 R612: raw_overlapping(pick_white_box_3,place_blue_box_2)
   - 1e+12 check_parallelism(pick_white_box_3,place_blue_box_2) >= -1e+12
 R613: raw_overlapping(pick_white_box_3,place_blue_box_2)
   - 1e+12 check_parallelism(pick_white_box_3,place_blue_box_2) <= -1e-12
 R614: raw_overlapping(pick_white_box_3,place_blue_box_3)
   - min_t_end(pick_white_box_3,_place_blue_box_3)
   + max_t_start(pick_white_box_3,_place_blue_box_3) = 0
 R615: raw_overlapping(pick_white_box_3,place_blue_box_3)
   - 1e+12 check_parallelism(pick_white_box_3,place_blue_box_3) >= -1e+12
 R616: raw_overlapping(pick_white_box_3,place_blue_box_3)
   - 1e+12 check_parallelism(pick_white_box_3,place_blue_box_3) <= -1e-12
 R617: raw_overlapping(pick_white_box_3,place_blue_box_4)
   - min_t_end(pick_white_box_3,_place_blue_box_4)
   + max_t_start(pick_white_box_3,_place_blue_box_4) = 0
 R618: raw_overlapping(pick_white_box_3,place_blue_box_4)
   - 1e+12 check_parallelism(pick_white_box_3,place_blue_box_4) >= -1e+12
 R619: raw_overlapping(pick_white_box_3,place_blue_box_4)
   - 1e+12 check_parallelism(pick_white_box_3,place_blue_box_4) <= -1e-12
 R620: raw_overlapping(pick_white_box_3,place_white_box_1)
   - min_t_end(pick_white_box_3,_place_white_box_1)
   + max_t_start(pick_white_box_3,_place_white_box_1) = 0
 R621: raw_overlapping(pick_white_box_3,place_white_box_1)
   - 1e+12 check_parallelism(pick_white_box_3,place_white_box_1) >= -1e+12
 R622: raw_overlapping(pick_white_box_3,place_white_box_1)
   - 1e+12 check_parallelism(pick_white_box_3,place_white_box_1) <= -1e-12
 R623: raw_overlapping(pick_white_box_3,place_white_box_2)
   - min_t_end(pick_white_box_3,_place_white_box_2)
   + max_t_start(pick_white_box_3,_place_white_box_2) = 0
 R624: raw_overlapping(pick_white_box_3,place_white_box_2)
   - 1e+12 check_parallelism(pick_white_box_3,place_white_box_2) >= -1e+12
 R625: raw_overlapping(pick_white_box_3,place_white_box_2)
   - 1e+12 check_parallelism(pick_white_box_3,place_white_box_2) <= -1e-12
 R626: raw_overlapping(pick_white_box_3,place_white_box_3)
   - min_t_end(pick_white_box_3,_place_white_box_3)
   + max_t_start(pick_white_box_3,_place_white_box_3) = 0
 R627: raw_overlapping(pick_white_box_3,place_white_box_3)
   - 1e+12 check_parallelism(pick_white_box_3,place_white_box_3) >= -1e+12
 R628: raw_overlapping(pick_white_box_3,place_white_box_3)
   - 1e+12 check_parallelism(pick_white_box_3,place_white_box_3) <= -1e-12
 R629: raw_overlapping(pick_white_box_3,place_white_box_4)
   - min_t_end(pick_white_box_3,_place_white_box_4)
   + max_t_start(pick_white_box_3,_place_white_box_4) = 0
 R630: raw_overlapping(pick_white_box_3,place_white_box_4)
   - 1e+12 check_parallelism(pick_white_box_3,place_white_box_4) >= -1e+12
 R631: raw_overlapping(pick_white_box_3,place_white_box_4)
   - 1e+12 check_parallelism(pick_white_box_3,place_white_box_4) <= -1e-12
 R632: raw_overlapping(pick_white_box_4,place_orange_box_1)
   - min_t_end(pick_white_box_4,_place_orange_box_1)
   + max_t_start(pick_white_box_4,_place_orange_box_1) = 0
 R633: raw_overlapping(pick_white_box_4,place_orange_box_1)
   - 1e+12 check_parallelism(pick_white_box_4,place_orange_box_1)
   >= -1e+12
 R634: raw_overlapping(pick_white_box_4,place_orange_box_1)
   - 1e+12 check_parallelism(pick_white_box_4,place_orange_box_1)
   <= -1e-12
 R635: raw_overlapping(pick_white_box_4,place_orange_box_2)
   - min_t_end(pick_white_box_4,_place_orange_box_2)
   + max_t_start(pick_white_box_4,_place_orange_box_2) = 0
 R636: raw_overlapping(pick_white_box_4,place_orange_box_2)
   - 1e+12 check_parallelism(pick_white_box_4,place_orange_box_2)
   >= -1e+12
 R637: raw_overlapping(pick_white_box_4,place_orange_box_2)
   - 1e+12 check_parallelism(pick_white_box_4,place_orange_box_2)
   <= -1e-12
 R638: raw_overlapping(pick_white_box_4,place_orange_box_3)
   - min_t_end(pick_white_box_4,_place_orange_box_3)
   + max_t_start(pick_white_box_4,_place_orange_box_3) = 0
 R639: raw_overlapping(pick_white_box_4,place_orange_box_3)
   - 1e+12 check_parallelism(pick_white_box_4,place_orange_box_3)
   >= -1e+12
 R640: raw_overlapping(pick_white_box_4,place_orange_box_3)
   - 1e+12 check_parallelism(pick_white_box_4,place_orange_box_3)
   <= -1e-12
 R641: raw_overlapping(pick_white_box_4,place_orange_box_4)
   - min_t_end(pick_white_box_4,_place_orange_box_4)
   + max_t_start(pick_white_box_4,_place_orange_box_4) = 0
 R642: raw_overlapping(pick_white_box_4,place_orange_box_4)
   - 1e+12 check_parallelism(pick_white_box_4,place_orange_box_4)
   >= -1e+12
 R643: raw_overlapping(pick_white_box_4,place_orange_box_4)
   - 1e+12 check_parallelism(pick_white_box_4,place_orange_box_4)
   <= -1e-12
 R644: raw_overlapping(pick_white_box_4,place_blue_box_1)
   - min_t_end(pick_white_box_4,_place_blue_box_1)
   + max_t_start(pick_white_box_4,_place_blue_box_1) = 0
 R645: raw_overlapping(pick_white_box_4,place_blue_box_1)
   - 1e+12 check_parallelism(pick_white_box_4,place_blue_box_1) >= -1e+12
 R646: raw_overlapping(pick_white_box_4,place_blue_box_1)
   - 1e+12 check_parallelism(pick_white_box_4,place_blue_box_1) <= -1e-12
 R647: raw_overlapping(pick_white_box_4,place_blue_box_2)
   - min_t_end(pick_white_box_4,_place_blue_box_2)
   + max_t_start(pick_white_box_4,_place_blue_box_2) = 0
 R648: raw_overlapping(pick_white_box_4,place_blue_box_2)
   - 1e+12 check_parallelism(pick_white_box_4,place_blue_box_2) >= -1e+12
 R649: raw_overlapping(pick_white_box_4,place_blue_box_2)
   - 1e+12 check_parallelism(pick_white_box_4,place_blue_box_2) <= -1e-12
 R650: raw_overlapping(pick_white_box_4,place_blue_box_3)
   - min_t_end(pick_white_box_4,_place_blue_box_3)
   + max_t_start(pick_white_box_4,_place_blue_box_3) = 0
 R651: raw_overlapping(pick_white_box_4,place_blue_box_3)
   - 1e+12 check_parallelism(pick_white_box_4,place_blue_box_3) >= -1e+12
 R652: raw_overlapping(pick_white_box_4,place_blue_box_3)
   - 1e+12 check_parallelism(pick_white_box_4,place_blue_box_3) <= -1e-12
 R653: raw_overlapping(pick_white_box_4,place_blue_box_4)
   - min_t_end(pick_white_box_4,_place_blue_box_4)
   + max_t_start(pick_white_box_4,_place_blue_box_4) = 0
 R654: raw_overlapping(pick_white_box_4,place_blue_box_4)
   - 1e+12 check_parallelism(pick_white_box_4,place_blue_box_4) >= -1e+12
 R655: raw_overlapping(pick_white_box_4,place_blue_box_4)
   - 1e+12 check_parallelism(pick_white_box_4,place_blue_box_4) <= -1e-12
 R656: raw_overlapping(pick_white_box_4,place_white_box_1)
   - min_t_end(pick_white_box_4,_place_white_box_1)
   + max_t_start(pick_white_box_4,_place_white_box_1) = 0
 R657: raw_overlapping(pick_white_box_4,place_white_box_1)
   - 1e+12 check_parallelism(pick_white_box_4,place_white_box_1) >= -1e+12
 R658: raw_overlapping(pick_white_box_4,place_white_box_1)
   - 1e+12 check_parallelism(pick_white_box_4,place_white_box_1) <= -1e-12
 R659: raw_overlapping(pick_white_box_4,place_white_box_2)
   - min_t_end(pick_white_box_4,_place_white_box_2)
   + max_t_start(pick_white_box_4,_place_white_box_2) = 0
 R660: raw_overlapping(pick_white_box_4,place_white_box_2)
   - 1e+12 check_parallelism(pick_white_box_4,place_white_box_2) >= -1e+12
 R661: raw_overlapping(pick_white_box_4,place_white_box_2)
   - 1e+12 check_parallelism(pick_white_box_4,place_white_box_2) <= -1e-12
 R662: raw_overlapping(pick_white_box_4,place_white_box_3)
   - min_t_end(pick_white_box_4,_place_white_box_3)
   + max_t_start(pick_white_box_4,_place_white_box_3) = 0
 R663: raw_overlapping(pick_white_box_4,place_white_box_3)
   - 1e+12 check_parallelism(pick_white_box_4,place_white_box_3) >= -1e+12
 R664: raw_overlapping(pick_white_box_4,place_white_box_3)
   - 1e+12 check_parallelism(pick_white_box_4,place_white_box_3) <= -1e-12
 R665: raw_overlapping(pick_white_box_4,place_white_box_4)
   - min_t_end(pick_white_box_4,_place_white_box_4)
   + max_t_start(pick_white_box_4,_place_white_box_4) = 0
 R666: raw_overlapping(pick_white_box_4,place_white_box_4)
   - 1e+12 check_parallelism(pick_white_box_4,place_white_box_4) >= -1e+12
 R667: raw_overlapping(pick_white_box_4,place_white_box_4)
   - 1e+12 check_parallelism(pick_white_box_4,place_white_box_4) <= -1e-12
 R668: raw_overlapping(place_orange_box_1,place_orange_box_2)
   - min_t_end(place_orange_box_1,_place_orange_box_2)
   + max_t_start(place_orange_box_1,_place_orange_box_2) = 0
 R669: raw_overlapping(place_orange_box_1,place_orange_box_2)
   - 1e+12 check_parallelism(place_orange_box_1,place_orange_box_2)
   >= -1e+12
 R670: raw_overlapping(place_orange_box_1,place_orange_box_2)
   - 1e+12 check_parallelism(place_orange_box_1,place_orange_box_2)
   <= -1e-12
 R671: raw_overlapping(place_orange_box_1,place_orange_box_3)
   - min_t_end(place_orange_box_1,_place_orange_box_3)
   + max_t_start(place_orange_box_1,_place_orange_box_3) = 0
 R672: raw_overlapping(place_orange_box_1,place_orange_box_3)
   - 1e+12 check_parallelism(place_orange_box_1,place_orange_box_3)
   >= -1e+12
 R673: raw_overlapping(place_orange_box_1,place_orange_box_3)
   - 1e+12 check_parallelism(place_orange_box_1,place_orange_box_3)
   <= -1e-12
 R674: raw_overlapping(place_orange_box_1,place_orange_box_4)
   - min_t_end(place_orange_box_1,_place_orange_box_4)
   + max_t_start(place_orange_box_1,_place_orange_box_4) = 0
 R675: raw_overlapping(place_orange_box_1,place_orange_box_4)
   - 1e+12 check_parallelism(place_orange_box_1,place_orange_box_4)
   >= -1e+12
 R676: raw_overlapping(place_orange_box_1,place_orange_box_4)
   - 1e+12 check_parallelism(place_orange_box_1,place_orange_box_4)
   <= -1e-12
 R677: raw_overlapping(place_orange_box_1,place_blue_box_1)
   - min_t_end(place_orange_box_1,_place_blue_box_1)
   + max_t_start(place_orange_box_1,_place_blue_box_1) = 0
 R678: raw_overlapping(place_orange_box_1,place_blue_box_1)
   - 1e+12 check_parallelism(place_orange_box_1,place_blue_box_1)
   >= -1e+12
 R679: raw_overlapping(place_orange_box_1,place_blue_box_1)
   - 1e+12 check_parallelism(place_orange_box_1,place_blue_box_1)
   <= -1e-12
 R680: raw_overlapping(place_orange_box_1,place_blue_box_2)
   - min_t_end(place_orange_box_1,_place_blue_box_2)
   + max_t_start(place_orange_box_1,_place_blue_box_2) = 0
 R681: raw_overlapping(place_orange_box_1,place_blue_box_2)
   - 1e+12 check_parallelism(place_orange_box_1,place_blue_box_2)
   >= -1e+12
 R682: raw_overlapping(place_orange_box_1,place_blue_box_2)
   - 1e+12 check_parallelism(place_orange_box_1,place_blue_box_2)
   <= -1e-12
 R683: raw_overlapping(place_orange_box_1,place_blue_box_3)
   - min_t_end(place_orange_box_1,_place_blue_box_3)
   + max_t_start(place_orange_box_1,_place_blue_box_3) = 0
 R684: raw_overlapping(place_orange_box_1,place_blue_box_3)
   - 1e+12 check_parallelism(place_orange_box_1,place_blue_box_3)
   >= -1e+12
 R685: raw_overlapping(place_orange_box_1,place_blue_box_3)
   - 1e+12 check_parallelism(place_orange_box_1,place_blue_box_3)
   <= -1e-12
 R686: raw_overlapping(place_orange_box_1,place_blue_box_4)
   - min_t_end(place_orange_box_1,_place_blue_box_4)
   + max_t_start(place_orange_box_1,_place_blue_box_4) = 0
 R687: raw_overlapping(place_orange_box_1,place_blue_box_4)
   - 1e+12 check_parallelism(place_orange_box_1,place_blue_box_4)
   >= -1e+12
 R688: raw_overlapping(place_orange_box_1,place_blue_box_4)
   - 1e+12 check_parallelism(place_orange_box_1,place_blue_box_4)
   <= -1e-12
 R689: raw_overlapping(place_orange_box_1,place_white_box_1)
   - min_t_end(place_orange_box_1,_place_white_box_1)
   + max_t_start(place_orange_box_1,_place_white_box_1) = 0
 R690: raw_overlapping(place_orange_box_1,place_white_box_1)
   - 1e+12 check_parallelism(place_orange_box_1,place_white_box_1)
   >= -1e+12
 R691: raw_overlapping(place_orange_box_1,place_white_box_1)
   - 1e+12 check_parallelism(place_orange_box_1,place_white_box_1)
   <= -1e-12
 R692: raw_overlapping(place_orange_box_1,place_white_box_2)
   - min_t_end(place_orange_box_1,_place_white_box_2)
   + max_t_start(place_orange_box_1,_place_white_box_2) = 0
 R693: raw_overlapping(place_orange_box_1,place_white_box_2)
   - 1e+12 check_parallelism(place_orange_box_1,place_white_box_2)
   >= -1e+12
 R694: raw_overlapping(place_orange_box_1,place_white_box_2)
   - 1e+12 check_parallelism(place_orange_box_1,place_white_box_2)
   <= -1e-12
 R695: raw_overlapping(place_orange_box_1,place_white_box_3)
   - min_t_end(place_orange_box_1,_place_white_box_3)
   + max_t_start(place_orange_box_1,_place_white_box_3) = 0
 R696: raw_overlapping(place_orange_box_1,place_white_box_3)
   - 1e+12 check_parallelism(place_orange_box_1,place_white_box_3)
   >= -1e+12
 R697: raw_overlapping(place_orange_box_1,place_white_box_3)
   - 1e+12 check_parallelism(place_orange_box_1,place_white_box_3)
   <= -1e-12
 R698: raw_overlapping(place_orange_box_1,place_white_box_4)
   - min_t_end(place_orange_box_1,_place_white_box_4)
   + max_t_start(place_orange_box_1,_place_white_box_4) = 0
 R699: raw_overlapping(place_orange_box_1,place_white_box_4)
   - 1e+12 check_parallelism(place_orange_box_1,place_white_box_4)
   >= -1e+12
 R700: raw_overlapping(place_orange_box_1,place_white_box_4)
   - 1e+12 check_parallelism(place_orange_box_1,place_white_box_4)
   <= -1e-12
 R701: raw_overlapping(place_orange_box_2,place_orange_box_3)
   - min_t_end(place_orange_box_2,_place_orange_box_3)
   + max_t_start(place_orange_box_2,_place_orange_box_3) = 0
 R702: raw_overlapping(place_orange_box_2,place_orange_box_3)
   - 1e+12 check_parallelism(place_orange_box_2,place_orange_box_3)
   >= -1e+12
 R703: raw_overlapping(place_orange_box_2,place_orange_box_3)
   - 1e+12 check_parallelism(place_orange_box_2,place_orange_box_3)
   <= -1e-12
 R704: raw_overlapping(place_orange_box_2,place_orange_box_4)
   - min_t_end(place_orange_box_2,_place_orange_box_4)
   + max_t_start(place_orange_box_2,_place_orange_box_4) = 0
 R705: raw_overlapping(place_orange_box_2,place_orange_box_4)
   - 1e+12 check_parallelism(place_orange_box_2,place_orange_box_4)
   >= -1e+12
 R706: raw_overlapping(place_orange_box_2,place_orange_box_4)
   - 1e+12 check_parallelism(place_orange_box_2,place_orange_box_4)
   <= -1e-12
 R707: raw_overlapping(place_orange_box_2,place_blue_box_1)
   - min_t_end(place_orange_box_2,_place_blue_box_1)
   + max_t_start(place_orange_box_2,_place_blue_box_1) = 0
 R708: raw_overlapping(place_orange_box_2,place_blue_box_1)
   - 1e+12 check_parallelism(place_orange_box_2,place_blue_box_1)
   >= -1e+12
 R709: raw_overlapping(place_orange_box_2,place_blue_box_1)
   - 1e+12 check_parallelism(place_orange_box_2,place_blue_box_1)
   <= -1e-12
 R710: raw_overlapping(place_orange_box_2,place_blue_box_2)
   - min_t_end(place_orange_box_2,_place_blue_box_2)
   + max_t_start(place_orange_box_2,_place_blue_box_2) = 0
 R711: raw_overlapping(place_orange_box_2,place_blue_box_2)
   - 1e+12 check_parallelism(place_orange_box_2,place_blue_box_2)
   >= -1e+12
 R712: raw_overlapping(place_orange_box_2,place_blue_box_2)
   - 1e+12 check_parallelism(place_orange_box_2,place_blue_box_2)
   <= -1e-12
 R713: raw_overlapping(place_orange_box_2,place_blue_box_3)
   - min_t_end(place_orange_box_2,_place_blue_box_3)
   + max_t_start(place_orange_box_2,_place_blue_box_3) = 0
 R714: raw_overlapping(place_orange_box_2,place_blue_box_3)
   - 1e+12 check_parallelism(place_orange_box_2,place_blue_box_3)
   >= -1e+12
 R715: raw_overlapping(place_orange_box_2,place_blue_box_3)
   - 1e+12 check_parallelism(place_orange_box_2,place_blue_box_3)
   <= -1e-12
 R716: raw_overlapping(place_orange_box_2,place_blue_box_4)
   - min_t_end(place_orange_box_2,_place_blue_box_4)
   + max_t_start(place_orange_box_2,_place_blue_box_4) = 0
 R717: raw_overlapping(place_orange_box_2,place_blue_box_4)
   - 1e+12 check_parallelism(place_orange_box_2,place_blue_box_4)
   >= -1e+12
 R718: raw_overlapping(place_orange_box_2,place_blue_box_4)
   - 1e+12 check_parallelism(place_orange_box_2,place_blue_box_4)
   <= -1e-12
 R719: raw_overlapping(place_orange_box_2,place_white_box_1)
   - min_t_end(place_orange_box_2,_place_white_box_1)
   + max_t_start(place_orange_box_2,_place_white_box_1) = 0
 R720: raw_overlapping(place_orange_box_2,place_white_box_1)
   - 1e+12 check_parallelism(place_orange_box_2,place_white_box_1)
   >= -1e+12
 R721: raw_overlapping(place_orange_box_2,place_white_box_1)
   - 1e+12 check_parallelism(place_orange_box_2,place_white_box_1)
   <= -1e-12
 R722: raw_overlapping(place_orange_box_2,place_white_box_2)
   - min_t_end(place_orange_box_2,_place_white_box_2)
   + max_t_start(place_orange_box_2,_place_white_box_2) = 0
 R723: raw_overlapping(place_orange_box_2,place_white_box_2)
   - 1e+12 check_parallelism(place_orange_box_2,place_white_box_2)
   >= -1e+12
 R724: raw_overlapping(place_orange_box_2,place_white_box_2)
   - 1e+12 check_parallelism(place_orange_box_2,place_white_box_2)
   <= -1e-12
 R725: raw_overlapping(place_orange_box_2,place_white_box_3)
   - min_t_end(place_orange_box_2,_place_white_box_3)
   + max_t_start(place_orange_box_2,_place_white_box_3) = 0
 R726: raw_overlapping(place_orange_box_2,place_white_box_3)
   - 1e+12 check_parallelism(place_orange_box_2,place_white_box_3)
   >= -1e+12
 R727: raw_overlapping(place_orange_box_2,place_white_box_3)
   - 1e+12 check_parallelism(place_orange_box_2,place_white_box_3)
   <= -1e-12
 R728: raw_overlapping(place_orange_box_2,place_white_box_4)
   - min_t_end(place_orange_box_2,_place_white_box_4)
   + max_t_start(place_orange_box_2,_place_white_box_4) = 0
 R729: raw_overlapping(place_orange_box_2,place_white_box_4)
   - 1e+12 check_parallelism(place_orange_box_2,place_white_box_4)
   >= -1e+12
 R730: raw_overlapping(place_orange_box_2,place_white_box_4)
   - 1e+12 check_parallelism(place_orange_box_2,place_white_box_4)
   <= -1e-12
 R731: raw_overlapping(place_orange_box_3,place_orange_box_4)
   - min_t_end(place_orange_box_3,_place_orange_box_4)
   + max_t_start(place_orange_box_3,_place_orange_box_4) = 0
 R732: raw_overlapping(place_orange_box_3,place_orange_box_4)
   - 1e+12 check_parallelism(place_orange_box_3,place_orange_box_4)
   >= -1e+12
 R733: raw_overlapping(place_orange_box_3,place_orange_box_4)
   - 1e+12 check_parallelism(place_orange_box_3,place_orange_box_4)
   <= -1e-12
 R734: raw_overlapping(place_orange_box_3,place_blue_box_1)
   - min_t_end(place_orange_box_3,_place_blue_box_1)
   + max_t_start(place_orange_box_3,_place_blue_box_1) = 0
 R735: raw_overlapping(place_orange_box_3,place_blue_box_1)
   - 1e+12 check_parallelism(place_orange_box_3,place_blue_box_1)
   >= -1e+12
 R736: raw_overlapping(place_orange_box_3,place_blue_box_1)
   - 1e+12 check_parallelism(place_orange_box_3,place_blue_box_1)
   <= -1e-12
 R737: raw_overlapping(place_orange_box_3,place_blue_box_2)
   - min_t_end(place_orange_box_3,_place_blue_box_2)
   + max_t_start(place_orange_box_3,_place_blue_box_2) = 0
 R738: raw_overlapping(place_orange_box_3,place_blue_box_2)
   - 1e+12 check_parallelism(place_orange_box_3,place_blue_box_2)
   >= -1e+12
 R739: raw_overlapping(place_orange_box_3,place_blue_box_2)
   - 1e+12 check_parallelism(place_orange_box_3,place_blue_box_2)
   <= -1e-12
 R740: raw_overlapping(place_orange_box_3,place_blue_box_3)
   - min_t_end(place_orange_box_3,_place_blue_box_3)
   + max_t_start(place_orange_box_3,_place_blue_box_3) = 0
 R741: raw_overlapping(place_orange_box_3,place_blue_box_3)
   - 1e+12 check_parallelism(place_orange_box_3,place_blue_box_3)
   >= -1e+12
 R742: raw_overlapping(place_orange_box_3,place_blue_box_3)
   - 1e+12 check_parallelism(place_orange_box_3,place_blue_box_3)
   <= -1e-12
 R743: raw_overlapping(place_orange_box_3,place_blue_box_4)
   - min_t_end(place_orange_box_3,_place_blue_box_4)
   + max_t_start(place_orange_box_3,_place_blue_box_4) = 0
 R744: raw_overlapping(place_orange_box_3,place_blue_box_4)
   - 1e+12 check_parallelism(place_orange_box_3,place_blue_box_4)
   >= -1e+12
 R745: raw_overlapping(place_orange_box_3,place_blue_box_4)
   - 1e+12 check_parallelism(place_orange_box_3,place_blue_box_4)
   <= -1e-12
 R746: raw_overlapping(place_orange_box_3,place_white_box_1)
   - min_t_end(place_orange_box_3,_place_white_box_1)
   + max_t_start(place_orange_box_3,_place_white_box_1) = 0
 R747: raw_overlapping(place_orange_box_3,place_white_box_1)
   - 1e+12 check_parallelism(place_orange_box_3,place_white_box_1)
   >= -1e+12
 R748: raw_overlapping(place_orange_box_3,place_white_box_1)
   - 1e+12 check_parallelism(place_orange_box_3,place_white_box_1)
   <= -1e-12
 R749: raw_overlapping(place_orange_box_3,place_white_box_2)
   - min_t_end(place_orange_box_3,_place_white_box_2)
   + max_t_start(place_orange_box_3,_place_white_box_2) = 0
 R750: raw_overlapping(place_orange_box_3,place_white_box_2)
   - 1e+12 check_parallelism(place_orange_box_3,place_white_box_2)
   >= -1e+12
 R751: raw_overlapping(place_orange_box_3,place_white_box_2)
   - 1e+12 check_parallelism(place_orange_box_3,place_white_box_2)
   <= -1e-12
 R752: raw_overlapping(place_orange_box_3,place_white_box_3)
   - min_t_end(place_orange_box_3,_place_white_box_3)
   + max_t_start(place_orange_box_3,_place_white_box_3) = 0
 R753: raw_overlapping(place_orange_box_3,place_white_box_3)
   - 1e+12 check_parallelism(place_orange_box_3,place_white_box_3)
   >= -1e+12
 R754: raw_overlapping(place_orange_box_3,place_white_box_3)
   - 1e+12 check_parallelism(place_orange_box_3,place_white_box_3)
   <= -1e-12
 R755: raw_overlapping(place_orange_box_3,place_white_box_4)
   - min_t_end(place_orange_box_3,_place_white_box_4)
   + max_t_start(place_orange_box_3,_place_white_box_4) = 0
 R756: raw_overlapping(place_orange_box_3,place_white_box_4)
   - 1e+12 check_parallelism(place_orange_box_3,place_white_box_4)
   >= -1e+12
 R757: raw_overlapping(place_orange_box_3,place_white_box_4)
   - 1e+12 check_parallelism(place_orange_box_3,place_white_box_4)
   <= -1e-12
 R758: raw_overlapping(place_orange_box_4,place_blue_box_1)
   - min_t_end(place_orange_box_4,_place_blue_box_1)
   + max_t_start(place_orange_box_4,_place_blue_box_1) = 0
 R759: raw_overlapping(place_orange_box_4,place_blue_box_1)
   - 1e+12 check_parallelism(place_orange_box_4,place_blue_box_1)
   >= -1e+12
 R760: raw_overlapping(place_orange_box_4,place_blue_box_1)
   - 1e+12 check_parallelism(place_orange_box_4,place_blue_box_1)
   <= -1e-12
 R761: raw_overlapping(place_orange_box_4,place_blue_box_2)
   - min_t_end(place_orange_box_4,_place_blue_box_2)
   + max_t_start(place_orange_box_4,_place_blue_box_2) = 0
 R762: raw_overlapping(place_orange_box_4,place_blue_box_2)
   - 1e+12 check_parallelism(place_orange_box_4,place_blue_box_2)
   >= -1e+12
 R763: raw_overlapping(place_orange_box_4,place_blue_box_2)
   - 1e+12 check_parallelism(place_orange_box_4,place_blue_box_2)
   <= -1e-12
 R764: raw_overlapping(place_orange_box_4,place_blue_box_3)
   - min_t_end(place_orange_box_4,_place_blue_box_3)
   + max_t_start(place_orange_box_4,_place_blue_box_3) = 0
 R765: raw_overlapping(place_orange_box_4,place_blue_box_3)
   - 1e+12 check_parallelism(place_orange_box_4,place_blue_box_3)
   >= -1e+12
 R766: raw_overlapping(place_orange_box_4,place_blue_box_3)
   - 1e+12 check_parallelism(place_orange_box_4,place_blue_box_3)
   <= -1e-12
 R767: raw_overlapping(place_orange_box_4,place_blue_box_4)
   - min_t_end(place_orange_box_4,_place_blue_box_4)
   + max_t_start(place_orange_box_4,_place_blue_box_4) = 0
 R768: raw_overlapping(place_orange_box_4,place_blue_box_4)
   - 1e+12 check_parallelism(place_orange_box_4,place_blue_box_4)
   >= -1e+12
 R769: raw_overlapping(place_orange_box_4,place_blue_box_4)
   - 1e+12 check_parallelism(place_orange_box_4,place_blue_box_4)
   <= -1e-12
 R770: raw_overlapping(place_orange_box_4,place_white_box_1)
   - min_t_end(place_orange_box_4,_place_white_box_1)
   + max_t_start(place_orange_box_4,_place_white_box_1) = 0
 R771: raw_overlapping(place_orange_box_4,place_white_box_1)
   - 1e+12 check_parallelism(place_orange_box_4,place_white_box_1)
   >= -1e+12
 R772: raw_overlapping(place_orange_box_4,place_white_box_1)
   - 1e+12 check_parallelism(place_orange_box_4,place_white_box_1)
   <= -1e-12
 R773: raw_overlapping(place_orange_box_4,place_white_box_2)
   - min_t_end(place_orange_box_4,_place_white_box_2)
   + max_t_start(place_orange_box_4,_place_white_box_2) = 0
 R774: raw_overlapping(place_orange_box_4,place_white_box_2)
   - 1e+12 check_parallelism(place_orange_box_4,place_white_box_2)
   >= -1e+12
 R775: raw_overlapping(place_orange_box_4,place_white_box_2)
   - 1e+12 check_parallelism(place_orange_box_4,place_white_box_2)
   <= -1e-12
 R776: raw_overlapping(place_orange_box_4,place_white_box_3)
   - min_t_end(place_orange_box_4,_place_white_box_3)
   + max_t_start(place_orange_box_4,_place_white_box_3) = 0
 R777: raw_overlapping(place_orange_box_4,place_white_box_3)
   - 1e+12 check_parallelism(place_orange_box_4,place_white_box_3)
   >= -1e+12
 R778: raw_overlapping(place_orange_box_4,place_white_box_3)
   - 1e+12 check_parallelism(place_orange_box_4,place_white_box_3)
   <= -1e-12
 R779: raw_overlapping(place_orange_box_4,place_white_box_4)
   - min_t_end(place_orange_box_4,_place_white_box_4)
   + max_t_start(place_orange_box_4,_place_white_box_4) = 0
 R780: raw_overlapping(place_orange_box_4,place_white_box_4)
   - 1e+12 check_parallelism(place_orange_box_4,place_white_box_4)
   >= -1e+12
 R781: raw_overlapping(place_orange_box_4,place_white_box_4)
   - 1e+12 check_parallelism(place_orange_box_4,place_white_box_4)
   <= -1e-12
 R782: raw_overlapping(place_blue_box_1,place_blue_box_2)
   - min_t_end(place_blue_box_1,_place_blue_box_2)
   + max_t_start(place_blue_box_1,_place_blue_box_2) = 0
 R783: raw_overlapping(place_blue_box_1,place_blue_box_2)
   - 1e+12 check_parallelism(place_blue_box_1,place_blue_box_2) >= -1e+12
 R784: raw_overlapping(place_blue_box_1,place_blue_box_2)
   - 1e+12 check_parallelism(place_blue_box_1,place_blue_box_2) <= -1e-12
 R785: raw_overlapping(place_blue_box_1,place_blue_box_3)
   - min_t_end(place_blue_box_1,_place_blue_box_3)
   + max_t_start(place_blue_box_1,_place_blue_box_3) = 0
 R786: raw_overlapping(place_blue_box_1,place_blue_box_3)
   - 1e+12 check_parallelism(place_blue_box_1,place_blue_box_3) >= -1e+12
 R787: raw_overlapping(place_blue_box_1,place_blue_box_3)
   - 1e+12 check_parallelism(place_blue_box_1,place_blue_box_3) <= -1e-12
 R788: raw_overlapping(place_blue_box_1,place_blue_box_4)
   - min_t_end(place_blue_box_1,_place_blue_box_4)
   + max_t_start(place_blue_box_1,_place_blue_box_4) = 0
 R789: raw_overlapping(place_blue_box_1,place_blue_box_4)
   - 1e+12 check_parallelism(place_blue_box_1,place_blue_box_4) >= -1e+12
 R790: raw_overlapping(place_blue_box_1,place_blue_box_4)
   - 1e+12 check_parallelism(place_blue_box_1,place_blue_box_4) <= -1e-12
 R791: raw_overlapping(place_blue_box_1,place_white_box_1)
   - min_t_end(place_blue_box_1,_place_white_box_1)
   + max_t_start(place_blue_box_1,_place_white_box_1) = 0
 R792: raw_overlapping(place_blue_box_1,place_white_box_1)
   - 1e+12 check_parallelism(place_blue_box_1,place_white_box_1) >= -1e+12
 R793: raw_overlapping(place_blue_box_1,place_white_box_1)
   - 1e+12 check_parallelism(place_blue_box_1,place_white_box_1) <= -1e-12
 R794: raw_overlapping(place_blue_box_1,place_white_box_2)
   - min_t_end(place_blue_box_1,_place_white_box_2)
   + max_t_start(place_blue_box_1,_place_white_box_2) = 0
 R795: raw_overlapping(place_blue_box_1,place_white_box_2)
   - 1e+12 check_parallelism(place_blue_box_1,place_white_box_2) >= -1e+12
 R796: raw_overlapping(place_blue_box_1,place_white_box_2)
   - 1e+12 check_parallelism(place_blue_box_1,place_white_box_2) <= -1e-12
 R797: raw_overlapping(place_blue_box_1,place_white_box_3)
   - min_t_end(place_blue_box_1,_place_white_box_3)
   + max_t_start(place_blue_box_1,_place_white_box_3) = 0
 R798: raw_overlapping(place_blue_box_1,place_white_box_3)
   - 1e+12 check_parallelism(place_blue_box_1,place_white_box_3) >= -1e+12
 R799: raw_overlapping(place_blue_box_1,place_white_box_3)
   - 1e+12 check_parallelism(place_blue_box_1,place_white_box_3) <= -1e-12
 R800: raw_overlapping(place_blue_box_1,place_white_box_4)
   - min_t_end(place_blue_box_1,_place_white_box_4)
   + max_t_start(place_blue_box_1,_place_white_box_4) = 0
 R801: raw_overlapping(place_blue_box_1,place_white_box_4)
   - 1e+12 check_parallelism(place_blue_box_1,place_white_box_4) >= -1e+12
 R802: raw_overlapping(place_blue_box_1,place_white_box_4)
   - 1e+12 check_parallelism(place_blue_box_1,place_white_box_4) <= -1e-12
 R803: raw_overlapping(place_blue_box_2,place_blue_box_3)
   - min_t_end(place_blue_box_2,_place_blue_box_3)
   + max_t_start(place_blue_box_2,_place_blue_box_3) = 0
 R804: raw_overlapping(place_blue_box_2,place_blue_box_3)
   - 1e+12 check_parallelism(place_blue_box_2,place_blue_box_3) >= -1e+12
 R805: raw_overlapping(place_blue_box_2,place_blue_box_3)
   - 1e+12 check_parallelism(place_blue_box_2,place_blue_box_3) <= -1e-12
 R806: raw_overlapping(place_blue_box_2,place_blue_box_4)
   - min_t_end(place_blue_box_2,_place_blue_box_4)
   + max_t_start(place_blue_box_2,_place_blue_box_4) = 0
 R807: raw_overlapping(place_blue_box_2,place_blue_box_4)
   - 1e+12 check_parallelism(place_blue_box_2,place_blue_box_4) >= -1e+12
 R808: raw_overlapping(place_blue_box_2,place_blue_box_4)
   - 1e+12 check_parallelism(place_blue_box_2,place_blue_box_4) <= -1e-12
 R809: raw_overlapping(place_blue_box_2,place_white_box_1)
   - min_t_end(place_blue_box_2,_place_white_box_1)
   + max_t_start(place_blue_box_2,_place_white_box_1) = 0
 R810: raw_overlapping(place_blue_box_2,place_white_box_1)
   - 1e+12 check_parallelism(place_blue_box_2,place_white_box_1) >= -1e+12
 R811: raw_overlapping(place_blue_box_2,place_white_box_1)
   - 1e+12 check_parallelism(place_blue_box_2,place_white_box_1) <= -1e-12
 R812: raw_overlapping(place_blue_box_2,place_white_box_2)
   - min_t_end(place_blue_box_2,_place_white_box_2)
   + max_t_start(place_blue_box_2,_place_white_box_2) = 0
 R813: raw_overlapping(place_blue_box_2,place_white_box_2)
   - 1e+12 check_parallelism(place_blue_box_2,place_white_box_2) >= -1e+12
 R814: raw_overlapping(place_blue_box_2,place_white_box_2)
   - 1e+12 check_parallelism(place_blue_box_2,place_white_box_2) <= -1e-12
 R815: raw_overlapping(place_blue_box_2,place_white_box_3)
   - min_t_end(place_blue_box_2,_place_white_box_3)
   + max_t_start(place_blue_box_2,_place_white_box_3) = 0
 R816: raw_overlapping(place_blue_box_2,place_white_box_3)
   - 1e+12 check_parallelism(place_blue_box_2,place_white_box_3) >= -1e+12
 R817: raw_overlapping(place_blue_box_2,place_white_box_3)
   - 1e+12 check_parallelism(place_blue_box_2,place_white_box_3) <= -1e-12
 R818: raw_overlapping(place_blue_box_2,place_white_box_4)
   - min_t_end(place_blue_box_2,_place_white_box_4)
   + max_t_start(place_blue_box_2,_place_white_box_4) = 0
 R819: raw_overlapping(place_blue_box_2,place_white_box_4)
   - 1e+12 check_parallelism(place_blue_box_2,place_white_box_4) >= -1e+12
 R820: raw_overlapping(place_blue_box_2,place_white_box_4)
   - 1e+12 check_parallelism(place_blue_box_2,place_white_box_4) <= -1e-12
 R821: raw_overlapping(place_blue_box_3,place_blue_box_4)
   - min_t_end(place_blue_box_3,_place_blue_box_4)
   + max_t_start(place_blue_box_3,_place_blue_box_4) = 0
 R822: raw_overlapping(place_blue_box_3,place_blue_box_4)
   - 1e+12 check_parallelism(place_blue_box_3,place_blue_box_4) >= -1e+12
 R823: raw_overlapping(place_blue_box_3,place_blue_box_4)
   - 1e+12 check_parallelism(place_blue_box_3,place_blue_box_4) <= -1e-12
 R824: raw_overlapping(place_blue_box_3,place_white_box_1)
   - min_t_end(place_blue_box_3,_place_white_box_1)
   + max_t_start(place_blue_box_3,_place_white_box_1) = 0
 R825: raw_overlapping(place_blue_box_3,place_white_box_1)
   - 1e+12 check_parallelism(place_blue_box_3,place_white_box_1) >= -1e+12
 R826: raw_overlapping(place_blue_box_3,place_white_box_1)
   - 1e+12 check_parallelism(place_blue_box_3,place_white_box_1) <= -1e-12
 R827: raw_overlapping(place_blue_box_3,place_white_box_2)
   - min_t_end(place_blue_box_3,_place_white_box_2)
   + max_t_start(place_blue_box_3,_place_white_box_2) = 0
 R828: raw_overlapping(place_blue_box_3,place_white_box_2)
   - 1e+12 check_parallelism(place_blue_box_3,place_white_box_2) >= -1e+12
 R829: raw_overlapping(place_blue_box_3,place_white_box_2)
   - 1e+12 check_parallelism(place_blue_box_3,place_white_box_2) <= -1e-12
 R830: raw_overlapping(place_blue_box_3,place_white_box_3)
   - min_t_end(place_blue_box_3,_place_white_box_3)
   + max_t_start(place_blue_box_3,_place_white_box_3) = 0
 R831: raw_overlapping(place_blue_box_3,place_white_box_3)
   - 1e+12 check_parallelism(place_blue_box_3,place_white_box_3) >= -1e+12
 R832: raw_overlapping(place_blue_box_3,place_white_box_3)
   - 1e+12 check_parallelism(place_blue_box_3,place_white_box_3) <= -1e-12
 R833: raw_overlapping(place_blue_box_3,place_white_box_4)
   - min_t_end(place_blue_box_3,_place_white_box_4)
   + max_t_start(place_blue_box_3,_place_white_box_4) = 0
 R834: raw_overlapping(place_blue_box_3,place_white_box_4)
   - 1e+12 check_parallelism(place_blue_box_3,place_white_box_4) >= -1e+12
 R835: raw_overlapping(place_blue_box_3,place_white_box_4)
   - 1e+12 check_parallelism(place_blue_box_3,place_white_box_4) <= -1e-12
 R836: raw_overlapping(place_blue_box_4,place_white_box_1)
   - min_t_end(place_blue_box_4,_place_white_box_1)
   + max_t_start(place_blue_box_4,_place_white_box_1) = 0
 R837: raw_overlapping(place_blue_box_4,place_white_box_1)
   - 1e+12 check_parallelism(place_blue_box_4,place_white_box_1) >= -1e+12
 R838: raw_overlapping(place_blue_box_4,place_white_box_1)
   - 1e+12 check_parallelism(place_blue_box_4,place_white_box_1) <= -1e-12
 R839: raw_overlapping(place_blue_box_4,place_white_box_2)
   - min_t_end(place_blue_box_4,_place_white_box_2)
   + max_t_start(place_blue_box_4,_place_white_box_2) = 0
 R840: raw_overlapping(place_blue_box_4,place_white_box_2)
   - 1e+12 check_parallelism(place_blue_box_4,place_white_box_2) >= -1e+12
 R841: raw_overlapping(place_blue_box_4,place_white_box_2)
   - 1e+12 check_parallelism(place_blue_box_4,place_white_box_2) <= -1e-12
 R842: raw_overlapping(place_blue_box_4,place_white_box_3)
   - min_t_end(place_blue_box_4,_place_white_box_3)
   + max_t_start(place_blue_box_4,_place_white_box_3) = 0
 R843: raw_overlapping(place_blue_box_4,place_white_box_3)
   - 1e+12 check_parallelism(place_blue_box_4,place_white_box_3) >= -1e+12
 R844: raw_overlapping(place_blue_box_4,place_white_box_3)
   - 1e+12 check_parallelism(place_blue_box_4,place_white_box_3) <= -1e-12
 R845: raw_overlapping(place_blue_box_4,place_white_box_4)
   - min_t_end(place_blue_box_4,_place_white_box_4)
   + max_t_start(place_blue_box_4,_place_white_box_4) = 0
 R846: raw_overlapping(place_blue_box_4,place_white_box_4)
   - 1e+12 check_parallelism(place_blue_box_4,place_white_box_4) >= -1e+12
 R847: raw_overlapping(place_blue_box_4,place_white_box_4)
   - 1e+12 check_parallelism(place_blue_box_4,place_white_box_4) <= -1e-12
 R848: raw_overlapping(place_white_box_1,place_white_box_2)
   - min_t_end(place_white_box_1,_place_white_box_2)
   + max_t_start(place_white_box_1,_place_white_box_2) = 0
 R849: raw_overlapping(place_white_box_1,place_white_box_2)
   - 1e+12 check_parallelism(place_white_box_1,place_white_box_2)
   >= -1e+12
 R850: raw_overlapping(place_white_box_1,place_white_box_2)
   - 1e+12 check_parallelism(place_white_box_1,place_white_box_2)
   <= -1e-12
 R851: raw_overlapping(place_white_box_1,place_white_box_3)
   - min_t_end(place_white_box_1,_place_white_box_3)
   + max_t_start(place_white_box_1,_place_white_box_3) = 0
 R852: raw_overlapping(place_white_box_1,place_white_box_3)
   - 1e+12 check_parallelism(place_white_box_1,place_white_box_3)
   >= -1e+12
 R853: raw_overlapping(place_white_box_1,place_white_box_3)
   - 1e+12 check_parallelism(place_white_box_1,place_white_box_3)
   <= -1e-12
 R854: raw_overlapping(place_white_box_1,place_white_box_4)
   - min_t_end(place_white_box_1,_place_white_box_4)
   + max_t_start(place_white_box_1,_place_white_box_4) = 0
 R855: raw_overlapping(place_white_box_1,place_white_box_4)
   - 1e+12 check_parallelism(place_white_box_1,place_white_box_4)
   >= -1e+12
 R856: raw_overlapping(place_white_box_1,place_white_box_4)
   - 1e+12 check_parallelism(place_white_box_1,place_white_box_4)
   <= -1e-12
 R857: raw_overlapping(place_white_box_2,place_white_box_3)
   - min_t_end(place_white_box_2,_place_white_box_3)
   + max_t_start(place_white_box_2,_place_white_box_3) = 0
 R858: raw_overlapping(place_white_box_2,place_white_box_3)
   - 1e+12 check_parallelism(place_white_box_2,place_white_box_3)
   >= -1e+12
 R859: raw_overlapping(place_white_box_2,place_white_box_3)
   - 1e+12 check_parallelism(place_white_box_2,place_white_box_3)
   <= -1e-12
 R860: raw_overlapping(place_white_box_2,place_white_box_4)
   - min_t_end(place_white_box_2,_place_white_box_4)
   + max_t_start(place_white_box_2,_place_white_box_4) = 0
 R861: raw_overlapping(place_white_box_2,place_white_box_4)
   - 1e+12 check_parallelism(place_white_box_2,place_white_box_4)
   >= -1e+12
 R862: raw_overlapping(place_white_box_2,place_white_box_4)
   - 1e+12 check_parallelism(place_white_box_2,place_white_box_4)
   <= -1e-12
 R863: raw_overlapping(place_white_box_3,place_white_box_4)
   - min_t_end(place_white_box_3,_place_white_box_4)
   + max_t_start(place_white_box_3,_place_white_box_4) = 0
 R864: raw_overlapping(place_white_box_3,place_white_box_4)
   - 1e+12 check_parallelism(place_white_box_3,place_white_box_4)
   >= -1e+12
 R865: raw_overlapping(place_white_box_3,place_white_box_4)
   - 1e+12 check_parallelism(place_white_box_3,place_white_box_4)
   <= -1e-12
 R866: t_start[place_orange_box_1] - t_end[pick_orange_box_1] = 0
 R867: t_start[place_orange_box_2] - t_end[pick_orange_box_2] = 0
 R868: t_start[place_orange_box_3] - t_end[pick_orange_box_3] = 0
 R869: t_start[place_orange_box_4] - t_end[pick_orange_box_4] = 0
 R870: t_start[place_blue_box_1] - t_end[pick_blue_box_1] = 0
 R871: t_start[place_blue_box_2] - t_end[pick_blue_box_2] = 0
 R872: t_start[place_blue_box_3] - t_end[pick_blue_box_3] = 0
 R873: t_start[place_blue_box_4] - t_end[pick_blue_box_4] = 0
 R874: t_start[place_white_box_1] - t_end[pick_white_box_1] = 0
 R875: t_start[place_white_box_2] - t_end[pick_white_box_2] = 0
 R876: t_start[place_white_box_3] - t_end[pick_white_box_3] = 0
 R877: t_start[place_white_box_4] - t_end[pick_white_box_4] = 0
 t_end(pick_orange_box_1):
   - 10.60008821317128 assignment[ur5_on_guide,pick_orange_box_1]
   - t_start[pick_orange_box_1] + t_end[pick_orange_box_1] + [
   - 0.9666509893424524 overlapping[pick_orange_box_1,pick_blue_box_1] * assignment[ur5_on_guide,pick_orange_box_1]
   - 0.9666509893424524 overlapping[pick_orange_box_1,pick_blue_box_2] * assignment[ur5_on_guide,pick_orange_box_1]
   - 0.9666509893424524 overlapping[pick_orange_box_1,pick_blue_box_3] * assignment[ur5_on_guide,pick_orange_box_1]
   - 0.9666509893424524 overlapping[pick_orange_box_1,pick_blue_box_4] * assignment[ur5_on_guide,pick_orange_box_1]
   + 0.1326450146107383 overlapping[pick_orange_box_1,pick_white_box_1] * assignment[ur5_on_guide,pick_orange_box_1]
   + 0.1326450146107383 overlapping[pick_orange_box_1,pick_white_box_2] * assignment[ur5_on_guide,pick_orange_box_1]
   + 0.1326450146107383 overlapping[pick_orange_box_1,pick_white_box_3] * assignment[ur5_on_guide,pick_orange_box_1]
   + 0.1326450146107383 overlapping[pick_orange_box_1,pick_white_box_4] * assignment[ur5_on_guide,pick_orange_box_1]
   - 0.8977116924849142 overlapping[pick_orange_box_1,place_blue_box_1] * assignment[ur5_on_guide,pick_orange_box_1]
   - 0.8977116924849142 overlapping[pick_orange_box_1,place_blue_box_2] * assignment[ur5_on_guide,pick_orange_box_1]
   + 0.2393885289173742 overlapping[pick_orange_box_1,place_white_box_1] * assignment[ur5_on_guide,pick_orange_box_1]
   + 0.2393885289173742 overlapping[pick_orange_box_1,place_white_box_2] * assignment[ur5_on_guide,pick_orange_box_1]
   + 0.2393885289173742 overlapping[pick_orange_box_1,place_white_box_3] * assignment[ur5_on_guide,pick_orange_box_1]
   + 0.2393885289173742 overlapping[pick_orange_box_1,place_white_box_4] * assignment[ur5_on_guide,pick_orange_box_1]
   ] = 0
 t_end(pick_orange_box_2):
   - 10.60008821317128 assignment[ur5_on_guide,pick_orange_box_2]
   - t_start[pick_orange_box_2] + t_end[pick_orange_box_2] + [
   - 0.9666509893424524 overlapping[pick_orange_box_2,pick_blue_box_1] * assignment[ur5_on_guide,pick_orange_box_2]
   - 0.9666509893424524 overlapping[pick_orange_box_2,pick_blue_box_2] * assignment[ur5_on_guide,pick_orange_box_2]
   - 0.9666509893424524 overlapping[pick_orange_box_2,pick_blue_box_3] * assignment[ur5_on_guide,pick_orange_box_2]
   - 0.9666509893424524 overlapping[pick_orange_box_2,pick_blue_box_4] * assignment[ur5_on_guide,pick_orange_box_2]
   + 0.1326450146107383 overlapping[pick_orange_box_2,pick_white_box_1] * assignment[ur5_on_guide,pick_orange_box_2]
   + 0.1326450146107383 overlapping[pick_orange_box_2,pick_white_box_2] * assignment[ur5_on_guide,pick_orange_box_2]
   + 0.1326450146107383 overlapping[pick_orange_box_2,pick_white_box_3] * assignment[ur5_on_guide,pick_orange_box_2]
   + 0.1326450146107383 overlapping[pick_orange_box_2,pick_white_box_4] * assignment[ur5_on_guide,pick_orange_box_2]
   - 0.8977116924849142 overlapping[pick_orange_box_2,place_blue_box_1] * assignment[ur5_on_guide,pick_orange_box_2]
   - 0.8977116924849142 overlapping[pick_orange_box_2,place_blue_box_2] * assignment[ur5_on_guide,pick_orange_box_2]
   + 0.2393885289173742 overlapping[pick_orange_box_2,place_white_box_1] * assignment[ur5_on_guide,pick_orange_box_2]
   + 0.2393885289173742 overlapping[pick_orange_box_2,place_white_box_2] * assignment[ur5_on_guide,pick_orange_box_2]
   + 0.2393885289173742 overlapping[pick_orange_box_2,place_white_box_3] * assignment[ur5_on_guide,pick_orange_box_2]
   + 0.2393885289173742 overlapping[pick_orange_box_2,place_white_box_4] * assignment[ur5_on_guide,pick_orange_box_2]
   ] = 0
 t_end(pick_orange_box_3):
   - 10.60008821317128 assignment[ur5_on_guide,pick_orange_box_3]
   - t_start[pick_orange_box_3] + t_end[pick_orange_box_3] + [
   - 0.9666509893424524 overlapping[pick_orange_box_3,pick_blue_box_1] * assignment[ur5_on_guide,pick_orange_box_3]
   - 0.9666509893424524 overlapping[pick_orange_box_3,pick_blue_box_2] * assignment[ur5_on_guide,pick_orange_box_3]
   - 0.9666509893424524 overlapping[pick_orange_box_3,pick_blue_box_3] * assignment[ur5_on_guide,pick_orange_box_3]
   - 0.9666509893424524 overlapping[pick_orange_box_3,pick_blue_box_4] * assignment[ur5_on_guide,pick_orange_box_3]
   + 0.1326450146107383 overlapping[pick_orange_box_3,pick_white_box_1] * assignment[ur5_on_guide,pick_orange_box_3]
   + 0.1326450146107383 overlapping[pick_orange_box_3,pick_white_box_2] * assignment[ur5_on_guide,pick_orange_box_3]
   + 0.1326450146107383 overlapping[pick_orange_box_3,pick_white_box_3] * assignment[ur5_on_guide,pick_orange_box_3]
   + 0.1326450146107383 overlapping[pick_orange_box_3,pick_white_box_4] * assignment[ur5_on_guide,pick_orange_box_3]
   - 0.8977116924849142 overlapping[pick_orange_box_3,place_blue_box_1] * assignment[ur5_on_guide,pick_orange_box_3]
   - 0.8977116924849142 overlapping[pick_orange_box_3,place_blue_box_2] * assignment[ur5_on_guide,pick_orange_box_3]
   + 0.2393885289173742 overlapping[pick_orange_box_3,place_white_box_1] * assignment[ur5_on_guide,pick_orange_box_3]
   + 0.2393885289173742 overlapping[pick_orange_box_3,place_white_box_2] * assignment[ur5_on_guide,pick_orange_box_3]
   + 0.2393885289173742 overlapping[pick_orange_box_3,place_white_box_3] * assignment[ur5_on_guide,pick_orange_box_3]
   + 0.2393885289173742 overlapping[pick_orange_box_3,place_white_box_4] * assignment[ur5_on_guide,pick_orange_box_3]
   ] = 0
 t_end(pick_orange_box_4):
   - 10.60008821317128 assignment[ur5_on_guide,pick_orange_box_4]
   - t_start[pick_orange_box_4] + t_end[pick_orange_box_4] + [
   - 0.9666509893424524 overlapping[pick_orange_box_4,pick_blue_box_1] * assignment[ur5_on_guide,pick_orange_box_4]
   - 0.9666509893424524 overlapping[pick_orange_box_4,pick_blue_box_2] * assignment[ur5_on_guide,pick_orange_box_4]
   - 0.9666509893424524 overlapping[pick_orange_box_4,pick_blue_box_3] * assignment[ur5_on_guide,pick_orange_box_4]
   - 0.9666509893424524 overlapping[pick_orange_box_4,pick_blue_box_4] * assignment[ur5_on_guide,pick_orange_box_4]
   + 0.1326450146107383 overlapping[pick_orange_box_4,pick_white_box_1] * assignment[ur5_on_guide,pick_orange_box_4]
   + 0.1326450146107383 overlapping[pick_orange_box_4,pick_white_box_2] * assignment[ur5_on_guide,pick_orange_box_4]
   + 0.1326450146107383 overlapping[pick_orange_box_4,pick_white_box_3] * assignment[ur5_on_guide,pick_orange_box_4]
   + 0.1326450146107383 overlapping[pick_orange_box_4,pick_white_box_4] * assignment[ur5_on_guide,pick_orange_box_4]
   - 0.8977116924849142 overlapping[pick_orange_box_4,place_blue_box_1] * assignment[ur5_on_guide,pick_orange_box_4]
   - 0.8977116924849142 overlapping[pick_orange_box_4,place_blue_box_2] * assignment[ur5_on_guide,pick_orange_box_4]
   + 0.2393885289173742 overlapping[pick_orange_box_4,place_white_box_1] * assignment[ur5_on_guide,pick_orange_box_4]
   + 0.2393885289173742 overlapping[pick_orange_box_4,place_white_box_2] * assignment[ur5_on_guide,pick_orange_box_4]
   + 0.2393885289173742 overlapping[pick_orange_box_4,place_white_box_3] * assignment[ur5_on_guide,pick_orange_box_4]
   + 0.2393885289173742 overlapping[pick_orange_box_4,place_white_box_4] * assignment[ur5_on_guide,pick_orange_box_4]
   ] = 0
 t_end(pick_blue_box_1):
   - 8.78856864267466 assignment[ur5_on_guide,pick_blue_box_1]
   - 7.413828785419464 assignment[human_right_arm,pick_blue_box_1]
   - t_start[pick_blue_box_1] + t_end[pick_blue_box_1] + [
   - 0.8855062825232063 overlapping[pick_blue_box_1,pick_blue_box_2] * assignment[ur5_on_guide,pick_blue_box_1]
   - 0.8855062825232063 overlapping[pick_blue_box_1,pick_blue_box_3] * assignment[ur5_on_guide,pick_blue_box_1]
   - 0.8855062825232063 overlapping[pick_blue_box_1,pick_blue_box_4] * assignment[ur5_on_guide,pick_blue_box_1]
   + 0.1907921031582629 overlapping[pick_blue_box_1,pick_white_box_1] * assignment[ur5_on_guide,pick_blue_box_1]
   + 0.1907921031582629 overlapping[pick_blue_box_1,pick_white_box_2] * assignment[ur5_on_guide,pick_blue_box_1]
   + 0.1907921031582629 overlapping[pick_blue_box_1,pick_white_box_3] * assignment[ur5_on_guide,pick_blue_box_1]
   + 0.1907921031582629 overlapping[pick_blue_box_1,pick_white_box_4] * assignment[ur5_on_guide,pick_blue_box_1]
   - 0.8390158788221753 overlapping[pick_blue_box_1,place_blue_box_1] * assignment[ur5_on_guide,pick_blue_box_1]
   - 0.8390158788221753 overlapping[pick_blue_box_1,place_blue_box_2] * assignment[ur5_on_guide,pick_blue_box_1]
   + 0.3441159162648654 overlapping[pick_blue_box_1,place_white_box_1] * assignment[ur5_on_guide,pick_blue_box_1]
   + 0.3441159162648654 overlapping[pick_blue_box_1,place_white_box_2] * assignment[ur5_on_guide,pick_blue_box_1]
   + 0.3441159162648654 overlapping[pick_blue_box_1,place_white_box_3] * assignment[ur5_on_guide,pick_blue_box_1]
   + 0.3441159162648654 overlapping[pick_blue_box_1,place_white_box_4] * assignment[ur5_on_guide,pick_blue_box_1]
   ] = 0
 t_end(pick_blue_box_2):
   - 8.78856864267466 assignment[ur5_on_guide,pick_blue_box_2]
   - 7.413828785419464 assignment[human_right_arm,pick_blue_box_2]
   - t_start[pick_blue_box_2] + t_end[pick_blue_box_2] + [
   - 0.8855062825232063 overlapping[pick_blue_box_1,pick_blue_box_2] * assignment[ur5_on_guide,pick_blue_box_2]
   - 0.8855062825232063 overlapping[pick_blue_box_2,pick_blue_box_3] * assignment[ur5_on_guide,pick_blue_box_2]
   - 0.8855062825232063 overlapping[pick_blue_box_2,pick_blue_box_4] * assignment[ur5_on_guide,pick_blue_box_2]
   + 0.1907921031582629 overlapping[pick_blue_box_2,pick_white_box_1] * assignment[ur5_on_guide,pick_blue_box_2]
   + 0.1907921031582629 overlapping[pick_blue_box_2,pick_white_box_2] * assignment[ur5_on_guide,pick_blue_box_2]
   + 0.1907921031582629 overlapping[pick_blue_box_2,pick_white_box_3] * assignment[ur5_on_guide,pick_blue_box_2]
   + 0.1907921031582629 overlapping[pick_blue_box_2,pick_white_box_4] * assignment[ur5_on_guide,pick_blue_box_2]
   - 0.8390158788221753 overlapping[pick_blue_box_2,place_blue_box_1] * assignment[ur5_on_guide,pick_blue_box_2]
   - 0.8390158788221753 overlapping[pick_blue_box_2,place_blue_box_2] * assignment[ur5_on_guide,pick_blue_box_2]
   + 0.3441159162648654 overlapping[pick_blue_box_2,place_white_box_1] * assignment[ur5_on_guide,pick_blue_box_2]
   + 0.3441159162648654 overlapping[pick_blue_box_2,place_white_box_2] * assignment[ur5_on_guide,pick_blue_box_2]
   + 0.3441159162648654 overlapping[pick_blue_box_2,place_white_box_3] * assignment[ur5_on_guide,pick_blue_box_2]
   + 0.3441159162648654 overlapping[pick_blue_box_2,place_white_box_4] * assignment[ur5_on_guide,pick_blue_box_2]
   ] = 0
 t_end(pick_blue_box_3):
   - 8.78856864267466 assignment[ur5_on_guide,pick_blue_box_3]
   - 7.413828785419464 assignment[human_right_arm,pick_blue_box_3]
   - t_start[pick_blue_box_3] + t_end[pick_blue_box_3] + [
   - 0.8855062825232063 overlapping[pick_blue_box_1,pick_blue_box_3] * assignment[ur5_on_guide,pick_blue_box_3]
   - 0.8855062825232063 overlapping[pick_blue_box_2,pick_blue_box_3] * assignment[ur5_on_guide,pick_blue_box_3]
   - 0.8855062825232063 overlapping[pick_blue_box_3,pick_blue_box_4] * assignment[ur5_on_guide,pick_blue_box_3]
   + 0.1907921031582629 overlapping[pick_blue_box_3,pick_white_box_1] * assignment[ur5_on_guide,pick_blue_box_3]
   + 0.1907921031582629 overlapping[pick_blue_box_3,pick_white_box_2] * assignment[ur5_on_guide,pick_blue_box_3]
   + 0.1907921031582629 overlapping[pick_blue_box_3,pick_white_box_3] * assignment[ur5_on_guide,pick_blue_box_3]
   + 0.1907921031582629 overlapping[pick_blue_box_3,pick_white_box_4] * assignment[ur5_on_guide,pick_blue_box_3]
   - 0.8390158788221753 overlapping[pick_blue_box_3,place_blue_box_1] * assignment[ur5_on_guide,pick_blue_box_3]
   - 0.8390158788221753 overlapping[pick_blue_box_3,place_blue_box_2] * assignment[ur5_on_guide,pick_blue_box_3]
   + 0.3441159162648654 overlapping[pick_blue_box_3,place_white_box_1] * assignment[ur5_on_guide,pick_blue_box_3]
   + 0.3441159162648654 overlapping[pick_blue_box_3,place_white_box_2] * assignment[ur5_on_guide,pick_blue_box_3]
   + 0.3441159162648654 overlapping[pick_blue_box_3,place_white_box_3] * assignment[ur5_on_guide,pick_blue_box_3]
   + 0.3441159162648654 overlapping[pick_blue_box_3,place_white_box_4] * assignment[ur5_on_guide,pick_blue_box_3]
   ] = 0
 t_end(pick_blue_box_4):
   - 8.78856864267466 assignment[ur5_on_guide,pick_blue_box_4]
   - 7.413828785419464 assignment[human_right_arm,pick_blue_box_4]
   - t_start[pick_blue_box_4] + t_end[pick_blue_box_4] + [
   - 0.8855062825232063 overlapping[pick_blue_box_1,pick_blue_box_4] * assignment[ur5_on_guide,pick_blue_box_4]
   - 0.8855062825232063 overlapping[pick_blue_box_2,pick_blue_box_4] * assignment[ur5_on_guide,pick_blue_box_4]
   - 0.8855062825232063 overlapping[pick_blue_box_3,pick_blue_box_4] * assignment[ur5_on_guide,pick_blue_box_4]
   + 0.1907921031582629 overlapping[pick_blue_box_4,pick_white_box_1] * assignment[ur5_on_guide,pick_blue_box_4]
   + 0.1907921031582629 overlapping[pick_blue_box_4,pick_white_box_2] * assignment[ur5_on_guide,pick_blue_box_4]
   + 0.1907921031582629 overlapping[pick_blue_box_4,pick_white_box_3] * assignment[ur5_on_guide,pick_blue_box_4]
   + 0.1907921031582629 overlapping[pick_blue_box_4,pick_white_box_4] * assignment[ur5_on_guide,pick_blue_box_4]
   - 0.8390158788221753 overlapping[pick_blue_box_4,place_blue_box_1] * assignment[ur5_on_guide,pick_blue_box_4]
   - 0.8390158788221753 overlapping[pick_blue_box_4,place_blue_box_2] * assignment[ur5_on_guide,pick_blue_box_4]
   + 0.3441159162648654 overlapping[pick_blue_box_4,place_white_box_1] * assignment[ur5_on_guide,pick_blue_box_4]
   + 0.3441159162648654 overlapping[pick_blue_box_4,place_white_box_2] * assignment[ur5_on_guide,pick_blue_box_4]
   + 0.3441159162648654 overlapping[pick_blue_box_4,place_white_box_3] * assignment[ur5_on_guide,pick_blue_box_4]
   + 0.3441159162648654 overlapping[pick_blue_box_4,place_white_box_4] * assignment[ur5_on_guide,pick_blue_box_4]
   ] = 0
 t_end(place_orange_box_1):
   - 6.88590753138973 assignment[ur5_on_guide,place_orange_box_1]
   - t_start[place_orange_box_1] + t_end[place_orange_box_1] + [
   - 1.495520092632073 overlapping[pick_blue_box_1,place_orange_box_1] * assignment[ur5_on_guide,place_orange_box_1]
   - 1.495520092632073 overlapping[pick_blue_box_2,place_orange_box_1] * assignment[ur5_on_guide,place_orange_box_1]
   - 1.495520092632073 overlapping[pick_blue_box_3,place_orange_box_1] * assignment[ur5_on_guide,place_orange_box_1]
   - 1.495520092632073 overlapping[pick_blue_box_4,place_orange_box_1] * assignment[ur5_on_guide,place_orange_box_1]
   + 0.0176704980269988 overlapping[pick_white_box_1,place_orange_box_1] * assignment[ur5_on_guide,place_orange_box_1]
   + 0.0176704980269988 overlapping[pick_white_box_2,place_orange_box_1] * assignment[ur5_on_guide,place_orange_box_1]
   + 0.0176704980269988 overlapping[pick_white_box_3,place_orange_box_1] * assignment[ur5_on_guide,place_orange_box_1]
   + 0.0176704980269988 overlapping[pick_white_box_4,place_orange_box_1] * assignment[ur5_on_guide,place_orange_box_1]
   - 1.633384541352432 overlapping[place_orange_box_1,place_blue_box_1] * assignment[ur5_on_guide,place_orange_box_1]
   - 1.633384541352432 overlapping[place_orange_box_1,place_blue_box_2] * assignment[ur5_on_guide,place_orange_box_1]
   + 0.1084490023152702 overlapping[place_orange_box_1,place_white_box_1] * assignment[ur5_on_guide,place_orange_box_1]
   + 0.1084490023152702 overlapping[place_orange_box_1,place_white_box_2] * assignment[ur5_on_guide,place_orange_box_1]
   + 0.1084490023152702 overlapping[place_orange_box_1,place_white_box_3] * assignment[ur5_on_guide,place_orange_box_1]
   + 0.1084490023152702 overlapping[place_orange_box_1,place_white_box_4] * assignment[ur5_on_guide,place_orange_box_1]
   ] = 0
 t_end(place_orange_box_2):
   - 6.88590753138973 assignment[ur5_on_guide,place_orange_box_2]
   - t_start[place_orange_box_2] + t_end[place_orange_box_2] + [
   - 1.495520092632073 overlapping[pick_blue_box_1,place_orange_box_2] * assignment[ur5_on_guide,place_orange_box_2]
   - 1.495520092632073 overlapping[pick_blue_box_2,place_orange_box_2] * assignment[ur5_on_guide,place_orange_box_2]
   - 1.495520092632073 overlapping[pick_blue_box_3,place_orange_box_2] * assignment[ur5_on_guide,place_orange_box_2]
   - 1.495520092632073 overlapping[pick_blue_box_4,place_orange_box_2] * assignment[ur5_on_guide,place_orange_box_2]
   + 0.0176704980269988 overlapping[pick_white_box_1,place_orange_box_2] * assignment[ur5_on_guide,place_orange_box_2]
   + 0.0176704980269988 overlapping[pick_white_box_2,place_orange_box_2] * assignment[ur5_on_guide,place_orange_box_2]
   + 0.0176704980269988 overlapping[pick_white_box_3,place_orange_box_2] * assignment[ur5_on_guide,place_orange_box_2]
   + 0.0176704980269988 overlapping[pick_white_box_4,place_orange_box_2] * assignment[ur5_on_guide,place_orange_box_2]
   - 1.633384541352432 overlapping[place_orange_box_2,place_blue_box_1] * assignment[ur5_on_guide,place_orange_box_2]
   - 1.633384541352432 overlapping[place_orange_box_2,place_blue_box_2] * assignment[ur5_on_guide,place_orange_box_2]
   + 0.1084490023152702 overlapping[place_orange_box_2,place_white_box_1] * assignment[ur5_on_guide,place_orange_box_2]
   + 0.1084490023152702 overlapping[place_orange_box_2,place_white_box_2] * assignment[ur5_on_guide,place_orange_box_2]
   + 0.1084490023152702 overlapping[place_orange_box_2,place_white_box_3] * assignment[ur5_on_guide,place_orange_box_2]
   + 0.1084490023152702 overlapping[place_orange_box_2,place_white_box_4] * assignment[ur5_on_guide,place_orange_box_2]
   ] = 0
 t_end(place_orange_box_3):
   - 6.88590753138973 assignment[ur5_on_guide,place_orange_box_3]
   - t_start[place_orange_box_3] + t_end[place_orange_box_3] + [
   - 1.495520092632073 overlapping[pick_blue_box_1,place_orange_box_3] * assignment[ur5_on_guide,place_orange_box_3]
   - 1.495520092632073 overlapping[pick_blue_box_2,place_orange_box_3] * assignment[ur5_on_guide,place_orange_box_3]
   - 1.495520092632073 overlapping[pick_blue_box_3,place_orange_box_3] * assignment[ur5_on_guide,place_orange_box_3]
   - 1.495520092632073 overlapping[pick_blue_box_4,place_orange_box_3] * assignment[ur5_on_guide,place_orange_box_3]
   + 0.0176704980269988 overlapping[pick_white_box_1,place_orange_box_3] * assignment[ur5_on_guide,place_orange_box_3]
   + 0.0176704980269988 overlapping[pick_white_box_2,place_orange_box_3] * assignment[ur5_on_guide,place_orange_box_3]
   + 0.0176704980269988 overlapping[pick_white_box_3,place_orange_box_3] * assignment[ur5_on_guide,place_orange_box_3]
   + 0.0176704980269988 overlapping[pick_white_box_4,place_orange_box_3] * assignment[ur5_on_guide,place_orange_box_3]
   - 1.633384541352432 overlapping[place_orange_box_3,place_blue_box_1] * assignment[ur5_on_guide,place_orange_box_3]
   - 1.633384541352432 overlapping[place_orange_box_3,place_blue_box_2] * assignment[ur5_on_guide,place_orange_box_3]
   + 0.1084490023152702 overlapping[place_orange_box_3,place_white_box_1] * assignment[ur5_on_guide,place_orange_box_3]
   + 0.1084490023152702 overlapping[place_orange_box_3,place_white_box_2] * assignment[ur5_on_guide,place_orange_box_3]
   + 0.1084490023152702 overlapping[place_orange_box_3,place_white_box_3] * assignment[ur5_on_guide,place_orange_box_3]
   + 0.1084490023152702 overlapping[place_orange_box_3,place_white_box_4] * assignment[ur5_on_guide,place_orange_box_3]
   ] = 0
 t_end(place_orange_box_4):
   - 6.88590753138973 assignment[ur5_on_guide,place_orange_box_4]
   - t_start[place_orange_box_4] + t_end[place_orange_box_4] + [
   - 1.495520092632073 overlapping[pick_blue_box_1,place_orange_box_4] * assignment[ur5_on_guide,place_orange_box_4]
   - 1.495520092632073 overlapping[pick_blue_box_2,place_orange_box_4] * assignment[ur5_on_guide,place_orange_box_4]
   - 1.495520092632073 overlapping[pick_blue_box_3,place_orange_box_4] * assignment[ur5_on_guide,place_orange_box_4]
   - 1.495520092632073 overlapping[pick_blue_box_4,place_orange_box_4] * assignment[ur5_on_guide,place_orange_box_4]
   + 0.0176704980269988 overlapping[pick_white_box_1,place_orange_box_4] * assignment[ur5_on_guide,place_orange_box_4]
   + 0.0176704980269988 overlapping[pick_white_box_2,place_orange_box_4] * assignment[ur5_on_guide,place_orange_box_4]
   + 0.0176704980269988 overlapping[pick_white_box_3,place_orange_box_4] * assignment[ur5_on_guide,place_orange_box_4]
   + 0.0176704980269988 overlapping[pick_white_box_4,place_orange_box_4] * assignment[ur5_on_guide,place_orange_box_4]
   - 1.633384541352432 overlapping[place_orange_box_4,place_blue_box_1] * assignment[ur5_on_guide,place_orange_box_4]
   - 1.633384541352432 overlapping[place_orange_box_4,place_blue_box_2] * assignment[ur5_on_guide,place_orange_box_4]
   + 0.1084490023152702 overlapping[place_orange_box_4,place_white_box_1] * assignment[ur5_on_guide,place_orange_box_4]
   + 0.1084490023152702 overlapping[place_orange_box_4,place_white_box_2] * assignment[ur5_on_guide,place_orange_box_4]
   + 0.1084490023152702 overlapping[place_orange_box_4,place_white_box_3] * assignment[ur5_on_guide,place_orange_box_4]
   + 0.1084490023152702 overlapping[place_orange_box_4,place_white_box_4] * assignment[ur5_on_guide,place_orange_box_4]
   ] = 0
 t_end(place_blue_box_3):
   - 5.064355108202721 assignment[ur5_on_guide,place_blue_box_3]
   - t_start[place_blue_box_3] + t_end[place_blue_box_3] + [
   - 5.075296604441036 overlapping[pick_blue_box_1,place_blue_box_3] * assignment[ur5_on_guide,place_blue_box_3]
   - 5.075296604441036 overlapping[pick_blue_box_2,place_blue_box_3] * assignment[ur5_on_guide,place_blue_box_3]
   - 5.075296604441036 overlapping[pick_blue_box_3,place_blue_box_3] * assignment[ur5_on_guide,place_blue_box_3]
   - 5.075296604441036 overlapping[pick_blue_box_4,place_blue_box_3] * assignment[ur5_on_guide,place_blue_box_3]
   + 0.041830962660658 overlapping[pick_white_box_1,place_blue_box_3] * assignment[ur5_on_guide,place_blue_box_3]
   + 0.041830962660658 overlapping[pick_white_box_2,place_blue_box_3] * assignment[ur5_on_guide,place_blue_box_3]
   + 0.041830962660658 overlapping[pick_white_box_3,place_blue_box_3] * assignment[ur5_on_guide,place_blue_box_3]
   + 0.041830962660658 overlapping[pick_white_box_4,place_blue_box_3] * assignment[ur5_on_guide,place_blue_box_3]
   - 2.51432359800111 overlapping[place_blue_box_1,place_blue_box_3] * assignment[ur5_on_guide,place_blue_box_3]
   - 2.51432359800111 overlapping[place_blue_box_2,place_blue_box_3] * assignment[ur5_on_guide,place_blue_box_3]
   + 0.0335777405026962 overlapping[place_blue_box_3,place_white_box_1] * assignment[ur5_on_guide,place_blue_box_3]
   + 0.0335777405026962 overlapping[place_blue_box_3,place_white_box_2] * assignment[ur5_on_guide,place_blue_box_3]
   + 0.0335777405026962 overlapping[place_blue_box_3,place_white_box_3] * assignment[ur5_on_guide,place_blue_box_3]
   + 0.0335777405026962 overlapping[place_blue_box_3,place_white_box_4] * assignment[ur5_on_guide,place_blue_box_3]
   ] = 0
 t_end(place_blue_box_4):
   - 5.064355108202721 assignment[ur5_on_guide,place_blue_box_4]
   - t_start[place_blue_box_4] + t_end[place_blue_box_4] + [
   - 5.075296604441036 overlapping[pick_blue_box_1,place_blue_box_4] * assignment[ur5_on_guide,place_blue_box_4]
   - 5.075296604441036 overlapping[pick_blue_box_2,place_blue_box_4] * assignment[ur5_on_guide,place_blue_box_4]
   - 5.075296604441036 overlapping[pick_blue_box_3,place_blue_box_4] * assignment[ur5_on_guide,place_blue_box_4]
   - 5.075296604441036 overlapping[pick_blue_box_4,place_blue_box_4] * assignment[ur5_on_guide,place_blue_box_4]
   + 0.041830962660658 overlapping[pick_white_box_1,place_blue_box_4] * assignment[ur5_on_guide,place_blue_box_4]
   + 0.041830962660658 overlapping[pick_white_box_2,place_blue_box_4] * assignment[ur5_on_guide,place_blue_box_4]
   + 0.041830962660658 overlapping[pick_white_box_3,place_blue_box_4] * assignment[ur5_on_guide,place_blue_box_4]
   + 0.041830962660658 overlapping[pick_white_box_4,place_blue_box_4] * assignment[ur5_on_guide,place_blue_box_4]
   - 2.51432359800111 overlapping[place_blue_box_1,place_blue_box_4] * assignment[ur5_on_guide,place_blue_box_4]
   - 2.51432359800111 overlapping[place_blue_box_2,place_blue_box_4] * assignment[ur5_on_guide,place_blue_box_4]
   + 0.0335777405026962 overlapping[place_blue_box_4,place_white_box_1] * assignment[ur5_on_guide,place_blue_box_4]
   + 0.0335777405026962 overlapping[place_blue_box_4,place_white_box_2] * assignment[ur5_on_guide,place_blue_box_4]
   + 0.0335777405026962 overlapping[place_blue_box_4,place_white_box_3] * assignment[ur5_on_guide,place_blue_box_4]
   + 0.0335777405026962 overlapping[place_blue_box_4,place_white_box_4] * assignment[ur5_on_guide,place_blue_box_4]
   ] = 0
 qc14: overlapping[pick_orange_box_1,pick_orange_box_2] + [
   - min_t_end(pick_orange_box_1,_pick_orange_box_2) * check_parallelism(pick_orange_box_1,pick_orange_box_2)
   + max_t_start(pick_orange_box_1,_pick_orange_box_2) * check_parallelism(pick_orange_box_1,pick_orange_box_2)
   ] = 0
 qc15: overlapping[pick_orange_box_1,pick_orange_box_3] + [
   - min_t_end(pick_orange_box_1,_pick_orange_box_3) * check_parallelism(pick_orange_box_1,pick_orange_box_3)
   + max_t_start(pick_orange_box_1,_pick_orange_box_3) * check_parallelism(pick_orange_box_1,pick_orange_box_3)
   ] = 0
 qc16: overlapping[pick_orange_box_1,pick_orange_box_4] + [
   - min_t_end(pick_orange_box_1,_pick_orange_box_4) * check_parallelism(pick_orange_box_1,pick_orange_box_4)
   + max_t_start(pick_orange_box_1,_pick_orange_box_4) * check_parallelism(pick_orange_box_1,pick_orange_box_4)
   ] = 0
 qc17: overlapping[pick_orange_box_1,pick_blue_box_1] + [
   - min_t_end(pick_orange_box_1,_pick_blue_box_1) * check_parallelism(pick_orange_box_1,pick_blue_box_1)
   + max_t_start(pick_orange_box_1,_pick_blue_box_1) * check_parallelism(pick_orange_box_1,pick_blue_box_1)
   ] = 0
 qc18: overlapping[pick_orange_box_1,pick_blue_box_2] + [
   - min_t_end(pick_orange_box_1,_pick_blue_box_2) * check_parallelism(pick_orange_box_1,pick_blue_box_2)
   + max_t_start(pick_orange_box_1,_pick_blue_box_2) * check_parallelism(pick_orange_box_1,pick_blue_box_2)
   ] = 0
 qc19: overlapping[pick_orange_box_1,pick_blue_box_3] + [
   - min_t_end(pick_orange_box_1,_pick_blue_box_3) * check_parallelism(pick_orange_box_1,pick_blue_box_3)
   + max_t_start(pick_orange_box_1,_pick_blue_box_3) * check_parallelism(pick_orange_box_1,pick_blue_box_3)
   ] = 0
 qc20: overlapping[pick_orange_box_1,pick_blue_box_4] + [
   - min_t_end(pick_orange_box_1,_pick_blue_box_4) * check_parallelism(pick_orange_box_1,pick_blue_box_4)
   + max_t_start(pick_orange_box_1,_pick_blue_box_4) * check_parallelism(pick_orange_box_1,pick_blue_box_4)
   ] = 0
 qc21: overlapping[pick_orange_box_1,pick_white_box_1] + [
   - min_t_end(pick_orange_box_1,_pick_white_box_1) * check_parallelism(pick_orange_box_1,pick_white_box_1)
   + max_t_start(pick_orange_box_1,_pick_white_box_1) * check_parallelism(pick_orange_box_1,pick_white_box_1)
   ] = 0
 qc22: overlapping[pick_orange_box_1,pick_white_box_2] + [
   - min_t_end(pick_orange_box_1,_pick_white_box_2) * check_parallelism(pick_orange_box_1,pick_white_box_2)
   + max_t_start(pick_orange_box_1,_pick_white_box_2) * check_parallelism(pick_orange_box_1,pick_white_box_2)
   ] = 0
 qc23: overlapping[pick_orange_box_1,pick_white_box_3] + [
   - min_t_end(pick_orange_box_1,_pick_white_box_3) * check_parallelism(pick_orange_box_1,pick_white_box_3)
   + max_t_start(pick_orange_box_1,_pick_white_box_3) * check_parallelism(pick_orange_box_1,pick_white_box_3)
   ] = 0
 qc24: overlapping[pick_orange_box_1,pick_white_box_4] + [
   - min_t_end(pick_orange_box_1,_pick_white_box_4) * check_parallelism(pick_orange_box_1,pick_white_box_4)
   + max_t_start(pick_orange_box_1,_pick_white_box_4) * check_parallelism(pick_orange_box_1,pick_white_box_4)
   ] = 0
 qc25: overlapping[pick_orange_box_1,place_orange_box_1] + [
   - min_t_end(pick_orange_box_1,_place_orange_box_1) * check_parallelism(pick_orange_box_1,place_orange_box_1)
   + max_t_start(pick_orange_box_1,_place_orange_box_1) * check_parallelism(pick_orange_box_1,place_orange_box_1)
   ] = 0
 qc26: overlapping[pick_orange_box_1,place_orange_box_2] + [
   - min_t_end(pick_orange_box_1,_place_orange_box_2) * check_parallelism(pick_orange_box_1,place_orange_box_2)
   + max_t_start(pick_orange_box_1,_place_orange_box_2) * check_parallelism(pick_orange_box_1,place_orange_box_2)
   ] = 0
 qc27: overlapping[pick_orange_box_1,place_orange_box_3] + [
   - min_t_end(pick_orange_box_1,_place_orange_box_3) * check_parallelism(pick_orange_box_1,place_orange_box_3)
   + max_t_start(pick_orange_box_1,_place_orange_box_3) * check_parallelism(pick_orange_box_1,place_orange_box_3)
   ] = 0
 qc28: overlapping[pick_orange_box_1,place_orange_box_4] + [
   - min_t_end(pick_orange_box_1,_place_orange_box_4) * check_parallelism(pick_orange_box_1,place_orange_box_4)
   + max_t_start(pick_orange_box_1,_place_orange_box_4) * check_parallelism(pick_orange_box_1,place_orange_box_4)
   ] = 0
 qc29: overlapping[pick_orange_box_1,place_blue_box_1] + [
   - min_t_end(pick_orange_box_1,_place_blue_box_1) * check_parallelism(pick_orange_box_1,place_blue_box_1)
   + max_t_start(pick_orange_box_1,_place_blue_box_1) * check_parallelism(pick_orange_box_1,place_blue_box_1)
   ] = 0
 qc30: overlapping[pick_orange_box_1,place_blue_box_2] + [
   - min_t_end(pick_orange_box_1,_place_blue_box_2) * check_parallelism(pick_orange_box_1,place_blue_box_2)
   + max_t_start(pick_orange_box_1,_place_blue_box_2) * check_parallelism(pick_orange_box_1,place_blue_box_2)
   ] = 0
 qc31: overlapping[pick_orange_box_1,place_blue_box_3] + [
   - min_t_end(pick_orange_box_1,_place_blue_box_3) * check_parallelism(pick_orange_box_1,place_blue_box_3)
   + max_t_start(pick_orange_box_1,_place_blue_box_3) * check_parallelism(pick_orange_box_1,place_blue_box_3)
   ] = 0
 qc32: overlapping[pick_orange_box_1,place_blue_box_4] + [
   - min_t_end(pick_orange_box_1,_place_blue_box_4) * check_parallelism(pick_orange_box_1,place_blue_box_4)
   + max_t_start(pick_orange_box_1,_place_blue_box_4) * check_parallelism(pick_orange_box_1,place_blue_box_4)
   ] = 0
 qc33: overlapping[pick_orange_box_1,place_white_box_1] + [
   - min_t_end(pick_orange_box_1,_place_white_box_1) * check_parallelism(pick_orange_box_1,place_white_box_1)
   + max_t_start(pick_orange_box_1,_place_white_box_1) * check_parallelism(pick_orange_box_1,place_white_box_1)
   ] = 0
 qc34: overlapping[pick_orange_box_1,place_white_box_2] + [
   - min_t_end(pick_orange_box_1,_place_white_box_2) * check_parallelism(pick_orange_box_1,place_white_box_2)
   + max_t_start(pick_orange_box_1,_place_white_box_2) * check_parallelism(pick_orange_box_1,place_white_box_2)
   ] = 0
 qc35: overlapping[pick_orange_box_1,place_white_box_3] + [
   - min_t_end(pick_orange_box_1,_place_white_box_3) * check_parallelism(pick_orange_box_1,place_white_box_3)
   + max_t_start(pick_orange_box_1,_place_white_box_3) * check_parallelism(pick_orange_box_1,place_white_box_3)
   ] = 0
 qc36: overlapping[pick_orange_box_1,place_white_box_4] + [
   - min_t_end(pick_orange_box_1,_place_white_box_4) * check_parallelism(pick_orange_box_1,place_white_box_4)
   + max_t_start(pick_orange_box_1,_place_white_box_4) * check_parallelism(pick_orange_box_1,place_white_box_4)
   ] = 0
 qc37: overlapping[pick_orange_box_2,pick_orange_box_3] + [
   - min_t_end(pick_orange_box_2,_pick_orange_box_3) * check_parallelism(pick_orange_box_2,pick_orange_box_3)
   + max_t_start(pick_orange_box_2,_pick_orange_box_3) * check_parallelism(pick_orange_box_2,pick_orange_box_3)
   ] = 0
 qc38: overlapping[pick_orange_box_2,pick_orange_box_4] + [
   - min_t_end(pick_orange_box_2,_pick_orange_box_4) * check_parallelism(pick_orange_box_2,pick_orange_box_4)
   + max_t_start(pick_orange_box_2,_pick_orange_box_4) * check_parallelism(pick_orange_box_2,pick_orange_box_4)
   ] = 0
 qc39: overlapping[pick_orange_box_2,pick_blue_box_1] + [
   - min_t_end(pick_orange_box_2,_pick_blue_box_1) * check_parallelism(pick_orange_box_2,pick_blue_box_1)
   + max_t_start(pick_orange_box_2,_pick_blue_box_1) * check_parallelism(pick_orange_box_2,pick_blue_box_1)
   ] = 0
 qc40: overlapping[pick_orange_box_2,pick_blue_box_2] + [
   - min_t_end(pick_orange_box_2,_pick_blue_box_2) * check_parallelism(pick_orange_box_2,pick_blue_box_2)
   + max_t_start(pick_orange_box_2,_pick_blue_box_2) * check_parallelism(pick_orange_box_2,pick_blue_box_2)
   ] = 0
 qc41: overlapping[pick_orange_box_2,pick_blue_box_3] + [
   - min_t_end(pick_orange_box_2,_pick_blue_box_3) * check_parallelism(pick_orange_box_2,pick_blue_box_3)
   + max_t_start(pick_orange_box_2,_pick_blue_box_3) * check_parallelism(pick_orange_box_2,pick_blue_box_3)
   ] = 0
 qc42: overlapping[pick_orange_box_2,pick_blue_box_4] + [
   - min_t_end(pick_orange_box_2,_pick_blue_box_4) * check_parallelism(pick_orange_box_2,pick_blue_box_4)
   + max_t_start(pick_orange_box_2,_pick_blue_box_4) * check_parallelism(pick_orange_box_2,pick_blue_box_4)
   ] = 0
 qc43: overlapping[pick_orange_box_2,pick_white_box_1] + [
   - min_t_end(pick_orange_box_2,_pick_white_box_1) * check_parallelism(pick_orange_box_2,pick_white_box_1)
   + max_t_start(pick_orange_box_2,_pick_white_box_1) * check_parallelism(pick_orange_box_2,pick_white_box_1)
   ] = 0
 qc44: overlapping[pick_orange_box_2,pick_white_box_2] + [
   - min_t_end(pick_orange_box_2,_pick_white_box_2) * check_parallelism(pick_orange_box_2,pick_white_box_2)
   + max_t_start(pick_orange_box_2,_pick_white_box_2) * check_parallelism(pick_orange_box_2,pick_white_box_2)
   ] = 0
 qc45: overlapping[pick_orange_box_2,pick_white_box_3] + [
   - min_t_end(pick_orange_box_2,_pick_white_box_3) * check_parallelism(pick_orange_box_2,pick_white_box_3)
   + max_t_start(pick_orange_box_2,_pick_white_box_3) * check_parallelism(pick_orange_box_2,pick_white_box_3)
   ] = 0
 qc46: overlapping[pick_orange_box_2,pick_white_box_4] + [
   - min_t_end(pick_orange_box_2,_pick_white_box_4) * check_parallelism(pick_orange_box_2,pick_white_box_4)
   + max_t_start(pick_orange_box_2,_pick_white_box_4) * check_parallelism(pick_orange_box_2,pick_white_box_4)
   ] = 0
 qc47: overlapping[pick_orange_box_2,place_orange_box_1] + [
   - min_t_end(pick_orange_box_2,_place_orange_box_1) * check_parallelism(pick_orange_box_2,place_orange_box_1)
   + max_t_start(pick_orange_box_2,_place_orange_box_1) * check_parallelism(pick_orange_box_2,place_orange_box_1)
   ] = 0
 qc48: overlapping[pick_orange_box_2,place_orange_box_2] + [
   - min_t_end(pick_orange_box_2,_place_orange_box_2) * check_parallelism(pick_orange_box_2,place_orange_box_2)
   + max_t_start(pick_orange_box_2,_place_orange_box_2) * check_parallelism(pick_orange_box_2,place_orange_box_2)
   ] = 0
 qc49: overlapping[pick_orange_box_2,place_orange_box_3] + [
   - min_t_end(pick_orange_box_2,_place_orange_box_3) * check_parallelism(pick_orange_box_2,place_orange_box_3)
   + max_t_start(pick_orange_box_2,_place_orange_box_3) * check_parallelism(pick_orange_box_2,place_orange_box_3)
   ] = 0
 qc50: overlapping[pick_orange_box_2,place_orange_box_4] + [
   - min_t_end(pick_orange_box_2,_place_orange_box_4) * check_parallelism(pick_orange_box_2,place_orange_box_4)
   + max_t_start(pick_orange_box_2,_place_orange_box_4) * check_parallelism(pick_orange_box_2,place_orange_box_4)
   ] = 0
 qc51: overlapping[pick_orange_box_2,place_blue_box_1] + [
   - min_t_end(pick_orange_box_2,_place_blue_box_1) * check_parallelism(pick_orange_box_2,place_blue_box_1)
   + max_t_start(pick_orange_box_2,_place_blue_box_1) * check_parallelism(pick_orange_box_2,place_blue_box_1)
   ] = 0
 qc52: overlapping[pick_orange_box_2,place_blue_box_2] + [
   - min_t_end(pick_orange_box_2,_place_blue_box_2) * check_parallelism(pick_orange_box_2,place_blue_box_2)
   + max_t_start(pick_orange_box_2,_place_blue_box_2) * check_parallelism(pick_orange_box_2,place_blue_box_2)
   ] = 0
 qc53: overlapping[pick_orange_box_2,place_blue_box_3] + [
   - min_t_end(pick_orange_box_2,_place_blue_box_3) * check_parallelism(pick_orange_box_2,place_blue_box_3)
   + max_t_start(pick_orange_box_2,_place_blue_box_3) * check_parallelism(pick_orange_box_2,place_blue_box_3)
   ] = 0
 qc54: overlapping[pick_orange_box_2,place_blue_box_4] + [
   - min_t_end(pick_orange_box_2,_place_blue_box_4) * check_parallelism(pick_orange_box_2,place_blue_box_4)
   + max_t_start(pick_orange_box_2,_place_blue_box_4) * check_parallelism(pick_orange_box_2,place_blue_box_4)
   ] = 0
 qc55: overlapping[pick_orange_box_2,place_white_box_1] + [
   - min_t_end(pick_orange_box_2,_place_white_box_1) * check_parallelism(pick_orange_box_2,place_white_box_1)
   + max_t_start(pick_orange_box_2,_place_white_box_1) * check_parallelism(pick_orange_box_2,place_white_box_1)
   ] = 0
 qc56: overlapping[pick_orange_box_2,place_white_box_2] + [
   - min_t_end(pick_orange_box_2,_place_white_box_2) * check_parallelism(pick_orange_box_2,place_white_box_2)
   + max_t_start(pick_orange_box_2,_place_white_box_2) * check_parallelism(pick_orange_box_2,place_white_box_2)
   ] = 0
 qc57: overlapping[pick_orange_box_2,place_white_box_3] + [
   - min_t_end(pick_orange_box_2,_place_white_box_3) * check_parallelism(pick_orange_box_2,place_white_box_3)
   + max_t_start(pick_orange_box_2,_place_white_box_3) * check_parallelism(pick_orange_box_2,place_white_box_3)
   ] = 0
 qc58: overlapping[pick_orange_box_2,place_white_box_4] + [
   - min_t_end(pick_orange_box_2,_place_white_box_4) * check_parallelism(pick_orange_box_2,place_white_box_4)
   + max_t_start(pick_orange_box_2,_place_white_box_4) * check_parallelism(pick_orange_box_2,place_white_box_4)
   ] = 0
 qc59: overlapping[pick_orange_box_3,pick_orange_box_4] + [
   - min_t_end(pick_orange_box_3,_pick_orange_box_4) * check_parallelism(pick_orange_box_3,pick_orange_box_4)
   + max_t_start(pick_orange_box_3,_pick_orange_box_4) * check_parallelism(pick_orange_box_3,pick_orange_box_4)
   ] = 0
 qc60: overlapping[pick_orange_box_3,pick_blue_box_1] + [
   - min_t_end(pick_orange_box_3,_pick_blue_box_1) * check_parallelism(pick_orange_box_3,pick_blue_box_1)
   + max_t_start(pick_orange_box_3,_pick_blue_box_1) * check_parallelism(pick_orange_box_3,pick_blue_box_1)
   ] = 0
 qc61: overlapping[pick_orange_box_3,pick_blue_box_2] + [
   - min_t_end(pick_orange_box_3,_pick_blue_box_2) * check_parallelism(pick_orange_box_3,pick_blue_box_2)
   + max_t_start(pick_orange_box_3,_pick_blue_box_2) * check_parallelism(pick_orange_box_3,pick_blue_box_2)
   ] = 0
 qc62: overlapping[pick_orange_box_3,pick_blue_box_3] + [
   - min_t_end(pick_orange_box_3,_pick_blue_box_3) * check_parallelism(pick_orange_box_3,pick_blue_box_3)
   + max_t_start(pick_orange_box_3,_pick_blue_box_3) * check_parallelism(pick_orange_box_3,pick_blue_box_3)
   ] = 0
 qc63: overlapping[pick_orange_box_3,pick_blue_box_4] + [
   - min_t_end(pick_orange_box_3,_pick_blue_box_4) * check_parallelism(pick_orange_box_3,pick_blue_box_4)
   + max_t_start(pick_orange_box_3,_pick_blue_box_4) * check_parallelism(pick_orange_box_3,pick_blue_box_4)
   ] = 0
 qc64: overlapping[pick_orange_box_3,pick_white_box_1] + [
   - min_t_end(pick_orange_box_3,_pick_white_box_1) * check_parallelism(pick_orange_box_3,pick_white_box_1)
   + max_t_start(pick_orange_box_3,_pick_white_box_1) * check_parallelism(pick_orange_box_3,pick_white_box_1)
   ] = 0
 qc65: overlapping[pick_orange_box_3,pick_white_box_2] + [
   - min_t_end(pick_orange_box_3,_pick_white_box_2) * check_parallelism(pick_orange_box_3,pick_white_box_2)
   + max_t_start(pick_orange_box_3,_pick_white_box_2) * check_parallelism(pick_orange_box_3,pick_white_box_2)
   ] = 0
 qc66: overlapping[pick_orange_box_3,pick_white_box_3] + [
   - min_t_end(pick_orange_box_3,_pick_white_box_3) * check_parallelism(pick_orange_box_3,pick_white_box_3)
   + max_t_start(pick_orange_box_3,_pick_white_box_3) * check_parallelism(pick_orange_box_3,pick_white_box_3)
   ] = 0
 qc67: overlapping[pick_orange_box_3,pick_white_box_4] + [
   - min_t_end(pick_orange_box_3,_pick_white_box_4) * check_parallelism(pick_orange_box_3,pick_white_box_4)
   + max_t_start(pick_orange_box_3,_pick_white_box_4) * check_parallelism(pick_orange_box_3,pick_white_box_4)
   ] = 0
 qc68: overlapping[pick_orange_box_3,place_orange_box_1] + [
   - min_t_end(pick_orange_box_3,_place_orange_box_1) * check_parallelism(pick_orange_box_3,place_orange_box_1)
   + max_t_start(pick_orange_box_3,_place_orange_box_1) * check_parallelism(pick_orange_box_3,place_orange_box_1)
   ] = 0
 qc69: overlapping[pick_orange_box_3,place_orange_box_2] + [
   - min_t_end(pick_orange_box_3,_place_orange_box_2) * check_parallelism(pick_orange_box_3,place_orange_box_2)
   + max_t_start(pick_orange_box_3,_place_orange_box_2) * check_parallelism(pick_orange_box_3,place_orange_box_2)
   ] = 0
 qc70: overlapping[pick_orange_box_3,place_orange_box_3] + [
   - min_t_end(pick_orange_box_3,_place_orange_box_3) * check_parallelism(pick_orange_box_3,place_orange_box_3)
   + max_t_start(pick_orange_box_3,_place_orange_box_3) * check_parallelism(pick_orange_box_3,place_orange_box_3)
   ] = 0
 qc71: overlapping[pick_orange_box_3,place_orange_box_4] + [
   - min_t_end(pick_orange_box_3,_place_orange_box_4) * check_parallelism(pick_orange_box_3,place_orange_box_4)
   + max_t_start(pick_orange_box_3,_place_orange_box_4) * check_parallelism(pick_orange_box_3,place_orange_box_4)
   ] = 0
 qc72: overlapping[pick_orange_box_3,place_blue_box_1] + [
   - min_t_end(pick_orange_box_3,_place_blue_box_1) * check_parallelism(pick_orange_box_3,place_blue_box_1)
   + max_t_start(pick_orange_box_3,_place_blue_box_1) * check_parallelism(pick_orange_box_3,place_blue_box_1)
   ] = 0
 qc73: overlapping[pick_orange_box_3,place_blue_box_2] + [
   - min_t_end(pick_orange_box_3,_place_blue_box_2) * check_parallelism(pick_orange_box_3,place_blue_box_2)
   + max_t_start(pick_orange_box_3,_place_blue_box_2) * check_parallelism(pick_orange_box_3,place_blue_box_2)
   ] = 0
 qc74: overlapping[pick_orange_box_3,place_blue_box_3] + [
   - min_t_end(pick_orange_box_3,_place_blue_box_3) * check_parallelism(pick_orange_box_3,place_blue_box_3)
   + max_t_start(pick_orange_box_3,_place_blue_box_3) * check_parallelism(pick_orange_box_3,place_blue_box_3)
   ] = 0
 qc75: overlapping[pick_orange_box_3,place_blue_box_4] + [
   - min_t_end(pick_orange_box_3,_place_blue_box_4) * check_parallelism(pick_orange_box_3,place_blue_box_4)
   + max_t_start(pick_orange_box_3,_place_blue_box_4) * check_parallelism(pick_orange_box_3,place_blue_box_4)
   ] = 0
 qc76: overlapping[pick_orange_box_3,place_white_box_1] + [
   - min_t_end(pick_orange_box_3,_place_white_box_1) * check_parallelism(pick_orange_box_3,place_white_box_1)
   + max_t_start(pick_orange_box_3,_place_white_box_1) * check_parallelism(pick_orange_box_3,place_white_box_1)
   ] = 0
 qc77: overlapping[pick_orange_box_3,place_white_box_2] + [
   - min_t_end(pick_orange_box_3,_place_white_box_2) * check_parallelism(pick_orange_box_3,place_white_box_2)
   + max_t_start(pick_orange_box_3,_place_white_box_2) * check_parallelism(pick_orange_box_3,place_white_box_2)
   ] = 0
 qc78: overlapping[pick_orange_box_3,place_white_box_3] + [
   - min_t_end(pick_orange_box_3,_place_white_box_3) * check_parallelism(pick_orange_box_3,place_white_box_3)
   + max_t_start(pick_orange_box_3,_place_white_box_3) * check_parallelism(pick_orange_box_3,place_white_box_3)
   ] = 0
 qc79: overlapping[pick_orange_box_3,place_white_box_4] + [
   - min_t_end(pick_orange_box_3,_place_white_box_4) * check_parallelism(pick_orange_box_3,place_white_box_4)
   + max_t_start(pick_orange_box_3,_place_white_box_4) * check_parallelism(pick_orange_box_3,place_white_box_4)
   ] = 0
 qc80: overlapping[pick_orange_box_4,pick_blue_box_1] + [
   - min_t_end(pick_orange_box_4,_pick_blue_box_1) * check_parallelism(pick_orange_box_4,pick_blue_box_1)
   + max_t_start(pick_orange_box_4,_pick_blue_box_1) * check_parallelism(pick_orange_box_4,pick_blue_box_1)
   ] = 0
 qc81: overlapping[pick_orange_box_4,pick_blue_box_2] + [
   - min_t_end(pick_orange_box_4,_pick_blue_box_2) * check_parallelism(pick_orange_box_4,pick_blue_box_2)
   + max_t_start(pick_orange_box_4,_pick_blue_box_2) * check_parallelism(pick_orange_box_4,pick_blue_box_2)
   ] = 0
 qc82: overlapping[pick_orange_box_4,pick_blue_box_3] + [
   - min_t_end(pick_orange_box_4,_pick_blue_box_3) * check_parallelism(pick_orange_box_4,pick_blue_box_3)
   + max_t_start(pick_orange_box_4,_pick_blue_box_3) * check_parallelism(pick_orange_box_4,pick_blue_box_3)
   ] = 0
 qc83: overlapping[pick_orange_box_4,pick_blue_box_4] + [
   - min_t_end(pick_orange_box_4,_pick_blue_box_4) * check_parallelism(pick_orange_box_4,pick_blue_box_4)
   + max_t_start(pick_orange_box_4,_pick_blue_box_4) * check_parallelism(pick_orange_box_4,pick_blue_box_4)
   ] = 0
 qc84: overlapping[pick_orange_box_4,pick_white_box_1] + [
   - min_t_end(pick_orange_box_4,_pick_white_box_1) * check_parallelism(pick_orange_box_4,pick_white_box_1)
   + max_t_start(pick_orange_box_4,_pick_white_box_1) * check_parallelism(pick_orange_box_4,pick_white_box_1)
   ] = 0
 qc85: overlapping[pick_orange_box_4,pick_white_box_2] + [
   - min_t_end(pick_orange_box_4,_pick_white_box_2) * check_parallelism(pick_orange_box_4,pick_white_box_2)
   + max_t_start(pick_orange_box_4,_pick_white_box_2) * check_parallelism(pick_orange_box_4,pick_white_box_2)
   ] = 0
 qc86: overlapping[pick_orange_box_4,pick_white_box_3] + [
   - min_t_end(pick_orange_box_4,_pick_white_box_3) * check_parallelism(pick_orange_box_4,pick_white_box_3)
   + max_t_start(pick_orange_box_4,_pick_white_box_3) * check_parallelism(pick_orange_box_4,pick_white_box_3)
   ] = 0
 qc87: overlapping[pick_orange_box_4,pick_white_box_4] + [
   - min_t_end(pick_orange_box_4,_pick_white_box_4) * check_parallelism(pick_orange_box_4,pick_white_box_4)
   + max_t_start(pick_orange_box_4,_pick_white_box_4) * check_parallelism(pick_orange_box_4,pick_white_box_4)
   ] = 0
 qc88: overlapping[pick_orange_box_4,place_orange_box_1] + [
   - min_t_end(pick_orange_box_4,_place_orange_box_1) * check_parallelism(pick_orange_box_4,place_orange_box_1)
   + max_t_start(pick_orange_box_4,_place_orange_box_1) * check_parallelism(pick_orange_box_4,place_orange_box_1)
   ] = 0
 qc89: overlapping[pick_orange_box_4,place_orange_box_2] + [
   - min_t_end(pick_orange_box_4,_place_orange_box_2) * check_parallelism(pick_orange_box_4,place_orange_box_2)
   + max_t_start(pick_orange_box_4,_place_orange_box_2) * check_parallelism(pick_orange_box_4,place_orange_box_2)
   ] = 0
 qc90: overlapping[pick_orange_box_4,place_orange_box_3] + [
   - min_t_end(pick_orange_box_4,_place_orange_box_3) * check_parallelism(pick_orange_box_4,place_orange_box_3)
   + max_t_start(pick_orange_box_4,_place_orange_box_3) * check_parallelism(pick_orange_box_4,place_orange_box_3)
   ] = 0
 qc91: overlapping[pick_orange_box_4,place_orange_box_4] + [
   - min_t_end(pick_orange_box_4,_place_orange_box_4) * check_parallelism(pick_orange_box_4,place_orange_box_4)
   + max_t_start(pick_orange_box_4,_place_orange_box_4) * check_parallelism(pick_orange_box_4,place_orange_box_4)
   ] = 0
 qc92: overlapping[pick_orange_box_4,place_blue_box_1] + [
   - min_t_end(pick_orange_box_4,_place_blue_box_1) * check_parallelism(pick_orange_box_4,place_blue_box_1)
   + max_t_start(pick_orange_box_4,_place_blue_box_1) * check_parallelism(pick_orange_box_4,place_blue_box_1)
   ] = 0
 qc93: overlapping[pick_orange_box_4,place_blue_box_2] + [
   - min_t_end(pick_orange_box_4,_place_blue_box_2) * check_parallelism(pick_orange_box_4,place_blue_box_2)
   + max_t_start(pick_orange_box_4,_place_blue_box_2) * check_parallelism(pick_orange_box_4,place_blue_box_2)
   ] = 0
 qc94: overlapping[pick_orange_box_4,place_blue_box_3] + [
   - min_t_end(pick_orange_box_4,_place_blue_box_3) * check_parallelism(pick_orange_box_4,place_blue_box_3)
   + max_t_start(pick_orange_box_4,_place_blue_box_3) * check_parallelism(pick_orange_box_4,place_blue_box_3)
   ] = 0
 qc95: overlapping[pick_orange_box_4,place_blue_box_4] + [
   - min_t_end(pick_orange_box_4,_place_blue_box_4) * check_parallelism(pick_orange_box_4,place_blue_box_4)
   + max_t_start(pick_orange_box_4,_place_blue_box_4) * check_parallelism(pick_orange_box_4,place_blue_box_4)
   ] = 0
 qc96: overlapping[pick_orange_box_4,place_white_box_1] + [
   - min_t_end(pick_orange_box_4,_place_white_box_1) * check_parallelism(pick_orange_box_4,place_white_box_1)
   + max_t_start(pick_orange_box_4,_place_white_box_1) * check_parallelism(pick_orange_box_4,place_white_box_1)
   ] = 0
 qc97: overlapping[pick_orange_box_4,place_white_box_2] + [
   - min_t_end(pick_orange_box_4,_place_white_box_2) * check_parallelism(pick_orange_box_4,place_white_box_2)
   + max_t_start(pick_orange_box_4,_place_white_box_2) * check_parallelism(pick_orange_box_4,place_white_box_2)
   ] = 0
 qc98: overlapping[pick_orange_box_4,place_white_box_3] + [
   - min_t_end(pick_orange_box_4,_place_white_box_3) * check_parallelism(pick_orange_box_4,place_white_box_3)
   + max_t_start(pick_orange_box_4,_place_white_box_3) * check_parallelism(pick_orange_box_4,place_white_box_3)
   ] = 0
 qc99: overlapping[pick_orange_box_4,place_white_box_4] + [
   - min_t_end(pick_orange_box_4,_place_white_box_4) * check_parallelism(pick_orange_box_4,place_white_box_4)
   + max_t_start(pick_orange_box_4,_place_white_box_4) * check_parallelism(pick_orange_box_4,place_white_box_4)
   ] = 0
 qc100: overlapping[pick_blue_box_1,pick_blue_box_2] + [
   - min_t_end(pick_blue_box_1,_pick_blue_box_2) * check_parallelism(pick_blue_box_1,pick_blue_box_2)
   + max_t_start(pick_blue_box_1,_pick_blue_box_2) * check_parallelism(pick_blue_box_1,pick_blue_box_2)
   ] = 0
 qc101: overlapping[pick_blue_box_1,pick_blue_box_3] + [
   - min_t_end(pick_blue_box_1,_pick_blue_box_3) * check_parallelism(pick_blue_box_1,pick_blue_box_3)
   + max_t_start(pick_blue_box_1,_pick_blue_box_3) * check_parallelism(pick_blue_box_1,pick_blue_box_3)
   ] = 0
 qc102: overlapping[pick_blue_box_1,pick_blue_box_4] + [
   - min_t_end(pick_blue_box_1,_pick_blue_box_4) * check_parallelism(pick_blue_box_1,pick_blue_box_4)
   + max_t_start(pick_blue_box_1,_pick_blue_box_4) * check_parallelism(pick_blue_box_1,pick_blue_box_4)
   ] = 0
 qc103: overlapping[pick_blue_box_1,pick_white_box_1] + [
   - min_t_end(pick_blue_box_1,_pick_white_box_1) * check_parallelism(pick_blue_box_1,pick_white_box_1)
   + max_t_start(pick_blue_box_1,_pick_white_box_1) * check_parallelism(pick_blue_box_1,pick_white_box_1)
   ] = 0
 qc104: overlapping[pick_blue_box_1,pick_white_box_2] + [
   - min_t_end(pick_blue_box_1,_pick_white_box_2) * check_parallelism(pick_blue_box_1,pick_white_box_2)
   + max_t_start(pick_blue_box_1,_pick_white_box_2) * check_parallelism(pick_blue_box_1,pick_white_box_2)
   ] = 0
 qc105: overlapping[pick_blue_box_1,pick_white_box_3] + [
   - min_t_end(pick_blue_box_1,_pick_white_box_3) * check_parallelism(pick_blue_box_1,pick_white_box_3)
   + max_t_start(pick_blue_box_1,_pick_white_box_3) * check_parallelism(pick_blue_box_1,pick_white_box_3)
   ] = 0
 qc106: overlapping[pick_blue_box_1,pick_white_box_4] + [
   - min_t_end(pick_blue_box_1,_pick_white_box_4) * check_parallelism(pick_blue_box_1,pick_white_box_4)
   + max_t_start(pick_blue_box_1,_pick_white_box_4) * check_parallelism(pick_blue_box_1,pick_white_box_4)
   ] = 0
 qc107: overlapping[pick_blue_box_1,place_orange_box_1] + [
   - min_t_end(pick_blue_box_1,_place_orange_box_1) * check_parallelism(pick_blue_box_1,place_orange_box_1)
   + max_t_start(pick_blue_box_1,_place_orange_box_1) * check_parallelism(pick_blue_box_1,place_orange_box_1)
   ] = 0
 qc108: overlapping[pick_blue_box_1,place_orange_box_2] + [
   - min_t_end(pick_blue_box_1,_place_orange_box_2) * check_parallelism(pick_blue_box_1,place_orange_box_2)
   + max_t_start(pick_blue_box_1,_place_orange_box_2) * check_parallelism(pick_blue_box_1,place_orange_box_2)
   ] = 0
 qc109: overlapping[pick_blue_box_1,place_orange_box_3] + [
   - min_t_end(pick_blue_box_1,_place_orange_box_3) * check_parallelism(pick_blue_box_1,place_orange_box_3)
   + max_t_start(pick_blue_box_1,_place_orange_box_3) * check_parallelism(pick_blue_box_1,place_orange_box_3)
   ] = 0
 qc110: overlapping[pick_blue_box_1,place_orange_box_4] + [
   - min_t_end(pick_blue_box_1,_place_orange_box_4) * check_parallelism(pick_blue_box_1,place_orange_box_4)
   + max_t_start(pick_blue_box_1,_place_orange_box_4) * check_parallelism(pick_blue_box_1,place_orange_box_4)
   ] = 0
 qc111: overlapping[pick_blue_box_1,place_blue_box_1] + [
   - min_t_end(pick_blue_box_1,_place_blue_box_1) * check_parallelism(pick_blue_box_1,place_blue_box_1)
   + max_t_start(pick_blue_box_1,_place_blue_box_1) * check_parallelism(pick_blue_box_1,place_blue_box_1)
   ] = 0
 qc112: overlapping[pick_blue_box_1,place_blue_box_2] + [
   - min_t_end(pick_blue_box_1,_place_blue_box_2) * check_parallelism(pick_blue_box_1,place_blue_box_2)
   + max_t_start(pick_blue_box_1,_place_blue_box_2) * check_parallelism(pick_blue_box_1,place_blue_box_2)
   ] = 0
 qc113: overlapping[pick_blue_box_1,place_blue_box_3] + [
   - min_t_end(pick_blue_box_1,_place_blue_box_3) * check_parallelism(pick_blue_box_1,place_blue_box_3)
   + max_t_start(pick_blue_box_1,_place_blue_box_3) * check_parallelism(pick_blue_box_1,place_blue_box_3)
   ] = 0
 qc114: overlapping[pick_blue_box_1,place_blue_box_4] + [
   - min_t_end(pick_blue_box_1,_place_blue_box_4) * check_parallelism(pick_blue_box_1,place_blue_box_4)
   + max_t_start(pick_blue_box_1,_place_blue_box_4) * check_parallelism(pick_blue_box_1,place_blue_box_4)
   ] = 0
 qc115: overlapping[pick_blue_box_1,place_white_box_1] + [
   - min_t_end(pick_blue_box_1,_place_white_box_1) * check_parallelism(pick_blue_box_1,place_white_box_1)
   + max_t_start(pick_blue_box_1,_place_white_box_1) * check_parallelism(pick_blue_box_1,place_white_box_1)
   ] = 0
 qc116: overlapping[pick_blue_box_1,place_white_box_2] + [
   - min_t_end(pick_blue_box_1,_place_white_box_2) * check_parallelism(pick_blue_box_1,place_white_box_2)
   + max_t_start(pick_blue_box_1,_place_white_box_2) * check_parallelism(pick_blue_box_1,place_white_box_2)
   ] = 0
 qc117: overlapping[pick_blue_box_1,place_white_box_3] + [
   - min_t_end(pick_blue_box_1,_place_white_box_3) * check_parallelism(pick_blue_box_1,place_white_box_3)
   + max_t_start(pick_blue_box_1,_place_white_box_3) * check_parallelism(pick_blue_box_1,place_white_box_3)
   ] = 0
 qc118: overlapping[pick_blue_box_1,place_white_box_4] + [
   - min_t_end(pick_blue_box_1,_place_white_box_4) * check_parallelism(pick_blue_box_1,place_white_box_4)
   + max_t_start(pick_blue_box_1,_place_white_box_4) * check_parallelism(pick_blue_box_1,place_white_box_4)
   ] = 0
 qc119: overlapping[pick_blue_box_2,pick_blue_box_3] + [
   - min_t_end(pick_blue_box_2,_pick_blue_box_3) * check_parallelism(pick_blue_box_2,pick_blue_box_3)
   + max_t_start(pick_blue_box_2,_pick_blue_box_3) * check_parallelism(pick_blue_box_2,pick_blue_box_3)
   ] = 0
 qc120: overlapping[pick_blue_box_2,pick_blue_box_4] + [
   - min_t_end(pick_blue_box_2,_pick_blue_box_4) * check_parallelism(pick_blue_box_2,pick_blue_box_4)
   + max_t_start(pick_blue_box_2,_pick_blue_box_4) * check_parallelism(pick_blue_box_2,pick_blue_box_4)
   ] = 0
 qc121: overlapping[pick_blue_box_2,pick_white_box_1] + [
   - min_t_end(pick_blue_box_2,_pick_white_box_1) * check_parallelism(pick_blue_box_2,pick_white_box_1)
   + max_t_start(pick_blue_box_2,_pick_white_box_1) * check_parallelism(pick_blue_box_2,pick_white_box_1)
   ] = 0
 qc122: overlapping[pick_blue_box_2,pick_white_box_2] + [
   - min_t_end(pick_blue_box_2,_pick_white_box_2) * check_parallelism(pick_blue_box_2,pick_white_box_2)
   + max_t_start(pick_blue_box_2,_pick_white_box_2) * check_parallelism(pick_blue_box_2,pick_white_box_2)
   ] = 0
 qc123: overlapping[pick_blue_box_2,pick_white_box_3] + [
   - min_t_end(pick_blue_box_2,_pick_white_box_3) * check_parallelism(pick_blue_box_2,pick_white_box_3)
   + max_t_start(pick_blue_box_2,_pick_white_box_3) * check_parallelism(pick_blue_box_2,pick_white_box_3)
   ] = 0
 qc124: overlapping[pick_blue_box_2,pick_white_box_4] + [
   - min_t_end(pick_blue_box_2,_pick_white_box_4) * check_parallelism(pick_blue_box_2,pick_white_box_4)
   + max_t_start(pick_blue_box_2,_pick_white_box_4) * check_parallelism(pick_blue_box_2,pick_white_box_4)
   ] = 0
 qc125: overlapping[pick_blue_box_2,place_orange_box_1] + [
   - min_t_end(pick_blue_box_2,_place_orange_box_1) * check_parallelism(pick_blue_box_2,place_orange_box_1)
   + max_t_start(pick_blue_box_2,_place_orange_box_1) * check_parallelism(pick_blue_box_2,place_orange_box_1)
   ] = 0
 qc126: overlapping[pick_blue_box_2,place_orange_box_2] + [
   - min_t_end(pick_blue_box_2,_place_orange_box_2) * check_parallelism(pick_blue_box_2,place_orange_box_2)
   + max_t_start(pick_blue_box_2,_place_orange_box_2) * check_parallelism(pick_blue_box_2,place_orange_box_2)
   ] = 0
 qc127: overlapping[pick_blue_box_2,place_orange_box_3] + [
   - min_t_end(pick_blue_box_2,_place_orange_box_3) * check_parallelism(pick_blue_box_2,place_orange_box_3)
   + max_t_start(pick_blue_box_2,_place_orange_box_3) * check_parallelism(pick_blue_box_2,place_orange_box_3)
   ] = 0
 qc128: overlapping[pick_blue_box_2,place_orange_box_4] + [
   - min_t_end(pick_blue_box_2,_place_orange_box_4) * check_parallelism(pick_blue_box_2,place_orange_box_4)
   + max_t_start(pick_blue_box_2,_place_orange_box_4) * check_parallelism(pick_blue_box_2,place_orange_box_4)
   ] = 0
 qc129: overlapping[pick_blue_box_2,place_blue_box_1] + [
   - min_t_end(pick_blue_box_2,_place_blue_box_1) * check_parallelism(pick_blue_box_2,place_blue_box_1)
   + max_t_start(pick_blue_box_2,_place_blue_box_1) * check_parallelism(pick_blue_box_2,place_blue_box_1)
   ] = 0
 qc130: overlapping[pick_blue_box_2,place_blue_box_2] + [
   - min_t_end(pick_blue_box_2,_place_blue_box_2) * check_parallelism(pick_blue_box_2,place_blue_box_2)
   + max_t_start(pick_blue_box_2,_place_blue_box_2) * check_parallelism(pick_blue_box_2,place_blue_box_2)
   ] = 0
 qc131: overlapping[pick_blue_box_2,place_blue_box_3] + [
   - min_t_end(pick_blue_box_2,_place_blue_box_3) * check_parallelism(pick_blue_box_2,place_blue_box_3)
   + max_t_start(pick_blue_box_2,_place_blue_box_3) * check_parallelism(pick_blue_box_2,place_blue_box_3)
   ] = 0
 qc132: overlapping[pick_blue_box_2,place_blue_box_4] + [
   - min_t_end(pick_blue_box_2,_place_blue_box_4) * check_parallelism(pick_blue_box_2,place_blue_box_4)
   + max_t_start(pick_blue_box_2,_place_blue_box_4) * check_parallelism(pick_blue_box_2,place_blue_box_4)
   ] = 0
 qc133: overlapping[pick_blue_box_2,place_white_box_1] + [
   - min_t_end(pick_blue_box_2,_place_white_box_1) * check_parallelism(pick_blue_box_2,place_white_box_1)
   + max_t_start(pick_blue_box_2,_place_white_box_1) * check_parallelism(pick_blue_box_2,place_white_box_1)
   ] = 0
 qc134: overlapping[pick_blue_box_2,place_white_box_2] + [
   - min_t_end(pick_blue_box_2,_place_white_box_2) * check_parallelism(pick_blue_box_2,place_white_box_2)
   + max_t_start(pick_blue_box_2,_place_white_box_2) * check_parallelism(pick_blue_box_2,place_white_box_2)
   ] = 0
 qc135: overlapping[pick_blue_box_2,place_white_box_3] + [
   - min_t_end(pick_blue_box_2,_place_white_box_3) * check_parallelism(pick_blue_box_2,place_white_box_3)
   + max_t_start(pick_blue_box_2,_place_white_box_3) * check_parallelism(pick_blue_box_2,place_white_box_3)
   ] = 0
 qc136: overlapping[pick_blue_box_2,place_white_box_4] + [
   - min_t_end(pick_blue_box_2,_place_white_box_4) * check_parallelism(pick_blue_box_2,place_white_box_4)
   + max_t_start(pick_blue_box_2,_place_white_box_4) * check_parallelism(pick_blue_box_2,place_white_box_4)
   ] = 0
 qc137: overlapping[pick_blue_box_3,pick_blue_box_4] + [
   - min_t_end(pick_blue_box_3,_pick_blue_box_4) * check_parallelism(pick_blue_box_3,pick_blue_box_4)
   + max_t_start(pick_blue_box_3,_pick_blue_box_4) * check_parallelism(pick_blue_box_3,pick_blue_box_4)
   ] = 0
 qc138: overlapping[pick_blue_box_3,pick_white_box_1] + [
   - min_t_end(pick_blue_box_3,_pick_white_box_1) * check_parallelism(pick_blue_box_3,pick_white_box_1)
   + max_t_start(pick_blue_box_3,_pick_white_box_1) * check_parallelism(pick_blue_box_3,pick_white_box_1)
   ] = 0
 qc139: overlapping[pick_blue_box_3,pick_white_box_2] + [
   - min_t_end(pick_blue_box_3,_pick_white_box_2) * check_parallelism(pick_blue_box_3,pick_white_box_2)
   + max_t_start(pick_blue_box_3,_pick_white_box_2) * check_parallelism(pick_blue_box_3,pick_white_box_2)
   ] = 0
 qc140: overlapping[pick_blue_box_3,pick_white_box_3] + [
   - min_t_end(pick_blue_box_3,_pick_white_box_3) * check_parallelism(pick_blue_box_3,pick_white_box_3)
   + max_t_start(pick_blue_box_3,_pick_white_box_3) * check_parallelism(pick_blue_box_3,pick_white_box_3)
   ] = 0
 qc141: overlapping[pick_blue_box_3,pick_white_box_4] + [
   - min_t_end(pick_blue_box_3,_pick_white_box_4) * check_parallelism(pick_blue_box_3,pick_white_box_4)
   + max_t_start(pick_blue_box_3,_pick_white_box_4) * check_parallelism(pick_blue_box_3,pick_white_box_4)
   ] = 0
 qc142: overlapping[pick_blue_box_3,place_orange_box_1] + [
   - min_t_end(pick_blue_box_3,_place_orange_box_1) * check_parallelism(pick_blue_box_3,place_orange_box_1)
   + max_t_start(pick_blue_box_3,_place_orange_box_1) * check_parallelism(pick_blue_box_3,place_orange_box_1)
   ] = 0
 qc143: overlapping[pick_blue_box_3,place_orange_box_2] + [
   - min_t_end(pick_blue_box_3,_place_orange_box_2) * check_parallelism(pick_blue_box_3,place_orange_box_2)
   + max_t_start(pick_blue_box_3,_place_orange_box_2) * check_parallelism(pick_blue_box_3,place_orange_box_2)
   ] = 0
 qc144: overlapping[pick_blue_box_3,place_orange_box_3] + [
   - min_t_end(pick_blue_box_3,_place_orange_box_3) * check_parallelism(pick_blue_box_3,place_orange_box_3)
   + max_t_start(pick_blue_box_3,_place_orange_box_3) * check_parallelism(pick_blue_box_3,place_orange_box_3)
   ] = 0
 qc145: overlapping[pick_blue_box_3,place_orange_box_4] + [
   - min_t_end(pick_blue_box_3,_place_orange_box_4) * check_parallelism(pick_blue_box_3,place_orange_box_4)
   + max_t_start(pick_blue_box_3,_place_orange_box_4) * check_parallelism(pick_blue_box_3,place_orange_box_4)
   ] = 0
 qc146: overlapping[pick_blue_box_3,place_blue_box_1] + [
   - min_t_end(pick_blue_box_3,_place_blue_box_1) * check_parallelism(pick_blue_box_3,place_blue_box_1)
   + max_t_start(pick_blue_box_3,_place_blue_box_1) * check_parallelism(pick_blue_box_3,place_blue_box_1)
   ] = 0
 qc147: overlapping[pick_blue_box_3,place_blue_box_2] + [
   - min_t_end(pick_blue_box_3,_place_blue_box_2) * check_parallelism(pick_blue_box_3,place_blue_box_2)
   + max_t_start(pick_blue_box_3,_place_blue_box_2) * check_parallelism(pick_blue_box_3,place_blue_box_2)
   ] = 0
 qc148: overlapping[pick_blue_box_3,place_blue_box_3] + [
   - min_t_end(pick_blue_box_3,_place_blue_box_3) * check_parallelism(pick_blue_box_3,place_blue_box_3)
   + max_t_start(pick_blue_box_3,_place_blue_box_3) * check_parallelism(pick_blue_box_3,place_blue_box_3)
   ] = 0
 qc149: overlapping[pick_blue_box_3,place_blue_box_4] + [
   - min_t_end(pick_blue_box_3,_place_blue_box_4) * check_parallelism(pick_blue_box_3,place_blue_box_4)
   + max_t_start(pick_blue_box_3,_place_blue_box_4) * check_parallelism(pick_blue_box_3,place_blue_box_4)
   ] = 0
 qc150: overlapping[pick_blue_box_3,place_white_box_1] + [
   - min_t_end(pick_blue_box_3,_place_white_box_1) * check_parallelism(pick_blue_box_3,place_white_box_1)
   + max_t_start(pick_blue_box_3,_place_white_box_1) * check_parallelism(pick_blue_box_3,place_white_box_1)
   ] = 0
 qc151: overlapping[pick_blue_box_3,place_white_box_2] + [
   - min_t_end(pick_blue_box_3,_place_white_box_2) * check_parallelism(pick_blue_box_3,place_white_box_2)
   + max_t_start(pick_blue_box_3,_place_white_box_2) * check_parallelism(pick_blue_box_3,place_white_box_2)
   ] = 0
 qc152: overlapping[pick_blue_box_3,place_white_box_3] + [
   - min_t_end(pick_blue_box_3,_place_white_box_3) * check_parallelism(pick_blue_box_3,place_white_box_3)
   + max_t_start(pick_blue_box_3,_place_white_box_3) * check_parallelism(pick_blue_box_3,place_white_box_3)
   ] = 0
 qc153: overlapping[pick_blue_box_3,place_white_box_4] + [
   - min_t_end(pick_blue_box_3,_place_white_box_4) * check_parallelism(pick_blue_box_3,place_white_box_4)
   + max_t_start(pick_blue_box_3,_place_white_box_4) * check_parallelism(pick_blue_box_3,place_white_box_4)
   ] = 0
 qc154: overlapping[pick_blue_box_4,pick_white_box_1] + [
   - min_t_end(pick_blue_box_4,_pick_white_box_1) * check_parallelism(pick_blue_box_4,pick_white_box_1)
   + max_t_start(pick_blue_box_4,_pick_white_box_1) * check_parallelism(pick_blue_box_4,pick_white_box_1)
   ] = 0
 qc155: overlapping[pick_blue_box_4,pick_white_box_2] + [
   - min_t_end(pick_blue_box_4,_pick_white_box_2) * check_parallelism(pick_blue_box_4,pick_white_box_2)
   + max_t_start(pick_blue_box_4,_pick_white_box_2) * check_parallelism(pick_blue_box_4,pick_white_box_2)
   ] = 0
 qc156: overlapping[pick_blue_box_4,pick_white_box_3] + [
   - min_t_end(pick_blue_box_4,_pick_white_box_3) * check_parallelism(pick_blue_box_4,pick_white_box_3)
   + max_t_start(pick_blue_box_4,_pick_white_box_3) * check_parallelism(pick_blue_box_4,pick_white_box_3)
   ] = 0
 qc157: overlapping[pick_blue_box_4,pick_white_box_4] + [
   - min_t_end(pick_blue_box_4,_pick_white_box_4) * check_parallelism(pick_blue_box_4,pick_white_box_4)
   + max_t_start(pick_blue_box_4,_pick_white_box_4) * check_parallelism(pick_blue_box_4,pick_white_box_4)
   ] = 0
 qc158: overlapping[pick_blue_box_4,place_orange_box_1] + [
   - min_t_end(pick_blue_box_4,_place_orange_box_1) * check_parallelism(pick_blue_box_4,place_orange_box_1)
   + max_t_start(pick_blue_box_4,_place_orange_box_1) * check_parallelism(pick_blue_box_4,place_orange_box_1)
   ] = 0
 qc159: overlapping[pick_blue_box_4,place_orange_box_2] + [
   - min_t_end(pick_blue_box_4,_place_orange_box_2) * check_parallelism(pick_blue_box_4,place_orange_box_2)
   + max_t_start(pick_blue_box_4,_place_orange_box_2) * check_parallelism(pick_blue_box_4,place_orange_box_2)
   ] = 0
 qc160: overlapping[pick_blue_box_4,place_orange_box_3] + [
   - min_t_end(pick_blue_box_4,_place_orange_box_3) * check_parallelism(pick_blue_box_4,place_orange_box_3)
   + max_t_start(pick_blue_box_4,_place_orange_box_3) * check_parallelism(pick_blue_box_4,place_orange_box_3)
   ] = 0
 qc161: overlapping[pick_blue_box_4,place_orange_box_4] + [
   - min_t_end(pick_blue_box_4,_place_orange_box_4) * check_parallelism(pick_blue_box_4,place_orange_box_4)
   + max_t_start(pick_blue_box_4,_place_orange_box_4) * check_parallelism(pick_blue_box_4,place_orange_box_4)
   ] = 0
 qc162: overlapping[pick_blue_box_4,place_blue_box_1] + [
   - min_t_end(pick_blue_box_4,_place_blue_box_1) * check_parallelism(pick_blue_box_4,place_blue_box_1)
   + max_t_start(pick_blue_box_4,_place_blue_box_1) * check_parallelism(pick_blue_box_4,place_blue_box_1)
   ] = 0
 qc163: overlapping[pick_blue_box_4,place_blue_box_2] + [
   - min_t_end(pick_blue_box_4,_place_blue_box_2) * check_parallelism(pick_blue_box_4,place_blue_box_2)
   + max_t_start(pick_blue_box_4,_place_blue_box_2) * check_parallelism(pick_blue_box_4,place_blue_box_2)
   ] = 0
 qc164: overlapping[pick_blue_box_4,place_blue_box_3] + [
   - min_t_end(pick_blue_box_4,_place_blue_box_3) * check_parallelism(pick_blue_box_4,place_blue_box_3)
   + max_t_start(pick_blue_box_4,_place_blue_box_3) * check_parallelism(pick_blue_box_4,place_blue_box_3)
   ] = 0
 qc165: overlapping[pick_blue_box_4,place_blue_box_4] + [
   - min_t_end(pick_blue_box_4,_place_blue_box_4) * check_parallelism(pick_blue_box_4,place_blue_box_4)
   + max_t_start(pick_blue_box_4,_place_blue_box_4) * check_parallelism(pick_blue_box_4,place_blue_box_4)
   ] = 0
 qc166: overlapping[pick_blue_box_4,place_white_box_1] + [
   - min_t_end(pick_blue_box_4,_place_white_box_1) * check_parallelism(pick_blue_box_4,place_white_box_1)
   + max_t_start(pick_blue_box_4,_place_white_box_1) * check_parallelism(pick_blue_box_4,place_white_box_1)
   ] = 0
 qc167: overlapping[pick_blue_box_4,place_white_box_2] + [
   - min_t_end(pick_blue_box_4,_place_white_box_2) * check_parallelism(pick_blue_box_4,place_white_box_2)
   + max_t_start(pick_blue_box_4,_place_white_box_2) * check_parallelism(pick_blue_box_4,place_white_box_2)
   ] = 0
 qc168: overlapping[pick_blue_box_4,place_white_box_3] + [
   - min_t_end(pick_blue_box_4,_place_white_box_3) * check_parallelism(pick_blue_box_4,place_white_box_3)
   + max_t_start(pick_blue_box_4,_place_white_box_3) * check_parallelism(pick_blue_box_4,place_white_box_3)
   ] = 0
 qc169: overlapping[pick_blue_box_4,place_white_box_4] + [
   - min_t_end(pick_blue_box_4,_place_white_box_4) * check_parallelism(pick_blue_box_4,place_white_box_4)
   + max_t_start(pick_blue_box_4,_place_white_box_4) * check_parallelism(pick_blue_box_4,place_white_box_4)
   ] = 0
 qc170: overlapping[pick_white_box_1,pick_white_box_2] + [
   - min_t_end(pick_white_box_1,_pick_white_box_2) * check_parallelism(pick_white_box_1,pick_white_box_2)
   + max_t_start(pick_white_box_1,_pick_white_box_2) * check_parallelism(pick_white_box_1,pick_white_box_2)
   ] = 0
 qc171: overlapping[pick_white_box_1,pick_white_box_3] + [
   - min_t_end(pick_white_box_1,_pick_white_box_3) * check_parallelism(pick_white_box_1,pick_white_box_3)
   + max_t_start(pick_white_box_1,_pick_white_box_3) * check_parallelism(pick_white_box_1,pick_white_box_3)
   ] = 0
 qc172: overlapping[pick_white_box_1,pick_white_box_4] + [
   - min_t_end(pick_white_box_1,_pick_white_box_4) * check_parallelism(pick_white_box_1,pick_white_box_4)
   + max_t_start(pick_white_box_1,_pick_white_box_4) * check_parallelism(pick_white_box_1,pick_white_box_4)
   ] = 0
 qc173: overlapping[pick_white_box_1,place_orange_box_1] + [
   - min_t_end(pick_white_box_1,_place_orange_box_1) * check_parallelism(pick_white_box_1,place_orange_box_1)
   + max_t_start(pick_white_box_1,_place_orange_box_1) * check_parallelism(pick_white_box_1,place_orange_box_1)
   ] = 0
 qc174: overlapping[pick_white_box_1,place_orange_box_2] + [
   - min_t_end(pick_white_box_1,_place_orange_box_2) * check_parallelism(pick_white_box_1,place_orange_box_2)
   + max_t_start(pick_white_box_1,_place_orange_box_2) * check_parallelism(pick_white_box_1,place_orange_box_2)
   ] = 0
 qc175: overlapping[pick_white_box_1,place_orange_box_3] + [
   - min_t_end(pick_white_box_1,_place_orange_box_3) * check_parallelism(pick_white_box_1,place_orange_box_3)
   + max_t_start(pick_white_box_1,_place_orange_box_3) * check_parallelism(pick_white_box_1,place_orange_box_3)
   ] = 0
 qc176: overlapping[pick_white_box_1,place_orange_box_4] + [
   - min_t_end(pick_white_box_1,_place_orange_box_4) * check_parallelism(pick_white_box_1,place_orange_box_4)
   + max_t_start(pick_white_box_1,_place_orange_box_4) * check_parallelism(pick_white_box_1,place_orange_box_4)
   ] = 0
 qc177: overlapping[pick_white_box_1,place_blue_box_1] + [
   - min_t_end(pick_white_box_1,_place_blue_box_1) * check_parallelism(pick_white_box_1,place_blue_box_1)
   + max_t_start(pick_white_box_1,_place_blue_box_1) * check_parallelism(pick_white_box_1,place_blue_box_1)
   ] = 0
 qc178: overlapping[pick_white_box_1,place_blue_box_2] + [
   - min_t_end(pick_white_box_1,_place_blue_box_2) * check_parallelism(pick_white_box_1,place_blue_box_2)
   + max_t_start(pick_white_box_1,_place_blue_box_2) * check_parallelism(pick_white_box_1,place_blue_box_2)
   ] = 0
 qc179: overlapping[pick_white_box_1,place_blue_box_3] + [
   - min_t_end(pick_white_box_1,_place_blue_box_3) * check_parallelism(pick_white_box_1,place_blue_box_3)
   + max_t_start(pick_white_box_1,_place_blue_box_3) * check_parallelism(pick_white_box_1,place_blue_box_3)
   ] = 0
 qc180: overlapping[pick_white_box_1,place_blue_box_4] + [
   - min_t_end(pick_white_box_1,_place_blue_box_4) * check_parallelism(pick_white_box_1,place_blue_box_4)
   + max_t_start(pick_white_box_1,_place_blue_box_4) * check_parallelism(pick_white_box_1,place_blue_box_4)
   ] = 0
 qc181: overlapping[pick_white_box_1,place_white_box_1] + [
   - min_t_end(pick_white_box_1,_place_white_box_1) * check_parallelism(pick_white_box_1,place_white_box_1)
   + max_t_start(pick_white_box_1,_place_white_box_1) * check_parallelism(pick_white_box_1,place_white_box_1)
   ] = 0
 qc182: overlapping[pick_white_box_1,place_white_box_2] + [
   - min_t_end(pick_white_box_1,_place_white_box_2) * check_parallelism(pick_white_box_1,place_white_box_2)
   + max_t_start(pick_white_box_1,_place_white_box_2) * check_parallelism(pick_white_box_1,place_white_box_2)
   ] = 0
 qc183: overlapping[pick_white_box_1,place_white_box_3] + [
   - min_t_end(pick_white_box_1,_place_white_box_3) * check_parallelism(pick_white_box_1,place_white_box_3)
   + max_t_start(pick_white_box_1,_place_white_box_3) * check_parallelism(pick_white_box_1,place_white_box_3)
   ] = 0
 qc184: overlapping[pick_white_box_1,place_white_box_4] + [
   - min_t_end(pick_white_box_1,_place_white_box_4) * check_parallelism(pick_white_box_1,place_white_box_4)
   + max_t_start(pick_white_box_1,_place_white_box_4) * check_parallelism(pick_white_box_1,place_white_box_4)
   ] = 0
 qc185: overlapping[pick_white_box_2,pick_white_box_3] + [
   - min_t_end(pick_white_box_2,_pick_white_box_3) * check_parallelism(pick_white_box_2,pick_white_box_3)
   + max_t_start(pick_white_box_2,_pick_white_box_3) * check_parallelism(pick_white_box_2,pick_white_box_3)
   ] = 0
 qc186: overlapping[pick_white_box_2,pick_white_box_4] + [
   - min_t_end(pick_white_box_2,_pick_white_box_4) * check_parallelism(pick_white_box_2,pick_white_box_4)
   + max_t_start(pick_white_box_2,_pick_white_box_4) * check_parallelism(pick_white_box_2,pick_white_box_4)
   ] = 0
 qc187: overlapping[pick_white_box_2,place_orange_box_1] + [
   - min_t_end(pick_white_box_2,_place_orange_box_1) * check_parallelism(pick_white_box_2,place_orange_box_1)
   + max_t_start(pick_white_box_2,_place_orange_box_1) * check_parallelism(pick_white_box_2,place_orange_box_1)
   ] = 0
 qc188: overlapping[pick_white_box_2,place_orange_box_2] + [
   - min_t_end(pick_white_box_2,_place_orange_box_2) * check_parallelism(pick_white_box_2,place_orange_box_2)
   + max_t_start(pick_white_box_2,_place_orange_box_2) * check_parallelism(pick_white_box_2,place_orange_box_2)
   ] = 0
 qc189: overlapping[pick_white_box_2,place_orange_box_3] + [
   - min_t_end(pick_white_box_2,_place_orange_box_3) * check_parallelism(pick_white_box_2,place_orange_box_3)
   + max_t_start(pick_white_box_2,_place_orange_box_3) * check_parallelism(pick_white_box_2,place_orange_box_3)
   ] = 0
 qc190: overlapping[pick_white_box_2,place_orange_box_4] + [
   - min_t_end(pick_white_box_2,_place_orange_box_4) * check_parallelism(pick_white_box_2,place_orange_box_4)
   + max_t_start(pick_white_box_2,_place_orange_box_4) * check_parallelism(pick_white_box_2,place_orange_box_4)
   ] = 0
 qc191: overlapping[pick_white_box_2,place_blue_box_1] + [
   - min_t_end(pick_white_box_2,_place_blue_box_1) * check_parallelism(pick_white_box_2,place_blue_box_1)
   + max_t_start(pick_white_box_2,_place_blue_box_1) * check_parallelism(pick_white_box_2,place_blue_box_1)
   ] = 0
 qc192: overlapping[pick_white_box_2,place_blue_box_2] + [
   - min_t_end(pick_white_box_2,_place_blue_box_2) * check_parallelism(pick_white_box_2,place_blue_box_2)
   + max_t_start(pick_white_box_2,_place_blue_box_2) * check_parallelism(pick_white_box_2,place_blue_box_2)
   ] = 0
 qc193: overlapping[pick_white_box_2,place_blue_box_3] + [
   - min_t_end(pick_white_box_2,_place_blue_box_3) * check_parallelism(pick_white_box_2,place_blue_box_3)
   + max_t_start(pick_white_box_2,_place_blue_box_3) * check_parallelism(pick_white_box_2,place_blue_box_3)
   ] = 0
 qc194: overlapping[pick_white_box_2,place_blue_box_4] + [
   - min_t_end(pick_white_box_2,_place_blue_box_4) * check_parallelism(pick_white_box_2,place_blue_box_4)
   + max_t_start(pick_white_box_2,_place_blue_box_4) * check_parallelism(pick_white_box_2,place_blue_box_4)
   ] = 0
 qc195: overlapping[pick_white_box_2,place_white_box_1] + [
   - min_t_end(pick_white_box_2,_place_white_box_1) * check_parallelism(pick_white_box_2,place_white_box_1)
   + max_t_start(pick_white_box_2,_place_white_box_1) * check_parallelism(pick_white_box_2,place_white_box_1)
   ] = 0
 qc196: overlapping[pick_white_box_2,place_white_box_2] + [
   - min_t_end(pick_white_box_2,_place_white_box_2) * check_parallelism(pick_white_box_2,place_white_box_2)
   + max_t_start(pick_white_box_2,_place_white_box_2) * check_parallelism(pick_white_box_2,place_white_box_2)
   ] = 0
 qc197: overlapping[pick_white_box_2,place_white_box_3] + [
   - min_t_end(pick_white_box_2,_place_white_box_3) * check_parallelism(pick_white_box_2,place_white_box_3)
   + max_t_start(pick_white_box_2,_place_white_box_3) * check_parallelism(pick_white_box_2,place_white_box_3)
   ] = 0
 qc198: overlapping[pick_white_box_2,place_white_box_4] + [
   - min_t_end(pick_white_box_2,_place_white_box_4) * check_parallelism(pick_white_box_2,place_white_box_4)
   + max_t_start(pick_white_box_2,_place_white_box_4) * check_parallelism(pick_white_box_2,place_white_box_4)
   ] = 0
 qc199: overlapping[pick_white_box_3,pick_white_box_4] + [
   - min_t_end(pick_white_box_3,_pick_white_box_4) * check_parallelism(pick_white_box_3,pick_white_box_4)
   + max_t_start(pick_white_box_3,_pick_white_box_4) * check_parallelism(pick_white_box_3,pick_white_box_4)
   ] = 0
 qc200: overlapping[pick_white_box_3,place_orange_box_1] + [
   - min_t_end(pick_white_box_3,_place_orange_box_1) * check_parallelism(pick_white_box_3,place_orange_box_1)
   + max_t_start(pick_white_box_3,_place_orange_box_1) * check_parallelism(pick_white_box_3,place_orange_box_1)
   ] = 0
 qc201: overlapping[pick_white_box_3,place_orange_box_2] + [
   - min_t_end(pick_white_box_3,_place_orange_box_2) * check_parallelism(pick_white_box_3,place_orange_box_2)
   + max_t_start(pick_white_box_3,_place_orange_box_2) * check_parallelism(pick_white_box_3,place_orange_box_2)
   ] = 0
 qc202: overlapping[pick_white_box_3,place_orange_box_3] + [
   - min_t_end(pick_white_box_3,_place_orange_box_3) * check_parallelism(pick_white_box_3,place_orange_box_3)
   + max_t_start(pick_white_box_3,_place_orange_box_3) * check_parallelism(pick_white_box_3,place_orange_box_3)
   ] = 0
 qc203: overlapping[pick_white_box_3,place_orange_box_4] + [
   - min_t_end(pick_white_box_3,_place_orange_box_4) * check_parallelism(pick_white_box_3,place_orange_box_4)
   + max_t_start(pick_white_box_3,_place_orange_box_4) * check_parallelism(pick_white_box_3,place_orange_box_4)
   ] = 0
 qc204: overlapping[pick_white_box_3,place_blue_box_1] + [
   - min_t_end(pick_white_box_3,_place_blue_box_1) * check_parallelism(pick_white_box_3,place_blue_box_1)
   + max_t_start(pick_white_box_3,_place_blue_box_1) * check_parallelism(pick_white_box_3,place_blue_box_1)
   ] = 0
 qc205: overlapping[pick_white_box_3,place_blue_box_2] + [
   - min_t_end(pick_white_box_3,_place_blue_box_2) * check_parallelism(pick_white_box_3,place_blue_box_2)
   + max_t_start(pick_white_box_3,_place_blue_box_2) * check_parallelism(pick_white_box_3,place_blue_box_2)
   ] = 0
 qc206: overlapping[pick_white_box_3,place_blue_box_3] + [
   - min_t_end(pick_white_box_3,_place_blue_box_3) * check_parallelism(pick_white_box_3,place_blue_box_3)
   + max_t_start(pick_white_box_3,_place_blue_box_3) * check_parallelism(pick_white_box_3,place_blue_box_3)
   ] = 0
 qc207: overlapping[pick_white_box_3,place_blue_box_4] + [
   - min_t_end(pick_white_box_3,_place_blue_box_4) * check_parallelism(pick_white_box_3,place_blue_box_4)
   + max_t_start(pick_white_box_3,_place_blue_box_4) * check_parallelism(pick_white_box_3,place_blue_box_4)
   ] = 0
 qc208: overlapping[pick_white_box_3,place_white_box_1] + [
   - min_t_end(pick_white_box_3,_place_white_box_1) * check_parallelism(pick_white_box_3,place_white_box_1)
   + max_t_start(pick_white_box_3,_place_white_box_1) * check_parallelism(pick_white_box_3,place_white_box_1)
   ] = 0
 qc209: overlapping[pick_white_box_3,place_white_box_2] + [
   - min_t_end(pick_white_box_3,_place_white_box_2) * check_parallelism(pick_white_box_3,place_white_box_2)
   + max_t_start(pick_white_box_3,_place_white_box_2) * check_parallelism(pick_white_box_3,place_white_box_2)
   ] = 0
 qc210: overlapping[pick_white_box_3,place_white_box_3] + [
   - min_t_end(pick_white_box_3,_place_white_box_3) * check_parallelism(pick_white_box_3,place_white_box_3)
   + max_t_start(pick_white_box_3,_place_white_box_3) * check_parallelism(pick_white_box_3,place_white_box_3)
   ] = 0
 qc211: overlapping[pick_white_box_3,place_white_box_4] + [
   - min_t_end(pick_white_box_3,_place_white_box_4) * check_parallelism(pick_white_box_3,place_white_box_4)
   + max_t_start(pick_white_box_3,_place_white_box_4) * check_parallelism(pick_white_box_3,place_white_box_4)
   ] = 0
 qc212: overlapping[pick_white_box_4,place_orange_box_1] + [
   - min_t_end(pick_white_box_4,_place_orange_box_1) * check_parallelism(pick_white_box_4,place_orange_box_1)
   + max_t_start(pick_white_box_4,_place_orange_box_1) * check_parallelism(pick_white_box_4,place_orange_box_1)
   ] = 0
 qc213: overlapping[pick_white_box_4,place_orange_box_2] + [
   - min_t_end(pick_white_box_4,_place_orange_box_2) * check_parallelism(pick_white_box_4,place_orange_box_2)
   + max_t_start(pick_white_box_4,_place_orange_box_2) * check_parallelism(pick_white_box_4,place_orange_box_2)
   ] = 0
 qc214: overlapping[pick_white_box_4,place_orange_box_3] + [
   - min_t_end(pick_white_box_4,_place_orange_box_3) * check_parallelism(pick_white_box_4,place_orange_box_3)
   + max_t_start(pick_white_box_4,_place_orange_box_3) * check_parallelism(pick_white_box_4,place_orange_box_3)
   ] = 0
 qc215: overlapping[pick_white_box_4,place_orange_box_4] + [
   - min_t_end(pick_white_box_4,_place_orange_box_4) * check_parallelism(pick_white_box_4,place_orange_box_4)
   + max_t_start(pick_white_box_4,_place_orange_box_4) * check_parallelism(pick_white_box_4,place_orange_box_4)
   ] = 0
 qc216: overlapping[pick_white_box_4,place_blue_box_1] + [
   - min_t_end(pick_white_box_4,_place_blue_box_1) * check_parallelism(pick_white_box_4,place_blue_box_1)
   + max_t_start(pick_white_box_4,_place_blue_box_1) * check_parallelism(pick_white_box_4,place_blue_box_1)
   ] = 0
 qc217: overlapping[pick_white_box_4,place_blue_box_2] + [
   - min_t_end(pick_white_box_4,_place_blue_box_2) * check_parallelism(pick_white_box_4,place_blue_box_2)
   + max_t_start(pick_white_box_4,_place_blue_box_2) * check_parallelism(pick_white_box_4,place_blue_box_2)
   ] = 0
 qc218: overlapping[pick_white_box_4,place_blue_box_3] + [
   - min_t_end(pick_white_box_4,_place_blue_box_3) * check_parallelism(pick_white_box_4,place_blue_box_3)
   + max_t_start(pick_white_box_4,_place_blue_box_3) * check_parallelism(pick_white_box_4,place_blue_box_3)
   ] = 0
 qc219: overlapping[pick_white_box_4,place_blue_box_4] + [
   - min_t_end(pick_white_box_4,_place_blue_box_4) * check_parallelism(pick_white_box_4,place_blue_box_4)
   + max_t_start(pick_white_box_4,_place_blue_box_4) * check_parallelism(pick_white_box_4,place_blue_box_4)
   ] = 0
 qc220: overlapping[pick_white_box_4,place_white_box_1] + [
   - min_t_end(pick_white_box_4,_place_white_box_1) * check_parallelism(pick_white_box_4,place_white_box_1)
   + max_t_start(pick_white_box_4,_place_white_box_1) * check_parallelism(pick_white_box_4,place_white_box_1)
   ] = 0
 qc221: overlapping[pick_white_box_4,place_white_box_2] + [
   - min_t_end(pick_white_box_4,_place_white_box_2) * check_parallelism(pick_white_box_4,place_white_box_2)
   + max_t_start(pick_white_box_4,_place_white_box_2) * check_parallelism(pick_white_box_4,place_white_box_2)
   ] = 0
 qc222: overlapping[pick_white_box_4,place_white_box_3] + [
   - min_t_end(pick_white_box_4,_place_white_box_3) * check_parallelism(pick_white_box_4,place_white_box_3)
   + max_t_start(pick_white_box_4,_place_white_box_3) * check_parallelism(pick_white_box_4,place_white_box_3)
   ] = 0
 qc223: overlapping[pick_white_box_4,place_white_box_4] + [
   - min_t_end(pick_white_box_4,_place_white_box_4) * check_parallelism(pick_white_box_4,place_white_box_4)
   + max_t_start(pick_white_box_4,_place_white_box_4) * check_parallelism(pick_white_box_4,place_white_box_4)
   ] = 0
 qc224: overlapping[place_orange_box_1,place_orange_box_2] + [
   - min_t_end(place_orange_box_1,_place_orange_box_2) * check_parallelism(place_orange_box_1,place_orange_box_2)
   + max_t_start(place_orange_box_1,_place_orange_box_2) * check_parallelism(place_orange_box_1,place_orange_box_2)
   ] = 0
 qc225: overlapping[place_orange_box_1,place_orange_box_3] + [
   - min_t_end(place_orange_box_1,_place_orange_box_3) * check_parallelism(place_orange_box_1,place_orange_box_3)
   + max_t_start(place_orange_box_1,_place_orange_box_3) * check_parallelism(place_orange_box_1,place_orange_box_3)
   ] = 0
 qc226: overlapping[place_orange_box_1,place_orange_box_4] + [
   - min_t_end(place_orange_box_1,_place_orange_box_4) * check_parallelism(place_orange_box_1,place_orange_box_4)
   + max_t_start(place_orange_box_1,_place_orange_box_4) * check_parallelism(place_orange_box_1,place_orange_box_4)
   ] = 0
 qc227: overlapping[place_orange_box_1,place_blue_box_1] + [
   - min_t_end(place_orange_box_1,_place_blue_box_1) * check_parallelism(place_orange_box_1,place_blue_box_1)
   + max_t_start(place_orange_box_1,_place_blue_box_1) * check_parallelism(place_orange_box_1,place_blue_box_1)
   ] = 0
 qc228: overlapping[place_orange_box_1,place_blue_box_2] + [
   - min_t_end(place_orange_box_1,_place_blue_box_2) * check_parallelism(place_orange_box_1,place_blue_box_2)
   + max_t_start(place_orange_box_1,_place_blue_box_2) * check_parallelism(place_orange_box_1,place_blue_box_2)
   ] = 0
 qc229: overlapping[place_orange_box_1,place_blue_box_3] + [
   - min_t_end(place_orange_box_1,_place_blue_box_3) * check_parallelism(place_orange_box_1,place_blue_box_3)
   + max_t_start(place_orange_box_1,_place_blue_box_3) * check_parallelism(place_orange_box_1,place_blue_box_3)
   ] = 0
 qc230: overlapping[place_orange_box_1,place_blue_box_4] + [
   - min_t_end(place_orange_box_1,_place_blue_box_4) * check_parallelism(place_orange_box_1,place_blue_box_4)
   + max_t_start(place_orange_box_1,_place_blue_box_4) * check_parallelism(place_orange_box_1,place_blue_box_4)
   ] = 0
 qc231: overlapping[place_orange_box_1,place_white_box_1] + [
   - min_t_end(place_orange_box_1,_place_white_box_1) * check_parallelism(place_orange_box_1,place_white_box_1)
   + max_t_start(place_orange_box_1,_place_white_box_1) * check_parallelism(place_orange_box_1,place_white_box_1)
   ] = 0
 qc232: overlapping[place_orange_box_1,place_white_box_2] + [
   - min_t_end(place_orange_box_1,_place_white_box_2) * check_parallelism(place_orange_box_1,place_white_box_2)
   + max_t_start(place_orange_box_1,_place_white_box_2) * check_parallelism(place_orange_box_1,place_white_box_2)
   ] = 0
 qc233: overlapping[place_orange_box_1,place_white_box_3] + [
   - min_t_end(place_orange_box_1,_place_white_box_3) * check_parallelism(place_orange_box_1,place_white_box_3)
   + max_t_start(place_orange_box_1,_place_white_box_3) * check_parallelism(place_orange_box_1,place_white_box_3)
   ] = 0
 qc234: overlapping[place_orange_box_1,place_white_box_4] + [
   - min_t_end(place_orange_box_1,_place_white_box_4) * check_parallelism(place_orange_box_1,place_white_box_4)
   + max_t_start(place_orange_box_1,_place_white_box_4) * check_parallelism(place_orange_box_1,place_white_box_4)
   ] = 0
 qc235: overlapping[place_orange_box_2,place_orange_box_3] + [
   - min_t_end(place_orange_box_2,_place_orange_box_3) * check_parallelism(place_orange_box_2,place_orange_box_3)
   + max_t_start(place_orange_box_2,_place_orange_box_3) * check_parallelism(place_orange_box_2,place_orange_box_3)
   ] = 0
 qc236: overlapping[place_orange_box_2,place_orange_box_4] + [
   - min_t_end(place_orange_box_2,_place_orange_box_4) * check_parallelism(place_orange_box_2,place_orange_box_4)
   + max_t_start(place_orange_box_2,_place_orange_box_4) * check_parallelism(place_orange_box_2,place_orange_box_4)
   ] = 0
 qc237: overlapping[place_orange_box_2,place_blue_box_1] + [
   - min_t_end(place_orange_box_2,_place_blue_box_1) * check_parallelism(place_orange_box_2,place_blue_box_1)
   + max_t_start(place_orange_box_2,_place_blue_box_1) * check_parallelism(place_orange_box_2,place_blue_box_1)
   ] = 0
 qc238: overlapping[place_orange_box_2,place_blue_box_2] + [
   - min_t_end(place_orange_box_2,_place_blue_box_2) * check_parallelism(place_orange_box_2,place_blue_box_2)
   + max_t_start(place_orange_box_2,_place_blue_box_2) * check_parallelism(place_orange_box_2,place_blue_box_2)
   ] = 0
 qc239: overlapping[place_orange_box_2,place_blue_box_3] + [
   - min_t_end(place_orange_box_2,_place_blue_box_3) * check_parallelism(place_orange_box_2,place_blue_box_3)
   + max_t_start(place_orange_box_2,_place_blue_box_3) * check_parallelism(place_orange_box_2,place_blue_box_3)
   ] = 0
 qc240: overlapping[place_orange_box_2,place_blue_box_4] + [
   - min_t_end(place_orange_box_2,_place_blue_box_4) * check_parallelism(place_orange_box_2,place_blue_box_4)
   + max_t_start(place_orange_box_2,_place_blue_box_4) * check_parallelism(place_orange_box_2,place_blue_box_4)
   ] = 0
 qc241: overlapping[place_orange_box_2,place_white_box_1] + [
   - min_t_end(place_orange_box_2,_place_white_box_1) * check_parallelism(place_orange_box_2,place_white_box_1)
   + max_t_start(place_orange_box_2,_place_white_box_1) * check_parallelism(place_orange_box_2,place_white_box_1)
   ] = 0
 qc242: overlapping[place_orange_box_2,place_white_box_2] + [
   - min_t_end(place_orange_box_2,_place_white_box_2) * check_parallelism(place_orange_box_2,place_white_box_2)
   + max_t_start(place_orange_box_2,_place_white_box_2) * check_parallelism(place_orange_box_2,place_white_box_2)
   ] = 0
 qc243: overlapping[place_orange_box_2,place_white_box_3] + [
   - min_t_end(place_orange_box_2,_place_white_box_3) * check_parallelism(place_orange_box_2,place_white_box_3)
   + max_t_start(place_orange_box_2,_place_white_box_3) * check_parallelism(place_orange_box_2,place_white_box_3)
   ] = 0
 qc244: overlapping[place_orange_box_2,place_white_box_4] + [
   - min_t_end(place_orange_box_2,_place_white_box_4) * check_parallelism(place_orange_box_2,place_white_box_4)
   + max_t_start(place_orange_box_2,_place_white_box_4) * check_parallelism(place_orange_box_2,place_white_box_4)
   ] = 0
 qc245: overlapping[place_orange_box_3,place_orange_box_4] + [
   - min_t_end(place_orange_box_3,_place_orange_box_4) * check_parallelism(place_orange_box_3,place_orange_box_4)
   + max_t_start(place_orange_box_3,_place_orange_box_4) * check_parallelism(place_orange_box_3,place_orange_box_4)
   ] = 0
 qc246: overlapping[place_orange_box_3,place_blue_box_1] + [
   - min_t_end(place_orange_box_3,_place_blue_box_1) * check_parallelism(place_orange_box_3,place_blue_box_1)
   + max_t_start(place_orange_box_3,_place_blue_box_1) * check_parallelism(place_orange_box_3,place_blue_box_1)
   ] = 0
 qc247: overlapping[place_orange_box_3,place_blue_box_2] + [
   - min_t_end(place_orange_box_3,_place_blue_box_2) * check_parallelism(place_orange_box_3,place_blue_box_2)
   + max_t_start(place_orange_box_3,_place_blue_box_2) * check_parallelism(place_orange_box_3,place_blue_box_2)
   ] = 0
 qc248: overlapping[place_orange_box_3,place_blue_box_3] + [
   - min_t_end(place_orange_box_3,_place_blue_box_3) * check_parallelism(place_orange_box_3,place_blue_box_3)
   + max_t_start(place_orange_box_3,_place_blue_box_3) * check_parallelism(place_orange_box_3,place_blue_box_3)
   ] = 0
 qc249: overlapping[place_orange_box_3,place_blue_box_4] + [
   - min_t_end(place_orange_box_3,_place_blue_box_4) * check_parallelism(place_orange_box_3,place_blue_box_4)
   + max_t_start(place_orange_box_3,_place_blue_box_4) * check_parallelism(place_orange_box_3,place_blue_box_4)
   ] = 0
 qc250: overlapping[place_orange_box_3,place_white_box_1] + [
   - min_t_end(place_orange_box_3,_place_white_box_1) * check_parallelism(place_orange_box_3,place_white_box_1)
   + max_t_start(place_orange_box_3,_place_white_box_1) * check_parallelism(place_orange_box_3,place_white_box_1)
   ] = 0
 qc251: overlapping[place_orange_box_3,place_white_box_2] + [
   - min_t_end(place_orange_box_3,_place_white_box_2) * check_parallelism(place_orange_box_3,place_white_box_2)
   + max_t_start(place_orange_box_3,_place_white_box_2) * check_parallelism(place_orange_box_3,place_white_box_2)
   ] = 0
 qc252: overlapping[place_orange_box_3,place_white_box_3] + [
   - min_t_end(place_orange_box_3,_place_white_box_3) * check_parallelism(place_orange_box_3,place_white_box_3)
   + max_t_start(place_orange_box_3,_place_white_box_3) * check_parallelism(place_orange_box_3,place_white_box_3)
   ] = 0
 qc253: overlapping[place_orange_box_3,place_white_box_4] + [
   - min_t_end(place_orange_box_3,_place_white_box_4) * check_parallelism(place_orange_box_3,place_white_box_4)
   + max_t_start(place_orange_box_3,_place_white_box_4) * check_parallelism(place_orange_box_3,place_white_box_4)
   ] = 0
 qc254: overlapping[place_orange_box_4,place_blue_box_1] + [
   - min_t_end(place_orange_box_4,_place_blue_box_1) * check_parallelism(place_orange_box_4,place_blue_box_1)
   + max_t_start(place_orange_box_4,_place_blue_box_1) * check_parallelism(place_orange_box_4,place_blue_box_1)
   ] = 0
 qc255: overlapping[place_orange_box_4,place_blue_box_2] + [
   - min_t_end(place_orange_box_4,_place_blue_box_2) * check_parallelism(place_orange_box_4,place_blue_box_2)
   + max_t_start(place_orange_box_4,_place_blue_box_2) * check_parallelism(place_orange_box_4,place_blue_box_2)
   ] = 0
 qc256: overlapping[place_orange_box_4,place_blue_box_3] + [
   - min_t_end(place_orange_box_4,_place_blue_box_3) * check_parallelism(place_orange_box_4,place_blue_box_3)
   + max_t_start(place_orange_box_4,_place_blue_box_3) * check_parallelism(place_orange_box_4,place_blue_box_3)
   ] = 0
 qc257: overlapping[place_orange_box_4,place_blue_box_4] + [
   - min_t_end(place_orange_box_4,_place_blue_box_4) * check_parallelism(place_orange_box_4,place_blue_box_4)
   + max_t_start(place_orange_box_4,_place_blue_box_4) * check_parallelism(place_orange_box_4,place_blue_box_4)
   ] = 0
 qc258: overlapping[place_orange_box_4,place_white_box_1] + [
   - min_t_end(place_orange_box_4,_place_white_box_1) * check_parallelism(place_orange_box_4,place_white_box_1)
   + max_t_start(place_orange_box_4,_place_white_box_1) * check_parallelism(place_orange_box_4,place_white_box_1)
   ] = 0
 qc259: overlapping[place_orange_box_4,place_white_box_2] + [
   - min_t_end(place_orange_box_4,_place_white_box_2) * check_parallelism(place_orange_box_4,place_white_box_2)
   + max_t_start(place_orange_box_4,_place_white_box_2) * check_parallelism(place_orange_box_4,place_white_box_2)
   ] = 0
 qc260: overlapping[place_orange_box_4,place_white_box_3] + [
   - min_t_end(place_orange_box_4,_place_white_box_3) * check_parallelism(place_orange_box_4,place_white_box_3)
   + max_t_start(place_orange_box_4,_place_white_box_3) * check_parallelism(place_orange_box_4,place_white_box_3)
   ] = 0
 qc261: overlapping[place_orange_box_4,place_white_box_4] + [
   - min_t_end(place_orange_box_4,_place_white_box_4) * check_parallelism(place_orange_box_4,place_white_box_4)
   + max_t_start(place_orange_box_4,_place_white_box_4) * check_parallelism(place_orange_box_4,place_white_box_4)
   ] = 0
 qc262: overlapping[place_blue_box_1,place_blue_box_2] + [
   - min_t_end(place_blue_box_1,_place_blue_box_2) * check_parallelism(place_blue_box_1,place_blue_box_2)
   + max_t_start(place_blue_box_1,_place_blue_box_2) * check_parallelism(place_blue_box_1,place_blue_box_2)
   ] = 0
 qc263: overlapping[place_blue_box_1,place_blue_box_3] + [
   - min_t_end(place_blue_box_1,_place_blue_box_3) * check_parallelism(place_blue_box_1,place_blue_box_3)
   + max_t_start(place_blue_box_1,_place_blue_box_3) * check_parallelism(place_blue_box_1,place_blue_box_3)
   ] = 0
 qc264: overlapping[place_blue_box_1,place_blue_box_4] + [
   - min_t_end(place_blue_box_1,_place_blue_box_4) * check_parallelism(place_blue_box_1,place_blue_box_4)
   + max_t_start(place_blue_box_1,_place_blue_box_4) * check_parallelism(place_blue_box_1,place_blue_box_4)
   ] = 0
 qc265: overlapping[place_blue_box_1,place_white_box_1] + [
   - min_t_end(place_blue_box_1,_place_white_box_1) * check_parallelism(place_blue_box_1,place_white_box_1)
   + max_t_start(place_blue_box_1,_place_white_box_1) * check_parallelism(place_blue_box_1,place_white_box_1)
   ] = 0
 qc266: overlapping[place_blue_box_1,place_white_box_2] + [
   - min_t_end(place_blue_box_1,_place_white_box_2) * check_parallelism(place_blue_box_1,place_white_box_2)
   + max_t_start(place_blue_box_1,_place_white_box_2) * check_parallelism(place_blue_box_1,place_white_box_2)
   ] = 0
 qc267: overlapping[place_blue_box_1,place_white_box_3] + [
   - min_t_end(place_blue_box_1,_place_white_box_3) * check_parallelism(place_blue_box_1,place_white_box_3)
   + max_t_start(place_blue_box_1,_place_white_box_3) * check_parallelism(place_blue_box_1,place_white_box_3)
   ] = 0
 qc268: overlapping[place_blue_box_1,place_white_box_4] + [
   - min_t_end(place_blue_box_1,_place_white_box_4) * check_parallelism(place_blue_box_1,place_white_box_4)
   + max_t_start(place_blue_box_1,_place_white_box_4) * check_parallelism(place_blue_box_1,place_white_box_4)
   ] = 0
 qc269: overlapping[place_blue_box_2,place_blue_box_3] + [
   - min_t_end(place_blue_box_2,_place_blue_box_3) * check_parallelism(place_blue_box_2,place_blue_box_3)
   + max_t_start(place_blue_box_2,_place_blue_box_3) * check_parallelism(place_blue_box_2,place_blue_box_3)
   ] = 0
 qc270: overlapping[place_blue_box_2,place_blue_box_4] + [
   - min_t_end(place_blue_box_2,_place_blue_box_4) * check_parallelism(place_blue_box_2,place_blue_box_4)
   + max_t_start(place_blue_box_2,_place_blue_box_4) * check_parallelism(place_blue_box_2,place_blue_box_4)
   ] = 0
 qc271: overlapping[place_blue_box_2,place_white_box_1] + [
   - min_t_end(place_blue_box_2,_place_white_box_1) * check_parallelism(place_blue_box_2,place_white_box_1)
   + max_t_start(place_blue_box_2,_place_white_box_1) * check_parallelism(place_blue_box_2,place_white_box_1)
   ] = 0
 qc272: overlapping[place_blue_box_2,place_white_box_2] + [
   - min_t_end(place_blue_box_2,_place_white_box_2) * check_parallelism(place_blue_box_2,place_white_box_2)
   + max_t_start(place_blue_box_2,_place_white_box_2) * check_parallelism(place_blue_box_2,place_white_box_2)
   ] = 0
 qc273: overlapping[place_blue_box_2,place_white_box_3] + [
   - min_t_end(place_blue_box_2,_place_white_box_3) * check_parallelism(place_blue_box_2,place_white_box_3)
   + max_t_start(place_blue_box_2,_place_white_box_3) * check_parallelism(place_blue_box_2,place_white_box_3)
   ] = 0
 qc274: overlapping[place_blue_box_2,place_white_box_4] + [
   - min_t_end(place_blue_box_2,_place_white_box_4) * check_parallelism(place_blue_box_2,place_white_box_4)
   + max_t_start(place_blue_box_2,_place_white_box_4) * check_parallelism(place_blue_box_2,place_white_box_4)
   ] = 0
 qc275: overlapping[place_blue_box_3,place_blue_box_4] + [
   - min_t_end(place_blue_box_3,_place_blue_box_4) * check_parallelism(place_blue_box_3,place_blue_box_4)
   + max_t_start(place_blue_box_3,_place_blue_box_4) * check_parallelism(place_blue_box_3,place_blue_box_4)
   ] = 0
 qc276: overlapping[place_blue_box_3,place_white_box_1] + [
   - min_t_end(place_blue_box_3,_place_white_box_1) * check_parallelism(place_blue_box_3,place_white_box_1)
   + max_t_start(place_blue_box_3,_place_white_box_1) * check_parallelism(place_blue_box_3,place_white_box_1)
   ] = 0
 qc277: overlapping[place_blue_box_3,place_white_box_2] + [
   - min_t_end(place_blue_box_3,_place_white_box_2) * check_parallelism(place_blue_box_3,place_white_box_2)
   + max_t_start(place_blue_box_3,_place_white_box_2) * check_parallelism(place_blue_box_3,place_white_box_2)
   ] = 0
 qc278: overlapping[place_blue_box_3,place_white_box_3] + [
   - min_t_end(place_blue_box_3,_place_white_box_3) * check_parallelism(place_blue_box_3,place_white_box_3)
   + max_t_start(place_blue_box_3,_place_white_box_3) * check_parallelism(place_blue_box_3,place_white_box_3)
   ] = 0
 qc279: overlapping[place_blue_box_3,place_white_box_4] + [
   - min_t_end(place_blue_box_3,_place_white_box_4) * check_parallelism(place_blue_box_3,place_white_box_4)
   + max_t_start(place_blue_box_3,_place_white_box_4) * check_parallelism(place_blue_box_3,place_white_box_4)
   ] = 0
 qc280: overlapping[place_blue_box_4,place_white_box_1] + [
   - min_t_end(place_blue_box_4,_place_white_box_1) * check_parallelism(place_blue_box_4,place_white_box_1)
   + max_t_start(place_blue_box_4,_place_white_box_1) * check_parallelism(place_blue_box_4,place_white_box_1)
   ] = 0
 qc281: overlapping[place_blue_box_4,place_white_box_2] + [
   - min_t_end(place_blue_box_4,_place_white_box_2) * check_parallelism(place_blue_box_4,place_white_box_2)
   + max_t_start(place_blue_box_4,_place_white_box_2) * check_parallelism(place_blue_box_4,place_white_box_2)
   ] = 0
 qc282: overlapping[place_blue_box_4,place_white_box_3] + [
   - min_t_end(place_blue_box_4,_place_white_box_3) * check_parallelism(place_blue_box_4,place_white_box_3)
   + max_t_start(place_blue_box_4,_place_white_box_3) * check_parallelism(place_blue_box_4,place_white_box_3)
   ] = 0
 qc283: overlapping[place_blue_box_4,place_white_box_4] + [
   - min_t_end(place_blue_box_4,_place_white_box_4) * check_parallelism(place_blue_box_4,place_white_box_4)
   + max_t_start(place_blue_box_4,_place_white_box_4) * check_parallelism(place_blue_box_4,place_white_box_4)
   ] = 0
 qc284: overlapping[place_white_box_1,place_white_box_2] + [
   - min_t_end(place_white_box_1,_place_white_box_2) * check_parallelism(place_white_box_1,place_white_box_2)
   + max_t_start(place_white_box_1,_place_white_box_2) * check_parallelism(place_white_box_1,place_white_box_2)
   ] = 0
 qc285: overlapping[place_white_box_1,place_white_box_3] + [
   - min_t_end(place_white_box_1,_place_white_box_3) * check_parallelism(place_white_box_1,place_white_box_3)
   + max_t_start(place_white_box_1,_place_white_box_3) * check_parallelism(place_white_box_1,place_white_box_3)
   ] = 0
 qc286: overlapping[place_white_box_1,place_white_box_4] + [
   - min_t_end(place_white_box_1,_place_white_box_4) * check_parallelism(place_white_box_1,place_white_box_4)
   + max_t_start(place_white_box_1,_place_white_box_4) * check_parallelism(place_white_box_1,place_white_box_4)
   ] = 0
 qc287: overlapping[place_white_box_2,place_white_box_3] + [
   - min_t_end(place_white_box_2,_place_white_box_3) * check_parallelism(place_white_box_2,place_white_box_3)
   + max_t_start(place_white_box_2,_place_white_box_3) * check_parallelism(place_white_box_2,place_white_box_3)
   ] = 0
 qc288: overlapping[place_white_box_2,place_white_box_4] + [
   - min_t_end(place_white_box_2,_place_white_box_4) * check_parallelism(place_white_box_2,place_white_box_4)
   + max_t_start(place_white_box_2,_place_white_box_4) * check_parallelism(place_white_box_2,place_white_box_4)
   ] = 0
 qc289: overlapping[place_white_box_3,place_white_box_4] + [
   - min_t_end(place_white_box_3,_place_white_box_4) * check_parallelism(place_white_box_3,place_white_box_4)
   + max_t_start(place_white_box_3,_place_white_box_4) * check_parallelism(place_white_box_3,place_white_box_4)
   ] = 0
 GC553: both_pick_orange_box_1_pick_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC554: both_pick_orange_box_1_pick_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_2'),ur5_on_guide]
   <= 0
 GC555: both_pick_orange_box_1_pick_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[pick_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC556: both_pick_orange_box_1_pick_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_2'),ur5_on_guide]
   >= 0
 GC558: both_pick_orange_box_1_pick_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC559: both_pick_orange_box_1_pick_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_3'),ur5_on_guide]
   <= 0
 GC560: both_pick_orange_box_1_pick_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[pick_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_3'),ur5_on_guide]
   >= -1e+07
 GC561: both_pick_orange_box_1_pick_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_3'),ur5_on_guide]
   >= 0
 GC563: both_pick_orange_box_1_pick_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC564: both_pick_orange_box_1_pick_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_4'),ur5_on_guide]
   <= 0
 GC565: both_pick_orange_box_1_pick_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[pick_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_4'),ur5_on_guide]
   >= -1e+07
 GC566: both_pick_orange_box_1_pick_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'pick_orange_box_4'),ur5_on_guide]
   >= 0
 GC568: both_pick_orange_box_1_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC569: both_pick_orange_box_1_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_1'),ur5_on_guide]
   <= 0
 GC570: both_pick_orange_box_1_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_1'),ur5_on_guide]
   >= -1e+07
 GC571: both_pick_orange_box_1_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_1'),ur5_on_guide]
   >= 0
 GC573: both_pick_orange_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC574: both_pick_orange_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_2'),ur5_on_guide]
   <= 0
 GC575: both_pick_orange_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_2'),ur5_on_guide]
   >= -1e+07
 GC576: both_pick_orange_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_2'),ur5_on_guide]
   >= 0
 GC578: both_pick_orange_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC579: both_pick_orange_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_3'),ur5_on_guide]
   <= 0
 GC580: both_pick_orange_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC581: both_pick_orange_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_3'),ur5_on_guide]
   >= 0
 GC583: both_pick_orange_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC584: both_pick_orange_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_4'),ur5_on_guide]
   <= 0
 GC585: both_pick_orange_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC586: both_pick_orange_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'pick_blue_box_4'),ur5_on_guide]
   >= 0
 GC588: both_pick_orange_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC589: both_pick_orange_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_1'),ur5_on_guide]
   <= 0
 GC590: both_pick_orange_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_1'),ur5_on_guide]
   >= -1e+07
 GC591: both_pick_orange_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_1'),ur5_on_guide]
   >= 0
 GC593: both_pick_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC594: both_pick_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC595: both_pick_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC596: both_pick_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC598: both_pick_orange_box_1_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC599: both_pick_orange_box_1_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_3'),ur5_on_guide]
   <= 0
 GC600: both_pick_orange_box_1_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[place_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_3'),ur5_on_guide]
   >= -1e+07
 GC601: both_pick_orange_box_1_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_3'),ur5_on_guide]
   >= 0
 GC603: both_pick_orange_box_1_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC604: both_pick_orange_box_1_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_4'),ur5_on_guide]
   <= 0
 GC605: both_pick_orange_box_1_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[place_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_4'),ur5_on_guide]
   >= -1e+07
 GC606: both_pick_orange_box_1_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'place_orange_box_4'),ur5_on_guide]
   >= 0
 GC608: both_pick_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC609: both_pick_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC610: both_pick_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC611: both_pick_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC613: both_pick_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC614: both_pick_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC615: both_pick_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_1] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC616: both_pick_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[pick_orange_box_1]
   + 1e+07 same_agent[('pick_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC618: both_pick_orange_box_2_pick_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_orange_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC619: both_pick_orange_box_2_pick_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_orange_box_3'),ur5_on_guide]
   <= 0
 GC620: both_pick_orange_box_2_pick_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[pick_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_orange_box_3'),ur5_on_guide]
   >= -1e+07
 GC621: both_pick_orange_box_2_pick_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'pick_orange_box_3'),ur5_on_guide]
   >= 0
 GC623: both_pick_orange_box_2_pick_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_orange_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC624: both_pick_orange_box_2_pick_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_orange_box_4'),ur5_on_guide]
   <= 0
 GC625: both_pick_orange_box_2_pick_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[pick_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_orange_box_4'),ur5_on_guide]
   >= -1e+07
 GC626: both_pick_orange_box_2_pick_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'pick_orange_box_4'),ur5_on_guide]
   >= 0
 GC628: both_pick_orange_box_2_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC629: both_pick_orange_box_2_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_1'),ur5_on_guide]
   <= 0
 GC630: both_pick_orange_box_2_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_1'),ur5_on_guide]
   >= -1e+07
 GC631: both_pick_orange_box_2_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_1'),ur5_on_guide]
   >= 0
 GC633: both_pick_orange_box_2_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC634: both_pick_orange_box_2_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_2'),ur5_on_guide]
   <= 0
 GC635: both_pick_orange_box_2_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_2'),ur5_on_guide]
   >= -1e+07
 GC636: both_pick_orange_box_2_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_2'),ur5_on_guide]
   >= 0
 GC638: both_pick_orange_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC639: both_pick_orange_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_3'),ur5_on_guide]
   <= 0
 GC640: both_pick_orange_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC641: both_pick_orange_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_3'),ur5_on_guide]
   >= 0
 GC643: both_pick_orange_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC644: both_pick_orange_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_4'),ur5_on_guide]
   <= 0
 GC645: both_pick_orange_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC646: both_pick_orange_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'pick_blue_box_4'),ur5_on_guide]
   >= 0
 GC648: both_pick_orange_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC649: both_pick_orange_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_1'),ur5_on_guide]
   <= 0
 GC650: both_pick_orange_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_1'),ur5_on_guide]
   >= -1e+07
 GC651: both_pick_orange_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_1'),ur5_on_guide]
   >= 0
 GC653: both_pick_orange_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC654: both_pick_orange_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC655: both_pick_orange_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC656: both_pick_orange_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC658: both_pick_orange_box_2_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC659: both_pick_orange_box_2_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_3'),ur5_on_guide]
   <= 0
 GC660: both_pick_orange_box_2_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[place_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_3'),ur5_on_guide]
   >= -1e+07
 GC661: both_pick_orange_box_2_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_3'),ur5_on_guide]
   >= 0
 GC663: both_pick_orange_box_2_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC664: both_pick_orange_box_2_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_4'),ur5_on_guide]
   <= 0
 GC665: both_pick_orange_box_2_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[place_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_4'),ur5_on_guide]
   >= -1e+07
 GC666: both_pick_orange_box_2_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'place_orange_box_4'),ur5_on_guide]
   >= 0
 GC668: both_pick_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC669: both_pick_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC670: both_pick_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC671: both_pick_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC673: both_pick_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC674: both_pick_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC675: both_pick_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_2] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC676: both_pick_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[pick_orange_box_2]
   + 1e+07 same_agent[('pick_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC678: both_pick_orange_box_3_pick_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[pick_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_orange_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC679: both_pick_orange_box_3_pick_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[pick_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_orange_box_4'),ur5_on_guide]
   <= 0
 GC680: both_pick_orange_box_3_pick_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_end[pick_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_orange_box_4'),ur5_on_guide]
   >= -1e+07
 GC681: both_pick_orange_box_3_pick_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_end[pick_orange_box_3]
   + 1e+07 same_agent[('pick_orange_box_3',_'pick_orange_box_4'),ur5_on_guide]
   >= 0
 GC683: both_pick_orange_box_3_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC684: both_pick_orange_box_3_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_1'),ur5_on_guide]
   <= 0
 GC685: both_pick_orange_box_3_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_end[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_1'),ur5_on_guide]
   >= -1e+07
 GC686: both_pick_orange_box_3_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_orange_box_3]
   + 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_1'),ur5_on_guide]
   >= 0
 GC688: both_pick_orange_box_3_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC689: both_pick_orange_box_3_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_2'),ur5_on_guide]
   <= 0
 GC690: both_pick_orange_box_3_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_end[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_2'),ur5_on_guide]
   >= -1e+07
 GC691: both_pick_orange_box_3_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_orange_box_3]
   + 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_2'),ur5_on_guide]
   >= 0
 GC693: both_pick_orange_box_3_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC694: both_pick_orange_box_3_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_3'),ur5_on_guide]
   <= 0
 GC695: both_pick_orange_box_3_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_end[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC696: both_pick_orange_box_3_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_orange_box_3]
   + 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_3'),ur5_on_guide]
   >= 0
 GC698: both_pick_orange_box_3_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC699: both_pick_orange_box_3_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_4'),ur5_on_guide]
   <= 0
 GC700: both_pick_orange_box_3_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC701: both_pick_orange_box_3_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_orange_box_3]
   + 1e+07 same_agent[('pick_orange_box_3',_'pick_blue_box_4'),ur5_on_guide]
   >= 0
 GC703: both_pick_orange_box_3_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC704: both_pick_orange_box_3_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_1'),ur5_on_guide]
   <= 0
 GC705: both_pick_orange_box_3_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_end[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_1'),ur5_on_guide]
   >= -1e+07
 GC706: both_pick_orange_box_3_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[pick_orange_box_3]
   + 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_1'),ur5_on_guide]
   >= 0
 GC708: both_pick_orange_box_3_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC709: both_pick_orange_box_3_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC710: both_pick_orange_box_3_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC711: both_pick_orange_box_3_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[pick_orange_box_3]
   + 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC713: both_pick_orange_box_3_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC714: both_pick_orange_box_3_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_3'),ur5_on_guide]
   <= 0
 GC715: both_pick_orange_box_3_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_end[place_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_3'),ur5_on_guide]
   >= -1e+07
 GC716: both_pick_orange_box_3_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_end[pick_orange_box_3]
   + 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_3'),ur5_on_guide]
   >= 0
 GC718: both_pick_orange_box_3_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC719: both_pick_orange_box_3_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_4'),ur5_on_guide]
   <= 0
 GC720: both_pick_orange_box_3_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_end[place_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_4'),ur5_on_guide]
   >= -1e+07
 GC721: both_pick_orange_box_3_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_end[pick_orange_box_3]
   + 1e+07 same_agent[('pick_orange_box_3',_'place_orange_box_4'),ur5_on_guide]
   >= 0
 GC723: both_pick_orange_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC724: both_pick_orange_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC725: both_pick_orange_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC726: both_pick_orange_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[pick_orange_box_3]
   + 1e+07 same_agent[('pick_orange_box_3',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC728: both_pick_orange_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC729: both_pick_orange_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC730: both_pick_orange_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_3] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC731: both_pick_orange_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[pick_orange_box_3]
   + 1e+07 same_agent[('pick_orange_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC733: both_pick_orange_box_4_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC734: both_pick_orange_box_4_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_1'),ur5_on_guide]
   <= 0
 GC735: both_pick_orange_box_4_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_end[pick_blue_box_1]
   - 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_1'),ur5_on_guide]
   >= -1e+07
 GC736: both_pick_orange_box_4_pick_blue_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_orange_box_4]
   + 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_1'),ur5_on_guide]
   >= 0
 GC738: both_pick_orange_box_4_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC739: both_pick_orange_box_4_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_2'),ur5_on_guide]
   <= 0
 GC740: both_pick_orange_box_4_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_end[pick_blue_box_2]
   - 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_2'),ur5_on_guide]
   >= -1e+07
 GC741: both_pick_orange_box_4_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_orange_box_4]
   + 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_2'),ur5_on_guide]
   >= 0
 GC743: both_pick_orange_box_4_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC744: both_pick_orange_box_4_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_3'),ur5_on_guide]
   <= 0
 GC745: both_pick_orange_box_4_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_end[pick_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC746: both_pick_orange_box_4_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_orange_box_4]
   + 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_3'),ur5_on_guide]
   >= 0
 GC748: both_pick_orange_box_4_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC749: both_pick_orange_box_4_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_4'),ur5_on_guide]
   <= 0
 GC750: both_pick_orange_box_4_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC751: both_pick_orange_box_4_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_orange_box_4]
   + 1e+07 same_agent[('pick_orange_box_4',_'pick_blue_box_4'),ur5_on_guide]
   >= 0
 GC753: both_pick_orange_box_4_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC754: both_pick_orange_box_4_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_1'),ur5_on_guide]
   <= 0
 GC755: both_pick_orange_box_4_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_end[place_orange_box_1]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_1'),ur5_on_guide]
   >= -1e+07
 GC756: both_pick_orange_box_4_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[pick_orange_box_4]
   + 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_1'),ur5_on_guide]
   >= 0
 GC758: both_pick_orange_box_4_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC759: both_pick_orange_box_4_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC760: both_pick_orange_box_4_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC761: both_pick_orange_box_4_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[pick_orange_box_4]
   + 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC763: both_pick_orange_box_4_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC764: both_pick_orange_box_4_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_3'),ur5_on_guide]
   <= 0
 GC765: both_pick_orange_box_4_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_end[place_orange_box_3]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_3'),ur5_on_guide]
   >= -1e+07
 GC766: both_pick_orange_box_4_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_end[pick_orange_box_4]
   + 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_3'),ur5_on_guide]
   >= 0
 GC768: both_pick_orange_box_4_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC769: both_pick_orange_box_4_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_4'),ur5_on_guide]
   <= 0
 GC770: both_pick_orange_box_4_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_end[place_orange_box_4]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_4'),ur5_on_guide]
   >= -1e+07
 GC771: both_pick_orange_box_4_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_end[pick_orange_box_4]
   + 1e+07 same_agent[('pick_orange_box_4',_'place_orange_box_4'),ur5_on_guide]
   >= 0
 GC773: both_pick_orange_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC774: both_pick_orange_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC775: both_pick_orange_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC776: both_pick_orange_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[pick_orange_box_4]
   + 1e+07 same_agent[('pick_orange_box_4',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC778: both_pick_orange_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC779: both_pick_orange_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC780: both_pick_orange_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_orange_box_4] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('pick_orange_box_4',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC781: both_pick_orange_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[pick_orange_box_4]
   + 1e+07 same_agent[('pick_orange_box_4',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC783: both_pick_blue_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC784: both_pick_blue_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),ur5_on_guide]
   <= 0
 GC785: both_pick_blue_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),ur5_on_guide]
   >= -1e+07
 GC786: both_pick_blue_box_1_pick_blue_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),ur5_on_guide]
   >= 0
 GC788: both_pick_blue_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC789: both_pick_blue_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),ur5_on_guide]
   <= 0
 GC790: both_pick_blue_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC791: both_pick_blue_box_1_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),ur5_on_guide]
   >= 0
 GC793: both_pick_blue_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC794: both_pick_blue_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),ur5_on_guide]
   <= 0
 GC795: both_pick_blue_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC796: both_pick_blue_box_1_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),ur5_on_guide]
   >= 0
 GC798: both_pick_blue_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC799: both_pick_blue_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_1'),ur5_on_guide]
   <= 0
 GC800: both_pick_blue_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_1'),ur5_on_guide]
   >= -1e+07
 GC801: both_pick_blue_box_1_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_1'),ur5_on_guide]
   >= 0
 GC803: both_pick_blue_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC804: both_pick_blue_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC805: both_pick_blue_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC806: both_pick_blue_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC808: both_pick_blue_box_1_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC809: both_pick_blue_box_1_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_3'),ur5_on_guide]
   <= 0
 GC810: both_pick_blue_box_1_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[place_orange_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_3'),ur5_on_guide]
   >= -1e+07
 GC811: both_pick_blue_box_1_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_3'),ur5_on_guide]
   >= 0
 GC813: both_pick_blue_box_1_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC814: both_pick_blue_box_1_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_4'),ur5_on_guide]
   <= 0
 GC815: both_pick_blue_box_1_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[place_orange_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_4'),ur5_on_guide]
   >= -1e+07
 GC816: both_pick_blue_box_1_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_orange_box_4'),ur5_on_guide]
   >= 0
 GC818: both_pick_blue_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC819: both_pick_blue_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC820: both_pick_blue_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC821: both_pick_blue_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC823: both_pick_blue_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC824: both_pick_blue_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC825: both_pick_blue_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_1] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC826: both_pick_blue_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC828: both_pick_blue_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC829: both_pick_blue_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),ur5_on_guide]
   <= 0
 GC830: both_pick_blue_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC831: both_pick_blue_box_2_pick_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),ur5_on_guide]
   >= 0
 GC833: both_pick_blue_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC834: both_pick_blue_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),ur5_on_guide]
   <= 0
 GC835: both_pick_blue_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC836: both_pick_blue_box_2_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),ur5_on_guide]
   >= 0
 GC838: both_pick_blue_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC839: both_pick_blue_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_1'),ur5_on_guide]
   <= 0
 GC840: both_pick_blue_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_1'),ur5_on_guide]
   >= -1e+07
 GC841: both_pick_blue_box_2_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_1'),ur5_on_guide]
   >= 0
 GC843: both_pick_blue_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC844: both_pick_blue_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC845: both_pick_blue_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC846: both_pick_blue_box_2_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC848: both_pick_blue_box_2_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC849: both_pick_blue_box_2_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_3'),ur5_on_guide]
   <= 0
 GC850: both_pick_blue_box_2_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[place_orange_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_3'),ur5_on_guide]
   >= -1e+07
 GC851: both_pick_blue_box_2_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_3'),ur5_on_guide]
   >= 0
 GC853: both_pick_blue_box_2_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC854: both_pick_blue_box_2_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_4'),ur5_on_guide]
   <= 0
 GC855: both_pick_blue_box_2_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[place_orange_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_4'),ur5_on_guide]
   >= -1e+07
 GC856: both_pick_blue_box_2_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_orange_box_4'),ur5_on_guide]
   >= 0
 GC858: both_pick_blue_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC859: both_pick_blue_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC860: both_pick_blue_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC861: both_pick_blue_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC863: both_pick_blue_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC864: both_pick_blue_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC865: both_pick_blue_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_2] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC866: both_pick_blue_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC868: both_pick_blue_box_3_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC869: both_pick_blue_box_3_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),ur5_on_guide]
   <= 0
 GC870: both_pick_blue_box_3_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC871: both_pick_blue_box_3_pick_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),ur5_on_guide]
   >= 0
 GC873: both_pick_blue_box_3_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC874: both_pick_blue_box_3_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_1'),ur5_on_guide]
   <= 0
 GC875: both_pick_blue_box_3_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_1'),ur5_on_guide]
   >= -1e+07
 GC876: both_pick_blue_box_3_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_1'),ur5_on_guide]
   >= 0
 GC878: both_pick_blue_box_3_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC879: both_pick_blue_box_3_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC880: both_pick_blue_box_3_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC881: both_pick_blue_box_3_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC883: both_pick_blue_box_3_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC884: both_pick_blue_box_3_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_3'),ur5_on_guide]
   <= 0
 GC885: both_pick_blue_box_3_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[place_orange_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_3'),ur5_on_guide]
   >= -1e+07
 GC886: both_pick_blue_box_3_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_3'),ur5_on_guide]
   >= 0
 GC888: both_pick_blue_box_3_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC889: both_pick_blue_box_3_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_4'),ur5_on_guide]
   <= 0
 GC890: both_pick_blue_box_3_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[place_orange_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_4'),ur5_on_guide]
   >= -1e+07
 GC891: both_pick_blue_box_3_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_orange_box_4'),ur5_on_guide]
   >= 0
 GC893: both_pick_blue_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC894: both_pick_blue_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC895: both_pick_blue_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC896: both_pick_blue_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC898: both_pick_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC899: both_pick_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC900: both_pick_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_3] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC901: both_pick_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC903: both_pick_blue_box_4_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_1'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC904: both_pick_blue_box_4_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_1'),ur5_on_guide]
   <= 0
 GC905: both_pick_blue_box_4_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[place_orange_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_1'),ur5_on_guide]
   >= -1e+07
 GC906: both_pick_blue_box_4_place_orange_box_1_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_1'),ur5_on_guide]
   >= 0
 GC908: both_pick_blue_box_4_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC909: both_pick_blue_box_4_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC910: both_pick_blue_box_4_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC911: both_pick_blue_box_4_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC913: both_pick_blue_box_4_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC914: both_pick_blue_box_4_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_3'),ur5_on_guide]
   <= 0
 GC915: both_pick_blue_box_4_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[place_orange_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_3'),ur5_on_guide]
   >= -1e+07
 GC916: both_pick_blue_box_4_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_3'),ur5_on_guide]
   >= 0
 GC918: both_pick_blue_box_4_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC919: both_pick_blue_box_4_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_4'),ur5_on_guide]
   <= 0
 GC920: both_pick_blue_box_4_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[place_orange_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_4'),ur5_on_guide]
   >= -1e+07
 GC921: both_pick_blue_box_4_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_orange_box_4'),ur5_on_guide]
   >= 0
 GC923: both_pick_blue_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC924: both_pick_blue_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC925: both_pick_blue_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC926: both_pick_blue_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC928: both_pick_blue_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC929: both_pick_blue_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC930: both_pick_blue_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[pick_blue_box_4] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC931: both_pick_blue_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC933: both_place_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC934: both_place_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_orange_box_2]
   - 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   <= 0
 GC935: both_place_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[place_orange_box_2]
   - 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= -1e+07
 GC936: both_place_orange_box_1_place_orange_box_2_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[place_orange_box_1]
   + 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
   >= 0
 GC938: both_place_orange_box_1_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC939: both_place_orange_box_1_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_3'),ur5_on_guide]
   <= 0
 GC940: both_place_orange_box_1_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[place_orange_box_3]
   - 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_3'),ur5_on_guide]
   >= -1e+07
 GC941: both_place_orange_box_1_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_end[place_orange_box_1]
   + 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_3'),ur5_on_guide]
   >= 0
 GC943: both_place_orange_box_1_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC944: both_place_orange_box_1_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_4'),ur5_on_guide]
   <= 0
 GC945: both_place_orange_box_1_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[place_orange_box_4]
   - 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_4'),ur5_on_guide]
   >= -1e+07
 GC946: both_place_orange_box_1_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_end[place_orange_box_1]
   + 1e+07 same_agent[('place_orange_box_1',_'place_orange_box_4'),ur5_on_guide]
   >= 0
 GC948: both_place_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC949: both_place_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC950: both_place_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC951: both_place_orange_box_1_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[place_orange_box_1]
   + 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC953: both_place_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC954: both_place_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC955: both_place_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_1] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC956: both_place_orange_box_1_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[place_orange_box_1]
   + 1e+07 same_agent[('place_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC958: both_place_orange_box_2_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('place_orange_box_2',_'place_orange_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC959: both_place_orange_box_2_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_start[place_orange_box_3]
   - 1e+07 same_agent[('place_orange_box_2',_'place_orange_box_3'),ur5_on_guide]
   <= 0
 GC960: both_place_orange_box_2_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[place_orange_box_3]
   - 1e+07 same_agent[('place_orange_box_2',_'place_orange_box_3'),ur5_on_guide]
   >= -1e+07
 GC961: both_place_orange_box_2_place_orange_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_end[place_orange_box_2]
   + 1e+07 same_agent[('place_orange_box_2',_'place_orange_box_3'),ur5_on_guide]
   >= 0
 GC963: both_place_orange_box_2_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('place_orange_box_2',_'place_orange_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC964: both_place_orange_box_2_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('place_orange_box_2',_'place_orange_box_4'),ur5_on_guide]
   <= 0
 GC965: both_place_orange_box_2_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[place_orange_box_4]
   - 1e+07 same_agent[('place_orange_box_2',_'place_orange_box_4'),ur5_on_guide]
   >= -1e+07
 GC966: both_place_orange_box_2_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_end[place_orange_box_2]
   + 1e+07 same_agent[('place_orange_box_2',_'place_orange_box_4'),ur5_on_guide]
   >= 0
 GC968: both_place_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC969: both_place_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC970: both_place_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC971: both_place_orange_box_2_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[place_orange_box_2]
   + 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC973: both_place_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC974: both_place_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC975: both_place_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_2] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC976: both_place_orange_box_2_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[place_orange_box_2]
   + 1e+07 same_agent[('place_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC978: both_place_orange_box_3_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('place_orange_box_3',_'place_orange_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC979: both_place_orange_box_3_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_start[place_orange_box_4]
   - 1e+07 same_agent[('place_orange_box_3',_'place_orange_box_4'),ur5_on_guide]
   <= 0
 GC980: both_place_orange_box_3_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_end[place_orange_box_4]
   - 1e+07 same_agent[('place_orange_box_3',_'place_orange_box_4'),ur5_on_guide]
   >= -1e+07
 GC981: both_place_orange_box_3_place_orange_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_end[place_orange_box_3]
   + 1e+07 same_agent[('place_orange_box_3',_'place_orange_box_4'),ur5_on_guide]
   >= 0
 GC983: both_place_orange_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_3',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC984: both_place_orange_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_3',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC985: both_place_orange_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_3',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC986: both_place_orange_box_3_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[place_orange_box_3]
   + 1e+07 same_agent[('place_orange_box_3',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC988: both_place_orange_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC989: both_place_orange_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_3',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC990: both_place_orange_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_3] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC991: both_place_orange_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[place_orange_box_3]
   + 1e+07 same_agent[('place_orange_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC993: both_place_orange_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_4',_'place_blue_box_3'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC994: both_place_orange_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_start[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_4',_'place_blue_box_3'),ur5_on_guide]
   <= 0
 GC995: both_place_orange_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_end[place_blue_box_3]
   - 1e+07 same_agent[('place_orange_box_4',_'place_blue_box_3'),ur5_on_guide]
   >= -1e+07
 GC996: both_place_orange_box_4_place_blue_box_3_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[place_orange_box_4]
   + 1e+07 same_agent[('place_orange_box_4',_'place_blue_box_3'),ur5_on_guide]
   >= 0
 GC998: both_place_orange_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_4',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC999: both_place_orange_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_4',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC1000: both_place_orange_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_orange_box_4] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('place_orange_box_4',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC1001: both_place_orange_box_4_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[place_orange_box_4]
   + 1e+07 same_agent[('place_orange_box_4',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC1003: both_place_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= -9.9999999999989998e+06
 GC1004: both_place_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_start[place_blue_box_4]
   - 1e+07 same_agent[('place_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   <= 0
 GC1005: both_place_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_3] - t_end[place_blue_box_4]
   - 1e+07 same_agent[('place_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= -1e+07
 GC1006: both_place_blue_box_3_place_blue_box_4_to_ur5_on_guide = 1 ->
   t_start[place_blue_box_4] - t_end[place_blue_box_3]
   + 1e+07 same_agent[('place_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
   >= 0
 GC1008: both_pick_blue_box_1_pick_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1009: both_pick_blue_box_1_pick_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),human_right_arm]
   <= 0
 GC1010: both_pick_blue_box_1_pick_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),human_right_arm]
   >= -1e+07
 GC1011: both_pick_blue_box_1_pick_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),human_right_arm]
   >= 0
 GC1013: both_pick_blue_box_1_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1014: both_pick_blue_box_1_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),human_right_arm]
   <= 0
 GC1015: both_pick_blue_box_1_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),human_right_arm]
   >= -1e+07
 GC1016: both_pick_blue_box_1_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),human_right_arm]
   >= 0
 GC1018: both_pick_blue_box_1_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1019: both_pick_blue_box_1_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),human_right_arm]
   <= 0
 GC1020: both_pick_blue_box_1_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),human_right_arm]
   >= -1e+07
 GC1021: both_pick_blue_box_1_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),human_right_arm]
   >= 0
 GC1023: both_pick_blue_box_1_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1024: both_pick_blue_box_1_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_1'),human_right_arm]
   <= 0
 GC1025: both_pick_blue_box_1_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_1'),human_right_arm]
   >= -1e+07
 GC1026: both_pick_blue_box_1_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_1'),human_right_arm]
   >= 0
 GC1028: both_pick_blue_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1029: both_pick_blue_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_2'),human_right_arm]
   <= 0
 GC1030: both_pick_blue_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_2'),human_right_arm]
   >= -1e+07
 GC1031: both_pick_blue_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_2'),human_right_arm]
   >= 0
 GC1033: both_pick_blue_box_1_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1034: both_pick_blue_box_1_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_3'),human_right_arm]
   <= 0
 GC1035: both_pick_blue_box_1_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_3'),human_right_arm]
   >= -1e+07
 GC1036: both_pick_blue_box_1_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_3'),human_right_arm]
   >= 0
 GC1038: both_pick_blue_box_1_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1039: both_pick_blue_box_1_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[pick_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_4'),human_right_arm]
   <= 0
 GC1040: both_pick_blue_box_1_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[pick_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_4'),human_right_arm]
   >= -1e+07
 GC1041: both_pick_blue_box_1_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'pick_white_box_4'),human_right_arm]
   >= 0
 GC1043: both_pick_blue_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1044: both_pick_blue_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_1'),human_right_arm]
   <= 0
 GC1045: both_pick_blue_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_1'),human_right_arm]
   >= -1e+07
 GC1046: both_pick_blue_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_1'),human_right_arm]
   >= 0
 GC1048: both_pick_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1049: both_pick_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC1050: both_pick_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC1051: both_pick_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC1053: both_pick_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1054: both_pick_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_1'),human_right_arm]
   <= 0
 GC1055: both_pick_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC1056: both_pick_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_1'),human_right_arm]
   >= 0
 GC1058: both_pick_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1059: both_pick_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_2'),human_right_arm]
   <= 0
 GC1060: both_pick_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC1061: both_pick_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_2'),human_right_arm]
   >= 0
 GC1063: both_pick_blue_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1064: both_pick_blue_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_3'),human_right_arm]
   <= 0
 GC1065: both_pick_blue_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[place_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_3'),human_right_arm]
   >= -1e+07
 GC1066: both_pick_blue_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_3'),human_right_arm]
   >= 0
 GC1068: both_pick_blue_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1069: both_pick_blue_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_4'),human_right_arm]
   <= 0
 GC1070: both_pick_blue_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_1] - t_end[place_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_4'),human_right_arm]
   >= -1e+07
 GC1071: both_pick_blue_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_4] - t_end[pick_blue_box_1]
   + 1e+07 same_agent[('pick_blue_box_1',_'place_white_box_4'),human_right_arm]
   >= 0
 GC1073: both_pick_blue_box_2_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1074: both_pick_blue_box_2_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),human_right_arm]
   <= 0
 GC1075: both_pick_blue_box_2_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_blue_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),human_right_arm]
   >= -1e+07
 GC1076: both_pick_blue_box_2_pick_blue_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),human_right_arm]
   >= 0
 GC1078: both_pick_blue_box_2_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1079: both_pick_blue_box_2_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),human_right_arm]
   <= 0
 GC1080: both_pick_blue_box_2_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),human_right_arm]
   >= -1e+07
 GC1081: both_pick_blue_box_2_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),human_right_arm]
   >= 0
 GC1083: both_pick_blue_box_2_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1084: both_pick_blue_box_2_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_1'),human_right_arm]
   <= 0
 GC1085: both_pick_blue_box_2_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_1'),human_right_arm]
   >= -1e+07
 GC1086: both_pick_blue_box_2_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_1'),human_right_arm]
   >= 0
 GC1088: both_pick_blue_box_2_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1089: both_pick_blue_box_2_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_2'),human_right_arm]
   <= 0
 GC1090: both_pick_blue_box_2_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_2'),human_right_arm]
   >= -1e+07
 GC1091: both_pick_blue_box_2_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_2'),human_right_arm]
   >= 0
 GC1093: both_pick_blue_box_2_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1094: both_pick_blue_box_2_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_3'),human_right_arm]
   <= 0
 GC1095: both_pick_blue_box_2_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_3'),human_right_arm]
   >= -1e+07
 GC1096: both_pick_blue_box_2_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_3'),human_right_arm]
   >= 0
 GC1098: both_pick_blue_box_2_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1099: both_pick_blue_box_2_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[pick_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_4'),human_right_arm]
   <= 0
 GC1100: both_pick_blue_box_2_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[pick_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_4'),human_right_arm]
   >= -1e+07
 GC1101: both_pick_blue_box_2_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'pick_white_box_4'),human_right_arm]
   >= 0
 GC1103: both_pick_blue_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1104: both_pick_blue_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_1'),human_right_arm]
   <= 0
 GC1105: both_pick_blue_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_1'),human_right_arm]
   >= -1e+07
 GC1106: both_pick_blue_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_1'),human_right_arm]
   >= 0
 GC1108: both_pick_blue_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1109: both_pick_blue_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC1110: both_pick_blue_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC1111: both_pick_blue_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC1113: both_pick_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1114: both_pick_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_1'),human_right_arm]
   <= 0
 GC1115: both_pick_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC1116: both_pick_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_1'),human_right_arm]
   >= 0
 GC1118: both_pick_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1119: both_pick_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_2'),human_right_arm]
   <= 0
 GC1120: both_pick_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC1121: both_pick_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_2'),human_right_arm]
   >= 0
 GC1123: both_pick_blue_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1124: both_pick_blue_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_3'),human_right_arm]
   <= 0
 GC1125: both_pick_blue_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[place_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_3'),human_right_arm]
   >= -1e+07
 GC1126: both_pick_blue_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_3'),human_right_arm]
   >= 0
 GC1128: both_pick_blue_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1129: both_pick_blue_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_4'),human_right_arm]
   <= 0
 GC1130: both_pick_blue_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_2] - t_end[place_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_4'),human_right_arm]
   >= -1e+07
 GC1131: both_pick_blue_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_4] - t_end[pick_blue_box_2]
   + 1e+07 same_agent[('pick_blue_box_2',_'place_white_box_4'),human_right_arm]
   >= 0
 GC1133: both_pick_blue_box_3_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1134: both_pick_blue_box_3_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),human_right_arm]
   <= 0
 GC1135: both_pick_blue_box_3_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_blue_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),human_right_arm]
   >= -1e+07
 GC1136: both_pick_blue_box_3_pick_blue_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),human_right_arm]
   >= 0
 GC1138: both_pick_blue_box_3_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1139: both_pick_blue_box_3_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_1'),human_right_arm]
   <= 0
 GC1140: both_pick_blue_box_3_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_1'),human_right_arm]
   >= -1e+07
 GC1141: both_pick_blue_box_3_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_1'),human_right_arm]
   >= 0
 GC1143: both_pick_blue_box_3_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1144: both_pick_blue_box_3_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_2'),human_right_arm]
   <= 0
 GC1145: both_pick_blue_box_3_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_2'),human_right_arm]
   >= -1e+07
 GC1146: both_pick_blue_box_3_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_2'),human_right_arm]
   >= 0
 GC1148: both_pick_blue_box_3_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1149: both_pick_blue_box_3_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_3'),human_right_arm]
   <= 0
 GC1150: both_pick_blue_box_3_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_3'),human_right_arm]
   >= -1e+07
 GC1151: both_pick_blue_box_3_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_3'),human_right_arm]
   >= 0
 GC1153: both_pick_blue_box_3_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1154: both_pick_blue_box_3_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[pick_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_4'),human_right_arm]
   <= 0
 GC1155: both_pick_blue_box_3_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[pick_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_4'),human_right_arm]
   >= -1e+07
 GC1156: both_pick_blue_box_3_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'pick_white_box_4'),human_right_arm]
   >= 0
 GC1158: both_pick_blue_box_3_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1159: both_pick_blue_box_3_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_1'),human_right_arm]
   <= 0
 GC1160: both_pick_blue_box_3_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_1'),human_right_arm]
   >= -1e+07
 GC1161: both_pick_blue_box_3_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_1'),human_right_arm]
   >= 0
 GC1163: both_pick_blue_box_3_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1164: both_pick_blue_box_3_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC1165: both_pick_blue_box_3_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC1166: both_pick_blue_box_3_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC1168: both_pick_blue_box_3_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1169: both_pick_blue_box_3_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_1'),human_right_arm]
   <= 0
 GC1170: both_pick_blue_box_3_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC1171: both_pick_blue_box_3_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_1'),human_right_arm]
   >= 0
 GC1173: both_pick_blue_box_3_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1174: both_pick_blue_box_3_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_2'),human_right_arm]
   <= 0
 GC1175: both_pick_blue_box_3_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC1176: both_pick_blue_box_3_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_2'),human_right_arm]
   >= 0
 GC1178: both_pick_blue_box_3_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1179: both_pick_blue_box_3_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_3'),human_right_arm]
   <= 0
 GC1180: both_pick_blue_box_3_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[place_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_3'),human_right_arm]
   >= -1e+07
 GC1181: both_pick_blue_box_3_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_3'),human_right_arm]
   >= 0
 GC1183: both_pick_blue_box_3_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1184: both_pick_blue_box_3_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_4'),human_right_arm]
   <= 0
 GC1185: both_pick_blue_box_3_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_3] - t_end[place_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_4'),human_right_arm]
   >= -1e+07
 GC1186: both_pick_blue_box_3_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_4] - t_end[pick_blue_box_3]
   + 1e+07 same_agent[('pick_blue_box_3',_'place_white_box_4'),human_right_arm]
   >= 0
 GC1188: both_pick_blue_box_4_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1189: both_pick_blue_box_4_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_1'),human_right_arm]
   <= 0
 GC1190: both_pick_blue_box_4_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_1'),human_right_arm]
   >= -1e+07
 GC1191: both_pick_blue_box_4_pick_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_1'),human_right_arm]
   >= 0
 GC1193: both_pick_blue_box_4_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1194: both_pick_blue_box_4_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_2'),human_right_arm]
   <= 0
 GC1195: both_pick_blue_box_4_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_2'),human_right_arm]
   >= -1e+07
 GC1196: both_pick_blue_box_4_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_2'),human_right_arm]
   >= 0
 GC1198: both_pick_blue_box_4_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[pick_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1199: both_pick_blue_box_4_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[pick_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_3'),human_right_arm]
   <= 0
 GC1200: both_pick_blue_box_4_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_3'),human_right_arm]
   >= -1e+07
 GC1201: both_pick_blue_box_4_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_3'),human_right_arm]
   >= 0
 GC1203: both_pick_blue_box_4_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[pick_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1204: both_pick_blue_box_4_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[pick_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_4'),human_right_arm]
   <= 0
 GC1205: both_pick_blue_box_4_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[pick_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_4'),human_right_arm]
   >= -1e+07
 GC1206: both_pick_blue_box_4_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'pick_white_box_4'),human_right_arm]
   >= 0
 GC1208: both_pick_blue_box_4_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1209: both_pick_blue_box_4_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_1'),human_right_arm]
   <= 0
 GC1210: both_pick_blue_box_4_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[place_blue_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_1'),human_right_arm]
   >= -1e+07
 GC1211: both_pick_blue_box_4_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_1'),human_right_arm]
   >= 0
 GC1213: both_pick_blue_box_4_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1214: both_pick_blue_box_4_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC1215: both_pick_blue_box_4_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC1216: both_pick_blue_box_4_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC1218: both_pick_blue_box_4_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1219: both_pick_blue_box_4_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_1'),human_right_arm]
   <= 0
 GC1220: both_pick_blue_box_4_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[place_white_box_1]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC1221: both_pick_blue_box_4_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_1'),human_right_arm]
   >= 0
 GC1223: both_pick_blue_box_4_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1224: both_pick_blue_box_4_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_2'),human_right_arm]
   <= 0
 GC1225: both_pick_blue_box_4_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[place_white_box_2]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC1226: both_pick_blue_box_4_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_2'),human_right_arm]
   >= 0
 GC1228: both_pick_blue_box_4_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1229: both_pick_blue_box_4_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_3'),human_right_arm]
   <= 0
 GC1230: both_pick_blue_box_4_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[place_white_box_3]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_3'),human_right_arm]
   >= -1e+07
 GC1231: both_pick_blue_box_4_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_3'),human_right_arm]
   >= 0
 GC1233: both_pick_blue_box_4_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1234: both_pick_blue_box_4_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_4'),human_right_arm]
   <= 0
 GC1235: both_pick_blue_box_4_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_blue_box_4] - t_end[place_white_box_4]
   - 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_4'),human_right_arm]
   >= -1e+07
 GC1236: both_pick_blue_box_4_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_4] - t_end[pick_blue_box_4]
   + 1e+07 same_agent[('pick_blue_box_4',_'place_white_box_4'),human_right_arm]
   >= 0
 GC1238: both_pick_white_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1239: both_pick_white_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[pick_white_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_2'),human_right_arm]
   <= 0
 GC1240: both_pick_white_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[pick_white_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_2'),human_right_arm]
   >= -1e+07
 GC1241: both_pick_white_box_1_pick_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[pick_white_box_1]
   + 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_2'),human_right_arm]
   >= 0
 GC1243: both_pick_white_box_1_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[pick_white_box_3]
   - 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1244: both_pick_white_box_1_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[pick_white_box_3]
   - 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_3'),human_right_arm]
   <= 0
 GC1245: both_pick_white_box_1_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[pick_white_box_3]
   - 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_3'),human_right_arm]
   >= -1e+07
 GC1246: both_pick_white_box_1_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_end[pick_white_box_1]
   + 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_3'),human_right_arm]
   >= 0
 GC1248: both_pick_white_box_1_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[pick_white_box_4]
   - 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1249: both_pick_white_box_1_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[pick_white_box_4]
   - 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_4'),human_right_arm]
   <= 0
 GC1250: both_pick_white_box_1_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[pick_white_box_4]
   - 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_4'),human_right_arm]
   >= -1e+07
 GC1251: both_pick_white_box_1_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_end[pick_white_box_1]
   + 1e+07 same_agent[('pick_white_box_1',_'pick_white_box_4'),human_right_arm]
   >= 0
 GC1253: both_pick_white_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1254: both_pick_white_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_1'),human_right_arm]
   <= 0
 GC1255: both_pick_white_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_1'),human_right_arm]
   >= -1e+07
 GC1256: both_pick_white_box_1_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[pick_white_box_1]
   + 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_1'),human_right_arm]
   >= 0
 GC1258: both_pick_white_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1259: both_pick_white_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC1260: both_pick_white_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC1261: both_pick_white_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[pick_white_box_1]
   + 1e+07 same_agent[('pick_white_box_1',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC1263: both_pick_white_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1264: both_pick_white_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_1'),human_right_arm]
   <= 0
 GC1265: both_pick_white_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC1266: both_pick_white_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[pick_white_box_1]
   + 1e+07 same_agent[('pick_white_box_1',_'place_white_box_1'),human_right_arm]
   >= 0
 GC1268: both_pick_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1269: both_pick_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_2'),human_right_arm]
   <= 0
 GC1270: both_pick_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC1271: both_pick_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[pick_white_box_1]
   + 1e+07 same_agent[('pick_white_box_1',_'place_white_box_2'),human_right_arm]
   >= 0
 GC1273: both_pick_white_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1274: both_pick_white_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_3'),human_right_arm]
   <= 0
 GC1275: both_pick_white_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[place_white_box_3]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_3'),human_right_arm]
   >= -1e+07
 GC1276: both_pick_white_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_end[pick_white_box_1]
   + 1e+07 same_agent[('pick_white_box_1',_'place_white_box_3'),human_right_arm]
   >= 0
 GC1278: both_pick_white_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1279: both_pick_white_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_4'),human_right_arm]
   <= 0
 GC1280: both_pick_white_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_1] - t_end[place_white_box_4]
   - 1e+07 same_agent[('pick_white_box_1',_'place_white_box_4'),human_right_arm]
   >= -1e+07
 GC1281: both_pick_white_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_4] - t_end[pick_white_box_1]
   + 1e+07 same_agent[('pick_white_box_1',_'place_white_box_4'),human_right_arm]
   >= 0
 GC1283: both_pick_white_box_2_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[pick_white_box_3]
   - 1e+07 same_agent[('pick_white_box_2',_'pick_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1284: both_pick_white_box_2_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[pick_white_box_3]
   - 1e+07 same_agent[('pick_white_box_2',_'pick_white_box_3'),human_right_arm]
   <= 0
 GC1285: both_pick_white_box_2_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[pick_white_box_3]
   - 1e+07 same_agent[('pick_white_box_2',_'pick_white_box_3'),human_right_arm]
   >= -1e+07
 GC1286: both_pick_white_box_2_pick_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_end[pick_white_box_2]
   + 1e+07 same_agent[('pick_white_box_2',_'pick_white_box_3'),human_right_arm]
   >= 0
 GC1288: both_pick_white_box_2_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[pick_white_box_4]
   - 1e+07 same_agent[('pick_white_box_2',_'pick_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1289: both_pick_white_box_2_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[pick_white_box_4]
   - 1e+07 same_agent[('pick_white_box_2',_'pick_white_box_4'),human_right_arm]
   <= 0
 GC1290: both_pick_white_box_2_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[pick_white_box_4]
   - 1e+07 same_agent[('pick_white_box_2',_'pick_white_box_4'),human_right_arm]
   >= -1e+07
 GC1291: both_pick_white_box_2_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_end[pick_white_box_2]
   + 1e+07 same_agent[('pick_white_box_2',_'pick_white_box_4'),human_right_arm]
   >= 0
 GC1293: both_pick_white_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1294: both_pick_white_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_1'),human_right_arm]
   <= 0
 GC1295: both_pick_white_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_1'),human_right_arm]
   >= -1e+07
 GC1296: both_pick_white_box_2_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[pick_white_box_2]
   + 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_1'),human_right_arm]
   >= 0
 GC1298: both_pick_white_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1299: both_pick_white_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC1300: both_pick_white_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC1301: both_pick_white_box_2_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[pick_white_box_2]
   + 1e+07 same_agent[('pick_white_box_2',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC1303: both_pick_white_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1304: both_pick_white_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_1'),human_right_arm]
   <= 0
 GC1305: both_pick_white_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC1306: both_pick_white_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[pick_white_box_2]
   + 1e+07 same_agent[('pick_white_box_2',_'place_white_box_1'),human_right_arm]
   >= 0
 GC1308: both_pick_white_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1309: both_pick_white_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_2'),human_right_arm]
   <= 0
 GC1310: both_pick_white_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC1311: both_pick_white_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[pick_white_box_2]
   + 1e+07 same_agent[('pick_white_box_2',_'place_white_box_2'),human_right_arm]
   >= 0
 GC1313: both_pick_white_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1314: both_pick_white_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_3'),human_right_arm]
   <= 0
 GC1315: both_pick_white_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[place_white_box_3]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_3'),human_right_arm]
   >= -1e+07
 GC1316: both_pick_white_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_end[pick_white_box_2]
   + 1e+07 same_agent[('pick_white_box_2',_'place_white_box_3'),human_right_arm]
   >= 0
 GC1318: both_pick_white_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1319: both_pick_white_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_4'),human_right_arm]
   <= 0
 GC1320: both_pick_white_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_2] - t_end[place_white_box_4]
   - 1e+07 same_agent[('pick_white_box_2',_'place_white_box_4'),human_right_arm]
   >= -1e+07
 GC1321: both_pick_white_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_4] - t_end[pick_white_box_2]
   + 1e+07 same_agent[('pick_white_box_2',_'place_white_box_4'),human_right_arm]
   >= 0
 GC1323: both_pick_white_box_3_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_start[pick_white_box_4]
   - 1e+07 same_agent[('pick_white_box_3',_'pick_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1324: both_pick_white_box_3_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_start[pick_white_box_4]
   - 1e+07 same_agent[('pick_white_box_3',_'pick_white_box_4'),human_right_arm]
   <= 0
 GC1325: both_pick_white_box_3_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_end[pick_white_box_4]
   - 1e+07 same_agent[('pick_white_box_3',_'pick_white_box_4'),human_right_arm]
   >= -1e+07
 GC1326: both_pick_white_box_3_pick_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_end[pick_white_box_3]
   + 1e+07 same_agent[('pick_white_box_3',_'pick_white_box_4'),human_right_arm]
   >= 0
 GC1328: both_pick_white_box_3_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_3',_'place_blue_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1329: both_pick_white_box_3_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_3',_'place_blue_box_1'),human_right_arm]
   <= 0
 GC1330: both_pick_white_box_3_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_end[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_3',_'place_blue_box_1'),human_right_arm]
   >= -1e+07
 GC1331: both_pick_white_box_3_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[pick_white_box_3]
   + 1e+07 same_agent[('pick_white_box_3',_'place_blue_box_1'),human_right_arm]
   >= 0
 GC1333: both_pick_white_box_3_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_3',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1334: both_pick_white_box_3_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_3',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC1335: both_pick_white_box_3_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_3',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC1336: both_pick_white_box_3_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[pick_white_box_3]
   + 1e+07 same_agent[('pick_white_box_3',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC1338: both_pick_white_box_3_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_3',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1339: both_pick_white_box_3_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_3',_'place_white_box_1'),human_right_arm]
   <= 0
 GC1340: both_pick_white_box_3_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_end[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_3',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC1341: both_pick_white_box_3_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[pick_white_box_3]
   + 1e+07 same_agent[('pick_white_box_3',_'place_white_box_1'),human_right_arm]
   >= 0
 GC1343: both_pick_white_box_3_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_3',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1344: both_pick_white_box_3_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_3',_'place_white_box_2'),human_right_arm]
   <= 0
 GC1345: both_pick_white_box_3_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_end[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_3',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC1346: both_pick_white_box_3_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[pick_white_box_3]
   + 1e+07 same_agent[('pick_white_box_3',_'place_white_box_2'),human_right_arm]
   >= 0
 GC1348: both_pick_white_box_3_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_white_box_3',_'place_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1349: both_pick_white_box_3_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_white_box_3',_'place_white_box_3'),human_right_arm]
   <= 0
 GC1350: both_pick_white_box_3_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_end[place_white_box_3]
   - 1e+07 same_agent[('pick_white_box_3',_'place_white_box_3'),human_right_arm]
   >= -1e+07
 GC1351: both_pick_white_box_3_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_end[pick_white_box_3]
   + 1e+07 same_agent[('pick_white_box_3',_'place_white_box_3'),human_right_arm]
   >= 0
 GC1353: both_pick_white_box_3_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_white_box_3',_'place_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1354: both_pick_white_box_3_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_white_box_3',_'place_white_box_4'),human_right_arm]
   <= 0
 GC1355: both_pick_white_box_3_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_3] - t_end[place_white_box_4]
   - 1e+07 same_agent[('pick_white_box_3',_'place_white_box_4'),human_right_arm]
   >= -1e+07
 GC1356: both_pick_white_box_3_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_4] - t_end[pick_white_box_3]
   + 1e+07 same_agent[('pick_white_box_3',_'place_white_box_4'),human_right_arm]
   >= 0
 GC1358: both_pick_white_box_4_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_4',_'place_blue_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1359: both_pick_white_box_4_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_start[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_4',_'place_blue_box_1'),human_right_arm]
   <= 0
 GC1360: both_pick_white_box_4_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_end[place_blue_box_1]
   - 1e+07 same_agent[('pick_white_box_4',_'place_blue_box_1'),human_right_arm]
   >= -1e+07
 GC1361: both_pick_white_box_4_place_blue_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[pick_white_box_4]
   + 1e+07 same_agent[('pick_white_box_4',_'place_blue_box_1'),human_right_arm]
   >= 0
 GC1363: both_pick_white_box_4_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_4',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1364: both_pick_white_box_4_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_4',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC1365: both_pick_white_box_4_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('pick_white_box_4',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC1366: both_pick_white_box_4_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[pick_white_box_4]
   + 1e+07 same_agent[('pick_white_box_4',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC1368: both_pick_white_box_4_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_4',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1369: both_pick_white_box_4_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_start[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_4',_'place_white_box_1'),human_right_arm]
   <= 0
 GC1370: both_pick_white_box_4_place_white_box_1_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_end[place_white_box_1]
   - 1e+07 same_agent[('pick_white_box_4',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC1371: both_pick_white_box_4_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[pick_white_box_4]
   + 1e+07 same_agent[('pick_white_box_4',_'place_white_box_1'),human_right_arm]
   >= 0
 GC1373: both_pick_white_box_4_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_4',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1374: both_pick_white_box_4_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_start[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_4',_'place_white_box_2'),human_right_arm]
   <= 0
 GC1375: both_pick_white_box_4_place_white_box_2_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_end[place_white_box_2]
   - 1e+07 same_agent[('pick_white_box_4',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC1376: both_pick_white_box_4_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[pick_white_box_4]
   + 1e+07 same_agent[('pick_white_box_4',_'place_white_box_2'),human_right_arm]
   >= 0
 GC1378: both_pick_white_box_4_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_white_box_4',_'place_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1379: both_pick_white_box_4_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_start[place_white_box_3]
   - 1e+07 same_agent[('pick_white_box_4',_'place_white_box_3'),human_right_arm]
   <= 0
 GC1380: both_pick_white_box_4_place_white_box_3_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_end[place_white_box_3]
   - 1e+07 same_agent[('pick_white_box_4',_'place_white_box_3'),human_right_arm]
   >= -1e+07
 GC1381: both_pick_white_box_4_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_end[pick_white_box_4]
   + 1e+07 same_agent[('pick_white_box_4',_'place_white_box_3'),human_right_arm]
   >= 0
 GC1383: both_pick_white_box_4_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_white_box_4',_'place_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1384: both_pick_white_box_4_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_start[place_white_box_4]
   - 1e+07 same_agent[('pick_white_box_4',_'place_white_box_4'),human_right_arm]
   <= 0
 GC1385: both_pick_white_box_4_place_white_box_4_to_human_right_arm = 1 ->
   t_start[pick_white_box_4] - t_end[place_white_box_4]
   - 1e+07 same_agent[('pick_white_box_4',_'place_white_box_4'),human_right_arm]
   >= -1e+07
 GC1386: both_pick_white_box_4_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_4] - t_end[pick_white_box_4]
   + 1e+07 same_agent[('pick_white_box_4',_'place_white_box_4'),human_right_arm]
   >= 0
 GC1388: both_place_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('place_blue_box_1',_'place_blue_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1389: both_place_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_blue_box_2]
   - 1e+07 same_agent[('place_blue_box_1',_'place_blue_box_2'),human_right_arm]
   <= 0
 GC1390: both_place_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[place_blue_box_2]
   - 1e+07 same_agent[('place_blue_box_1',_'place_blue_box_2'),human_right_arm]
   >= -1e+07
 GC1391: both_place_blue_box_1_place_blue_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[place_blue_box_1]
   + 1e+07 same_agent[('place_blue_box_1',_'place_blue_box_2'),human_right_arm]
   >= 0
 GC1393: both_place_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_white_box_1]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1394: both_place_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_white_box_1]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_1'),human_right_arm]
   <= 0
 GC1395: both_place_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[place_white_box_1]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC1396: both_place_blue_box_1_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[place_blue_box_1]
   + 1e+07 same_agent[('place_blue_box_1',_'place_white_box_1'),human_right_arm]
   >= 0
 GC1398: both_place_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1399: both_place_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_2'),human_right_arm]
   <= 0
 GC1400: both_place_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[place_white_box_2]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC1401: both_place_blue_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[place_blue_box_1]
   + 1e+07 same_agent[('place_blue_box_1',_'place_white_box_2'),human_right_arm]
   >= 0
 GC1403: both_place_blue_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_white_box_3]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1404: both_place_blue_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_white_box_3]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_3'),human_right_arm]
   <= 0
 GC1405: both_place_blue_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[place_white_box_3]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_3'),human_right_arm]
   >= -1e+07
 GC1406: both_place_blue_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_end[place_blue_box_1]
   + 1e+07 same_agent[('place_blue_box_1',_'place_white_box_3'),human_right_arm]
   >= 0
 GC1408: both_place_blue_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_white_box_4]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1409: both_place_blue_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_start[place_white_box_4]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_4'),human_right_arm]
   <= 0
 GC1410: both_place_blue_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_blue_box_1] - t_end[place_white_box_4]
   - 1e+07 same_agent[('place_blue_box_1',_'place_white_box_4'),human_right_arm]
   >= -1e+07
 GC1411: both_place_blue_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_4] - t_end[place_blue_box_1]
   + 1e+07 same_agent[('place_blue_box_1',_'place_white_box_4'),human_right_arm]
   >= 0
 GC1413: both_place_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_start[place_white_box_1]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_1'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1414: both_place_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_start[place_white_box_1]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_1'),human_right_arm]
   <= 0
 GC1415: both_place_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[place_white_box_1]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_1'),human_right_arm]
   >= -1e+07
 GC1416: both_place_blue_box_2_place_white_box_1_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[place_blue_box_2]
   + 1e+07 same_agent[('place_blue_box_2',_'place_white_box_1'),human_right_arm]
   >= 0
 GC1418: both_place_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_start[place_white_box_2]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1419: both_place_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_start[place_white_box_2]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_2'),human_right_arm]
   <= 0
 GC1420: both_place_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[place_white_box_2]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC1421: both_place_blue_box_2_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[place_blue_box_2]
   + 1e+07 same_agent[('place_blue_box_2',_'place_white_box_2'),human_right_arm]
   >= 0
 GC1423: both_place_blue_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_start[place_white_box_3]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1424: both_place_blue_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_start[place_white_box_3]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_3'),human_right_arm]
   <= 0
 GC1425: both_place_blue_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[place_white_box_3]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_3'),human_right_arm]
   >= -1e+07
 GC1426: both_place_blue_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_end[place_blue_box_2]
   + 1e+07 same_agent[('place_blue_box_2',_'place_white_box_3'),human_right_arm]
   >= 0
 GC1428: both_place_blue_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_start[place_white_box_4]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1429: both_place_blue_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_start[place_white_box_4]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_4'),human_right_arm]
   <= 0
 GC1430: both_place_blue_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_blue_box_2] - t_end[place_white_box_4]
   - 1e+07 same_agent[('place_blue_box_2',_'place_white_box_4'),human_right_arm]
   >= -1e+07
 GC1431: both_place_blue_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_4] - t_end[place_blue_box_2]
   + 1e+07 same_agent[('place_blue_box_2',_'place_white_box_4'),human_right_arm]
   >= 0
 GC1433: both_place_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('place_white_box_1',_'place_white_box_2'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1434: both_place_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_start[place_white_box_2]
   - 1e+07 same_agent[('place_white_box_1',_'place_white_box_2'),human_right_arm]
   <= 0
 GC1435: both_place_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[place_white_box_2]
   - 1e+07 same_agent[('place_white_box_1',_'place_white_box_2'),human_right_arm]
   >= -1e+07
 GC1436: both_place_white_box_1_place_white_box_2_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[place_white_box_1]
   + 1e+07 same_agent[('place_white_box_1',_'place_white_box_2'),human_right_arm]
   >= 0
 GC1438: both_place_white_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_start[place_white_box_3]
   - 1e+07 same_agent[('place_white_box_1',_'place_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1439: both_place_white_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_start[place_white_box_3]
   - 1e+07 same_agent[('place_white_box_1',_'place_white_box_3'),human_right_arm]
   <= 0
 GC1440: both_place_white_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[place_white_box_3]
   - 1e+07 same_agent[('place_white_box_1',_'place_white_box_3'),human_right_arm]
   >= -1e+07
 GC1441: both_place_white_box_1_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_end[place_white_box_1]
   + 1e+07 same_agent[('place_white_box_1',_'place_white_box_3'),human_right_arm]
   >= 0
 GC1443: both_place_white_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_start[place_white_box_4]
   - 1e+07 same_agent[('place_white_box_1',_'place_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1444: both_place_white_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_start[place_white_box_4]
   - 1e+07 same_agent[('place_white_box_1',_'place_white_box_4'),human_right_arm]
   <= 0
 GC1445: both_place_white_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_1] - t_end[place_white_box_4]
   - 1e+07 same_agent[('place_white_box_1',_'place_white_box_4'),human_right_arm]
   >= -1e+07
 GC1446: both_place_white_box_1_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_4] - t_end[place_white_box_1]
   + 1e+07 same_agent[('place_white_box_1',_'place_white_box_4'),human_right_arm]
   >= 0
 GC1448: both_place_white_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_start[place_white_box_3]
   - 1e+07 same_agent[('place_white_box_2',_'place_white_box_3'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1449: both_place_white_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_start[place_white_box_3]
   - 1e+07 same_agent[('place_white_box_2',_'place_white_box_3'),human_right_arm]
   <= 0
 GC1450: both_place_white_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[place_white_box_3]
   - 1e+07 same_agent[('place_white_box_2',_'place_white_box_3'),human_right_arm]
   >= -1e+07
 GC1451: both_place_white_box_2_place_white_box_3_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_end[place_white_box_2]
   + 1e+07 same_agent[('place_white_box_2',_'place_white_box_3'),human_right_arm]
   >= 0
 GC1453: both_place_white_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_start[place_white_box_4]
   - 1e+07 same_agent[('place_white_box_2',_'place_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1454: both_place_white_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_start[place_white_box_4]
   - 1e+07 same_agent[('place_white_box_2',_'place_white_box_4'),human_right_arm]
   <= 0
 GC1455: both_place_white_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_2] - t_end[place_white_box_4]
   - 1e+07 same_agent[('place_white_box_2',_'place_white_box_4'),human_right_arm]
   >= -1e+07
 GC1456: both_place_white_box_2_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_4] - t_end[place_white_box_2]
   + 1e+07 same_agent[('place_white_box_2',_'place_white_box_4'),human_right_arm]
   >= 0
 GC1458: both_place_white_box_3_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_start[place_white_box_4]
   - 1e+07 same_agent[('place_white_box_3',_'place_white_box_4'),human_right_arm]
   >= -9.9999999999989998e+06
 GC1459: both_place_white_box_3_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_start[place_white_box_4]
   - 1e+07 same_agent[('place_white_box_3',_'place_white_box_4'),human_right_arm]
   <= 0
 GC1460: both_place_white_box_3_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_3] - t_end[place_white_box_4]
   - 1e+07 same_agent[('place_white_box_3',_'place_white_box_4'),human_right_arm]
   >= -1e+07
 GC1461: both_place_white_box_3_place_white_box_4_to_human_right_arm = 1 ->
   t_start[place_white_box_4] - t_end[place_white_box_3]
   + 1e+07 same_agent[('place_white_box_3',_'place_white_box_4'),human_right_arm]
   >= 0
Bounds
 overlapping[pick_orange_box_1,pick_orange_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_1,pick_orange_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_1,pick_orange_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_1,pick_blue_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_1,pick_blue_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_1,pick_blue_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_1,pick_blue_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_1,pick_white_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_1,pick_white_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_1,pick_white_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_1,pick_white_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_1,place_orange_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_1,place_orange_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_1,place_orange_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_1,place_orange_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_1,place_blue_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_1,place_blue_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_1,place_blue_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_1,place_blue_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_1,place_white_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_1,place_white_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_1,place_white_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_1,place_white_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_2,pick_orange_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_2,pick_orange_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_2,pick_blue_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_2,pick_blue_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_2,pick_blue_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_2,pick_blue_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_2,pick_white_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_2,pick_white_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_2,pick_white_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_2,pick_white_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_2,place_orange_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_2,place_orange_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_2,place_orange_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_2,place_orange_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_2,place_blue_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_2,place_blue_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_2,place_blue_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_2,place_blue_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_2,place_white_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_2,place_white_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_2,place_white_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_2,place_white_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_3,pick_orange_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_3,pick_blue_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_3,pick_blue_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_3,pick_blue_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_3,pick_blue_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_3,pick_white_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_3,pick_white_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_3,pick_white_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_3,pick_white_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_3,place_orange_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_3,place_orange_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_3,place_orange_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_3,place_orange_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_3,place_blue_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_3,place_blue_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_3,place_blue_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_3,place_blue_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_3,place_white_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_3,place_white_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_3,place_white_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_3,place_white_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_4,pick_blue_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_4,pick_blue_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_4,pick_blue_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_4,pick_blue_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_4,pick_white_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_4,pick_white_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_4,pick_white_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_4,pick_white_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_4,place_orange_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_4,place_orange_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_4,place_orange_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_4,place_orange_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_4,place_blue_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_4,place_blue_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_4,place_blue_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_4,place_blue_box_4] <= 183.2813963713815
 overlapping[pick_orange_box_4,place_white_box_1] <= 183.2813963713815
 overlapping[pick_orange_box_4,place_white_box_2] <= 183.2813963713815
 overlapping[pick_orange_box_4,place_white_box_3] <= 183.2813963713815
 overlapping[pick_orange_box_4,place_white_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_1,pick_blue_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_1,pick_blue_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_1,pick_blue_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_1,pick_white_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_1,pick_white_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_1,pick_white_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_1,pick_white_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_1,place_orange_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_1,place_orange_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_1,place_orange_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_1,place_orange_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_1,place_blue_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_1,place_blue_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_1,place_blue_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_1,place_blue_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_1,place_white_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_1,place_white_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_1,place_white_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_1,place_white_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_2,pick_blue_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_2,pick_blue_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_2,pick_white_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_2,pick_white_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_2,pick_white_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_2,pick_white_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_2,place_orange_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_2,place_orange_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_2,place_orange_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_2,place_orange_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_2,place_blue_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_2,place_blue_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_2,place_blue_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_2,place_blue_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_2,place_white_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_2,place_white_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_2,place_white_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_2,place_white_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_3,pick_blue_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_3,pick_white_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_3,pick_white_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_3,pick_white_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_3,pick_white_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_3,place_orange_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_3,place_orange_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_3,place_orange_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_3,place_orange_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_3,place_blue_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_3,place_blue_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_3,place_blue_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_3,place_blue_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_3,place_white_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_3,place_white_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_3,place_white_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_3,place_white_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_4,pick_white_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_4,pick_white_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_4,pick_white_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_4,pick_white_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_4,place_orange_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_4,place_orange_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_4,place_orange_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_4,place_orange_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_4,place_blue_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_4,place_blue_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_4,place_blue_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_4,place_blue_box_4] <= 183.2813963713815
 overlapping[pick_blue_box_4,place_white_box_1] <= 183.2813963713815
 overlapping[pick_blue_box_4,place_white_box_2] <= 183.2813963713815
 overlapping[pick_blue_box_4,place_white_box_3] <= 183.2813963713815
 overlapping[pick_blue_box_4,place_white_box_4] <= 183.2813963713815
 overlapping[pick_white_box_1,pick_white_box_2] <= 183.2813963713815
 overlapping[pick_white_box_1,pick_white_box_3] <= 183.2813963713815
 overlapping[pick_white_box_1,pick_white_box_4] <= 183.2813963713815
 overlapping[pick_white_box_1,place_orange_box_1] <= 183.2813963713815
 overlapping[pick_white_box_1,place_orange_box_2] <= 183.2813963713815
 overlapping[pick_white_box_1,place_orange_box_3] <= 183.2813963713815
 overlapping[pick_white_box_1,place_orange_box_4] <= 183.2813963713815
 overlapping[pick_white_box_1,place_blue_box_1] <= 183.2813963713815
 overlapping[pick_white_box_1,place_blue_box_2] <= 183.2813963713815
 overlapping[pick_white_box_1,place_blue_box_3] <= 183.2813963713815
 overlapping[pick_white_box_1,place_blue_box_4] <= 183.2813963713815
 overlapping[pick_white_box_1,place_white_box_1] <= 183.2813963713815
 overlapping[pick_white_box_1,place_white_box_2] <= 183.2813963713815
 overlapping[pick_white_box_1,place_white_box_3] <= 183.2813963713815
 overlapping[pick_white_box_1,place_white_box_4] <= 183.2813963713815
 overlapping[pick_white_box_2,pick_white_box_3] <= 183.2813963713815
 overlapping[pick_white_box_2,pick_white_box_4] <= 183.2813963713815
 overlapping[pick_white_box_2,place_orange_box_1] <= 183.2813963713815
 overlapping[pick_white_box_2,place_orange_box_2] <= 183.2813963713815
 overlapping[pick_white_box_2,place_orange_box_3] <= 183.2813963713815
 overlapping[pick_white_box_2,place_orange_box_4] <= 183.2813963713815
 overlapping[pick_white_box_2,place_blue_box_1] <= 183.2813963713815
 overlapping[pick_white_box_2,place_blue_box_2] <= 183.2813963713815
 overlapping[pick_white_box_2,place_blue_box_3] <= 183.2813963713815
 overlapping[pick_white_box_2,place_blue_box_4] <= 183.2813963713815
 overlapping[pick_white_box_2,place_white_box_1] <= 183.2813963713815
 overlapping[pick_white_box_2,place_white_box_2] <= 183.2813963713815
 overlapping[pick_white_box_2,place_white_box_3] <= 183.2813963713815
 overlapping[pick_white_box_2,place_white_box_4] <= 183.2813963713815
 overlapping[pick_white_box_3,pick_white_box_4] <= 183.2813963713815
 overlapping[pick_white_box_3,place_orange_box_1] <= 183.2813963713815
 overlapping[pick_white_box_3,place_orange_box_2] <= 183.2813963713815
 overlapping[pick_white_box_3,place_orange_box_3] <= 183.2813963713815
 overlapping[pick_white_box_3,place_orange_box_4] <= 183.2813963713815
 overlapping[pick_white_box_3,place_blue_box_1] <= 183.2813963713815
 overlapping[pick_white_box_3,place_blue_box_2] <= 183.2813963713815
 overlapping[pick_white_box_3,place_blue_box_3] <= 183.2813963713815
 overlapping[pick_white_box_3,place_blue_box_4] <= 183.2813963713815
 overlapping[pick_white_box_3,place_white_box_1] <= 183.2813963713815
 overlapping[pick_white_box_3,place_white_box_2] <= 183.2813963713815
 overlapping[pick_white_box_3,place_white_box_3] <= 183.2813963713815
 overlapping[pick_white_box_3,place_white_box_4] <= 183.2813963713815
 overlapping[pick_white_box_4,place_orange_box_1] <= 183.2813963713815
 overlapping[pick_white_box_4,place_orange_box_2] <= 183.2813963713815
 overlapping[pick_white_box_4,place_orange_box_3] <= 183.2813963713815
 overlapping[pick_white_box_4,place_orange_box_4] <= 183.2813963713815
 overlapping[pick_white_box_4,place_blue_box_1] <= 183.2813963713815
 overlapping[pick_white_box_4,place_blue_box_2] <= 183.2813963713815
 overlapping[pick_white_box_4,place_blue_box_3] <= 183.2813963713815
 overlapping[pick_white_box_4,place_blue_box_4] <= 183.2813963713815
 overlapping[pick_white_box_4,place_white_box_1] <= 183.2813963713815
 overlapping[pick_white_box_4,place_white_box_2] <= 183.2813963713815
 overlapping[pick_white_box_4,place_white_box_3] <= 183.2813963713815
 overlapping[pick_white_box_4,place_white_box_4] <= 183.2813963713815
 overlapping[place_orange_box_1,place_orange_box_2] <= 183.2813963713815
 overlapping[place_orange_box_1,place_orange_box_3] <= 183.2813963713815
 overlapping[place_orange_box_1,place_orange_box_4] <= 183.2813963713815
 overlapping[place_orange_box_1,place_blue_box_1] <= 183.2813963713815
 overlapping[place_orange_box_1,place_blue_box_2] <= 183.2813963713815
 overlapping[place_orange_box_1,place_blue_box_3] <= 183.2813963713815
 overlapping[place_orange_box_1,place_blue_box_4] <= 183.2813963713815
 overlapping[place_orange_box_1,place_white_box_1] <= 183.2813963713815
 overlapping[place_orange_box_1,place_white_box_2] <= 183.2813963713815
 overlapping[place_orange_box_1,place_white_box_3] <= 183.2813963713815
 overlapping[place_orange_box_1,place_white_box_4] <= 183.2813963713815
 overlapping[place_orange_box_2,place_orange_box_3] <= 183.2813963713815
 overlapping[place_orange_box_2,place_orange_box_4] <= 183.2813963713815
 overlapping[place_orange_box_2,place_blue_box_1] <= 183.2813963713815
 overlapping[place_orange_box_2,place_blue_box_2] <= 183.2813963713815
 overlapping[place_orange_box_2,place_blue_box_3] <= 183.2813963713815
 overlapping[place_orange_box_2,place_blue_box_4] <= 183.2813963713815
 overlapping[place_orange_box_2,place_white_box_1] <= 183.2813963713815
 overlapping[place_orange_box_2,place_white_box_2] <= 183.2813963713815
 overlapping[place_orange_box_2,place_white_box_3] <= 183.2813963713815
 overlapping[place_orange_box_2,place_white_box_4] <= 183.2813963713815
 overlapping[place_orange_box_3,place_orange_box_4] <= 183.2813963713815
 overlapping[place_orange_box_3,place_blue_box_1] <= 183.2813963713815
 overlapping[place_orange_box_3,place_blue_box_2] <= 183.2813963713815
 overlapping[place_orange_box_3,place_blue_box_3] <= 183.2813963713815
 overlapping[place_orange_box_3,place_blue_box_4] <= 183.2813963713815
 overlapping[place_orange_box_3,place_white_box_1] <= 183.2813963713815
 overlapping[place_orange_box_3,place_white_box_2] <= 183.2813963713815
 overlapping[place_orange_box_3,place_white_box_3] <= 183.2813963713815
 overlapping[place_orange_box_3,place_white_box_4] <= 183.2813963713815
 overlapping[place_orange_box_4,place_blue_box_1] <= 183.2813963713815
 overlapping[place_orange_box_4,place_blue_box_2] <= 183.2813963713815
 overlapping[place_orange_box_4,place_blue_box_3] <= 183.2813963713815
 overlapping[place_orange_box_4,place_blue_box_4] <= 183.2813963713815
 overlapping[place_orange_box_4,place_white_box_1] <= 183.2813963713815
 overlapping[place_orange_box_4,place_white_box_2] <= 183.2813963713815
 overlapping[place_orange_box_4,place_white_box_3] <= 183.2813963713815
 overlapping[place_orange_box_4,place_white_box_4] <= 183.2813963713815
 overlapping[place_blue_box_1,place_blue_box_2] <= 183.2813963713815
 overlapping[place_blue_box_1,place_blue_box_3] <= 183.2813963713815
 overlapping[place_blue_box_1,place_blue_box_4] <= 183.2813963713815
 overlapping[place_blue_box_1,place_white_box_1] <= 183.2813963713815
 overlapping[place_blue_box_1,place_white_box_2] <= 183.2813963713815
 overlapping[place_blue_box_1,place_white_box_3] <= 183.2813963713815
 overlapping[place_blue_box_1,place_white_box_4] <= 183.2813963713815
 overlapping[place_blue_box_2,place_blue_box_3] <= 183.2813963713815
 overlapping[place_blue_box_2,place_blue_box_4] <= 183.2813963713815
 overlapping[place_blue_box_2,place_white_box_1] <= 183.2813963713815
 overlapping[place_blue_box_2,place_white_box_2] <= 183.2813963713815
 overlapping[place_blue_box_2,place_white_box_3] <= 183.2813963713815
 overlapping[place_blue_box_2,place_white_box_4] <= 183.2813963713815
 overlapping[place_blue_box_3,place_blue_box_4] <= 183.2813963713815
 overlapping[place_blue_box_3,place_white_box_1] <= 183.2813963713815
 overlapping[place_blue_box_3,place_white_box_2] <= 183.2813963713815
 overlapping[place_blue_box_3,place_white_box_3] <= 183.2813963713815
 overlapping[place_blue_box_3,place_white_box_4] <= 183.2813963713815
 overlapping[place_blue_box_4,place_white_box_1] <= 183.2813963713815
 overlapping[place_blue_box_4,place_white_box_2] <= 183.2813963713815
 overlapping[place_blue_box_4,place_white_box_3] <= 183.2813963713815
 overlapping[place_blue_box_4,place_white_box_4] <= 183.2813963713815
 overlapping[place_white_box_1,place_white_box_2] <= 183.2813963713815
 overlapping[place_white_box_1,place_white_box_3] <= 183.2813963713815
 overlapping[place_white_box_1,place_white_box_4] <= 183.2813963713815
 overlapping[place_white_box_2,place_white_box_3] <= 183.2813963713815
 overlapping[place_white_box_2,place_white_box_4] <= 183.2813963713815
 overlapping[place_white_box_3,place_white_box_4] <= 183.2813963713815
 t_start[pick_orange_box_1] <= 1832.813963713815
 t_start[pick_orange_box_2] <= 1832.813963713815
 t_start[pick_orange_box_3] <= 1832.813963713815
 t_start[pick_orange_box_4] <= 1832.813963713815
 t_start[pick_blue_box_1] <= 1832.813963713815
 t_start[pick_blue_box_2] <= 1832.813963713815
 t_start[pick_blue_box_3] <= 1832.813963713815
 t_start[pick_blue_box_4] <= 1832.813963713815
 t_start[pick_white_box_1] <= 1832.813963713815
 t_start[pick_white_box_2] <= 1832.813963713815
 t_start[pick_white_box_3] <= 1832.813963713815
 t_start[pick_white_box_4] <= 1832.813963713815
 t_start[place_orange_box_1] <= 1832.813963713815
 t_start[place_orange_box_2] <= 1832.813963713815
 t_start[place_orange_box_3] <= 1832.813963713815
 t_start[place_orange_box_4] <= 1832.813963713815
 t_start[place_blue_box_1] <= 1832.813963713815
 t_start[place_blue_box_2] <= 1832.813963713815
 t_start[place_blue_box_3] <= 1832.813963713815
 t_start[place_blue_box_4] <= 1832.813963713815
 t_start[place_white_box_1] <= 1832.813963713815
 t_start[place_white_box_2] <= 1832.813963713815
 t_start[place_white_box_3] <= 1832.813963713815
 t_start[place_white_box_4] <= 1832.813963713815
 t_end[pick_orange_box_1] <= 1832.813963713815
 t_end[pick_orange_box_2] <= 1832.813963713815
 t_end[pick_orange_box_3] <= 1832.813963713815
 t_end[pick_orange_box_4] <= 1832.813963713815
 t_end[pick_blue_box_1] <= 1832.813963713815
 t_end[pick_blue_box_2] <= 1832.813963713815
 t_end[pick_blue_box_3] <= 1832.813963713815
 t_end[pick_blue_box_4] <= 1832.813963713815
 t_end[pick_white_box_1] <= 1832.813963713815
 t_end[pick_white_box_2] <= 1832.813963713815
 t_end[pick_white_box_3] <= 1832.813963713815
 t_end[pick_white_box_4] <= 1832.813963713815
 t_end[place_orange_box_1] <= 1832.813963713815
 t_end[place_orange_box_2] <= 1832.813963713815
 t_end[place_orange_box_3] <= 1832.813963713815
 t_end[place_orange_box_4] <= 1832.813963713815
 t_end[place_blue_box_1] <= 1832.813963713815
 t_end[place_blue_box_2] <= 1832.813963713815
 t_end[place_blue_box_3] <= 1832.813963713815
 t_end[place_blue_box_4] <= 1832.813963713815
 t_end[place_white_box_1] <= 1832.813963713815
 t_end[place_white_box_2] <= 1832.813963713815
 t_end[place_white_box_3] <= 1832.813963713815
 t_end[place_white_box_4] <= 1832.813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,pick_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_pick_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_pick_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,pick_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_pick_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_pick_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,pick_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_pick_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_pick_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,pick_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_pick_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_pick_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_1,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_1,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_1,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,pick_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_pick_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_pick_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,pick_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_pick_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_pick_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,pick_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_pick_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_pick_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_2,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_2,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_2,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,pick_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_pick_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_pick_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,pick_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_pick_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_pick_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_3,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_3,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_3,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,pick_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_pick_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_pick_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_orange_box_4,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_orange_box_4,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_orange_box_4,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_pick_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_1,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_1,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_1,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_pick_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_2,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_2,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_2,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_pick_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_3,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_3,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_3,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_pick_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_blue_box_4,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_blue_box_4,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_blue_box_4,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_pick_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_1,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_1,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_1,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_2,pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_2,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_2,_pick_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_2,pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_2,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_2,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_2,place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_2,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_2,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_2,place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_2,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_2,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_2,place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_2,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_2,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_2,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_2,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_2,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_2,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_2,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_2,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_2,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_2,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_2,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_2,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_2,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_2,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_2,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_2,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_2,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_2,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_2,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_2,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_2,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_2,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_2,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_2,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_2,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_2,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_2,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_2,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_2,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_3,pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_3,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_3,_pick_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_3,place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_3,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_3,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_3,place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_3,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_3,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_3,place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_3,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_3,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_3,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_3,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_3,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_3,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_3,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_3,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_3,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_3,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_3,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_3,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_3,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_3,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_3,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_3,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_3,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_3,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_3,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_3,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_3,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_3,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_3,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_3,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_3,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_3,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_3,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_3,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_3,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_4,place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_4,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_4,_place_orange_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_4,place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_4,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_4,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_4,place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_4,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_4,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_4,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_4,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_4,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_4,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_4,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_4,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_4,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_4,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_4,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_4,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_4,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_4,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_4,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_4,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_4,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_4,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_4,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_4,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_4,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_4,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_4,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_4,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_4,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_4,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(pick_white_box_4,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(pick_white_box_4,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(pick_white_box_4,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_1,place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_1,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_1,_place_orange_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_1,place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_1,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_1,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_1,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_1,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_1,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_1,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_1,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_1,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_1,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_1,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_1,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_1,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_1,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_1,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_1,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_1,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_1,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_1,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_1,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_1,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_1,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_1,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_1,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_1,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_1,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_1,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_1,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_1,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_1,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_2,place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_2,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_2,_place_orange_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_2,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_2,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_2,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_2,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_2,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_2,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_2,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_2,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_2,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_2,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_2,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_2,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_2,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_2,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_2,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_2,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_2,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_2,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_2,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_2,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_2,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_2,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_2,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_2,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_2,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_2,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_2,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_3,place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_3,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_3,_place_orange_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_3,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_3,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_3,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_3,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_3,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_3,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_3,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_3,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_3,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_3,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_3,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_3,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_3,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_3,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_3,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_3,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_3,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_3,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_3,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_3,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_3,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_3,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_3,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_3,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_4,place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_4,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_4,_place_blue_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_4,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_4,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_4,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_4,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_4,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_4,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_4,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_4,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_4,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_4,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_4,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_4,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_4,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_4,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_4,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_4,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_4,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_4,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_orange_box_4,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_orange_box_4,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_orange_box_4,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_1,place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_1,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_1,_place_blue_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_1,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_1,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_1,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_1,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_1,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_1,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_1,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_1,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_1,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_1,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_1,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_1,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_1,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_1,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_1,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_1,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_1,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_1,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_2,place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_2,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_2,_place_blue_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_2,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_2,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_2,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_2,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_2,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_2,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_2,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_2,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_2,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_2,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_2,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_2,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_2,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_2,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_2,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_3,place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_3,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_3,_place_blue_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_3,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_3,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_3,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_3,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_3,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_3,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_3,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_3,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_3,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_3,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_3,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_3,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_4,place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_4,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_4,_place_white_box_1) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_4,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_4,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_4,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_4,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_4,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_4,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_blue_box_4,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_blue_box_4,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_blue_box_4,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_white_box_1,place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_white_box_1,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_white_box_1,_place_white_box_2) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_white_box_1,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_white_box_1,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_white_box_1,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_white_box_1,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_white_box_1,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_white_box_1,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_white_box_2,place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_white_box_2,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_white_box_2,_place_white_box_3) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_white_box_2,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_white_box_2,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_white_box_2,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= raw_overlapping(place_white_box_3,place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= min_t_end(place_white_box_3,_place_white_box_4) <= 183.2813963713815
 -183.2813963713815 <= max_t_start(place_white_box_3,_place_white_box_4) <= 183.2813963713815
Binaries
 assignment[ur5_on_guide,pick_orange_box_1]
 assignment[ur5_on_guide,pick_orange_box_2]
 assignment[ur5_on_guide,pick_orange_box_3]
 assignment[ur5_on_guide,pick_orange_box_4]
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
 assignment[human_right_arm,pick_white_box_3]
 assignment[human_right_arm,pick_white_box_4]
 assignment[ur5_on_guide,place_orange_box_1]
 assignment[ur5_on_guide,place_orange_box_2]
 assignment[ur5_on_guide,place_orange_box_3]
 assignment[ur5_on_guide,place_orange_box_4]
 assignment[human_right_arm,place_blue_box_1]
 assignment[human_right_arm,place_blue_box_2]
 assignment[ur5_on_guide,place_blue_box_3]
 assignment[ur5_on_guide,place_blue_box_4]
 assignment[human_right_arm,place_white_box_1]
 assignment[human_right_arm,place_white_box_2]
 assignment[human_right_arm,place_white_box_3]
 assignment[human_right_arm,place_white_box_4]
 same_agent[('pick_orange_box_1',_'pick_orange_box_2'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'pick_orange_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'pick_orange_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'pick_blue_box_1'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'pick_blue_box_2'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'pick_blue_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'pick_blue_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'place_orange_box_1'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'place_orange_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'place_orange_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'pick_orange_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'pick_orange_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'pick_blue_box_1'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'pick_blue_box_2'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'pick_blue_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'pick_blue_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'place_orange_box_1'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'place_orange_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'place_orange_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_3',_'pick_orange_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_3',_'pick_blue_box_1'),ur5_on_guide]
 same_agent[('pick_orange_box_3',_'pick_blue_box_2'),ur5_on_guide]
 same_agent[('pick_orange_box_3',_'pick_blue_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_3',_'pick_blue_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_3',_'place_orange_box_1'),ur5_on_guide]
 same_agent[('pick_orange_box_3',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('pick_orange_box_3',_'place_orange_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_3',_'place_orange_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_3',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_3',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_4',_'pick_blue_box_1'),ur5_on_guide]
 same_agent[('pick_orange_box_4',_'pick_blue_box_2'),ur5_on_guide]
 same_agent[('pick_orange_box_4',_'pick_blue_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_4',_'pick_blue_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_4',_'place_orange_box_1'),ur5_on_guide]
 same_agent[('pick_orange_box_4',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('pick_orange_box_4',_'place_orange_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_4',_'place_orange_box_4'),ur5_on_guide]
 same_agent[('pick_orange_box_4',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('pick_orange_box_4',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'place_orange_box_1'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'place_orange_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'place_orange_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_2',_'place_orange_box_1'),ur5_on_guide]
 same_agent[('pick_blue_box_2',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('pick_blue_box_2',_'place_orange_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_2',_'place_orange_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_2',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_2',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_3',_'place_orange_box_1'),ur5_on_guide]
 same_agent[('pick_blue_box_3',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('pick_blue_box_3',_'place_orange_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_3',_'place_orange_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_3',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_4',_'place_orange_box_1'),ur5_on_guide]
 same_agent[('pick_blue_box_4',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('pick_blue_box_4',_'place_orange_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_4',_'place_orange_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_4',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('pick_blue_box_4',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('place_orange_box_1',_'place_orange_box_2'),ur5_on_guide]
 same_agent[('place_orange_box_1',_'place_orange_box_3'),ur5_on_guide]
 same_agent[('place_orange_box_1',_'place_orange_box_4'),ur5_on_guide]
 same_agent[('place_orange_box_1',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('place_orange_box_1',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('place_orange_box_2',_'place_orange_box_3'),ur5_on_guide]
 same_agent[('place_orange_box_2',_'place_orange_box_4'),ur5_on_guide]
 same_agent[('place_orange_box_2',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('place_orange_box_2',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('place_orange_box_3',_'place_orange_box_4'),ur5_on_guide]
 same_agent[('place_orange_box_3',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('place_orange_box_3',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('place_orange_box_4',_'place_blue_box_3'),ur5_on_guide]
 same_agent[('place_orange_box_4',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('place_blue_box_3',_'place_blue_box_4'),ur5_on_guide]
 same_agent[('pick_blue_box_1',_'pick_blue_box_2'),human_right_arm]
 same_agent[('pick_blue_box_1',_'pick_blue_box_3'),human_right_arm]
 same_agent[('pick_blue_box_1',_'pick_blue_box_4'),human_right_arm]
 same_agent[('pick_blue_box_1',_'pick_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_1',_'pick_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_1',_'pick_white_box_3'),human_right_arm]
 same_agent[('pick_blue_box_1',_'pick_white_box_4'),human_right_arm]
 same_agent[('pick_blue_box_1',_'place_blue_box_1'),human_right_arm]
 same_agent[('pick_blue_box_1',_'place_blue_box_2'),human_right_arm]
 same_agent[('pick_blue_box_1',_'place_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_1',_'place_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_1',_'place_white_box_3'),human_right_arm]
 same_agent[('pick_blue_box_1',_'place_white_box_4'),human_right_arm]
 same_agent[('pick_blue_box_2',_'pick_blue_box_3'),human_right_arm]
 same_agent[('pick_blue_box_2',_'pick_blue_box_4'),human_right_arm]
 same_agent[('pick_blue_box_2',_'pick_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_2',_'pick_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_2',_'pick_white_box_3'),human_right_arm]
 same_agent[('pick_blue_box_2',_'pick_white_box_4'),human_right_arm]
 same_agent[('pick_blue_box_2',_'place_blue_box_1'),human_right_arm]
 same_agent[('pick_blue_box_2',_'place_blue_box_2'),human_right_arm]
 same_agent[('pick_blue_box_2',_'place_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_2',_'place_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_2',_'place_white_box_3'),human_right_arm]
 same_agent[('pick_blue_box_2',_'place_white_box_4'),human_right_arm]
 same_agent[('pick_blue_box_3',_'pick_blue_box_4'),human_right_arm]
 same_agent[('pick_blue_box_3',_'pick_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_3',_'pick_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_3',_'pick_white_box_3'),human_right_arm]
 same_agent[('pick_blue_box_3',_'pick_white_box_4'),human_right_arm]
 same_agent[('pick_blue_box_3',_'place_blue_box_1'),human_right_arm]
 same_agent[('pick_blue_box_3',_'place_blue_box_2'),human_right_arm]
 same_agent[('pick_blue_box_3',_'place_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_3',_'place_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_3',_'place_white_box_3'),human_right_arm]
 same_agent[('pick_blue_box_3',_'place_white_box_4'),human_right_arm]
 same_agent[('pick_blue_box_4',_'pick_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_4',_'pick_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_4',_'pick_white_box_3'),human_right_arm]
 same_agent[('pick_blue_box_4',_'pick_white_box_4'),human_right_arm]
 same_agent[('pick_blue_box_4',_'place_blue_box_1'),human_right_arm]
 same_agent[('pick_blue_box_4',_'place_blue_box_2'),human_right_arm]
 same_agent[('pick_blue_box_4',_'place_white_box_1'),human_right_arm]
 same_agent[('pick_blue_box_4',_'place_white_box_2'),human_right_arm]
 same_agent[('pick_blue_box_4',_'place_white_box_3'),human_right_arm]
 same_agent[('pick_blue_box_4',_'place_white_box_4'),human_right_arm]
 same_agent[('pick_white_box_1',_'pick_white_box_2'),human_right_arm]
 same_agent[('pick_white_box_1',_'pick_white_box_3'),human_right_arm]
 same_agent[('pick_white_box_1',_'pick_white_box_4'),human_right_arm]
 same_agent[('pick_white_box_1',_'place_blue_box_1'),human_right_arm]
 same_agent[('pick_white_box_1',_'place_blue_box_2'),human_right_arm]
 same_agent[('pick_white_box_1',_'place_white_box_1'),human_right_arm]
 same_agent[('pick_white_box_1',_'place_white_box_2'),human_right_arm]
 same_agent[('pick_white_box_1',_'place_white_box_3'),human_right_arm]
 same_agent[('pick_white_box_1',_'place_white_box_4'),human_right_arm]
 same_agent[('pick_white_box_2',_'pick_white_box_3'),human_right_arm]
 same_agent[('pick_white_box_2',_'pick_white_box_4'),human_right_arm]
 same_agent[('pick_white_box_2',_'place_blue_box_1'),human_right_arm]
 same_agent[('pick_white_box_2',_'place_blue_box_2'),human_right_arm]
 same_agent[('pick_white_box_2',_'place_white_box_1'),human_right_arm]
 same_agent[('pick_white_box_2',_'place_white_box_2'),human_right_arm]
 same_agent[('pick_white_box_2',_'place_white_box_3'),human_right_arm]
 same_agent[('pick_white_box_2',_'place_white_box_4'),human_right_arm]
 same_agent[('pick_white_box_3',_'pick_white_box_4'),human_right_arm]
 same_agent[('pick_white_box_3',_'place_blue_box_1'),human_right_arm]
 same_agent[('pick_white_box_3',_'place_blue_box_2'),human_right_arm]
 same_agent[('pick_white_box_3',_'place_white_box_1'),human_right_arm]
 same_agent[('pick_white_box_3',_'place_white_box_2'),human_right_arm]
 same_agent[('pick_white_box_3',_'place_white_box_3'),human_right_arm]
 same_agent[('pick_white_box_3',_'place_white_box_4'),human_right_arm]
 same_agent[('pick_white_box_4',_'place_blue_box_1'),human_right_arm]
 same_agent[('pick_white_box_4',_'place_blue_box_2'),human_right_arm]
 same_agent[('pick_white_box_4',_'place_white_box_1'),human_right_arm]
 same_agent[('pick_white_box_4',_'place_white_box_2'),human_right_arm]
 same_agent[('pick_white_box_4',_'place_white_box_3'),human_right_arm]
 same_agent[('pick_white_box_4',_'place_white_box_4'),human_right_arm]
 same_agent[('place_blue_box_1',_'place_blue_box_2'),human_right_arm]
 same_agent[('place_blue_box_1',_'place_white_box_1'),human_right_arm]
 same_agent[('place_blue_box_1',_'place_white_box_2'),human_right_arm]
 same_agent[('place_blue_box_1',_'place_white_box_3'),human_right_arm]
 same_agent[('place_blue_box_1',_'place_white_box_4'),human_right_arm]
 same_agent[('place_blue_box_2',_'place_white_box_1'),human_right_arm]
 same_agent[('place_blue_box_2',_'place_white_box_2'),human_right_arm]
 same_agent[('place_blue_box_2',_'place_white_box_3'),human_right_arm]
 same_agent[('place_blue_box_2',_'place_white_box_4'),human_right_arm]
 same_agent[('place_white_box_1',_'place_white_box_2'),human_right_arm]
 same_agent[('place_white_box_1',_'place_white_box_3'),human_right_arm]
 same_agent[('place_white_box_1',_'place_white_box_4'),human_right_arm]
 same_agent[('place_white_box_2',_'place_white_box_3'),human_right_arm]
 same_agent[('place_white_box_2',_'place_white_box_4'),human_right_arm]
 same_agent[('place_white_box_3',_'place_white_box_4'),human_right_arm]
 check_parallelism(pick_orange_box_1,pick_orange_box_2)
 check_parallelism(pick_orange_box_1,pick_orange_box_3)
 check_parallelism(pick_orange_box_1,pick_orange_box_4)
 check_parallelism(pick_orange_box_1,pick_blue_box_1)
 check_parallelism(pick_orange_box_1,pick_blue_box_2)
 check_parallelism(pick_orange_box_1,pick_blue_box_3)
 check_parallelism(pick_orange_box_1,pick_blue_box_4)
 check_parallelism(pick_orange_box_1,pick_white_box_1)
 check_parallelism(pick_orange_box_1,pick_white_box_2)
 check_parallelism(pick_orange_box_1,pick_white_box_3)
 check_parallelism(pick_orange_box_1,pick_white_box_4)
 check_parallelism(pick_orange_box_1,place_orange_box_1)
 check_parallelism(pick_orange_box_1,place_orange_box_2)
 check_parallelism(pick_orange_box_1,place_orange_box_3)
 check_parallelism(pick_orange_box_1,place_orange_box_4)
 check_parallelism(pick_orange_box_1,place_blue_box_1)
 check_parallelism(pick_orange_box_1,place_blue_box_2)
 check_parallelism(pick_orange_box_1,place_blue_box_3)
 check_parallelism(pick_orange_box_1,place_blue_box_4)
 check_parallelism(pick_orange_box_1,place_white_box_1)
 check_parallelism(pick_orange_box_1,place_white_box_2)
 check_parallelism(pick_orange_box_1,place_white_box_3)
 check_parallelism(pick_orange_box_1,place_white_box_4)
 check_parallelism(pick_orange_box_2,pick_orange_box_3)
 check_parallelism(pick_orange_box_2,pick_orange_box_4)
 check_parallelism(pick_orange_box_2,pick_blue_box_1)
 check_parallelism(pick_orange_box_2,pick_blue_box_2)
 check_parallelism(pick_orange_box_2,pick_blue_box_3)
 check_parallelism(pick_orange_box_2,pick_blue_box_4)
 check_parallelism(pick_orange_box_2,pick_white_box_1)
 check_parallelism(pick_orange_box_2,pick_white_box_2)
 check_parallelism(pick_orange_box_2,pick_white_box_3)
 check_parallelism(pick_orange_box_2,pick_white_box_4)
 check_parallelism(pick_orange_box_2,place_orange_box_1)
 check_parallelism(pick_orange_box_2,place_orange_box_2)
 check_parallelism(pick_orange_box_2,place_orange_box_3)
 check_parallelism(pick_orange_box_2,place_orange_box_4)
 check_parallelism(pick_orange_box_2,place_blue_box_1)
 check_parallelism(pick_orange_box_2,place_blue_box_2)
 check_parallelism(pick_orange_box_2,place_blue_box_3)
 check_parallelism(pick_orange_box_2,place_blue_box_4)
 check_parallelism(pick_orange_box_2,place_white_box_1)
 check_parallelism(pick_orange_box_2,place_white_box_2)
 check_parallelism(pick_orange_box_2,place_white_box_3)
 check_parallelism(pick_orange_box_2,place_white_box_4)
 check_parallelism(pick_orange_box_3,pick_orange_box_4)
 check_parallelism(pick_orange_box_3,pick_blue_box_1)
 check_parallelism(pick_orange_box_3,pick_blue_box_2)
 check_parallelism(pick_orange_box_3,pick_blue_box_3)
 check_parallelism(pick_orange_box_3,pick_blue_box_4)
 check_parallelism(pick_orange_box_3,pick_white_box_1)
 check_parallelism(pick_orange_box_3,pick_white_box_2)
 check_parallelism(pick_orange_box_3,pick_white_box_3)
 check_parallelism(pick_orange_box_3,pick_white_box_4)
 check_parallelism(pick_orange_box_3,place_orange_box_1)
 check_parallelism(pick_orange_box_3,place_orange_box_2)
 check_parallelism(pick_orange_box_3,place_orange_box_3)
 check_parallelism(pick_orange_box_3,place_orange_box_4)
 check_parallelism(pick_orange_box_3,place_blue_box_1)
 check_parallelism(pick_orange_box_3,place_blue_box_2)
 check_parallelism(pick_orange_box_3,place_blue_box_3)
 check_parallelism(pick_orange_box_3,place_blue_box_4)
 check_parallelism(pick_orange_box_3,place_white_box_1)
 check_parallelism(pick_orange_box_3,place_white_box_2)
 check_parallelism(pick_orange_box_3,place_white_box_3)
 check_parallelism(pick_orange_box_3,place_white_box_4)
 check_parallelism(pick_orange_box_4,pick_blue_box_1)
 check_parallelism(pick_orange_box_4,pick_blue_box_2)
 check_parallelism(pick_orange_box_4,pick_blue_box_3)
 check_parallelism(pick_orange_box_4,pick_blue_box_4)
 check_parallelism(pick_orange_box_4,pick_white_box_1)
 check_parallelism(pick_orange_box_4,pick_white_box_2)
 check_parallelism(pick_orange_box_4,pick_white_box_3)
 check_parallelism(pick_orange_box_4,pick_white_box_4)
 check_parallelism(pick_orange_box_4,place_orange_box_1)
 check_parallelism(pick_orange_box_4,place_orange_box_2)
 check_parallelism(pick_orange_box_4,place_orange_box_3)
 check_parallelism(pick_orange_box_4,place_orange_box_4)
 check_parallelism(pick_orange_box_4,place_blue_box_1)
 check_parallelism(pick_orange_box_4,place_blue_box_2)
 check_parallelism(pick_orange_box_4,place_blue_box_3)
 check_parallelism(pick_orange_box_4,place_blue_box_4)
 check_parallelism(pick_orange_box_4,place_white_box_1)
 check_parallelism(pick_orange_box_4,place_white_box_2)
 check_parallelism(pick_orange_box_4,place_white_box_3)
 check_parallelism(pick_orange_box_4,place_white_box_4)
 check_parallelism(pick_blue_box_1,pick_blue_box_2)
 check_parallelism(pick_blue_box_1,pick_blue_box_3)
 check_parallelism(pick_blue_box_1,pick_blue_box_4)
 check_parallelism(pick_blue_box_1,pick_white_box_1)
 check_parallelism(pick_blue_box_1,pick_white_box_2)
 check_parallelism(pick_blue_box_1,pick_white_box_3)
 check_parallelism(pick_blue_box_1,pick_white_box_4)
 check_parallelism(pick_blue_box_1,place_orange_box_1)
 check_parallelism(pick_blue_box_1,place_orange_box_2)
 check_parallelism(pick_blue_box_1,place_orange_box_3)
 check_parallelism(pick_blue_box_1,place_orange_box_4)
 check_parallelism(pick_blue_box_1,place_blue_box_1)
 check_parallelism(pick_blue_box_1,place_blue_box_2)
 check_parallelism(pick_blue_box_1,place_blue_box_3)
 check_parallelism(pick_blue_box_1,place_blue_box_4)
 check_parallelism(pick_blue_box_1,place_white_box_1)
 check_parallelism(pick_blue_box_1,place_white_box_2)
 check_parallelism(pick_blue_box_1,place_white_box_3)
 check_parallelism(pick_blue_box_1,place_white_box_4)
 check_parallelism(pick_blue_box_2,pick_blue_box_3)
 check_parallelism(pick_blue_box_2,pick_blue_box_4)
 check_parallelism(pick_blue_box_2,pick_white_box_1)
 check_parallelism(pick_blue_box_2,pick_white_box_2)
 check_parallelism(pick_blue_box_2,pick_white_box_3)
 check_parallelism(pick_blue_box_2,pick_white_box_4)
 check_parallelism(pick_blue_box_2,place_orange_box_1)
 check_parallelism(pick_blue_box_2,place_orange_box_2)
 check_parallelism(pick_blue_box_2,place_orange_box_3)
 check_parallelism(pick_blue_box_2,place_orange_box_4)
 check_parallelism(pick_blue_box_2,place_blue_box_1)
 check_parallelism(pick_blue_box_2,place_blue_box_2)
 check_parallelism(pick_blue_box_2,place_blue_box_3)
 check_parallelism(pick_blue_box_2,place_blue_box_4)
 check_parallelism(pick_blue_box_2,place_white_box_1)
 check_parallelism(pick_blue_box_2,place_white_box_2)
 check_parallelism(pick_blue_box_2,place_white_box_3)
 check_parallelism(pick_blue_box_2,place_white_box_4)
 check_parallelism(pick_blue_box_3,pick_blue_box_4)
 check_parallelism(pick_blue_box_3,pick_white_box_1)
 check_parallelism(pick_blue_box_3,pick_white_box_2)
 check_parallelism(pick_blue_box_3,pick_white_box_3)
 check_parallelism(pick_blue_box_3,pick_white_box_4)
 check_parallelism(pick_blue_box_3,place_orange_box_1)
 check_parallelism(pick_blue_box_3,place_orange_box_2)
 check_parallelism(pick_blue_box_3,place_orange_box_3)
 check_parallelism(pick_blue_box_3,place_orange_box_4)
 check_parallelism(pick_blue_box_3,place_blue_box_1)
 check_parallelism(pick_blue_box_3,place_blue_box_2)
 check_parallelism(pick_blue_box_3,place_blue_box_3)
 check_parallelism(pick_blue_box_3,place_blue_box_4)
 check_parallelism(pick_blue_box_3,place_white_box_1)
 check_parallelism(pick_blue_box_3,place_white_box_2)
 check_parallelism(pick_blue_box_3,place_white_box_3)
 check_parallelism(pick_blue_box_3,place_white_box_4)
 check_parallelism(pick_blue_box_4,pick_white_box_1)
 check_parallelism(pick_blue_box_4,pick_white_box_2)
 check_parallelism(pick_blue_box_4,pick_white_box_3)
 check_parallelism(pick_blue_box_4,pick_white_box_4)
 check_parallelism(pick_blue_box_4,place_orange_box_1)
 check_parallelism(pick_blue_box_4,place_orange_box_2)
 check_parallelism(pick_blue_box_4,place_orange_box_3)
 check_parallelism(pick_blue_box_4,place_orange_box_4)
 check_parallelism(pick_blue_box_4,place_blue_box_1)
 check_parallelism(pick_blue_box_4,place_blue_box_2)
 check_parallelism(pick_blue_box_4,place_blue_box_3)
 check_parallelism(pick_blue_box_4,place_blue_box_4)
 check_parallelism(pick_blue_box_4,place_white_box_1)
 check_parallelism(pick_blue_box_4,place_white_box_2)
 check_parallelism(pick_blue_box_4,place_white_box_3)
 check_parallelism(pick_blue_box_4,place_white_box_4)
 check_parallelism(pick_white_box_1,pick_white_box_2)
 check_parallelism(pick_white_box_1,pick_white_box_3)
 check_parallelism(pick_white_box_1,pick_white_box_4)
 check_parallelism(pick_white_box_1,place_orange_box_1)
 check_parallelism(pick_white_box_1,place_orange_box_2)
 check_parallelism(pick_white_box_1,place_orange_box_3)
 check_parallelism(pick_white_box_1,place_orange_box_4)
 check_parallelism(pick_white_box_1,place_blue_box_1)
 check_parallelism(pick_white_box_1,place_blue_box_2)
 check_parallelism(pick_white_box_1,place_blue_box_3)
 check_parallelism(pick_white_box_1,place_blue_box_4)
 check_parallelism(pick_white_box_1,place_white_box_1)
 check_parallelism(pick_white_box_1,place_white_box_2)
 check_parallelism(pick_white_box_1,place_white_box_3)
 check_parallelism(pick_white_box_1,place_white_box_4)
 check_parallelism(pick_white_box_2,pick_white_box_3)
 check_parallelism(pick_white_box_2,pick_white_box_4)
 check_parallelism(pick_white_box_2,place_orange_box_1)
 check_parallelism(pick_white_box_2,place_orange_box_2)
 check_parallelism(pick_white_box_2,place_orange_box_3)
 check_parallelism(pick_white_box_2,place_orange_box_4)
 check_parallelism(pick_white_box_2,place_blue_box_1)
 check_parallelism(pick_white_box_2,place_blue_box_2)
 check_parallelism(pick_white_box_2,place_blue_box_3)
 check_parallelism(pick_white_box_2,place_blue_box_4)
 check_parallelism(pick_white_box_2,place_white_box_1)
 check_parallelism(pick_white_box_2,place_white_box_2)
 check_parallelism(pick_white_box_2,place_white_box_3)
 check_parallelism(pick_white_box_2,place_white_box_4)
 check_parallelism(pick_white_box_3,pick_white_box_4)
 check_parallelism(pick_white_box_3,place_orange_box_1)
 check_parallelism(pick_white_box_3,place_orange_box_2)
 check_parallelism(pick_white_box_3,place_orange_box_3)
 check_parallelism(pick_white_box_3,place_orange_box_4)
 check_parallelism(pick_white_box_3,place_blue_box_1)
 check_parallelism(pick_white_box_3,place_blue_box_2)
 check_parallelism(pick_white_box_3,place_blue_box_3)
 check_parallelism(pick_white_box_3,place_blue_box_4)
 check_parallelism(pick_white_box_3,place_white_box_1)
 check_parallelism(pick_white_box_3,place_white_box_2)
 check_parallelism(pick_white_box_3,place_white_box_3)
 check_parallelism(pick_white_box_3,place_white_box_4)
 check_parallelism(pick_white_box_4,place_orange_box_1)
 check_parallelism(pick_white_box_4,place_orange_box_2)
 check_parallelism(pick_white_box_4,place_orange_box_3)
 check_parallelism(pick_white_box_4,place_orange_box_4)
 check_parallelism(pick_white_box_4,place_blue_box_1)
 check_parallelism(pick_white_box_4,place_blue_box_2)
 check_parallelism(pick_white_box_4,place_blue_box_3)
 check_parallelism(pick_white_box_4,place_blue_box_4)
 check_parallelism(pick_white_box_4,place_white_box_1)
 check_parallelism(pick_white_box_4,place_white_box_2)
 check_parallelism(pick_white_box_4,place_white_box_3)
 check_parallelism(pick_white_box_4,place_white_box_4)
 check_parallelism(place_orange_box_1,place_orange_box_2)
 check_parallelism(place_orange_box_1,place_orange_box_3)
 check_parallelism(place_orange_box_1,place_orange_box_4)
 check_parallelism(place_orange_box_1,place_blue_box_1)
 check_parallelism(place_orange_box_1,place_blue_box_2)
 check_parallelism(place_orange_box_1,place_blue_box_3)
 check_parallelism(place_orange_box_1,place_blue_box_4)
 check_parallelism(place_orange_box_1,place_white_box_1)
 check_parallelism(place_orange_box_1,place_white_box_2)
 check_parallelism(place_orange_box_1,place_white_box_3)
 check_parallelism(place_orange_box_1,place_white_box_4)
 check_parallelism(place_orange_box_2,place_orange_box_3)
 check_parallelism(place_orange_box_2,place_orange_box_4)
 check_parallelism(place_orange_box_2,place_blue_box_1)
 check_parallelism(place_orange_box_2,place_blue_box_2)
 check_parallelism(place_orange_box_2,place_blue_box_3)
 check_parallelism(place_orange_box_2,place_blue_box_4)
 check_parallelism(place_orange_box_2,place_white_box_1)
 check_parallelism(place_orange_box_2,place_white_box_2)
 check_parallelism(place_orange_box_2,place_white_box_3)
 check_parallelism(place_orange_box_2,place_white_box_4)
 check_parallelism(place_orange_box_3,place_orange_box_4)
 check_parallelism(place_orange_box_3,place_blue_box_1)
 check_parallelism(place_orange_box_3,place_blue_box_2)
 check_parallelism(place_orange_box_3,place_blue_box_3)
 check_parallelism(place_orange_box_3,place_blue_box_4)
 check_parallelism(place_orange_box_3,place_white_box_1)
 check_parallelism(place_orange_box_3,place_white_box_2)
 check_parallelism(place_orange_box_3,place_white_box_3)
 check_parallelism(place_orange_box_3,place_white_box_4)
 check_parallelism(place_orange_box_4,place_blue_box_1)
 check_parallelism(place_orange_box_4,place_blue_box_2)
 check_parallelism(place_orange_box_4,place_blue_box_3)
 check_parallelism(place_orange_box_4,place_blue_box_4)
 check_parallelism(place_orange_box_4,place_white_box_1)
 check_parallelism(place_orange_box_4,place_white_box_2)
 check_parallelism(place_orange_box_4,place_white_box_3)
 check_parallelism(place_orange_box_4,place_white_box_4)
 check_parallelism(place_blue_box_1,place_blue_box_2)
 check_parallelism(place_blue_box_1,place_blue_box_3)
 check_parallelism(place_blue_box_1,place_blue_box_4)
 check_parallelism(place_blue_box_1,place_white_box_1)
 check_parallelism(place_blue_box_1,place_white_box_2)
 check_parallelism(place_blue_box_1,place_white_box_3)
 check_parallelism(place_blue_box_1,place_white_box_4)
 check_parallelism(place_blue_box_2,place_blue_box_3)
 check_parallelism(place_blue_box_2,place_blue_box_4)
 check_parallelism(place_blue_box_2,place_white_box_1)
 check_parallelism(place_blue_box_2,place_white_box_2)
 check_parallelism(place_blue_box_2,place_white_box_3)
 check_parallelism(place_blue_box_2,place_white_box_4)
 check_parallelism(place_blue_box_3,place_blue_box_4)
 check_parallelism(place_blue_box_3,place_white_box_1)
 check_parallelism(place_blue_box_3,place_white_box_2)
 check_parallelism(place_blue_box_3,place_white_box_3)
 check_parallelism(place_blue_box_3,place_white_box_4)
 check_parallelism(place_blue_box_4,place_white_box_1)
 check_parallelism(place_blue_box_4,place_white_box_2)
 check_parallelism(place_blue_box_4,place_white_box_3)
 check_parallelism(place_blue_box_4,place_white_box_4)
 check_parallelism(place_white_box_1,place_white_box_2)
 check_parallelism(place_white_box_1,place_white_box_3)
 check_parallelism(place_white_box_1,place_white_box_4)
 check_parallelism(place_white_box_2,place_white_box_3)
 check_parallelism(place_white_box_2,place_white_box_4)
 check_parallelism(place_white_box_3,place_white_box_4)
 both_pick_orange_box_1_pick_orange_box_2_to_ur5_on_guide
 both_pick_orange_box_1_pick_orange_box_3_to_ur5_on_guide
 both_pick_orange_box_1_pick_orange_box_4_to_ur5_on_guide
 both_pick_orange_box_1_pick_blue_box_1_to_ur5_on_guide
 both_pick_orange_box_1_pick_blue_box_2_to_ur5_on_guide
 both_pick_orange_box_1_pick_blue_box_3_to_ur5_on_guide
 both_pick_orange_box_1_pick_blue_box_4_to_ur5_on_guide
 both_pick_orange_box_1_place_orange_box_1_to_ur5_on_guide
 both_pick_orange_box_1_place_orange_box_2_to_ur5_on_guide
 both_pick_orange_box_1_place_orange_box_3_to_ur5_on_guide
 both_pick_orange_box_1_place_orange_box_4_to_ur5_on_guide
 both_pick_orange_box_1_place_blue_box_3_to_ur5_on_guide
 both_pick_orange_box_1_place_blue_box_4_to_ur5_on_guide
 both_pick_orange_box_2_pick_orange_box_3_to_ur5_on_guide
 both_pick_orange_box_2_pick_orange_box_4_to_ur5_on_guide
 both_pick_orange_box_2_pick_blue_box_1_to_ur5_on_guide
 both_pick_orange_box_2_pick_blue_box_2_to_ur5_on_guide
 both_pick_orange_box_2_pick_blue_box_3_to_ur5_on_guide
 both_pick_orange_box_2_pick_blue_box_4_to_ur5_on_guide
 both_pick_orange_box_2_place_orange_box_1_to_ur5_on_guide
 both_pick_orange_box_2_place_orange_box_2_to_ur5_on_guide
 both_pick_orange_box_2_place_orange_box_3_to_ur5_on_guide
 both_pick_orange_box_2_place_orange_box_4_to_ur5_on_guide
 both_pick_orange_box_2_place_blue_box_3_to_ur5_on_guide
 both_pick_orange_box_2_place_blue_box_4_to_ur5_on_guide
 both_pick_orange_box_3_pick_orange_box_4_to_ur5_on_guide
 both_pick_orange_box_3_pick_blue_box_1_to_ur5_on_guide
 both_pick_orange_box_3_pick_blue_box_2_to_ur5_on_guide
 both_pick_orange_box_3_pick_blue_box_3_to_ur5_on_guide
 both_pick_orange_box_3_pick_blue_box_4_to_ur5_on_guide
 both_pick_orange_box_3_place_orange_box_1_to_ur5_on_guide
 both_pick_orange_box_3_place_orange_box_2_to_ur5_on_guide
 both_pick_orange_box_3_place_orange_box_3_to_ur5_on_guide
 both_pick_orange_box_3_place_orange_box_4_to_ur5_on_guide
 both_pick_orange_box_3_place_blue_box_3_to_ur5_on_guide
 both_pick_orange_box_3_place_blue_box_4_to_ur5_on_guide
 both_pick_orange_box_4_pick_blue_box_1_to_ur5_on_guide
 both_pick_orange_box_4_pick_blue_box_2_to_ur5_on_guide
 both_pick_orange_box_4_pick_blue_box_3_to_ur5_on_guide
 both_pick_orange_box_4_pick_blue_box_4_to_ur5_on_guide
 both_pick_orange_box_4_place_orange_box_1_to_ur5_on_guide
 both_pick_orange_box_4_place_orange_box_2_to_ur5_on_guide
 both_pick_orange_box_4_place_orange_box_3_to_ur5_on_guide
 both_pick_orange_box_4_place_orange_box_4_to_ur5_on_guide
 both_pick_orange_box_4_place_blue_box_3_to_ur5_on_guide
 both_pick_orange_box_4_place_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_1_pick_blue_box_2_to_ur5_on_guide
 both_pick_blue_box_1_pick_blue_box_3_to_ur5_on_guide
 both_pick_blue_box_1_pick_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_1_place_orange_box_1_to_ur5_on_guide
 both_pick_blue_box_1_place_orange_box_2_to_ur5_on_guide
 both_pick_blue_box_1_place_orange_box_3_to_ur5_on_guide
 both_pick_blue_box_1_place_orange_box_4_to_ur5_on_guide
 both_pick_blue_box_1_place_blue_box_3_to_ur5_on_guide
 both_pick_blue_box_1_place_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_2_pick_blue_box_3_to_ur5_on_guide
 both_pick_blue_box_2_pick_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_2_place_orange_box_1_to_ur5_on_guide
 both_pick_blue_box_2_place_orange_box_2_to_ur5_on_guide
 both_pick_blue_box_2_place_orange_box_3_to_ur5_on_guide
 both_pick_blue_box_2_place_orange_box_4_to_ur5_on_guide
 both_pick_blue_box_2_place_blue_box_3_to_ur5_on_guide
 both_pick_blue_box_2_place_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_3_pick_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_3_place_orange_box_1_to_ur5_on_guide
 both_pick_blue_box_3_place_orange_box_2_to_ur5_on_guide
 both_pick_blue_box_3_place_orange_box_3_to_ur5_on_guide
 both_pick_blue_box_3_place_orange_box_4_to_ur5_on_guide
 both_pick_blue_box_3_place_blue_box_3_to_ur5_on_guide
 both_pick_blue_box_3_place_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_4_place_orange_box_1_to_ur5_on_guide
 both_pick_blue_box_4_place_orange_box_2_to_ur5_on_guide
 both_pick_blue_box_4_place_orange_box_3_to_ur5_on_guide
 both_pick_blue_box_4_place_orange_box_4_to_ur5_on_guide
 both_pick_blue_box_4_place_blue_box_3_to_ur5_on_guide
 both_pick_blue_box_4_place_blue_box_4_to_ur5_on_guide
 both_place_orange_box_1_place_orange_box_2_to_ur5_on_guide
 both_place_orange_box_1_place_orange_box_3_to_ur5_on_guide
 both_place_orange_box_1_place_orange_box_4_to_ur5_on_guide
 both_place_orange_box_1_place_blue_box_3_to_ur5_on_guide
 both_place_orange_box_1_place_blue_box_4_to_ur5_on_guide
 both_place_orange_box_2_place_orange_box_3_to_ur5_on_guide
 both_place_orange_box_2_place_orange_box_4_to_ur5_on_guide
 both_place_orange_box_2_place_blue_box_3_to_ur5_on_guide
 both_place_orange_box_2_place_blue_box_4_to_ur5_on_guide
 both_place_orange_box_3_place_orange_box_4_to_ur5_on_guide
 both_place_orange_box_3_place_blue_box_3_to_ur5_on_guide
 both_place_orange_box_3_place_blue_box_4_to_ur5_on_guide
 both_place_orange_box_4_place_blue_box_3_to_ur5_on_guide
 both_place_orange_box_4_place_blue_box_4_to_ur5_on_guide
 both_place_blue_box_3_place_blue_box_4_to_ur5_on_guide
 both_pick_blue_box_1_pick_blue_box_2_to_human_right_arm
 both_pick_blue_box_1_pick_blue_box_3_to_human_right_arm
 both_pick_blue_box_1_pick_blue_box_4_to_human_right_arm
 both_pick_blue_box_1_pick_white_box_1_to_human_right_arm
 both_pick_blue_box_1_pick_white_box_2_to_human_right_arm
 both_pick_blue_box_1_pick_white_box_3_to_human_right_arm
 both_pick_blue_box_1_pick_white_box_4_to_human_right_arm
 both_pick_blue_box_1_place_blue_box_1_to_human_right_arm
 both_pick_blue_box_1_place_blue_box_2_to_human_right_arm
 both_pick_blue_box_1_place_white_box_1_to_human_right_arm
 both_pick_blue_box_1_place_white_box_2_to_human_right_arm
 both_pick_blue_box_1_place_white_box_3_to_human_right_arm
 both_pick_blue_box_1_place_white_box_4_to_human_right_arm
 both_pick_blue_box_2_pick_blue_box_3_to_human_right_arm
 both_pick_blue_box_2_pick_blue_box_4_to_human_right_arm
 both_pick_blue_box_2_pick_white_box_1_to_human_right_arm
 both_pick_blue_box_2_pick_white_box_2_to_human_right_arm
 both_pick_blue_box_2_pick_white_box_3_to_human_right_arm
 both_pick_blue_box_2_pick_white_box_4_to_human_right_arm
 both_pick_blue_box_2_place_blue_box_1_to_human_right_arm
 both_pick_blue_box_2_place_blue_box_2_to_human_right_arm
 both_pick_blue_box_2_place_white_box_1_to_human_right_arm
 both_pick_blue_box_2_place_white_box_2_to_human_right_arm
 both_pick_blue_box_2_place_white_box_3_to_human_right_arm
 both_pick_blue_box_2_place_white_box_4_to_human_right_arm
 both_pick_blue_box_3_pick_blue_box_4_to_human_right_arm
 both_pick_blue_box_3_pick_white_box_1_to_human_right_arm
 both_pick_blue_box_3_pick_white_box_2_to_human_right_arm
 both_pick_blue_box_3_pick_white_box_3_to_human_right_arm
 both_pick_blue_box_3_pick_white_box_4_to_human_right_arm
 both_pick_blue_box_3_place_blue_box_1_to_human_right_arm
 both_pick_blue_box_3_place_blue_box_2_to_human_right_arm
 both_pick_blue_box_3_place_white_box_1_to_human_right_arm
 both_pick_blue_box_3_place_white_box_2_to_human_right_arm
 both_pick_blue_box_3_place_white_box_3_to_human_right_arm
 both_pick_blue_box_3_place_white_box_4_to_human_right_arm
 both_pick_blue_box_4_pick_white_box_1_to_human_right_arm
 both_pick_blue_box_4_pick_white_box_2_to_human_right_arm
 both_pick_blue_box_4_pick_white_box_3_to_human_right_arm
 both_pick_blue_box_4_pick_white_box_4_to_human_right_arm
 both_pick_blue_box_4_place_blue_box_1_to_human_right_arm
 both_pick_blue_box_4_place_blue_box_2_to_human_right_arm
 both_pick_blue_box_4_place_white_box_1_to_human_right_arm
 both_pick_blue_box_4_place_white_box_2_to_human_right_arm
 both_pick_blue_box_4_place_white_box_3_to_human_right_arm
 both_pick_blue_box_4_place_white_box_4_to_human_right_arm
 both_pick_white_box_1_pick_white_box_2_to_human_right_arm
 both_pick_white_box_1_pick_white_box_3_to_human_right_arm
 both_pick_white_box_1_pick_white_box_4_to_human_right_arm
 both_pick_white_box_1_place_blue_box_1_to_human_right_arm
 both_pick_white_box_1_place_blue_box_2_to_human_right_arm
 both_pick_white_box_1_place_white_box_1_to_human_right_arm
 both_pick_white_box_1_place_white_box_2_to_human_right_arm
 both_pick_white_box_1_place_white_box_3_to_human_right_arm
 both_pick_white_box_1_place_white_box_4_to_human_right_arm
 both_pick_white_box_2_pick_white_box_3_to_human_right_arm
 both_pick_white_box_2_pick_white_box_4_to_human_right_arm
 both_pick_white_box_2_place_blue_box_1_to_human_right_arm
 both_pick_white_box_2_place_blue_box_2_to_human_right_arm
 both_pick_white_box_2_place_white_box_1_to_human_right_arm
 both_pick_white_box_2_place_white_box_2_to_human_right_arm
 both_pick_white_box_2_place_white_box_3_to_human_right_arm
 both_pick_white_box_2_place_white_box_4_to_human_right_arm
 both_pick_white_box_3_pick_white_box_4_to_human_right_arm
 both_pick_white_box_3_place_blue_box_1_to_human_right_arm
 both_pick_white_box_3_place_blue_box_2_to_human_right_arm
 both_pick_white_box_3_place_white_box_1_to_human_right_arm
 both_pick_white_box_3_place_white_box_2_to_human_right_arm
 both_pick_white_box_3_place_white_box_3_to_human_right_arm
 both_pick_white_box_3_place_white_box_4_to_human_right_arm
 both_pick_white_box_4_place_blue_box_1_to_human_right_arm
 both_pick_white_box_4_place_blue_box_2_to_human_right_arm
 both_pick_white_box_4_place_white_box_1_to_human_right_arm
 both_pick_white_box_4_place_white_box_2_to_human_right_arm
 both_pick_white_box_4_place_white_box_3_to_human_right_arm
 both_pick_white_box_4_place_white_box_4_to_human_right_arm
 both_place_blue_box_1_place_blue_box_2_to_human_right_arm
 both_place_blue_box_1_place_white_box_1_to_human_right_arm
 both_place_blue_box_1_place_white_box_2_to_human_right_arm
 both_place_blue_box_1_place_white_box_3_to_human_right_arm
 both_place_blue_box_1_place_white_box_4_to_human_right_arm
 both_place_blue_box_2_place_white_box_1_to_human_right_arm
 both_place_blue_box_2_place_white_box_2_to_human_right_arm
 both_place_blue_box_2_place_white_box_3_to_human_right_arm
 both_place_blue_box_2_place_white_box_4_to_human_right_arm
 both_place_white_box_1_place_white_box_2_to_human_right_arm
 both_place_white_box_1_place_white_box_3_to_human_right_arm
 both_place_white_box_1_place_white_box_4_to_human_right_arm
 both_place_white_box_2_place_white_box_3_to_human_right_arm
 both_place_white_box_2_place_white_box_4_to_human_right_arm
 both_place_white_box_3_place_white_box_4_to_human_right_arm
General Constraints
 GC0: min_t_end(pick_orange_box_1,_pick_orange_box_2) = MIN (
   t_end[pick_orange_box_1] , t_end[pick_orange_box_2] )
 GC1: max_t_start(pick_orange_box_1,_pick_orange_box_2) = MAX (
   t_start[pick_orange_box_1] , t_start[pick_orange_box_2] )
 GC2: min_t_end(pick_orange_box_1,_pick_orange_box_3) = MIN (
   t_end[pick_orange_box_1] , t_end[pick_orange_box_3] )
 GC3: max_t_start(pick_orange_box_1,_pick_orange_box_3) = MAX (
   t_start[pick_orange_box_1] , t_start[pick_orange_box_3] )
 GC4: min_t_end(pick_orange_box_1,_pick_orange_box_4) = MIN (
   t_end[pick_orange_box_1] , t_end[pick_orange_box_4] )
 GC5: max_t_start(pick_orange_box_1,_pick_orange_box_4) = MAX (
   t_start[pick_orange_box_1] , t_start[pick_orange_box_4] )
 GC6: min_t_end(pick_orange_box_1,_pick_blue_box_1) = MIN (
   t_end[pick_orange_box_1] , t_end[pick_blue_box_1] )
 GC7: max_t_start(pick_orange_box_1,_pick_blue_box_1) = MAX (
   t_start[pick_orange_box_1] , t_start[pick_blue_box_1] )
 GC8: min_t_end(pick_orange_box_1,_pick_blue_box_2) = MIN (
   t_end[pick_orange_box_1] , t_end[pick_blue_box_2] )
 GC9: max_t_start(pick_orange_box_1,_pick_blue_box_2) = MAX (
   t_start[pick_orange_box_1] , t_start[pick_blue_box_2] )
 GC10: min_t_end(pick_orange_box_1,_pick_blue_box_3) = MIN (
   t_end[pick_orange_box_1] , t_end[pick_blue_box_3] )
 GC11: max_t_start(pick_orange_box_1,_pick_blue_box_3) = MAX (
   t_start[pick_orange_box_1] , t_start[pick_blue_box_3] )
 GC12: min_t_end(pick_orange_box_1,_pick_blue_box_4) = MIN (
   t_end[pick_orange_box_1] , t_end[pick_blue_box_4] )
 GC13: max_t_start(pick_orange_box_1,_pick_blue_box_4) = MAX (
   t_start[pick_orange_box_1] , t_start[pick_blue_box_4] )
 GC14: min_t_end(pick_orange_box_1,_pick_white_box_1) = MIN (
   t_end[pick_orange_box_1] , t_end[pick_white_box_1] )
 GC15: max_t_start(pick_orange_box_1,_pick_white_box_1) = MAX (
   t_start[pick_orange_box_1] , t_start[pick_white_box_1] )
 GC16: min_t_end(pick_orange_box_1,_pick_white_box_2) = MIN (
   t_end[pick_orange_box_1] , t_end[pick_white_box_2] )
 GC17: max_t_start(pick_orange_box_1,_pick_white_box_2) = MAX (
   t_start[pick_orange_box_1] , t_start[pick_white_box_2] )
 GC18: min_t_end(pick_orange_box_1,_pick_white_box_3) = MIN (
   t_end[pick_orange_box_1] , t_end[pick_white_box_3] )
 GC19: max_t_start(pick_orange_box_1,_pick_white_box_3) = MAX (
   t_start[pick_orange_box_1] , t_start[pick_white_box_3] )
 GC20: min_t_end(pick_orange_box_1,_pick_white_box_4) = MIN (
   t_end[pick_orange_box_1] , t_end[pick_white_box_4] )
 GC21: max_t_start(pick_orange_box_1,_pick_white_box_4) = MAX (
   t_start[pick_orange_box_1] , t_start[pick_white_box_4] )
 GC22: min_t_end(pick_orange_box_1,_place_orange_box_1) = MIN (
   t_end[pick_orange_box_1] , t_end[place_orange_box_1] )
 GC23: max_t_start(pick_orange_box_1,_place_orange_box_1) = MAX (
   t_start[pick_orange_box_1] , t_start[place_orange_box_1] )
 GC24: min_t_end(pick_orange_box_1,_place_orange_box_2) = MIN (
   t_end[pick_orange_box_1] , t_end[place_orange_box_2] )
 GC25: max_t_start(pick_orange_box_1,_place_orange_box_2) = MAX (
   t_start[pick_orange_box_1] , t_start[place_orange_box_2] )
 GC26: min_t_end(pick_orange_box_1,_place_orange_box_3) = MIN (
   t_end[pick_orange_box_1] , t_end[place_orange_box_3] )
 GC27: max_t_start(pick_orange_box_1,_place_orange_box_3) = MAX (
   t_start[pick_orange_box_1] , t_start[place_orange_box_3] )
 GC28: min_t_end(pick_orange_box_1,_place_orange_box_4) = MIN (
   t_end[pick_orange_box_1] , t_end[place_orange_box_4] )
 GC29: max_t_start(pick_orange_box_1,_place_orange_box_4) = MAX (
   t_start[pick_orange_box_1] , t_start[place_orange_box_4] )
 GC30: min_t_end(pick_orange_box_1,_place_blue_box_1) = MIN (
   t_end[pick_orange_box_1] , t_end[place_blue_box_1] )
 GC31: max_t_start(pick_orange_box_1,_place_blue_box_1) = MAX (
   t_start[pick_orange_box_1] , t_start[place_blue_box_1] )
 GC32: min_t_end(pick_orange_box_1,_place_blue_box_2) = MIN (
   t_end[pick_orange_box_1] , t_end[place_blue_box_2] )
 GC33: max_t_start(pick_orange_box_1,_place_blue_box_2) = MAX (
   t_start[pick_orange_box_1] , t_start[place_blue_box_2] )
 GC34: min_t_end(pick_orange_box_1,_place_blue_box_3) = MIN (
   t_end[pick_orange_box_1] , t_end[place_blue_box_3] )
 GC35: max_t_start(pick_orange_box_1,_place_blue_box_3) = MAX (
   t_start[pick_orange_box_1] , t_start[place_blue_box_3] )
 GC36: min_t_end(pick_orange_box_1,_place_blue_box_4) = MIN (
   t_end[pick_orange_box_1] , t_end[place_blue_box_4] )
 GC37: max_t_start(pick_orange_box_1,_place_blue_box_4) = MAX (
   t_start[pick_orange_box_1] , t_start[place_blue_box_4] )
 GC38: min_t_end(pick_orange_box_1,_place_white_box_1) = MIN (
   t_end[pick_orange_box_1] , t_end[place_white_box_1] )
 GC39: max_t_start(pick_orange_box_1,_place_white_box_1) = MAX (
   t_start[pick_orange_box_1] , t_start[place_white_box_1] )
 GC40: min_t_end(pick_orange_box_1,_place_white_box_2) = MIN (
   t_end[pick_orange_box_1] , t_end[place_white_box_2] )
 GC41: max_t_start(pick_orange_box_1,_place_white_box_2) = MAX (
   t_start[pick_orange_box_1] , t_start[place_white_box_2] )
 GC42: min_t_end(pick_orange_box_1,_place_white_box_3) = MIN (
   t_end[pick_orange_box_1] , t_end[place_white_box_3] )
 GC43: max_t_start(pick_orange_box_1,_place_white_box_3) = MAX (
   t_start[pick_orange_box_1] , t_start[place_white_box_3] )
 GC44: min_t_end(pick_orange_box_1,_place_white_box_4) = MIN (
   t_end[pick_orange_box_1] , t_end[place_white_box_4] )
 GC45: max_t_start(pick_orange_box_1,_place_white_box_4) = MAX (
   t_start[pick_orange_box_1] , t_start[place_white_box_4] )
 GC46: min_t_end(pick_orange_box_2,_pick_orange_box_3) = MIN (
   t_end[pick_orange_box_2] , t_end[pick_orange_box_3] )
 GC47: max_t_start(pick_orange_box_2,_pick_orange_box_3) = MAX (
   t_start[pick_orange_box_2] , t_start[pick_orange_box_3] )
 GC48: min_t_end(pick_orange_box_2,_pick_orange_box_4) = MIN (
   t_end[pick_orange_box_2] , t_end[pick_orange_box_4] )
 GC49: max_t_start(pick_orange_box_2,_pick_orange_box_4) = MAX (
   t_start[pick_orange_box_2] , t_start[pick_orange_box_4] )
 GC50: min_t_end(pick_orange_box_2,_pick_blue_box_1) = MIN (
   t_end[pick_orange_box_2] , t_end[pick_blue_box_1] )
 GC51: max_t_start(pick_orange_box_2,_pick_blue_box_1) = MAX (
   t_start[pick_orange_box_2] , t_start[pick_blue_box_1] )
 GC52: min_t_end(pick_orange_box_2,_pick_blue_box_2) = MIN (
   t_end[pick_orange_box_2] , t_end[pick_blue_box_2] )
 GC53: max_t_start(pick_orange_box_2,_pick_blue_box_2) = MAX (
   t_start[pick_orange_box_2] , t_start[pick_blue_box_2] )
 GC54: min_t_end(pick_orange_box_2,_pick_blue_box_3) = MIN (
   t_end[pick_orange_box_2] , t_end[pick_blue_box_3] )
 GC55: max_t_start(pick_orange_box_2,_pick_blue_box_3) = MAX (
   t_start[pick_orange_box_2] , t_start[pick_blue_box_3] )
 GC56: min_t_end(pick_orange_box_2,_pick_blue_box_4) = MIN (
   t_end[pick_orange_box_2] , t_end[pick_blue_box_4] )
 GC57: max_t_start(pick_orange_box_2,_pick_blue_box_4) = MAX (
   t_start[pick_orange_box_2] , t_start[pick_blue_box_4] )
 GC58: min_t_end(pick_orange_box_2,_pick_white_box_1) = MIN (
   t_end[pick_orange_box_2] , t_end[pick_white_box_1] )
 GC59: max_t_start(pick_orange_box_2,_pick_white_box_1) = MAX (
   t_start[pick_orange_box_2] , t_start[pick_white_box_1] )
 GC60: min_t_end(pick_orange_box_2,_pick_white_box_2) = MIN (
   t_end[pick_orange_box_2] , t_end[pick_white_box_2] )
 GC61: max_t_start(pick_orange_box_2,_pick_white_box_2) = MAX (
   t_start[pick_orange_box_2] , t_start[pick_white_box_2] )
 GC62: min_t_end(pick_orange_box_2,_pick_white_box_3) = MIN (
   t_end[pick_orange_box_2] , t_end[pick_white_box_3] )
 GC63: max_t_start(pick_orange_box_2,_pick_white_box_3) = MAX (
   t_start[pick_orange_box_2] , t_start[pick_white_box_3] )
 GC64: min_t_end(pick_orange_box_2,_pick_white_box_4) = MIN (
   t_end[pick_orange_box_2] , t_end[pick_white_box_4] )
 GC65: max_t_start(pick_orange_box_2,_pick_white_box_4) = MAX (
   t_start[pick_orange_box_2] , t_start[pick_white_box_4] )
 GC66: min_t_end(pick_orange_box_2,_place_orange_box_1) = MIN (
   t_end[pick_orange_box_2] , t_end[place_orange_box_1] )
 GC67: max_t_start(pick_orange_box_2,_place_orange_box_1) = MAX (
   t_start[pick_orange_box_2] , t_start[place_orange_box_1] )
 GC68: min_t_end(pick_orange_box_2,_place_orange_box_2) = MIN (
   t_end[pick_orange_box_2] , t_end[place_orange_box_2] )
 GC69: max_t_start(pick_orange_box_2,_place_orange_box_2) = MAX (
   t_start[pick_orange_box_2] , t_start[place_orange_box_2] )
 GC70: min_t_end(pick_orange_box_2,_place_orange_box_3) = MIN (
   t_end[pick_orange_box_2] , t_end[place_orange_box_3] )
 GC71: max_t_start(pick_orange_box_2,_place_orange_box_3) = MAX (
   t_start[pick_orange_box_2] , t_start[place_orange_box_3] )
 GC72: min_t_end(pick_orange_box_2,_place_orange_box_4) = MIN (
   t_end[pick_orange_box_2] , t_end[place_orange_box_4] )
 GC73: max_t_start(pick_orange_box_2,_place_orange_box_4) = MAX (
   t_start[pick_orange_box_2] , t_start[place_orange_box_4] )
 GC74: min_t_end(pick_orange_box_2,_place_blue_box_1) = MIN (
   t_end[pick_orange_box_2] , t_end[place_blue_box_1] )
 GC75: max_t_start(pick_orange_box_2,_place_blue_box_1) = MAX (
   t_start[pick_orange_box_2] , t_start[place_blue_box_1] )
 GC76: min_t_end(pick_orange_box_2,_place_blue_box_2) = MIN (
   t_end[pick_orange_box_2] , t_end[place_blue_box_2] )
 GC77: max_t_start(pick_orange_box_2,_place_blue_box_2) = MAX (
   t_start[pick_orange_box_2] , t_start[place_blue_box_2] )
 GC78: min_t_end(pick_orange_box_2,_place_blue_box_3) = MIN (
   t_end[pick_orange_box_2] , t_end[place_blue_box_3] )
 GC79: max_t_start(pick_orange_box_2,_place_blue_box_3) = MAX (
   t_start[pick_orange_box_2] , t_start[place_blue_box_3] )
 GC80: min_t_end(pick_orange_box_2,_place_blue_box_4) = MIN (
   t_end[pick_orange_box_2] , t_end[place_blue_box_4] )
 GC81: max_t_start(pick_orange_box_2,_place_blue_box_4) = MAX (
   t_start[pick_orange_box_2] , t_start[place_blue_box_4] )
 GC82: min_t_end(pick_orange_box_2,_place_white_box_1) = MIN (
   t_end[pick_orange_box_2] , t_end[place_white_box_1] )
 GC83: max_t_start(pick_orange_box_2,_place_white_box_1) = MAX (
   t_start[pick_orange_box_2] , t_start[place_white_box_1] )
 GC84: min_t_end(pick_orange_box_2,_place_white_box_2) = MIN (
   t_end[pick_orange_box_2] , t_end[place_white_box_2] )
 GC85: max_t_start(pick_orange_box_2,_place_white_box_2) = MAX (
   t_start[pick_orange_box_2] , t_start[place_white_box_2] )
 GC86: min_t_end(pick_orange_box_2,_place_white_box_3) = MIN (
   t_end[pick_orange_box_2] , t_end[place_white_box_3] )
 GC87: max_t_start(pick_orange_box_2,_place_white_box_3) = MAX (
   t_start[pick_orange_box_2] , t_start[place_white_box_3] )
 GC88: min_t_end(pick_orange_box_2,_place_white_box_4) = MIN (
   t_end[pick_orange_box_2] , t_end[place_white_box_4] )
 GC89: max_t_start(pick_orange_box_2,_place_white_box_4) = MAX (
   t_start[pick_orange_box_2] , t_start[place_white_box_4] )
 GC90: min_t_end(pick_orange_box_3,_pick_orange_box_4) = MIN (
   t_end[pick_orange_box_3] , t_end[pick_orange_box_4] )
 GC91: max_t_start(pick_orange_box_3,_pick_orange_box_4) = MAX (
   t_start[pick_orange_box_3] , t_start[pick_orange_box_4] )
 GC92: min_t_end(pick_orange_box_3,_pick_blue_box_1) = MIN (
   t_end[pick_orange_box_3] , t_end[pick_blue_box_1] )
 GC93: max_t_start(pick_orange_box_3,_pick_blue_box_1) = MAX (
   t_start[pick_orange_box_3] , t_start[pick_blue_box_1] )
 GC94: min_t_end(pick_orange_box_3,_pick_blue_box_2) = MIN (
   t_end[pick_orange_box_3] , t_end[pick_blue_box_2] )
 GC95: max_t_start(pick_orange_box_3,_pick_blue_box_2) = MAX (
   t_start[pick_orange_box_3] , t_start[pick_blue_box_2] )
 GC96: min_t_end(pick_orange_box_3,_pick_blue_box_3) = MIN (
   t_end[pick_orange_box_3] , t_end[pick_blue_box_3] )
 GC97: max_t_start(pick_orange_box_3,_pick_blue_box_3) = MAX (
   t_start[pick_orange_box_3] , t_start[pick_blue_box_3] )
 GC98: min_t_end(pick_orange_box_3,_pick_blue_box_4) = MIN (
   t_end[pick_orange_box_3] , t_end[pick_blue_box_4] )
 GC99: max_t_start(pick_orange_box_3,_pick_blue_box_4) = MAX (
   t_start[pick_orange_box_3] , t_start[pick_blue_box_4] )
 GC100: min_t_end(pick_orange_box_3,_pick_white_box_1) = MIN (
   t_end[pick_orange_box_3] , t_end[pick_white_box_1] )
 GC101: max_t_start(pick_orange_box_3,_pick_white_box_1) = MAX (
   t_start[pick_orange_box_3] , t_start[pick_white_box_1] )
 GC102: min_t_end(pick_orange_box_3,_pick_white_box_2) = MIN (
   t_end[pick_orange_box_3] , t_end[pick_white_box_2] )
 GC103: max_t_start(pick_orange_box_3,_pick_white_box_2) = MAX (
   t_start[pick_orange_box_3] , t_start[pick_white_box_2] )
 GC104: min_t_end(pick_orange_box_3,_pick_white_box_3) = MIN (
   t_end[pick_orange_box_3] , t_end[pick_white_box_3] )
 GC105: max_t_start(pick_orange_box_3,_pick_white_box_3) = MAX (
   t_start[pick_orange_box_3] , t_start[pick_white_box_3] )
 GC106: min_t_end(pick_orange_box_3,_pick_white_box_4) = MIN (
   t_end[pick_orange_box_3] , t_end[pick_white_box_4] )
 GC107: max_t_start(pick_orange_box_3,_pick_white_box_4) = MAX (
   t_start[pick_orange_box_3] , t_start[pick_white_box_4] )
 GC108: min_t_end(pick_orange_box_3,_place_orange_box_1) = MIN (
   t_end[pick_orange_box_3] , t_end[place_orange_box_1] )
 GC109: max_t_start(pick_orange_box_3,_place_orange_box_1) = MAX (
   t_start[pick_orange_box_3] , t_start[place_orange_box_1] )
 GC110: min_t_end(pick_orange_box_3,_place_orange_box_2) = MIN (
   t_end[pick_orange_box_3] , t_end[place_orange_box_2] )
 GC111: max_t_start(pick_orange_box_3,_place_orange_box_2) = MAX (
   t_start[pick_orange_box_3] , t_start[place_orange_box_2] )
 GC112: min_t_end(pick_orange_box_3,_place_orange_box_3) = MIN (
   t_end[pick_orange_box_3] , t_end[place_orange_box_3] )
 GC113: max_t_start(pick_orange_box_3,_place_orange_box_3) = MAX (
   t_start[pick_orange_box_3] , t_start[place_orange_box_3] )
 GC114: min_t_end(pick_orange_box_3,_place_orange_box_4) = MIN (
   t_end[pick_orange_box_3] , t_end[place_orange_box_4] )
 GC115: max_t_start(pick_orange_box_3,_place_orange_box_4) = MAX (
   t_start[pick_orange_box_3] , t_start[place_orange_box_4] )
 GC116: min_t_end(pick_orange_box_3,_place_blue_box_1) = MIN (
   t_end[pick_orange_box_3] , t_end[place_blue_box_1] )
 GC117: max_t_start(pick_orange_box_3,_place_blue_box_1) = MAX (
   t_start[pick_orange_box_3] , t_start[place_blue_box_1] )
 GC118: min_t_end(pick_orange_box_3,_place_blue_box_2) = MIN (
   t_end[pick_orange_box_3] , t_end[place_blue_box_2] )
 GC119: max_t_start(pick_orange_box_3,_place_blue_box_2) = MAX (
   t_start[pick_orange_box_3] , t_start[place_blue_box_2] )
 GC120: min_t_end(pick_orange_box_3,_place_blue_box_3) = MIN (
   t_end[pick_orange_box_3] , t_end[place_blue_box_3] )
 GC121: max_t_start(pick_orange_box_3,_place_blue_box_3) = MAX (
   t_start[pick_orange_box_3] , t_start[place_blue_box_3] )
 GC122: min_t_end(pick_orange_box_3,_place_blue_box_4) = MIN (
   t_end[pick_orange_box_3] , t_end[place_blue_box_4] )
 GC123: max_t_start(pick_orange_box_3,_place_blue_box_4) = MAX (
   t_start[pick_orange_box_3] , t_start[place_blue_box_4] )
 GC124: min_t_end(pick_orange_box_3,_place_white_box_1) = MIN (
   t_end[pick_orange_box_3] , t_end[place_white_box_1] )
 GC125: max_t_start(pick_orange_box_3,_place_white_box_1) = MAX (
   t_start[pick_orange_box_3] , t_start[place_white_box_1] )
 GC126: min_t_end(pick_orange_box_3,_place_white_box_2) = MIN (
   t_end[pick_orange_box_3] , t_end[place_white_box_2] )
 GC127: max_t_start(pick_orange_box_3,_place_white_box_2) = MAX (
   t_start[pick_orange_box_3] , t_start[place_white_box_2] )
 GC128: min_t_end(pick_orange_box_3,_place_white_box_3) = MIN (
   t_end[pick_orange_box_3] , t_end[place_white_box_3] )
 GC129: max_t_start(pick_orange_box_3,_place_white_box_3) = MAX (
   t_start[pick_orange_box_3] , t_start[place_white_box_3] )
 GC130: min_t_end(pick_orange_box_3,_place_white_box_4) = MIN (
   t_end[pick_orange_box_3] , t_end[place_white_box_4] )
 GC131: max_t_start(pick_orange_box_3,_place_white_box_4) = MAX (
   t_start[pick_orange_box_3] , t_start[place_white_box_4] )
 GC132: min_t_end(pick_orange_box_4,_pick_blue_box_1) = MIN (
   t_end[pick_orange_box_4] , t_end[pick_blue_box_1] )
 GC133: max_t_start(pick_orange_box_4,_pick_blue_box_1) = MAX (
   t_start[pick_orange_box_4] , t_start[pick_blue_box_1] )
 GC134: min_t_end(pick_orange_box_4,_pick_blue_box_2) = MIN (
   t_end[pick_orange_box_4] , t_end[pick_blue_box_2] )
 GC135: max_t_start(pick_orange_box_4,_pick_blue_box_2) = MAX (
   t_start[pick_orange_box_4] , t_start[pick_blue_box_2] )
 GC136: min_t_end(pick_orange_box_4,_pick_blue_box_3) = MIN (
   t_end[pick_orange_box_4] , t_end[pick_blue_box_3] )
 GC137: max_t_start(pick_orange_box_4,_pick_blue_box_3) = MAX (
   t_start[pick_orange_box_4] , t_start[pick_blue_box_3] )
 GC138: min_t_end(pick_orange_box_4,_pick_blue_box_4) = MIN (
   t_end[pick_orange_box_4] , t_end[pick_blue_box_4] )
 GC139: max_t_start(pick_orange_box_4,_pick_blue_box_4) = MAX (
   t_start[pick_orange_box_4] , t_start[pick_blue_box_4] )
 GC140: min_t_end(pick_orange_box_4,_pick_white_box_1) = MIN (
   t_end[pick_orange_box_4] , t_end[pick_white_box_1] )
 GC141: max_t_start(pick_orange_box_4,_pick_white_box_1) = MAX (
   t_start[pick_orange_box_4] , t_start[pick_white_box_1] )
 GC142: min_t_end(pick_orange_box_4,_pick_white_box_2) = MIN (
   t_end[pick_orange_box_4] , t_end[pick_white_box_2] )
 GC143: max_t_start(pick_orange_box_4,_pick_white_box_2) = MAX (
   t_start[pick_orange_box_4] , t_start[pick_white_box_2] )
 GC144: min_t_end(pick_orange_box_4,_pick_white_box_3) = MIN (
   t_end[pick_orange_box_4] , t_end[pick_white_box_3] )
 GC145: max_t_start(pick_orange_box_4,_pick_white_box_3) = MAX (
   t_start[pick_orange_box_4] , t_start[pick_white_box_3] )
 GC146: min_t_end(pick_orange_box_4,_pick_white_box_4) = MIN (
   t_end[pick_orange_box_4] , t_end[pick_white_box_4] )
 GC147: max_t_start(pick_orange_box_4,_pick_white_box_4) = MAX (
   t_start[pick_orange_box_4] , t_start[pick_white_box_4] )
 GC148: min_t_end(pick_orange_box_4,_place_orange_box_1) = MIN (
   t_end[pick_orange_box_4] , t_end[place_orange_box_1] )
 GC149: max_t_start(pick_orange_box_4,_place_orange_box_1) = MAX (
   t_start[pick_orange_box_4] , t_start[place_orange_box_1] )
 GC150: min_t_end(pick_orange_box_4,_place_orange_box_2) = MIN (
   t_end[pick_orange_box_4] , t_end[place_orange_box_2] )
 GC151: max_t_start(pick_orange_box_4,_place_orange_box_2) = MAX (
   t_start[pick_orange_box_4] , t_start[place_orange_box_2] )
 GC152: min_t_end(pick_orange_box_4,_place_orange_box_3) = MIN (
   t_end[pick_orange_box_4] , t_end[place_orange_box_3] )
 GC153: max_t_start(pick_orange_box_4,_place_orange_box_3) = MAX (
   t_start[pick_orange_box_4] , t_start[place_orange_box_3] )
 GC154: min_t_end(pick_orange_box_4,_place_orange_box_4) = MIN (
   t_end[pick_orange_box_4] , t_end[place_orange_box_4] )
 GC155: max_t_start(pick_orange_box_4,_place_orange_box_4) = MAX (
   t_start[pick_orange_box_4] , t_start[place_orange_box_4] )
 GC156: min_t_end(pick_orange_box_4,_place_blue_box_1) = MIN (
   t_end[pick_orange_box_4] , t_end[place_blue_box_1] )
 GC157: max_t_start(pick_orange_box_4,_place_blue_box_1) = MAX (
   t_start[pick_orange_box_4] , t_start[place_blue_box_1] )
 GC158: min_t_end(pick_orange_box_4,_place_blue_box_2) = MIN (
   t_end[pick_orange_box_4] , t_end[place_blue_box_2] )
 GC159: max_t_start(pick_orange_box_4,_place_blue_box_2) = MAX (
   t_start[pick_orange_box_4] , t_start[place_blue_box_2] )
 GC160: min_t_end(pick_orange_box_4,_place_blue_box_3) = MIN (
   t_end[pick_orange_box_4] , t_end[place_blue_box_3] )
 GC161: max_t_start(pick_orange_box_4,_place_blue_box_3) = MAX (
   t_start[pick_orange_box_4] , t_start[place_blue_box_3] )
 GC162: min_t_end(pick_orange_box_4,_place_blue_box_4) = MIN (
   t_end[pick_orange_box_4] , t_end[place_blue_box_4] )
 GC163: max_t_start(pick_orange_box_4,_place_blue_box_4) = MAX (
   t_start[pick_orange_box_4] , t_start[place_blue_box_4] )
 GC164: min_t_end(pick_orange_box_4,_place_white_box_1) = MIN (
   t_end[pick_orange_box_4] , t_end[place_white_box_1] )
 GC165: max_t_start(pick_orange_box_4,_place_white_box_1) = MAX (
   t_start[pick_orange_box_4] , t_start[place_white_box_1] )
 GC166: min_t_end(pick_orange_box_4,_place_white_box_2) = MIN (
   t_end[pick_orange_box_4] , t_end[place_white_box_2] )
 GC167: max_t_start(pick_orange_box_4,_place_white_box_2) = MAX (
   t_start[pick_orange_box_4] , t_start[place_white_box_2] )
 GC168: min_t_end(pick_orange_box_4,_place_white_box_3) = MIN (
   t_end[pick_orange_box_4] , t_end[place_white_box_3] )
 GC169: max_t_start(pick_orange_box_4,_place_white_box_3) = MAX (
   t_start[pick_orange_box_4] , t_start[place_white_box_3] )
 GC170: min_t_end(pick_orange_box_4,_place_white_box_4) = MIN (
   t_end[pick_orange_box_4] , t_end[place_white_box_4] )
 GC171: max_t_start(pick_orange_box_4,_place_white_box_4) = MAX (
   t_start[pick_orange_box_4] , t_start[place_white_box_4] )
 GC172: min_t_end(pick_blue_box_1,_pick_blue_box_2) = MIN (
   t_end[pick_blue_box_1] , t_end[pick_blue_box_2] )
 GC173: max_t_start(pick_blue_box_1,_pick_blue_box_2) = MAX (
   t_start[pick_blue_box_1] , t_start[pick_blue_box_2] )
 GC174: min_t_end(pick_blue_box_1,_pick_blue_box_3) = MIN (
   t_end[pick_blue_box_1] , t_end[pick_blue_box_3] )
 GC175: max_t_start(pick_blue_box_1,_pick_blue_box_3) = MAX (
   t_start[pick_blue_box_1] , t_start[pick_blue_box_3] )
 GC176: min_t_end(pick_blue_box_1,_pick_blue_box_4) = MIN (
   t_end[pick_blue_box_1] , t_end[pick_blue_box_4] )
 GC177: max_t_start(pick_blue_box_1,_pick_blue_box_4) = MAX (
   t_start[pick_blue_box_1] , t_start[pick_blue_box_4] )
 GC178: min_t_end(pick_blue_box_1,_pick_white_box_1) = MIN (
   t_end[pick_blue_box_1] , t_end[pick_white_box_1] )
 GC179: max_t_start(pick_blue_box_1,_pick_white_box_1) = MAX (
   t_start[pick_blue_box_1] , t_start[pick_white_box_1] )
 GC180: min_t_end(pick_blue_box_1,_pick_white_box_2) = MIN (
   t_end[pick_blue_box_1] , t_end[pick_white_box_2] )
 GC181: max_t_start(pick_blue_box_1,_pick_white_box_2) = MAX (
   t_start[pick_blue_box_1] , t_start[pick_white_box_2] )
 GC182: min_t_end(pick_blue_box_1,_pick_white_box_3) = MIN (
   t_end[pick_blue_box_1] , t_end[pick_white_box_3] )
 GC183: max_t_start(pick_blue_box_1,_pick_white_box_3) = MAX (
   t_start[pick_blue_box_1] , t_start[pick_white_box_3] )
 GC184: min_t_end(pick_blue_box_1,_pick_white_box_4) = MIN (
   t_end[pick_blue_box_1] , t_end[pick_white_box_4] )
 GC185: max_t_start(pick_blue_box_1,_pick_white_box_4) = MAX (
   t_start[pick_blue_box_1] , t_start[pick_white_box_4] )
 GC186: min_t_end(pick_blue_box_1,_place_orange_box_1) = MIN (
   t_end[pick_blue_box_1] , t_end[place_orange_box_1] )
 GC187: max_t_start(pick_blue_box_1,_place_orange_box_1) = MAX (
   t_start[pick_blue_box_1] , t_start[place_orange_box_1] )
 GC188: min_t_end(pick_blue_box_1,_place_orange_box_2) = MIN (
   t_end[pick_blue_box_1] , t_end[place_orange_box_2] )
 GC189: max_t_start(pick_blue_box_1,_place_orange_box_2) = MAX (
   t_start[pick_blue_box_1] , t_start[place_orange_box_2] )
 GC190: min_t_end(pick_blue_box_1,_place_orange_box_3) = MIN (
   t_end[pick_blue_box_1] , t_end[place_orange_box_3] )
 GC191: max_t_start(pick_blue_box_1,_place_orange_box_3) = MAX (
   t_start[pick_blue_box_1] , t_start[place_orange_box_3] )
 GC192: min_t_end(pick_blue_box_1,_place_orange_box_4) = MIN (
   t_end[pick_blue_box_1] , t_end[place_orange_box_4] )
 GC193: max_t_start(pick_blue_box_1,_place_orange_box_4) = MAX (
   t_start[pick_blue_box_1] , t_start[place_orange_box_4] )
 GC194: min_t_end(pick_blue_box_1,_place_blue_box_1) = MIN (
   t_end[pick_blue_box_1] , t_end[place_blue_box_1] )
 GC195: max_t_start(pick_blue_box_1,_place_blue_box_1) = MAX (
   t_start[pick_blue_box_1] , t_start[place_blue_box_1] )
 GC196: min_t_end(pick_blue_box_1,_place_blue_box_2) = MIN (
   t_end[pick_blue_box_1] , t_end[place_blue_box_2] )
 GC197: max_t_start(pick_blue_box_1,_place_blue_box_2) = MAX (
   t_start[pick_blue_box_1] , t_start[place_blue_box_2] )
 GC198: min_t_end(pick_blue_box_1,_place_blue_box_3) = MIN (
   t_end[pick_blue_box_1] , t_end[place_blue_box_3] )
 GC199: max_t_start(pick_blue_box_1,_place_blue_box_3) = MAX (
   t_start[pick_blue_box_1] , t_start[place_blue_box_3] )
 GC200: min_t_end(pick_blue_box_1,_place_blue_box_4) = MIN (
   t_end[pick_blue_box_1] , t_end[place_blue_box_4] )
 GC201: max_t_start(pick_blue_box_1,_place_blue_box_4) = MAX (
   t_start[pick_blue_box_1] , t_start[place_blue_box_4] )
 GC202: min_t_end(pick_blue_box_1,_place_white_box_1) = MIN (
   t_end[pick_blue_box_1] , t_end[place_white_box_1] )
 GC203: max_t_start(pick_blue_box_1,_place_white_box_1) = MAX (
   t_start[pick_blue_box_1] , t_start[place_white_box_1] )
 GC204: min_t_end(pick_blue_box_1,_place_white_box_2) = MIN (
   t_end[pick_blue_box_1] , t_end[place_white_box_2] )
 GC205: max_t_start(pick_blue_box_1,_place_white_box_2) = MAX (
   t_start[pick_blue_box_1] , t_start[place_white_box_2] )
 GC206: min_t_end(pick_blue_box_1,_place_white_box_3) = MIN (
   t_end[pick_blue_box_1] , t_end[place_white_box_3] )
 GC207: max_t_start(pick_blue_box_1,_place_white_box_3) = MAX (
   t_start[pick_blue_box_1] , t_start[place_white_box_3] )
 GC208: min_t_end(pick_blue_box_1,_place_white_box_4) = MIN (
   t_end[pick_blue_box_1] , t_end[place_white_box_4] )
 GC209: max_t_start(pick_blue_box_1,_place_white_box_4) = MAX (
   t_start[pick_blue_box_1] , t_start[place_white_box_4] )
 GC210: min_t_end(pick_blue_box_2,_pick_blue_box_3) = MIN (
   t_end[pick_blue_box_2] , t_end[pick_blue_box_3] )
 GC211: max_t_start(pick_blue_box_2,_pick_blue_box_3) = MAX (
   t_start[pick_blue_box_2] , t_start[pick_blue_box_3] )
 GC212: min_t_end(pick_blue_box_2,_pick_blue_box_4) = MIN (
   t_end[pick_blue_box_2] , t_end[pick_blue_box_4] )
 GC213: max_t_start(pick_blue_box_2,_pick_blue_box_4) = MAX (
   t_start[pick_blue_box_2] , t_start[pick_blue_box_4] )
 GC214: min_t_end(pick_blue_box_2,_pick_white_box_1) = MIN (
   t_end[pick_blue_box_2] , t_end[pick_white_box_1] )
 GC215: max_t_start(pick_blue_box_2,_pick_white_box_1) = MAX (
   t_start[pick_blue_box_2] , t_start[pick_white_box_1] )
 GC216: min_t_end(pick_blue_box_2,_pick_white_box_2) = MIN (
   t_end[pick_blue_box_2] , t_end[pick_white_box_2] )
 GC217: max_t_start(pick_blue_box_2,_pick_white_box_2) = MAX (
   t_start[pick_blue_box_2] , t_start[pick_white_box_2] )
 GC218: min_t_end(pick_blue_box_2,_pick_white_box_3) = MIN (
   t_end[pick_blue_box_2] , t_end[pick_white_box_3] )
 GC219: max_t_start(pick_blue_box_2,_pick_white_box_3) = MAX (
   t_start[pick_blue_box_2] , t_start[pick_white_box_3] )
 GC220: min_t_end(pick_blue_box_2,_pick_white_box_4) = MIN (
   t_end[pick_blue_box_2] , t_end[pick_white_box_4] )
 GC221: max_t_start(pick_blue_box_2,_pick_white_box_4) = MAX (
   t_start[pick_blue_box_2] , t_start[pick_white_box_4] )
 GC222: min_t_end(pick_blue_box_2,_place_orange_box_1) = MIN (
   t_end[pick_blue_box_2] , t_end[place_orange_box_1] )
 GC223: max_t_start(pick_blue_box_2,_place_orange_box_1) = MAX (
   t_start[pick_blue_box_2] , t_start[place_orange_box_1] )
 GC224: min_t_end(pick_blue_box_2,_place_orange_box_2) = MIN (
   t_end[pick_blue_box_2] , t_end[place_orange_box_2] )
 GC225: max_t_start(pick_blue_box_2,_place_orange_box_2) = MAX (
   t_start[pick_blue_box_2] , t_start[place_orange_box_2] )
 GC226: min_t_end(pick_blue_box_2,_place_orange_box_3) = MIN (
   t_end[pick_blue_box_2] , t_end[place_orange_box_3] )
 GC227: max_t_start(pick_blue_box_2,_place_orange_box_3) = MAX (
   t_start[pick_blue_box_2] , t_start[place_orange_box_3] )
 GC228: min_t_end(pick_blue_box_2,_place_orange_box_4) = MIN (
   t_end[pick_blue_box_2] , t_end[place_orange_box_4] )
 GC229: max_t_start(pick_blue_box_2,_place_orange_box_4) = MAX (
   t_start[pick_blue_box_2] , t_start[place_orange_box_4] )
 GC230: min_t_end(pick_blue_box_2,_place_blue_box_1) = MIN (
   t_end[pick_blue_box_2] , t_end[place_blue_box_1] )
 GC231: max_t_start(pick_blue_box_2,_place_blue_box_1) = MAX (
   t_start[pick_blue_box_2] , t_start[place_blue_box_1] )
 GC232: min_t_end(pick_blue_box_2,_place_blue_box_2) = MIN (
   t_end[pick_blue_box_2] , t_end[place_blue_box_2] )
 GC233: max_t_start(pick_blue_box_2,_place_blue_box_2) = MAX (
   t_start[pick_blue_box_2] , t_start[place_blue_box_2] )
 GC234: min_t_end(pick_blue_box_2,_place_blue_box_3) = MIN (
   t_end[pick_blue_box_2] , t_end[place_blue_box_3] )
 GC235: max_t_start(pick_blue_box_2,_place_blue_box_3) = MAX (
   t_start[pick_blue_box_2] , t_start[place_blue_box_3] )
 GC236: min_t_end(pick_blue_box_2,_place_blue_box_4) = MIN (
   t_end[pick_blue_box_2] , t_end[place_blue_box_4] )
 GC237: max_t_start(pick_blue_box_2,_place_blue_box_4) = MAX (
   t_start[pick_blue_box_2] , t_start[place_blue_box_4] )
 GC238: min_t_end(pick_blue_box_2,_place_white_box_1) = MIN (
   t_end[pick_blue_box_2] , t_end[place_white_box_1] )
 GC239: max_t_start(pick_blue_box_2,_place_white_box_1) = MAX (
   t_start[pick_blue_box_2] , t_start[place_white_box_1] )
 GC240: min_t_end(pick_blue_box_2,_place_white_box_2) = MIN (
   t_end[pick_blue_box_2] , t_end[place_white_box_2] )
 GC241: max_t_start(pick_blue_box_2,_place_white_box_2) = MAX (
   t_start[pick_blue_box_2] , t_start[place_white_box_2] )
 GC242: min_t_end(pick_blue_box_2,_place_white_box_3) = MIN (
   t_end[pick_blue_box_2] , t_end[place_white_box_3] )
 GC243: max_t_start(pick_blue_box_2,_place_white_box_3) = MAX (
   t_start[pick_blue_box_2] , t_start[place_white_box_3] )
 GC244: min_t_end(pick_blue_box_2,_place_white_box_4) = MIN (
   t_end[pick_blue_box_2] , t_end[place_white_box_4] )
 GC245: max_t_start(pick_blue_box_2,_place_white_box_4) = MAX (
   t_start[pick_blue_box_2] , t_start[place_white_box_4] )
 GC246: min_t_end(pick_blue_box_3,_pick_blue_box_4) = MIN (
   t_end[pick_blue_box_3] , t_end[pick_blue_box_4] )
 GC247: max_t_start(pick_blue_box_3,_pick_blue_box_4) = MAX (
   t_start[pick_blue_box_3] , t_start[pick_blue_box_4] )
 GC248: min_t_end(pick_blue_box_3,_pick_white_box_1) = MIN (
   t_end[pick_blue_box_3] , t_end[pick_white_box_1] )
 GC249: max_t_start(pick_blue_box_3,_pick_white_box_1) = MAX (
   t_start[pick_blue_box_3] , t_start[pick_white_box_1] )
 GC250: min_t_end(pick_blue_box_3,_pick_white_box_2) = MIN (
   t_end[pick_blue_box_3] , t_end[pick_white_box_2] )
 GC251: max_t_start(pick_blue_box_3,_pick_white_box_2) = MAX (
   t_start[pick_blue_box_3] , t_start[pick_white_box_2] )
 GC252: min_t_end(pick_blue_box_3,_pick_white_box_3) = MIN (
   t_end[pick_blue_box_3] , t_end[pick_white_box_3] )
 GC253: max_t_start(pick_blue_box_3,_pick_white_box_3) = MAX (
   t_start[pick_blue_box_3] , t_start[pick_white_box_3] )
 GC254: min_t_end(pick_blue_box_3,_pick_white_box_4) = MIN (
   t_end[pick_blue_box_3] , t_end[pick_white_box_4] )
 GC255: max_t_start(pick_blue_box_3,_pick_white_box_4) = MAX (
   t_start[pick_blue_box_3] , t_start[pick_white_box_4] )
 GC256: min_t_end(pick_blue_box_3,_place_orange_box_1) = MIN (
   t_end[pick_blue_box_3] , t_end[place_orange_box_1] )
 GC257: max_t_start(pick_blue_box_3,_place_orange_box_1) = MAX (
   t_start[pick_blue_box_3] , t_start[place_orange_box_1] )
 GC258: min_t_end(pick_blue_box_3,_place_orange_box_2) = MIN (
   t_end[pick_blue_box_3] , t_end[place_orange_box_2] )
 GC259: max_t_start(pick_blue_box_3,_place_orange_box_2) = MAX (
   t_start[pick_blue_box_3] , t_start[place_orange_box_2] )
 GC260: min_t_end(pick_blue_box_3,_place_orange_box_3) = MIN (
   t_end[pick_blue_box_3] , t_end[place_orange_box_3] )
 GC261: max_t_start(pick_blue_box_3,_place_orange_box_3) = MAX (
   t_start[pick_blue_box_3] , t_start[place_orange_box_3] )
 GC262: min_t_end(pick_blue_box_3,_place_orange_box_4) = MIN (
   t_end[pick_blue_box_3] , t_end[place_orange_box_4] )
 GC263: max_t_start(pick_blue_box_3,_place_orange_box_4) = MAX (
   t_start[pick_blue_box_3] , t_start[place_orange_box_4] )
 GC264: min_t_end(pick_blue_box_3,_place_blue_box_1) = MIN (
   t_end[pick_blue_box_3] , t_end[place_blue_box_1] )
 GC265: max_t_start(pick_blue_box_3,_place_blue_box_1) = MAX (
   t_start[pick_blue_box_3] , t_start[place_blue_box_1] )
 GC266: min_t_end(pick_blue_box_3,_place_blue_box_2) = MIN (
   t_end[pick_blue_box_3] , t_end[place_blue_box_2] )
 GC267: max_t_start(pick_blue_box_3,_place_blue_box_2) = MAX (
   t_start[pick_blue_box_3] , t_start[place_blue_box_2] )
 GC268: min_t_end(pick_blue_box_3,_place_blue_box_3) = MIN (
   t_end[pick_blue_box_3] , t_end[place_blue_box_3] )
 GC269: max_t_start(pick_blue_box_3,_place_blue_box_3) = MAX (
   t_start[pick_blue_box_3] , t_start[place_blue_box_3] )
 GC270: min_t_end(pick_blue_box_3,_place_blue_box_4) = MIN (
   t_end[pick_blue_box_3] , t_end[place_blue_box_4] )
 GC271: max_t_start(pick_blue_box_3,_place_blue_box_4) = MAX (
   t_start[pick_blue_box_3] , t_start[place_blue_box_4] )
 GC272: min_t_end(pick_blue_box_3,_place_white_box_1) = MIN (
   t_end[pick_blue_box_3] , t_end[place_white_box_1] )
 GC273: max_t_start(pick_blue_box_3,_place_white_box_1) = MAX (
   t_start[pick_blue_box_3] , t_start[place_white_box_1] )
 GC274: min_t_end(pick_blue_box_3,_place_white_box_2) = MIN (
   t_end[pick_blue_box_3] , t_end[place_white_box_2] )
 GC275: max_t_start(pick_blue_box_3,_place_white_box_2) = MAX (
   t_start[pick_blue_box_3] , t_start[place_white_box_2] )
 GC276: min_t_end(pick_blue_box_3,_place_white_box_3) = MIN (
   t_end[pick_blue_box_3] , t_end[place_white_box_3] )
 GC277: max_t_start(pick_blue_box_3,_place_white_box_3) = MAX (
   t_start[pick_blue_box_3] , t_start[place_white_box_3] )
 GC278: min_t_end(pick_blue_box_3,_place_white_box_4) = MIN (
   t_end[pick_blue_box_3] , t_end[place_white_box_4] )
 GC279: max_t_start(pick_blue_box_3,_place_white_box_4) = MAX (
   t_start[pick_blue_box_3] , t_start[place_white_box_4] )
 GC280: min_t_end(pick_blue_box_4,_pick_white_box_1) = MIN (
   t_end[pick_blue_box_4] , t_end[pick_white_box_1] )
 GC281: max_t_start(pick_blue_box_4,_pick_white_box_1) = MAX (
   t_start[pick_blue_box_4] , t_start[pick_white_box_1] )
 GC282: min_t_end(pick_blue_box_4,_pick_white_box_2) = MIN (
   t_end[pick_blue_box_4] , t_end[pick_white_box_2] )
 GC283: max_t_start(pick_blue_box_4,_pick_white_box_2) = MAX (
   t_start[pick_blue_box_4] , t_start[pick_white_box_2] )
 GC284: min_t_end(pick_blue_box_4,_pick_white_box_3) = MIN (
   t_end[pick_blue_box_4] , t_end[pick_white_box_3] )
 GC285: max_t_start(pick_blue_box_4,_pick_white_box_3) = MAX (
   t_start[pick_blue_box_4] , t_start[pick_white_box_3] )
 GC286: min_t_end(pick_blue_box_4,_pick_white_box_4) = MIN (
   t_end[pick_blue_box_4] , t_end[pick_white_box_4] )
 GC287: max_t_start(pick_blue_box_4,_pick_white_box_4) = MAX (
   t_start[pick_blue_box_4] , t_start[pick_white_box_4] )
 GC288: min_t_end(pick_blue_box_4,_place_orange_box_1) = MIN (
   t_end[pick_blue_box_4] , t_end[place_orange_box_1] )
 GC289: max_t_start(pick_blue_box_4,_place_orange_box_1) = MAX (
   t_start[pick_blue_box_4] , t_start[place_orange_box_1] )
 GC290: min_t_end(pick_blue_box_4,_place_orange_box_2) = MIN (
   t_end[pick_blue_box_4] , t_end[place_orange_box_2] )
 GC291: max_t_start(pick_blue_box_4,_place_orange_box_2) = MAX (
   t_start[pick_blue_box_4] , t_start[place_orange_box_2] )
 GC292: min_t_end(pick_blue_box_4,_place_orange_box_3) = MIN (
   t_end[pick_blue_box_4] , t_end[place_orange_box_3] )
 GC293: max_t_start(pick_blue_box_4,_place_orange_box_3) = MAX (
   t_start[pick_blue_box_4] , t_start[place_orange_box_3] )
 GC294: min_t_end(pick_blue_box_4,_place_orange_box_4) = MIN (
   t_end[pick_blue_box_4] , t_end[place_orange_box_4] )
 GC295: max_t_start(pick_blue_box_4,_place_orange_box_4) = MAX (
   t_start[pick_blue_box_4] , t_start[place_orange_box_4] )
 GC296: min_t_end(pick_blue_box_4,_place_blue_box_1) = MIN (
   t_end[pick_blue_box_4] , t_end[place_blue_box_1] )
 GC297: max_t_start(pick_blue_box_4,_place_blue_box_1) = MAX (
   t_start[pick_blue_box_4] , t_start[place_blue_box_1] )
 GC298: min_t_end(pick_blue_box_4,_place_blue_box_2) = MIN (
   t_end[pick_blue_box_4] , t_end[place_blue_box_2] )
 GC299: max_t_start(pick_blue_box_4,_place_blue_box_2) = MAX (
   t_start[pick_blue_box_4] , t_start[place_blue_box_2] )
 GC300: min_t_end(pick_blue_box_4,_place_blue_box_3) = MIN (
   t_end[pick_blue_box_4] , t_end[place_blue_box_3] )
 GC301: max_t_start(pick_blue_box_4,_place_blue_box_3) = MAX (
   t_start[pick_blue_box_4] , t_start[place_blue_box_3] )
 GC302: min_t_end(pick_blue_box_4,_place_blue_box_4) = MIN (
   t_end[pick_blue_box_4] , t_end[place_blue_box_4] )
 GC303: max_t_start(pick_blue_box_4,_place_blue_box_4) = MAX (
   t_start[pick_blue_box_4] , t_start[place_blue_box_4] )
 GC304: min_t_end(pick_blue_box_4,_place_white_box_1) = MIN (
   t_end[pick_blue_box_4] , t_end[place_white_box_1] )
 GC305: max_t_start(pick_blue_box_4,_place_white_box_1) = MAX (
   t_start[pick_blue_box_4] , t_start[place_white_box_1] )
 GC306: min_t_end(pick_blue_box_4,_place_white_box_2) = MIN (
   t_end[pick_blue_box_4] , t_end[place_white_box_2] )
 GC307: max_t_start(pick_blue_box_4,_place_white_box_2) = MAX (
   t_start[pick_blue_box_4] , t_start[place_white_box_2] )
 GC308: min_t_end(pick_blue_box_4,_place_white_box_3) = MIN (
   t_end[pick_blue_box_4] , t_end[place_white_box_3] )
 GC309: max_t_start(pick_blue_box_4,_place_white_box_3) = MAX (
   t_start[pick_blue_box_4] , t_start[place_white_box_3] )
 GC310: min_t_end(pick_blue_box_4,_place_white_box_4) = MIN (
   t_end[pick_blue_box_4] , t_end[place_white_box_4] )
 GC311: max_t_start(pick_blue_box_4,_place_white_box_4) = MAX (
   t_start[pick_blue_box_4] , t_start[place_white_box_4] )
 GC312: min_t_end(pick_white_box_1,_pick_white_box_2) = MIN (
   t_end[pick_white_box_1] , t_end[pick_white_box_2] )
 GC313: max_t_start(pick_white_box_1,_pick_white_box_2) = MAX (
   t_start[pick_white_box_1] , t_start[pick_white_box_2] )
 GC314: min_t_end(pick_white_box_1,_pick_white_box_3) = MIN (
   t_end[pick_white_box_1] , t_end[pick_white_box_3] )
 GC315: max_t_start(pick_white_box_1,_pick_white_box_3) = MAX (
   t_start[pick_white_box_1] , t_start[pick_white_box_3] )
 GC316: min_t_end(pick_white_box_1,_pick_white_box_4) = MIN (
   t_end[pick_white_box_1] , t_end[pick_white_box_4] )
 GC317: max_t_start(pick_white_box_1,_pick_white_box_4) = MAX (
   t_start[pick_white_box_1] , t_start[pick_white_box_4] )
 GC318: min_t_end(pick_white_box_1,_place_orange_box_1) = MIN (
   t_end[pick_white_box_1] , t_end[place_orange_box_1] )
 GC319: max_t_start(pick_white_box_1,_place_orange_box_1) = MAX (
   t_start[pick_white_box_1] , t_start[place_orange_box_1] )
 GC320: min_t_end(pick_white_box_1,_place_orange_box_2) = MIN (
   t_end[pick_white_box_1] , t_end[place_orange_box_2] )
 GC321: max_t_start(pick_white_box_1,_place_orange_box_2) = MAX (
   t_start[pick_white_box_1] , t_start[place_orange_box_2] )
 GC322: min_t_end(pick_white_box_1,_place_orange_box_3) = MIN (
   t_end[pick_white_box_1] , t_end[place_orange_box_3] )
 GC323: max_t_start(pick_white_box_1,_place_orange_box_3) = MAX (
   t_start[pick_white_box_1] , t_start[place_orange_box_3] )
 GC324: min_t_end(pick_white_box_1,_place_orange_box_4) = MIN (
   t_end[pick_white_box_1] , t_end[place_orange_box_4] )
 GC325: max_t_start(pick_white_box_1,_place_orange_box_4) = MAX (
   t_start[pick_white_box_1] , t_start[place_orange_box_4] )
 GC326: min_t_end(pick_white_box_1,_place_blue_box_1) = MIN (
   t_end[pick_white_box_1] , t_end[place_blue_box_1] )
 GC327: max_t_start(pick_white_box_1,_place_blue_box_1) = MAX (
   t_start[pick_white_box_1] , t_start[place_blue_box_1] )
 GC328: min_t_end(pick_white_box_1,_place_blue_box_2) = MIN (
   t_end[pick_white_box_1] , t_end[place_blue_box_2] )
 GC329: max_t_start(pick_white_box_1,_place_blue_box_2) = MAX (
   t_start[pick_white_box_1] , t_start[place_blue_box_2] )
 GC330: min_t_end(pick_white_box_1,_place_blue_box_3) = MIN (
   t_end[pick_white_box_1] , t_end[place_blue_box_3] )
 GC331: max_t_start(pick_white_box_1,_place_blue_box_3) = MAX (
   t_start[pick_white_box_1] , t_start[place_blue_box_3] )
 GC332: min_t_end(pick_white_box_1,_place_blue_box_4) = MIN (
   t_end[pick_white_box_1] , t_end[place_blue_box_4] )
 GC333: max_t_start(pick_white_box_1,_place_blue_box_4) = MAX (
   t_start[pick_white_box_1] , t_start[place_blue_box_4] )
 GC334: min_t_end(pick_white_box_1,_place_white_box_1) = MIN (
   t_end[pick_white_box_1] , t_end[place_white_box_1] )
 GC335: max_t_start(pick_white_box_1,_place_white_box_1) = MAX (
   t_start[pick_white_box_1] , t_start[place_white_box_1] )
 GC336: min_t_end(pick_white_box_1,_place_white_box_2) = MIN (
   t_end[pick_white_box_1] , t_end[place_white_box_2] )
 GC337: max_t_start(pick_white_box_1,_place_white_box_2) = MAX (
   t_start[pick_white_box_1] , t_start[place_white_box_2] )
 GC338: min_t_end(pick_white_box_1,_place_white_box_3) = MIN (
   t_end[pick_white_box_1] , t_end[place_white_box_3] )
 GC339: max_t_start(pick_white_box_1,_place_white_box_3) = MAX (
   t_start[pick_white_box_1] , t_start[place_white_box_3] )
 GC340: min_t_end(pick_white_box_1,_place_white_box_4) = MIN (
   t_end[pick_white_box_1] , t_end[place_white_box_4] )
 GC341: max_t_start(pick_white_box_1,_place_white_box_4) = MAX (
   t_start[pick_white_box_1] , t_start[place_white_box_4] )
 GC342: min_t_end(pick_white_box_2,_pick_white_box_3) = MIN (
   t_end[pick_white_box_2] , t_end[pick_white_box_3] )
 GC343: max_t_start(pick_white_box_2,_pick_white_box_3) = MAX (
   t_start[pick_white_box_2] , t_start[pick_white_box_3] )
 GC344: min_t_end(pick_white_box_2,_pick_white_box_4) = MIN (
   t_end[pick_white_box_2] , t_end[pick_white_box_4] )
 GC345: max_t_start(pick_white_box_2,_pick_white_box_4) = MAX (
   t_start[pick_white_box_2] , t_start[pick_white_box_4] )
 GC346: min_t_end(pick_white_box_2,_place_orange_box_1) = MIN (
   t_end[pick_white_box_2] , t_end[place_orange_box_1] )
 GC347: max_t_start(pick_white_box_2,_place_orange_box_1) = MAX (
   t_start[pick_white_box_2] , t_start[place_orange_box_1] )
 GC348: min_t_end(pick_white_box_2,_place_orange_box_2) = MIN (
   t_end[pick_white_box_2] , t_end[place_orange_box_2] )
 GC349: max_t_start(pick_white_box_2,_place_orange_box_2) = MAX (
   t_start[pick_white_box_2] , t_start[place_orange_box_2] )
 GC350: min_t_end(pick_white_box_2,_place_orange_box_3) = MIN (
   t_end[pick_white_box_2] , t_end[place_orange_box_3] )
 GC351: max_t_start(pick_white_box_2,_place_orange_box_3) = MAX (
   t_start[pick_white_box_2] , t_start[place_orange_box_3] )
 GC352: min_t_end(pick_white_box_2,_place_orange_box_4) = MIN (
   t_end[pick_white_box_2] , t_end[place_orange_box_4] )
 GC353: max_t_start(pick_white_box_2,_place_orange_box_4) = MAX (
   t_start[pick_white_box_2] , t_start[place_orange_box_4] )
 GC354: min_t_end(pick_white_box_2,_place_blue_box_1) = MIN (
   t_end[pick_white_box_2] , t_end[place_blue_box_1] )
 GC355: max_t_start(pick_white_box_2,_place_blue_box_1) = MAX (
   t_start[pick_white_box_2] , t_start[place_blue_box_1] )
 GC356: min_t_end(pick_white_box_2,_place_blue_box_2) = MIN (
   t_end[pick_white_box_2] , t_end[place_blue_box_2] )
 GC357: max_t_start(pick_white_box_2,_place_blue_box_2) = MAX (
   t_start[pick_white_box_2] , t_start[place_blue_box_2] )
 GC358: min_t_end(pick_white_box_2,_place_blue_box_3) = MIN (
   t_end[pick_white_box_2] , t_end[place_blue_box_3] )
 GC359: max_t_start(pick_white_box_2,_place_blue_box_3) = MAX (
   t_start[pick_white_box_2] , t_start[place_blue_box_3] )
 GC360: min_t_end(pick_white_box_2,_place_blue_box_4) = MIN (
   t_end[pick_white_box_2] , t_end[place_blue_box_4] )
 GC361: max_t_start(pick_white_box_2,_place_blue_box_4) = MAX (
   t_start[pick_white_box_2] , t_start[place_blue_box_4] )
 GC362: min_t_end(pick_white_box_2,_place_white_box_1) = MIN (
   t_end[pick_white_box_2] , t_end[place_white_box_1] )
 GC363: max_t_start(pick_white_box_2,_place_white_box_1) = MAX (
   t_start[pick_white_box_2] , t_start[place_white_box_1] )
 GC364: min_t_end(pick_white_box_2,_place_white_box_2) = MIN (
   t_end[pick_white_box_2] , t_end[place_white_box_2] )
 GC365: max_t_start(pick_white_box_2,_place_white_box_2) = MAX (
   t_start[pick_white_box_2] , t_start[place_white_box_2] )
 GC366: min_t_end(pick_white_box_2,_place_white_box_3) = MIN (
   t_end[pick_white_box_2] , t_end[place_white_box_3] )
 GC367: max_t_start(pick_white_box_2,_place_white_box_3) = MAX (
   t_start[pick_white_box_2] , t_start[place_white_box_3] )
 GC368: min_t_end(pick_white_box_2,_place_white_box_4) = MIN (
   t_end[pick_white_box_2] , t_end[place_white_box_4] )
 GC369: max_t_start(pick_white_box_2,_place_white_box_4) = MAX (
   t_start[pick_white_box_2] , t_start[place_white_box_4] )
 GC370: min_t_end(pick_white_box_3,_pick_white_box_4) = MIN (
   t_end[pick_white_box_3] , t_end[pick_white_box_4] )
 GC371: max_t_start(pick_white_box_3,_pick_white_box_4) = MAX (
   t_start[pick_white_box_3] , t_start[pick_white_box_4] )
 GC372: min_t_end(pick_white_box_3,_place_orange_box_1) = MIN (
   t_end[pick_white_box_3] , t_end[place_orange_box_1] )
 GC373: max_t_start(pick_white_box_3,_place_orange_box_1) = MAX (
   t_start[pick_white_box_3] , t_start[place_orange_box_1] )
 GC374: min_t_end(pick_white_box_3,_place_orange_box_2) = MIN (
   t_end[pick_white_box_3] , t_end[place_orange_box_2] )
 GC375: max_t_start(pick_white_box_3,_place_orange_box_2) = MAX (
   t_start[pick_white_box_3] , t_start[place_orange_box_2] )
 GC376: min_t_end(pick_white_box_3,_place_orange_box_3) = MIN (
   t_end[pick_white_box_3] , t_end[place_orange_box_3] )
 GC377: max_t_start(pick_white_box_3,_place_orange_box_3) = MAX (
   t_start[pick_white_box_3] , t_start[place_orange_box_3] )
 GC378: min_t_end(pick_white_box_3,_place_orange_box_4) = MIN (
   t_end[pick_white_box_3] , t_end[place_orange_box_4] )
 GC379: max_t_start(pick_white_box_3,_place_orange_box_4) = MAX (
   t_start[pick_white_box_3] , t_start[place_orange_box_4] )
 GC380: min_t_end(pick_white_box_3,_place_blue_box_1) = MIN (
   t_end[pick_white_box_3] , t_end[place_blue_box_1] )
 GC381: max_t_start(pick_white_box_3,_place_blue_box_1) = MAX (
   t_start[pick_white_box_3] , t_start[place_blue_box_1] )
 GC382: min_t_end(pick_white_box_3,_place_blue_box_2) = MIN (
   t_end[pick_white_box_3] , t_end[place_blue_box_2] )
 GC383: max_t_start(pick_white_box_3,_place_blue_box_2) = MAX (
   t_start[pick_white_box_3] , t_start[place_blue_box_2] )
 GC384: min_t_end(pick_white_box_3,_place_blue_box_3) = MIN (
   t_end[pick_white_box_3] , t_end[place_blue_box_3] )
 GC385: max_t_start(pick_white_box_3,_place_blue_box_3) = MAX (
   t_start[pick_white_box_3] , t_start[place_blue_box_3] )
 GC386: min_t_end(pick_white_box_3,_place_blue_box_4) = MIN (
   t_end[pick_white_box_3] , t_end[place_blue_box_4] )
 GC387: max_t_start(pick_white_box_3,_place_blue_box_4) = MAX (
   t_start[pick_white_box_3] , t_start[place_blue_box_4] )
 GC388: min_t_end(pick_white_box_3,_place_white_box_1) = MIN (
   t_end[pick_white_box_3] , t_end[place_white_box_1] )
 GC389: max_t_start(pick_white_box_3,_place_white_box_1) = MAX (
   t_start[pick_white_box_3] , t_start[place_white_box_1] )
 GC390: min_t_end(pick_white_box_3,_place_white_box_2) = MIN (
   t_end[pick_white_box_3] , t_end[place_white_box_2] )
 GC391: max_t_start(pick_white_box_3,_place_white_box_2) = MAX (
   t_start[pick_white_box_3] , t_start[place_white_box_2] )
 GC392: min_t_end(pick_white_box_3,_place_white_box_3) = MIN (
   t_end[pick_white_box_3] , t_end[place_white_box_3] )
 GC393: max_t_start(pick_white_box_3,_place_white_box_3) = MAX (
   t_start[pick_white_box_3] , t_start[place_white_box_3] )
 GC394: min_t_end(pick_white_box_3,_place_white_box_4) = MIN (
   t_end[pick_white_box_3] , t_end[place_white_box_4] )
 GC395: max_t_start(pick_white_box_3,_place_white_box_4) = MAX (
   t_start[pick_white_box_3] , t_start[place_white_box_4] )
 GC396: min_t_end(pick_white_box_4,_place_orange_box_1) = MIN (
   t_end[pick_white_box_4] , t_end[place_orange_box_1] )
 GC397: max_t_start(pick_white_box_4,_place_orange_box_1) = MAX (
   t_start[pick_white_box_4] , t_start[place_orange_box_1] )
 GC398: min_t_end(pick_white_box_4,_place_orange_box_2) = MIN (
   t_end[pick_white_box_4] , t_end[place_orange_box_2] )
 GC399: max_t_start(pick_white_box_4,_place_orange_box_2) = MAX (
   t_start[pick_white_box_4] , t_start[place_orange_box_2] )
 GC400: min_t_end(pick_white_box_4,_place_orange_box_3) = MIN (
   t_end[pick_white_box_4] , t_end[place_orange_box_3] )
 GC401: max_t_start(pick_white_box_4,_place_orange_box_3) = MAX (
   t_start[pick_white_box_4] , t_start[place_orange_box_3] )
 GC402: min_t_end(pick_white_box_4,_place_orange_box_4) = MIN (
   t_end[pick_white_box_4] , t_end[place_orange_box_4] )
 GC403: max_t_start(pick_white_box_4,_place_orange_box_4) = MAX (
   t_start[pick_white_box_4] , t_start[place_orange_box_4] )
 GC404: min_t_end(pick_white_box_4,_place_blue_box_1) = MIN (
   t_end[pick_white_box_4] , t_end[place_blue_box_1] )
 GC405: max_t_start(pick_white_box_4,_place_blue_box_1) = MAX (
   t_start[pick_white_box_4] , t_start[place_blue_box_1] )
 GC406: min_t_end(pick_white_box_4,_place_blue_box_2) = MIN (
   t_end[pick_white_box_4] , t_end[place_blue_box_2] )
 GC407: max_t_start(pick_white_box_4,_place_blue_box_2) = MAX (
   t_start[pick_white_box_4] , t_start[place_blue_box_2] )
 GC408: min_t_end(pick_white_box_4,_place_blue_box_3) = MIN (
   t_end[pick_white_box_4] , t_end[place_blue_box_3] )
 GC409: max_t_start(pick_white_box_4,_place_blue_box_3) = MAX (
   t_start[pick_white_box_4] , t_start[place_blue_box_3] )
 GC410: min_t_end(pick_white_box_4,_place_blue_box_4) = MIN (
   t_end[pick_white_box_4] , t_end[place_blue_box_4] )
 GC411: max_t_start(pick_white_box_4,_place_blue_box_4) = MAX (
   t_start[pick_white_box_4] , t_start[place_blue_box_4] )
 GC412: min_t_end(pick_white_box_4,_place_white_box_1) = MIN (
   t_end[pick_white_box_4] , t_end[place_white_box_1] )
 GC413: max_t_start(pick_white_box_4,_place_white_box_1) = MAX (
   t_start[pick_white_box_4] , t_start[place_white_box_1] )
 GC414: min_t_end(pick_white_box_4,_place_white_box_2) = MIN (
   t_end[pick_white_box_4] , t_end[place_white_box_2] )
 GC415: max_t_start(pick_white_box_4,_place_white_box_2) = MAX (
   t_start[pick_white_box_4] , t_start[place_white_box_2] )
 GC416: min_t_end(pick_white_box_4,_place_white_box_3) = MIN (
   t_end[pick_white_box_4] , t_end[place_white_box_3] )
 GC417: max_t_start(pick_white_box_4,_place_white_box_3) = MAX (
   t_start[pick_white_box_4] , t_start[place_white_box_3] )
 GC418: min_t_end(pick_white_box_4,_place_white_box_4) = MIN (
   t_end[pick_white_box_4] , t_end[place_white_box_4] )
 GC419: max_t_start(pick_white_box_4,_place_white_box_4) = MAX (
   t_start[pick_white_box_4] , t_start[place_white_box_4] )
 GC420: min_t_end(place_orange_box_1,_place_orange_box_2) = MIN (
   t_end[place_orange_box_1] , t_end[place_orange_box_2] )
 GC421: max_t_start(place_orange_box_1,_place_orange_box_2) = MAX (
   t_start[place_orange_box_1] , t_start[place_orange_box_2] )
 GC422: min_t_end(place_orange_box_1,_place_orange_box_3) = MIN (
   t_end[place_orange_box_1] , t_end[place_orange_box_3] )
 GC423: max_t_start(place_orange_box_1,_place_orange_box_3) = MAX (
   t_start[place_orange_box_1] , t_start[place_orange_box_3] )
 GC424: min_t_end(place_orange_box_1,_place_orange_box_4) = MIN (
   t_end[place_orange_box_1] , t_end[place_orange_box_4] )
 GC425: max_t_start(place_orange_box_1,_place_orange_box_4) = MAX (
   t_start[place_orange_box_1] , t_start[place_orange_box_4] )
 GC426: min_t_end(place_orange_box_1,_place_blue_box_1) = MIN (
   t_end[place_orange_box_1] , t_end[place_blue_box_1] )
 GC427: max_t_start(place_orange_box_1,_place_blue_box_1) = MAX (
   t_start[place_orange_box_1] , t_start[place_blue_box_1] )
 GC428: min_t_end(place_orange_box_1,_place_blue_box_2) = MIN (
   t_end[place_orange_box_1] , t_end[place_blue_box_2] )
 GC429: max_t_start(place_orange_box_1,_place_blue_box_2) = MAX (
   t_start[place_orange_box_1] , t_start[place_blue_box_2] )
 GC430: min_t_end(place_orange_box_1,_place_blue_box_3) = MIN (
   t_end[place_orange_box_1] , t_end[place_blue_box_3] )
 GC431: max_t_start(place_orange_box_1,_place_blue_box_3) = MAX (
   t_start[place_orange_box_1] , t_start[place_blue_box_3] )
 GC432: min_t_end(place_orange_box_1,_place_blue_box_4) = MIN (
   t_end[place_orange_box_1] , t_end[place_blue_box_4] )
 GC433: max_t_start(place_orange_box_1,_place_blue_box_4) = MAX (
   t_start[place_orange_box_1] , t_start[place_blue_box_4] )
 GC434: min_t_end(place_orange_box_1,_place_white_box_1) = MIN (
   t_end[place_orange_box_1] , t_end[place_white_box_1] )
 GC435: max_t_start(place_orange_box_1,_place_white_box_1) = MAX (
   t_start[place_orange_box_1] , t_start[place_white_box_1] )
 GC436: min_t_end(place_orange_box_1,_place_white_box_2) = MIN (
   t_end[place_orange_box_1] , t_end[place_white_box_2] )
 GC437: max_t_start(place_orange_box_1,_place_white_box_2) = MAX (
   t_start[place_orange_box_1] , t_start[place_white_box_2] )
 GC438: min_t_end(place_orange_box_1,_place_white_box_3) = MIN (
   t_end[place_orange_box_1] , t_end[place_white_box_3] )
 GC439: max_t_start(place_orange_box_1,_place_white_box_3) = MAX (
   t_start[place_orange_box_1] , t_start[place_white_box_3] )
 GC440: min_t_end(place_orange_box_1,_place_white_box_4) = MIN (
   t_end[place_orange_box_1] , t_end[place_white_box_4] )
 GC441: max_t_start(place_orange_box_1,_place_white_box_4) = MAX (
   t_start[place_orange_box_1] , t_start[place_white_box_4] )
 GC442: min_t_end(place_orange_box_2,_place_orange_box_3) = MIN (
   t_end[place_orange_box_2] , t_end[place_orange_box_3] )
 GC443: max_t_start(place_orange_box_2,_place_orange_box_3) = MAX (
   t_start[place_orange_box_2] , t_start[place_orange_box_3] )
 GC444: min_t_end(place_orange_box_2,_place_orange_box_4) = MIN (
   t_end[place_orange_box_2] , t_end[place_orange_box_4] )
 GC445: max_t_start(place_orange_box_2,_place_orange_box_4) = MAX (
   t_start[place_orange_box_2] , t_start[place_orange_box_4] )
 GC446: min_t_end(place_orange_box_2,_place_blue_box_1) = MIN (
   t_end[place_orange_box_2] , t_end[place_blue_box_1] )
 GC447: max_t_start(place_orange_box_2,_place_blue_box_1) = MAX (
   t_start[place_orange_box_2] , t_start[place_blue_box_1] )
 GC448: min_t_end(place_orange_box_2,_place_blue_box_2) = MIN (
   t_end[place_orange_box_2] , t_end[place_blue_box_2] )
 GC449: max_t_start(place_orange_box_2,_place_blue_box_2) = MAX (
   t_start[place_orange_box_2] , t_start[place_blue_box_2] )
 GC450: min_t_end(place_orange_box_2,_place_blue_box_3) = MIN (
   t_end[place_orange_box_2] , t_end[place_blue_box_3] )
 GC451: max_t_start(place_orange_box_2,_place_blue_box_3) = MAX (
   t_start[place_orange_box_2] , t_start[place_blue_box_3] )
 GC452: min_t_end(place_orange_box_2,_place_blue_box_4) = MIN (
   t_end[place_orange_box_2] , t_end[place_blue_box_4] )
 GC453: max_t_start(place_orange_box_2,_place_blue_box_4) = MAX (
   t_start[place_orange_box_2] , t_start[place_blue_box_4] )
 GC454: min_t_end(place_orange_box_2,_place_white_box_1) = MIN (
   t_end[place_orange_box_2] , t_end[place_white_box_1] )
 GC455: max_t_start(place_orange_box_2,_place_white_box_1) = MAX (
   t_start[place_orange_box_2] , t_start[place_white_box_1] )
 GC456: min_t_end(place_orange_box_2,_place_white_box_2) = MIN (
   t_end[place_orange_box_2] , t_end[place_white_box_2] )
 GC457: max_t_start(place_orange_box_2,_place_white_box_2) = MAX (
   t_start[place_orange_box_2] , t_start[place_white_box_2] )
 GC458: min_t_end(place_orange_box_2,_place_white_box_3) = MIN (
   t_end[place_orange_box_2] , t_end[place_white_box_3] )
 GC459: max_t_start(place_orange_box_2,_place_white_box_3) = MAX (
   t_start[place_orange_box_2] , t_start[place_white_box_3] )
 GC460: min_t_end(place_orange_box_2,_place_white_box_4) = MIN (
   t_end[place_orange_box_2] , t_end[place_white_box_4] )
 GC461: max_t_start(place_orange_box_2,_place_white_box_4) = MAX (
   t_start[place_orange_box_2] , t_start[place_white_box_4] )
 GC462: min_t_end(place_orange_box_3,_place_orange_box_4) = MIN (
   t_end[place_orange_box_3] , t_end[place_orange_box_4] )
 GC463: max_t_start(place_orange_box_3,_place_orange_box_4) = MAX (
   t_start[place_orange_box_3] , t_start[place_orange_box_4] )
 GC464: min_t_end(place_orange_box_3,_place_blue_box_1) = MIN (
   t_end[place_orange_box_3] , t_end[place_blue_box_1] )
 GC465: max_t_start(place_orange_box_3,_place_blue_box_1) = MAX (
   t_start[place_orange_box_3] , t_start[place_blue_box_1] )
 GC466: min_t_end(place_orange_box_3,_place_blue_box_2) = MIN (
   t_end[place_orange_box_3] , t_end[place_blue_box_2] )
 GC467: max_t_start(place_orange_box_3,_place_blue_box_2) = MAX (
   t_start[place_orange_box_3] , t_start[place_blue_box_2] )
 GC468: min_t_end(place_orange_box_3,_place_blue_box_3) = MIN (
   t_end[place_orange_box_3] , t_end[place_blue_box_3] )
 GC469: max_t_start(place_orange_box_3,_place_blue_box_3) = MAX (
   t_start[place_orange_box_3] , t_start[place_blue_box_3] )
 GC470: min_t_end(place_orange_box_3,_place_blue_box_4) = MIN (
   t_end[place_orange_box_3] , t_end[place_blue_box_4] )
 GC471: max_t_start(place_orange_box_3,_place_blue_box_4) = MAX (
   t_start[place_orange_box_3] , t_start[place_blue_box_4] )
 GC472: min_t_end(place_orange_box_3,_place_white_box_1) = MIN (
   t_end[place_orange_box_3] , t_end[place_white_box_1] )
 GC473: max_t_start(place_orange_box_3,_place_white_box_1) = MAX (
   t_start[place_orange_box_3] , t_start[place_white_box_1] )
 GC474: min_t_end(place_orange_box_3,_place_white_box_2) = MIN (
   t_end[place_orange_box_3] , t_end[place_white_box_2] )
 GC475: max_t_start(place_orange_box_3,_place_white_box_2) = MAX (
   t_start[place_orange_box_3] , t_start[place_white_box_2] )
 GC476: min_t_end(place_orange_box_3,_place_white_box_3) = MIN (
   t_end[place_orange_box_3] , t_end[place_white_box_3] )
 GC477: max_t_start(place_orange_box_3,_place_white_box_3) = MAX (
   t_start[place_orange_box_3] , t_start[place_white_box_3] )
 GC478: min_t_end(place_orange_box_3,_place_white_box_4) = MIN (
   t_end[place_orange_box_3] , t_end[place_white_box_4] )
 GC479: max_t_start(place_orange_box_3,_place_white_box_4) = MAX (
   t_start[place_orange_box_3] , t_start[place_white_box_4] )
 GC480: min_t_end(place_orange_box_4,_place_blue_box_1) = MIN (
   t_end[place_orange_box_4] , t_end[place_blue_box_1] )
 GC481: max_t_start(place_orange_box_4,_place_blue_box_1) = MAX (
   t_start[place_orange_box_4] , t_start[place_blue_box_1] )
 GC482: min_t_end(place_orange_box_4,_place_blue_box_2) = MIN (
   t_end[place_orange_box_4] , t_end[place_blue_box_2] )
 GC483: max_t_start(place_orange_box_4,_place_blue_box_2) = MAX (
   t_start[place_orange_box_4] , t_start[place_blue_box_2] )
 GC484: min_t_end(place_orange_box_4,_place_blue_box_3) = MIN (
   t_end[place_orange_box_4] , t_end[place_blue_box_3] )
 GC485: max_t_start(place_orange_box_4,_place_blue_box_3) = MAX (
   t_start[place_orange_box_4] , t_start[place_blue_box_3] )
 GC486: min_t_end(place_orange_box_4,_place_blue_box_4) = MIN (
   t_end[place_orange_box_4] , t_end[place_blue_box_4] )
 GC487: max_t_start(place_orange_box_4,_place_blue_box_4) = MAX (
   t_start[place_orange_box_4] , t_start[place_blue_box_4] )
 GC488: min_t_end(place_orange_box_4,_place_white_box_1) = MIN (
   t_end[place_orange_box_4] , t_end[place_white_box_1] )
 GC489: max_t_start(place_orange_box_4,_place_white_box_1) = MAX (
   t_start[place_orange_box_4] , t_start[place_white_box_1] )
 GC490: min_t_end(place_orange_box_4,_place_white_box_2) = MIN (
   t_end[place_orange_box_4] , t_end[place_white_box_2] )
 GC491: max_t_start(place_orange_box_4,_place_white_box_2) = MAX (
   t_start[place_orange_box_4] , t_start[place_white_box_2] )
 GC492: min_t_end(place_orange_box_4,_place_white_box_3) = MIN (
   t_end[place_orange_box_4] , t_end[place_white_box_3] )
 GC493: max_t_start(place_orange_box_4,_place_white_box_3) = MAX (
   t_start[place_orange_box_4] , t_start[place_white_box_3] )
 GC494: min_t_end(place_orange_box_4,_place_white_box_4) = MIN (
   t_end[place_orange_box_4] , t_end[place_white_box_4] )
 GC495: max_t_start(place_orange_box_4,_place_white_box_4) = MAX (
   t_start[place_orange_box_4] , t_start[place_white_box_4] )
 GC496: min_t_end(place_blue_box_1,_place_blue_box_2) = MIN (
   t_end[place_blue_box_1] , t_end[place_blue_box_2] )
 GC497: max_t_start(place_blue_box_1,_place_blue_box_2) = MAX (
   t_start[place_blue_box_1] , t_start[place_blue_box_2] )
 GC498: min_t_end(place_blue_box_1,_place_blue_box_3) = MIN (
   t_end[place_blue_box_1] , t_end[place_blue_box_3] )
 GC499: max_t_start(place_blue_box_1,_place_blue_box_3) = MAX (
   t_start[place_blue_box_1] , t_start[place_blue_box_3] )
 GC500: min_t_end(place_blue_box_1,_place_blue_box_4) = MIN (
   t_end[place_blue_box_1] , t_end[place_blue_box_4] )
 GC501: max_t_start(place_blue_box_1,_place_blue_box_4) = MAX (
   t_start[place_blue_box_1] , t_start[place_blue_box_4] )
 GC502: min_t_end(place_blue_box_1,_place_white_box_1) = MIN (
   t_end[place_blue_box_1] , t_end[place_white_box_1] )
 GC503: max_t_start(place_blue_box_1,_place_white_box_1) = MAX (
   t_start[place_blue_box_1] , t_start[place_white_box_1] )
 GC504: min_t_end(place_blue_box_1,_place_white_box_2) = MIN (
   t_end[place_blue_box_1] , t_end[place_white_box_2] )
 GC505: max_t_start(place_blue_box_1,_place_white_box_2) = MAX (
   t_start[place_blue_box_1] , t_start[place_white_box_2] )
 GC506: min_t_end(place_blue_box_1,_place_white_box_3) = MIN (
   t_end[place_blue_box_1] , t_end[place_white_box_3] )
 GC507: max_t_start(place_blue_box_1,_place_white_box_3) = MAX (
   t_start[place_blue_box_1] , t_start[place_white_box_3] )
 GC508: min_t_end(place_blue_box_1,_place_white_box_4) = MIN (
   t_end[place_blue_box_1] , t_end[place_white_box_4] )
 GC509: max_t_start(place_blue_box_1,_place_white_box_4) = MAX (
   t_start[place_blue_box_1] , t_start[place_white_box_4] )
 GC510: min_t_end(place_blue_box_2,_place_blue_box_3) = MIN (
   t_end[place_blue_box_2] , t_end[place_blue_box_3] )
 GC511: max_t_start(place_blue_box_2,_place_blue_box_3) = MAX (
   t_start[place_blue_box_2] , t_start[place_blue_box_3] )
 GC512: min_t_end(place_blue_box_2,_place_blue_box_4) = MIN (
   t_end[place_blue_box_2] , t_end[place_blue_box_4] )
 GC513: max_t_start(place_blue_box_2,_place_blue_box_4) = MAX (
   t_start[place_blue_box_2] , t_start[place_blue_box_4] )
 GC514: min_t_end(place_blue_box_2,_place_white_box_1) = MIN (
   t_end[place_blue_box_2] , t_end[place_white_box_1] )
 GC515: max_t_start(place_blue_box_2,_place_white_box_1) = MAX (
   t_start[place_blue_box_2] , t_start[place_white_box_1] )
 GC516: min_t_end(place_blue_box_2,_place_white_box_2) = MIN (
   t_end[place_blue_box_2] , t_end[place_white_box_2] )
 GC517: max_t_start(place_blue_box_2,_place_white_box_2) = MAX (
   t_start[place_blue_box_2] , t_start[place_white_box_2] )
 GC518: min_t_end(place_blue_box_2,_place_white_box_3) = MIN (
   t_end[place_blue_box_2] , t_end[place_white_box_3] )
 GC519: max_t_start(place_blue_box_2,_place_white_box_3) = MAX (
   t_start[place_blue_box_2] , t_start[place_white_box_3] )
 GC520: min_t_end(place_blue_box_2,_place_white_box_4) = MIN (
   t_end[place_blue_box_2] , t_end[place_white_box_4] )
 GC521: max_t_start(place_blue_box_2,_place_white_box_4) = MAX (
   t_start[place_blue_box_2] , t_start[place_white_box_4] )
 GC522: min_t_end(place_blue_box_3,_place_blue_box_4) = MIN (
   t_end[place_blue_box_3] , t_end[place_blue_box_4] )
 GC523: max_t_start(place_blue_box_3,_place_blue_box_4) = MAX (
   t_start[place_blue_box_3] , t_start[place_blue_box_4] )
 GC524: min_t_end(place_blue_box_3,_place_white_box_1) = MIN (
   t_end[place_blue_box_3] , t_end[place_white_box_1] )
 GC525: max_t_start(place_blue_box_3,_place_white_box_1) = MAX (
   t_start[place_blue_box_3] , t_start[place_white_box_1] )
 GC526: min_t_end(place_blue_box_3,_place_white_box_2) = MIN (
   t_end[place_blue_box_3] , t_end[place_white_box_2] )
 GC527: max_t_start(place_blue_box_3,_place_white_box_2) = MAX (
   t_start[place_blue_box_3] , t_start[place_white_box_2] )
 GC528: min_t_end(place_blue_box_3,_place_white_box_3) = MIN (
   t_end[place_blue_box_3] , t_end[place_white_box_3] )
 GC529: max_t_start(place_blue_box_3,_place_white_box_3) = MAX (
   t_start[place_blue_box_3] , t_start[place_white_box_3] )
 GC530: min_t_end(place_blue_box_3,_place_white_box_4) = MIN (
   t_end[place_blue_box_3] , t_end[place_white_box_4] )
 GC531: max_t_start(place_blue_box_3,_place_white_box_4) = MAX (
   t_start[place_blue_box_3] , t_start[place_white_box_4] )
 GC532: min_t_end(place_blue_box_4,_place_white_box_1) = MIN (
   t_end[place_blue_box_4] , t_end[place_white_box_1] )
 GC533: max_t_start(place_blue_box_4,_place_white_box_1) = MAX (
   t_start[place_blue_box_4] , t_start[place_white_box_1] )
 GC534: min_t_end(place_blue_box_4,_place_white_box_2) = MIN (
   t_end[place_blue_box_4] , t_end[place_white_box_2] )
 GC535: max_t_start(place_blue_box_4,_place_white_box_2) = MAX (
   t_start[place_blue_box_4] , t_start[place_white_box_2] )
 GC536: min_t_end(place_blue_box_4,_place_white_box_3) = MIN (
   t_end[place_blue_box_4] , t_end[place_white_box_3] )
 GC537: max_t_start(place_blue_box_4,_place_white_box_3) = MAX (
   t_start[place_blue_box_4] , t_start[place_white_box_3] )
 GC538: min_t_end(place_blue_box_4,_place_white_box_4) = MIN (
   t_end[place_blue_box_4] , t_end[place_white_box_4] )
 GC539: max_t_start(place_blue_box_4,_place_white_box_4) = MAX (
   t_start[place_blue_box_4] , t_start[place_white_box_4] )
 GC540: min_t_end(place_white_box_1,_place_white_box_2) = MIN (
   t_end[place_white_box_1] , t_end[place_white_box_2] )
 GC541: max_t_start(place_white_box_1,_place_white_box_2) = MAX (
   t_start[place_white_box_1] , t_start[place_white_box_2] )
 GC542: min_t_end(place_white_box_1,_place_white_box_3) = MIN (
   t_end[place_white_box_1] , t_end[place_white_box_3] )
 GC543: max_t_start(place_white_box_1,_place_white_box_3) = MAX (
   t_start[place_white_box_1] , t_start[place_white_box_3] )
 GC544: min_t_end(place_white_box_1,_place_white_box_4) = MIN (
   t_end[place_white_box_1] , t_end[place_white_box_4] )
 GC545: max_t_start(place_white_box_1,_place_white_box_4) = MAX (
   t_start[place_white_box_1] , t_start[place_white_box_4] )
 GC546: min_t_end(place_white_box_2,_place_white_box_3) = MIN (
   t_end[place_white_box_2] , t_end[place_white_box_3] )
 GC547: max_t_start(place_white_box_2,_place_white_box_3) = MAX (
   t_start[place_white_box_2] , t_start[place_white_box_3] )
 GC548: min_t_end(place_white_box_2,_place_white_box_4) = MIN (
   t_end[place_white_box_2] , t_end[place_white_box_4] )
 GC549: max_t_start(place_white_box_2,_place_white_box_4) = MAX (
   t_start[place_white_box_2] , t_start[place_white_box_4] )
 GC550: min_t_end(place_white_box_3,_place_white_box_4) = MIN (
   t_end[place_white_box_3] , t_end[place_white_box_4] )
 GC551: max_t_start(place_white_box_3,_place_white_box_4) = MAX (
   t_start[place_white_box_3] , t_start[place_white_box_4] )
 GC552: both_pick_orange_box_1_pick_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,pick_orange_box_2] )
 GC557: both_pick_orange_box_1_pick_orange_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,pick_orange_box_3] )
 GC562: both_pick_orange_box_1_pick_orange_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,pick_orange_box_4] )
 GC567: both_pick_orange_box_1_pick_blue_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,pick_blue_box_1] )
 GC572: both_pick_orange_box_1_pick_blue_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,pick_blue_box_2] )
 GC577: both_pick_orange_box_1_pick_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,pick_blue_box_3] )
 GC582: both_pick_orange_box_1_pick_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,pick_blue_box_4] )
 GC587: both_pick_orange_box_1_place_orange_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,place_orange_box_1] )
 GC592: both_pick_orange_box_1_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC597: both_pick_orange_box_1_place_orange_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,place_orange_box_3] )
 GC602: both_pick_orange_box_1_place_orange_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,place_orange_box_4] )
 GC607: both_pick_orange_box_1_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC612: both_pick_orange_box_1_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_1] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC617: both_pick_orange_box_2_pick_orange_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,pick_orange_box_3] )
 GC622: both_pick_orange_box_2_pick_orange_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,pick_orange_box_4] )
 GC627: both_pick_orange_box_2_pick_blue_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,pick_blue_box_1] )
 GC632: both_pick_orange_box_2_pick_blue_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,pick_blue_box_2] )
 GC637: both_pick_orange_box_2_pick_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,pick_blue_box_3] )
 GC642: both_pick_orange_box_2_pick_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,pick_blue_box_4] )
 GC647: both_pick_orange_box_2_place_orange_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,place_orange_box_1] )
 GC652: both_pick_orange_box_2_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC657: both_pick_orange_box_2_place_orange_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,place_orange_box_3] )
 GC662: both_pick_orange_box_2_place_orange_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,place_orange_box_4] )
 GC667: both_pick_orange_box_2_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC672: both_pick_orange_box_2_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_2] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC677: both_pick_orange_box_3_pick_orange_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_3] ,
   assignment[ur5_on_guide,pick_orange_box_4] )
 GC682: both_pick_orange_box_3_pick_blue_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_3] ,
   assignment[ur5_on_guide,pick_blue_box_1] )
 GC687: both_pick_orange_box_3_pick_blue_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_3] ,
   assignment[ur5_on_guide,pick_blue_box_2] )
 GC692: both_pick_orange_box_3_pick_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_3] ,
   assignment[ur5_on_guide,pick_blue_box_3] )
 GC697: both_pick_orange_box_3_pick_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_3] ,
   assignment[ur5_on_guide,pick_blue_box_4] )
 GC702: both_pick_orange_box_3_place_orange_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_3] ,
   assignment[ur5_on_guide,place_orange_box_1] )
 GC707: both_pick_orange_box_3_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_3] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC712: both_pick_orange_box_3_place_orange_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_3] ,
   assignment[ur5_on_guide,place_orange_box_3] )
 GC717: both_pick_orange_box_3_place_orange_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_3] ,
   assignment[ur5_on_guide,place_orange_box_4] )
 GC722: both_pick_orange_box_3_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_3] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC727: both_pick_orange_box_3_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_3] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC732: both_pick_orange_box_4_pick_blue_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_4] ,
   assignment[ur5_on_guide,pick_blue_box_1] )
 GC737: both_pick_orange_box_4_pick_blue_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_4] ,
   assignment[ur5_on_guide,pick_blue_box_2] )
 GC742: both_pick_orange_box_4_pick_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_4] ,
   assignment[ur5_on_guide,pick_blue_box_3] )
 GC747: both_pick_orange_box_4_pick_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_4] ,
   assignment[ur5_on_guide,pick_blue_box_4] )
 GC752: both_pick_orange_box_4_place_orange_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_4] ,
   assignment[ur5_on_guide,place_orange_box_1] )
 GC757: both_pick_orange_box_4_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_4] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC762: both_pick_orange_box_4_place_orange_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_4] ,
   assignment[ur5_on_guide,place_orange_box_3] )
 GC767: both_pick_orange_box_4_place_orange_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_4] ,
   assignment[ur5_on_guide,place_orange_box_4] )
 GC772: both_pick_orange_box_4_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_4] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC777: both_pick_orange_box_4_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_orange_box_4] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC782: both_pick_blue_box_1_pick_blue_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,pick_blue_box_2] )
 GC787: both_pick_blue_box_1_pick_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,pick_blue_box_3] )
 GC792: both_pick_blue_box_1_pick_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,pick_blue_box_4] )
 GC797: both_pick_blue_box_1_place_orange_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,place_orange_box_1] )
 GC802: both_pick_blue_box_1_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC807: both_pick_blue_box_1_place_orange_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,place_orange_box_3] )
 GC812: both_pick_blue_box_1_place_orange_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,place_orange_box_4] )
 GC817: both_pick_blue_box_1_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC822: both_pick_blue_box_1_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_1] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC827: both_pick_blue_box_2_pick_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_2] ,
   assignment[ur5_on_guide,pick_blue_box_3] )
 GC832: both_pick_blue_box_2_pick_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_2] ,
   assignment[ur5_on_guide,pick_blue_box_4] )
 GC837: both_pick_blue_box_2_place_orange_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_2] ,
   assignment[ur5_on_guide,place_orange_box_1] )
 GC842: both_pick_blue_box_2_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_2] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC847: both_pick_blue_box_2_place_orange_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_2] ,
   assignment[ur5_on_guide,place_orange_box_3] )
 GC852: both_pick_blue_box_2_place_orange_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_2] ,
   assignment[ur5_on_guide,place_orange_box_4] )
 GC857: both_pick_blue_box_2_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_2] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC862: both_pick_blue_box_2_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_2] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC867: both_pick_blue_box_3_pick_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_3] ,
   assignment[ur5_on_guide,pick_blue_box_4] )
 GC872: both_pick_blue_box_3_place_orange_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_3] ,
   assignment[ur5_on_guide,place_orange_box_1] )
 GC877: both_pick_blue_box_3_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_3] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC882: both_pick_blue_box_3_place_orange_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_3] ,
   assignment[ur5_on_guide,place_orange_box_3] )
 GC887: both_pick_blue_box_3_place_orange_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_3] ,
   assignment[ur5_on_guide,place_orange_box_4] )
 GC892: both_pick_blue_box_3_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_3] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC897: both_pick_blue_box_3_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_3] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC902: both_pick_blue_box_4_place_orange_box_1_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_4] ,
   assignment[ur5_on_guide,place_orange_box_1] )
 GC907: both_pick_blue_box_4_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_4] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC912: both_pick_blue_box_4_place_orange_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_4] ,
   assignment[ur5_on_guide,place_orange_box_3] )
 GC917: both_pick_blue_box_4_place_orange_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_4] ,
   assignment[ur5_on_guide,place_orange_box_4] )
 GC922: both_pick_blue_box_4_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_4] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC927: both_pick_blue_box_4_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,pick_blue_box_4] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC932: both_place_orange_box_1_place_orange_box_2_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_1] ,
   assignment[ur5_on_guide,place_orange_box_2] )
 GC937: both_place_orange_box_1_place_orange_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_1] ,
   assignment[ur5_on_guide,place_orange_box_3] )
 GC942: both_place_orange_box_1_place_orange_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_1] ,
   assignment[ur5_on_guide,place_orange_box_4] )
 GC947: both_place_orange_box_1_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_1] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC952: both_place_orange_box_1_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_1] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC957: both_place_orange_box_2_place_orange_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_2] ,
   assignment[ur5_on_guide,place_orange_box_3] )
 GC962: both_place_orange_box_2_place_orange_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_2] ,
   assignment[ur5_on_guide,place_orange_box_4] )
 GC967: both_place_orange_box_2_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_2] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC972: both_place_orange_box_2_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_2] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC977: both_place_orange_box_3_place_orange_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_3] ,
   assignment[ur5_on_guide,place_orange_box_4] )
 GC982: both_place_orange_box_3_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_3] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC987: both_place_orange_box_3_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_3] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC992: both_place_orange_box_4_place_blue_box_3_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_4] ,
   assignment[ur5_on_guide,place_blue_box_3] )
 GC997: both_place_orange_box_4_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_orange_box_4] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC1002: both_place_blue_box_3_place_blue_box_4_to_ur5_on_guide = AND (
   assignment[ur5_on_guide,place_blue_box_3] ,
   assignment[ur5_on_guide,place_blue_box_4] )
 GC1007: both_pick_blue_box_1_pick_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,pick_blue_box_2] )
 GC1012: both_pick_blue_box_1_pick_blue_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,pick_blue_box_3] )
 GC1017: both_pick_blue_box_1_pick_blue_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,pick_blue_box_4] )
 GC1022: both_pick_blue_box_1_pick_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,pick_white_box_1] )
 GC1027: both_pick_blue_box_1_pick_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,pick_white_box_2] )
 GC1032: both_pick_blue_box_1_pick_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,pick_white_box_3] )
 GC1037: both_pick_blue_box_1_pick_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,pick_white_box_4] )
 GC1042: both_pick_blue_box_1_place_blue_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,place_blue_box_1] )
 GC1047: both_pick_blue_box_1_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC1052: both_pick_blue_box_1_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,place_white_box_1] )
 GC1057: both_pick_blue_box_1_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,place_white_box_2] )
 GC1062: both_pick_blue_box_1_place_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,place_white_box_3] )
 GC1067: both_pick_blue_box_1_place_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_1] ,
   assignment[human_right_arm,place_white_box_4] )
 GC1072: both_pick_blue_box_2_pick_blue_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,pick_blue_box_3] )
 GC1077: both_pick_blue_box_2_pick_blue_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,pick_blue_box_4] )
 GC1082: both_pick_blue_box_2_pick_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,pick_white_box_1] )
 GC1087: both_pick_blue_box_2_pick_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,pick_white_box_2] )
 GC1092: both_pick_blue_box_2_pick_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,pick_white_box_3] )
 GC1097: both_pick_blue_box_2_pick_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,pick_white_box_4] )
 GC1102: both_pick_blue_box_2_place_blue_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,place_blue_box_1] )
 GC1107: both_pick_blue_box_2_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC1112: both_pick_blue_box_2_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,place_white_box_1] )
 GC1117: both_pick_blue_box_2_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,place_white_box_2] )
 GC1122: both_pick_blue_box_2_place_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,place_white_box_3] )
 GC1127: both_pick_blue_box_2_place_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_2] ,
   assignment[human_right_arm,place_white_box_4] )
 GC1132: both_pick_blue_box_3_pick_blue_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,pick_blue_box_4] )
 GC1137: both_pick_blue_box_3_pick_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,pick_white_box_1] )
 GC1142: both_pick_blue_box_3_pick_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,pick_white_box_2] )
 GC1147: both_pick_blue_box_3_pick_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,pick_white_box_3] )
 GC1152: both_pick_blue_box_3_pick_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,pick_white_box_4] )
 GC1157: both_pick_blue_box_3_place_blue_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,place_blue_box_1] )
 GC1162: both_pick_blue_box_3_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC1167: both_pick_blue_box_3_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,place_white_box_1] )
 GC1172: both_pick_blue_box_3_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,place_white_box_2] )
 GC1177: both_pick_blue_box_3_place_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,place_white_box_3] )
 GC1182: both_pick_blue_box_3_place_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_3] ,
   assignment[human_right_arm,place_white_box_4] )
 GC1187: both_pick_blue_box_4_pick_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,pick_white_box_1] )
 GC1192: both_pick_blue_box_4_pick_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,pick_white_box_2] )
 GC1197: both_pick_blue_box_4_pick_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,pick_white_box_3] )
 GC1202: both_pick_blue_box_4_pick_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,pick_white_box_4] )
 GC1207: both_pick_blue_box_4_place_blue_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,place_blue_box_1] )
 GC1212: both_pick_blue_box_4_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC1217: both_pick_blue_box_4_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,place_white_box_1] )
 GC1222: both_pick_blue_box_4_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,place_white_box_2] )
 GC1227: both_pick_blue_box_4_place_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,place_white_box_3] )
 GC1232: both_pick_blue_box_4_place_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_blue_box_4] ,
   assignment[human_right_arm,place_white_box_4] )
 GC1237: both_pick_white_box_1_pick_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_1] ,
   assignment[human_right_arm,pick_white_box_2] )
 GC1242: both_pick_white_box_1_pick_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_1] ,
   assignment[human_right_arm,pick_white_box_3] )
 GC1247: both_pick_white_box_1_pick_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_1] ,
   assignment[human_right_arm,pick_white_box_4] )
 GC1252: both_pick_white_box_1_place_blue_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_1] ,
   assignment[human_right_arm,place_blue_box_1] )
 GC1257: both_pick_white_box_1_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_1] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC1262: both_pick_white_box_1_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_1] ,
   assignment[human_right_arm,place_white_box_1] )
 GC1267: both_pick_white_box_1_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_1] ,
   assignment[human_right_arm,place_white_box_2] )
 GC1272: both_pick_white_box_1_place_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_1] ,
   assignment[human_right_arm,place_white_box_3] )
 GC1277: both_pick_white_box_1_place_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_1] ,
   assignment[human_right_arm,place_white_box_4] )
 GC1282: both_pick_white_box_2_pick_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_2] ,
   assignment[human_right_arm,pick_white_box_3] )
 GC1287: both_pick_white_box_2_pick_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_2] ,
   assignment[human_right_arm,pick_white_box_4] )
 GC1292: both_pick_white_box_2_place_blue_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_2] ,
   assignment[human_right_arm,place_blue_box_1] )
 GC1297: both_pick_white_box_2_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_2] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC1302: both_pick_white_box_2_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_2] ,
   assignment[human_right_arm,place_white_box_1] )
 GC1307: both_pick_white_box_2_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_2] ,
   assignment[human_right_arm,place_white_box_2] )
 GC1312: both_pick_white_box_2_place_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_2] ,
   assignment[human_right_arm,place_white_box_3] )
 GC1317: both_pick_white_box_2_place_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_2] ,
   assignment[human_right_arm,place_white_box_4] )
 GC1322: both_pick_white_box_3_pick_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_3] ,
   assignment[human_right_arm,pick_white_box_4] )
 GC1327: both_pick_white_box_3_place_blue_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_3] ,
   assignment[human_right_arm,place_blue_box_1] )
 GC1332: both_pick_white_box_3_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_3] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC1337: both_pick_white_box_3_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_3] ,
   assignment[human_right_arm,place_white_box_1] )
 GC1342: both_pick_white_box_3_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_3] ,
   assignment[human_right_arm,place_white_box_2] )
 GC1347: both_pick_white_box_3_place_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_3] ,
   assignment[human_right_arm,place_white_box_3] )
 GC1352: both_pick_white_box_3_place_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_3] ,
   assignment[human_right_arm,place_white_box_4] )
 GC1357: both_pick_white_box_4_place_blue_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_4] ,
   assignment[human_right_arm,place_blue_box_1] )
 GC1362: both_pick_white_box_4_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_4] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC1367: both_pick_white_box_4_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_4] ,
   assignment[human_right_arm,place_white_box_1] )
 GC1372: both_pick_white_box_4_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_4] ,
   assignment[human_right_arm,place_white_box_2] )
 GC1377: both_pick_white_box_4_place_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_4] ,
   assignment[human_right_arm,place_white_box_3] )
 GC1382: both_pick_white_box_4_place_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,pick_white_box_4] ,
   assignment[human_right_arm,place_white_box_4] )
 GC1387: both_place_blue_box_1_place_blue_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,place_blue_box_1] ,
   assignment[human_right_arm,place_blue_box_2] )
 GC1392: both_place_blue_box_1_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,place_blue_box_1] ,
   assignment[human_right_arm,place_white_box_1] )
 GC1397: both_place_blue_box_1_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,place_blue_box_1] ,
   assignment[human_right_arm,place_white_box_2] )
 GC1402: both_place_blue_box_1_place_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,place_blue_box_1] ,
   assignment[human_right_arm,place_white_box_3] )
 GC1407: both_place_blue_box_1_place_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,place_blue_box_1] ,
   assignment[human_right_arm,place_white_box_4] )
 GC1412: both_place_blue_box_2_place_white_box_1_to_human_right_arm = AND (
   assignment[human_right_arm,place_blue_box_2] ,
   assignment[human_right_arm,place_white_box_1] )
 GC1417: both_place_blue_box_2_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,place_blue_box_2] ,
   assignment[human_right_arm,place_white_box_2] )
 GC1422: both_place_blue_box_2_place_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,place_blue_box_2] ,
   assignment[human_right_arm,place_white_box_3] )
 GC1427: both_place_blue_box_2_place_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,place_blue_box_2] ,
   assignment[human_right_arm,place_white_box_4] )
 GC1432:
   both_place_white_box_1_place_white_box_2_to_human_right_arm = AND (
   assignment[human_right_arm,place_white_box_1] ,
   assignment[human_right_arm,place_white_box_2] )
 GC1437:
   both_place_white_box_1_place_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,place_white_box_1] ,
   assignment[human_right_arm,place_white_box_3] )
 GC1442:
   both_place_white_box_1_place_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,place_white_box_1] ,
   assignment[human_right_arm,place_white_box_4] )
 GC1447:
   both_place_white_box_2_place_white_box_3_to_human_right_arm = AND (
   assignment[human_right_arm,place_white_box_2] ,
   assignment[human_right_arm,place_white_box_3] )
 GC1452:
   both_place_white_box_2_place_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,place_white_box_2] ,
   assignment[human_right_arm,place_white_box_4] )
 GC1457:
   both_place_white_box_3_place_white_box_4_to_human_right_arm = AND (
   assignment[human_right_arm,place_white_box_3] ,
   assignment[human_right_arm,place_white_box_4] )
 GC1462: makespan = MAX ( t_end[pick_orange_box_1] ,
   t_end[pick_orange_box_2] , t_end[pick_orange_box_3] ,
   t_end[pick_orange_box_4] , t_end[pick_blue_box_1] ,
   t_end[pick_blue_box_2] , t_end[pick_blue_box_3] ,
   t_end[pick_blue_box_4] , t_end[pick_white_box_1] ,
   t_end[pick_white_box_2] , t_end[pick_white_box_3] ,
   t_end[pick_white_box_4] , t_end[place_orange_box_1] ,
   t_end[place_orange_box_2] , t_end[place_orange_box_3] ,
   t_end[place_orange_box_4] , t_end[place_blue_box_1] ,
   t_end[place_blue_box_2] , t_end[place_blue_box_3] ,
   t_end[place_blue_box_4] , t_end[place_white_box_1] ,
   t_end[place_white_box_2] , t_end[place_white_box_3] ,
   t_end[place_white_box_4] )
End
