#!/usr/bin/env python
import rospy
from q_model_install.msg import Q_state


if __name__=='__main__':
    rospy.init_node('q_installer')
    pub = rospy.Publisher('q_installer', Q_state, queue_size=10)
    rate=rospy.Rate(10)
    msg=Q_state()

    with open("/home/alex/catkin_ws/src/q_model_install/src/model.txt", "r") as f:
        contents = f.read()
    data = [x.strip() for x in contents.split('\n')]
    
    while not rospy.is_shutdown():
	    state_elements_0 = [float(x) for x in data[0].split(',')] 
	    msg.state0 = state_elements_0
	    state_elements_1 = [float(x) for x in data[1].split(',')] 
	    msg.state1 = state_elements_1
	    state_elements_2 = [float(x) for x in data[2].split(',')] 
	    msg.state2 = state_elements_2
	    state_elements_3 = [float(x) for x in data[3].split(',')] 
	    msg.state3 = state_elements_3
	    state_elements_4 = [float(x) for x in data[4].split(',')] 
	    msg.state4 = state_elements_4
	    state_elements_5 = [float(x) for x in data[5].split(',')] 
	    msg.state5 = state_elements_5
	    state_elements_6 = [float(x) for x in data[6].split(',')] 
	    msg.state6 = state_elements_6
	    state_elements_7 = [float(x) for x in data[7].split(',')] 
	    msg.state7 = state_elements_7
	    state_elements_8 = [float(x) for x in data[8].split(',')] 
	    msg.state8 = state_elements_8
	    state_elements_9 = [float(x) for x in data[9].split(',')] 
	    msg.state9 = state_elements_9
	    state_elements_10 = [float(x) for x in data[10].split(',')] 
	    msg.state10 = state_elements_10
	    state_elements_11 = [float(x) for x in data[11].split(',')] 
	    msg.state11 = state_elements_11
	    state_elements_12 = [float(x) for x in data[12].split(',')] 
	    msg.state12 = state_elements_12
	    state_elements_13 = [float(x) for x in data[13].split(',')] 
	    msg.state13 = state_elements_13
	    state_elements_14 = [float(x) for x in data[14].split(',')] 
	    msg.state14 = state_elements_14
	    state_elements_15 = [float(x) for x in data[15].split(',')] 
	    msg.state15 = state_elements_15
	    state_elements_16 = [float(x) for x in data[16].split(',')] 
	    msg.state16 = state_elements_16
	    state_elements_17 = [float(x) for x in data[17].split(',')] 
	    msg.state17 = state_elements_17
	    state_elements_18 = [float(x) for x in data[18].split(',')] 
	    msg.state18 = state_elements_18
	    state_elements_19 = [float(x) for x in data[19].split(',')] 
	    msg.state19 = state_elements_19
	    state_elements_20 = [float(x) for x in data[20].split(',')] 
	    msg.state20 = state_elements_20
	    state_elements_21 = [float(x) for x in data[21].split(',')] 
	    msg.state21 = state_elements_21
	    state_elements_22 = [float(x) for x in data[22].split(',')] 
	    msg.state22 = state_elements_22
	    state_elements_23 = [float(x) for x in data[23].split(',')] 
	    msg.state23 = state_elements_23
	    state_elements_24 = [float(x) for x in data[24].split(',')] 
	    msg.state24 = state_elements_24
	    state_elements_25 = [float(x) for x in data[25].split(',')] 
	    msg.state25 = state_elements_25
	    state_elements_26 = [float(x) for x in data[26].split(',')] 
	    msg.state26 = state_elements_26
	    state_elements_27 = [float(x) for x in data[27].split(',')] 
	    msg.state27 = state_elements_27
	    state_elements_28 = [float(x) for x in data[28].split(',')] 
	    msg.state28 = state_elements_28
	    state_elements_29 = [float(x) for x in data[29].split(',')] 
	    msg.state29 = state_elements_29
	    state_elements_30 = [float(x) for x in data[30].split(',')] 
	    msg.state30 = state_elements_30
	    state_elements_31 = [float(x) for x in data[31].split(',')] 
	    msg.state31 = state_elements_31
	    state_elements_32 = [float(x) for x in data[32].split(',')] 
	    msg.state32 = state_elements_32
	    state_elements_33 = [float(x) for x in data[33].split(',')] 
	    msg.state33 = state_elements_33
	    state_elements_34 = [float(x) for x in data[34].split(',')] 
	    msg.state34 = state_elements_34
	    state_elements_35 = [float(x) for x in data[35].split(',')] 
	    msg.state35 = state_elements_35
	    state_elements_36 = [float(x) for x in data[36].split(',')] 
	    msg.state36 = state_elements_36
	    state_elements_37 = [float(x) for x in data[37].split(',')] 
	    msg.state37 = state_elements_37
	    state_elements_38 = [float(x) for x in data[38].split(',')] 
	    msg.state38 = state_elements_38
	    state_elements_39 = [float(x) for x in data[39].split(',')] 
	    msg.state39 = state_elements_39
	    state_elements_40 = [float(x) for x in data[40].split(',')] 
	    msg.state40 = state_elements_40
	    state_elements_41 = [float(x) for x in data[41].split(',')] 
	    msg.state41 = state_elements_41
	    state_elements_42 = [float(x) for x in data[42].split(',')] 
	    msg.state42 = state_elements_42
	    state_elements_43 = [float(x) for x in data[43].split(',')] 
	    msg.state43 = state_elements_43
	    state_elements_44 = [float(x) for x in data[44].split(',')] 
	    msg.state44 = state_elements_44
	    state_elements_45 = [float(x) for x in data[45].split(',')] 
	    msg.state45 = state_elements_45
	    state_elements_46 = [float(x) for x in data[46].split(',')] 
	    msg.state46 = state_elements_46
	    state_elements_47 = [float(x) for x in data[47].split(',')] 
	    msg.state47 = state_elements_47
	    state_elements_48 = [float(x) for x in data[48].split(',')] 
	    msg.state48 = state_elements_48
	    state_elements_49 = [float(x) for x in data[49].split(',')] 
	    msg.state49 = state_elements_49
	    state_elements_50 = [float(x) for x in data[50].split(',')] 
	    msg.state50 = state_elements_50
	    state_elements_51 = [float(x) for x in data[51].split(',')] 
	    msg.state51 = state_elements_51
	    state_elements_52 = [float(x) for x in data[52].split(',')] 
	    msg.state52 = state_elements_52
	    state_elements_53 = [float(x) for x in data[53].split(',')] 
	    msg.state53 = state_elements_53
	    state_elements_54 = [float(x) for x in data[54].split(',')] 
	    msg.state54 = state_elements_54
	    state_elements_55 = [float(x) for x in data[55].split(',')] 
	    msg.state55 = state_elements_55
	    state_elements_56 = [float(x) for x in data[56].split(',')] 
	    msg.state56 = state_elements_56
	    state_elements_57 = [float(x) for x in data[57].split(',')] 
	    msg.state57 = state_elements_57
	    state_elements_58 = [float(x) for x in data[58].split(',')] 
	    msg.state58 = state_elements_58
	    state_elements_59 = [float(x) for x in data[59].split(',')] 
	    msg.state59 = state_elements_59
	    state_elements_60 = [float(x) for x in data[60].split(',')] 
	    msg.state60 = state_elements_60
	    state_elements_61 = [float(x) for x in data[61].split(',')] 
	    msg.state61 = state_elements_61
	    state_elements_62 = [float(x) for x in data[62].split(',')] 
	    msg.state62 = state_elements_62
	    state_elements_63 = [float(x) for x in data[63].split(',')] 
	    msg.state63 = state_elements_63
	    state_elements_64 = [float(x) for x in data[64].split(',')] 
	    msg.state64 = state_elements_64
	    state_elements_65 = [float(x) for x in data[65].split(',')] 
	    msg.state65 = state_elements_65
	    state_elements_66 = [float(x) for x in data[66].split(',')] 
	    msg.state66 = state_elements_66
	    state_elements_67 = [float(x) for x in data[67].split(',')] 
	    msg.state67 = state_elements_67
	    state_elements_68 = [float(x) for x in data[68].split(',')] 
	    msg.state68 = state_elements_68
	    state_elements_69 = [float(x) for x in data[69].split(',')] 
	    msg.state69 = state_elements_69
	    state_elements_70 = [float(x) for x in data[70].split(',')] 
	    msg.state70 = state_elements_70
	    state_elements_71 = [float(x) for x in data[71].split(',')] 
	    msg.state71 = state_elements_71
	    state_elements_72 = [float(x) for x in data[72].split(',')] 
	    msg.state72 = state_elements_72
	    state_elements_73 = [float(x) for x in data[73].split(',')] 
	    msg.state73 = state_elements_73
	    state_elements_74 = [float(x) for x in data[74].split(',')] 
	    msg.state74 = state_elements_74
	    state_elements_75 = [float(x) for x in data[75].split(',')] 
	    msg.state75 = state_elements_75
	    state_elements_76 = [float(x) for x in data[76].split(',')] 
	    msg.state76 = state_elements_76
	    state_elements_77 = [float(x) for x in data[77].split(',')] 
	    msg.state77 = state_elements_77
	    state_elements_78 = [float(x) for x in data[78].split(',')] 
	    msg.state78 = state_elements_78
	    state_elements_79 = [float(x) for x in data[79].split(',')] 
	    msg.state79 = state_elements_79
	    state_elements_80 = [float(x) for x in data[80].split(',')] 
	    msg.state80 = state_elements_80
	    state_elements_81 = [float(x) for x in data[81].split(',')] 
	    msg.state81 = state_elements_81
	    state_elements_82 = [float(x) for x in data[82].split(',')] 
	    msg.state82 = state_elements_82
	    state_elements_83 = [float(x) for x in data[83].split(',')] 
	    msg.state83 = state_elements_83
	    state_elements_84 = [float(x) for x in data[84].split(',')] 
	    msg.state84 = state_elements_84
	    state_elements_85 = [float(x) for x in data[85].split(',')] 
	    msg.state85 = state_elements_85
	    state_elements_86 = [float(x) for x in data[86].split(',')] 
	    msg.state86 = state_elements_86
	    state_elements_87 = [float(x) for x in data[87].split(',')] 
	    msg.state87 = state_elements_87
	    state_elements_88 = [float(x) for x in data[88].split(',')] 
	    msg.state88 = state_elements_88
	    state_elements_89 = [float(x) for x in data[89].split(',')] 
	    msg.state89 = state_elements_89
	    state_elements_90 = [float(x) for x in data[90].split(',')] 
	    msg.state90 = state_elements_90
	    state_elements_91 = [float(x) for x in data[91].split(',')] 
	    msg.state91 = state_elements_91
	    state_elements_92 = [float(x) for x in data[92].split(',')] 
	    msg.state92 = state_elements_92
	    state_elements_93 = [float(x) for x in data[93].split(',')] 
	    msg.state93 = state_elements_93
	    state_elements_94 = [float(x) for x in data[94].split(',')] 
	    msg.state94 = state_elements_94
	    state_elements_95 = [float(x) for x in data[95].split(',')] 
	    msg.state95 = state_elements_95
	    state_elements_96 = [float(x) for x in data[96].split(',')] 
	    msg.state96 = state_elements_96
	    state_elements_97 = [float(x) for x in data[97].split(',')] 
	    msg.state97 = state_elements_97
	    state_elements_98 = [float(x) for x in data[98].split(',')] 
	    msg.state98 = state_elements_98
	    state_elements_99 = [float(x) for x in data[99].split(',')] 
	    msg.state99 = state_elements_99
	    state_elements_100 = [float(x) for x in data[100].split(',')] 
	    msg.state100 = state_elements_100
	    state_elements_101 = [float(x) for x in data[101].split(',')] 
	    msg.state101 = state_elements_101
	    state_elements_102 = [float(x) for x in data[102].split(',')] 
	    msg.state102 = state_elements_102
	    state_elements_103 = [float(x) for x in data[103].split(',')] 
	    msg.state103 = state_elements_103
	    state_elements_104 = [float(x) for x in data[104].split(',')] 
	    msg.state104 = state_elements_104
	    state_elements_105 = [float(x) for x in data[105].split(',')] 
	    msg.state105 = state_elements_105
	    state_elements_106 = [float(x) for x in data[106].split(',')] 
	    msg.state106 = state_elements_106
	    state_elements_107 = [float(x) for x in data[107].split(',')] 
	    msg.state107 = state_elements_107
	    state_elements_108 = [float(x) for x in data[108].split(',')] 
	    msg.state108 = state_elements_108
	    state_elements_109 = [float(x) for x in data[109].split(',')] 
	    msg.state109 = state_elements_109
	    state_elements_110 = [float(x) for x in data[110].split(',')] 
	    msg.state110 = state_elements_110
	    state_elements_111 = [float(x) for x in data[111].split(',')] 
	    msg.state111 = state_elements_111
	    state_elements_112 = [float(x) for x in data[112].split(',')] 
	    msg.state112 = state_elements_112
	    state_elements_113 = [float(x) for x in data[113].split(',')] 
	    msg.state113 = state_elements_113
	    state_elements_114 = [float(x) for x in data[114].split(',')] 
	    msg.state114 = state_elements_114
	    state_elements_115 = [float(x) for x in data[115].split(',')] 
	    msg.state115 = state_elements_115
	    state_elements_116 = [float(x) for x in data[116].split(',')] 
	    msg.state116 = state_elements_116
	    state_elements_117 = [float(x) for x in data[117].split(',')] 
	    msg.state117 = state_elements_117
	    state_elements_118 = [float(x) for x in data[118].split(',')] 
	    msg.state118 = state_elements_118
	    state_elements_119 = [float(x) for x in data[119].split(',')] 
	    msg.state119 = state_elements_119
            pub.publish(msg)
            rate.sleep()
    print msg
#	for j, val in enumerate(state_elements):
#	    msg[i][j] = val

 #   print msg




