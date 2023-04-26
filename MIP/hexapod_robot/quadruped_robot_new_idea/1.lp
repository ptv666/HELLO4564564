\ Model matlab
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 COM_state_3 + 0 COM_state_4 + 0 T_1 + 0 T_2 + 0 T_3 + 0 T_4
   + 0 Leg_state_4 + 0 Leg_state_8 + 0 Leg_state_12 + 0 Leg_state_16
Subject To
 T_self_con_1: T_5 + T_6 <= 1
 T_self_con_2: T_7 + T_8 <= 1
 T_self_con_3: T_5 + T_8 <= 1
 T_self_con_4: T_6 + T_7 <= 1
 T_self_con_5: T_9 + T_10 <= 1
 T_self_con_6: T_11 + T_12 <= 1
 T_self_con_7: T_9 + T_12 <= 1
 T_self_con_8: T_10 + T_11 <= 1
 T_self_con_9: T_13 + T_14 <= 1
 T_self_con_10: T_15 + T_16 <= 1
 T_self_con_11: T_13 + T_16 <= 1
 T_self_con_12: T_14 + T_15 <= 1
 T_self_con_13: T_17 + T_18 <= 1
 T_self_con_14: T_19 + T_20 <= 1
 T_self_con_15: T_17 + T_20 <= 1
 T_self_con_16: T_18 + T_19 <= 1
 T_self_con_17: T_21 + T_22 <= 1
 T_self_con_18: T_23 + T_24 <= 1
 T_self_con_19: T_21 + T_24 <= 1
 T_self_con_20: T_22 + T_23 <= 1
 T_self_con_21: T_25 + T_26 <= 1
 T_self_con_22: T_27 + T_28 <= 1
 T_self_con_23: T_25 + T_28 <= 1
 T_self_con_24: T_26 + T_27 <= 1
 T_self_con_25: T_29 + T_30 <= 1
 T_self_con_26: T_31 + T_32 <= 1
 T_self_con_27: T_29 + T_32 <= 1
 T_self_con_28: T_30 + T_31 <= 1
 T_self_con_29: T_33 + T_34 <= 1
 T_self_con_30: T_35 + T_36 <= 1
 T_self_con_31: T_33 + T_36 <= 1
 T_self_con_32: T_34 + T_35 <= 1
 T_self_con_33: T_37 + T_38 <= 1
 T_self_con_34: T_39 + T_40 <= 1
 T_self_con_35: T_37 + T_40 <= 1
 T_self_con_36: T_38 + T_39 <= 1
 T_self_con_37: T_41 + T_42 <= 1
 T_self_con_38: T_43 + T_44 <= 1
 T_self_con_39: T_41 + T_44 <= 1
 T_self_con_40: T_42 + T_43 <= 1
 T_self_con_41: T_45 + T_46 <= 1
 T_self_con_42: T_47 + T_48 <= 1
 T_self_con_43: T_45 + T_48 <= 1
 T_self_con_44: T_46 + T_47 <= 1
 T_self_con_45: T_49 + T_50 <= 1
 T_self_con_46: T_51 + T_52 <= 1
 T_self_con_47: T_49 + T_52 <= 1
 T_self_con_48: T_50 + T_51 <= 1
 T(1,2)_AND_Leg_state__x_1: - 999999 T_5 - Leg_state_1 + Leg_state_17 <= 0
 T(1,2)_AND_Leg_state__x_2: - 999999 T_5 + Leg_state_1 - Leg_state_17 <= 0
 T(1,2)_AND_Leg_state__x_3: - COM_state_1 + 999999 T_5 + Leg_state_17
   <= 999999.6739989705
 T(1,2)_AND_Leg_state__x_4: COM_state_1 + 999999 T_5 - Leg_state_17
   <= 999998.8260010295
 T(1,2)_AND_Leg_state__y_1: - 999999 T_5 - Leg_state_2 + Leg_state_18 <= 0
 T(1,2)_AND_Leg_state__y_2: - 999999 T_5 + Leg_state_2 - Leg_state_18 <= 0
 T(1,2)_AND_Leg_state__y_3: - COM_state_2 + 999999 T_5 + Leg_state_18
   <= 999999.6739989705
 T(1,2)_AND_Leg_state__y_4: COM_state_2 + 999999 T_5 - Leg_state_18
   <= 999998.8260010295
 T(1,2)_AND_Leg_state__z_1: - 999999 T_5 - Leg_state_3 + Leg_state_19 <= 0
 T(1,2)_AND_Leg_state__z_2: - 999999 T_5 + Leg_state_3 - Leg_state_19 <= 0
 T(1,2)_AND_Leg_state__z_3: 999999 T_5 - Leg_state_3 + Leg_state_19
   <= 999999.1
 T(1,2)_AND_Leg_state__z_4: 999999 T_5 + Leg_state_3 - Leg_state_19
   <= 999999.1
 T(2,2)_AND_Leg_state__x_1: - 999999 T_6 - Leg_state_5 + Leg_state_21 <= 0
 T(2,2)_AND_Leg_state__x_2: - 999999 T_6 + Leg_state_5 - Leg_state_21 <= 0
 T(2,2)_AND_Leg_state__x_3: - COM_state_1 + 999999 T_6 + Leg_state_21
   <= 999998.8260010295
 T(2,2)_AND_Leg_state__x_4: COM_state_1 + 999999 T_6 - Leg_state_21
   <= 999999.6739989705
 T(2,2)_AND_Leg_state__y_1: - 999999 T_6 - Leg_state_6 + Leg_state_22 <= 0
 T(2,2)_AND_Leg_state__y_2: - 999999 T_6 + Leg_state_6 - Leg_state_22 <= 0
 T(2,2)_AND_Leg_state__y_3: - COM_state_2 + 999999 T_6 + Leg_state_22
   <= 999998.8260010295
 T(2,2)_AND_Leg_state__y_4: COM_state_2 + 999999 T_6 - Leg_state_22
   <= 999999.6739989705
 T(2,2)_AND_Leg_state__z_1: - 999999 T_6 - Leg_state_7 + Leg_state_23 <= 0
 T(2,2)_AND_Leg_state__z_2: - 999999 T_6 + Leg_state_7 - Leg_state_23 <= 0
 T(2,2)_AND_Leg_state__z_3: 999999 T_6 - Leg_state_7 + Leg_state_23
   <= 999999.1
 T(2,2)_AND_Leg_state__z_4: 999999 T_6 + Leg_state_7 - Leg_state_23
   <= 999999.1
 T(3,2)_AND_Leg_state__x_1: - 999999 T_7 - Leg_state_9 + Leg_state_25 <= 0
 T(3,2)_AND_Leg_state__x_2: - 999999 T_7 + Leg_state_9 - Leg_state_25 <= 0
 T(3,2)_AND_Leg_state__x_3: - COM_state_1 + 999999 T_7 + Leg_state_25
   <= 999998.8260010295
 T(3,2)_AND_Leg_state__x_4: COM_state_1 + 999999 T_7 - Leg_state_25
   <= 999999.6739989705
 T(3,2)_AND_Leg_state__y_1: - 999999 T_7 - Leg_state_10 + Leg_state_26
   <= 0
 T(3,2)_AND_Leg_state__y_2: - 999999 T_7 + Leg_state_10 - Leg_state_26
   <= 0
 T(3,2)_AND_Leg_state__y_3: - COM_state_2 + 999999 T_7 + Leg_state_26
   <= 999998.8260010295
 T(3,2)_AND_Leg_state__y_4: COM_state_2 + 999999 T_7 - Leg_state_26
   <= 999999.6739989705
 T(3,2)_AND_Leg_state__z_1: - 999999 T_7 - Leg_state_11 + Leg_state_27
   <= 0
 T(3,2)_AND_Leg_state__z_2: - 999999 T_7 + Leg_state_11 - Leg_state_27
   <= 0
 T(3,2)_AND_Leg_state__z_3: 999999 T_7 - Leg_state_11 + Leg_state_27
   <= 999999.1
 T(3,2)_AND_Leg_state__z_4: 999999 T_7 + Leg_state_11 - Leg_state_27
   <= 999999.1
 T(4,2)_AND_Leg_state__x_1: - 999999 T_8 - Leg_state_13 + Leg_state_29
   <= 0
 T(4,2)_AND_Leg_state__x_2: - 999999 T_8 + Leg_state_13 - Leg_state_29
   <= 0
 T(4,2)_AND_Leg_state__x_3: - COM_state_1 + 999999 T_8 + Leg_state_29
   <= 999999.6739989705
 T(4,2)_AND_Leg_state__x_4: COM_state_1 + 999999 T_8 - Leg_state_29
   <= 999998.8260010295
 T(4,2)_AND_Leg_state__y_1: - 999999 T_8 - Leg_state_14 + Leg_state_30
   <= 0
 T(4,2)_AND_Leg_state__y_2: - 999999 T_8 + Leg_state_14 - Leg_state_30
   <= 0
 T(4,2)_AND_Leg_state__y_3: - COM_state_2 + 999999 T_8 + Leg_state_30
   <= 999999.6739989705
 T(4,2)_AND_Leg_state__y_4: COM_state_2 + 999999 T_8 - Leg_state_30
   <= 999998.8260010295
 T(4,2)_AND_Leg_state__z_1: - 999999 T_8 - Leg_state_15 + Leg_state_31
   <= 0
 T(4,2)_AND_Leg_state__z_2: - 999999 T_8 + Leg_state_15 - Leg_state_31
   <= 0
 T(4,2)_AND_Leg_state__z_3: 999999 T_8 - Leg_state_15 + Leg_state_31
   <= 999999.1
 T(4,2)_AND_Leg_state__z_4: 999999 T_8 + Leg_state_15 - Leg_state_31
   <= 999999.1
 T(1,3)_AND_Leg_state__x_1: - 999999 T_9 - Leg_state_17 + Leg_state_33
   <= 0
 T(1,3)_AND_Leg_state__x_2: - 999999 T_9 + Leg_state_17 - Leg_state_33
   <= 0
 T(1,3)_AND_Leg_state__x_3: - COM_state_5 + 999999 T_9 + Leg_state_33
   <= 999999.6739989705
 T(1,3)_AND_Leg_state__x_4: COM_state_5 + 999999 T_9 - Leg_state_33
   <= 999998.8260010295
 T(1,3)_AND_Leg_state__y_1: - 999999 T_9 - Leg_state_18 + Leg_state_34
   <= 0
 T(1,3)_AND_Leg_state__y_2: - 999999 T_9 + Leg_state_18 - Leg_state_34
   <= 0
 T(1,3)_AND_Leg_state__y_3: - COM_state_6 + 999999 T_9 + Leg_state_34
   <= 999999.6739989705
 T(1,3)_AND_Leg_state__y_4: COM_state_6 + 999999 T_9 - Leg_state_34
   <= 999998.8260010295
 T(1,3)_AND_Leg_state__z_1: - 999999 T_9 - Leg_state_19 + Leg_state_35
   <= 0
 T(1,3)_AND_Leg_state__z_2: - 999999 T_9 + Leg_state_19 - Leg_state_35
   <= 0
 T(1,3)_AND_Leg_state__z_3: 999999 T_9 - Leg_state_19 + Leg_state_35
   <= 999999.1
 T(1,3)_AND_Leg_state__z_4: 999999 T_9 + Leg_state_19 - Leg_state_35
   <= 999999.1
 T(2,3)_AND_Leg_state__x_1: - 999999 T_10 - Leg_state_21 + Leg_state_37
   <= 0
 T(2,3)_AND_Leg_state__x_2: - 999999 T_10 + Leg_state_21 - Leg_state_37
   <= 0
 T(2,3)_AND_Leg_state__x_3: - COM_state_5 + 999999 T_10 + Leg_state_37
   <= 999998.8260010295
 T(2,3)_AND_Leg_state__x_4: COM_state_5 + 999999 T_10 - Leg_state_37
   <= 999999.6739989705
 T(2,3)_AND_Leg_state__y_1: - 999999 T_10 - Leg_state_22 + Leg_state_38
   <= 0
 T(2,3)_AND_Leg_state__y_2: - 999999 T_10 + Leg_state_22 - Leg_state_38
   <= 0
 T(2,3)_AND_Leg_state__y_3: - COM_state_6 + 999999 T_10 + Leg_state_38
   <= 999998.8260010295
 T(2,3)_AND_Leg_state__y_4: COM_state_6 + 999999 T_10 - Leg_state_38
   <= 999999.6739989705
 T(2,3)_AND_Leg_state__z_1: - 999999 T_10 - Leg_state_23 + Leg_state_39
   <= 0
 T(2,3)_AND_Leg_state__z_2: - 999999 T_10 + Leg_state_23 - Leg_state_39
   <= 0
 T(2,3)_AND_Leg_state__z_3: 999999 T_10 - Leg_state_23 + Leg_state_39
   <= 999999.1
 T(2,3)_AND_Leg_state__z_4: 999999 T_10 + Leg_state_23 - Leg_state_39
   <= 999999.1
 T(3,3)_AND_Leg_state__x_1: - 999999 T_11 - Leg_state_25 + Leg_state_41
   <= 0
 T(3,3)_AND_Leg_state__x_2: - 999999 T_11 + Leg_state_25 - Leg_state_41
   <= 0
 T(3,3)_AND_Leg_state__x_3: - COM_state_5 + 999999 T_11 + Leg_state_41
   <= 999998.8260010295
 T(3,3)_AND_Leg_state__x_4: COM_state_5 + 999999 T_11 - Leg_state_41
   <= 999999.6739989705
 T(3,3)_AND_Leg_state__y_1: - 999999 T_11 - Leg_state_26 + Leg_state_42
   <= 0
 T(3,3)_AND_Leg_state__y_2: - 999999 T_11 + Leg_state_26 - Leg_state_42
   <= 0
 T(3,3)_AND_Leg_state__y_3: - COM_state_6 + 999999 T_11 + Leg_state_42
   <= 999998.8260010295
 T(3,3)_AND_Leg_state__y_4: COM_state_6 + 999999 T_11 - Leg_state_42
   <= 999999.6739989705
 T(3,3)_AND_Leg_state__z_1: - 999999 T_11 - Leg_state_27 + Leg_state_43
   <= 0
 T(3,3)_AND_Leg_state__z_2: - 999999 T_11 + Leg_state_27 - Leg_state_43
   <= 0
 T(3,3)_AND_Leg_state__z_3: 999999 T_11 - Leg_state_27 + Leg_state_43
   <= 999999.1
 T(3,3)_AND_Leg_state__z_4: 999999 T_11 + Leg_state_27 - Leg_state_43
   <= 999999.1
 T(4,3)_AND_Leg_state__x_1: - 999999 T_12 - Leg_state_29 + Leg_state_45
   <= 0
 T(4,3)_AND_Leg_state__x_2: - 999999 T_12 + Leg_state_29 - Leg_state_45
   <= 0
 T(4,3)_AND_Leg_state__x_3: - COM_state_5 + 999999 T_12 + Leg_state_45
   <= 999999.6739989705
 T(4,3)_AND_Leg_state__x_4: COM_state_5 + 999999 T_12 - Leg_state_45
   <= 999998.8260010295
 T(4,3)_AND_Leg_state__y_1: - 999999 T_12 - Leg_state_30 + Leg_state_46
   <= 0
 T(4,3)_AND_Leg_state__y_2: - 999999 T_12 + Leg_state_30 - Leg_state_46
   <= 0
 T(4,3)_AND_Leg_state__y_3: - COM_state_6 + 999999 T_12 + Leg_state_46
   <= 999999.6739989705
 T(4,3)_AND_Leg_state__y_4: COM_state_6 + 999999 T_12 - Leg_state_46
   <= 999998.8260010295
 T(4,3)_AND_Leg_state__z_1: - 999999 T_12 - Leg_state_31 + Leg_state_47
   <= 0
 T(4,3)_AND_Leg_state__z_2: - 999999 T_12 + Leg_state_31 - Leg_state_47
   <= 0
 T(4,3)_AND_Leg_state__z_3: 999999 T_12 - Leg_state_31 + Leg_state_47
   <= 999999.1
 T(4,3)_AND_Leg_state__z_4: 999999 T_12 + Leg_state_31 - Leg_state_47
   <= 999999.1
 T(1,4)_AND_Leg_state__x_1: - 999999 T_13 - Leg_state_33 + Leg_state_49
   <= 0
 T(1,4)_AND_Leg_state__x_2: - 999999 T_13 + Leg_state_33 - Leg_state_49
   <= 0
 T(1,4)_AND_Leg_state__x_3: - COM_state_9 + 999999 T_13 + Leg_state_49
   <= 999999.6739989705
 T(1,4)_AND_Leg_state__x_4: COM_state_9 + 999999 T_13 - Leg_state_49
   <= 999998.8260010295
 T(1,4)_AND_Leg_state__y_1: - 999999 T_13 - Leg_state_34 + Leg_state_50
   <= 0
 T(1,4)_AND_Leg_state__y_2: - 999999 T_13 + Leg_state_34 - Leg_state_50
   <= 0
 T(1,4)_AND_Leg_state__y_3: - COM_state_10 + 999999 T_13 + Leg_state_50
   <= 999999.6739989705
 T(1,4)_AND_Leg_state__y_4: COM_state_10 + 999999 T_13 - Leg_state_50
   <= 999998.8260010295
 T(1,4)_AND_Leg_state__z_1: - 999999 T_13 - Leg_state_35 + Leg_state_51
   <= 0
 T(1,4)_AND_Leg_state__z_2: - 999999 T_13 + Leg_state_35 - Leg_state_51
   <= 0
 T(1,4)_AND_Leg_state__z_3: 999999 T_13 - Leg_state_35 + Leg_state_51
   <= 999999.1
 T(1,4)_AND_Leg_state__z_4: 999999 T_13 + Leg_state_35 - Leg_state_51
   <= 999999.1
 T(2,4)_AND_Leg_state__x_1: - 999999 T_14 - Leg_state_37 + Leg_state_53
   <= 0
 T(2,4)_AND_Leg_state__x_2: - 999999 T_14 + Leg_state_37 - Leg_state_53
   <= 0
 T(2,4)_AND_Leg_state__x_3: - COM_state_9 + 999999 T_14 + Leg_state_53
   <= 999998.8260010295
 T(2,4)_AND_Leg_state__x_4: COM_state_9 + 999999 T_14 - Leg_state_53
   <= 999999.6739989705
 T(2,4)_AND_Leg_state__y_1: - 999999 T_14 - Leg_state_38 + Leg_state_54
   <= 0
 T(2,4)_AND_Leg_state__y_2: - 999999 T_14 + Leg_state_38 - Leg_state_54
   <= 0
 T(2,4)_AND_Leg_state__y_3: - COM_state_10 + 999999 T_14 + Leg_state_54
   <= 999998.8260010295
 T(2,4)_AND_Leg_state__y_4: COM_state_10 + 999999 T_14 - Leg_state_54
   <= 999999.6739989705
 T(2,4)_AND_Leg_state__z_1: - 999999 T_14 - Leg_state_39 + Leg_state_55
   <= 0
 T(2,4)_AND_Leg_state__z_2: - 999999 T_14 + Leg_state_39 - Leg_state_55
   <= 0
 T(2,4)_AND_Leg_state__z_3: 999999 T_14 - Leg_state_39 + Leg_state_55
   <= 999999.1
 T(2,4)_AND_Leg_state__z_4: 999999 T_14 + Leg_state_39 - Leg_state_55
   <= 999999.1
 T(3,4)_AND_Leg_state__x_1: - 999999 T_15 - Leg_state_41 + Leg_state_57
   <= 0
 T(3,4)_AND_Leg_state__x_2: - 999999 T_15 + Leg_state_41 - Leg_state_57
   <= 0
 T(3,4)_AND_Leg_state__x_3: - COM_state_9 + 999999 T_15 + Leg_state_57
   <= 999998.8260010295
 T(3,4)_AND_Leg_state__x_4: COM_state_9 + 999999 T_15 - Leg_state_57
   <= 999999.6739989705
 T(3,4)_AND_Leg_state__y_1: - 999999 T_15 - Leg_state_42 + Leg_state_58
   <= 0
 T(3,4)_AND_Leg_state__y_2: - 999999 T_15 + Leg_state_42 - Leg_state_58
   <= 0
 T(3,4)_AND_Leg_state__y_3: - COM_state_10 + 999999 T_15 + Leg_state_58
   <= 999998.8260010295
 T(3,4)_AND_Leg_state__y_4: COM_state_10 + 999999 T_15 - Leg_state_58
   <= 999999.6739989705
 T(3,4)_AND_Leg_state__z_1: - 999999 T_15 - Leg_state_43 + Leg_state_59
   <= 0
 T(3,4)_AND_Leg_state__z_2: - 999999 T_15 + Leg_state_43 - Leg_state_59
   <= 0
 T(3,4)_AND_Leg_state__z_3: 999999 T_15 - Leg_state_43 + Leg_state_59
   <= 999999.1
 T(3,4)_AND_Leg_state__z_4: 999999 T_15 + Leg_state_43 - Leg_state_59
   <= 999999.1
 T(4,4)_AND_Leg_state__x_1: - 999999 T_16 - Leg_state_45 + Leg_state_61
   <= 0
 T(4,4)_AND_Leg_state__x_2: - 999999 T_16 + Leg_state_45 - Leg_state_61
   <= 0
 T(4,4)_AND_Leg_state__x_3: - COM_state_9 + 999999 T_16 + Leg_state_61
   <= 999999.6739989705
 T(4,4)_AND_Leg_state__x_4: COM_state_9 + 999999 T_16 - Leg_state_61
   <= 999998.8260010295
 T(4,4)_AND_Leg_state__y_1: - 999999 T_16 - Leg_state_46 + Leg_state_62
   <= 0
 T(4,4)_AND_Leg_state__y_2: - 999999 T_16 + Leg_state_46 - Leg_state_62
   <= 0
 T(4,4)_AND_Leg_state__y_3: - COM_state_10 + 999999 T_16 + Leg_state_62
   <= 999999.6739989705
 T(4,4)_AND_Leg_state__y_4: COM_state_10 + 999999 T_16 - Leg_state_62
   <= 999998.8260010295
 T(4,4)_AND_Leg_state__z_1: - 999999 T_16 - Leg_state_47 + Leg_state_63
   <= 0
 T(4,4)_AND_Leg_state__z_2: - 999999 T_16 + Leg_state_47 - Leg_state_63
   <= 0
 T(4,4)_AND_Leg_state__z_3: 999999 T_16 - Leg_state_47 + Leg_state_63
   <= 999999.1
 T(4,4)_AND_Leg_state__z_4: 999999 T_16 + Leg_state_47 - Leg_state_63
   <= 999999.1
 T(1,5)_AND_Leg_state__x_1: - 999999 T_17 - Leg_state_49 + Leg_state_65
   <= 0
 T(1,5)_AND_Leg_state__x_2: - 999999 T_17 + Leg_state_49 - Leg_state_65
   <= 0
 T(1,5)_AND_Leg_state__x_3: - COM_state_13 + 999999 T_17 + Leg_state_65
   <= 999999.6739989705
 T(1,5)_AND_Leg_state__x_4: COM_state_13 + 999999 T_17 - Leg_state_65
   <= 999998.8260010295
 T(1,5)_AND_Leg_state__y_1: - 999999 T_17 - Leg_state_50 + Leg_state_66
   <= 0
 T(1,5)_AND_Leg_state__y_2: - 999999 T_17 + Leg_state_50 - Leg_state_66
   <= 0
 T(1,5)_AND_Leg_state__y_3: - COM_state_14 + 999999 T_17 + Leg_state_66
   <= 999999.6739989705
 T(1,5)_AND_Leg_state__y_4: COM_state_14 + 999999 T_17 - Leg_state_66
   <= 999998.8260010295
 T(1,5)_AND_Leg_state__z_1: - 999999 T_17 - Leg_state_51 + Leg_state_67
   <= 0
 T(1,5)_AND_Leg_state__z_2: - 999999 T_17 + Leg_state_51 - Leg_state_67
   <= 0
 T(1,5)_AND_Leg_state__z_3: 999999 T_17 - Leg_state_51 + Leg_state_67
   <= 999999.1
 T(1,5)_AND_Leg_state__z_4: 999999 T_17 + Leg_state_51 - Leg_state_67
   <= 999999.1
 T(2,5)_AND_Leg_state__x_1: - 999999 T_18 - Leg_state_53 + Leg_state_69
   <= 0
 T(2,5)_AND_Leg_state__x_2: - 999999 T_18 + Leg_state_53 - Leg_state_69
   <= 0
 T(2,5)_AND_Leg_state__x_3: - COM_state_13 + 999999 T_18 + Leg_state_69
   <= 999998.8260010295
 T(2,5)_AND_Leg_state__x_4: COM_state_13 + 999999 T_18 - Leg_state_69
   <= 999999.6739989705
 T(2,5)_AND_Leg_state__y_1: - 999999 T_18 - Leg_state_54 + Leg_state_70
   <= 0
 T(2,5)_AND_Leg_state__y_2: - 999999 T_18 + Leg_state_54 - Leg_state_70
   <= 0
 T(2,5)_AND_Leg_state__y_3: - COM_state_14 + 999999 T_18 + Leg_state_70
   <= 999998.8260010295
 T(2,5)_AND_Leg_state__y_4: COM_state_14 + 999999 T_18 - Leg_state_70
   <= 999999.6739989705
 T(2,5)_AND_Leg_state__z_1: - 999999 T_18 - Leg_state_55 + Leg_state_71
   <= 0
 T(2,5)_AND_Leg_state__z_2: - 999999 T_18 + Leg_state_55 - Leg_state_71
   <= 0
 T(2,5)_AND_Leg_state__z_3: 999999 T_18 - Leg_state_55 + Leg_state_71
   <= 999999.1
 T(2,5)_AND_Leg_state__z_4: 999999 T_18 + Leg_state_55 - Leg_state_71
   <= 999999.1
 T(3,5)_AND_Leg_state__x_1: - 999999 T_19 - Leg_state_57 + Leg_state_73
   <= 0
 T(3,5)_AND_Leg_state__x_2: - 999999 T_19 + Leg_state_57 - Leg_state_73
   <= 0
 T(3,5)_AND_Leg_state__x_3: - COM_state_13 + 999999 T_19 + Leg_state_73
   <= 999998.8260010295
 T(3,5)_AND_Leg_state__x_4: COM_state_13 + 999999 T_19 - Leg_state_73
   <= 999999.6739989705
 T(3,5)_AND_Leg_state__y_1: - 999999 T_19 - Leg_state_58 + Leg_state_74
   <= 0
 T(3,5)_AND_Leg_state__y_2: - 999999 T_19 + Leg_state_58 - Leg_state_74
   <= 0
 T(3,5)_AND_Leg_state__y_3: - COM_state_14 + 999999 T_19 + Leg_state_74
   <= 999998.8260010295
 T(3,5)_AND_Leg_state__y_4: COM_state_14 + 999999 T_19 - Leg_state_74
   <= 999999.6739989705
 T(3,5)_AND_Leg_state__z_1: - 999999 T_19 - Leg_state_59 + Leg_state_75
   <= 0
 T(3,5)_AND_Leg_state__z_2: - 999999 T_19 + Leg_state_59 - Leg_state_75
   <= 0
 T(3,5)_AND_Leg_state__z_3: 999999 T_19 - Leg_state_59 + Leg_state_75
   <= 999999.1
 T(3,5)_AND_Leg_state__z_4: 999999 T_19 + Leg_state_59 - Leg_state_75
   <= 999999.1
 T(4,5)_AND_Leg_state__x_1: - 999999 T_20 - Leg_state_61 + Leg_state_77
   <= 0
 T(4,5)_AND_Leg_state__x_2: - 999999 T_20 + Leg_state_61 - Leg_state_77
   <= 0
 T(4,5)_AND_Leg_state__x_3: - COM_state_13 + 999999 T_20 + Leg_state_77
   <= 999999.6739989705
 T(4,5)_AND_Leg_state__x_4: COM_state_13 + 999999 T_20 - Leg_state_77
   <= 999998.8260010295
 T(4,5)_AND_Leg_state__y_1: - 999999 T_20 - Leg_state_62 + Leg_state_78
   <= 0
 T(4,5)_AND_Leg_state__y_2: - 999999 T_20 + Leg_state_62 - Leg_state_78
   <= 0
 T(4,5)_AND_Leg_state__y_3: - COM_state_14 + 999999 T_20 + Leg_state_78
   <= 999999.6739989705
 T(4,5)_AND_Leg_state__y_4: COM_state_14 + 999999 T_20 - Leg_state_78
   <= 999998.8260010295
 T(4,5)_AND_Leg_state__z_1: - 999999 T_20 - Leg_state_63 + Leg_state_79
   <= 0
 T(4,5)_AND_Leg_state__z_2: - 999999 T_20 + Leg_state_63 - Leg_state_79
   <= 0
 T(4,5)_AND_Leg_state__z_3: 999999 T_20 - Leg_state_63 + Leg_state_79
   <= 999999.1
 T(4,5)_AND_Leg_state__z_4: 999999 T_20 + Leg_state_63 - Leg_state_79
   <= 999999.1
 T(1,6)_AND_Leg_state__x_1: - 999999 T_21 - Leg_state_65 + Leg_state_81
   <= 0
 T(1,6)_AND_Leg_state__x_2: - 999999 T_21 + Leg_state_65 - Leg_state_81
   <= 0
 T(1,6)_AND_Leg_state__x_3: - COM_state_17 + 999999 T_21 + Leg_state_81
   <= 999999.6739989705
 T(1,6)_AND_Leg_state__x_4: COM_state_17 + 999999 T_21 - Leg_state_81
   <= 999998.8260010295
 T(1,6)_AND_Leg_state__y_1: - 999999 T_21 - Leg_state_66 + Leg_state_82
   <= 0
 T(1,6)_AND_Leg_state__y_2: - 999999 T_21 + Leg_state_66 - Leg_state_82
   <= 0
 T(1,6)_AND_Leg_state__y_3: - COM_state_18 + 999999 T_21 + Leg_state_82
   <= 999999.6739989705
 T(1,6)_AND_Leg_state__y_4: COM_state_18 + 999999 T_21 - Leg_state_82
   <= 999998.8260010295
 T(1,6)_AND_Leg_state__z_1: - 999999 T_21 - Leg_state_67 + Leg_state_83
   <= 0
 T(1,6)_AND_Leg_state__z_2: - 999999 T_21 + Leg_state_67 - Leg_state_83
   <= 0
 T(1,6)_AND_Leg_state__z_3: 999999 T_21 - Leg_state_67 + Leg_state_83
   <= 999999.1
 T(1,6)_AND_Leg_state__z_4: 999999 T_21 + Leg_state_67 - Leg_state_83
   <= 999999.1
 T(2,6)_AND_Leg_state__x_1: - 999999 T_22 - Leg_state_69 + Leg_state_85
   <= 0
 T(2,6)_AND_Leg_state__x_2: - 999999 T_22 + Leg_state_69 - Leg_state_85
   <= 0
 T(2,6)_AND_Leg_state__x_3: - COM_state_17 + 999999 T_22 + Leg_state_85
   <= 999998.8260010295
 T(2,6)_AND_Leg_state__x_4: COM_state_17 + 999999 T_22 - Leg_state_85
   <= 999999.6739989705
 T(2,6)_AND_Leg_state__y_1: - 999999 T_22 - Leg_state_70 + Leg_state_86
   <= 0
 T(2,6)_AND_Leg_state__y_2: - 999999 T_22 + Leg_state_70 - Leg_state_86
   <= 0
 T(2,6)_AND_Leg_state__y_3: - COM_state_18 + 999999 T_22 + Leg_state_86
   <= 999998.8260010295
 T(2,6)_AND_Leg_state__y_4: COM_state_18 + 999999 T_22 - Leg_state_86
   <= 999999.6739989705
 T(2,6)_AND_Leg_state__z_1: - 999999 T_22 - Leg_state_71 + Leg_state_87
   <= 0
 T(2,6)_AND_Leg_state__z_2: - 999999 T_22 + Leg_state_71 - Leg_state_87
   <= 0
 T(2,6)_AND_Leg_state__z_3: 999999 T_22 - Leg_state_71 + Leg_state_87
   <= 999999.1
 T(2,6)_AND_Leg_state__z_4: 999999 T_22 + Leg_state_71 - Leg_state_87
   <= 999999.1
 T(3,6)_AND_Leg_state__x_1: - 999999 T_23 - Leg_state_73 + Leg_state_89
   <= 0
 T(3,6)_AND_Leg_state__x_2: - 999999 T_23 + Leg_state_73 - Leg_state_89
   <= 0
 T(3,6)_AND_Leg_state__x_3: - COM_state_17 + 999999 T_23 + Leg_state_89
   <= 999998.8260010295
 T(3,6)_AND_Leg_state__x_4: COM_state_17 + 999999 T_23 - Leg_state_89
   <= 999999.6739989705
 T(3,6)_AND_Leg_state__y_1: - 999999 T_23 - Leg_state_74 + Leg_state_90
   <= 0
 T(3,6)_AND_Leg_state__y_2: - 999999 T_23 + Leg_state_74 - Leg_state_90
   <= 0
 T(3,6)_AND_Leg_state__y_3: - COM_state_18 + 999999 T_23 + Leg_state_90
   <= 999998.8260010295
 T(3,6)_AND_Leg_state__y_4: COM_state_18 + 999999 T_23 - Leg_state_90
   <= 999999.6739989705
 T(3,6)_AND_Leg_state__z_1: - 999999 T_23 - Leg_state_75 + Leg_state_91
   <= 0
 T(3,6)_AND_Leg_state__z_2: - 999999 T_23 + Leg_state_75 - Leg_state_91
   <= 0
 T(3,6)_AND_Leg_state__z_3: 999999 T_23 - Leg_state_75 + Leg_state_91
   <= 999999.1
 T(3,6)_AND_Leg_state__z_4: 999999 T_23 + Leg_state_75 - Leg_state_91
   <= 999999.1
 T(4,6)_AND_Leg_state__x_1: - 999999 T_24 - Leg_state_77 + Leg_state_93
   <= 0
 T(4,6)_AND_Leg_state__x_2: - 999999 T_24 + Leg_state_77 - Leg_state_93
   <= 0
 T(4,6)_AND_Leg_state__x_3: - COM_state_17 + 999999 T_24 + Leg_state_93
   <= 999999.6739989705
 T(4,6)_AND_Leg_state__x_4: COM_state_17 + 999999 T_24 - Leg_state_93
   <= 999998.8260010295
 T(4,6)_AND_Leg_state__y_1: - 999999 T_24 - Leg_state_78 + Leg_state_94
   <= 0
 T(4,6)_AND_Leg_state__y_2: - 999999 T_24 + Leg_state_78 - Leg_state_94
   <= 0
 T(4,6)_AND_Leg_state__y_3: - COM_state_18 + 999999 T_24 + Leg_state_94
   <= 999999.6739989705
 T(4,6)_AND_Leg_state__y_4: COM_state_18 + 999999 T_24 - Leg_state_94
   <= 999998.8260010295
 T(4,6)_AND_Leg_state__z_1: - 999999 T_24 - Leg_state_79 + Leg_state_95
   <= 0
 T(4,6)_AND_Leg_state__z_2: - 999999 T_24 + Leg_state_79 - Leg_state_95
   <= 0
 T(4,6)_AND_Leg_state__z_3: 999999 T_24 - Leg_state_79 + Leg_state_95
   <= 999999.1
 T(4,6)_AND_Leg_state__z_4: 999999 T_24 + Leg_state_79 - Leg_state_95
   <= 999999.1
 T(1,7)_AND_Leg_state__x_1: - 999999 T_25 - Leg_state_81 + Leg_state_97
   <= 0
 T(1,7)_AND_Leg_state__x_2: - 999999 T_25 + Leg_state_81 - Leg_state_97
   <= 0
 T(1,7)_AND_Leg_state__x_3: - COM_state_21 + 999999 T_25 + Leg_state_97
   <= 999999.6739989705
 T(1,7)_AND_Leg_state__x_4: COM_state_21 + 999999 T_25 - Leg_state_97
   <= 999998.8260010295
 T(1,7)_AND_Leg_state__y_1: - 999999 T_25 - Leg_state_82 + Leg_state_98
   <= 0
 T(1,7)_AND_Leg_state__y_2: - 999999 T_25 + Leg_state_82 - Leg_state_98
   <= 0
 T(1,7)_AND_Leg_state__y_3: - COM_state_22 + 999999 T_25 + Leg_state_98
   <= 999999.6739989705
 T(1,7)_AND_Leg_state__y_4: COM_state_22 + 999999 T_25 - Leg_state_98
   <= 999998.8260010295
 T(1,7)_AND_Leg_state__z_1: - 999999 T_25 - Leg_state_83 + Leg_state_99
   <= 0
 T(1,7)_AND_Leg_state__z_2: - 999999 T_25 + Leg_state_83 - Leg_state_99
   <= 0
 T(1,7)_AND_Leg_state__z_3: 999999 T_25 - Leg_state_83 + Leg_state_99
   <= 999999.1
 T(1,7)_AND_Leg_state__z_4: 999999 T_25 + Leg_state_83 - Leg_state_99
   <= 999999.1
 T(2,7)_AND_Leg_state__x_1: - 999999 T_26 - Leg_state_85 + Leg_state_101
   <= 0
 T(2,7)_AND_Leg_state__x_2: - 999999 T_26 + Leg_state_85 - Leg_state_101
   <= 0
 T(2,7)_AND_Leg_state__x_3: - COM_state_21 + 999999 T_26 + Leg_state_101
   <= 999998.8260010295
 T(2,7)_AND_Leg_state__x_4: COM_state_21 + 999999 T_26 - Leg_state_101
   <= 999999.6739989705
 T(2,7)_AND_Leg_state__y_1: - 999999 T_26 - Leg_state_86 + Leg_state_102
   <= 0
 T(2,7)_AND_Leg_state__y_2: - 999999 T_26 + Leg_state_86 - Leg_state_102
   <= 0
 T(2,7)_AND_Leg_state__y_3: - COM_state_22 + 999999 T_26 + Leg_state_102
   <= 999998.8260010295
 T(2,7)_AND_Leg_state__y_4: COM_state_22 + 999999 T_26 - Leg_state_102
   <= 999999.6739989705
 T(2,7)_AND_Leg_state__z_1: - 999999 T_26 - Leg_state_87 + Leg_state_103
   <= 0
 T(2,7)_AND_Leg_state__z_2: - 999999 T_26 + Leg_state_87 - Leg_state_103
   <= 0
 T(2,7)_AND_Leg_state__z_3: 999999 T_26 - Leg_state_87 + Leg_state_103
   <= 999999.1
 T(2,7)_AND_Leg_state__z_4: 999999 T_26 + Leg_state_87 - Leg_state_103
   <= 999999.1
 T(3,7)_AND_Leg_state__x_1: - 999999 T_27 - Leg_state_89 + Leg_state_105
   <= 0
 T(3,7)_AND_Leg_state__x_2: - 999999 T_27 + Leg_state_89 - Leg_state_105
   <= 0
 T(3,7)_AND_Leg_state__x_3: - COM_state_21 + 999999 T_27 + Leg_state_105
   <= 999998.8260010295
 T(3,7)_AND_Leg_state__x_4: COM_state_21 + 999999 T_27 - Leg_state_105
   <= 999999.6739989705
 T(3,7)_AND_Leg_state__y_1: - 999999 T_27 - Leg_state_90 + Leg_state_106
   <= 0
 T(3,7)_AND_Leg_state__y_2: - 999999 T_27 + Leg_state_90 - Leg_state_106
   <= 0
 T(3,7)_AND_Leg_state__y_3: - COM_state_22 + 999999 T_27 + Leg_state_106
   <= 999998.8260010295
 T(3,7)_AND_Leg_state__y_4: COM_state_22 + 999999 T_27 - Leg_state_106
   <= 999999.6739989705
 T(3,7)_AND_Leg_state__z_1: - 999999 T_27 - Leg_state_91 + Leg_state_107
   <= 0
 T(3,7)_AND_Leg_state__z_2: - 999999 T_27 + Leg_state_91 - Leg_state_107
   <= 0
 T(3,7)_AND_Leg_state__z_3: 999999 T_27 - Leg_state_91 + Leg_state_107
   <= 999999.1
 T(3,7)_AND_Leg_state__z_4: 999999 T_27 + Leg_state_91 - Leg_state_107
   <= 999999.1
 T(4,7)_AND_Leg_state__x_1: - 999999 T_28 - Leg_state_93 + Leg_state_109
   <= 0
 T(4,7)_AND_Leg_state__x_2: - 999999 T_28 + Leg_state_93 - Leg_state_109
   <= 0
 T(4,7)_AND_Leg_state__x_3: - COM_state_21 + 999999 T_28 + Leg_state_109
   <= 999999.6739989705
 T(4,7)_AND_Leg_state__x_4: COM_state_21 + 999999 T_28 - Leg_state_109
   <= 999998.8260010295
 T(4,7)_AND_Leg_state__y_1: - 999999 T_28 - Leg_state_94 + Leg_state_110
   <= 0
 T(4,7)_AND_Leg_state__y_2: - 999999 T_28 + Leg_state_94 - Leg_state_110
   <= 0
 T(4,7)_AND_Leg_state__y_3: - COM_state_22 + 999999 T_28 + Leg_state_110
   <= 999999.6739989705
 T(4,7)_AND_Leg_state__y_4: COM_state_22 + 999999 T_28 - Leg_state_110
   <= 999998.8260010295
 T(4,7)_AND_Leg_state__z_1: - 999999 T_28 - Leg_state_95 + Leg_state_111
   <= 0
 T(4,7)_AND_Leg_state__z_2: - 999999 T_28 + Leg_state_95 - Leg_state_111
   <= 0
 T(4,7)_AND_Leg_state__z_3: 999999 T_28 - Leg_state_95 + Leg_state_111
   <= 999999.1
 T(4,7)_AND_Leg_state__z_4: 999999 T_28 + Leg_state_95 - Leg_state_111
   <= 999999.1
 T(1,8)_AND_Leg_state__x_1: - 999999 T_29 - Leg_state_97 + Leg_state_113
   <= 0
 T(1,8)_AND_Leg_state__x_2: - 999999 T_29 + Leg_state_97 - Leg_state_113
   <= 0
 T(1,8)_AND_Leg_state__x_3: - COM_state_25 + 999999 T_29 + Leg_state_113
   <= 999999.6739989705
 T(1,8)_AND_Leg_state__x_4: COM_state_25 + 999999 T_29 - Leg_state_113
   <= 999998.8260010295
 T(1,8)_AND_Leg_state__y_1: - 999999 T_29 - Leg_state_98 + Leg_state_114
   <= 0
 T(1,8)_AND_Leg_state__y_2: - 999999 T_29 + Leg_state_98 - Leg_state_114
   <= 0
 T(1,8)_AND_Leg_state__y_3: - COM_state_26 + 999999 T_29 + Leg_state_114
   <= 999999.6739989705
 T(1,8)_AND_Leg_state__y_4: COM_state_26 + 999999 T_29 - Leg_state_114
   <= 999998.8260010295
 T(1,8)_AND_Leg_state__z_1: - 999999 T_29 - Leg_state_99 + Leg_state_115
   <= 0
 T(1,8)_AND_Leg_state__z_2: - 999999 T_29 + Leg_state_99 - Leg_state_115
   <= 0
 T(1,8)_AND_Leg_state__z_3: 999999 T_29 - Leg_state_99 + Leg_state_115
   <= 999999.1
 T(1,8)_AND_Leg_state__z_4: 999999 T_29 + Leg_state_99 - Leg_state_115
   <= 999999.1
 T(2,8)_AND_Leg_state__x_1: - 999999 T_30 - Leg_state_101 + Leg_state_117
   <= 0
 T(2,8)_AND_Leg_state__x_2: - 999999 T_30 + Leg_state_101 - Leg_state_117
   <= 0
 T(2,8)_AND_Leg_state__x_3: - COM_state_25 + 999999 T_30 + Leg_state_117
   <= 999998.8260010295
 T(2,8)_AND_Leg_state__x_4: COM_state_25 + 999999 T_30 - Leg_state_117
   <= 999999.6739989705
 T(2,8)_AND_Leg_state__y_1: - 999999 T_30 - Leg_state_102 + Leg_state_118
   <= 0
 T(2,8)_AND_Leg_state__y_2: - 999999 T_30 + Leg_state_102 - Leg_state_118
   <= 0
 T(2,8)_AND_Leg_state__y_3: - COM_state_26 + 999999 T_30 + Leg_state_118
   <= 999998.8260010295
 T(2,8)_AND_Leg_state__y_4: COM_state_26 + 999999 T_30 - Leg_state_118
   <= 999999.6739989705
 T(2,8)_AND_Leg_state__z_1: - 999999 T_30 - Leg_state_103 + Leg_state_119
   <= 0
 T(2,8)_AND_Leg_state__z_2: - 999999 T_30 + Leg_state_103 - Leg_state_119
   <= 0
 T(2,8)_AND_Leg_state__z_3: 999999 T_30 - Leg_state_103 + Leg_state_119
   <= 999999.1
 T(2,8)_AND_Leg_state__z_4: 999999 T_30 + Leg_state_103 - Leg_state_119
   <= 999999.1
 T(3,8)_AND_Leg_state__x_1: - 999999 T_31 - Leg_state_105 + Leg_state_121
   <= 0
 T(3,8)_AND_Leg_state__x_2: - 999999 T_31 + Leg_state_105 - Leg_state_121
   <= 0
 T(3,8)_AND_Leg_state__x_3: - COM_state_25 + 999999 T_31 + Leg_state_121
   <= 999998.8260010295
 T(3,8)_AND_Leg_state__x_4: COM_state_25 + 999999 T_31 - Leg_state_121
   <= 999999.6739989705
 T(3,8)_AND_Leg_state__y_1: - 999999 T_31 - Leg_state_106 + Leg_state_122
   <= 0
 T(3,8)_AND_Leg_state__y_2: - 999999 T_31 + Leg_state_106 - Leg_state_122
   <= 0
 T(3,8)_AND_Leg_state__y_3: - COM_state_26 + 999999 T_31 + Leg_state_122
   <= 999998.8260010295
 T(3,8)_AND_Leg_state__y_4: COM_state_26 + 999999 T_31 - Leg_state_122
   <= 999999.6739989705
 T(3,8)_AND_Leg_state__z_1: - 999999 T_31 - Leg_state_107 + Leg_state_123
   <= 0
 T(3,8)_AND_Leg_state__z_2: - 999999 T_31 + Leg_state_107 - Leg_state_123
   <= 0
 T(3,8)_AND_Leg_state__z_3: 999999 T_31 - Leg_state_107 + Leg_state_123
   <= 999999.1
 T(3,8)_AND_Leg_state__z_4: 999999 T_31 + Leg_state_107 - Leg_state_123
   <= 999999.1
 T(4,8)_AND_Leg_state__x_1: - 999999 T_32 - Leg_state_109 + Leg_state_125
   <= 0
 T(4,8)_AND_Leg_state__x_2: - 999999 T_32 + Leg_state_109 - Leg_state_125
   <= 0
 T(4,8)_AND_Leg_state__x_3: - COM_state_25 + 999999 T_32 + Leg_state_125
   <= 999999.6739989705
 T(4,8)_AND_Leg_state__x_4: COM_state_25 + 999999 T_32 - Leg_state_125
   <= 999998.8260010295
 T(4,8)_AND_Leg_state__y_1: - 999999 T_32 - Leg_state_110 + Leg_state_126
   <= 0
 T(4,8)_AND_Leg_state__y_2: - 999999 T_32 + Leg_state_110 - Leg_state_126
   <= 0
 T(4,8)_AND_Leg_state__y_3: - COM_state_26 + 999999 T_32 + Leg_state_126
   <= 999999.6739989705
 T(4,8)_AND_Leg_state__y_4: COM_state_26 + 999999 T_32 - Leg_state_126
   <= 999998.8260010295
 T(4,8)_AND_Leg_state__z_1: - 999999 T_32 - Leg_state_111 + Leg_state_127
   <= 0
 T(4,8)_AND_Leg_state__z_2: - 999999 T_32 + Leg_state_111 - Leg_state_127
   <= 0
 T(4,8)_AND_Leg_state__z_3: 999999 T_32 - Leg_state_111 + Leg_state_127
   <= 999999.1
 T(4,8)_AND_Leg_state__z_4: 999999 T_32 + Leg_state_111 - Leg_state_127
   <= 999999.1
 T(1,9)_AND_Leg_state__x_1: - 999999 T_33 - Leg_state_113 + Leg_state_129
   <= 0
 T(1,9)_AND_Leg_state__x_2: - 999999 T_33 + Leg_state_113 - Leg_state_129
   <= 0
 T(1,9)_AND_Leg_state__x_3: - COM_state_29 + 999999 T_33 + Leg_state_129
   <= 999999.6739989705
 T(1,9)_AND_Leg_state__x_4: COM_state_29 + 999999 T_33 - Leg_state_129
   <= 999998.8260010295
 T(1,9)_AND_Leg_state__y_1: - 999999 T_33 - Leg_state_114 + Leg_state_130
   <= 0
 T(1,9)_AND_Leg_state__y_2: - 999999 T_33 + Leg_state_114 - Leg_state_130
   <= 0
 T(1,9)_AND_Leg_state__y_3: - COM_state_30 + 999999 T_33 + Leg_state_130
   <= 999999.6739989705
 T(1,9)_AND_Leg_state__y_4: COM_state_30 + 999999 T_33 - Leg_state_130
   <= 999998.8260010295
 T(1,9)_AND_Leg_state__z_1: - 999999 T_33 - Leg_state_115 + Leg_state_131
   <= 0
 T(1,9)_AND_Leg_state__z_2: - 999999 T_33 + Leg_state_115 - Leg_state_131
   <= 0
 T(1,9)_AND_Leg_state__z_3: 999999 T_33 - Leg_state_115 + Leg_state_131
   <= 999999.1
 T(1,9)_AND_Leg_state__z_4: 999999 T_33 + Leg_state_115 - Leg_state_131
   <= 999999.1
 T(2,9)_AND_Leg_state__x_1: - 999999 T_34 - Leg_state_117 + Leg_state_133
   <= 0
 T(2,9)_AND_Leg_state__x_2: - 999999 T_34 + Leg_state_117 - Leg_state_133
   <= 0
 T(2,9)_AND_Leg_state__x_3: - COM_state_29 + 999999 T_34 + Leg_state_133
   <= 999998.8260010295
 T(2,9)_AND_Leg_state__x_4: COM_state_29 + 999999 T_34 - Leg_state_133
   <= 999999.6739989705
 T(2,9)_AND_Leg_state__y_1: - 999999 T_34 - Leg_state_118 + Leg_state_134
   <= 0
 T(2,9)_AND_Leg_state__y_2: - 999999 T_34 + Leg_state_118 - Leg_state_134
   <= 0
 T(2,9)_AND_Leg_state__y_3: - COM_state_30 + 999999 T_34 + Leg_state_134
   <= 999998.8260010295
 T(2,9)_AND_Leg_state__y_4: COM_state_30 + 999999 T_34 - Leg_state_134
   <= 999999.6739989705
 T(2,9)_AND_Leg_state__z_1: - 999999 T_34 - Leg_state_119 + Leg_state_135
   <= 0
 T(2,9)_AND_Leg_state__z_2: - 999999 T_34 + Leg_state_119 - Leg_state_135
   <= 0
 T(2,9)_AND_Leg_state__z_3: 999999 T_34 - Leg_state_119 + Leg_state_135
   <= 999999.1
 T(2,9)_AND_Leg_state__z_4: 999999 T_34 + Leg_state_119 - Leg_state_135
   <= 999999.1
 T(3,9)_AND_Leg_state__x_1: - 999999 T_35 - Leg_state_121 + Leg_state_137
   <= 0
 T(3,9)_AND_Leg_state__x_2: - 999999 T_35 + Leg_state_121 - Leg_state_137
   <= 0
 T(3,9)_AND_Leg_state__x_3: - COM_state_29 + 999999 T_35 + Leg_state_137
   <= 999998.8260010295
 T(3,9)_AND_Leg_state__x_4: COM_state_29 + 999999 T_35 - Leg_state_137
   <= 999999.6739989705
 T(3,9)_AND_Leg_state__y_1: - 999999 T_35 - Leg_state_122 + Leg_state_138
   <= 0
 T(3,9)_AND_Leg_state__y_2: - 999999 T_35 + Leg_state_122 - Leg_state_138
   <= 0
 T(3,9)_AND_Leg_state__y_3: - COM_state_30 + 999999 T_35 + Leg_state_138
   <= 999998.8260010295
 T(3,9)_AND_Leg_state__y_4: COM_state_30 + 999999 T_35 - Leg_state_138
   <= 999999.6739989705
 T(3,9)_AND_Leg_state__z_1: - 999999 T_35 - Leg_state_123 + Leg_state_139
   <= 0
 T(3,9)_AND_Leg_state__z_2: - 999999 T_35 + Leg_state_123 - Leg_state_139
   <= 0
 T(3,9)_AND_Leg_state__z_3: 999999 T_35 - Leg_state_123 + Leg_state_139
   <= 999999.1
 T(3,9)_AND_Leg_state__z_4: 999999 T_35 + Leg_state_123 - Leg_state_139
   <= 999999.1
 T(4,9)_AND_Leg_state__x_1: - 999999 T_36 - Leg_state_125 + Leg_state_141
   <= 0
 T(4,9)_AND_Leg_state__x_2: - 999999 T_36 + Leg_state_125 - Leg_state_141
   <= 0
 T(4,9)_AND_Leg_state__x_3: - COM_state_29 + 999999 T_36 + Leg_state_141
   <= 999999.6739989705
 T(4,9)_AND_Leg_state__x_4: COM_state_29 + 999999 T_36 - Leg_state_141
   <= 999998.8260010295
 T(4,9)_AND_Leg_state__y_1: - 999999 T_36 - Leg_state_126 + Leg_state_142
   <= 0
 T(4,9)_AND_Leg_state__y_2: - 999999 T_36 + Leg_state_126 - Leg_state_142
   <= 0
 T(4,9)_AND_Leg_state__y_3: - COM_state_30 + 999999 T_36 + Leg_state_142
   <= 999999.6739989705
 T(4,9)_AND_Leg_state__y_4: COM_state_30 + 999999 T_36 - Leg_state_142
   <= 999998.8260010295
 T(4,9)_AND_Leg_state__z_1: - 999999 T_36 - Leg_state_127 + Leg_state_143
   <= 0
 T(4,9)_AND_Leg_state__z_2: - 999999 T_36 + Leg_state_127 - Leg_state_143
   <= 0
 T(4,9)_AND_Leg_state__z_3: 999999 T_36 - Leg_state_127 + Leg_state_143
   <= 999999.1
 T(4,9)_AND_Leg_state__z_4: 999999 T_36 + Leg_state_127 - Leg_state_143
   <= 999999.1
 T(1,10)_AND_Leg_state__x_1: - 999999 T_37 - Leg_state_129 + Leg_state_145
   <= 0
 T(1,10)_AND_Leg_state__x_2: - 999999 T_37 + Leg_state_129 - Leg_state_145
   <= 0
 T(1,10)_AND_Leg_state__x_3: - COM_state_33 + 999999 T_37 + Leg_state_145
   <= 999999.6739989705
 T(1,10)_AND_Leg_state__x_4: COM_state_33 + 999999 T_37 - Leg_state_145
   <= 999998.8260010295
 T(1,10)_AND_Leg_state__y_1: - 999999 T_37 - Leg_state_130 + Leg_state_146
   <= 0
 T(1,10)_AND_Leg_state__y_2: - 999999 T_37 + Leg_state_130 - Leg_state_146
   <= 0
 T(1,10)_AND_Leg_state__y_3: - COM_state_34 + 999999 T_37 + Leg_state_146
   <= 999999.6739989705
 T(1,10)_AND_Leg_state__y_4: COM_state_34 + 999999 T_37 - Leg_state_146
   <= 999998.8260010295
 T(1,10)_AND_Leg_state__z_1: - 999999 T_37 - Leg_state_131 + Leg_state_147
   <= 0
 T(1,10)_AND_Leg_state__z_2: - 999999 T_37 + Leg_state_131 - Leg_state_147
   <= 0
 T(1,10)_AND_Leg_state__z_3: 999999 T_37 - Leg_state_131 + Leg_state_147
   <= 999999.1
 T(1,10)_AND_Leg_state__z_4: 999999 T_37 + Leg_state_131 - Leg_state_147
   <= 999999.1
 T(2,10)_AND_Leg_state__x_1: - 999999 T_38 - Leg_state_133 + Leg_state_149
   <= 0
 T(2,10)_AND_Leg_state__x_2: - 999999 T_38 + Leg_state_133 - Leg_state_149
   <= 0
 T(2,10)_AND_Leg_state__x_3: - COM_state_33 + 999999 T_38 + Leg_state_149
   <= 999998.8260010295
 T(2,10)_AND_Leg_state__x_4: COM_state_33 + 999999 T_38 - Leg_state_149
   <= 999999.6739989705
 T(2,10)_AND_Leg_state__y_1: - 999999 T_38 - Leg_state_134 + Leg_state_150
   <= 0
 T(2,10)_AND_Leg_state__y_2: - 999999 T_38 + Leg_state_134 - Leg_state_150
   <= 0
 T(2,10)_AND_Leg_state__y_3: - COM_state_34 + 999999 T_38 + Leg_state_150
   <= 999998.8260010295
 T(2,10)_AND_Leg_state__y_4: COM_state_34 + 999999 T_38 - Leg_state_150
   <= 999999.6739989705
 T(2,10)_AND_Leg_state__z_1: - 999999 T_38 - Leg_state_135 + Leg_state_151
   <= 0
 T(2,10)_AND_Leg_state__z_2: - 999999 T_38 + Leg_state_135 - Leg_state_151
   <= 0
 T(2,10)_AND_Leg_state__z_3: 999999 T_38 - Leg_state_135 + Leg_state_151
   <= 999999.1
 T(2,10)_AND_Leg_state__z_4: 999999 T_38 + Leg_state_135 - Leg_state_151
   <= 999999.1
 T(3,10)_AND_Leg_state__x_1: - 999999 T_39 - Leg_state_137 + Leg_state_153
   <= 0
 T(3,10)_AND_Leg_state__x_2: - 999999 T_39 + Leg_state_137 - Leg_state_153
   <= 0
 T(3,10)_AND_Leg_state__x_3: - COM_state_33 + 999999 T_39 + Leg_state_153
   <= 999998.8260010295
 T(3,10)_AND_Leg_state__x_4: COM_state_33 + 999999 T_39 - Leg_state_153
   <= 999999.6739989705
 T(3,10)_AND_Leg_state__y_1: - 999999 T_39 - Leg_state_138 + Leg_state_154
   <= 0
 T(3,10)_AND_Leg_state__y_2: - 999999 T_39 + Leg_state_138 - Leg_state_154
   <= 0
 T(3,10)_AND_Leg_state__y_3: - COM_state_34 + 999999 T_39 + Leg_state_154
   <= 999998.8260010295
 T(3,10)_AND_Leg_state__y_4: COM_state_34 + 999999 T_39 - Leg_state_154
   <= 999999.6739989705
 T(3,10)_AND_Leg_state__z_1: - 999999 T_39 - Leg_state_139 + Leg_state_155
   <= 0
 T(3,10)_AND_Leg_state__z_2: - 999999 T_39 + Leg_state_139 - Leg_state_155
   <= 0
 T(3,10)_AND_Leg_state__z_3: 999999 T_39 - Leg_state_139 + Leg_state_155
   <= 999999.1
 T(3,10)_AND_Leg_state__z_4: 999999 T_39 + Leg_state_139 - Leg_state_155
   <= 999999.1
 T(4,10)_AND_Leg_state__x_1: - 999999 T_40 - Leg_state_141 + Leg_state_157
   <= 0
 T(4,10)_AND_Leg_state__x_2: - 999999 T_40 + Leg_state_141 - Leg_state_157
   <= 0
 T(4,10)_AND_Leg_state__x_3: - COM_state_33 + 999999 T_40 + Leg_state_157
   <= 999999.6739989705
 T(4,10)_AND_Leg_state__x_4: COM_state_33 + 999999 T_40 - Leg_state_157
   <= 999998.8260010295
 T(4,10)_AND_Leg_state__y_1: - 999999 T_40 - Leg_state_142 + Leg_state_158
   <= 0
 T(4,10)_AND_Leg_state__y_2: - 999999 T_40 + Leg_state_142 - Leg_state_158
   <= 0
 T(4,10)_AND_Leg_state__y_3: - COM_state_34 + 999999 T_40 + Leg_state_158
   <= 999999.6739989705
 T(4,10)_AND_Leg_state__y_4: COM_state_34 + 999999 T_40 - Leg_state_158
   <= 999998.8260010295
 T(4,10)_AND_Leg_state__z_1: - 999999 T_40 - Leg_state_143 + Leg_state_159
   <= 0
 T(4,10)_AND_Leg_state__z_2: - 999999 T_40 + Leg_state_143 - Leg_state_159
   <= 0
 T(4,10)_AND_Leg_state__z_3: 999999 T_40 - Leg_state_143 + Leg_state_159
   <= 999999.1
 T(4,10)_AND_Leg_state__z_4: 999999 T_40 + Leg_state_143 - Leg_state_159
   <= 999999.1
 T(1,11)_AND_Leg_state__x_1: - 999999 T_41 - Leg_state_145 + Leg_state_161
   <= 0
 T(1,11)_AND_Leg_state__x_2: - 999999 T_41 + Leg_state_145 - Leg_state_161
   <= 0
 T(1,11)_AND_Leg_state__x_3: - COM_state_37 + 999999 T_41 + Leg_state_161
   <= 999999.6739989705
 T(1,11)_AND_Leg_state__x_4: COM_state_37 + 999999 T_41 - Leg_state_161
   <= 999998.8260010295
 T(1,11)_AND_Leg_state__y_1: - 999999 T_41 - Leg_state_146 + Leg_state_162
   <= 0
 T(1,11)_AND_Leg_state__y_2: - 999999 T_41 + Leg_state_146 - Leg_state_162
   <= 0
 T(1,11)_AND_Leg_state__y_3: - COM_state_38 + 999999 T_41 + Leg_state_162
   <= 999999.6739989705
 T(1,11)_AND_Leg_state__y_4: COM_state_38 + 999999 T_41 - Leg_state_162
   <= 999998.8260010295
 T(1,11)_AND_Leg_state__z_1: - 999999 T_41 - Leg_state_147 + Leg_state_163
   <= 0
 T(1,11)_AND_Leg_state__z_2: - 999999 T_41 + Leg_state_147 - Leg_state_163
   <= 0
 T(1,11)_AND_Leg_state__z_3: 999999 T_41 - Leg_state_147 + Leg_state_163
   <= 999999.1
 T(1,11)_AND_Leg_state__z_4: 999999 T_41 + Leg_state_147 - Leg_state_163
   <= 999999.1
 T(2,11)_AND_Leg_state__x_1: - 999999 T_42 - Leg_state_149 + Leg_state_165
   <= 0
 T(2,11)_AND_Leg_state__x_2: - 999999 T_42 + Leg_state_149 - Leg_state_165
   <= 0
 T(2,11)_AND_Leg_state__x_3: - COM_state_37 + 999999 T_42 + Leg_state_165
   <= 999998.8260010295
 T(2,11)_AND_Leg_state__x_4: COM_state_37 + 999999 T_42 - Leg_state_165
   <= 999999.6739989705
 T(2,11)_AND_Leg_state__y_1: - 999999 T_42 - Leg_state_150 + Leg_state_166
   <= 0
 T(2,11)_AND_Leg_state__y_2: - 999999 T_42 + Leg_state_150 - Leg_state_166
   <= 0
 T(2,11)_AND_Leg_state__y_3: - COM_state_38 + 999999 T_42 + Leg_state_166
   <= 999998.8260010295
 T(2,11)_AND_Leg_state__y_4: COM_state_38 + 999999 T_42 - Leg_state_166
   <= 999999.6739989705
 T(2,11)_AND_Leg_state__z_1: - 999999 T_42 - Leg_state_151 + Leg_state_167
   <= 0
 T(2,11)_AND_Leg_state__z_2: - 999999 T_42 + Leg_state_151 - Leg_state_167
   <= 0
 T(2,11)_AND_Leg_state__z_3: 999999 T_42 - Leg_state_151 + Leg_state_167
   <= 999999.1
 T(2,11)_AND_Leg_state__z_4: 999999 T_42 + Leg_state_151 - Leg_state_167
   <= 999999.1
 T(3,11)_AND_Leg_state__x_1: - 999999 T_43 - Leg_state_153 + Leg_state_169
   <= 0
 T(3,11)_AND_Leg_state__x_2: - 999999 T_43 + Leg_state_153 - Leg_state_169
   <= 0
 T(3,11)_AND_Leg_state__x_3: - COM_state_37 + 999999 T_43 + Leg_state_169
   <= 999998.8260010295
 T(3,11)_AND_Leg_state__x_4: COM_state_37 + 999999 T_43 - Leg_state_169
   <= 999999.6739989705
 T(3,11)_AND_Leg_state__y_1: - 999999 T_43 - Leg_state_154 + Leg_state_170
   <= 0
 T(3,11)_AND_Leg_state__y_2: - 999999 T_43 + Leg_state_154 - Leg_state_170
   <= 0
 T(3,11)_AND_Leg_state__y_3: - COM_state_38 + 999999 T_43 + Leg_state_170
   <= 999998.8260010295
 T(3,11)_AND_Leg_state__y_4: COM_state_38 + 999999 T_43 - Leg_state_170
   <= 999999.6739989705
 T(3,11)_AND_Leg_state__z_1: - 999999 T_43 - Leg_state_155 + Leg_state_171
   <= 0
 T(3,11)_AND_Leg_state__z_2: - 999999 T_43 + Leg_state_155 - Leg_state_171
   <= 0
 T(3,11)_AND_Leg_state__z_3: 999999 T_43 - Leg_state_155 + Leg_state_171
   <= 999999.1
 T(3,11)_AND_Leg_state__z_4: 999999 T_43 + Leg_state_155 - Leg_state_171
   <= 999999.1
 T(4,11)_AND_Leg_state__x_1: - 999999 T_44 - Leg_state_157 + Leg_state_173
   <= 0
 T(4,11)_AND_Leg_state__x_2: - 999999 T_44 + Leg_state_157 - Leg_state_173
   <= 0
 T(4,11)_AND_Leg_state__x_3: - COM_state_37 + 999999 T_44 + Leg_state_173
   <= 999999.6739989705
 T(4,11)_AND_Leg_state__x_4: COM_state_37 + 999999 T_44 - Leg_state_173
   <= 999998.8260010295
 T(4,11)_AND_Leg_state__y_1: - 999999 T_44 - Leg_state_158 + Leg_state_174
   <= 0
 T(4,11)_AND_Leg_state__y_2: - 999999 T_44 + Leg_state_158 - Leg_state_174
   <= 0
 T(4,11)_AND_Leg_state__y_3: - COM_state_38 + 999999 T_44 + Leg_state_174
   <= 999999.6739989705
 T(4,11)_AND_Leg_state__y_4: COM_state_38 + 999999 T_44 - Leg_state_174
   <= 999998.8260010295
 T(4,11)_AND_Leg_state__z_1: - 999999 T_44 - Leg_state_159 + Leg_state_175
   <= 0
 T(4,11)_AND_Leg_state__z_2: - 999999 T_44 + Leg_state_159 - Leg_state_175
   <= 0
 T(4,11)_AND_Leg_state__z_3: 999999 T_44 - Leg_state_159 + Leg_state_175
   <= 999999.1
 T(4,11)_AND_Leg_state__z_4: 999999 T_44 + Leg_state_159 - Leg_state_175
   <= 999999.1
 T(1,12)_AND_Leg_state__x_1: - 999999 T_45 - Leg_state_161 + Leg_state_177
   <= 0
 T(1,12)_AND_Leg_state__x_2: - 999999 T_45 + Leg_state_161 - Leg_state_177
   <= 0
 T(1,12)_AND_Leg_state__x_3: - COM_state_41 + 999999 T_45 + Leg_state_177
   <= 999999.6739989705
 T(1,12)_AND_Leg_state__x_4: COM_state_41 + 999999 T_45 - Leg_state_177
   <= 999998.8260010295
 T(1,12)_AND_Leg_state__y_1: - 999999 T_45 - Leg_state_162 + Leg_state_178
   <= 0
 T(1,12)_AND_Leg_state__y_2: - 999999 T_45 + Leg_state_162 - Leg_state_178
   <= 0
 T(1,12)_AND_Leg_state__y_3: - COM_state_42 + 999999 T_45 + Leg_state_178
   <= 999999.6739989705
 T(1,12)_AND_Leg_state__y_4: COM_state_42 + 999999 T_45 - Leg_state_178
   <= 999998.8260010295
 T(1,12)_AND_Leg_state__z_1: - 999999 T_45 - Leg_state_163 + Leg_state_179
   <= 0
 T(1,12)_AND_Leg_state__z_2: - 999999 T_45 + Leg_state_163 - Leg_state_179
   <= 0
 T(1,12)_AND_Leg_state__z_3: 999999 T_45 - Leg_state_163 + Leg_state_179
   <= 999999.1
 T(1,12)_AND_Leg_state__z_4: 999999 T_45 + Leg_state_163 - Leg_state_179
   <= 999999.1
 T(2,12)_AND_Leg_state__x_1: - 999999 T_46 - Leg_state_165 + Leg_state_181
   <= 0
 T(2,12)_AND_Leg_state__x_2: - 999999 T_46 + Leg_state_165 - Leg_state_181
   <= 0
 T(2,12)_AND_Leg_state__x_3: - COM_state_41 + 999999 T_46 + Leg_state_181
   <= 999998.8260010295
 T(2,12)_AND_Leg_state__x_4: COM_state_41 + 999999 T_46 - Leg_state_181
   <= 999999.6739989705
 T(2,12)_AND_Leg_state__y_1: - 999999 T_46 - Leg_state_166 + Leg_state_182
   <= 0
 T(2,12)_AND_Leg_state__y_2: - 999999 T_46 + Leg_state_166 - Leg_state_182
   <= 0
 T(2,12)_AND_Leg_state__y_3: - COM_state_42 + 999999 T_46 + Leg_state_182
   <= 999998.8260010295
 T(2,12)_AND_Leg_state__y_4: COM_state_42 + 999999 T_46 - Leg_state_182
   <= 999999.6739989705
 T(2,12)_AND_Leg_state__z_1: - 999999 T_46 - Leg_state_167 + Leg_state_183
   <= 0
 T(2,12)_AND_Leg_state__z_2: - 999999 T_46 + Leg_state_167 - Leg_state_183
   <= 0
 T(2,12)_AND_Leg_state__z_3: 999999 T_46 - Leg_state_167 + Leg_state_183
   <= 999999.1
 T(2,12)_AND_Leg_state__z_4: 999999 T_46 + Leg_state_167 - Leg_state_183
   <= 999999.1
 T(3,12)_AND_Leg_state__x_1: - 999999 T_47 - Leg_state_169 + Leg_state_185
   <= 0
 T(3,12)_AND_Leg_state__x_2: - 999999 T_47 + Leg_state_169 - Leg_state_185
   <= 0
 T(3,12)_AND_Leg_state__x_3: - COM_state_41 + 999999 T_47 + Leg_state_185
   <= 999998.8260010295
 T(3,12)_AND_Leg_state__x_4: COM_state_41 + 999999 T_47 - Leg_state_185
   <= 999999.6739989705
 T(3,12)_AND_Leg_state__y_1: - 999999 T_47 - Leg_state_170 + Leg_state_186
   <= 0
 T(3,12)_AND_Leg_state__y_2: - 999999 T_47 + Leg_state_170 - Leg_state_186
   <= 0
 T(3,12)_AND_Leg_state__y_3: - COM_state_42 + 999999 T_47 + Leg_state_186
   <= 999998.8260010295
 T(3,12)_AND_Leg_state__y_4: COM_state_42 + 999999 T_47 - Leg_state_186
   <= 999999.6739989705
 T(3,12)_AND_Leg_state__z_1: - 999999 T_47 - Leg_state_171 + Leg_state_187
   <= 0
 T(3,12)_AND_Leg_state__z_2: - 999999 T_47 + Leg_state_171 - Leg_state_187
   <= 0
 T(3,12)_AND_Leg_state__z_3: 999999 T_47 - Leg_state_171 + Leg_state_187
   <= 999999.1
 T(3,12)_AND_Leg_state__z_4: 999999 T_47 + Leg_state_171 - Leg_state_187
   <= 999999.1
 T(4,12)_AND_Leg_state__x_1: - 999999 T_48 - Leg_state_173 + Leg_state_189
   <= 0
 T(4,12)_AND_Leg_state__x_2: - 999999 T_48 + Leg_state_173 - Leg_state_189
   <= 0
 T(4,12)_AND_Leg_state__x_3: - COM_state_41 + 999999 T_48 + Leg_state_189
   <= 999999.6739989705
 T(4,12)_AND_Leg_state__x_4: COM_state_41 + 999999 T_48 - Leg_state_189
   <= 999998.8260010295
 T(4,12)_AND_Leg_state__y_1: - 999999 T_48 - Leg_state_174 + Leg_state_190
   <= 0
 T(4,12)_AND_Leg_state__y_2: - 999999 T_48 + Leg_state_174 - Leg_state_190
   <= 0
 T(4,12)_AND_Leg_state__y_3: - COM_state_42 + 999999 T_48 + Leg_state_190
   <= 999999.6739989705
 T(4,12)_AND_Leg_state__y_4: COM_state_42 + 999999 T_48 - Leg_state_190
   <= 999998.8260010295
 T(4,12)_AND_Leg_state__z_1: - 999999 T_48 - Leg_state_175 + Leg_state_191
   <= 0
 T(4,12)_AND_Leg_state__z_2: - 999999 T_48 + Leg_state_175 - Leg_state_191
   <= 0
 T(4,12)_AND_Leg_state__z_3: 999999 T_48 - Leg_state_175 + Leg_state_191
   <= 999999.1
 T(4,12)_AND_Leg_state__z_4: 999999 T_48 + Leg_state_175 - Leg_state_191
   <= 999999.1
 T(1,13)_AND_Leg_state__x_1: - 999999 T_49 - Leg_state_177 + Leg_state_193
   <= 0
 T(1,13)_AND_Leg_state__x_2: - 999999 T_49 + Leg_state_177 - Leg_state_193
   <= 0
 T(1,13)_AND_Leg_state__x_3: - COM_state_45 + 999999 T_49 + Leg_state_193
   <= 999999.6739989705
 T(1,13)_AND_Leg_state__x_4: COM_state_45 + 999999 T_49 - Leg_state_193
   <= 999998.8260010295
 T(1,13)_AND_Leg_state__y_1: - 999999 T_49 - Leg_state_178 + Leg_state_194
   <= 0
 T(1,13)_AND_Leg_state__y_2: - 999999 T_49 + Leg_state_178 - Leg_state_194
   <= 0
 T(1,13)_AND_Leg_state__y_3: - COM_state_46 + 999999 T_49 + Leg_state_194
   <= 999999.6739989705
 T(1,13)_AND_Leg_state__y_4: COM_state_46 + 999999 T_49 - Leg_state_194
   <= 999998.8260010295
 T(1,13)_AND_Leg_state__z_1: - 999999 T_49 - Leg_state_179 + Leg_state_195
   <= 0
 T(1,13)_AND_Leg_state__z_2: - 999999 T_49 + Leg_state_179 - Leg_state_195
   <= 0
 T(1,13)_AND_Leg_state__z_3: 999999 T_49 - Leg_state_179 + Leg_state_195
   <= 999999.1
 T(1,13)_AND_Leg_state__z_4: 999999 T_49 + Leg_state_179 - Leg_state_195
   <= 999999.1
 T(2,13)_AND_Leg_state__x_1: - 999999 T_50 - Leg_state_181 + Leg_state_197
   <= 0
 T(2,13)_AND_Leg_state__x_2: - 999999 T_50 + Leg_state_181 - Leg_state_197
   <= 0
 T(2,13)_AND_Leg_state__x_3: - COM_state_45 + 999999 T_50 + Leg_state_197
   <= 999998.8260010295
 T(2,13)_AND_Leg_state__x_4: COM_state_45 + 999999 T_50 - Leg_state_197
   <= 999999.6739989705
 T(2,13)_AND_Leg_state__y_1: - 999999 T_50 - Leg_state_182 + Leg_state_198
   <= 0
 T(2,13)_AND_Leg_state__y_2: - 999999 T_50 + Leg_state_182 - Leg_state_198
   <= 0
 T(2,13)_AND_Leg_state__y_3: - COM_state_46 + 999999 T_50 + Leg_state_198
   <= 999998.8260010295
 T(2,13)_AND_Leg_state__y_4: COM_state_46 + 999999 T_50 - Leg_state_198
   <= 999999.6739989705
 T(2,13)_AND_Leg_state__z_1: - 999999 T_50 - Leg_state_183 + Leg_state_199
   <= 0
 T(2,13)_AND_Leg_state__z_2: - 999999 T_50 + Leg_state_183 - Leg_state_199
   <= 0
 T(2,13)_AND_Leg_state__z_3: 999999 T_50 - Leg_state_183 + Leg_state_199
   <= 999999.1
 T(2,13)_AND_Leg_state__z_4: 999999 T_50 + Leg_state_183 - Leg_state_199
   <= 999999.1
 T(3,13)_AND_Leg_state__x_1: - 999999 T_51 - Leg_state_185 + Leg_state_201
   <= 0
 T(3,13)_AND_Leg_state__x_2: - 999999 T_51 + Leg_state_185 - Leg_state_201
   <= 0
 T(3,13)_AND_Leg_state__x_3: - COM_state_45 + 999999 T_51 + Leg_state_201
   <= 999998.8260010295
 T(3,13)_AND_Leg_state__x_4: COM_state_45 + 999999 T_51 - Leg_state_201
   <= 999999.6739989705
 T(3,13)_AND_Leg_state__y_1: - 999999 T_51 - Leg_state_186 + Leg_state_202
   <= 0
 T(3,13)_AND_Leg_state__y_2: - 999999 T_51 + Leg_state_186 - Leg_state_202
   <= 0
 T(3,13)_AND_Leg_state__y_3: - COM_state_46 + 999999 T_51 + Leg_state_202
   <= 999998.8260010295
 T(3,13)_AND_Leg_state__y_4: COM_state_46 + 999999 T_51 - Leg_state_202
   <= 999999.6739989705
 T(3,13)_AND_Leg_state__z_1: - 999999 T_51 - Leg_state_187 + Leg_state_203
   <= 0
 T(3,13)_AND_Leg_state__z_2: - 999999 T_51 + Leg_state_187 - Leg_state_203
   <= 0
 T(3,13)_AND_Leg_state__z_3: 999999 T_51 - Leg_state_187 + Leg_state_203
   <= 999999.1
 T(3,13)_AND_Leg_state__z_4: 999999 T_51 + Leg_state_187 - Leg_state_203
   <= 999999.1
 T(4,13)_AND_Leg_state__x_1: - 999999 T_52 - Leg_state_189 + Leg_state_205
   <= 0
 T(4,13)_AND_Leg_state__x_2: - 999999 T_52 + Leg_state_189 - Leg_state_205
   <= 0
 T(4,13)_AND_Leg_state__x_3: - COM_state_45 + 999999 T_52 + Leg_state_205
   <= 999999.6739989705
 T(4,13)_AND_Leg_state__x_4: COM_state_45 + 999999 T_52 - Leg_state_205
   <= 999998.8260010295
 T(4,13)_AND_Leg_state__y_1: - 999999 T_52 - Leg_state_190 + Leg_state_206
   <= 0
 T(4,13)_AND_Leg_state__y_2: - 999999 T_52 + Leg_state_190 - Leg_state_206
   <= 0
 T(4,13)_AND_Leg_state__y_3: - COM_state_46 + 999999 T_52 + Leg_state_206
   <= 999999.6739989705
 T(4,13)_AND_Leg_state__y_4: COM_state_46 + 999999 T_52 - Leg_state_206
   <= 999998.8260010295
 T(4,13)_AND_Leg_state__z_1: - 999999 T_52 - Leg_state_191 + Leg_state_207
   <= 0
 T(4,13)_AND_Leg_state__z_2: - 999999 T_52 + Leg_state_191 - Leg_state_207
   <= 0
 T(4,13)_AND_Leg_state__z_3: 999999 T_52 - Leg_state_191 + Leg_state_207
   <= 999999.1
 T(4,13)_AND_Leg_state__z_4: 999999 T_52 + Leg_state_191 - Leg_state_207
   <= 999999.1
 COM_state_AND_Leg_state_1: - 4 COM_state_5 + Leg_state_17 + Leg_state_21
   + Leg_state_25 + Leg_state_29 = 0
 COM_state_AND_Leg_state_2: - 4 COM_state_6 + Leg_state_18 + Leg_state_22
   + Leg_state_26 + Leg_state_30 = 0
 COM_state_AND_Leg_state_3: - 4 COM_state_7 + Leg_state_19 + Leg_state_23
   + Leg_state_27 + Leg_state_31 = 0
 COM_state_AND_Leg_state_4: - 4 COM_state_8 + Leg_state_20 + Leg_state_24
   + Leg_state_28 + Leg_state_32 = 0
 COM_state_AND_Leg_state_5: - 4 COM_state_9 + Leg_state_33 + Leg_state_37
   + Leg_state_41 + Leg_state_45 = 0
 COM_state_AND_Leg_state_6: - 4 COM_state_10 + Leg_state_34 + Leg_state_38
   + Leg_state_42 + Leg_state_46 = 0
 COM_state_AND_Leg_state_7: - 4 COM_state_11 + Leg_state_35 + Leg_state_39
   + Leg_state_43 + Leg_state_47 = 0
 COM_state_AND_Leg_state_8: - 4 COM_state_12 + Leg_state_36 + Leg_state_40
   + Leg_state_44 + Leg_state_48 = 0
 COM_state_AND_Leg_state_9: - 4 COM_state_13 + Leg_state_49 + Leg_state_53
   + Leg_state_57 + Leg_state_61 = 0
 COM_state_AND_Leg_state_10: - 4 COM_state_14 + Leg_state_50 + Leg_state_54
   + Leg_state_58 + Leg_state_62 = 0
 COM_state_AND_Leg_state_11: - 4 COM_state_15 + Leg_state_51 + Leg_state_55
   + Leg_state_59 + Leg_state_63 = 0
 COM_state_AND_Leg_state_12: - 4 COM_state_16 + Leg_state_52 + Leg_state_56
   + Leg_state_60 + Leg_state_64 = 0
 COM_state_AND_Leg_state_13: - 4 COM_state_17 + Leg_state_65 + Leg_state_69
   + Leg_state_73 + Leg_state_77 = 0
 COM_state_AND_Leg_state_14: - 4 COM_state_18 + Leg_state_66 + Leg_state_70
   + Leg_state_74 + Leg_state_78 = 0
 COM_state_AND_Leg_state_15: - 4 COM_state_19 + Leg_state_67 + Leg_state_71
   + Leg_state_75 + Leg_state_79 = 0
 COM_state_AND_Leg_state_16: - 4 COM_state_20 + Leg_state_68 + Leg_state_72
   + Leg_state_76 + Leg_state_80 = 0
 COM_state_AND_Leg_state_17: - 4 COM_state_21 + Leg_state_81 + Leg_state_85
   + Leg_state_89 + Leg_state_93 = 0
 COM_state_AND_Leg_state_18: - 4 COM_state_22 + Leg_state_82 + Leg_state_86
   + Leg_state_90 + Leg_state_94 = 0
 COM_state_AND_Leg_state_19: - 4 COM_state_23 + Leg_state_83 + Leg_state_87
   + Leg_state_91 + Leg_state_95 = 0
 COM_state_AND_Leg_state_20: - 4 COM_state_24 + Leg_state_84 + Leg_state_88
   + Leg_state_92 + Leg_state_96 = 0
 COM_state_AND_Leg_state_21: - 4 COM_state_25 + Leg_state_97
   + Leg_state_101 + Leg_state_105 + Leg_state_109 = 0
 COM_state_AND_Leg_state_22: - 4 COM_state_26 + Leg_state_98
   + Leg_state_102 + Leg_state_106 + Leg_state_110 = 0
 COM_state_AND_Leg_state_23: - 4 COM_state_27 + Leg_state_99
   + Leg_state_103 + Leg_state_107 + Leg_state_111 = 0
 COM_state_AND_Leg_state_24: - 4 COM_state_28 + Leg_state_100
   + Leg_state_104 + Leg_state_108 + Leg_state_112 = 0
 COM_state_AND_Leg_state_25: - 4 COM_state_29 + Leg_state_113
   + Leg_state_117 + Leg_state_121 + Leg_state_125 = 0
 COM_state_AND_Leg_state_26: - 4 COM_state_30 + Leg_state_114
   + Leg_state_118 + Leg_state_122 + Leg_state_126 = 0
 COM_state_AND_Leg_state_27: - 4 COM_state_31 + Leg_state_115
   + Leg_state_119 + Leg_state_123 + Leg_state_127 = 0
 COM_state_AND_Leg_state_28: - 4 COM_state_32 + Leg_state_116
   + Leg_state_120 + Leg_state_124 + Leg_state_128 = 0
 COM_state_AND_Leg_state_29: - 4 COM_state_33 + Leg_state_129
   + Leg_state_133 + Leg_state_137 + Leg_state_141 = 0
 COM_state_AND_Leg_state_30: - 4 COM_state_34 + Leg_state_130
   + Leg_state_134 + Leg_state_138 + Leg_state_142 = 0
 COM_state_AND_Leg_state_31: - 4 COM_state_35 + Leg_state_131
   + Leg_state_135 + Leg_state_139 + Leg_state_143 = 0
 COM_state_AND_Leg_state_32: - 4 COM_state_36 + Leg_state_132
   + Leg_state_136 + Leg_state_140 + Leg_state_144 = 0
 COM_state_AND_Leg_state_33: - 4 COM_state_37 + Leg_state_145
   + Leg_state_149 + Leg_state_153 + Leg_state_157 = 0
 COM_state_AND_Leg_state_34: - 4 COM_state_38 + Leg_state_146
   + Leg_state_150 + Leg_state_154 + Leg_state_158 = 0
 COM_state_AND_Leg_state_35: - 4 COM_state_39 + Leg_state_147
   + Leg_state_151 + Leg_state_155 + Leg_state_159 = 0
 COM_state_AND_Leg_state_36: - 4 COM_state_40 + Leg_state_148
   + Leg_state_152 + Leg_state_156 + Leg_state_160 = 0
 COM_state_AND_Leg_state_37: - 4 COM_state_41 + Leg_state_161
   + Leg_state_165 + Leg_state_169 + Leg_state_173 = 0
 COM_state_AND_Leg_state_38: - 4 COM_state_42 + Leg_state_162
   + Leg_state_166 + Leg_state_170 + Leg_state_174 = 0
 COM_state_AND_Leg_state_39: - 4 COM_state_43 + Leg_state_163
   + Leg_state_167 + Leg_state_171 + Leg_state_175 = 0
 COM_state_AND_Leg_state_40: - 4 COM_state_44 + Leg_state_164
   + Leg_state_168 + Leg_state_172 + Leg_state_176 = 0
 COM_state_AND_Leg_state_41: - 4 COM_state_45 + Leg_state_177
   + Leg_state_181 + Leg_state_185 + Leg_state_189 = 0
 COM_state_AND_Leg_state_42: - 4 COM_state_46 + Leg_state_178
   + Leg_state_182 + Leg_state_186 + Leg_state_190 = 0
 COM_state_AND_Leg_state_43: - 4 COM_state_47 + Leg_state_179
   + Leg_state_183 + Leg_state_187 + Leg_state_191 = 0
 COM_state_AND_Leg_state_44: - 4 COM_state_48 + Leg_state_180
   + Leg_state_184 + Leg_state_188 + Leg_state_192 = 0
 COM_state_AND_Leg_state_45: - 4 COM_state_49 + Leg_state_193
   + Leg_state_197 + Leg_state_201 + Leg_state_205 = 0
 COM_state_AND_Leg_state_46: - 4 COM_state_50 + Leg_state_194
   + Leg_state_198 + Leg_state_202 + Leg_state_206 = 0
 COM_state_AND_Leg_state_47: - 4 COM_state_51 + Leg_state_195
   + Leg_state_199 + Leg_state_203 + Leg_state_207 = 0
 COM_state_AND_Leg_state_48: - 4 COM_state_52 + Leg_state_196
   + Leg_state_200 + Leg_state_204 + Leg_state_208 = 0
Bounds
 COM_state_1 = 0
 COM_state_2 = 0
 COM_state_3 = 0
 COM_state_4 = 0
 -10 <= COM_state_5 <= 10
 -10 <= COM_state_6 <= 10
 -10 <= COM_state_7 <= 10
 -10 <= COM_state_8 <= 10
 -10 <= COM_state_9 <= 10
 -10 <= COM_state_10 <= 10
 -10 <= COM_state_11 <= 10
 -10 <= COM_state_12 <= 10
 -10 <= COM_state_13 <= 10
 -10 <= COM_state_14 <= 10
 -10 <= COM_state_15 <= 10
 -10 <= COM_state_16 <= 10
 -10 <= COM_state_17 <= 10
 -10 <= COM_state_18 <= 10
 -10 <= COM_state_19 <= 10
 -10 <= COM_state_20 <= 10
 -10 <= COM_state_21 <= 10
 -10 <= COM_state_22 <= 10
 -10 <= COM_state_23 <= 10
 -10 <= COM_state_24 <= 10
 -10 <= COM_state_25 <= 10
 -10 <= COM_state_26 <= 10
 -10 <= COM_state_27 <= 10
 -10 <= COM_state_28 <= 10
 -10 <= COM_state_29 <= 10
 -10 <= COM_state_30 <= 10
 -10 <= COM_state_31 <= 10
 -10 <= COM_state_32 <= 10
 -10 <= COM_state_33 <= 10
 -10 <= COM_state_34 <= 10
 -10 <= COM_state_35 <= 10
 -10 <= COM_state_36 <= 10
 -10 <= COM_state_37 <= 10
 -10 <= COM_state_38 <= 10
 -10 <= COM_state_39 <= 10
 -10 <= COM_state_40 <= 10
 -10 <= COM_state_41 <= 10
 -10 <= COM_state_42 <= 10
 -10 <= COM_state_43 <= 10
 -10 <= COM_state_44 <= 10
 -10 <= COM_state_45 <= 10
 -10 <= COM_state_46 <= 10
 -10 <= COM_state_47 <= 10
 -10 <= COM_state_48 <= 10
 -10 <= COM_state_49 <= 10
 -10 <= COM_state_50 <= 10
 -10 <= COM_state_51 <= 10
 -10 <= COM_state_52 <= 10
 T_1 = 1
 T_2 = 1
 T_3 = 1
 T_4 = 1
 Leg_state_1 = 0.4239989705688516
 Leg_state_2 = 0.2649997602952768
 Leg_state_3 = 0
 Leg_state_4 = 0
 Leg_state_5 = -0.4239989705688514
 Leg_state_6 = 0.2649997602952769
 Leg_state_7 = 0
 Leg_state_8 = 0
 Leg_state_9 = -0.4239989705688514
 Leg_state_10 = -0.2649997602952769
 Leg_state_11 = 0
 Leg_state_12 = 0
 Leg_state_13 = 0.4239989705688516
 Leg_state_14 = -0.2649997602952768
 Leg_state_15 = 0
 Leg_state_16 = 0
 -20 <= Leg_state_17 <= 20
 -20 <= Leg_state_18 <= 20
 -20 <= Leg_state_19 <= 20
 Leg_state_20 = 0
 -20 <= Leg_state_21 <= 20
 -20 <= Leg_state_22 <= 20
 -20 <= Leg_state_23 <= 20
 Leg_state_24 = 0
 -20 <= Leg_state_25 <= 20
 -20 <= Leg_state_26 <= 20
 -20 <= Leg_state_27 <= 20
 Leg_state_28 = 0
 -20 <= Leg_state_29 <= 20
 -20 <= Leg_state_30 <= 20
 -20 <= Leg_state_31 <= 20
 Leg_state_32 = 0
 -20 <= Leg_state_33 <= 20
 -20 <= Leg_state_34 <= 20
 -20 <= Leg_state_35 <= 20
 Leg_state_36 = 0
 -20 <= Leg_state_37 <= 20
 -20 <= Leg_state_38 <= 20
 -20 <= Leg_state_39 <= 20
 Leg_state_40 = 0
 -20 <= Leg_state_41 <= 20
 -20 <= Leg_state_42 <= 20
 -20 <= Leg_state_43 <= 20
 Leg_state_44 = 0
 -20 <= Leg_state_45 <= 20
 -20 <= Leg_state_46 <= 20
 -20 <= Leg_state_47 <= 20
 Leg_state_48 = 0
 -20 <= Leg_state_49 <= 20
 -20 <= Leg_state_50 <= 20
 -20 <= Leg_state_51 <= 20
 Leg_state_52 = 0
 -20 <= Leg_state_53 <= 20
 -20 <= Leg_state_54 <= 20
 -20 <= Leg_state_55 <= 20
 Leg_state_56 = 0
 -20 <= Leg_state_57 <= 20
 -20 <= Leg_state_58 <= 20
 -20 <= Leg_state_59 <= 20
 Leg_state_60 = 0
 -20 <= Leg_state_61 <= 20
 -20 <= Leg_state_62 <= 20
 -20 <= Leg_state_63 <= 20
 Leg_state_64 = 0
 -20 <= Leg_state_65 <= 20
 -20 <= Leg_state_66 <= 20
 -20 <= Leg_state_67 <= 20
 Leg_state_68 = 0
 -20 <= Leg_state_69 <= 20
 -20 <= Leg_state_70 <= 20
 -20 <= Leg_state_71 <= 20
 Leg_state_72 = 0
 -20 <= Leg_state_73 <= 20
 -20 <= Leg_state_74 <= 20
 -20 <= Leg_state_75 <= 20
 Leg_state_76 = 0
 -20 <= Leg_state_77 <= 20
 -20 <= Leg_state_78 <= 20
 -20 <= Leg_state_79 <= 20
 Leg_state_80 = 0
 -20 <= Leg_state_81 <= 20
 -20 <= Leg_state_82 <= 20
 -20 <= Leg_state_83 <= 20
 Leg_state_84 = 0
 -20 <= Leg_state_85 <= 20
 -20 <= Leg_state_86 <= 20
 -20 <= Leg_state_87 <= 20
 Leg_state_88 = 0
 -20 <= Leg_state_89 <= 20
 -20 <= Leg_state_90 <= 20
 -20 <= Leg_state_91 <= 20
 Leg_state_92 = 0
 -20 <= Leg_state_93 <= 20
 -20 <= Leg_state_94 <= 20
 -20 <= Leg_state_95 <= 20
 Leg_state_96 = 0
 -20 <= Leg_state_97 <= 20
 -20 <= Leg_state_98 <= 20
 -20 <= Leg_state_99 <= 20
 Leg_state_100 = 0
 -20 <= Leg_state_101 <= 20
 -20 <= Leg_state_102 <= 20
 -20 <= Leg_state_103 <= 20
 Leg_state_104 = 0
 -20 <= Leg_state_105 <= 20
 -20 <= Leg_state_106 <= 20
 -20 <= Leg_state_107 <= 20
 Leg_state_108 = 0
 -20 <= Leg_state_109 <= 20
 -20 <= Leg_state_110 <= 20
 -20 <= Leg_state_111 <= 20
 Leg_state_112 = 0
 -20 <= Leg_state_113 <= 20
 -20 <= Leg_state_114 <= 20
 -20 <= Leg_state_115 <= 20
 Leg_state_116 = 0
 -20 <= Leg_state_117 <= 20
 -20 <= Leg_state_118 <= 20
 -20 <= Leg_state_119 <= 20
 Leg_state_120 = 0
 -20 <= Leg_state_121 <= 20
 -20 <= Leg_state_122 <= 20
 -20 <= Leg_state_123 <= 20
 Leg_state_124 = 0
 -20 <= Leg_state_125 <= 20
 -20 <= Leg_state_126 <= 20
 -20 <= Leg_state_127 <= 20
 Leg_state_128 = 0
 -20 <= Leg_state_129 <= 20
 -20 <= Leg_state_130 <= 20
 -20 <= Leg_state_131 <= 20
 Leg_state_132 = 0
 -20 <= Leg_state_133 <= 20
 -20 <= Leg_state_134 <= 20
 -20 <= Leg_state_135 <= 20
 Leg_state_136 = 0
 -20 <= Leg_state_137 <= 20
 -20 <= Leg_state_138 <= 20
 -20 <= Leg_state_139 <= 20
 Leg_state_140 = 0
 -20 <= Leg_state_141 <= 20
 -20 <= Leg_state_142 <= 20
 -20 <= Leg_state_143 <= 20
 Leg_state_144 = 0
 -20 <= Leg_state_145 <= 20
 -20 <= Leg_state_146 <= 20
 -20 <= Leg_state_147 <= 20
 Leg_state_148 = 0
 -20 <= Leg_state_149 <= 20
 -20 <= Leg_state_150 <= 20
 -20 <= Leg_state_151 <= 20
 Leg_state_152 = 0
 -20 <= Leg_state_153 <= 20
 -20 <= Leg_state_154 <= 20
 -20 <= Leg_state_155 <= 20
 Leg_state_156 = 0
 -20 <= Leg_state_157 <= 20
 -20 <= Leg_state_158 <= 20
 -20 <= Leg_state_159 <= 20
 Leg_state_160 = 0
 -20 <= Leg_state_161 <= 20
 -20 <= Leg_state_162 <= 20
 -20 <= Leg_state_163 <= 20
 Leg_state_164 = 0
 -20 <= Leg_state_165 <= 20
 -20 <= Leg_state_166 <= 20
 -20 <= Leg_state_167 <= 20
 Leg_state_168 = 0
 -20 <= Leg_state_169 <= 20
 -20 <= Leg_state_170 <= 20
 -20 <= Leg_state_171 <= 20
 Leg_state_172 = 0
 -20 <= Leg_state_173 <= 20
 -20 <= Leg_state_174 <= 20
 -20 <= Leg_state_175 <= 20
 Leg_state_176 = 0
 -20 <= Leg_state_177 <= 20
 -20 <= Leg_state_178 <= 20
 -20 <= Leg_state_179 <= 20
 Leg_state_180 = 0
 -20 <= Leg_state_181 <= 20
 -20 <= Leg_state_182 <= 20
 -20 <= Leg_state_183 <= 20
 Leg_state_184 = 0
 -20 <= Leg_state_185 <= 20
 -20 <= Leg_state_186 <= 20
 -20 <= Leg_state_187 <= 20
 Leg_state_188 = 0
 -20 <= Leg_state_189 <= 20
 -20 <= Leg_state_190 <= 20
 -20 <= Leg_state_191 <= 20
 Leg_state_192 = 0
 -20 <= Leg_state_193 <= 20
 -20 <= Leg_state_194 <= 20
 -20 <= Leg_state_195 <= 20
 Leg_state_196 = 0
 -20 <= Leg_state_197 <= 20
 -20 <= Leg_state_198 <= 20
 -20 <= Leg_state_199 <= 20
 Leg_state_200 = 0
 -20 <= Leg_state_201 <= 20
 -20 <= Leg_state_202 <= 20
 -20 <= Leg_state_203 <= 20
 Leg_state_204 = 0
 -20 <= Leg_state_205 <= 20
 -20 <= Leg_state_206 <= 20
 -20 <= Leg_state_207 <= 20
 Leg_state_208 = 0
Binaries
 T_1 T_2 T_3 T_4 T_5 T_6 T_7 T_8 T_9 T_10 T_11 T_12 T_13 T_14 T_15 T_16
 T_17 T_18 T_19 T_20 T_21 T_22 T_23 T_24 T_25 T_26 T_27 T_28 T_29 T_30 T_31
 T_32 T_33 T_34 T_35 T_36 T_37 T_38 T_39 T_40 T_41 T_42 T_43 T_44 T_45 T_46
 T_47 T_48 T_49 T_50 T_51 T_52
End
