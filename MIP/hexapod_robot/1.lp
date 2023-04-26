\ Model matlab
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  - Z_1 - Z_2 - Z_3 - Z_4 - Z_5 - Z_6 - Z_7 - Z_8 - Z_9
Subject To
 comput_delta_x_y_1: x_1 - delta_x_1 = 0.1193
 comput_delta_x_y_2: y_1 - delta_y_1 = 0.2502
 comput_delta_x_y_3: x_1 - delta_x_2 = 0.1367
 comput_delta_x_y_4: y_1 - delta_y_2 = -0.2478
 comput_delta_x_y_5: x_1 - delta_x_3 = 0.3065
 comput_delta_x_y_6: y_1 - delta_y_3 = -0.2502
 comput_delta_x_y_7: x_1 - delta_x_4 = 0.3065
 comput_delta_x_y_8: y_1 - delta_y_4 = 0.2494
 comput_delta_x_y_9: x_1 - delta_x_5 = 0.8467
 comput_delta_x_y_10: y_1 - delta_y_5 = 0.2502
 comput_delta_x_y_11: x_1 - delta_x_6 = 0.8776
 comput_delta_x_y_12: y_1 - delta_y_6 = -0.2494
 comput_delta_x_y_13: x_1 - delta_x_7 = 0.9895
 comput_delta_x_y_14: y_1 - delta_y_7 = 0.2494
 comput_delta_x_y_15: x_1 - delta_x_8 = 1.0338
 comput_delta_x_y_16: y_1 - delta_y_8 = -0.2494
 comput_delta_x_y_17: x_1 - delta_x_9 = 1.1457
 comput_delta_x_y_18: y_1 - delta_y_9 = 0.2494
 eq_1: X_1 + Y_1 + 0.0001 Temp_1 = 0
 eq_2: X_2 + Y_2 + 0.0001 Temp_2 = 0
 eq_3: X_3 + Y_3 + 0.0001 Temp_3 = 0
 eq_4: X_4 + Y_4 + 0.0001 Temp_4 = 0
 eq_5: X_5 + Y_5 + 0.0001 Temp_5 = 0
 eq_6: X_6 + Y_6 + 0.0001 Temp_6 = 0
 eq_7: X_7 + Y_7 + 0.0001 Temp_7 = 0
 eq_8: X_8 + Y_8 + 0.0001 Temp_8 = 0
 eq_9: X_9 + Y_9 + 0.0001 Temp_9 = 0
Bounds
 -2 <= x_1 <= 2
 -2 <= y_1 <= 2
 -2 <= delta_x_1 <= 2
 -2 <= delta_x_2 <= 2
 -2 <= delta_x_3 <= 2
 -2 <= delta_x_4 <= 2
 -2 <= delta_x_5 <= 2
 -2 <= delta_x_6 <= 2
 -2 <= delta_x_7 <= 2
 -2 <= delta_x_8 <= 2
 -2 <= delta_x_9 <= 2
 -2 <= delta_y_1 <= 2
 -2 <= delta_y_2 <= 2
 -2 <= delta_y_3 <= 2
 -2 <= delta_y_4 <= 2
 -2 <= delta_y_5 <= 2
 -2 <= delta_y_6 <= 2
 -2 <= delta_y_7 <= 2
 -2 <= delta_y_8 <= 2
 -2 <= delta_y_9 <= 2
 X_1 free
 X_2 free
 X_3 free
 X_4 free
 X_5 free
 X_6 free
 X_7 free
 X_8 free
 X_9 free
 Y_1 free
 Y_2 free
 Y_3 free
 Y_4 free
 Y_5 free
 Y_6 free
 Y_7 free
 Y_8 free
 Y_9 free
 Temp_1 free
 Temp_2 free
 Temp_3 free
 Temp_4 free
 Temp_5 free
 Temp_6 free
 Temp_7 free
 Temp_8 free
 Temp_9 free
 Z_1 free
 Z_2 free
 Z_3 free
 Z_4 free
 Z_5 free
 Z_6 free
 Z_7 free
 Z_8 free
 Z_9 free
General Constraints
 GC0: X_1 = POLY ( 1 delta_x_1 ^ 2 )
 GC1: Y_1 = POLY ( 1 delta_y_1 ^ 2 )
 GC2: X_2 = POLY ( 1 delta_x_2 ^ 2 )
 GC3: Y_2 = POLY ( 1 delta_y_2 ^ 2 )
 GC4: X_3 = POLY ( 1 delta_x_3 ^ 2 )
 GC5: Y_3 = POLY ( 1 delta_y_3 ^ 2 )
 GC6: X_4 = POLY ( 1 delta_x_4 ^ 2 )
 GC7: Y_4 = POLY ( 1 delta_y_4 ^ 2 )
 GC8: X_5 = POLY ( 1 delta_x_5 ^ 2 )
 GC9: Y_5 = POLY ( 1 delta_y_5 ^ 2 )
 GC10: X_6 = POLY ( 1 delta_x_6 ^ 2 )
 GC11: Y_6 = POLY ( 1 delta_y_6 ^ 2 )
 GC12: X_7 = POLY ( 1 delta_x_7 ^ 2 )
 GC13: Y_7 = POLY ( 1 delta_y_7 ^ 2 )
 GC14: X_8 = POLY ( 1 delta_x_8 ^ 2 )
 GC15: Y_8 = POLY ( 1 delta_y_8 ^ 2 )
 GC16: X_9 = POLY ( 1 delta_x_9 ^ 2 )
 GC17: Y_9 = POLY ( 1 delta_y_9 ^ 2 )
 GC18: ( FuncPieces=-2 ) Z_1 = EXP ( Temp_1 )
 GC19: ( FuncPieces=-2 ) Z_2 = EXP ( Temp_2 )
 GC20: ( FuncPieces=-2 ) Z_3 = EXP ( Temp_3 )
 GC21: ( FuncPieces=-2 ) Z_4 = EXP ( Temp_4 )
 GC22: ( FuncPieces=-2 ) Z_5 = EXP ( Temp_5 )
 GC23: ( FuncPieces=-2 ) Z_6 = EXP ( Temp_6 )
 GC24: ( FuncPieces=-2 ) Z_7 = EXP ( Temp_7 )
 GC25: ( FuncPieces=-2 ) Z_8 = EXP ( Temp_8 )
 GC26: ( FuncPieces=-2 ) Z_9 = EXP ( Temp_9 )
End
