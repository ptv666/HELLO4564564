\ Model matlab
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  10 x_1 + x_2
Subject To
 LP_1: - x_1 <= -1
 LP_2: x_1 + x_2 <= 2
 LP_3: x_1 - x_2 <= 2
Bounds
 x_1 free
 x_2 free
End
