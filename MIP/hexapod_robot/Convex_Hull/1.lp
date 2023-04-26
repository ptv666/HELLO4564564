\ Model matlab
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  - 100 alpha_1 - 100 alpha_2 - 100 alpha_3 - 100 beta_1 - 100 beta_2
   - 100 beta_3 - 2 Constant
Subject To
 ÏßÐÔ×éºÏ: alpha_2 - alpha_3 + beta_2 + 2 beta_3 = 1
 ÏßÐÔ×éºÏ: alpha_2 + alpha_3 + 2 beta_2 = 1
 ×éºÏÏµÊýÖ®ºÍÎ: alpha_1 + alpha_2 + alpha_3 = 1
 ×éºÏÏµÊýÖ®ºÍÎ: beta_1 + beta_2 + beta_3 = 1
Bounds
 alpha_1 <= 1
 alpha_2 <= 1
 alpha_3 <= 1
 beta_1 <= 1
 beta_2 <= 1
 beta_3 <= 1
 Constant = 1
End
