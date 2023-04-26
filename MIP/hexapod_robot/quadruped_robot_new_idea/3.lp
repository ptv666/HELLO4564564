\ Model matlab
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  - 600 COM_state_81 + 0 T_1 + 0 T_2 + 0 T_3 + 0 T_4
   - 547.8398352910162 Leg_state_321 - 42.3999616472443 Leg_state_322
   - 412.1601647089838 Leg_state_325 - 42.39996164724431 Leg_state_326
   - 412.1601647089838 Leg_state_329 + 42.39996164724431 Leg_state_330
   - 547.8398352910162 Leg_state_333 + 42.3999616472443 Leg_state_334
   + 0 delta_z_1 - 2 Trim_1 - 2 Trim_2 - 2 Trim_3 - 2 Trim_4 - 2 Trim_5
   - 2 Trim_6 - 2 Trim_7 - 2 Trim_8 - 2 Trim_9 - 2 Trim_10 - 2 Trim_11
   - 2 Trim_12 - 2 Trim_13 - 2 Trim_14 - 2 Trim_15 - 2 Trim_16 - 2 Trim_17
   - 2 Trim_18 - 2 Trim_19 - 2 Trim_20 - 2 Trim_21 + 0 H_1 + 0 H_2 + 0 H_3
   + 0 H_4 + 0 H_5 + 0 H_6 + 0 H_7 + 0 H_8 + 0 H_9 + 0 H_10 + 0 H_11
   + 0 H_12 + 0 H_13 + 0 H_14 + 0 H_15 + 0 H_16 + 0 H_17 + 0 H_18 + 0 H_19
   + 0 H_20 + 3900 Constant + [ 20 COM_state_1 ^2
   - 40 COM_state_1 * COM_state_5 + 20 COM_state_2 ^2
   - 40 COM_state_2 * COM_state_6 + 20 COM_state_3 ^2
   - 40 COM_state_3 * COM_state_7 + 20 COM_state_4 ^2
   - 40 COM_state_4 * COM_state_8 + 40 COM_state_5 ^2
   - 40 COM_state_5 * COM_state_9 + 40 COM_state_6 ^2
   - 40 COM_state_6 * COM_state_10 + 40 COM_state_7 ^2
   - 40 COM_state_7 * COM_state_11 + 40 COM_state_8 ^2
   - 40 COM_state_8 * COM_state_12 + 40 COM_state_9 ^2
   - 40 COM_state_9 * COM_state_13 + 40 COM_state_10 ^2
   - 40 COM_state_10 * COM_state_14 + 40 COM_state_11 ^2
   - 40 COM_state_11 * COM_state_15 + 40 COM_state_12 ^2
   - 40 COM_state_12 * COM_state_16 + 40 COM_state_13 ^2
   - 40 COM_state_13 * COM_state_17 + 40 COM_state_14 ^2
   - 40 COM_state_14 * COM_state_18 + 40 COM_state_15 ^2
   - 40 COM_state_15 * COM_state_19 + 40 COM_state_16 ^2
   - 40 COM_state_16 * COM_state_20 + 40 COM_state_17 ^2
   - 40 COM_state_17 * COM_state_21 + 40 COM_state_18 ^2
   - 40 COM_state_18 * COM_state_22 + 40 COM_state_19 ^2
   - 40 COM_state_19 * COM_state_23 + 40 COM_state_20 ^2
   - 40 COM_state_20 * COM_state_24 + 40 COM_state_21 ^2
   - 40 COM_state_21 * COM_state_25 + 40 COM_state_22 ^2
   - 40 COM_state_22 * COM_state_26 + 40 COM_state_23 ^2
   - 40 COM_state_23 * COM_state_27 + 40 COM_state_24 ^2
   - 40 COM_state_24 * COM_state_28 + 40 COM_state_25 ^2
   - 40 COM_state_25 * COM_state_29 + 40 COM_state_26 ^2
   - 40 COM_state_26 * COM_state_30 + 40 COM_state_27 ^2
   - 40 COM_state_27 * COM_state_31 + 40 COM_state_28 ^2
   - 40 COM_state_28 * COM_state_32 + 40 COM_state_29 ^2
   - 40 COM_state_29 * COM_state_33 + 40 COM_state_30 ^2
   - 40 COM_state_30 * COM_state_34 + 40 COM_state_31 ^2
   - 40 COM_state_31 * COM_state_35 + 40 COM_state_32 ^2
   - 40 COM_state_32 * COM_state_36 + 40 COM_state_33 ^2
   - 40 COM_state_33 * COM_state_37 + 40 COM_state_34 ^2
   - 40 COM_state_34 * COM_state_38 + 40 COM_state_35 ^2
   - 40 COM_state_35 * COM_state_39 + 40 COM_state_36 ^2
   - 40 COM_state_36 * COM_state_40 + 40 COM_state_37 ^2
   - 40 COM_state_37 * COM_state_41 + 40 COM_state_38 ^2
   - 40 COM_state_38 * COM_state_42 + 40 COM_state_39 ^2
   - 40 COM_state_39 * COM_state_43 + 40 COM_state_40 ^2
   - 40 COM_state_40 * COM_state_44 + 40 COM_state_41 ^2
   - 40 COM_state_41 * COM_state_45 + 40 COM_state_42 ^2
   - 40 COM_state_42 * COM_state_46 + 40 COM_state_43 ^2
   - 40 COM_state_43 * COM_state_47 + 40 COM_state_44 ^2
   - 40 COM_state_44 * COM_state_48 + 40 COM_state_45 ^2
   - 40 COM_state_45 * COM_state_49 + 40 COM_state_46 ^2
   - 40 COM_state_46 * COM_state_50 + 40 COM_state_47 ^2
   - 40 COM_state_47 * COM_state_51 + 40 COM_state_48 ^2
   - 40 COM_state_48 * COM_state_52 + 40 COM_state_49 ^2
   - 40 COM_state_49 * COM_state_53 + 40 COM_state_50 ^2
   - 40 COM_state_50 * COM_state_54 + 40 COM_state_51 ^2
   - 40 COM_state_51 * COM_state_55 + 40 COM_state_52 ^2
   - 40 COM_state_52 * COM_state_56 + 40 COM_state_53 ^2
   - 40 COM_state_53 * COM_state_57 + 40 COM_state_54 ^2
   - 40 COM_state_54 * COM_state_58 + 40 COM_state_55 ^2
   - 40 COM_state_55 * COM_state_59 + 40 COM_state_56 ^2
   - 40 COM_state_56 * COM_state_60 + 40 COM_state_57 ^2
   - 40 COM_state_57 * COM_state_61 + 40 COM_state_58 ^2
   - 40 COM_state_58 * COM_state_62 + 40 COM_state_59 ^2
   - 40 COM_state_59 * COM_state_63 + 40 COM_state_60 ^2
   - 40 COM_state_60 * COM_state_64 + 40 COM_state_61 ^2
   - 40 COM_state_61 * COM_state_65 + 40 COM_state_62 ^2
   - 40 COM_state_62 * COM_state_66 + 40 COM_state_63 ^2
   - 40 COM_state_63 * COM_state_67 + 40 COM_state_64 ^2
   - 40 COM_state_64 * COM_state_68 + 40 COM_state_65 ^2
   - 40 COM_state_65 * COM_state_69 + 40 COM_state_66 ^2
   - 40 COM_state_66 * COM_state_70 + 40 COM_state_67 ^2
   - 40 COM_state_67 * COM_state_71 + 40 COM_state_68 ^2
   - 40 COM_state_68 * COM_state_72 + 40 COM_state_69 ^2
   - 40 COM_state_69 * COM_state_73 + 40 COM_state_70 ^2
   - 40 COM_state_70 * COM_state_74 + 40 COM_state_71 ^2
   - 40 COM_state_71 * COM_state_75 + 40 COM_state_72 ^2
   - 40 COM_state_72 * COM_state_76 + 40 COM_state_73 ^2
   - 40 COM_state_73 * COM_state_77 + 40 COM_state_74 ^2
   - 40 COM_state_74 * COM_state_78 + 40 COM_state_75 ^2
   - 40 COM_state_75 * COM_state_79 + 40 COM_state_76 ^2
   - 40 COM_state_76 * COM_state_80 + 40 COM_state_77 ^2
   - 40 COM_state_77 * COM_state_81 + 40 COM_state_78 ^2
   - 40 COM_state_78 * COM_state_82 + 40 COM_state_79 ^2
   - 40 COM_state_79 * COM_state_83 + 40 COM_state_80 ^2
   - 40 COM_state_80 * COM_state_84 + 220 COM_state_81 ^2
   + 220 COM_state_82 ^2 + 220 COM_state_83 ^2 + 220 COM_state_84 ^2
   + 16 Leg_state_1 ^2 - 32 Leg_state_1 * Leg_state_17 + 16 Leg_state_2 ^2
   - 32 Leg_state_2 * Leg_state_18 + 16 Leg_state_3 ^2
   - 32 Leg_state_3 * Leg_state_19 + 16 Leg_state_4 ^2
   - 32 Leg_state_4 * Leg_state_20 + 16 Leg_state_5 ^2
   - 32 Leg_state_5 * Leg_state_21 + 16 Leg_state_6 ^2
   - 32 Leg_state_6 * Leg_state_22 + 16 Leg_state_7 ^2
   - 32 Leg_state_7 * Leg_state_23 + 16 Leg_state_8 ^2
   - 32 Leg_state_8 * Leg_state_24 + 16 Leg_state_9 ^2
   - 32 Leg_state_9 * Leg_state_25 + 16 Leg_state_10 ^2
   - 32 Leg_state_10 * Leg_state_26 + 16 Leg_state_11 ^2
   - 32 Leg_state_11 * Leg_state_27 + 16 Leg_state_12 ^2
   - 32 Leg_state_12 * Leg_state_28 + 16 Leg_state_13 ^2
   - 32 Leg_state_13 * Leg_state_29 + 16 Leg_state_14 ^2
   - 32 Leg_state_14 * Leg_state_30 + 16 Leg_state_15 ^2
   - 32 Leg_state_15 * Leg_state_31 + 16 Leg_state_16 ^2
   - 32 Leg_state_16 * Leg_state_32 + 32 Leg_state_17 ^2
   - 32 Leg_state_17 * Leg_state_33 + 32 Leg_state_18 ^2
   - 32 Leg_state_18 * Leg_state_34 + 32 Leg_state_19 ^2
   - 32 Leg_state_19 * Leg_state_35 + 32 Leg_state_20 ^2
   - 32 Leg_state_20 * Leg_state_36 + 32 Leg_state_21 ^2
   - 32 Leg_state_21 * Leg_state_37 + 32 Leg_state_22 ^2
   - 32 Leg_state_22 * Leg_state_38 + 32 Leg_state_23 ^2
   - 32 Leg_state_23 * Leg_state_39 + 32 Leg_state_24 ^2
   - 32 Leg_state_24 * Leg_state_40 + 32 Leg_state_25 ^2
   - 32 Leg_state_25 * Leg_state_41 + 32 Leg_state_26 ^2
   - 32 Leg_state_26 * Leg_state_42 + 32 Leg_state_27 ^2
   - 32 Leg_state_27 * Leg_state_43 + 32 Leg_state_28 ^2
   - 32 Leg_state_28 * Leg_state_44 + 32 Leg_state_29 ^2
   - 32 Leg_state_29 * Leg_state_45 + 32 Leg_state_30 ^2
   - 32 Leg_state_30 * Leg_state_46 + 32 Leg_state_31 ^2
   - 32 Leg_state_31 * Leg_state_47 + 32 Leg_state_32 ^2
   - 32 Leg_state_32 * Leg_state_48 + 32 Leg_state_33 ^2
   - 32 Leg_state_33 * Leg_state_49 + 32 Leg_state_34 ^2
   - 32 Leg_state_34 * Leg_state_50 + 32 Leg_state_35 ^2
   - 32 Leg_state_35 * Leg_state_51 + 32 Leg_state_36 ^2
   - 32 Leg_state_36 * Leg_state_52 + 32 Leg_state_37 ^2
   - 32 Leg_state_37 * Leg_state_53 + 32 Leg_state_38 ^2
   - 32 Leg_state_38 * Leg_state_54 + 32 Leg_state_39 ^2
   - 32 Leg_state_39 * Leg_state_55 + 32 Leg_state_40 ^2
   - 32 Leg_state_40 * Leg_state_56 + 32 Leg_state_41 ^2
   - 32 Leg_state_41 * Leg_state_57 + 32 Leg_state_42 ^2
   - 32 Leg_state_42 * Leg_state_58 + 32 Leg_state_43 ^2
   - 32 Leg_state_43 * Leg_state_59 + 32 Leg_state_44 ^2
   - 32 Leg_state_44 * Leg_state_60 + 32 Leg_state_45 ^2
   - 32 Leg_state_45 * Leg_state_61 + 32 Leg_state_46 ^2
   - 32 Leg_state_46 * Leg_state_62 + 32 Leg_state_47 ^2
   - 32 Leg_state_47 * Leg_state_63 + 32 Leg_state_48 ^2
   - 32 Leg_state_48 * Leg_state_64 + 32 Leg_state_49 ^2
   - 32 Leg_state_49 * Leg_state_65 + 32 Leg_state_50 ^2
   - 32 Leg_state_50 * Leg_state_66 + 32 Leg_state_51 ^2
   - 32 Leg_state_51 * Leg_state_67 + 32 Leg_state_52 ^2
   - 32 Leg_state_52 * Leg_state_68 + 32 Leg_state_53 ^2
   - 32 Leg_state_53 * Leg_state_69 + 32 Leg_state_54 ^2
   - 32 Leg_state_54 * Leg_state_70 + 32 Leg_state_55 ^2
   - 32 Leg_state_55 * Leg_state_71 + 32 Leg_state_56 ^2
   - 32 Leg_state_56 * Leg_state_72 + 32 Leg_state_57 ^2
   - 32 Leg_state_57 * Leg_state_73 + 32 Leg_state_58 ^2
   - 32 Leg_state_58 * Leg_state_74 + 32 Leg_state_59 ^2
   - 32 Leg_state_59 * Leg_state_75 + 32 Leg_state_60 ^2
   - 32 Leg_state_60 * Leg_state_76 + 32 Leg_state_61 ^2
   - 32 Leg_state_61 * Leg_state_77 + 32 Leg_state_62 ^2
   - 32 Leg_state_62 * Leg_state_78 + 32 Leg_state_63 ^2
   - 32 Leg_state_63 * Leg_state_79 + 32 Leg_state_64 ^2
   - 32 Leg_state_64 * Leg_state_80 + 32 Leg_state_65 ^2
   - 32 Leg_state_65 * Leg_state_81 + 32 Leg_state_66 ^2
   - 32 Leg_state_66 * Leg_state_82 + 32 Leg_state_67 ^2
   - 32 Leg_state_67 * Leg_state_83 + 32 Leg_state_68 ^2
   - 32 Leg_state_68 * Leg_state_84 + 32 Leg_state_69 ^2
   - 32 Leg_state_69 * Leg_state_85 + 32 Leg_state_70 ^2
   - 32 Leg_state_70 * Leg_state_86 + 32 Leg_state_71 ^2
   - 32 Leg_state_71 * Leg_state_87 + 32 Leg_state_72 ^2
   - 32 Leg_state_72 * Leg_state_88 + 32 Leg_state_73 ^2
   - 32 Leg_state_73 * Leg_state_89 + 32 Leg_state_74 ^2
   - 32 Leg_state_74 * Leg_state_90 + 32 Leg_state_75 ^2
   - 32 Leg_state_75 * Leg_state_91 + 32 Leg_state_76 ^2
   - 32 Leg_state_76 * Leg_state_92 + 32 Leg_state_77 ^2
   - 32 Leg_state_77 * Leg_state_93 + 32 Leg_state_78 ^2
   - 32 Leg_state_78 * Leg_state_94 + 32 Leg_state_79 ^2
   - 32 Leg_state_79 * Leg_state_95 + 32 Leg_state_80 ^2
   - 32 Leg_state_80 * Leg_state_96 + 32 Leg_state_81 ^2
   - 32 Leg_state_81 * Leg_state_97 + 32 Leg_state_82 ^2
   - 32 Leg_state_82 * Leg_state_98 + 32 Leg_state_83 ^2
   - 32 Leg_state_83 * Leg_state_99 + 32 Leg_state_84 ^2
   - 32 Leg_state_84 * Leg_state_100 + 32 Leg_state_85 ^2
   - 32 Leg_state_85 * Leg_state_101 + 32 Leg_state_86 ^2
   - 32 Leg_state_86 * Leg_state_102 + 32 Leg_state_87 ^2
   - 32 Leg_state_87 * Leg_state_103 + 32 Leg_state_88 ^2
   - 32 Leg_state_88 * Leg_state_104 + 32 Leg_state_89 ^2
   - 32 Leg_state_89 * Leg_state_105 + 32 Leg_state_90 ^2
   - 32 Leg_state_90 * Leg_state_106 + 32 Leg_state_91 ^2
   - 32 Leg_state_91 * Leg_state_107 + 32 Leg_state_92 ^2
   - 32 Leg_state_92 * Leg_state_108 + 32 Leg_state_93 ^2
   - 32 Leg_state_93 * Leg_state_109 + 32 Leg_state_94 ^2
   - 32 Leg_state_94 * Leg_state_110 + 32 Leg_state_95 ^2
   - 32 Leg_state_95 * Leg_state_111 + 32 Leg_state_96 ^2
   - 32 Leg_state_96 * Leg_state_112 + 32 Leg_state_97 ^2
   - 32 Leg_state_97 * Leg_state_113 + 32 Leg_state_98 ^2
   - 32 Leg_state_98 * Leg_state_114 + 32 Leg_state_99 ^2
   - 32 Leg_state_99 * Leg_state_115 + 32 Leg_state_100 ^2
   - 32 Leg_state_100 * Leg_state_116 + 32 Leg_state_101 ^2
   - 32 Leg_state_101 * Leg_state_117 + 32 Leg_state_102 ^2
   - 32 Leg_state_102 * Leg_state_118 + 32 Leg_state_103 ^2
   - 32 Leg_state_103 * Leg_state_119 + 32 Leg_state_104 ^2
   - 32 Leg_state_104 * Leg_state_120 + 32 Leg_state_105 ^2
   - 32 Leg_state_105 * Leg_state_121 + 32 Leg_state_106 ^2
   - 32 Leg_state_106 * Leg_state_122 + 32 Leg_state_107 ^2
   - 32 Leg_state_107 * Leg_state_123 + 32 Leg_state_108 ^2
   - 32 Leg_state_108 * Leg_state_124 + 32 Leg_state_109 ^2
   - 32 Leg_state_109 * Leg_state_125 + 32 Leg_state_110 ^2
   - 32 Leg_state_110 * Leg_state_126 + 32 Leg_state_111 ^2
   - 32 Leg_state_111 * Leg_state_127 + 32 Leg_state_112 ^2
   - 32 Leg_state_112 * Leg_state_128 + 32 Leg_state_113 ^2
   - 32 Leg_state_113 * Leg_state_129 + 32 Leg_state_114 ^2
   - 32 Leg_state_114 * Leg_state_130 + 32 Leg_state_115 ^2
   - 32 Leg_state_115 * Leg_state_131 + 32 Leg_state_116 ^2
   - 32 Leg_state_116 * Leg_state_132 + 32 Leg_state_117 ^2
   - 32 Leg_state_117 * Leg_state_133 + 32 Leg_state_118 ^2
   - 32 Leg_state_118 * Leg_state_134 + 32 Leg_state_119 ^2
   - 32 Leg_state_119 * Leg_state_135 + 32 Leg_state_120 ^2
   - 32 Leg_state_120 * Leg_state_136 + 32 Leg_state_121 ^2
   - 32 Leg_state_121 * Leg_state_137 + 32 Leg_state_122 ^2
   - 32 Leg_state_122 * Leg_state_138 + 32 Leg_state_123 ^2
   - 32 Leg_state_123 * Leg_state_139 + 32 Leg_state_124 ^2
   - 32 Leg_state_124 * Leg_state_140 + 32 Leg_state_125 ^2
   - 32 Leg_state_125 * Leg_state_141 + 32 Leg_state_126 ^2
   - 32 Leg_state_126 * Leg_state_142 + 32 Leg_state_127 ^2
   - 32 Leg_state_127 * Leg_state_143 + 32 Leg_state_128 ^2
   - 32 Leg_state_128 * Leg_state_144 + 32 Leg_state_129 ^2
   - 32 Leg_state_129 * Leg_state_145 + 32 Leg_state_130 ^2
   - 32 Leg_state_130 * Leg_state_146 + 32 Leg_state_131 ^2
   - 32 Leg_state_131 * Leg_state_147 + 32 Leg_state_132 ^2
   - 32 Leg_state_132 * Leg_state_148 + 32 Leg_state_133 ^2
   - 32 Leg_state_133 * Leg_state_149 + 32 Leg_state_134 ^2
   - 32 Leg_state_134 * Leg_state_150 + 32 Leg_state_135 ^2
   - 32 Leg_state_135 * Leg_state_151 + 32 Leg_state_136 ^2
   - 32 Leg_state_136 * Leg_state_152 + 32 Leg_state_137 ^2
   - 32 Leg_state_137 * Leg_state_153 + 32 Leg_state_138 ^2
   - 32 Leg_state_138 * Leg_state_154 + 32 Leg_state_139 ^2
   - 32 Leg_state_139 * Leg_state_155 + 32 Leg_state_140 ^2
   - 32 Leg_state_140 * Leg_state_156 + 32 Leg_state_141 ^2
   - 32 Leg_state_141 * Leg_state_157 + 32 Leg_state_142 ^2
   - 32 Leg_state_142 * Leg_state_158 + 32 Leg_state_143 ^2
   - 32 Leg_state_143 * Leg_state_159 + 32 Leg_state_144 ^2
   - 32 Leg_state_144 * Leg_state_160 + 32 Leg_state_145 ^2
   - 32 Leg_state_145 * Leg_state_161 + 32 Leg_state_146 ^2
   - 32 Leg_state_146 * Leg_state_162 + 32 Leg_state_147 ^2
   - 32 Leg_state_147 * Leg_state_163 + 32 Leg_state_148 ^2
   - 32 Leg_state_148 * Leg_state_164 + 32 Leg_state_149 ^2
   - 32 Leg_state_149 * Leg_state_165 + 32 Leg_state_150 ^2
   - 32 Leg_state_150 * Leg_state_166 + 32 Leg_state_151 ^2
   - 32 Leg_state_151 * Leg_state_167 + 32 Leg_state_152 ^2
   - 32 Leg_state_152 * Leg_state_168 + 32 Leg_state_153 ^2
   - 32 Leg_state_153 * Leg_state_169 + 32 Leg_state_154 ^2
   - 32 Leg_state_154 * Leg_state_170 + 32 Leg_state_155 ^2
   - 32 Leg_state_155 * Leg_state_171 + 32 Leg_state_156 ^2
   - 32 Leg_state_156 * Leg_state_172 + 32 Leg_state_157 ^2
   - 32 Leg_state_157 * Leg_state_173 + 32 Leg_state_158 ^2
   - 32 Leg_state_158 * Leg_state_174 + 32 Leg_state_159 ^2
   - 32 Leg_state_159 * Leg_state_175 + 32 Leg_state_160 ^2
   - 32 Leg_state_160 * Leg_state_176 + 32 Leg_state_161 ^2
   - 32 Leg_state_161 * Leg_state_177 + 32 Leg_state_162 ^2
   - 32 Leg_state_162 * Leg_state_178 + 32 Leg_state_163 ^2
   - 32 Leg_state_163 * Leg_state_179 + 32 Leg_state_164 ^2
   - 32 Leg_state_164 * Leg_state_180 + 32 Leg_state_165 ^2
   - 32 Leg_state_165 * Leg_state_181 + 32 Leg_state_166 ^2
   - 32 Leg_state_166 * Leg_state_182 + 32 Leg_state_167 ^2
   - 32 Leg_state_167 * Leg_state_183 + 32 Leg_state_168 ^2
   - 32 Leg_state_168 * Leg_state_184 + 32 Leg_state_169 ^2
   - 32 Leg_state_169 * Leg_state_185 + 32 Leg_state_170 ^2
   - 32 Leg_state_170 * Leg_state_186 + 32 Leg_state_171 ^2
   - 32 Leg_state_171 * Leg_state_187 + 32 Leg_state_172 ^2
   - 32 Leg_state_172 * Leg_state_188 + 32 Leg_state_173 ^2
   - 32 Leg_state_173 * Leg_state_189 + 32 Leg_state_174 ^2
   - 32 Leg_state_174 * Leg_state_190 + 32 Leg_state_175 ^2
   - 32 Leg_state_175 * Leg_state_191 + 32 Leg_state_176 ^2
   - 32 Leg_state_176 * Leg_state_192 + 32 Leg_state_177 ^2
   - 32 Leg_state_177 * Leg_state_193 + 32 Leg_state_178 ^2
   - 32 Leg_state_178 * Leg_state_194 + 32 Leg_state_179 ^2
   - 32 Leg_state_179 * Leg_state_195 + 32 Leg_state_180 ^2
   - 32 Leg_state_180 * Leg_state_196 + 32 Leg_state_181 ^2
   - 32 Leg_state_181 * Leg_state_197 + 32 Leg_state_182 ^2
   - 32 Leg_state_182 * Leg_state_198 + 32 Leg_state_183 ^2
   - 32 Leg_state_183 * Leg_state_199 + 32 Leg_state_184 ^2
   - 32 Leg_state_184 * Leg_state_200 + 32 Leg_state_185 ^2
   - 32 Leg_state_185 * Leg_state_201 + 32 Leg_state_186 ^2
   - 32 Leg_state_186 * Leg_state_202 + 32 Leg_state_187 ^2
   - 32 Leg_state_187 * Leg_state_203 + 32 Leg_state_188 ^2
   - 32 Leg_state_188 * Leg_state_204 + 32 Leg_state_189 ^2
   - 32 Leg_state_189 * Leg_state_205 + 32 Leg_state_190 ^2
   - 32 Leg_state_190 * Leg_state_206 + 32 Leg_state_191 ^2
   - 32 Leg_state_191 * Leg_state_207 + 32 Leg_state_192 ^2
   - 32 Leg_state_192 * Leg_state_208 + 32 Leg_state_193 ^2
   - 32 Leg_state_193 * Leg_state_209 + 32 Leg_state_194 ^2
   - 32 Leg_state_194 * Leg_state_210 + 32 Leg_state_195 ^2
   - 32 Leg_state_195 * Leg_state_211 + 32 Leg_state_196 ^2
   - 32 Leg_state_196 * Leg_state_212 + 32 Leg_state_197 ^2
   - 32 Leg_state_197 * Leg_state_213 + 32 Leg_state_198 ^2
   - 32 Leg_state_198 * Leg_state_214 + 32 Leg_state_199 ^2
   - 32 Leg_state_199 * Leg_state_215 + 32 Leg_state_200 ^2
   - 32 Leg_state_200 * Leg_state_216 + 32 Leg_state_201 ^2
   - 32 Leg_state_201 * Leg_state_217 + 32 Leg_state_202 ^2
   - 32 Leg_state_202 * Leg_state_218 + 32 Leg_state_203 ^2
   - 32 Leg_state_203 * Leg_state_219 + 32 Leg_state_204 ^2
   - 32 Leg_state_204 * Leg_state_220 + 32 Leg_state_205 ^2
   - 32 Leg_state_205 * Leg_state_221 + 32 Leg_state_206 ^2
   - 32 Leg_state_206 * Leg_state_222 + 32 Leg_state_207 ^2
   - 32 Leg_state_207 * Leg_state_223 + 32 Leg_state_208 ^2
   - 32 Leg_state_208 * Leg_state_224 + 32 Leg_state_209 ^2
   - 32 Leg_state_209 * Leg_state_225 + 32 Leg_state_210 ^2
   - 32 Leg_state_210 * Leg_state_226 + 32 Leg_state_211 ^2
   - 32 Leg_state_211 * Leg_state_227 + 32 Leg_state_212 ^2
   - 32 Leg_state_212 * Leg_state_228 + 32 Leg_state_213 ^2
   - 32 Leg_state_213 * Leg_state_229 + 32 Leg_state_214 ^2
   - 32 Leg_state_214 * Leg_state_230 + 32 Leg_state_215 ^2
   - 32 Leg_state_215 * Leg_state_231 + 32 Leg_state_216 ^2
   - 32 Leg_state_216 * Leg_state_232 + 32 Leg_state_217 ^2
   - 32 Leg_state_217 * Leg_state_233 + 32 Leg_state_218 ^2
   - 32 Leg_state_218 * Leg_state_234 + 32 Leg_state_219 ^2
   - 32 Leg_state_219 * Leg_state_235 + 32 Leg_state_220 ^2
   - 32 Leg_state_220 * Leg_state_236 + 32 Leg_state_221 ^2
   - 32 Leg_state_221 * Leg_state_237 + 32 Leg_state_222 ^2
   - 32 Leg_state_222 * Leg_state_238 + 32 Leg_state_223 ^2
   - 32 Leg_state_223 * Leg_state_239 + 32 Leg_state_224 ^2
   - 32 Leg_state_224 * Leg_state_240 + 32 Leg_state_225 ^2
   - 32 Leg_state_225 * Leg_state_241 + 32 Leg_state_226 ^2
   - 32 Leg_state_226 * Leg_state_242 + 32 Leg_state_227 ^2
   - 32 Leg_state_227 * Leg_state_243 + 32 Leg_state_228 ^2
   - 32 Leg_state_228 * Leg_state_244 + 32 Leg_state_229 ^2
   - 32 Leg_state_229 * Leg_state_245 + 32 Leg_state_230 ^2
   - 32 Leg_state_230 * Leg_state_246 + 32 Leg_state_231 ^2
   - 32 Leg_state_231 * Leg_state_247 + 32 Leg_state_232 ^2
   - 32 Leg_state_232 * Leg_state_248 + 32 Leg_state_233 ^2
   - 32 Leg_state_233 * Leg_state_249 + 32 Leg_state_234 ^2
   - 32 Leg_state_234 * Leg_state_250 + 32 Leg_state_235 ^2
   - 32 Leg_state_235 * Leg_state_251 + 32 Leg_state_236 ^2
   - 32 Leg_state_236 * Leg_state_252 + 32 Leg_state_237 ^2
   - 32 Leg_state_237 * Leg_state_253 + 32 Leg_state_238 ^2
   - 32 Leg_state_238 * Leg_state_254 + 32 Leg_state_239 ^2
   - 32 Leg_state_239 * Leg_state_255 + 32 Leg_state_240 ^2
   - 32 Leg_state_240 * Leg_state_256 + 32 Leg_state_241 ^2
   - 32 Leg_state_241 * Leg_state_257 + 32 Leg_state_242 ^2
   - 32 Leg_state_242 * Leg_state_258 + 32 Leg_state_243 ^2
   - 32 Leg_state_243 * Leg_state_259 + 32 Leg_state_244 ^2
   - 32 Leg_state_244 * Leg_state_260 + 32 Leg_state_245 ^2
   - 32 Leg_state_245 * Leg_state_261 + 32 Leg_state_246 ^2
   - 32 Leg_state_246 * Leg_state_262 + 32 Leg_state_247 ^2
   - 32 Leg_state_247 * Leg_state_263 + 32 Leg_state_248 ^2
   - 32 Leg_state_248 * Leg_state_264 + 32 Leg_state_249 ^2
   - 32 Leg_state_249 * Leg_state_265 + 32 Leg_state_250 ^2
   - 32 Leg_state_250 * Leg_state_266 + 32 Leg_state_251 ^2
   - 32 Leg_state_251 * Leg_state_267 + 32 Leg_state_252 ^2
   - 32 Leg_state_252 * Leg_state_268 + 32 Leg_state_253 ^2
   - 32 Leg_state_253 * Leg_state_269 + 32 Leg_state_254 ^2
   - 32 Leg_state_254 * Leg_state_270 + 32 Leg_state_255 ^2
   - 32 Leg_state_255 * Leg_state_271 + 32 Leg_state_256 ^2
   - 32 Leg_state_256 * Leg_state_272 + 32 Leg_state_257 ^2
   - 32 Leg_state_257 * Leg_state_273 + 32 Leg_state_258 ^2
   - 32 Leg_state_258 * Leg_state_274 + 32 Leg_state_259 ^2
   - 32 Leg_state_259 * Leg_state_275 + 32 Leg_state_260 ^2
   - 32 Leg_state_260 * Leg_state_276 + 32 Leg_state_261 ^2
   - 32 Leg_state_261 * Leg_state_277 + 32 Leg_state_262 ^2
   - 32 Leg_state_262 * Leg_state_278 + 32 Leg_state_263 ^2
   - 32 Leg_state_263 * Leg_state_279 + 32 Leg_state_264 ^2
   - 32 Leg_state_264 * Leg_state_280 + 32 Leg_state_265 ^2
   - 32 Leg_state_265 * Leg_state_281 + 32 Leg_state_266 ^2
   - 32 Leg_state_266 * Leg_state_282 + 32 Leg_state_267 ^2
   - 32 Leg_state_267 * Leg_state_283 + 32 Leg_state_268 ^2
   - 32 Leg_state_268 * Leg_state_284 + 32 Leg_state_269 ^2
   - 32 Leg_state_269 * Leg_state_285 + 32 Leg_state_270 ^2
   - 32 Leg_state_270 * Leg_state_286 + 32 Leg_state_271 ^2
   - 32 Leg_state_271 * Leg_state_287 + 32 Leg_state_272 ^2
   - 32 Leg_state_272 * Leg_state_288 + 32 Leg_state_273 ^2
   - 32 Leg_state_273 * Leg_state_289 + 32 Leg_state_274 ^2
   - 32 Leg_state_274 * Leg_state_290 + 32 Leg_state_275 ^2
   - 32 Leg_state_275 * Leg_state_291 + 32 Leg_state_276 ^2
   - 32 Leg_state_276 * Leg_state_292 + 32 Leg_state_277 ^2
   - 32 Leg_state_277 * Leg_state_293 + 32 Leg_state_278 ^2
   - 32 Leg_state_278 * Leg_state_294 + 32 Leg_state_279 ^2
   - 32 Leg_state_279 * Leg_state_295 + 32 Leg_state_280 ^2
   - 32 Leg_state_280 * Leg_state_296 + 32 Leg_state_281 ^2
   - 32 Leg_state_281 * Leg_state_297 + 32 Leg_state_282 ^2
   - 32 Leg_state_282 * Leg_state_298 + 32 Leg_state_283 ^2
   - 32 Leg_state_283 * Leg_state_299 + 32 Leg_state_284 ^2
   - 32 Leg_state_284 * Leg_state_300 + 32 Leg_state_285 ^2
   - 32 Leg_state_285 * Leg_state_301 + 32 Leg_state_286 ^2
   - 32 Leg_state_286 * Leg_state_302 + 32 Leg_state_287 ^2
   - 32 Leg_state_287 * Leg_state_303 + 32 Leg_state_288 ^2
   - 32 Leg_state_288 * Leg_state_304 + 32 Leg_state_289 ^2
   - 32 Leg_state_289 * Leg_state_305 + 32 Leg_state_290 ^2
   - 32 Leg_state_290 * Leg_state_306 + 32 Leg_state_291 ^2
   - 32 Leg_state_291 * Leg_state_307 + 32 Leg_state_292 ^2
   - 32 Leg_state_292 * Leg_state_308 + 32 Leg_state_293 ^2
   - 32 Leg_state_293 * Leg_state_309 + 32 Leg_state_294 ^2
   - 32 Leg_state_294 * Leg_state_310 + 32 Leg_state_295 ^2
   - 32 Leg_state_295 * Leg_state_311 + 32 Leg_state_296 ^2
   - 32 Leg_state_296 * Leg_state_312 + 32 Leg_state_297 ^2
   - 32 Leg_state_297 * Leg_state_313 + 32 Leg_state_298 ^2
   - 32 Leg_state_298 * Leg_state_314 + 32 Leg_state_299 ^2
   - 32 Leg_state_299 * Leg_state_315 + 32 Leg_state_300 ^2
   - 32 Leg_state_300 * Leg_state_316 + 32 Leg_state_301 ^2
   - 32 Leg_state_301 * Leg_state_317 + 32 Leg_state_302 ^2
   - 32 Leg_state_302 * Leg_state_318 + 32 Leg_state_303 ^2
   - 32 Leg_state_303 * Leg_state_319 + 32 Leg_state_304 ^2
   - 32 Leg_state_304 * Leg_state_320 + 32 Leg_state_305 ^2
   - 32 Leg_state_305 * Leg_state_321 + 32 Leg_state_306 ^2
   - 32 Leg_state_306 * Leg_state_322 + 32 Leg_state_307 ^2
   - 32 Leg_state_307 * Leg_state_323 + 32 Leg_state_308 ^2
   - 32 Leg_state_308 * Leg_state_324 + 32 Leg_state_309 ^2
   - 32 Leg_state_309 * Leg_state_325 + 32 Leg_state_310 ^2
   - 32 Leg_state_310 * Leg_state_326 + 32 Leg_state_311 ^2
   - 32 Leg_state_311 * Leg_state_327 + 32 Leg_state_312 ^2
   - 32 Leg_state_312 * Leg_state_328 + 32 Leg_state_313 ^2
   - 32 Leg_state_313 * Leg_state_329 + 32 Leg_state_314 ^2
   - 32 Leg_state_314 * Leg_state_330 + 32 Leg_state_315 ^2
   - 32 Leg_state_315 * Leg_state_331 + 32 Leg_state_316 ^2
   - 32 Leg_state_316 * Leg_state_332 + 32 Leg_state_317 ^2
   - 32 Leg_state_317 * Leg_state_333 + 32 Leg_state_318 ^2
   - 32 Leg_state_318 * Leg_state_334 + 32 Leg_state_319 ^2
   - 32 Leg_state_319 * Leg_state_335 + 32 Leg_state_320 ^2
   - 32 Leg_state_320 * Leg_state_336 + 176 Leg_state_321 ^2
   + 176 Leg_state_322 ^2 + 176 Leg_state_323 ^2 + 176 Leg_state_324 ^2
   + 176 Leg_state_325 ^2 + 176 Leg_state_326 ^2 + 176 Leg_state_327 ^2
   + 176 Leg_state_328 ^2 + 176 Leg_state_329 ^2 + 176 Leg_state_330 ^2
   + 176 Leg_state_331 ^2 + 176 Leg_state_332 ^2 + 176 Leg_state_333 ^2
   + 176 Leg_state_334 ^2 + 176 Leg_state_335 ^2 + 176 Leg_state_336 ^2
   ] / 2 
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
 T_self_con_49: T_53 + T_54 <= 1
 T_self_con_50: T_55 + T_56 <= 1
 T_self_con_51: T_53 + T_56 <= 1
 T_self_con_52: T_54 + T_55 <= 1
 T_self_con_53: T_57 + T_58 <= 1
 T_self_con_54: T_59 + T_60 <= 1
 T_self_con_55: T_57 + T_60 <= 1
 T_self_con_56: T_58 + T_59 <= 1
 T_self_con_57: T_61 + T_62 <= 1
 T_self_con_58: T_63 + T_64 <= 1
 T_self_con_59: T_61 + T_64 <= 1
 T_self_con_60: T_62 + T_63 <= 1
 T_self_con_61: T_65 + T_66 <= 1
 T_self_con_62: T_67 + T_68 <= 1
 T_self_con_63: T_65 + T_68 <= 1
 T_self_con_64: T_66 + T_67 <= 1
 T_self_con_65: T_69 + T_70 <= 1
 T_self_con_66: T_71 + T_72 <= 1
 T_self_con_67: T_69 + T_72 <= 1
 T_self_con_68: T_70 + T_71 <= 1
 T_self_con_69: T_73 + T_74 <= 1
 T_self_con_70: T_75 + T_76 <= 1
 T_self_con_71: T_73 + T_76 <= 1
 T_self_con_72: T_74 + T_75 <= 1
 T_self_con_73: T_77 + T_78 <= 1
 T_self_con_74: T_79 + T_80 <= 1
 T_self_con_75: T_77 + T_80 <= 1
 T_self_con_76: T_78 + T_79 <= 1
 T_self_con_77: T_81 + T_82 <= 1
 T_self_con_78: T_83 + T_84 <= 1
 T_self_con_79: T_81 + T_84 <= 1
 T_self_con_80: T_82 + T_83 <= 1
 adjoint_timing_1: T_5 + T_9 <= 1
 adjoint_timing_2: T_6 + T_10 <= 1
 adjoint_timing_3: T_7 + T_11 <= 1
 adjoint_timing_4: T_8 + T_12 <= 1
 adjoint_timing_5: T_9 + T_13 <= 1
 adjoint_timing_6: T_10 + T_14 <= 1
 adjoint_timing_7: T_11 + T_15 <= 1
 adjoint_timing_8: T_12 + T_16 <= 1
 adjoint_timing_9: T_13 + T_17 <= 1
 adjoint_timing_10: T_14 + T_18 <= 1
 adjoint_timing_11: T_15 + T_19 <= 1
 adjoint_timing_12: T_16 + T_20 <= 1
 adjoint_timing_13: T_17 + T_21 <= 1
 adjoint_timing_14: T_18 + T_22 <= 1
 adjoint_timing_15: T_19 + T_23 <= 1
 adjoint_timing_16: T_20 + T_24 <= 1
 adjoint_timing_17: T_21 + T_25 <= 1
 adjoint_timing_18: T_22 + T_26 <= 1
 adjoint_timing_19: T_23 + T_27 <= 1
 adjoint_timing_20: T_24 + T_28 <= 1
 adjoint_timing_21: T_25 + T_29 <= 1
 adjoint_timing_22: T_26 + T_30 <= 1
 adjoint_timing_23: T_27 + T_31 <= 1
 adjoint_timing_24: T_28 + T_32 <= 1
 adjoint_timing_25: T_29 + T_33 <= 1
 adjoint_timing_26: T_30 + T_34 <= 1
 adjoint_timing_27: T_31 + T_35 <= 1
 adjoint_timing_28: T_32 + T_36 <= 1
 adjoint_timing_29: T_33 + T_37 <= 1
 adjoint_timing_30: T_34 + T_38 <= 1
 adjoint_timing_31: T_35 + T_39 <= 1
 adjoint_timing_32: T_36 + T_40 <= 1
 adjoint_timing_33: T_37 + T_41 <= 1
 adjoint_timing_34: T_38 + T_42 <= 1
 adjoint_timing_35: T_39 + T_43 <= 1
 adjoint_timing_36: T_40 + T_44 <= 1
 adjoint_timing_37: T_41 + T_45 <= 1
 adjoint_timing_38: T_42 + T_46 <= 1
 adjoint_timing_39: T_43 + T_47 <= 1
 adjoint_timing_40: T_44 + T_48 <= 1
 adjoint_timing_41: T_45 + T_49 <= 1
 adjoint_timing_42: T_46 + T_50 <= 1
 adjoint_timing_43: T_47 + T_51 <= 1
 adjoint_timing_44: T_48 + T_52 <= 1
 adjoint_timing_45: T_49 + T_53 <= 1
 adjoint_timing_46: T_50 + T_54 <= 1
 adjoint_timing_47: T_51 + T_55 <= 1
 adjoint_timing_48: T_52 + T_56 <= 1
 adjoint_timing_49: T_53 + T_57 <= 1
 adjoint_timing_50: T_54 + T_58 <= 1
 adjoint_timing_51: T_55 + T_59 <= 1
 adjoint_timing_52: T_56 + T_60 <= 1
 adjoint_timing_53: T_57 + T_61 <= 1
 adjoint_timing_54: T_58 + T_62 <= 1
 adjoint_timing_55: T_59 + T_63 <= 1
 adjoint_timing_56: T_60 + T_64 <= 1
 adjoint_timing_57: T_61 + T_65 <= 1
 adjoint_timing_58: T_62 + T_66 <= 1
 adjoint_timing_59: T_63 + T_67 <= 1
 adjoint_timing_60: T_64 + T_68 <= 1
 adjoint_timing_61: T_65 + T_69 <= 1
 adjoint_timing_62: T_66 + T_70 <= 1
 adjoint_timing_63: T_67 + T_71 <= 1
 adjoint_timing_64: T_68 + T_72 <= 1
 adjoint_timing_65: T_69 + T_73 <= 1
 adjoint_timing_66: T_70 + T_74 <= 1
 adjoint_timing_67: T_71 + T_75 <= 1
 adjoint_timing_68: T_72 + T_76 <= 1
 adjoint_timing_69: T_73 + T_77 <= 1
 adjoint_timing_70: T_74 + T_78 <= 1
 adjoint_timing_71: T_75 + T_79 <= 1
 adjoint_timing_72: T_76 + T_80 <= 1
 adjoint_timing_73: T_77 + T_81 <= 1
 adjoint_timing_74: T_78 + T_82 <= 1
 adjoint_timing_75: T_79 + T_83 <= 1
 adjoint_timing_76: T_80 + T_84 <= 1
 T(1,2)_AND_Leg_state__x_1: - 999999 T_5 - Leg_state_1 + Leg_state_17 <= 0
 T(1,2)_AND_Leg_state__x_2: - 999999 T_5 + Leg_state_1 - Leg_state_17 <= 0
 T(1,2)_AND_Leg_state__x_3: - COM_state_1 + 999999 T_5 + Leg_state_17
   <= 999999.6739989705
 T(1,2)_AND_Leg_state__x_4: COM_state_1 + 999999 T_5 - Leg_state_17
   <= 999998.8260010295
 T(1,2)_AND_Leg_state__y_1: - 999999 T_5 - Leg_state_2 + Leg_state_18 <= 0
 T(1,2)_AND_Leg_state__y_2: - 999999 T_5 + Leg_state_2 - Leg_state_18 <= 0
 T(1,2)_AND_Leg_state__y_3: - COM_state_2 + 999999 T_5 + Leg_state_18
   <= 999999.5149997603
 T(1,2)_AND_Leg_state__y_4: COM_state_2 + 999999 T_5 - Leg_state_18
   <= 999998.9850002397
 T(1,2)_AND_Leg_state__z_1: - 999999 T_5 - Leg_state_3 + Leg_state_19 <= 0
 T(1,2)_AND_Leg_state__z_2: - 999999 T_5 + Leg_state_3 - Leg_state_19 <= 0
 T(1,2)_AND_Leg_state__z_3: 999999 T_5 - Leg_state_3 + Leg_state_19
   <= 999999.12
 T(1,2)_AND_Leg_state__z_4: 999999 T_5 + Leg_state_3 - Leg_state_19
   <= 999999.12
 T(2,2)_AND_Leg_state__x_1: - 999999 T_6 - Leg_state_5 + Leg_state_21 <= 0
 T(2,2)_AND_Leg_state__x_2: - 999999 T_6 + Leg_state_5 - Leg_state_21 <= 0
 T(2,2)_AND_Leg_state__x_3: - COM_state_1 + 999999 T_6 + Leg_state_21
   <= 999998.8260010295
 T(2,2)_AND_Leg_state__x_4: COM_state_1 + 999999 T_6 - Leg_state_21
   <= 999999.6739989705
 T(2,2)_AND_Leg_state__y_1: - 999999 T_6 - Leg_state_6 + Leg_state_22 <= 0
 T(2,2)_AND_Leg_state__y_2: - 999999 T_6 + Leg_state_6 - Leg_state_22 <= 0
 T(2,2)_AND_Leg_state__y_3: - COM_state_2 + 999999 T_6 + Leg_state_22
   <= 999999.5149997603
 T(2,2)_AND_Leg_state__y_4: COM_state_2 + 999999 T_6 - Leg_state_22
   <= 999998.9850002397
 T(2,2)_AND_Leg_state__z_1: - 999999 T_6 - Leg_state_7 + Leg_state_23 <= 0
 T(2,2)_AND_Leg_state__z_2: - 999999 T_6 + Leg_state_7 - Leg_state_23 <= 0
 T(2,2)_AND_Leg_state__z_3: 999999 T_6 - Leg_state_7 + Leg_state_23
   <= 999999.12
 T(2,2)_AND_Leg_state__z_4: 999999 T_6 + Leg_state_7 - Leg_state_23
   <= 999999.12
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
   <= 999998.9850002397
 T(3,2)_AND_Leg_state__y_4: COM_state_2 + 999999 T_7 - Leg_state_26
   <= 999999.5149997603
 T(3,2)_AND_Leg_state__z_1: - 999999 T_7 - Leg_state_11 + Leg_state_27
   <= 0
 T(3,2)_AND_Leg_state__z_2: - 999999 T_7 + Leg_state_11 - Leg_state_27
   <= 0
 T(3,2)_AND_Leg_state__z_3: 999999 T_7 - Leg_state_11 + Leg_state_27
   <= 999999.12
 T(3,2)_AND_Leg_state__z_4: 999999 T_7 + Leg_state_11 - Leg_state_27
   <= 999999.12
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
   <= 999998.9850002397
 T(4,2)_AND_Leg_state__y_4: COM_state_2 + 999999 T_8 - Leg_state_30
   <= 999999.5149997603
 T(4,2)_AND_Leg_state__z_1: - 999999 T_8 - Leg_state_15 + Leg_state_31
   <= 0
 T(4,2)_AND_Leg_state__z_2: - 999999 T_8 + Leg_state_15 - Leg_state_31
   <= 0
 T(4,2)_AND_Leg_state__z_3: 999999 T_8 - Leg_state_15 + Leg_state_31
   <= 999999.12
 T(4,2)_AND_Leg_state__z_4: 999999 T_8 + Leg_state_15 - Leg_state_31
   <= 999999.12
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
   <= 999999.5149997603
 T(1,3)_AND_Leg_state__y_4: COM_state_6 + 999999 T_9 - Leg_state_34
   <= 999998.9850002397
 T(1,3)_AND_Leg_state__z_1: - 999999 T_9 - Leg_state_19 + Leg_state_35
   <= 0
 T(1,3)_AND_Leg_state__z_2: - 999999 T_9 + Leg_state_19 - Leg_state_35
   <= 0
 T(1,3)_AND_Leg_state__z_3: 999999 T_9 - Leg_state_19 + Leg_state_35
   <= 999999.12
 T(1,3)_AND_Leg_state__z_4: 999999 T_9 + Leg_state_19 - Leg_state_35
   <= 999999.12
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
   <= 999999.5149997603
 T(2,3)_AND_Leg_state__y_4: COM_state_6 + 999999 T_10 - Leg_state_38
   <= 999998.9850002397
 T(2,3)_AND_Leg_state__z_1: - 999999 T_10 - Leg_state_23 + Leg_state_39
   <= 0
 T(2,3)_AND_Leg_state__z_2: - 999999 T_10 + Leg_state_23 - Leg_state_39
   <= 0
 T(2,3)_AND_Leg_state__z_3: 999999 T_10 - Leg_state_23 + Leg_state_39
   <= 999999.12
 T(2,3)_AND_Leg_state__z_4: 999999 T_10 + Leg_state_23 - Leg_state_39
   <= 999999.12
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
   <= 999998.9850002397
 T(3,3)_AND_Leg_state__y_4: COM_state_6 + 999999 T_11 - Leg_state_42
   <= 999999.5149997603
 T(3,3)_AND_Leg_state__z_1: - 999999 T_11 - Leg_state_27 + Leg_state_43
   <= 0
 T(3,3)_AND_Leg_state__z_2: - 999999 T_11 + Leg_state_27 - Leg_state_43
   <= 0
 T(3,3)_AND_Leg_state__z_3: 999999 T_11 - Leg_state_27 + Leg_state_43
   <= 999999.12
 T(3,3)_AND_Leg_state__z_4: 999999 T_11 + Leg_state_27 - Leg_state_43
   <= 999999.12
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
   <= 999998.9850002397
 T(4,3)_AND_Leg_state__y_4: COM_state_6 + 999999 T_12 - Leg_state_46
   <= 999999.5149997603
 T(4,3)_AND_Leg_state__z_1: - 999999 T_12 - Leg_state_31 + Leg_state_47
   <= 0
 T(4,3)_AND_Leg_state__z_2: - 999999 T_12 + Leg_state_31 - Leg_state_47
   <= 0
 T(4,3)_AND_Leg_state__z_3: 999999 T_12 - Leg_state_31 + Leg_state_47
   <= 999999.12
 T(4,3)_AND_Leg_state__z_4: 999999 T_12 + Leg_state_31 - Leg_state_47
   <= 999999.12
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
   <= 999999.5149997603
 T(1,4)_AND_Leg_state__y_4: COM_state_10 + 999999 T_13 - Leg_state_50
   <= 999998.9850002397
 T(1,4)_AND_Leg_state__z_1: - 999999 T_13 - Leg_state_35 + Leg_state_51
   <= 0
 T(1,4)_AND_Leg_state__z_2: - 999999 T_13 + Leg_state_35 - Leg_state_51
   <= 0
 T(1,4)_AND_Leg_state__z_3: 999999 T_13 - Leg_state_35 + Leg_state_51
   <= 999999.12
 T(1,4)_AND_Leg_state__z_4: 999999 T_13 + Leg_state_35 - Leg_state_51
   <= 999999.12
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
   <= 999999.5149997603
 T(2,4)_AND_Leg_state__y_4: COM_state_10 + 999999 T_14 - Leg_state_54
   <= 999998.9850002397
 T(2,4)_AND_Leg_state__z_1: - 999999 T_14 - Leg_state_39 + Leg_state_55
   <= 0
 T(2,4)_AND_Leg_state__z_2: - 999999 T_14 + Leg_state_39 - Leg_state_55
   <= 0
 T(2,4)_AND_Leg_state__z_3: 999999 T_14 - Leg_state_39 + Leg_state_55
   <= 999999.12
 T(2,4)_AND_Leg_state__z_4: 999999 T_14 + Leg_state_39 - Leg_state_55
   <= 999999.12
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
   <= 999998.9850002397
 T(3,4)_AND_Leg_state__y_4: COM_state_10 + 999999 T_15 - Leg_state_58
   <= 999999.5149997603
 T(3,4)_AND_Leg_state__z_1: - 999999 T_15 - Leg_state_43 + Leg_state_59
   <= 0
 T(3,4)_AND_Leg_state__z_2: - 999999 T_15 + Leg_state_43 - Leg_state_59
   <= 0
 T(3,4)_AND_Leg_state__z_3: 999999 T_15 - Leg_state_43 + Leg_state_59
   <= 999999.12
 T(3,4)_AND_Leg_state__z_4: 999999 T_15 + Leg_state_43 - Leg_state_59
   <= 999999.12
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
   <= 999998.9850002397
 T(4,4)_AND_Leg_state__y_4: COM_state_10 + 999999 T_16 - Leg_state_62
   <= 999999.5149997603
 T(4,4)_AND_Leg_state__z_1: - 999999 T_16 - Leg_state_47 + Leg_state_63
   <= 0
 T(4,4)_AND_Leg_state__z_2: - 999999 T_16 + Leg_state_47 - Leg_state_63
   <= 0
 T(4,4)_AND_Leg_state__z_3: 999999 T_16 - Leg_state_47 + Leg_state_63
   <= 999999.12
 T(4,4)_AND_Leg_state__z_4: 999999 T_16 + Leg_state_47 - Leg_state_63
   <= 999999.12
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
   <= 999999.5149997603
 T(1,5)_AND_Leg_state__y_4: COM_state_14 + 999999 T_17 - Leg_state_66
   <= 999998.9850002397
 T(1,5)_AND_Leg_state__z_1: - 999999 T_17 - Leg_state_51 + Leg_state_67
   <= 0
 T(1,5)_AND_Leg_state__z_2: - 999999 T_17 + Leg_state_51 - Leg_state_67
   <= 0
 T(1,5)_AND_Leg_state__z_3: 999999 T_17 - Leg_state_51 + Leg_state_67
   <= 999999.12
 T(1,5)_AND_Leg_state__z_4: 999999 T_17 + Leg_state_51 - Leg_state_67
   <= 999999.12
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
   <= 999999.5149997603
 T(2,5)_AND_Leg_state__y_4: COM_state_14 + 999999 T_18 - Leg_state_70
   <= 999998.9850002397
 T(2,5)_AND_Leg_state__z_1: - 999999 T_18 - Leg_state_55 + Leg_state_71
   <= 0
 T(2,5)_AND_Leg_state__z_2: - 999999 T_18 + Leg_state_55 - Leg_state_71
   <= 0
 T(2,5)_AND_Leg_state__z_3: 999999 T_18 - Leg_state_55 + Leg_state_71
   <= 999999.12
 T(2,5)_AND_Leg_state__z_4: 999999 T_18 + Leg_state_55 - Leg_state_71
   <= 999999.12
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
   <= 999998.9850002397
 T(3,5)_AND_Leg_state__y_4: COM_state_14 + 999999 T_19 - Leg_state_74
   <= 999999.5149997603
 T(3,5)_AND_Leg_state__z_1: - 999999 T_19 - Leg_state_59 + Leg_state_75
   <= 0
 T(3,5)_AND_Leg_state__z_2: - 999999 T_19 + Leg_state_59 - Leg_state_75
   <= 0
 T(3,5)_AND_Leg_state__z_3: 999999 T_19 - Leg_state_59 + Leg_state_75
   <= 999999.12
 T(3,5)_AND_Leg_state__z_4: 999999 T_19 + Leg_state_59 - Leg_state_75
   <= 999999.12
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
   <= 999998.9850002397
 T(4,5)_AND_Leg_state__y_4: COM_state_14 + 999999 T_20 - Leg_state_78
   <= 999999.5149997603
 T(4,5)_AND_Leg_state__z_1: - 999999 T_20 - Leg_state_63 + Leg_state_79
   <= 0
 T(4,5)_AND_Leg_state__z_2: - 999999 T_20 + Leg_state_63 - Leg_state_79
   <= 0
 T(4,5)_AND_Leg_state__z_3: 999999 T_20 - Leg_state_63 + Leg_state_79
   <= 999999.12
 T(4,5)_AND_Leg_state__z_4: 999999 T_20 + Leg_state_63 - Leg_state_79
   <= 999999.12
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
   <= 999999.5149997603
 T(1,6)_AND_Leg_state__y_4: COM_state_18 + 999999 T_21 - Leg_state_82
   <= 999998.9850002397
 T(1,6)_AND_Leg_state__z_1: - 999999 T_21 - Leg_state_67 + Leg_state_83
   <= 0
 T(1,6)_AND_Leg_state__z_2: - 999999 T_21 + Leg_state_67 - Leg_state_83
   <= 0
 T(1,6)_AND_Leg_state__z_3: 999999 T_21 - Leg_state_67 + Leg_state_83
   <= 999999.12
 T(1,6)_AND_Leg_state__z_4: 999999 T_21 + Leg_state_67 - Leg_state_83
   <= 999999.12
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
   <= 999999.5149997603
 T(2,6)_AND_Leg_state__y_4: COM_state_18 + 999999 T_22 - Leg_state_86
   <= 999998.9850002397
 T(2,6)_AND_Leg_state__z_1: - 999999 T_22 - Leg_state_71 + Leg_state_87
   <= 0
 T(2,6)_AND_Leg_state__z_2: - 999999 T_22 + Leg_state_71 - Leg_state_87
   <= 0
 T(2,6)_AND_Leg_state__z_3: 999999 T_22 - Leg_state_71 + Leg_state_87
   <= 999999.12
 T(2,6)_AND_Leg_state__z_4: 999999 T_22 + Leg_state_71 - Leg_state_87
   <= 999999.12
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
   <= 999998.9850002397
 T(3,6)_AND_Leg_state__y_4: COM_state_18 + 999999 T_23 - Leg_state_90
   <= 999999.5149997603
 T(3,6)_AND_Leg_state__z_1: - 999999 T_23 - Leg_state_75 + Leg_state_91
   <= 0
 T(3,6)_AND_Leg_state__z_2: - 999999 T_23 + Leg_state_75 - Leg_state_91
   <= 0
 T(3,6)_AND_Leg_state__z_3: 999999 T_23 - Leg_state_75 + Leg_state_91
   <= 999999.12
 T(3,6)_AND_Leg_state__z_4: 999999 T_23 + Leg_state_75 - Leg_state_91
   <= 999999.12
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
   <= 999998.9850002397
 T(4,6)_AND_Leg_state__y_4: COM_state_18 + 999999 T_24 - Leg_state_94
   <= 999999.5149997603
 T(4,6)_AND_Leg_state__z_1: - 999999 T_24 - Leg_state_79 + Leg_state_95
   <= 0
 T(4,6)_AND_Leg_state__z_2: - 999999 T_24 + Leg_state_79 - Leg_state_95
   <= 0
 T(4,6)_AND_Leg_state__z_3: 999999 T_24 - Leg_state_79 + Leg_state_95
   <= 999999.12
 T(4,6)_AND_Leg_state__z_4: 999999 T_24 + Leg_state_79 - Leg_state_95
   <= 999999.12
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
   <= 999999.5149997603
 T(1,7)_AND_Leg_state__y_4: COM_state_22 + 999999 T_25 - Leg_state_98
   <= 999998.9850002397
 T(1,7)_AND_Leg_state__z_1: - 999999 T_25 - Leg_state_83 + Leg_state_99
   <= 0
 T(1,7)_AND_Leg_state__z_2: - 999999 T_25 + Leg_state_83 - Leg_state_99
   <= 0
 T(1,7)_AND_Leg_state__z_3: 999999 T_25 - Leg_state_83 + Leg_state_99
   <= 999999.12
 T(1,7)_AND_Leg_state__z_4: 999999 T_25 + Leg_state_83 - Leg_state_99
   <= 999999.12
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
   <= 999999.5149997603
 T(2,7)_AND_Leg_state__y_4: COM_state_22 + 999999 T_26 - Leg_state_102
   <= 999998.9850002397
 T(2,7)_AND_Leg_state__z_1: - 999999 T_26 - Leg_state_87 + Leg_state_103
   <= 0
 T(2,7)_AND_Leg_state__z_2: - 999999 T_26 + Leg_state_87 - Leg_state_103
   <= 0
 T(2,7)_AND_Leg_state__z_3: 999999 T_26 - Leg_state_87 + Leg_state_103
   <= 999999.12
 T(2,7)_AND_Leg_state__z_4: 999999 T_26 + Leg_state_87 - Leg_state_103
   <= 999999.12
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
   <= 999998.9850002397
 T(3,7)_AND_Leg_state__y_4: COM_state_22 + 999999 T_27 - Leg_state_106
   <= 999999.5149997603
 T(3,7)_AND_Leg_state__z_1: - 999999 T_27 - Leg_state_91 + Leg_state_107
   <= 0
 T(3,7)_AND_Leg_state__z_2: - 999999 T_27 + Leg_state_91 - Leg_state_107
   <= 0
 T(3,7)_AND_Leg_state__z_3: 999999 T_27 - Leg_state_91 + Leg_state_107
   <= 999999.12
 T(3,7)_AND_Leg_state__z_4: 999999 T_27 + Leg_state_91 - Leg_state_107
   <= 999999.12
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
   <= 999998.9850002397
 T(4,7)_AND_Leg_state__y_4: COM_state_22 + 999999 T_28 - Leg_state_110
   <= 999999.5149997603
 T(4,7)_AND_Leg_state__z_1: - 999999 T_28 - Leg_state_95 + Leg_state_111
   <= 0
 T(4,7)_AND_Leg_state__z_2: - 999999 T_28 + Leg_state_95 - Leg_state_111
   <= 0
 T(4,7)_AND_Leg_state__z_3: 999999 T_28 - Leg_state_95 + Leg_state_111
   <= 999999.12
 T(4,7)_AND_Leg_state__z_4: 999999 T_28 + Leg_state_95 - Leg_state_111
   <= 999999.12
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
   <= 999999.5149997603
 T(1,8)_AND_Leg_state__y_4: COM_state_26 + 999999 T_29 - Leg_state_114
   <= 999998.9850002397
 T(1,8)_AND_Leg_state__z_1: - 999999 T_29 - Leg_state_99 + Leg_state_115
   <= 0
 T(1,8)_AND_Leg_state__z_2: - 999999 T_29 + Leg_state_99 - Leg_state_115
   <= 0
 T(1,8)_AND_Leg_state__z_3: 999999 T_29 - Leg_state_99 + Leg_state_115
   <= 999999.12
 T(1,8)_AND_Leg_state__z_4: 999999 T_29 + Leg_state_99 - Leg_state_115
   <= 999999.12
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
   <= 999999.5149997603
 T(2,8)_AND_Leg_state__y_4: COM_state_26 + 999999 T_30 - Leg_state_118
   <= 999998.9850002397
 T(2,8)_AND_Leg_state__z_1: - 999999 T_30 - Leg_state_103 + Leg_state_119
   <= 0
 T(2,8)_AND_Leg_state__z_2: - 999999 T_30 + Leg_state_103 - Leg_state_119
   <= 0
 T(2,8)_AND_Leg_state__z_3: 999999 T_30 - Leg_state_103 + Leg_state_119
   <= 999999.12
 T(2,8)_AND_Leg_state__z_4: 999999 T_30 + Leg_state_103 - Leg_state_119
   <= 999999.12
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
   <= 999998.9850002397
 T(3,8)_AND_Leg_state__y_4: COM_state_26 + 999999 T_31 - Leg_state_122
   <= 999999.5149997603
 T(3,8)_AND_Leg_state__z_1: - 999999 T_31 - Leg_state_107 + Leg_state_123
   <= 0
 T(3,8)_AND_Leg_state__z_2: - 999999 T_31 + Leg_state_107 - Leg_state_123
   <= 0
 T(3,8)_AND_Leg_state__z_3: 999999 T_31 - Leg_state_107 + Leg_state_123
   <= 999999.12
 T(3,8)_AND_Leg_state__z_4: 999999 T_31 + Leg_state_107 - Leg_state_123
   <= 999999.12
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
   <= 999998.9850002397
 T(4,8)_AND_Leg_state__y_4: COM_state_26 + 999999 T_32 - Leg_state_126
   <= 999999.5149997603
 T(4,8)_AND_Leg_state__z_1: - 999999 T_32 - Leg_state_111 + Leg_state_127
   <= 0
 T(4,8)_AND_Leg_state__z_2: - 999999 T_32 + Leg_state_111 - Leg_state_127
   <= 0
 T(4,8)_AND_Leg_state__z_3: 999999 T_32 - Leg_state_111 + Leg_state_127
   <= 999999.12
 T(4,8)_AND_Leg_state__z_4: 999999 T_32 + Leg_state_111 - Leg_state_127
   <= 999999.12
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
   <= 999999.5149997603
 T(1,9)_AND_Leg_state__y_4: COM_state_30 + 999999 T_33 - Leg_state_130
   <= 999998.9850002397
 T(1,9)_AND_Leg_state__z_1: - 999999 T_33 - Leg_state_115 + Leg_state_131
   <= 0
 T(1,9)_AND_Leg_state__z_2: - 999999 T_33 + Leg_state_115 - Leg_state_131
   <= 0
 T(1,9)_AND_Leg_state__z_3: 999999 T_33 - Leg_state_115 + Leg_state_131
   <= 999999.12
 T(1,9)_AND_Leg_state__z_4: 999999 T_33 + Leg_state_115 - Leg_state_131
   <= 999999.12
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
   <= 999999.5149997603
 T(2,9)_AND_Leg_state__y_4: COM_state_30 + 999999 T_34 - Leg_state_134
   <= 999998.9850002397
 T(2,9)_AND_Leg_state__z_1: - 999999 T_34 - Leg_state_119 + Leg_state_135
   <= 0
 T(2,9)_AND_Leg_state__z_2: - 999999 T_34 + Leg_state_119 - Leg_state_135
   <= 0
 T(2,9)_AND_Leg_state__z_3: 999999 T_34 - Leg_state_119 + Leg_state_135
   <= 999999.12
 T(2,9)_AND_Leg_state__z_4: 999999 T_34 + Leg_state_119 - Leg_state_135
   <= 999999.12
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
   <= 999998.9850002397
 T(3,9)_AND_Leg_state__y_4: COM_state_30 + 999999 T_35 - Leg_state_138
   <= 999999.5149997603
 T(3,9)_AND_Leg_state__z_1: - 999999 T_35 - Leg_state_123 + Leg_state_139
   <= 0
 T(3,9)_AND_Leg_state__z_2: - 999999 T_35 + Leg_state_123 - Leg_state_139
   <= 0
 T(3,9)_AND_Leg_state__z_3: 999999 T_35 - Leg_state_123 + Leg_state_139
   <= 999999.12
 T(3,9)_AND_Leg_state__z_4: 999999 T_35 + Leg_state_123 - Leg_state_139
   <= 999999.12
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
   <= 999998.9850002397
 T(4,9)_AND_Leg_state__y_4: COM_state_30 + 999999 T_36 - Leg_state_142
   <= 999999.5149997603
 T(4,9)_AND_Leg_state__z_1: - 999999 T_36 - Leg_state_127 + Leg_state_143
   <= 0
 T(4,9)_AND_Leg_state__z_2: - 999999 T_36 + Leg_state_127 - Leg_state_143
   <= 0
 T(4,9)_AND_Leg_state__z_3: 999999 T_36 - Leg_state_127 + Leg_state_143
   <= 999999.12
 T(4,9)_AND_Leg_state__z_4: 999999 T_36 + Leg_state_127 - Leg_state_143
   <= 999999.12
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
   <= 999999.5149997603
 T(1,10)_AND_Leg_state__y_4: COM_state_34 + 999999 T_37 - Leg_state_146
   <= 999998.9850002397
 T(1,10)_AND_Leg_state__z_1: - 999999 T_37 - Leg_state_131 + Leg_state_147
   <= 0
 T(1,10)_AND_Leg_state__z_2: - 999999 T_37 + Leg_state_131 - Leg_state_147
   <= 0
 T(1,10)_AND_Leg_state__z_3: 999999 T_37 - Leg_state_131 + Leg_state_147
   <= 999999.12
 T(1,10)_AND_Leg_state__z_4: 999999 T_37 + Leg_state_131 - Leg_state_147
   <= 999999.12
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
   <= 999999.5149997603
 T(2,10)_AND_Leg_state__y_4: COM_state_34 + 999999 T_38 - Leg_state_150
   <= 999998.9850002397
 T(2,10)_AND_Leg_state__z_1: - 999999 T_38 - Leg_state_135 + Leg_state_151
   <= 0
 T(2,10)_AND_Leg_state__z_2: - 999999 T_38 + Leg_state_135 - Leg_state_151
   <= 0
 T(2,10)_AND_Leg_state__z_3: 999999 T_38 - Leg_state_135 + Leg_state_151
   <= 999999.12
 T(2,10)_AND_Leg_state__z_4: 999999 T_38 + Leg_state_135 - Leg_state_151
   <= 999999.12
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
   <= 999998.9850002397
 T(3,10)_AND_Leg_state__y_4: COM_state_34 + 999999 T_39 - Leg_state_154
   <= 999999.5149997603
 T(3,10)_AND_Leg_state__z_1: - 999999 T_39 - Leg_state_139 + Leg_state_155
   <= 0
 T(3,10)_AND_Leg_state__z_2: - 999999 T_39 + Leg_state_139 - Leg_state_155
   <= 0
 T(3,10)_AND_Leg_state__z_3: 999999 T_39 - Leg_state_139 + Leg_state_155
   <= 999999.12
 T(3,10)_AND_Leg_state__z_4: 999999 T_39 + Leg_state_139 - Leg_state_155
   <= 999999.12
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
   <= 999998.9850002397
 T(4,10)_AND_Leg_state__y_4: COM_state_34 + 999999 T_40 - Leg_state_158
   <= 999999.5149997603
 T(4,10)_AND_Leg_state__z_1: - 999999 T_40 - Leg_state_143 + Leg_state_159
   <= 0
 T(4,10)_AND_Leg_state__z_2: - 999999 T_40 + Leg_state_143 - Leg_state_159
   <= 0
 T(4,10)_AND_Leg_state__z_3: 999999 T_40 - Leg_state_143 + Leg_state_159
   <= 999999.12
 T(4,10)_AND_Leg_state__z_4: 999999 T_40 + Leg_state_143 - Leg_state_159
   <= 999999.12
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
   <= 999999.5149997603
 T(1,11)_AND_Leg_state__y_4: COM_state_38 + 999999 T_41 - Leg_state_162
   <= 999998.9850002397
 T(1,11)_AND_Leg_state__z_1: - 999999 T_41 - Leg_state_147 + Leg_state_163
   <= 0
 T(1,11)_AND_Leg_state__z_2: - 999999 T_41 + Leg_state_147 - Leg_state_163
   <= 0
 T(1,11)_AND_Leg_state__z_3: 999999 T_41 - Leg_state_147 + Leg_state_163
   <= 999999.12
 T(1,11)_AND_Leg_state__z_4: 999999 T_41 + Leg_state_147 - Leg_state_163
   <= 999999.12
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
   <= 999999.5149997603
 T(2,11)_AND_Leg_state__y_4: COM_state_38 + 999999 T_42 - Leg_state_166
   <= 999998.9850002397
 T(2,11)_AND_Leg_state__z_1: - 999999 T_42 - Leg_state_151 + Leg_state_167
   <= 0
 T(2,11)_AND_Leg_state__z_2: - 999999 T_42 + Leg_state_151 - Leg_state_167
   <= 0
 T(2,11)_AND_Leg_state__z_3: 999999 T_42 - Leg_state_151 + Leg_state_167
   <= 999999.12
 T(2,11)_AND_Leg_state__z_4: 999999 T_42 + Leg_state_151 - Leg_state_167
   <= 999999.12
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
   <= 999998.9850002397
 T(3,11)_AND_Leg_state__y_4: COM_state_38 + 999999 T_43 - Leg_state_170
   <= 999999.5149997603
 T(3,11)_AND_Leg_state__z_1: - 999999 T_43 - Leg_state_155 + Leg_state_171
   <= 0
 T(3,11)_AND_Leg_state__z_2: - 999999 T_43 + Leg_state_155 - Leg_state_171
   <= 0
 T(3,11)_AND_Leg_state__z_3: 999999 T_43 - Leg_state_155 + Leg_state_171
   <= 999999.12
 T(3,11)_AND_Leg_state__z_4: 999999 T_43 + Leg_state_155 - Leg_state_171
   <= 999999.12
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
   <= 999998.9850002397
 T(4,11)_AND_Leg_state__y_4: COM_state_38 + 999999 T_44 - Leg_state_174
   <= 999999.5149997603
 T(4,11)_AND_Leg_state__z_1: - 999999 T_44 - Leg_state_159 + Leg_state_175
   <= 0
 T(4,11)_AND_Leg_state__z_2: - 999999 T_44 + Leg_state_159 - Leg_state_175
   <= 0
 T(4,11)_AND_Leg_state__z_3: 999999 T_44 - Leg_state_159 + Leg_state_175
   <= 999999.12
 T(4,11)_AND_Leg_state__z_4: 999999 T_44 + Leg_state_159 - Leg_state_175
   <= 999999.12
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
   <= 999999.5149997603
 T(1,12)_AND_Leg_state__y_4: COM_state_42 + 999999 T_45 - Leg_state_178
   <= 999998.9850002397
 T(1,12)_AND_Leg_state__z_1: - 999999 T_45 - Leg_state_163 + Leg_state_179
   <= 0
 T(1,12)_AND_Leg_state__z_2: - 999999 T_45 + Leg_state_163 - Leg_state_179
   <= 0
 T(1,12)_AND_Leg_state__z_3: 999999 T_45 - Leg_state_163 + Leg_state_179
   <= 999999.12
 T(1,12)_AND_Leg_state__z_4: 999999 T_45 + Leg_state_163 - Leg_state_179
   <= 999999.12
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
   <= 999999.5149997603
 T(2,12)_AND_Leg_state__y_4: COM_state_42 + 999999 T_46 - Leg_state_182
   <= 999998.9850002397
 T(2,12)_AND_Leg_state__z_1: - 999999 T_46 - Leg_state_167 + Leg_state_183
   <= 0
 T(2,12)_AND_Leg_state__z_2: - 999999 T_46 + Leg_state_167 - Leg_state_183
   <= 0
 T(2,12)_AND_Leg_state__z_3: 999999 T_46 - Leg_state_167 + Leg_state_183
   <= 999999.12
 T(2,12)_AND_Leg_state__z_4: 999999 T_46 + Leg_state_167 - Leg_state_183
   <= 999999.12
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
   <= 999998.9850002397
 T(3,12)_AND_Leg_state__y_4: COM_state_42 + 999999 T_47 - Leg_state_186
   <= 999999.5149997603
 T(3,12)_AND_Leg_state__z_1: - 999999 T_47 - Leg_state_171 + Leg_state_187
   <= 0
 T(3,12)_AND_Leg_state__z_2: - 999999 T_47 + Leg_state_171 - Leg_state_187
   <= 0
 T(3,12)_AND_Leg_state__z_3: 999999 T_47 - Leg_state_171 + Leg_state_187
   <= 999999.12
 T(3,12)_AND_Leg_state__z_4: 999999 T_47 + Leg_state_171 - Leg_state_187
   <= 999999.12
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
   <= 999998.9850002397
 T(4,12)_AND_Leg_state__y_4: COM_state_42 + 999999 T_48 - Leg_state_190
   <= 999999.5149997603
 T(4,12)_AND_Leg_state__z_1: - 999999 T_48 - Leg_state_175 + Leg_state_191
   <= 0
 T(4,12)_AND_Leg_state__z_2: - 999999 T_48 + Leg_state_175 - Leg_state_191
   <= 0
 T(4,12)_AND_Leg_state__z_3: 999999 T_48 - Leg_state_175 + Leg_state_191
   <= 999999.12
 T(4,12)_AND_Leg_state__z_4: 999999 T_48 + Leg_state_175 - Leg_state_191
   <= 999999.12
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
   <= 999999.5149997603
 T(1,13)_AND_Leg_state__y_4: COM_state_46 + 999999 T_49 - Leg_state_194
   <= 999998.9850002397
 T(1,13)_AND_Leg_state__z_1: - 999999 T_49 - Leg_state_179 + Leg_state_195
   <= 0
 T(1,13)_AND_Leg_state__z_2: - 999999 T_49 + Leg_state_179 - Leg_state_195
   <= 0
 T(1,13)_AND_Leg_state__z_3: 999999 T_49 - Leg_state_179 + Leg_state_195
   <= 999999.12
 T(1,13)_AND_Leg_state__z_4: 999999 T_49 + Leg_state_179 - Leg_state_195
   <= 999999.12
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
   <= 999999.5149997603
 T(2,13)_AND_Leg_state__y_4: COM_state_46 + 999999 T_50 - Leg_state_198
   <= 999998.9850002397
 T(2,13)_AND_Leg_state__z_1: - 999999 T_50 - Leg_state_183 + Leg_state_199
   <= 0
 T(2,13)_AND_Leg_state__z_2: - 999999 T_50 + Leg_state_183 - Leg_state_199
   <= 0
 T(2,13)_AND_Leg_state__z_3: 999999 T_50 - Leg_state_183 + Leg_state_199
   <= 999999.12
 T(2,13)_AND_Leg_state__z_4: 999999 T_50 + Leg_state_183 - Leg_state_199
   <= 999999.12
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
   <= 999998.9850002397
 T(3,13)_AND_Leg_state__y_4: COM_state_46 + 999999 T_51 - Leg_state_202
   <= 999999.5149997603
 T(3,13)_AND_Leg_state__z_1: - 999999 T_51 - Leg_state_187 + Leg_state_203
   <= 0
 T(3,13)_AND_Leg_state__z_2: - 999999 T_51 + Leg_state_187 - Leg_state_203
   <= 0
 T(3,13)_AND_Leg_state__z_3: 999999 T_51 - Leg_state_187 + Leg_state_203
   <= 999999.12
 T(3,13)_AND_Leg_state__z_4: 999999 T_51 + Leg_state_187 - Leg_state_203
   <= 999999.12
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
   <= 999998.9850002397
 T(4,13)_AND_Leg_state__y_4: COM_state_46 + 999999 T_52 - Leg_state_206
   <= 999999.5149997603
 T(4,13)_AND_Leg_state__z_1: - 999999 T_52 - Leg_state_191 + Leg_state_207
   <= 0
 T(4,13)_AND_Leg_state__z_2: - 999999 T_52 + Leg_state_191 - Leg_state_207
   <= 0
 T(4,13)_AND_Leg_state__z_3: 999999 T_52 - Leg_state_191 + Leg_state_207
   <= 999999.12
 T(4,13)_AND_Leg_state__z_4: 999999 T_52 + Leg_state_191 - Leg_state_207
   <= 999999.12
 T(1,14)_AND_Leg_state__x_1: - 999999 T_53 - Leg_state_193 + Leg_state_209
   <= 0
 T(1,14)_AND_Leg_state__x_2: - 999999 T_53 + Leg_state_193 - Leg_state_209
   <= 0
 T(1,14)_AND_Leg_state__x_3: - COM_state_49 + 999999 T_53 + Leg_state_209
   <= 999999.6739989705
 T(1,14)_AND_Leg_state__x_4: COM_state_49 + 999999 T_53 - Leg_state_209
   <= 999998.8260010295
 T(1,14)_AND_Leg_state__y_1: - 999999 T_53 - Leg_state_194 + Leg_state_210
   <= 0
 T(1,14)_AND_Leg_state__y_2: - 999999 T_53 + Leg_state_194 - Leg_state_210
   <= 0
 T(1,14)_AND_Leg_state__y_3: - COM_state_50 + 999999 T_53 + Leg_state_210
   <= 999999.5149997603
 T(1,14)_AND_Leg_state__y_4: COM_state_50 + 999999 T_53 - Leg_state_210
   <= 999998.9850002397
 T(1,14)_AND_Leg_state__z_1: - 999999 T_53 - Leg_state_195 + Leg_state_211
   <= 0
 T(1,14)_AND_Leg_state__z_2: - 999999 T_53 + Leg_state_195 - Leg_state_211
   <= 0
 T(1,14)_AND_Leg_state__z_3: 999999 T_53 - Leg_state_195 + Leg_state_211
   <= 999999.12
 T(1,14)_AND_Leg_state__z_4: 999999 T_53 + Leg_state_195 - Leg_state_211
   <= 999999.12
 T(2,14)_AND_Leg_state__x_1: - 999999 T_54 - Leg_state_197 + Leg_state_213
   <= 0
 T(2,14)_AND_Leg_state__x_2: - 999999 T_54 + Leg_state_197 - Leg_state_213
   <= 0
 T(2,14)_AND_Leg_state__x_3: - COM_state_49 + 999999 T_54 + Leg_state_213
   <= 999998.8260010295
 T(2,14)_AND_Leg_state__x_4: COM_state_49 + 999999 T_54 - Leg_state_213
   <= 999999.6739989705
 T(2,14)_AND_Leg_state__y_1: - 999999 T_54 - Leg_state_198 + Leg_state_214
   <= 0
 T(2,14)_AND_Leg_state__y_2: - 999999 T_54 + Leg_state_198 - Leg_state_214
   <= 0
 T(2,14)_AND_Leg_state__y_3: - COM_state_50 + 999999 T_54 + Leg_state_214
   <= 999999.5149997603
 T(2,14)_AND_Leg_state__y_4: COM_state_50 + 999999 T_54 - Leg_state_214
   <= 999998.9850002397
 T(2,14)_AND_Leg_state__z_1: - 999999 T_54 - Leg_state_199 + Leg_state_215
   <= 0
 T(2,14)_AND_Leg_state__z_2: - 999999 T_54 + Leg_state_199 - Leg_state_215
   <= 0
 T(2,14)_AND_Leg_state__z_3: 999999 T_54 - Leg_state_199 + Leg_state_215
   <= 999999.12
 T(2,14)_AND_Leg_state__z_4: 999999 T_54 + Leg_state_199 - Leg_state_215
   <= 999999.12
 T(3,14)_AND_Leg_state__x_1: - 999999 T_55 - Leg_state_201 + Leg_state_217
   <= 0
 T(3,14)_AND_Leg_state__x_2: - 999999 T_55 + Leg_state_201 - Leg_state_217
   <= 0
 T(3,14)_AND_Leg_state__x_3: - COM_state_49 + 999999 T_55 + Leg_state_217
   <= 999998.8260010295
 T(3,14)_AND_Leg_state__x_4: COM_state_49 + 999999 T_55 - Leg_state_217
   <= 999999.6739989705
 T(3,14)_AND_Leg_state__y_1: - 999999 T_55 - Leg_state_202 + Leg_state_218
   <= 0
 T(3,14)_AND_Leg_state__y_2: - 999999 T_55 + Leg_state_202 - Leg_state_218
   <= 0
 T(3,14)_AND_Leg_state__y_3: - COM_state_50 + 999999 T_55 + Leg_state_218
   <= 999998.9850002397
 T(3,14)_AND_Leg_state__y_4: COM_state_50 + 999999 T_55 - Leg_state_218
   <= 999999.5149997603
 T(3,14)_AND_Leg_state__z_1: - 999999 T_55 - Leg_state_203 + Leg_state_219
   <= 0
 T(3,14)_AND_Leg_state__z_2: - 999999 T_55 + Leg_state_203 - Leg_state_219
   <= 0
 T(3,14)_AND_Leg_state__z_3: 999999 T_55 - Leg_state_203 + Leg_state_219
   <= 999999.12
 T(3,14)_AND_Leg_state__z_4: 999999 T_55 + Leg_state_203 - Leg_state_219
   <= 999999.12
 T(4,14)_AND_Leg_state__x_1: - 999999 T_56 - Leg_state_205 + Leg_state_221
   <= 0
 T(4,14)_AND_Leg_state__x_2: - 999999 T_56 + Leg_state_205 - Leg_state_221
   <= 0
 T(4,14)_AND_Leg_state__x_3: - COM_state_49 + 999999 T_56 + Leg_state_221
   <= 999999.6739989705
 T(4,14)_AND_Leg_state__x_4: COM_state_49 + 999999 T_56 - Leg_state_221
   <= 999998.8260010295
 T(4,14)_AND_Leg_state__y_1: - 999999 T_56 - Leg_state_206 + Leg_state_222
   <= 0
 T(4,14)_AND_Leg_state__y_2: - 999999 T_56 + Leg_state_206 - Leg_state_222
   <= 0
 T(4,14)_AND_Leg_state__y_3: - COM_state_50 + 999999 T_56 + Leg_state_222
   <= 999998.9850002397
 T(4,14)_AND_Leg_state__y_4: COM_state_50 + 999999 T_56 - Leg_state_222
   <= 999999.5149997603
 T(4,14)_AND_Leg_state__z_1: - 999999 T_56 - Leg_state_207 + Leg_state_223
   <= 0
 T(4,14)_AND_Leg_state__z_2: - 999999 T_56 + Leg_state_207 - Leg_state_223
   <= 0
 T(4,14)_AND_Leg_state__z_3: 999999 T_56 - Leg_state_207 + Leg_state_223
   <= 999999.12
 T(4,14)_AND_Leg_state__z_4: 999999 T_56 + Leg_state_207 - Leg_state_223
   <= 999999.12
 T(1,15)_AND_Leg_state__x_1: - 999999 T_57 - Leg_state_209 + Leg_state_225
   <= 0
 T(1,15)_AND_Leg_state__x_2: - 999999 T_57 + Leg_state_209 - Leg_state_225
   <= 0
 T(1,15)_AND_Leg_state__x_3: - COM_state_53 + 999999 T_57 + Leg_state_225
   <= 999999.6739989705
 T(1,15)_AND_Leg_state__x_4: COM_state_53 + 999999 T_57 - Leg_state_225
   <= 999998.8260010295
 T(1,15)_AND_Leg_state__y_1: - 999999 T_57 - Leg_state_210 + Leg_state_226
   <= 0
 T(1,15)_AND_Leg_state__y_2: - 999999 T_57 + Leg_state_210 - Leg_state_226
   <= 0
 T(1,15)_AND_Leg_state__y_3: - COM_state_54 + 999999 T_57 + Leg_state_226
   <= 999999.5149997603
 T(1,15)_AND_Leg_state__y_4: COM_state_54 + 999999 T_57 - Leg_state_226
   <= 999998.9850002397
 T(1,15)_AND_Leg_state__z_1: - 999999 T_57 - Leg_state_211 + Leg_state_227
   <= 0
 T(1,15)_AND_Leg_state__z_2: - 999999 T_57 + Leg_state_211 - Leg_state_227
   <= 0
 T(1,15)_AND_Leg_state__z_3: 999999 T_57 - Leg_state_211 + Leg_state_227
   <= 999999.12
 T(1,15)_AND_Leg_state__z_4: 999999 T_57 + Leg_state_211 - Leg_state_227
   <= 999999.12
 T(2,15)_AND_Leg_state__x_1: - 999999 T_58 - Leg_state_213 + Leg_state_229
   <= 0
 T(2,15)_AND_Leg_state__x_2: - 999999 T_58 + Leg_state_213 - Leg_state_229
   <= 0
 T(2,15)_AND_Leg_state__x_3: - COM_state_53 + 999999 T_58 + Leg_state_229
   <= 999998.8260010295
 T(2,15)_AND_Leg_state__x_4: COM_state_53 + 999999 T_58 - Leg_state_229
   <= 999999.6739989705
 T(2,15)_AND_Leg_state__y_1: - 999999 T_58 - Leg_state_214 + Leg_state_230
   <= 0
 T(2,15)_AND_Leg_state__y_2: - 999999 T_58 + Leg_state_214 - Leg_state_230
   <= 0
 T(2,15)_AND_Leg_state__y_3: - COM_state_54 + 999999 T_58 + Leg_state_230
   <= 999999.5149997603
 T(2,15)_AND_Leg_state__y_4: COM_state_54 + 999999 T_58 - Leg_state_230
   <= 999998.9850002397
 T(2,15)_AND_Leg_state__z_1: - 999999 T_58 - Leg_state_215 + Leg_state_231
   <= 0
 T(2,15)_AND_Leg_state__z_2: - 999999 T_58 + Leg_state_215 - Leg_state_231
   <= 0
 T(2,15)_AND_Leg_state__z_3: 999999 T_58 - Leg_state_215 + Leg_state_231
   <= 999999.12
 T(2,15)_AND_Leg_state__z_4: 999999 T_58 + Leg_state_215 - Leg_state_231
   <= 999999.12
 T(3,15)_AND_Leg_state__x_1: - 999999 T_59 - Leg_state_217 + Leg_state_233
   <= 0
 T(3,15)_AND_Leg_state__x_2: - 999999 T_59 + Leg_state_217 - Leg_state_233
   <= 0
 T(3,15)_AND_Leg_state__x_3: - COM_state_53 + 999999 T_59 + Leg_state_233
   <= 999998.8260010295
 T(3,15)_AND_Leg_state__x_4: COM_state_53 + 999999 T_59 - Leg_state_233
   <= 999999.6739989705
 T(3,15)_AND_Leg_state__y_1: - 999999 T_59 - Leg_state_218 + Leg_state_234
   <= 0
 T(3,15)_AND_Leg_state__y_2: - 999999 T_59 + Leg_state_218 - Leg_state_234
   <= 0
 T(3,15)_AND_Leg_state__y_3: - COM_state_54 + 999999 T_59 + Leg_state_234
   <= 999998.9850002397
 T(3,15)_AND_Leg_state__y_4: COM_state_54 + 999999 T_59 - Leg_state_234
   <= 999999.5149997603
 T(3,15)_AND_Leg_state__z_1: - 999999 T_59 - Leg_state_219 + Leg_state_235
   <= 0
 T(3,15)_AND_Leg_state__z_2: - 999999 T_59 + Leg_state_219 - Leg_state_235
   <= 0
 T(3,15)_AND_Leg_state__z_3: 999999 T_59 - Leg_state_219 + Leg_state_235
   <= 999999.12
 T(3,15)_AND_Leg_state__z_4: 999999 T_59 + Leg_state_219 - Leg_state_235
   <= 999999.12
 T(4,15)_AND_Leg_state__x_1: - 999999 T_60 - Leg_state_221 + Leg_state_237
   <= 0
 T(4,15)_AND_Leg_state__x_2: - 999999 T_60 + Leg_state_221 - Leg_state_237
   <= 0
 T(4,15)_AND_Leg_state__x_3: - COM_state_53 + 999999 T_60 + Leg_state_237
   <= 999999.6739989705
 T(4,15)_AND_Leg_state__x_4: COM_state_53 + 999999 T_60 - Leg_state_237
   <= 999998.8260010295
 T(4,15)_AND_Leg_state__y_1: - 999999 T_60 - Leg_state_222 + Leg_state_238
   <= 0
 T(4,15)_AND_Leg_state__y_2: - 999999 T_60 + Leg_state_222 - Leg_state_238
   <= 0
 T(4,15)_AND_Leg_state__y_3: - COM_state_54 + 999999 T_60 + Leg_state_238
   <= 999998.9850002397
 T(4,15)_AND_Leg_state__y_4: COM_state_54 + 999999 T_60 - Leg_state_238
   <= 999999.5149997603
 T(4,15)_AND_Leg_state__z_1: - 999999 T_60 - Leg_state_223 + Leg_state_239
   <= 0
 T(4,15)_AND_Leg_state__z_2: - 999999 T_60 + Leg_state_223 - Leg_state_239
   <= 0
 T(4,15)_AND_Leg_state__z_3: 999999 T_60 - Leg_state_223 + Leg_state_239
   <= 999999.12
 T(4,15)_AND_Leg_state__z_4: 999999 T_60 + Leg_state_223 - Leg_state_239
   <= 999999.12
 T(1,16)_AND_Leg_state__x_1: - 999999 T_61 - Leg_state_225 + Leg_state_241
   <= 0
 T(1,16)_AND_Leg_state__x_2: - 999999 T_61 + Leg_state_225 - Leg_state_241
   <= 0
 T(1,16)_AND_Leg_state__x_3: - COM_state_57 + 999999 T_61 + Leg_state_241
   <= 999999.6739989705
 T(1,16)_AND_Leg_state__x_4: COM_state_57 + 999999 T_61 - Leg_state_241
   <= 999998.8260010295
 T(1,16)_AND_Leg_state__y_1: - 999999 T_61 - Leg_state_226 + Leg_state_242
   <= 0
 T(1,16)_AND_Leg_state__y_2: - 999999 T_61 + Leg_state_226 - Leg_state_242
   <= 0
 T(1,16)_AND_Leg_state__y_3: - COM_state_58 + 999999 T_61 + Leg_state_242
   <= 999999.5149997603
 T(1,16)_AND_Leg_state__y_4: COM_state_58 + 999999 T_61 - Leg_state_242
   <= 999998.9850002397
 T(1,16)_AND_Leg_state__z_1: - 999999 T_61 - Leg_state_227 + Leg_state_243
   <= 0
 T(1,16)_AND_Leg_state__z_2: - 999999 T_61 + Leg_state_227 - Leg_state_243
   <= 0
 T(1,16)_AND_Leg_state__z_3: 999999 T_61 - Leg_state_227 + Leg_state_243
   <= 999999.12
 T(1,16)_AND_Leg_state__z_4: 999999 T_61 + Leg_state_227 - Leg_state_243
   <= 999999.12
 T(2,16)_AND_Leg_state__x_1: - 999999 T_62 - Leg_state_229 + Leg_state_245
   <= 0
 T(2,16)_AND_Leg_state__x_2: - 999999 T_62 + Leg_state_229 - Leg_state_245
   <= 0
 T(2,16)_AND_Leg_state__x_3: - COM_state_57 + 999999 T_62 + Leg_state_245
   <= 999998.8260010295
 T(2,16)_AND_Leg_state__x_4: COM_state_57 + 999999 T_62 - Leg_state_245
   <= 999999.6739989705
 T(2,16)_AND_Leg_state__y_1: - 999999 T_62 - Leg_state_230 + Leg_state_246
   <= 0
 T(2,16)_AND_Leg_state__y_2: - 999999 T_62 + Leg_state_230 - Leg_state_246
   <= 0
 T(2,16)_AND_Leg_state__y_3: - COM_state_58 + 999999 T_62 + Leg_state_246
   <= 999999.5149997603
 T(2,16)_AND_Leg_state__y_4: COM_state_58 + 999999 T_62 - Leg_state_246
   <= 999998.9850002397
 T(2,16)_AND_Leg_state__z_1: - 999999 T_62 - Leg_state_231 + Leg_state_247
   <= 0
 T(2,16)_AND_Leg_state__z_2: - 999999 T_62 + Leg_state_231 - Leg_state_247
   <= 0
 T(2,16)_AND_Leg_state__z_3: 999999 T_62 - Leg_state_231 + Leg_state_247
   <= 999999.12
 T(2,16)_AND_Leg_state__z_4: 999999 T_62 + Leg_state_231 - Leg_state_247
   <= 999999.12
 T(3,16)_AND_Leg_state__x_1: - 999999 T_63 - Leg_state_233 + Leg_state_249
   <= 0
 T(3,16)_AND_Leg_state__x_2: - 999999 T_63 + Leg_state_233 - Leg_state_249
   <= 0
 T(3,16)_AND_Leg_state__x_3: - COM_state_57 + 999999 T_63 + Leg_state_249
   <= 999998.8260010295
 T(3,16)_AND_Leg_state__x_4: COM_state_57 + 999999 T_63 - Leg_state_249
   <= 999999.6739989705
 T(3,16)_AND_Leg_state__y_1: - 999999 T_63 - Leg_state_234 + Leg_state_250
   <= 0
 T(3,16)_AND_Leg_state__y_2: - 999999 T_63 + Leg_state_234 - Leg_state_250
   <= 0
 T(3,16)_AND_Leg_state__y_3: - COM_state_58 + 999999 T_63 + Leg_state_250
   <= 999998.9850002397
 T(3,16)_AND_Leg_state__y_4: COM_state_58 + 999999 T_63 - Leg_state_250
   <= 999999.5149997603
 T(3,16)_AND_Leg_state__z_1: - 999999 T_63 - Leg_state_235 + Leg_state_251
   <= 0
 T(3,16)_AND_Leg_state__z_2: - 999999 T_63 + Leg_state_235 - Leg_state_251
   <= 0
 T(3,16)_AND_Leg_state__z_3: 999999 T_63 - Leg_state_235 + Leg_state_251
   <= 999999.12
 T(3,16)_AND_Leg_state__z_4: 999999 T_63 + Leg_state_235 - Leg_state_251
   <= 999999.12
 T(4,16)_AND_Leg_state__x_1: - 999999 T_64 - Leg_state_237 + Leg_state_253
   <= 0
 T(4,16)_AND_Leg_state__x_2: - 999999 T_64 + Leg_state_237 - Leg_state_253
   <= 0
 T(4,16)_AND_Leg_state__x_3: - COM_state_57 + 999999 T_64 + Leg_state_253
   <= 999999.6739989705
 T(4,16)_AND_Leg_state__x_4: COM_state_57 + 999999 T_64 - Leg_state_253
   <= 999998.8260010295
 T(4,16)_AND_Leg_state__y_1: - 999999 T_64 - Leg_state_238 + Leg_state_254
   <= 0
 T(4,16)_AND_Leg_state__y_2: - 999999 T_64 + Leg_state_238 - Leg_state_254
   <= 0
 T(4,16)_AND_Leg_state__y_3: - COM_state_58 + 999999 T_64 + Leg_state_254
   <= 999998.9850002397
 T(4,16)_AND_Leg_state__y_4: COM_state_58 + 999999 T_64 - Leg_state_254
   <= 999999.5149997603
 T(4,16)_AND_Leg_state__z_1: - 999999 T_64 - Leg_state_239 + Leg_state_255
   <= 0
 T(4,16)_AND_Leg_state__z_2: - 999999 T_64 + Leg_state_239 - Leg_state_255
   <= 0
 T(4,16)_AND_Leg_state__z_3: 999999 T_64 - Leg_state_239 + Leg_state_255
   <= 999999.12
 T(4,16)_AND_Leg_state__z_4: 999999 T_64 + Leg_state_239 - Leg_state_255
   <= 999999.12
 T(1,17)_AND_Leg_state__x_1: - 999999 T_65 - Leg_state_241 + Leg_state_257
   <= 0
 T(1,17)_AND_Leg_state__x_2: - 999999 T_65 + Leg_state_241 - Leg_state_257
   <= 0
 T(1,17)_AND_Leg_state__x_3: - COM_state_61 + 999999 T_65 + Leg_state_257
   <= 999999.6739989705
 T(1,17)_AND_Leg_state__x_4: COM_state_61 + 999999 T_65 - Leg_state_257
   <= 999998.8260010295
 T(1,17)_AND_Leg_state__y_1: - 999999 T_65 - Leg_state_242 + Leg_state_258
   <= 0
 T(1,17)_AND_Leg_state__y_2: - 999999 T_65 + Leg_state_242 - Leg_state_258
   <= 0
 T(1,17)_AND_Leg_state__y_3: - COM_state_62 + 999999 T_65 + Leg_state_258
   <= 999999.5149997603
 T(1,17)_AND_Leg_state__y_4: COM_state_62 + 999999 T_65 - Leg_state_258
   <= 999998.9850002397
 T(1,17)_AND_Leg_state__z_1: - 999999 T_65 - Leg_state_243 + Leg_state_259
   <= 0
 T(1,17)_AND_Leg_state__z_2: - 999999 T_65 + Leg_state_243 - Leg_state_259
   <= 0
 T(1,17)_AND_Leg_state__z_3: 999999 T_65 - Leg_state_243 + Leg_state_259
   <= 999999.12
 T(1,17)_AND_Leg_state__z_4: 999999 T_65 + Leg_state_243 - Leg_state_259
   <= 999999.12
 T(2,17)_AND_Leg_state__x_1: - 999999 T_66 - Leg_state_245 + Leg_state_261
   <= 0
 T(2,17)_AND_Leg_state__x_2: - 999999 T_66 + Leg_state_245 - Leg_state_261
   <= 0
 T(2,17)_AND_Leg_state__x_3: - COM_state_61 + 999999 T_66 + Leg_state_261
   <= 999998.8260010295
 T(2,17)_AND_Leg_state__x_4: COM_state_61 + 999999 T_66 - Leg_state_261
   <= 999999.6739989705
 T(2,17)_AND_Leg_state__y_1: - 999999 T_66 - Leg_state_246 + Leg_state_262
   <= 0
 T(2,17)_AND_Leg_state__y_2: - 999999 T_66 + Leg_state_246 - Leg_state_262
   <= 0
 T(2,17)_AND_Leg_state__y_3: - COM_state_62 + 999999 T_66 + Leg_state_262
   <= 999999.5149997603
 T(2,17)_AND_Leg_state__y_4: COM_state_62 + 999999 T_66 - Leg_state_262
   <= 999998.9850002397
 T(2,17)_AND_Leg_state__z_1: - 999999 T_66 - Leg_state_247 + Leg_state_263
   <= 0
 T(2,17)_AND_Leg_state__z_2: - 999999 T_66 + Leg_state_247 - Leg_state_263
   <= 0
 T(2,17)_AND_Leg_state__z_3: 999999 T_66 - Leg_state_247 + Leg_state_263
   <= 999999.12
 T(2,17)_AND_Leg_state__z_4: 999999 T_66 + Leg_state_247 - Leg_state_263
   <= 999999.12
 T(3,17)_AND_Leg_state__x_1: - 999999 T_67 - Leg_state_249 + Leg_state_265
   <= 0
 T(3,17)_AND_Leg_state__x_2: - 999999 T_67 + Leg_state_249 - Leg_state_265
   <= 0
 T(3,17)_AND_Leg_state__x_3: - COM_state_61 + 999999 T_67 + Leg_state_265
   <= 999998.8260010295
 T(3,17)_AND_Leg_state__x_4: COM_state_61 + 999999 T_67 - Leg_state_265
   <= 999999.6739989705
 T(3,17)_AND_Leg_state__y_1: - 999999 T_67 - Leg_state_250 + Leg_state_266
   <= 0
 T(3,17)_AND_Leg_state__y_2: - 999999 T_67 + Leg_state_250 - Leg_state_266
   <= 0
 T(3,17)_AND_Leg_state__y_3: - COM_state_62 + 999999 T_67 + Leg_state_266
   <= 999998.9850002397
 T(3,17)_AND_Leg_state__y_4: COM_state_62 + 999999 T_67 - Leg_state_266
   <= 999999.5149997603
 T(3,17)_AND_Leg_state__z_1: - 999999 T_67 - Leg_state_251 + Leg_state_267
   <= 0
 T(3,17)_AND_Leg_state__z_2: - 999999 T_67 + Leg_state_251 - Leg_state_267
   <= 0
 T(3,17)_AND_Leg_state__z_3: 999999 T_67 - Leg_state_251 + Leg_state_267
   <= 999999.12
 T(3,17)_AND_Leg_state__z_4: 999999 T_67 + Leg_state_251 - Leg_state_267
   <= 999999.12
 T(4,17)_AND_Leg_state__x_1: - 999999 T_68 - Leg_state_253 + Leg_state_269
   <= 0
 T(4,17)_AND_Leg_state__x_2: - 999999 T_68 + Leg_state_253 - Leg_state_269
   <= 0
 T(4,17)_AND_Leg_state__x_3: - COM_state_61 + 999999 T_68 + Leg_state_269
   <= 999999.6739989705
 T(4,17)_AND_Leg_state__x_4: COM_state_61 + 999999 T_68 - Leg_state_269
   <= 999998.8260010295
 T(4,17)_AND_Leg_state__y_1: - 999999 T_68 - Leg_state_254 + Leg_state_270
   <= 0
 T(4,17)_AND_Leg_state__y_2: - 999999 T_68 + Leg_state_254 - Leg_state_270
   <= 0
 T(4,17)_AND_Leg_state__y_3: - COM_state_62 + 999999 T_68 + Leg_state_270
   <= 999998.9850002397
 T(4,17)_AND_Leg_state__y_4: COM_state_62 + 999999 T_68 - Leg_state_270
   <= 999999.5149997603
 T(4,17)_AND_Leg_state__z_1: - 999999 T_68 - Leg_state_255 + Leg_state_271
   <= 0
 T(4,17)_AND_Leg_state__z_2: - 999999 T_68 + Leg_state_255 - Leg_state_271
   <= 0
 T(4,17)_AND_Leg_state__z_3: 999999 T_68 - Leg_state_255 + Leg_state_271
   <= 999999.12
 T(4,17)_AND_Leg_state__z_4: 999999 T_68 + Leg_state_255 - Leg_state_271
   <= 999999.12
 T(1,18)_AND_Leg_state__x_1: - 999999 T_69 - Leg_state_257 + Leg_state_273
   <= 0
 T(1,18)_AND_Leg_state__x_2: - 999999 T_69 + Leg_state_257 - Leg_state_273
   <= 0
 T(1,18)_AND_Leg_state__x_3: - COM_state_65 + 999999 T_69 + Leg_state_273
   <= 999999.6739989705
 T(1,18)_AND_Leg_state__x_4: COM_state_65 + 999999 T_69 - Leg_state_273
   <= 999998.8260010295
 T(1,18)_AND_Leg_state__y_1: - 999999 T_69 - Leg_state_258 + Leg_state_274
   <= 0
 T(1,18)_AND_Leg_state__y_2: - 999999 T_69 + Leg_state_258 - Leg_state_274
   <= 0
 T(1,18)_AND_Leg_state__y_3: - COM_state_66 + 999999 T_69 + Leg_state_274
   <= 999999.5149997603
 T(1,18)_AND_Leg_state__y_4: COM_state_66 + 999999 T_69 - Leg_state_274
   <= 999998.9850002397
 T(1,18)_AND_Leg_state__z_1: - 999999 T_69 - Leg_state_259 + Leg_state_275
   <= 0
 T(1,18)_AND_Leg_state__z_2: - 999999 T_69 + Leg_state_259 - Leg_state_275
   <= 0
 T(1,18)_AND_Leg_state__z_3: 999999 T_69 - Leg_state_259 + Leg_state_275
   <= 999999.12
 T(1,18)_AND_Leg_state__z_4: 999999 T_69 + Leg_state_259 - Leg_state_275
   <= 999999.12
 T(2,18)_AND_Leg_state__x_1: - 999999 T_70 - Leg_state_261 + Leg_state_277
   <= 0
 T(2,18)_AND_Leg_state__x_2: - 999999 T_70 + Leg_state_261 - Leg_state_277
   <= 0
 T(2,18)_AND_Leg_state__x_3: - COM_state_65 + 999999 T_70 + Leg_state_277
   <= 999998.8260010295
 T(2,18)_AND_Leg_state__x_4: COM_state_65 + 999999 T_70 - Leg_state_277
   <= 999999.6739989705
 T(2,18)_AND_Leg_state__y_1: - 999999 T_70 - Leg_state_262 + Leg_state_278
   <= 0
 T(2,18)_AND_Leg_state__y_2: - 999999 T_70 + Leg_state_262 - Leg_state_278
   <= 0
 T(2,18)_AND_Leg_state__y_3: - COM_state_66 + 999999 T_70 + Leg_state_278
   <= 999999.5149997603
 T(2,18)_AND_Leg_state__y_4: COM_state_66 + 999999 T_70 - Leg_state_278
   <= 999998.9850002397
 T(2,18)_AND_Leg_state__z_1: - 999999 T_70 - Leg_state_263 + Leg_state_279
   <= 0
 T(2,18)_AND_Leg_state__z_2: - 999999 T_70 + Leg_state_263 - Leg_state_279
   <= 0
 T(2,18)_AND_Leg_state__z_3: 999999 T_70 - Leg_state_263 + Leg_state_279
   <= 999999.12
 T(2,18)_AND_Leg_state__z_4: 999999 T_70 + Leg_state_263 - Leg_state_279
   <= 999999.12
 T(3,18)_AND_Leg_state__x_1: - 999999 T_71 - Leg_state_265 + Leg_state_281
   <= 0
 T(3,18)_AND_Leg_state__x_2: - 999999 T_71 + Leg_state_265 - Leg_state_281
   <= 0
 T(3,18)_AND_Leg_state__x_3: - COM_state_65 + 999999 T_71 + Leg_state_281
   <= 999998.8260010295
 T(3,18)_AND_Leg_state__x_4: COM_state_65 + 999999 T_71 - Leg_state_281
   <= 999999.6739989705
 T(3,18)_AND_Leg_state__y_1: - 999999 T_71 - Leg_state_266 + Leg_state_282
   <= 0
 T(3,18)_AND_Leg_state__y_2: - 999999 T_71 + Leg_state_266 - Leg_state_282
   <= 0
 T(3,18)_AND_Leg_state__y_3: - COM_state_66 + 999999 T_71 + Leg_state_282
   <= 999998.9850002397
 T(3,18)_AND_Leg_state__y_4: COM_state_66 + 999999 T_71 - Leg_state_282
   <= 999999.5149997603
 T(3,18)_AND_Leg_state__z_1: - 999999 T_71 - Leg_state_267 + Leg_state_283
   <= 0
 T(3,18)_AND_Leg_state__z_2: - 999999 T_71 + Leg_state_267 - Leg_state_283
   <= 0
 T(3,18)_AND_Leg_state__z_3: 999999 T_71 - Leg_state_267 + Leg_state_283
   <= 999999.12
 T(3,18)_AND_Leg_state__z_4: 999999 T_71 + Leg_state_267 - Leg_state_283
   <= 999999.12
 T(4,18)_AND_Leg_state__x_1: - 999999 T_72 - Leg_state_269 + Leg_state_285
   <= 0
 T(4,18)_AND_Leg_state__x_2: - 999999 T_72 + Leg_state_269 - Leg_state_285
   <= 0
 T(4,18)_AND_Leg_state__x_3: - COM_state_65 + 999999 T_72 + Leg_state_285
   <= 999999.6739989705
 T(4,18)_AND_Leg_state__x_4: COM_state_65 + 999999 T_72 - Leg_state_285
   <= 999998.8260010295
 T(4,18)_AND_Leg_state__y_1: - 999999 T_72 - Leg_state_270 + Leg_state_286
   <= 0
 T(4,18)_AND_Leg_state__y_2: - 999999 T_72 + Leg_state_270 - Leg_state_286
   <= 0
 T(4,18)_AND_Leg_state__y_3: - COM_state_66 + 999999 T_72 + Leg_state_286
   <= 999998.9850002397
 T(4,18)_AND_Leg_state__y_4: COM_state_66 + 999999 T_72 - Leg_state_286
   <= 999999.5149997603
 T(4,18)_AND_Leg_state__z_1: - 999999 T_72 - Leg_state_271 + Leg_state_287
   <= 0
 T(4,18)_AND_Leg_state__z_2: - 999999 T_72 + Leg_state_271 - Leg_state_287
   <= 0
 T(4,18)_AND_Leg_state__z_3: 999999 T_72 - Leg_state_271 + Leg_state_287
   <= 999999.12
 T(4,18)_AND_Leg_state__z_4: 999999 T_72 + Leg_state_271 - Leg_state_287
   <= 999999.12
 T(1,19)_AND_Leg_state__x_1: - 999999 T_73 - Leg_state_273 + Leg_state_289
   <= 0
 T(1,19)_AND_Leg_state__x_2: - 999999 T_73 + Leg_state_273 - Leg_state_289
   <= 0
 T(1,19)_AND_Leg_state__x_3: - COM_state_69 + 999999 T_73 + Leg_state_289
   <= 999999.6739989705
 T(1,19)_AND_Leg_state__x_4: COM_state_69 + 999999 T_73 - Leg_state_289
   <= 999998.8260010295
 T(1,19)_AND_Leg_state__y_1: - 999999 T_73 - Leg_state_274 + Leg_state_290
   <= 0
 T(1,19)_AND_Leg_state__y_2: - 999999 T_73 + Leg_state_274 - Leg_state_290
   <= 0
 T(1,19)_AND_Leg_state__y_3: - COM_state_70 + 999999 T_73 + Leg_state_290
   <= 999999.5149997603
 T(1,19)_AND_Leg_state__y_4: COM_state_70 + 999999 T_73 - Leg_state_290
   <= 999998.9850002397
 T(1,19)_AND_Leg_state__z_1: - 999999 T_73 - Leg_state_275 + Leg_state_291
   <= 0
 T(1,19)_AND_Leg_state__z_2: - 999999 T_73 + Leg_state_275 - Leg_state_291
   <= 0
 T(1,19)_AND_Leg_state__z_3: 999999 T_73 - Leg_state_275 + Leg_state_291
   <= 999999.12
 T(1,19)_AND_Leg_state__z_4: 999999 T_73 + Leg_state_275 - Leg_state_291
   <= 999999.12
 T(2,19)_AND_Leg_state__x_1: - 999999 T_74 - Leg_state_277 + Leg_state_293
   <= 0
 T(2,19)_AND_Leg_state__x_2: - 999999 T_74 + Leg_state_277 - Leg_state_293
   <= 0
 T(2,19)_AND_Leg_state__x_3: - COM_state_69 + 999999 T_74 + Leg_state_293
   <= 999998.8260010295
 T(2,19)_AND_Leg_state__x_4: COM_state_69 + 999999 T_74 - Leg_state_293
   <= 999999.6739989705
 T(2,19)_AND_Leg_state__y_1: - 999999 T_74 - Leg_state_278 + Leg_state_294
   <= 0
 T(2,19)_AND_Leg_state__y_2: - 999999 T_74 + Leg_state_278 - Leg_state_294
   <= 0
 T(2,19)_AND_Leg_state__y_3: - COM_state_70 + 999999 T_74 + Leg_state_294
   <= 999999.5149997603
 T(2,19)_AND_Leg_state__y_4: COM_state_70 + 999999 T_74 - Leg_state_294
   <= 999998.9850002397
 T(2,19)_AND_Leg_state__z_1: - 999999 T_74 - Leg_state_279 + Leg_state_295
   <= 0
 T(2,19)_AND_Leg_state__z_2: - 999999 T_74 + Leg_state_279 - Leg_state_295
   <= 0
 T(2,19)_AND_Leg_state__z_3: 999999 T_74 - Leg_state_279 + Leg_state_295
   <= 999999.12
 T(2,19)_AND_Leg_state__z_4: 999999 T_74 + Leg_state_279 - Leg_state_295
   <= 999999.12
 T(3,19)_AND_Leg_state__x_1: - 999999 T_75 - Leg_state_281 + Leg_state_297
   <= 0
 T(3,19)_AND_Leg_state__x_2: - 999999 T_75 + Leg_state_281 - Leg_state_297
   <= 0
 T(3,19)_AND_Leg_state__x_3: - COM_state_69 + 999999 T_75 + Leg_state_297
   <= 999998.8260010295
 T(3,19)_AND_Leg_state__x_4: COM_state_69 + 999999 T_75 - Leg_state_297
   <= 999999.6739989705
 T(3,19)_AND_Leg_state__y_1: - 999999 T_75 - Leg_state_282 + Leg_state_298
   <= 0
 T(3,19)_AND_Leg_state__y_2: - 999999 T_75 + Leg_state_282 - Leg_state_298
   <= 0
 T(3,19)_AND_Leg_state__y_3: - COM_state_70 + 999999 T_75 + Leg_state_298
   <= 999998.9850002397
 T(3,19)_AND_Leg_state__y_4: COM_state_70 + 999999 T_75 - Leg_state_298
   <= 999999.5149997603
 T(3,19)_AND_Leg_state__z_1: - 999999 T_75 - Leg_state_283 + Leg_state_299
   <= 0
 T(3,19)_AND_Leg_state__z_2: - 999999 T_75 + Leg_state_283 - Leg_state_299
   <= 0
 T(3,19)_AND_Leg_state__z_3: 999999 T_75 - Leg_state_283 + Leg_state_299
   <= 999999.12
 T(3,19)_AND_Leg_state__z_4: 999999 T_75 + Leg_state_283 - Leg_state_299
   <= 999999.12
 T(4,19)_AND_Leg_state__x_1: - 999999 T_76 - Leg_state_285 + Leg_state_301
   <= 0
 T(4,19)_AND_Leg_state__x_2: - 999999 T_76 + Leg_state_285 - Leg_state_301
   <= 0
 T(4,19)_AND_Leg_state__x_3: - COM_state_69 + 999999 T_76 + Leg_state_301
   <= 999999.6739989705
 T(4,19)_AND_Leg_state__x_4: COM_state_69 + 999999 T_76 - Leg_state_301
   <= 999998.8260010295
 T(4,19)_AND_Leg_state__y_1: - 999999 T_76 - Leg_state_286 + Leg_state_302
   <= 0
 T(4,19)_AND_Leg_state__y_2: - 999999 T_76 + Leg_state_286 - Leg_state_302
   <= 0
 T(4,19)_AND_Leg_state__y_3: - COM_state_70 + 999999 T_76 + Leg_state_302
   <= 999998.9850002397
 T(4,19)_AND_Leg_state__y_4: COM_state_70 + 999999 T_76 - Leg_state_302
   <= 999999.5149997603
 T(4,19)_AND_Leg_state__z_1: - 999999 T_76 - Leg_state_287 + Leg_state_303
   <= 0
 T(4,19)_AND_Leg_state__z_2: - 999999 T_76 + Leg_state_287 - Leg_state_303
   <= 0
 T(4,19)_AND_Leg_state__z_3: 999999 T_76 - Leg_state_287 + Leg_state_303
   <= 999999.12
 T(4,19)_AND_Leg_state__z_4: 999999 T_76 + Leg_state_287 - Leg_state_303
   <= 999999.12
 T(1,20)_AND_Leg_state__x_1: - 999999 T_77 - Leg_state_289 + Leg_state_305
   <= 0
 T(1,20)_AND_Leg_state__x_2: - 999999 T_77 + Leg_state_289 - Leg_state_305
   <= 0
 T(1,20)_AND_Leg_state__x_3: - COM_state_73 + 999999 T_77 + Leg_state_305
   <= 999999.6739989705
 T(1,20)_AND_Leg_state__x_4: COM_state_73 + 999999 T_77 - Leg_state_305
   <= 999998.8260010295
 T(1,20)_AND_Leg_state__y_1: - 999999 T_77 - Leg_state_290 + Leg_state_306
   <= 0
 T(1,20)_AND_Leg_state__y_2: - 999999 T_77 + Leg_state_290 - Leg_state_306
   <= 0
 T(1,20)_AND_Leg_state__y_3: - COM_state_74 + 999999 T_77 + Leg_state_306
   <= 999999.5149997603
 T(1,20)_AND_Leg_state__y_4: COM_state_74 + 999999 T_77 - Leg_state_306
   <= 999998.9850002397
 T(1,20)_AND_Leg_state__z_1: - 999999 T_77 - Leg_state_291 + Leg_state_307
   <= 0
 T(1,20)_AND_Leg_state__z_2: - 999999 T_77 + Leg_state_291 - Leg_state_307
   <= 0
 T(1,20)_AND_Leg_state__z_3: 999999 T_77 - Leg_state_291 + Leg_state_307
   <= 999999.12
 T(1,20)_AND_Leg_state__z_4: 999999 T_77 + Leg_state_291 - Leg_state_307
   <= 999999.12
 T(2,20)_AND_Leg_state__x_1: - 999999 T_78 - Leg_state_293 + Leg_state_309
   <= 0
 T(2,20)_AND_Leg_state__x_2: - 999999 T_78 + Leg_state_293 - Leg_state_309
   <= 0
 T(2,20)_AND_Leg_state__x_3: - COM_state_73 + 999999 T_78 + Leg_state_309
   <= 999998.8260010295
 T(2,20)_AND_Leg_state__x_4: COM_state_73 + 999999 T_78 - Leg_state_309
   <= 999999.6739989705
 T(2,20)_AND_Leg_state__y_1: - 999999 T_78 - Leg_state_294 + Leg_state_310
   <= 0
 T(2,20)_AND_Leg_state__y_2: - 999999 T_78 + Leg_state_294 - Leg_state_310
   <= 0
 T(2,20)_AND_Leg_state__y_3: - COM_state_74 + 999999 T_78 + Leg_state_310
   <= 999999.5149997603
 T(2,20)_AND_Leg_state__y_4: COM_state_74 + 999999 T_78 - Leg_state_310
   <= 999998.9850002397
 T(2,20)_AND_Leg_state__z_1: - 999999 T_78 - Leg_state_295 + Leg_state_311
   <= 0
 T(2,20)_AND_Leg_state__z_2: - 999999 T_78 + Leg_state_295 - Leg_state_311
   <= 0
 T(2,20)_AND_Leg_state__z_3: 999999 T_78 - Leg_state_295 + Leg_state_311
   <= 999999.12
 T(2,20)_AND_Leg_state__z_4: 999999 T_78 + Leg_state_295 - Leg_state_311
   <= 999999.12
 T(3,20)_AND_Leg_state__x_1: - 999999 T_79 - Leg_state_297 + Leg_state_313
   <= 0
 T(3,20)_AND_Leg_state__x_2: - 999999 T_79 + Leg_state_297 - Leg_state_313
   <= 0
 T(3,20)_AND_Leg_state__x_3: - COM_state_73 + 999999 T_79 + Leg_state_313
   <= 999998.8260010295
 T(3,20)_AND_Leg_state__x_4: COM_state_73 + 999999 T_79 - Leg_state_313
   <= 999999.6739989705
 T(3,20)_AND_Leg_state__y_1: - 999999 T_79 - Leg_state_298 + Leg_state_314
   <= 0
 T(3,20)_AND_Leg_state__y_2: - 999999 T_79 + Leg_state_298 - Leg_state_314
   <= 0
 T(3,20)_AND_Leg_state__y_3: - COM_state_74 + 999999 T_79 + Leg_state_314
   <= 999998.9850002397
 T(3,20)_AND_Leg_state__y_4: COM_state_74 + 999999 T_79 - Leg_state_314
   <= 999999.5149997603
 T(3,20)_AND_Leg_state__z_1: - 999999 T_79 - Leg_state_299 + Leg_state_315
   <= 0
 T(3,20)_AND_Leg_state__z_2: - 999999 T_79 + Leg_state_299 - Leg_state_315
   <= 0
 T(3,20)_AND_Leg_state__z_3: 999999 T_79 - Leg_state_299 + Leg_state_315
   <= 999999.12
 T(3,20)_AND_Leg_state__z_4: 999999 T_79 + Leg_state_299 - Leg_state_315
   <= 999999.12
 T(4,20)_AND_Leg_state__x_1: - 999999 T_80 - Leg_state_301 + Leg_state_317
   <= 0
 T(4,20)_AND_Leg_state__x_2: - 999999 T_80 + Leg_state_301 - Leg_state_317
   <= 0
 T(4,20)_AND_Leg_state__x_3: - COM_state_73 + 999999 T_80 + Leg_state_317
   <= 999999.6739989705
 T(4,20)_AND_Leg_state__x_4: COM_state_73 + 999999 T_80 - Leg_state_317
   <= 999998.8260010295
 T(4,20)_AND_Leg_state__y_1: - 999999 T_80 - Leg_state_302 + Leg_state_318
   <= 0
 T(4,20)_AND_Leg_state__y_2: - 999999 T_80 + Leg_state_302 - Leg_state_318
   <= 0
 T(4,20)_AND_Leg_state__y_3: - COM_state_74 + 999999 T_80 + Leg_state_318
   <= 999998.9850002397
 T(4,20)_AND_Leg_state__y_4: COM_state_74 + 999999 T_80 - Leg_state_318
   <= 999999.5149997603
 T(4,20)_AND_Leg_state__z_1: - 999999 T_80 - Leg_state_303 + Leg_state_319
   <= 0
 T(4,20)_AND_Leg_state__z_2: - 999999 T_80 + Leg_state_303 - Leg_state_319
   <= 0
 T(4,20)_AND_Leg_state__z_3: 999999 T_80 - Leg_state_303 + Leg_state_319
   <= 999999.12
 T(4,20)_AND_Leg_state__z_4: 999999 T_80 + Leg_state_303 - Leg_state_319
   <= 999999.12
 T(1,21)_AND_Leg_state__x_1: - 999999 T_81 - Leg_state_305 + Leg_state_321
   <= 0
 T(1,21)_AND_Leg_state__x_2: - 999999 T_81 + Leg_state_305 - Leg_state_321
   <= 0
 T(1,21)_AND_Leg_state__x_3: - COM_state_77 + 999999 T_81 + Leg_state_321
   <= 999999.6739989705
 T(1,21)_AND_Leg_state__x_4: COM_state_77 + 999999 T_81 - Leg_state_321
   <= 999998.8260010295
 T(1,21)_AND_Leg_state__y_1: - 999999 T_81 - Leg_state_306 + Leg_state_322
   <= 0
 T(1,21)_AND_Leg_state__y_2: - 999999 T_81 + Leg_state_306 - Leg_state_322
   <= 0
 T(1,21)_AND_Leg_state__y_3: - COM_state_78 + 999999 T_81 + Leg_state_322
   <= 999999.5149997603
 T(1,21)_AND_Leg_state__y_4: COM_state_78 + 999999 T_81 - Leg_state_322
   <= 999998.9850002397
 T(1,21)_AND_Leg_state__z_1: - 999999 T_81 - Leg_state_307 + Leg_state_323
   <= 0
 T(1,21)_AND_Leg_state__z_2: - 999999 T_81 + Leg_state_307 - Leg_state_323
   <= 0
 T(1,21)_AND_Leg_state__z_3: 999999 T_81 - Leg_state_307 + Leg_state_323
   <= 999999.12
 T(1,21)_AND_Leg_state__z_4: 999999 T_81 + Leg_state_307 - Leg_state_323
   <= 999999.12
 T(2,21)_AND_Leg_state__x_1: - 999999 T_82 - Leg_state_309 + Leg_state_325
   <= 0
 T(2,21)_AND_Leg_state__x_2: - 999999 T_82 + Leg_state_309 - Leg_state_325
   <= 0
 T(2,21)_AND_Leg_state__x_3: - COM_state_77 + 999999 T_82 + Leg_state_325
   <= 999998.8260010295
 T(2,21)_AND_Leg_state__x_4: COM_state_77 + 999999 T_82 - Leg_state_325
   <= 999999.6739989705
 T(2,21)_AND_Leg_state__y_1: - 999999 T_82 - Leg_state_310 + Leg_state_326
   <= 0
 T(2,21)_AND_Leg_state__y_2: - 999999 T_82 + Leg_state_310 - Leg_state_326
   <= 0
 T(2,21)_AND_Leg_state__y_3: - COM_state_78 + 999999 T_82 + Leg_state_326
   <= 999999.5149997603
 T(2,21)_AND_Leg_state__y_4: COM_state_78 + 999999 T_82 - Leg_state_326
   <= 999998.9850002397
 T(2,21)_AND_Leg_state__z_1: - 999999 T_82 - Leg_state_311 + Leg_state_327
   <= 0
 T(2,21)_AND_Leg_state__z_2: - 999999 T_82 + Leg_state_311 - Leg_state_327
   <= 0
 T(2,21)_AND_Leg_state__z_3: 999999 T_82 - Leg_state_311 + Leg_state_327
   <= 999999.12
 T(2,21)_AND_Leg_state__z_4: 999999 T_82 + Leg_state_311 - Leg_state_327
   <= 999999.12
 T(3,21)_AND_Leg_state__x_1: - 999999 T_83 - Leg_state_313 + Leg_state_329
   <= 0
 T(3,21)_AND_Leg_state__x_2: - 999999 T_83 + Leg_state_313 - Leg_state_329
   <= 0
 T(3,21)_AND_Leg_state__x_3: - COM_state_77 + 999999 T_83 + Leg_state_329
   <= 999998.8260010295
 T(3,21)_AND_Leg_state__x_4: COM_state_77 + 999999 T_83 - Leg_state_329
   <= 999999.6739989705
 T(3,21)_AND_Leg_state__y_1: - 999999 T_83 - Leg_state_314 + Leg_state_330
   <= 0
 T(3,21)_AND_Leg_state__y_2: - 999999 T_83 + Leg_state_314 - Leg_state_330
   <= 0
 T(3,21)_AND_Leg_state__y_3: - COM_state_78 + 999999 T_83 + Leg_state_330
   <= 999998.9850002397
 T(3,21)_AND_Leg_state__y_4: COM_state_78 + 999999 T_83 - Leg_state_330
   <= 999999.5149997603
 T(3,21)_AND_Leg_state__z_1: - 999999 T_83 - Leg_state_315 + Leg_state_331
   <= 0
 T(3,21)_AND_Leg_state__z_2: - 999999 T_83 + Leg_state_315 - Leg_state_331
   <= 0
 T(3,21)_AND_Leg_state__z_3: 999999 T_83 - Leg_state_315 + Leg_state_331
   <= 999999.12
 T(3,21)_AND_Leg_state__z_4: 999999 T_83 + Leg_state_315 - Leg_state_331
   <= 999999.12
 T(4,21)_AND_Leg_state__x_1: - 999999 T_84 - Leg_state_317 + Leg_state_333
   <= 0
 T(4,21)_AND_Leg_state__x_2: - 999999 T_84 + Leg_state_317 - Leg_state_333
   <= 0
 T(4,21)_AND_Leg_state__x_3: - COM_state_77 + 999999 T_84 + Leg_state_333
   <= 999999.6739989705
 T(4,21)_AND_Leg_state__x_4: COM_state_77 + 999999 T_84 - Leg_state_333
   <= 999998.8260010295
 T(4,21)_AND_Leg_state__y_1: - 999999 T_84 - Leg_state_318 + Leg_state_334
   <= 0
 T(4,21)_AND_Leg_state__y_2: - 999999 T_84 + Leg_state_318 - Leg_state_334
   <= 0
 T(4,21)_AND_Leg_state__y_3: - COM_state_78 + 999999 T_84 + Leg_state_334
   <= 999998.9850002397
 T(4,21)_AND_Leg_state__y_4: COM_state_78 + 999999 T_84 - Leg_state_334
   <= 999999.5149997603
 T(4,21)_AND_Leg_state__z_1: - 999999 T_84 - Leg_state_319 + Leg_state_335
   <= 0
 T(4,21)_AND_Leg_state__z_2: - 999999 T_84 + Leg_state_319 - Leg_state_335
   <= 0
 T(4,21)_AND_Leg_state__z_3: 999999 T_84 - Leg_state_319 + Leg_state_335
   <= 999999.12
 T(4,21)_AND_Leg_state__z_4: 999999 T_84 + Leg_state_319 - Leg_state_335
   <= 999999.12
 delta_z_FLAG_Fir_1: - delta_z_2 + 999999 delta_z_flag_3 <= 999998.95
 delta_z_FLAG_Fir_2: delta_z_2 - 999999 delta_z_flag_3 <= 0.05
 delta_z_FLAG_Fir_3: - delta_z_3 + 999999 delta_z_flag_5 <= 999998.95
 delta_z_FLAG_Fir_4: delta_z_3 - 999999 delta_z_flag_5 <= 0.05
 delta_z_FLAG_Fir_5: - delta_z_4 + 999999 delta_z_flag_7 <= 999998.95
 delta_z_FLAG_Fir_6: delta_z_4 - 999999 delta_z_flag_7 <= 0.05
 delta_z_FLAG_Fir_7: - delta_z_5 + 999999 delta_z_flag_9 <= 999998.95
 delta_z_FLAG_Fir_8: delta_z_5 - 999999 delta_z_flag_9 <= 0.05
 delta_z_FLAG_Fir_9: - delta_z_6 + 999999 delta_z_flag_11 <= 999998.95
 delta_z_FLAG_Fir_10: delta_z_6 - 999999 delta_z_flag_11 <= 0.05
 delta_z_FLAG_Fir_11: - delta_z_7 + 999999 delta_z_flag_13 <= 999998.95
 delta_z_FLAG_Fir_12: delta_z_7 - 999999 delta_z_flag_13 <= 0.05
 delta_z_FLAG_Fir_13: - delta_z_8 + 999999 delta_z_flag_15 <= 999998.95
 delta_z_FLAG_Fir_14: delta_z_8 - 999999 delta_z_flag_15 <= 0.05
 delta_z_FLAG_Fir_15: - delta_z_9 + 999999 delta_z_flag_17 <= 999998.95
 delta_z_FLAG_Fir_16: delta_z_9 - 999999 delta_z_flag_17 <= 0.05
 delta_z_FLAG_Fir_17: - delta_z_10 + 999999 delta_z_flag_19 <= 999998.95
 delta_z_FLAG_Fir_18: delta_z_10 - 999999 delta_z_flag_19 <= 0.05
 delta_z_FLAG_Fir_19: - delta_z_11 + 999999 delta_z_flag_21 <= 999998.95
 delta_z_FLAG_Fir_20: delta_z_11 - 999999 delta_z_flag_21 <= 0.05
 delta_z_FLAG_Fir_21: - delta_z_12 + 999999 delta_z_flag_23 <= 999998.95
 delta_z_FLAG_Fir_22: delta_z_12 - 999999 delta_z_flag_23 <= 0.05
 delta_z_FLAG_Fir_23: - delta_z_13 + 999999 delta_z_flag_25 <= 999998.95
 delta_z_FLAG_Fir_24: delta_z_13 - 999999 delta_z_flag_25 <= 0.05
 delta_z_FLAG_Fir_25: - delta_z_14 + 999999 delta_z_flag_27 <= 999998.95
 delta_z_FLAG_Fir_26: delta_z_14 - 999999 delta_z_flag_27 <= 0.05
 delta_z_FLAG_Fir_27: - delta_z_15 + 999999 delta_z_flag_29 <= 999998.95
 delta_z_FLAG_Fir_28: delta_z_15 - 999999 delta_z_flag_29 <= 0.05
 delta_z_FLAG_Fir_29: - delta_z_16 + 999999 delta_z_flag_31 <= 999998.95
 delta_z_FLAG_Fir_30: delta_z_16 - 999999 delta_z_flag_31 <= 0.05
 delta_z_FLAG_Fir_31: - delta_z_17 + 999999 delta_z_flag_33 <= 999998.95
 delta_z_FLAG_Fir_32: delta_z_17 - 999999 delta_z_flag_33 <= 0.05
 delta_z_FLAG_Fir_33: - delta_z_18 + 999999 delta_z_flag_35 <= 999998.95
 delta_z_FLAG_Fir_34: delta_z_18 - 999999 delta_z_flag_35 <= 0.05
 delta_z_FLAG_Fir_35: - delta_z_19 + 999999 delta_z_flag_37 <= 999998.95
 delta_z_FLAG_Fir_36: delta_z_19 - 999999 delta_z_flag_37 <= 0.05
 delta_z_FLAG_Fir_37: - delta_z_20 + 999999 delta_z_flag_39 <= 999998.95
 delta_z_FLAG_Fir_38: delta_z_20 - 999999 delta_z_flag_39 <= 0.05
 delta_z_FLAG_Fir_39: - delta_z_21 + 999999 delta_z_flag_41 <= 999998.95
 delta_z_FLAG_Fir_40: delta_z_21 - 999999 delta_z_flag_41 <= 0.05
 delta_z_FLAG_Sec_1: delta_z_2 + 999999 delta_z_flag_4 <= 999998.95
 delta_z_FLAG_Sec_2: - delta_z_2 - 999999 delta_z_flag_4 <= 0.05
 delta_z_FLAG_Sec_3: delta_z_3 + 999999 delta_z_flag_6 <= 999998.95
 delta_z_FLAG_Sec_4: - delta_z_3 - 999999 delta_z_flag_6 <= 0.05
 delta_z_FLAG_Sec_5: delta_z_4 + 999999 delta_z_flag_8 <= 999998.95
 delta_z_FLAG_Sec_6: - delta_z_4 - 999999 delta_z_flag_8 <= 0.05
 delta_z_FLAG_Sec_7: delta_z_5 + 999999 delta_z_flag_10 <= 999998.95
 delta_z_FLAG_Sec_8: - delta_z_5 - 999999 delta_z_flag_10 <= 0.05
 delta_z_FLAG_Sec_9: delta_z_6 + 999999 delta_z_flag_12 <= 999998.95
 delta_z_FLAG_Sec_10: - delta_z_6 - 999999 delta_z_flag_12 <= 0.05
 delta_z_FLAG_Sec_11: delta_z_7 + 999999 delta_z_flag_14 <= 999998.95
 delta_z_FLAG_Sec_12: - delta_z_7 - 999999 delta_z_flag_14 <= 0.05
 delta_z_FLAG_Sec_13: delta_z_8 + 999999 delta_z_flag_16 <= 999998.95
 delta_z_FLAG_Sec_14: - delta_z_8 - 999999 delta_z_flag_16 <= 0.05
 delta_z_FLAG_Sec_15: delta_z_9 + 999999 delta_z_flag_18 <= 999998.95
 delta_z_FLAG_Sec_16: - delta_z_9 - 999999 delta_z_flag_18 <= 0.05
 delta_z_FLAG_Sec_17: delta_z_10 + 999999 delta_z_flag_20 <= 999998.95
 delta_z_FLAG_Sec_18: - delta_z_10 - 999999 delta_z_flag_20 <= 0.05
 delta_z_FLAG_Sec_19: delta_z_11 + 999999 delta_z_flag_22 <= 999998.95
 delta_z_FLAG_Sec_20: - delta_z_11 - 999999 delta_z_flag_22 <= 0.05
 delta_z_FLAG_Sec_21: delta_z_12 + 999999 delta_z_flag_24 <= 999998.95
 delta_z_FLAG_Sec_22: - delta_z_12 - 999999 delta_z_flag_24 <= 0.05
 delta_z_FLAG_Sec_23: delta_z_13 + 999999 delta_z_flag_26 <= 999998.95
 delta_z_FLAG_Sec_24: - delta_z_13 - 999999 delta_z_flag_26 <= 0.05
 delta_z_FLAG_Sec_25: delta_z_14 + 999999 delta_z_flag_28 <= 999998.95
 delta_z_FLAG_Sec_26: - delta_z_14 - 999999 delta_z_flag_28 <= 0.05
 delta_z_FLAG_Sec_27: delta_z_15 + 999999 delta_z_flag_30 <= 999998.95
 delta_z_FLAG_Sec_28: - delta_z_15 - 999999 delta_z_flag_30 <= 0.05
 delta_z_FLAG_Sec_29: delta_z_16 + 999999 delta_z_flag_32 <= 999998.95
 delta_z_FLAG_Sec_30: - delta_z_16 - 999999 delta_z_flag_32 <= 0.05
 delta_z_FLAG_Sec_31: delta_z_17 + 999999 delta_z_flag_34 <= 999998.95
 delta_z_FLAG_Sec_32: - delta_z_17 - 999999 delta_z_flag_34 <= 0.05
 delta_z_FLAG_Sec_33: delta_z_18 + 999999 delta_z_flag_36 <= 999998.95
 delta_z_FLAG_Sec_34: - delta_z_18 - 999999 delta_z_flag_36 <= 0.05
 delta_z_FLAG_Sec_35: delta_z_19 + 999999 delta_z_flag_38 <= 999998.95
 delta_z_FLAG_Sec_36: - delta_z_19 - 999999 delta_z_flag_38 <= 0.05
 delta_z_FLAG_Sec_37: delta_z_20 + 999999 delta_z_flag_40 <= 999998.95
 delta_z_FLAG_Sec_38: - delta_z_20 - 999999 delta_z_flag_40 <= 0.05
 delta_z_FLAG_Sec_39: delta_z_21 + 999999 delta_z_flag_42 <= 999998.95
 delta_z_FLAG_Sec_40: - delta_z_21 - 999999 delta_z_flag_42 <= 0.05
 Trim_Seq_relation_1: Trim_2 - Trim_3 <= 0
 Trim_Seq_relation_2: Trim_3 - Trim_4 <= 0
 Trim_Seq_relation_3: Trim_4 - Trim_5 <= 0
 Trim_Seq_relation_4: Trim_5 - Trim_6 <= 0
 Trim_Seq_relation_5: Trim_6 - Trim_7 <= 0
 Trim_Seq_relation_6: Trim_7 - Trim_8 <= 0
 Trim_Seq_relation_7: Trim_8 - Trim_9 <= 0
 Trim_Seq_relation_8: Trim_9 - Trim_10 <= 0
 Trim_Seq_relation_9: Trim_10 - Trim_11 <= 0
 Trim_Seq_relation_10: Trim_11 - Trim_12 <= 0
 Trim_Seq_relation_11: Trim_12 - Trim_13 <= 0
 Trim_Seq_relation_12: Trim_13 - Trim_14 <= 0
 Trim_Seq_relation_13: Trim_14 - Trim_15 <= 0
 Trim_Seq_relation_14: Trim_15 - Trim_16 <= 0
 Trim_Seq_relation_15: Trim_16 - Trim_17 <= 0
 Trim_Seq_relation_16: Trim_17 - Trim_18 <= 0
 Trim_Seq_relation_17: Trim_18 - Trim_19 <= 0
 Trim_Seq_relation_18: Trim_19 - Trim_20 <= 0
 Trim_Seq_relation_19: Trim_20 - Trim_21 <= 0
 Trim_attribute_physics_1: T_5 + 999999 Trim_2 <= 999999
 Trim_attribute_physics_2: T_6 + 999999 Trim_2 <= 999999
 Trim_attribute_physics_3: T_7 + 999999 Trim_2 <= 999999
 Trim_attribute_physics_4: T_8 + 999999 Trim_2 <= 999999
 Trim_attribute_physics_5: T_9 + 999999 Trim_3 <= 999999
 Trim_attribute_physics_6: T_10 + 999999 Trim_3 <= 999999
 Trim_attribute_physics_7: T_11 + 999999 Trim_3 <= 999999
 Trim_attribute_physics_8: T_12 + 999999 Trim_3 <= 999999
 Trim_attribute_physics_9: T_13 + 999999 Trim_4 <= 999999
 Trim_attribute_physics_10: T_14 + 999999 Trim_4 <= 999999
 Trim_attribute_physics_11: T_15 + 999999 Trim_4 <= 999999
 Trim_attribute_physics_12: T_16 + 999999 Trim_4 <= 999999
 Trim_attribute_physics_13: T_17 + 999999 Trim_5 <= 999999
 Trim_attribute_physics_14: T_18 + 999999 Trim_5 <= 999999
 Trim_attribute_physics_15: T_19 + 999999 Trim_5 <= 999999
 Trim_attribute_physics_16: T_20 + 999999 Trim_5 <= 999999
 Trim_attribute_physics_17: T_21 + 999999 Trim_6 <= 999999
 Trim_attribute_physics_18: T_22 + 999999 Trim_6 <= 999999
 Trim_attribute_physics_19: T_23 + 999999 Trim_6 <= 999999
 Trim_attribute_physics_20: T_24 + 999999 Trim_6 <= 999999
 Trim_attribute_physics_21: T_25 + 999999 Trim_7 <= 999999
 Trim_attribute_physics_22: T_26 + 999999 Trim_7 <= 999999
 Trim_attribute_physics_23: T_27 + 999999 Trim_7 <= 999999
 Trim_attribute_physics_24: T_28 + 999999 Trim_7 <= 999999
 Trim_attribute_physics_25: T_29 + 999999 Trim_8 <= 999999
 Trim_attribute_physics_26: T_30 + 999999 Trim_8 <= 999999
 Trim_attribute_physics_27: T_31 + 999999 Trim_8 <= 999999
 Trim_attribute_physics_28: T_32 + 999999 Trim_8 <= 999999
 Trim_attribute_physics_29: T_33 + 999999 Trim_9 <= 999999
 Trim_attribute_physics_30: T_34 + 999999 Trim_9 <= 999999
 Trim_attribute_physics_31: T_35 + 999999 Trim_9 <= 999999
 Trim_attribute_physics_32: T_36 + 999999 Trim_9 <= 999999
 Trim_attribute_physics_33: T_37 + 999999 Trim_10 <= 999999
 Trim_attribute_physics_34: T_38 + 999999 Trim_10 <= 999999
 Trim_attribute_physics_35: T_39 + 999999 Trim_10 <= 999999
 Trim_attribute_physics_36: T_40 + 999999 Trim_10 <= 999999
 Trim_attribute_physics_37: T_41 + 999999 Trim_11 <= 999999
 Trim_attribute_physics_38: T_42 + 999999 Trim_11 <= 999999
 Trim_attribute_physics_39: T_43 + 999999 Trim_11 <= 999999
 Trim_attribute_physics_40: T_44 + 999999 Trim_11 <= 999999
 Trim_attribute_physics_41: T_45 + 999999 Trim_12 <= 999999
 Trim_attribute_physics_42: T_46 + 999999 Trim_12 <= 999999
 Trim_attribute_physics_43: T_47 + 999999 Trim_12 <= 999999
 Trim_attribute_physics_44: T_48 + 999999 Trim_12 <= 999999
 Trim_attribute_physics_45: T_49 + 999999 Trim_13 <= 999999
 Trim_attribute_physics_46: T_50 + 999999 Trim_13 <= 999999
 Trim_attribute_physics_47: T_51 + 999999 Trim_13 <= 999999
 Trim_attribute_physics_48: T_52 + 999999 Trim_13 <= 999999
 Trim_attribute_physics_49: T_53 + 999999 Trim_14 <= 999999
 Trim_attribute_physics_50: T_54 + 999999 Trim_14 <= 999999
 Trim_attribute_physics_51: T_55 + 999999 Trim_14 <= 999999
 Trim_attribute_physics_52: T_56 + 999999 Trim_14 <= 999999
 Trim_attribute_physics_53: T_57 + 999999 Trim_15 <= 999999
 Trim_attribute_physics_54: T_58 + 999999 Trim_15 <= 999999
 Trim_attribute_physics_55: T_59 + 999999 Trim_15 <= 999999
 Trim_attribute_physics_56: T_60 + 999999 Trim_15 <= 999999
 Trim_attribute_physics_57: T_61 + 999999 Trim_16 <= 999999
 Trim_attribute_physics_58: T_62 + 999999 Trim_16 <= 999999
 Trim_attribute_physics_59: T_63 + 999999 Trim_16 <= 999999
 Trim_attribute_physics_60: T_64 + 999999 Trim_16 <= 999999
 Trim_attribute_physics_61: T_65 + 999999 Trim_17 <= 999999
 Trim_attribute_physics_62: T_66 + 999999 Trim_17 <= 999999
 Trim_attribute_physics_63: T_67 + 999999 Trim_17 <= 999999
 Trim_attribute_physics_64: T_68 + 999999 Trim_17 <= 999999
 Trim_attribute_physics_65: T_69 + 999999 Trim_18 <= 999999
 Trim_attribute_physics_66: T_70 + 999999 Trim_18 <= 999999
 Trim_attribute_physics_67: T_71 + 999999 Trim_18 <= 999999
 Trim_attribute_physics_68: T_72 + 999999 Trim_18 <= 999999
 Trim_attribute_physics_69: T_73 + 999999 Trim_19 <= 999999
 Trim_attribute_physics_70: T_74 + 999999 Trim_19 <= 999999
 Trim_attribute_physics_71: T_75 + 999999 Trim_19 <= 999999
 Trim_attribute_physics_72: T_76 + 999999 Trim_19 <= 999999
 Trim_attribute_physics_73: T_77 + 999999 Trim_20 <= 999999
 Trim_attribute_physics_74: T_78 + 999999 Trim_20 <= 999999
 Trim_attribute_physics_75: T_79 + 999999 Trim_20 <= 999999
 Trim_attribute_physics_76: T_80 + 999999 Trim_20 <= 999999
 Trim_attribute_physics_77: T_81 + 999999 Trim_21 <= 999999
 Trim_attribute_physics_78: T_82 + 999999 Trim_21 <= 999999
 Trim_attribute_physics_79: T_83 + 999999 Trim_21 <= 999999
 Trim_attribute_physics_80: T_84 + 999999 Trim_21 <= 999999
 Trim_sum_con_1: Trim_1 + Trim_2 + Trim_3 + Trim_4 + Trim_5 + Trim_6
   + Trim_7 + Trim_8 + Trim_9 + Trim_10 + Trim_11 + Trim_12 + Trim_13
   + Trim_14 + Trim_15 + Trim_16 + Trim_17 + Trim_18 + Trim_19 + Trim_20
   + Trim_21 <= 17
 H_Physic_Sec_1: Leg_state_17 + 999999 H_21 <= 999999.475
 H_Physic_Sec_2: - Leg_state_17 + 999999 H_21 <= 999999.475
 H_Physic_Sec_3: Leg_state_18 + 999999 H_21 <= 999999.4
 H_Physic_Sec_4: - Leg_state_18 + 999999 H_21 <= 999999.4
 H_Physic_Sec_5: Leg_state_19 + 999999 H_21 <= 999999
 H_Physic_Sec_6: - Leg_state_19 + 999999 H_21 <= 999999
 H_Physic_Sec_7: Leg_state_17 + 999999 H_22 <= 1.0000003e+06
 H_Physic_Sec_8: - Leg_state_17 + 999999 H_22 <= 999998.5
 H_Physic_Sec_9: Leg_state_18 + 999999 H_22 <= 999999.4
 H_Physic_Sec_10: - Leg_state_18 + 999999 H_22 <= 999999.4
 H_Physic_Sec_11: Leg_state_19 + 999999 H_22 <= 999999.1
 H_Physic_Sec_12: - Leg_state_19 + 999999 H_22 <= 999998.9
 H_Physic_Sec_13: Leg_state_17 + 999999 H_23 <= 1.000000825e+06
 H_Physic_Sec_14: - Leg_state_17 + 999999 H_23 <= 999997.675
 H_Physic_Sec_15: Leg_state_18 + 999999 H_23 <= 999999.4
 H_Physic_Sec_16: - Leg_state_18 + 999999 H_23 <= 999999.4
 H_Physic_Sec_17: Leg_state_19 + 999999 H_23 <= 999999.2
 H_Physic_Sec_18: - Leg_state_19 + 999999 H_23 <= 999998.8
 H_Physic_Sec_19: Leg_state_17 + 999999 H_24 <= 1.00000135e+06
 H_Physic_Sec_20: - Leg_state_17 + 999999 H_24 <= 999997.15
 H_Physic_Sec_21: Leg_state_18 + 999999 H_24 <= 999999.4
 H_Physic_Sec_22: - Leg_state_18 + 999999 H_24 <= 999999.4
 H_Physic_Sec_23: Leg_state_19 + 999999 H_24 <= 999999.1
 H_Physic_Sec_24: - Leg_state_19 + 999999 H_24 <= 999998.9
 H_Physic_Sec_25: Leg_state_17 + 999999 H_25 <= 1.0000026e+06
 H_Physic_Sec_26: - Leg_state_17 + 999999 H_25 <= 999996.6
 H_Physic_Sec_27: Leg_state_18 + 999999 H_25 <= 999999.4
 H_Physic_Sec_28: - Leg_state_18 + 999999 H_25 <= 999999.4
 H_Physic_Sec_29: Leg_state_19 + 999999 H_25 <= 999999
 H_Physic_Sec_30: - Leg_state_19 + 999999 H_25 <= 999999
 H_Physic_Sec_31: Leg_state_21 + 999999 H_26 <= 999999.475
 H_Physic_Sec_32: - Leg_state_21 + 999999 H_26 <= 999999.475
 H_Physic_Sec_33: Leg_state_22 + 999999 H_26 <= 999999.4
 H_Physic_Sec_34: - Leg_state_22 + 999999 H_26 <= 999999.4
 H_Physic_Sec_35: Leg_state_23 + 999999 H_26 <= 999999
 H_Physic_Sec_36: - Leg_state_23 + 999999 H_26 <= 999999
 H_Physic_Sec_37: Leg_state_21 + 999999 H_27 <= 1.0000003e+06
 H_Physic_Sec_38: - Leg_state_21 + 999999 H_27 <= 999998.5
 H_Physic_Sec_39: Leg_state_22 + 999999 H_27 <= 999999.4
 H_Physic_Sec_40: - Leg_state_22 + 999999 H_27 <= 999999.4
 H_Physic_Sec_41: Leg_state_23 + 999999 H_27 <= 999999.1
 H_Physic_Sec_42: - Leg_state_23 + 999999 H_27 <= 999998.9
 H_Physic_Sec_43: Leg_state_21 + 999999 H_28 <= 1.000000825e+06
 H_Physic_Sec_44: - Leg_state_21 + 999999 H_28 <= 999997.675
 H_Physic_Sec_45: Leg_state_22 + 999999 H_28 <= 999999.4
 H_Physic_Sec_46: - Leg_state_22 + 999999 H_28 <= 999999.4
 H_Physic_Sec_47: Leg_state_23 + 999999 H_28 <= 999999.2
 H_Physic_Sec_48: - Leg_state_23 + 999999 H_28 <= 999998.8
 H_Physic_Sec_49: Leg_state_21 + 999999 H_29 <= 1.00000135e+06
 H_Physic_Sec_50: - Leg_state_21 + 999999 H_29 <= 999997.15
 H_Physic_Sec_51: Leg_state_22 + 999999 H_29 <= 999999.4
 H_Physic_Sec_52: - Leg_state_22 + 999999 H_29 <= 999999.4
 H_Physic_Sec_53: Leg_state_23 + 999999 H_29 <= 999999.1
 H_Physic_Sec_54: - Leg_state_23 + 999999 H_29 <= 999998.9
 H_Physic_Sec_55: Leg_state_21 + 999999 H_30 <= 1.0000026e+06
 H_Physic_Sec_56: - Leg_state_21 + 999999 H_30 <= 999996.6
 H_Physic_Sec_57: Leg_state_22 + 999999 H_30 <= 999999.4
 H_Physic_Sec_58: - Leg_state_22 + 999999 H_30 <= 999999.4
 H_Physic_Sec_59: Leg_state_23 + 999999 H_30 <= 999999
 H_Physic_Sec_60: - Leg_state_23 + 999999 H_30 <= 999999
 H_Physic_Sec_61: Leg_state_25 + 999999 H_31 <= 999999.475
 H_Physic_Sec_62: - Leg_state_25 + 999999 H_31 <= 999999.475
 H_Physic_Sec_63: Leg_state_26 + 999999 H_31 <= 999999.4
 H_Physic_Sec_64: - Leg_state_26 + 999999 H_31 <= 999999.4
 H_Physic_Sec_65: Leg_state_27 + 999999 H_31 <= 999999
 H_Physic_Sec_66: - Leg_state_27 + 999999 H_31 <= 999999
 H_Physic_Sec_67: Leg_state_25 + 999999 H_32 <= 1.0000003e+06
 H_Physic_Sec_68: - Leg_state_25 + 999999 H_32 <= 999998.5
 H_Physic_Sec_69: Leg_state_26 + 999999 H_32 <= 999999.4
 H_Physic_Sec_70: - Leg_state_26 + 999999 H_32 <= 999999.4
 H_Physic_Sec_71: Leg_state_27 + 999999 H_32 <= 999999.1
 H_Physic_Sec_72: - Leg_state_27 + 999999 H_32 <= 999998.9
 H_Physic_Sec_73: Leg_state_25 + 999999 H_33 <= 1.000000825e+06
 H_Physic_Sec_74: - Leg_state_25 + 999999 H_33 <= 999997.675
 H_Physic_Sec_75: Leg_state_26 + 999999 H_33 <= 999999.4
 H_Physic_Sec_76: - Leg_state_26 + 999999 H_33 <= 999999.4
 H_Physic_Sec_77: Leg_state_27 + 999999 H_33 <= 999999.2
 H_Physic_Sec_78: - Leg_state_27 + 999999 H_33 <= 999998.8
 H_Physic_Sec_79: Leg_state_25 + 999999 H_34 <= 1.00000135e+06
 H_Physic_Sec_80: - Leg_state_25 + 999999 H_34 <= 999997.15
 H_Physic_Sec_81: Leg_state_26 + 999999 H_34 <= 999999.4
 H_Physic_Sec_82: - Leg_state_26 + 999999 H_34 <= 999999.4
 H_Physic_Sec_83: Leg_state_27 + 999999 H_34 <= 999999.1
 H_Physic_Sec_84: - Leg_state_27 + 999999 H_34 <= 999998.9
 H_Physic_Sec_85: Leg_state_25 + 999999 H_35 <= 1.0000026e+06
 H_Physic_Sec_86: - Leg_state_25 + 999999 H_35 <= 999996.6
 H_Physic_Sec_87: Leg_state_26 + 999999 H_35 <= 999999.4
 H_Physic_Sec_88: - Leg_state_26 + 999999 H_35 <= 999999.4
 H_Physic_Sec_89: Leg_state_27 + 999999 H_35 <= 999999
 H_Physic_Sec_90: - Leg_state_27 + 999999 H_35 <= 999999
 H_Physic_Sec_91: Leg_state_29 + 999999 H_36 <= 999999.475
 H_Physic_Sec_92: - Leg_state_29 + 999999 H_36 <= 999999.475
 H_Physic_Sec_93: Leg_state_30 + 999999 H_36 <= 999999.4
 H_Physic_Sec_94: - Leg_state_30 + 999999 H_36 <= 999999.4
 H_Physic_Sec_95: Leg_state_31 + 999999 H_36 <= 999999
 H_Physic_Sec_96: - Leg_state_31 + 999999 H_36 <= 999999
 H_Physic_Sec_97: Leg_state_29 + 999999 H_37 <= 1.0000003e+06
 H_Physic_Sec_98: - Leg_state_29 + 999999 H_37 <= 999998.5
 H_Physic_Sec_99: Leg_state_30 + 999999 H_37 <= 999999.4
 H_Physic_Sec_100: - Leg_state_30 + 999999 H_37 <= 999999.4
 H_Physic_Sec_101: Leg_state_31 + 999999 H_37 <= 999999.1
 H_Physic_Sec_102: - Leg_state_31 + 999999 H_37 <= 999998.9
 H_Physic_Sec_103: Leg_state_29 + 999999 H_38 <= 1.000000825e+06
 H_Physic_Sec_104: - Leg_state_29 + 999999 H_38 <= 999997.675
 H_Physic_Sec_105: Leg_state_30 + 999999 H_38 <= 999999.4
 H_Physic_Sec_106: - Leg_state_30 + 999999 H_38 <= 999999.4
 H_Physic_Sec_107: Leg_state_31 + 999999 H_38 <= 999999.2
 H_Physic_Sec_108: - Leg_state_31 + 999999 H_38 <= 999998.8
 H_Physic_Sec_109: Leg_state_29 + 999999 H_39 <= 1.00000135e+06
 H_Physic_Sec_110: - Leg_state_29 + 999999 H_39 <= 999997.15
 H_Physic_Sec_111: Leg_state_30 + 999999 H_39 <= 999999.4
 H_Physic_Sec_112: - Leg_state_30 + 999999 H_39 <= 999999.4
 H_Physic_Sec_113: Leg_state_31 + 999999 H_39 <= 999999.1
 H_Physic_Sec_114: - Leg_state_31 + 999999 H_39 <= 999998.9
 H_Physic_Sec_115: Leg_state_29 + 999999 H_40 <= 1.0000026e+06
 H_Physic_Sec_116: - Leg_state_29 + 999999 H_40 <= 999996.6
 H_Physic_Sec_117: Leg_state_30 + 999999 H_40 <= 999999.4
 H_Physic_Sec_118: - Leg_state_30 + 999999 H_40 <= 999999.4
 H_Physic_Sec_119: Leg_state_31 + 999999 H_40 <= 999999
 H_Physic_Sec_120: - Leg_state_31 + 999999 H_40 <= 999999
 H_Physic_Sec_121: Leg_state_33 + 999999 H_41 <= 999999.475
 H_Physic_Sec_122: - Leg_state_33 + 999999 H_41 <= 999999.475
 H_Physic_Sec_123: Leg_state_34 + 999999 H_41 <= 999999.4
 H_Physic_Sec_124: - Leg_state_34 + 999999 H_41 <= 999999.4
 H_Physic_Sec_125: Leg_state_35 + 999999 H_41 <= 999999
 H_Physic_Sec_126: - Leg_state_35 + 999999 H_41 <= 999999
 H_Physic_Sec_127: Leg_state_33 + 999999 H_42 <= 1.0000003e+06
 H_Physic_Sec_128: - Leg_state_33 + 999999 H_42 <= 999998.5
 H_Physic_Sec_129: Leg_state_34 + 999999 H_42 <= 999999.4
 H_Physic_Sec_130: - Leg_state_34 + 999999 H_42 <= 999999.4
 H_Physic_Sec_131: Leg_state_35 + 999999 H_42 <= 999999.1
 H_Physic_Sec_132: - Leg_state_35 + 999999 H_42 <= 999998.9
 H_Physic_Sec_133: Leg_state_33 + 999999 H_43 <= 1.000000825e+06
 H_Physic_Sec_134: - Leg_state_33 + 999999 H_43 <= 999997.675
 H_Physic_Sec_135: Leg_state_34 + 999999 H_43 <= 999999.4
 H_Physic_Sec_136: - Leg_state_34 + 999999 H_43 <= 999999.4
 H_Physic_Sec_137: Leg_state_35 + 999999 H_43 <= 999999.2
 H_Physic_Sec_138: - Leg_state_35 + 999999 H_43 <= 999998.8
 H_Physic_Sec_139: Leg_state_33 + 999999 H_44 <= 1.00000135e+06
 H_Physic_Sec_140: - Leg_state_33 + 999999 H_44 <= 999997.15
 H_Physic_Sec_141: Leg_state_34 + 999999 H_44 <= 999999.4
 H_Physic_Sec_142: - Leg_state_34 + 999999 H_44 <= 999999.4
 H_Physic_Sec_143: Leg_state_35 + 999999 H_44 <= 999999.1
 H_Physic_Sec_144: - Leg_state_35 + 999999 H_44 <= 999998.9
 H_Physic_Sec_145: Leg_state_33 + 999999 H_45 <= 1.0000026e+06
 H_Physic_Sec_146: - Leg_state_33 + 999999 H_45 <= 999996.6
 H_Physic_Sec_147: Leg_state_34 + 999999 H_45 <= 999999.4
 H_Physic_Sec_148: - Leg_state_34 + 999999 H_45 <= 999999.4
 H_Physic_Sec_149: Leg_state_35 + 999999 H_45 <= 999999
 H_Physic_Sec_150: - Leg_state_35 + 999999 H_45 <= 999999
 H_Physic_Sec_151: Leg_state_37 + 999999 H_46 <= 999999.475
 H_Physic_Sec_152: - Leg_state_37 + 999999 H_46 <= 999999.475
 H_Physic_Sec_153: Leg_state_38 + 999999 H_46 <= 999999.4
 H_Physic_Sec_154: - Leg_state_38 + 999999 H_46 <= 999999.4
 H_Physic_Sec_155: Leg_state_39 + 999999 H_46 <= 999999
 H_Physic_Sec_156: - Leg_state_39 + 999999 H_46 <= 999999
 H_Physic_Sec_157: Leg_state_37 + 999999 H_47 <= 1.0000003e+06
 H_Physic_Sec_158: - Leg_state_37 + 999999 H_47 <= 999998.5
 H_Physic_Sec_159: Leg_state_38 + 999999 H_47 <= 999999.4
 H_Physic_Sec_160: - Leg_state_38 + 999999 H_47 <= 999999.4
 H_Physic_Sec_161: Leg_state_39 + 999999 H_47 <= 999999.1
 H_Physic_Sec_162: - Leg_state_39 + 999999 H_47 <= 999998.9
 H_Physic_Sec_163: Leg_state_37 + 999999 H_48 <= 1.000000825e+06
 H_Physic_Sec_164: - Leg_state_37 + 999999 H_48 <= 999997.675
 H_Physic_Sec_165: Leg_state_38 + 999999 H_48 <= 999999.4
 H_Physic_Sec_166: - Leg_state_38 + 999999 H_48 <= 999999.4
 H_Physic_Sec_167: Leg_state_39 + 999999 H_48 <= 999999.2
 H_Physic_Sec_168: - Leg_state_39 + 999999 H_48 <= 999998.8
 H_Physic_Sec_169: Leg_state_37 + 999999 H_49 <= 1.00000135e+06
 H_Physic_Sec_170: - Leg_state_37 + 999999 H_49 <= 999997.15
 H_Physic_Sec_171: Leg_state_38 + 999999 H_49 <= 999999.4
 H_Physic_Sec_172: - Leg_state_38 + 999999 H_49 <= 999999.4
 H_Physic_Sec_173: Leg_state_39 + 999999 H_49 <= 999999.1
 H_Physic_Sec_174: - Leg_state_39 + 999999 H_49 <= 999998.9
 H_Physic_Sec_175: Leg_state_37 + 999999 H_50 <= 1.0000026e+06
 H_Physic_Sec_176: - Leg_state_37 + 999999 H_50 <= 999996.6
 H_Physic_Sec_177: Leg_state_38 + 999999 H_50 <= 999999.4
 H_Physic_Sec_178: - Leg_state_38 + 999999 H_50 <= 999999.4
 H_Physic_Sec_179: Leg_state_39 + 999999 H_50 <= 999999
 H_Physic_Sec_180: - Leg_state_39 + 999999 H_50 <= 999999
 H_Physic_Sec_181: Leg_state_41 + 999999 H_51 <= 999999.475
 H_Physic_Sec_182: - Leg_state_41 + 999999 H_51 <= 999999.475
 H_Physic_Sec_183: Leg_state_42 + 999999 H_51 <= 999999.4
 H_Physic_Sec_184: - Leg_state_42 + 999999 H_51 <= 999999.4
 H_Physic_Sec_185: Leg_state_43 + 999999 H_51 <= 999999
 H_Physic_Sec_186: - Leg_state_43 + 999999 H_51 <= 999999
 H_Physic_Sec_187: Leg_state_41 + 999999 H_52 <= 1.0000003e+06
 H_Physic_Sec_188: - Leg_state_41 + 999999 H_52 <= 999998.5
 H_Physic_Sec_189: Leg_state_42 + 999999 H_52 <= 999999.4
 H_Physic_Sec_190: - Leg_state_42 + 999999 H_52 <= 999999.4
 H_Physic_Sec_191: Leg_state_43 + 999999 H_52 <= 999999.1
 H_Physic_Sec_192: - Leg_state_43 + 999999 H_52 <= 999998.9
 H_Physic_Sec_193: Leg_state_41 + 999999 H_53 <= 1.000000825e+06
 H_Physic_Sec_194: - Leg_state_41 + 999999 H_53 <= 999997.675
 H_Physic_Sec_195: Leg_state_42 + 999999 H_53 <= 999999.4
 H_Physic_Sec_196: - Leg_state_42 + 999999 H_53 <= 999999.4
 H_Physic_Sec_197: Leg_state_43 + 999999 H_53 <= 999999.2
 H_Physic_Sec_198: - Leg_state_43 + 999999 H_53 <= 999998.8
 H_Physic_Sec_199: Leg_state_41 + 999999 H_54 <= 1.00000135e+06
 H_Physic_Sec_200: - Leg_state_41 + 999999 H_54 <= 999997.15
 H_Physic_Sec_201: Leg_state_42 + 999999 H_54 <= 999999.4
 H_Physic_Sec_202: - Leg_state_42 + 999999 H_54 <= 999999.4
 H_Physic_Sec_203: Leg_state_43 + 999999 H_54 <= 999999.1
 H_Physic_Sec_204: - Leg_state_43 + 999999 H_54 <= 999998.9
 H_Physic_Sec_205: Leg_state_41 + 999999 H_55 <= 1.0000026e+06
 H_Physic_Sec_206: - Leg_state_41 + 999999 H_55 <= 999996.6
 H_Physic_Sec_207: Leg_state_42 + 999999 H_55 <= 999999.4
 H_Physic_Sec_208: - Leg_state_42 + 999999 H_55 <= 999999.4
 H_Physic_Sec_209: Leg_state_43 + 999999 H_55 <= 999999
 H_Physic_Sec_210: - Leg_state_43 + 999999 H_55 <= 999999
 H_Physic_Sec_211: Leg_state_45 + 999999 H_56 <= 999999.475
 H_Physic_Sec_212: - Leg_state_45 + 999999 H_56 <= 999999.475
 H_Physic_Sec_213: Leg_state_46 + 999999 H_56 <= 999999.4
 H_Physic_Sec_214: - Leg_state_46 + 999999 H_56 <= 999999.4
 H_Physic_Sec_215: Leg_state_47 + 999999 H_56 <= 999999
 H_Physic_Sec_216: - Leg_state_47 + 999999 H_56 <= 999999
 H_Physic_Sec_217: Leg_state_45 + 999999 H_57 <= 1.0000003e+06
 H_Physic_Sec_218: - Leg_state_45 + 999999 H_57 <= 999998.5
 H_Physic_Sec_219: Leg_state_46 + 999999 H_57 <= 999999.4
 H_Physic_Sec_220: - Leg_state_46 + 999999 H_57 <= 999999.4
 H_Physic_Sec_221: Leg_state_47 + 999999 H_57 <= 999999.1
 H_Physic_Sec_222: - Leg_state_47 + 999999 H_57 <= 999998.9
 H_Physic_Sec_223: Leg_state_45 + 999999 H_58 <= 1.000000825e+06
 H_Physic_Sec_224: - Leg_state_45 + 999999 H_58 <= 999997.675
 H_Physic_Sec_225: Leg_state_46 + 999999 H_58 <= 999999.4
 H_Physic_Sec_226: - Leg_state_46 + 999999 H_58 <= 999999.4
 H_Physic_Sec_227: Leg_state_47 + 999999 H_58 <= 999999.2
 H_Physic_Sec_228: - Leg_state_47 + 999999 H_58 <= 999998.8
 H_Physic_Sec_229: Leg_state_45 + 999999 H_59 <= 1.00000135e+06
 H_Physic_Sec_230: - Leg_state_45 + 999999 H_59 <= 999997.15
 H_Physic_Sec_231: Leg_state_46 + 999999 H_59 <= 999999.4
 H_Physic_Sec_232: - Leg_state_46 + 999999 H_59 <= 999999.4
 H_Physic_Sec_233: Leg_state_47 + 999999 H_59 <= 999999.1
 H_Physic_Sec_234: - Leg_state_47 + 999999 H_59 <= 999998.9
 H_Physic_Sec_235: Leg_state_45 + 999999 H_60 <= 1.0000026e+06
 H_Physic_Sec_236: - Leg_state_45 + 999999 H_60 <= 999996.6
 H_Physic_Sec_237: Leg_state_46 + 999999 H_60 <= 999999.4
 H_Physic_Sec_238: - Leg_state_46 + 999999 H_60 <= 999999.4
 H_Physic_Sec_239: Leg_state_47 + 999999 H_60 <= 999999
 H_Physic_Sec_240: - Leg_state_47 + 999999 H_60 <= 999999
 H_Physic_Sec_241: Leg_state_49 + 999999 H_61 <= 999999.475
 H_Physic_Sec_242: - Leg_state_49 + 999999 H_61 <= 999999.475
 H_Physic_Sec_243: Leg_state_50 + 999999 H_61 <= 999999.4
 H_Physic_Sec_244: - Leg_state_50 + 999999 H_61 <= 999999.4
 H_Physic_Sec_245: Leg_state_51 + 999999 H_61 <= 999999
 H_Physic_Sec_246: - Leg_state_51 + 999999 H_61 <= 999999
 H_Physic_Sec_247: Leg_state_49 + 999999 H_62 <= 1.0000003e+06
 H_Physic_Sec_248: - Leg_state_49 + 999999 H_62 <= 999998.5
 H_Physic_Sec_249: Leg_state_50 + 999999 H_62 <= 999999.4
 H_Physic_Sec_250: - Leg_state_50 + 999999 H_62 <= 999999.4
 H_Physic_Sec_251: Leg_state_51 + 999999 H_62 <= 999999.1
 H_Physic_Sec_252: - Leg_state_51 + 999999 H_62 <= 999998.9
 H_Physic_Sec_253: Leg_state_49 + 999999 H_63 <= 1.000000825e+06
 H_Physic_Sec_254: - Leg_state_49 + 999999 H_63 <= 999997.675
 H_Physic_Sec_255: Leg_state_50 + 999999 H_63 <= 999999.4
 H_Physic_Sec_256: - Leg_state_50 + 999999 H_63 <= 999999.4
 H_Physic_Sec_257: Leg_state_51 + 999999 H_63 <= 999999.2
 H_Physic_Sec_258: - Leg_state_51 + 999999 H_63 <= 999998.8
 H_Physic_Sec_259: Leg_state_49 + 999999 H_64 <= 1.00000135e+06
 H_Physic_Sec_260: - Leg_state_49 + 999999 H_64 <= 999997.15
 H_Physic_Sec_261: Leg_state_50 + 999999 H_64 <= 999999.4
 H_Physic_Sec_262: - Leg_state_50 + 999999 H_64 <= 999999.4
 H_Physic_Sec_263: Leg_state_51 + 999999 H_64 <= 999999.1
 H_Physic_Sec_264: - Leg_state_51 + 999999 H_64 <= 999998.9
 H_Physic_Sec_265: Leg_state_49 + 999999 H_65 <= 1.0000026e+06
 H_Physic_Sec_266: - Leg_state_49 + 999999 H_65 <= 999996.6
 H_Physic_Sec_267: Leg_state_50 + 999999 H_65 <= 999999.4
 H_Physic_Sec_268: - Leg_state_50 + 999999 H_65 <= 999999.4
 H_Physic_Sec_269: Leg_state_51 + 999999 H_65 <= 999999
 H_Physic_Sec_270: - Leg_state_51 + 999999 H_65 <= 999999
 H_Physic_Sec_271: Leg_state_53 + 999999 H_66 <= 999999.475
 H_Physic_Sec_272: - Leg_state_53 + 999999 H_66 <= 999999.475
 H_Physic_Sec_273: Leg_state_54 + 999999 H_66 <= 999999.4
 H_Physic_Sec_274: - Leg_state_54 + 999999 H_66 <= 999999.4
 H_Physic_Sec_275: Leg_state_55 + 999999 H_66 <= 999999
 H_Physic_Sec_276: - Leg_state_55 + 999999 H_66 <= 999999
 H_Physic_Sec_277: Leg_state_53 + 999999 H_67 <= 1.0000003e+06
 H_Physic_Sec_278: - Leg_state_53 + 999999 H_67 <= 999998.5
 H_Physic_Sec_279: Leg_state_54 + 999999 H_67 <= 999999.4
 H_Physic_Sec_280: - Leg_state_54 + 999999 H_67 <= 999999.4
 H_Physic_Sec_281: Leg_state_55 + 999999 H_67 <= 999999.1
 H_Physic_Sec_282: - Leg_state_55 + 999999 H_67 <= 999998.9
 H_Physic_Sec_283: Leg_state_53 + 999999 H_68 <= 1.000000825e+06
 H_Physic_Sec_284: - Leg_state_53 + 999999 H_68 <= 999997.675
 H_Physic_Sec_285: Leg_state_54 + 999999 H_68 <= 999999.4
 H_Physic_Sec_286: - Leg_state_54 + 999999 H_68 <= 999999.4
 H_Physic_Sec_287: Leg_state_55 + 999999 H_68 <= 999999.2
 H_Physic_Sec_288: - Leg_state_55 + 999999 H_68 <= 999998.8
 H_Physic_Sec_289: Leg_state_53 + 999999 H_69 <= 1.00000135e+06
 H_Physic_Sec_290: - Leg_state_53 + 999999 H_69 <= 999997.15
 H_Physic_Sec_291: Leg_state_54 + 999999 H_69 <= 999999.4
 H_Physic_Sec_292: - Leg_state_54 + 999999 H_69 <= 999999.4
 H_Physic_Sec_293: Leg_state_55 + 999999 H_69 <= 999999.1
 H_Physic_Sec_294: - Leg_state_55 + 999999 H_69 <= 999998.9
 H_Physic_Sec_295: Leg_state_53 + 999999 H_70 <= 1.0000026e+06
 H_Physic_Sec_296: - Leg_state_53 + 999999 H_70 <= 999996.6
 H_Physic_Sec_297: Leg_state_54 + 999999 H_70 <= 999999.4
 H_Physic_Sec_298: - Leg_state_54 + 999999 H_70 <= 999999.4
 H_Physic_Sec_299: Leg_state_55 + 999999 H_70 <= 999999
 H_Physic_Sec_300: - Leg_state_55 + 999999 H_70 <= 999999
 H_Physic_Sec_301: Leg_state_57 + 999999 H_71 <= 999999.475
 H_Physic_Sec_302: - Leg_state_57 + 999999 H_71 <= 999999.475
 H_Physic_Sec_303: Leg_state_58 + 999999 H_71 <= 999999.4
 H_Physic_Sec_304: - Leg_state_58 + 999999 H_71 <= 999999.4
 H_Physic_Sec_305: Leg_state_59 + 999999 H_71 <= 999999
 H_Physic_Sec_306: - Leg_state_59 + 999999 H_71 <= 999999
 H_Physic_Sec_307: Leg_state_57 + 999999 H_72 <= 1.0000003e+06
 H_Physic_Sec_308: - Leg_state_57 + 999999 H_72 <= 999998.5
 H_Physic_Sec_309: Leg_state_58 + 999999 H_72 <= 999999.4
 H_Physic_Sec_310: - Leg_state_58 + 999999 H_72 <= 999999.4
 H_Physic_Sec_311: Leg_state_59 + 999999 H_72 <= 999999.1
 H_Physic_Sec_312: - Leg_state_59 + 999999 H_72 <= 999998.9
 H_Physic_Sec_313: Leg_state_57 + 999999 H_73 <= 1.000000825e+06
 H_Physic_Sec_314: - Leg_state_57 + 999999 H_73 <= 999997.675
 H_Physic_Sec_315: Leg_state_58 + 999999 H_73 <= 999999.4
 H_Physic_Sec_316: - Leg_state_58 + 999999 H_73 <= 999999.4
 H_Physic_Sec_317: Leg_state_59 + 999999 H_73 <= 999999.2
 H_Physic_Sec_318: - Leg_state_59 + 999999 H_73 <= 999998.8
 H_Physic_Sec_319: Leg_state_57 + 999999 H_74 <= 1.00000135e+06
 H_Physic_Sec_320: - Leg_state_57 + 999999 H_74 <= 999997.15
 H_Physic_Sec_321: Leg_state_58 + 999999 H_74 <= 999999.4
 H_Physic_Sec_322: - Leg_state_58 + 999999 H_74 <= 999999.4
 H_Physic_Sec_323: Leg_state_59 + 999999 H_74 <= 999999.1
 H_Physic_Sec_324: - Leg_state_59 + 999999 H_74 <= 999998.9
 H_Physic_Sec_325: Leg_state_57 + 999999 H_75 <= 1.0000026e+06
 H_Physic_Sec_326: - Leg_state_57 + 999999 H_75 <= 999996.6
 H_Physic_Sec_327: Leg_state_58 + 999999 H_75 <= 999999.4
 H_Physic_Sec_328: - Leg_state_58 + 999999 H_75 <= 999999.4
 H_Physic_Sec_329: Leg_state_59 + 999999 H_75 <= 999999
 H_Physic_Sec_330: - Leg_state_59 + 999999 H_75 <= 999999
 H_Physic_Sec_331: Leg_state_61 + 999999 H_76 <= 999999.475
 H_Physic_Sec_332: - Leg_state_61 + 999999 H_76 <= 999999.475
 H_Physic_Sec_333: Leg_state_62 + 999999 H_76 <= 999999.4
 H_Physic_Sec_334: - Leg_state_62 + 999999 H_76 <= 999999.4
 H_Physic_Sec_335: Leg_state_63 + 999999 H_76 <= 999999
 H_Physic_Sec_336: - Leg_state_63 + 999999 H_76 <= 999999
 H_Physic_Sec_337: Leg_state_61 + 999999 H_77 <= 1.0000003e+06
 H_Physic_Sec_338: - Leg_state_61 + 999999 H_77 <= 999998.5
 H_Physic_Sec_339: Leg_state_62 + 999999 H_77 <= 999999.4
 H_Physic_Sec_340: - Leg_state_62 + 999999 H_77 <= 999999.4
 H_Physic_Sec_341: Leg_state_63 + 999999 H_77 <= 999999.1
 H_Physic_Sec_342: - Leg_state_63 + 999999 H_77 <= 999998.9
 H_Physic_Sec_343: Leg_state_61 + 999999 H_78 <= 1.000000825e+06
 H_Physic_Sec_344: - Leg_state_61 + 999999 H_78 <= 999997.675
 H_Physic_Sec_345: Leg_state_62 + 999999 H_78 <= 999999.4
 H_Physic_Sec_346: - Leg_state_62 + 999999 H_78 <= 999999.4
 H_Physic_Sec_347: Leg_state_63 + 999999 H_78 <= 999999.2
 H_Physic_Sec_348: - Leg_state_63 + 999999 H_78 <= 999998.8
 H_Physic_Sec_349: Leg_state_61 + 999999 H_79 <= 1.00000135e+06
 H_Physic_Sec_350: - Leg_state_61 + 999999 H_79 <= 999997.15
 H_Physic_Sec_351: Leg_state_62 + 999999 H_79 <= 999999.4
 H_Physic_Sec_352: - Leg_state_62 + 999999 H_79 <= 999999.4
 H_Physic_Sec_353: Leg_state_63 + 999999 H_79 <= 999999.1
 H_Physic_Sec_354: - Leg_state_63 + 999999 H_79 <= 999998.9
 H_Physic_Sec_355: Leg_state_61 + 999999 H_80 <= 1.0000026e+06
 H_Physic_Sec_356: - Leg_state_61 + 999999 H_80 <= 999996.6
 H_Physic_Sec_357: Leg_state_62 + 999999 H_80 <= 999999.4
 H_Physic_Sec_358: - Leg_state_62 + 999999 H_80 <= 999999.4
 H_Physic_Sec_359: Leg_state_63 + 999999 H_80 <= 999999
 H_Physic_Sec_360: - Leg_state_63 + 999999 H_80 <= 999999
 H_Physic_Sec_361: Leg_state_65 + 999999 H_81 <= 999999.475
 H_Physic_Sec_362: - Leg_state_65 + 999999 H_81 <= 999999.475
 H_Physic_Sec_363: Leg_state_66 + 999999 H_81 <= 999999.4
 H_Physic_Sec_364: - Leg_state_66 + 999999 H_81 <= 999999.4
 H_Physic_Sec_365: Leg_state_67 + 999999 H_81 <= 999999
 H_Physic_Sec_366: - Leg_state_67 + 999999 H_81 <= 999999
 H_Physic_Sec_367: Leg_state_65 + 999999 H_82 <= 1.0000003e+06
 H_Physic_Sec_368: - Leg_state_65 + 999999 H_82 <= 999998.5
 H_Physic_Sec_369: Leg_state_66 + 999999 H_82 <= 999999.4
 H_Physic_Sec_370: - Leg_state_66 + 999999 H_82 <= 999999.4
 H_Physic_Sec_371: Leg_state_67 + 999999 H_82 <= 999999.1
 H_Physic_Sec_372: - Leg_state_67 + 999999 H_82 <= 999998.9
 H_Physic_Sec_373: Leg_state_65 + 999999 H_83 <= 1.000000825e+06
 H_Physic_Sec_374: - Leg_state_65 + 999999 H_83 <= 999997.675
 H_Physic_Sec_375: Leg_state_66 + 999999 H_83 <= 999999.4
 H_Physic_Sec_376: - Leg_state_66 + 999999 H_83 <= 999999.4
 H_Physic_Sec_377: Leg_state_67 + 999999 H_83 <= 999999.2
 H_Physic_Sec_378: - Leg_state_67 + 999999 H_83 <= 999998.8
 H_Physic_Sec_379: Leg_state_65 + 999999 H_84 <= 1.00000135e+06
 H_Physic_Sec_380: - Leg_state_65 + 999999 H_84 <= 999997.15
 H_Physic_Sec_381: Leg_state_66 + 999999 H_84 <= 999999.4
 H_Physic_Sec_382: - Leg_state_66 + 999999 H_84 <= 999999.4
 H_Physic_Sec_383: Leg_state_67 + 999999 H_84 <= 999999.1
 H_Physic_Sec_384: - Leg_state_67 + 999999 H_84 <= 999998.9
 H_Physic_Sec_385: Leg_state_65 + 999999 H_85 <= 1.0000026e+06
 H_Physic_Sec_386: - Leg_state_65 + 999999 H_85 <= 999996.6
 H_Physic_Sec_387: Leg_state_66 + 999999 H_85 <= 999999.4
 H_Physic_Sec_388: - Leg_state_66 + 999999 H_85 <= 999999.4
 H_Physic_Sec_389: Leg_state_67 + 999999 H_85 <= 999999
 H_Physic_Sec_390: - Leg_state_67 + 999999 H_85 <= 999999
 H_Physic_Sec_391: Leg_state_69 + 999999 H_86 <= 999999.475
 H_Physic_Sec_392: - Leg_state_69 + 999999 H_86 <= 999999.475
 H_Physic_Sec_393: Leg_state_70 + 999999 H_86 <= 999999.4
 H_Physic_Sec_394: - Leg_state_70 + 999999 H_86 <= 999999.4
 H_Physic_Sec_395: Leg_state_71 + 999999 H_86 <= 999999
 H_Physic_Sec_396: - Leg_state_71 + 999999 H_86 <= 999999
 H_Physic_Sec_397: Leg_state_69 + 999999 H_87 <= 1.0000003e+06
 H_Physic_Sec_398: - Leg_state_69 + 999999 H_87 <= 999998.5
 H_Physic_Sec_399: Leg_state_70 + 999999 H_87 <= 999999.4
 H_Physic_Sec_400: - Leg_state_70 + 999999 H_87 <= 999999.4
 H_Physic_Sec_401: Leg_state_71 + 999999 H_87 <= 999999.1
 H_Physic_Sec_402: - Leg_state_71 + 999999 H_87 <= 999998.9
 H_Physic_Sec_403: Leg_state_69 + 999999 H_88 <= 1.000000825e+06
 H_Physic_Sec_404: - Leg_state_69 + 999999 H_88 <= 999997.675
 H_Physic_Sec_405: Leg_state_70 + 999999 H_88 <= 999999.4
 H_Physic_Sec_406: - Leg_state_70 + 999999 H_88 <= 999999.4
 H_Physic_Sec_407: Leg_state_71 + 999999 H_88 <= 999999.2
 H_Physic_Sec_408: - Leg_state_71 + 999999 H_88 <= 999998.8
 H_Physic_Sec_409: Leg_state_69 + 999999 H_89 <= 1.00000135e+06
 H_Physic_Sec_410: - Leg_state_69 + 999999 H_89 <= 999997.15
 H_Physic_Sec_411: Leg_state_70 + 999999 H_89 <= 999999.4
 H_Physic_Sec_412: - Leg_state_70 + 999999 H_89 <= 999999.4
 H_Physic_Sec_413: Leg_state_71 + 999999 H_89 <= 999999.1
 H_Physic_Sec_414: - Leg_state_71 + 999999 H_89 <= 999998.9
 H_Physic_Sec_415: Leg_state_69 + 999999 H_90 <= 1.0000026e+06
 H_Physic_Sec_416: - Leg_state_69 + 999999 H_90 <= 999996.6
 H_Physic_Sec_417: Leg_state_70 + 999999 H_90 <= 999999.4
 H_Physic_Sec_418: - Leg_state_70 + 999999 H_90 <= 999999.4
 H_Physic_Sec_419: Leg_state_71 + 999999 H_90 <= 999999
 H_Physic_Sec_420: - Leg_state_71 + 999999 H_90 <= 999999
 H_Physic_Sec_421: Leg_state_73 + 999999 H_91 <= 999999.475
 H_Physic_Sec_422: - Leg_state_73 + 999999 H_91 <= 999999.475
 H_Physic_Sec_423: Leg_state_74 + 999999 H_91 <= 999999.4
 H_Physic_Sec_424: - Leg_state_74 + 999999 H_91 <= 999999.4
 H_Physic_Sec_425: Leg_state_75 + 999999 H_91 <= 999999
 H_Physic_Sec_426: - Leg_state_75 + 999999 H_91 <= 999999
 H_Physic_Sec_427: Leg_state_73 + 999999 H_92 <= 1.0000003e+06
 H_Physic_Sec_428: - Leg_state_73 + 999999 H_92 <= 999998.5
 H_Physic_Sec_429: Leg_state_74 + 999999 H_92 <= 999999.4
 H_Physic_Sec_430: - Leg_state_74 + 999999 H_92 <= 999999.4
 H_Physic_Sec_431: Leg_state_75 + 999999 H_92 <= 999999.1
 H_Physic_Sec_432: - Leg_state_75 + 999999 H_92 <= 999998.9
 H_Physic_Sec_433: Leg_state_73 + 999999 H_93 <= 1.000000825e+06
 H_Physic_Sec_434: - Leg_state_73 + 999999 H_93 <= 999997.675
 H_Physic_Sec_435: Leg_state_74 + 999999 H_93 <= 999999.4
 H_Physic_Sec_436: - Leg_state_74 + 999999 H_93 <= 999999.4
 H_Physic_Sec_437: Leg_state_75 + 999999 H_93 <= 999999.2
 H_Physic_Sec_438: - Leg_state_75 + 999999 H_93 <= 999998.8
 H_Physic_Sec_439: Leg_state_73 + 999999 H_94 <= 1.00000135e+06
 H_Physic_Sec_440: - Leg_state_73 + 999999 H_94 <= 999997.15
 H_Physic_Sec_441: Leg_state_74 + 999999 H_94 <= 999999.4
 H_Physic_Sec_442: - Leg_state_74 + 999999 H_94 <= 999999.4
 H_Physic_Sec_443: Leg_state_75 + 999999 H_94 <= 999999.1
 H_Physic_Sec_444: - Leg_state_75 + 999999 H_94 <= 999998.9
 H_Physic_Sec_445: Leg_state_73 + 999999 H_95 <= 1.0000026e+06
 H_Physic_Sec_446: - Leg_state_73 + 999999 H_95 <= 999996.6
 H_Physic_Sec_447: Leg_state_74 + 999999 H_95 <= 999999.4
 H_Physic_Sec_448: - Leg_state_74 + 999999 H_95 <= 999999.4
 H_Physic_Sec_449: Leg_state_75 + 999999 H_95 <= 999999
 H_Physic_Sec_450: - Leg_state_75 + 999999 H_95 <= 999999
 H_Physic_Sec_451: Leg_state_77 + 999999 H_96 <= 999999.475
 H_Physic_Sec_452: - Leg_state_77 + 999999 H_96 <= 999999.475
 H_Physic_Sec_453: Leg_state_78 + 999999 H_96 <= 999999.4
 H_Physic_Sec_454: - Leg_state_78 + 999999 H_96 <= 999999.4
 H_Physic_Sec_455: Leg_state_79 + 999999 H_96 <= 999999
 H_Physic_Sec_456: - Leg_state_79 + 999999 H_96 <= 999999
 H_Physic_Sec_457: Leg_state_77 + 999999 H_97 <= 1.0000003e+06
 H_Physic_Sec_458: - Leg_state_77 + 999999 H_97 <= 999998.5
 H_Physic_Sec_459: Leg_state_78 + 999999 H_97 <= 999999.4
 H_Physic_Sec_460: - Leg_state_78 + 999999 H_97 <= 999999.4
 H_Physic_Sec_461: Leg_state_79 + 999999 H_97 <= 999999.1
 H_Physic_Sec_462: - Leg_state_79 + 999999 H_97 <= 999998.9
 H_Physic_Sec_463: Leg_state_77 + 999999 H_98 <= 1.000000825e+06
 H_Physic_Sec_464: - Leg_state_77 + 999999 H_98 <= 999997.675
 H_Physic_Sec_465: Leg_state_78 + 999999 H_98 <= 999999.4
 H_Physic_Sec_466: - Leg_state_78 + 999999 H_98 <= 999999.4
 H_Physic_Sec_467: Leg_state_79 + 999999 H_98 <= 999999.2
 H_Physic_Sec_468: - Leg_state_79 + 999999 H_98 <= 999998.8
 H_Physic_Sec_469: Leg_state_77 + 999999 H_99 <= 1.00000135e+06
 H_Physic_Sec_470: - Leg_state_77 + 999999 H_99 <= 999997.15
 H_Physic_Sec_471: Leg_state_78 + 999999 H_99 <= 999999.4
 H_Physic_Sec_472: - Leg_state_78 + 999999 H_99 <= 999999.4
 H_Physic_Sec_473: Leg_state_79 + 999999 H_99 <= 999999.1
 H_Physic_Sec_474: - Leg_state_79 + 999999 H_99 <= 999998.9
 H_Physic_Sec_475: Leg_state_77 + 999999 H_100 <= 1.0000026e+06
 H_Physic_Sec_476: - Leg_state_77 + 999999 H_100 <= 999996.6
 H_Physic_Sec_477: Leg_state_78 + 999999 H_100 <= 999999.4
 H_Physic_Sec_478: - Leg_state_78 + 999999 H_100 <= 999999.4
 H_Physic_Sec_479: Leg_state_79 + 999999 H_100 <= 999999
 H_Physic_Sec_480: - Leg_state_79 + 999999 H_100 <= 999999
 H_Physic_Sec_481: Leg_state_81 + 999999 H_101 <= 999999.475
 H_Physic_Sec_482: - Leg_state_81 + 999999 H_101 <= 999999.475
 H_Physic_Sec_483: Leg_state_82 + 999999 H_101 <= 999999.4
 H_Physic_Sec_484: - Leg_state_82 + 999999 H_101 <= 999999.4
 H_Physic_Sec_485: Leg_state_83 + 999999 H_101 <= 999999
 H_Physic_Sec_486: - Leg_state_83 + 999999 H_101 <= 999999
 H_Physic_Sec_487: Leg_state_81 + 999999 H_102 <= 1.0000003e+06
 H_Physic_Sec_488: - Leg_state_81 + 999999 H_102 <= 999998.5
 H_Physic_Sec_489: Leg_state_82 + 999999 H_102 <= 999999.4
 H_Physic_Sec_490: - Leg_state_82 + 999999 H_102 <= 999999.4
 H_Physic_Sec_491: Leg_state_83 + 999999 H_102 <= 999999.1
 H_Physic_Sec_492: - Leg_state_83 + 999999 H_102 <= 999998.9
 H_Physic_Sec_493: Leg_state_81 + 999999 H_103 <= 1.000000825e+06
 H_Physic_Sec_494: - Leg_state_81 + 999999 H_103 <= 999997.675
 H_Physic_Sec_495: Leg_state_82 + 999999 H_103 <= 999999.4
 H_Physic_Sec_496: - Leg_state_82 + 999999 H_103 <= 999999.4
 H_Physic_Sec_497: Leg_state_83 + 999999 H_103 <= 999999.2
 H_Physic_Sec_498: - Leg_state_83 + 999999 H_103 <= 999998.8
 H_Physic_Sec_499: Leg_state_81 + 999999 H_104 <= 1.00000135e+06
 H_Physic_Sec_500: - Leg_state_81 + 999999 H_104 <= 999997.15
 H_Physic_Sec_501: Leg_state_82 + 999999 H_104 <= 999999.4
 H_Physic_Sec_502: - Leg_state_82 + 999999 H_104 <= 999999.4
 H_Physic_Sec_503: Leg_state_83 + 999999 H_104 <= 999999.1
 H_Physic_Sec_504: - Leg_state_83 + 999999 H_104 <= 999998.9
 H_Physic_Sec_505: Leg_state_81 + 999999 H_105 <= 1.0000026e+06
 H_Physic_Sec_506: - Leg_state_81 + 999999 H_105 <= 999996.6
 H_Physic_Sec_507: Leg_state_82 + 999999 H_105 <= 999999.4
 H_Physic_Sec_508: - Leg_state_82 + 999999 H_105 <= 999999.4
 H_Physic_Sec_509: Leg_state_83 + 999999 H_105 <= 999999
 H_Physic_Sec_510: - Leg_state_83 + 999999 H_105 <= 999999
 H_Physic_Sec_511: Leg_state_85 + 999999 H_106 <= 999999.475
 H_Physic_Sec_512: - Leg_state_85 + 999999 H_106 <= 999999.475
 H_Physic_Sec_513: Leg_state_86 + 999999 H_106 <= 999999.4
 H_Physic_Sec_514: - Leg_state_86 + 999999 H_106 <= 999999.4
 H_Physic_Sec_515: Leg_state_87 + 999999 H_106 <= 999999
 H_Physic_Sec_516: - Leg_state_87 + 999999 H_106 <= 999999
 H_Physic_Sec_517: Leg_state_85 + 999999 H_107 <= 1.0000003e+06
 H_Physic_Sec_518: - Leg_state_85 + 999999 H_107 <= 999998.5
 H_Physic_Sec_519: Leg_state_86 + 999999 H_107 <= 999999.4
 H_Physic_Sec_520: - Leg_state_86 + 999999 H_107 <= 999999.4
 H_Physic_Sec_521: Leg_state_87 + 999999 H_107 <= 999999.1
 H_Physic_Sec_522: - Leg_state_87 + 999999 H_107 <= 999998.9
 H_Physic_Sec_523: Leg_state_85 + 999999 H_108 <= 1.000000825e+06
 H_Physic_Sec_524: - Leg_state_85 + 999999 H_108 <= 999997.675
 H_Physic_Sec_525: Leg_state_86 + 999999 H_108 <= 999999.4
 H_Physic_Sec_526: - Leg_state_86 + 999999 H_108 <= 999999.4
 H_Physic_Sec_527: Leg_state_87 + 999999 H_108 <= 999999.2
 H_Physic_Sec_528: - Leg_state_87 + 999999 H_108 <= 999998.8
 H_Physic_Sec_529: Leg_state_85 + 999999 H_109 <= 1.00000135e+06
 H_Physic_Sec_530: - Leg_state_85 + 999999 H_109 <= 999997.15
 H_Physic_Sec_531: Leg_state_86 + 999999 H_109 <= 999999.4
 H_Physic_Sec_532: - Leg_state_86 + 999999 H_109 <= 999999.4
 H_Physic_Sec_533: Leg_state_87 + 999999 H_109 <= 999999.1
 H_Physic_Sec_534: - Leg_state_87 + 999999 H_109 <= 999998.9
 H_Physic_Sec_535: Leg_state_85 + 999999 H_110 <= 1.0000026e+06
 H_Physic_Sec_536: - Leg_state_85 + 999999 H_110 <= 999996.6
 H_Physic_Sec_537: Leg_state_86 + 999999 H_110 <= 999999.4
 H_Physic_Sec_538: - Leg_state_86 + 999999 H_110 <= 999999.4
 H_Physic_Sec_539: Leg_state_87 + 999999 H_110 <= 999999
 H_Physic_Sec_540: - Leg_state_87 + 999999 H_110 <= 999999
 H_Physic_Sec_541: Leg_state_89 + 999999 H_111 <= 999999.475
 H_Physic_Sec_542: - Leg_state_89 + 999999 H_111 <= 999999.475
 H_Physic_Sec_543: Leg_state_90 + 999999 H_111 <= 999999.4
 H_Physic_Sec_544: - Leg_state_90 + 999999 H_111 <= 999999.4
 H_Physic_Sec_545: Leg_state_91 + 999999 H_111 <= 999999
 H_Physic_Sec_546: - Leg_state_91 + 999999 H_111 <= 999999
 H_Physic_Sec_547: Leg_state_89 + 999999 H_112 <= 1.0000003e+06
 H_Physic_Sec_548: - Leg_state_89 + 999999 H_112 <= 999998.5
 H_Physic_Sec_549: Leg_state_90 + 999999 H_112 <= 999999.4
 H_Physic_Sec_550: - Leg_state_90 + 999999 H_112 <= 999999.4
 H_Physic_Sec_551: Leg_state_91 + 999999 H_112 <= 999999.1
 H_Physic_Sec_552: - Leg_state_91 + 999999 H_112 <= 999998.9
 H_Physic_Sec_553: Leg_state_89 + 999999 H_113 <= 1.000000825e+06
 H_Physic_Sec_554: - Leg_state_89 + 999999 H_113 <= 999997.675
 H_Physic_Sec_555: Leg_state_90 + 999999 H_113 <= 999999.4
 H_Physic_Sec_556: - Leg_state_90 + 999999 H_113 <= 999999.4
 H_Physic_Sec_557: Leg_state_91 + 999999 H_113 <= 999999.2
 H_Physic_Sec_558: - Leg_state_91 + 999999 H_113 <= 999998.8
 H_Physic_Sec_559: Leg_state_89 + 999999 H_114 <= 1.00000135e+06
 H_Physic_Sec_560: - Leg_state_89 + 999999 H_114 <= 999997.15
 H_Physic_Sec_561: Leg_state_90 + 999999 H_114 <= 999999.4
 H_Physic_Sec_562: - Leg_state_90 + 999999 H_114 <= 999999.4
 H_Physic_Sec_563: Leg_state_91 + 999999 H_114 <= 999999.1
 H_Physic_Sec_564: - Leg_state_91 + 999999 H_114 <= 999998.9
 H_Physic_Sec_565: Leg_state_89 + 999999 H_115 <= 1.0000026e+06
 H_Physic_Sec_566: - Leg_state_89 + 999999 H_115 <= 999996.6
 H_Physic_Sec_567: Leg_state_90 + 999999 H_115 <= 999999.4
 H_Physic_Sec_568: - Leg_state_90 + 999999 H_115 <= 999999.4
 H_Physic_Sec_569: Leg_state_91 + 999999 H_115 <= 999999
 H_Physic_Sec_570: - Leg_state_91 + 999999 H_115 <= 999999
 H_Physic_Sec_571: Leg_state_93 + 999999 H_116 <= 999999.475
 H_Physic_Sec_572: - Leg_state_93 + 999999 H_116 <= 999999.475
 H_Physic_Sec_573: Leg_state_94 + 999999 H_116 <= 999999.4
 H_Physic_Sec_574: - Leg_state_94 + 999999 H_116 <= 999999.4
 H_Physic_Sec_575: Leg_state_95 + 999999 H_116 <= 999999
 H_Physic_Sec_576: - Leg_state_95 + 999999 H_116 <= 999999
 H_Physic_Sec_577: Leg_state_93 + 999999 H_117 <= 1.0000003e+06
 H_Physic_Sec_578: - Leg_state_93 + 999999 H_117 <= 999998.5
 H_Physic_Sec_579: Leg_state_94 + 999999 H_117 <= 999999.4
 H_Physic_Sec_580: - Leg_state_94 + 999999 H_117 <= 999999.4
 H_Physic_Sec_581: Leg_state_95 + 999999 H_117 <= 999999.1
 H_Physic_Sec_582: - Leg_state_95 + 999999 H_117 <= 999998.9
 H_Physic_Sec_583: Leg_state_93 + 999999 H_118 <= 1.000000825e+06
 H_Physic_Sec_584: - Leg_state_93 + 999999 H_118 <= 999997.675
 H_Physic_Sec_585: Leg_state_94 + 999999 H_118 <= 999999.4
 H_Physic_Sec_586: - Leg_state_94 + 999999 H_118 <= 999999.4
 H_Physic_Sec_587: Leg_state_95 + 999999 H_118 <= 999999.2
 H_Physic_Sec_588: - Leg_state_95 + 999999 H_118 <= 999998.8
 H_Physic_Sec_589: Leg_state_93 + 999999 H_119 <= 1.00000135e+06
 H_Physic_Sec_590: - Leg_state_93 + 999999 H_119 <= 999997.15
 H_Physic_Sec_591: Leg_state_94 + 999999 H_119 <= 999999.4
 H_Physic_Sec_592: - Leg_state_94 + 999999 H_119 <= 999999.4
 H_Physic_Sec_593: Leg_state_95 + 999999 H_119 <= 999999.1
 H_Physic_Sec_594: - Leg_state_95 + 999999 H_119 <= 999998.9
 H_Physic_Sec_595: Leg_state_93 + 999999 H_120 <= 1.0000026e+06
 H_Physic_Sec_596: - Leg_state_93 + 999999 H_120 <= 999996.6
 H_Physic_Sec_597: Leg_state_94 + 999999 H_120 <= 999999.4
 H_Physic_Sec_598: - Leg_state_94 + 999999 H_120 <= 999999.4
 H_Physic_Sec_599: Leg_state_95 + 999999 H_120 <= 999999
 H_Physic_Sec_600: - Leg_state_95 + 999999 H_120 <= 999999
 H_Physic_Sec_601: Leg_state_97 + 999999 H_121 <= 999999.475
 H_Physic_Sec_602: - Leg_state_97 + 999999 H_121 <= 999999.475
 H_Physic_Sec_603: Leg_state_98 + 999999 H_121 <= 999999.4
 H_Physic_Sec_604: - Leg_state_98 + 999999 H_121 <= 999999.4
 H_Physic_Sec_605: Leg_state_99 + 999999 H_121 <= 999999
 H_Physic_Sec_606: - Leg_state_99 + 999999 H_121 <= 999999
 H_Physic_Sec_607: Leg_state_97 + 999999 H_122 <= 1.0000003e+06
 H_Physic_Sec_608: - Leg_state_97 + 999999 H_122 <= 999998.5
 H_Physic_Sec_609: Leg_state_98 + 999999 H_122 <= 999999.4
 H_Physic_Sec_610: - Leg_state_98 + 999999 H_122 <= 999999.4
 H_Physic_Sec_611: Leg_state_99 + 999999 H_122 <= 999999.1
 H_Physic_Sec_612: - Leg_state_99 + 999999 H_122 <= 999998.9
 H_Physic_Sec_613: Leg_state_97 + 999999 H_123 <= 1.000000825e+06
 H_Physic_Sec_614: - Leg_state_97 + 999999 H_123 <= 999997.675
 H_Physic_Sec_615: Leg_state_98 + 999999 H_123 <= 999999.4
 H_Physic_Sec_616: - Leg_state_98 + 999999 H_123 <= 999999.4
 H_Physic_Sec_617: Leg_state_99 + 999999 H_123 <= 999999.2
 H_Physic_Sec_618: - Leg_state_99 + 999999 H_123 <= 999998.8
 H_Physic_Sec_619: Leg_state_97 + 999999 H_124 <= 1.00000135e+06
 H_Physic_Sec_620: - Leg_state_97 + 999999 H_124 <= 999997.15
 H_Physic_Sec_621: Leg_state_98 + 999999 H_124 <= 999999.4
 H_Physic_Sec_622: - Leg_state_98 + 999999 H_124 <= 999999.4
 H_Physic_Sec_623: Leg_state_99 + 999999 H_124 <= 999999.1
 H_Physic_Sec_624: - Leg_state_99 + 999999 H_124 <= 999998.9
 H_Physic_Sec_625: Leg_state_97 + 999999 H_125 <= 1.0000026e+06
 H_Physic_Sec_626: - Leg_state_97 + 999999 H_125 <= 999996.6
 H_Physic_Sec_627: Leg_state_98 + 999999 H_125 <= 999999.4
 H_Physic_Sec_628: - Leg_state_98 + 999999 H_125 <= 999999.4
 H_Physic_Sec_629: Leg_state_99 + 999999 H_125 <= 999999
 H_Physic_Sec_630: - Leg_state_99 + 999999 H_125 <= 999999
 H_Physic_Sec_631: Leg_state_101 + 999999 H_126 <= 999999.475
 H_Physic_Sec_632: - Leg_state_101 + 999999 H_126 <= 999999.475
 H_Physic_Sec_633: Leg_state_102 + 999999 H_126 <= 999999.4
 H_Physic_Sec_634: - Leg_state_102 + 999999 H_126 <= 999999.4
 H_Physic_Sec_635: Leg_state_103 + 999999 H_126 <= 999999
 H_Physic_Sec_636: - Leg_state_103 + 999999 H_126 <= 999999
 H_Physic_Sec_637: Leg_state_101 + 999999 H_127 <= 1.0000003e+06
 H_Physic_Sec_638: - Leg_state_101 + 999999 H_127 <= 999998.5
 H_Physic_Sec_639: Leg_state_102 + 999999 H_127 <= 999999.4
 H_Physic_Sec_640: - Leg_state_102 + 999999 H_127 <= 999999.4
 H_Physic_Sec_641: Leg_state_103 + 999999 H_127 <= 999999.1
 H_Physic_Sec_642: - Leg_state_103 + 999999 H_127 <= 999998.9
 H_Physic_Sec_643: Leg_state_101 + 999999 H_128 <= 1.000000825e+06
 H_Physic_Sec_644: - Leg_state_101 + 999999 H_128 <= 999997.675
 H_Physic_Sec_645: Leg_state_102 + 999999 H_128 <= 999999.4
 H_Physic_Sec_646: - Leg_state_102 + 999999 H_128 <= 999999.4
 H_Physic_Sec_647: Leg_state_103 + 999999 H_128 <= 999999.2
 H_Physic_Sec_648: - Leg_state_103 + 999999 H_128 <= 999998.8
 H_Physic_Sec_649: Leg_state_101 + 999999 H_129 <= 1.00000135e+06
 H_Physic_Sec_650: - Leg_state_101 + 999999 H_129 <= 999997.15
 H_Physic_Sec_651: Leg_state_102 + 999999 H_129 <= 999999.4
 H_Physic_Sec_652: - Leg_state_102 + 999999 H_129 <= 999999.4
 H_Physic_Sec_653: Leg_state_103 + 999999 H_129 <= 999999.1
 H_Physic_Sec_654: - Leg_state_103 + 999999 H_129 <= 999998.9
 H_Physic_Sec_655: Leg_state_101 + 999999 H_130 <= 1.0000026e+06
 H_Physic_Sec_656: - Leg_state_101 + 999999 H_130 <= 999996.6
 H_Physic_Sec_657: Leg_state_102 + 999999 H_130 <= 999999.4
 H_Physic_Sec_658: - Leg_state_102 + 999999 H_130 <= 999999.4
 H_Physic_Sec_659: Leg_state_103 + 999999 H_130 <= 999999
 H_Physic_Sec_660: - Leg_state_103 + 999999 H_130 <= 999999
 H_Physic_Sec_661: Leg_state_105 + 999999 H_131 <= 999999.475
 H_Physic_Sec_662: - Leg_state_105 + 999999 H_131 <= 999999.475
 H_Physic_Sec_663: Leg_state_106 + 999999 H_131 <= 999999.4
 H_Physic_Sec_664: - Leg_state_106 + 999999 H_131 <= 999999.4
 H_Physic_Sec_665: Leg_state_107 + 999999 H_131 <= 999999
 H_Physic_Sec_666: - Leg_state_107 + 999999 H_131 <= 999999
 H_Physic_Sec_667: Leg_state_105 + 999999 H_132 <= 1.0000003e+06
 H_Physic_Sec_668: - Leg_state_105 + 999999 H_132 <= 999998.5
 H_Physic_Sec_669: Leg_state_106 + 999999 H_132 <= 999999.4
 H_Physic_Sec_670: - Leg_state_106 + 999999 H_132 <= 999999.4
 H_Physic_Sec_671: Leg_state_107 + 999999 H_132 <= 999999.1
 H_Physic_Sec_672: - Leg_state_107 + 999999 H_132 <= 999998.9
 H_Physic_Sec_673: Leg_state_105 + 999999 H_133 <= 1.000000825e+06
 H_Physic_Sec_674: - Leg_state_105 + 999999 H_133 <= 999997.675
 H_Physic_Sec_675: Leg_state_106 + 999999 H_133 <= 999999.4
 H_Physic_Sec_676: - Leg_state_106 + 999999 H_133 <= 999999.4
 H_Physic_Sec_677: Leg_state_107 + 999999 H_133 <= 999999.2
 H_Physic_Sec_678: - Leg_state_107 + 999999 H_133 <= 999998.8
 H_Physic_Sec_679: Leg_state_105 + 999999 H_134 <= 1.00000135e+06
 H_Physic_Sec_680: - Leg_state_105 + 999999 H_134 <= 999997.15
 H_Physic_Sec_681: Leg_state_106 + 999999 H_134 <= 999999.4
 H_Physic_Sec_682: - Leg_state_106 + 999999 H_134 <= 999999.4
 H_Physic_Sec_683: Leg_state_107 + 999999 H_134 <= 999999.1
 H_Physic_Sec_684: - Leg_state_107 + 999999 H_134 <= 999998.9
 H_Physic_Sec_685: Leg_state_105 + 999999 H_135 <= 1.0000026e+06
 H_Physic_Sec_686: - Leg_state_105 + 999999 H_135 <= 999996.6
 H_Physic_Sec_687: Leg_state_106 + 999999 H_135 <= 999999.4
 H_Physic_Sec_688: - Leg_state_106 + 999999 H_135 <= 999999.4
 H_Physic_Sec_689: Leg_state_107 + 999999 H_135 <= 999999
 H_Physic_Sec_690: - Leg_state_107 + 999999 H_135 <= 999999
 H_Physic_Sec_691: Leg_state_109 + 999999 H_136 <= 999999.475
 H_Physic_Sec_692: - Leg_state_109 + 999999 H_136 <= 999999.475
 H_Physic_Sec_693: Leg_state_110 + 999999 H_136 <= 999999.4
 H_Physic_Sec_694: - Leg_state_110 + 999999 H_136 <= 999999.4
 H_Physic_Sec_695: Leg_state_111 + 999999 H_136 <= 999999
 H_Physic_Sec_696: - Leg_state_111 + 999999 H_136 <= 999999
 H_Physic_Sec_697: Leg_state_109 + 999999 H_137 <= 1.0000003e+06
 H_Physic_Sec_698: - Leg_state_109 + 999999 H_137 <= 999998.5
 H_Physic_Sec_699: Leg_state_110 + 999999 H_137 <= 999999.4
 H_Physic_Sec_700: - Leg_state_110 + 999999 H_137 <= 999999.4
 H_Physic_Sec_701: Leg_state_111 + 999999 H_137 <= 999999.1
 H_Physic_Sec_702: - Leg_state_111 + 999999 H_137 <= 999998.9
 H_Physic_Sec_703: Leg_state_109 + 999999 H_138 <= 1.000000825e+06
 H_Physic_Sec_704: - Leg_state_109 + 999999 H_138 <= 999997.675
 H_Physic_Sec_705: Leg_state_110 + 999999 H_138 <= 999999.4
 H_Physic_Sec_706: - Leg_state_110 + 999999 H_138 <= 999999.4
 H_Physic_Sec_707: Leg_state_111 + 999999 H_138 <= 999999.2
 H_Physic_Sec_708: - Leg_state_111 + 999999 H_138 <= 999998.8
 H_Physic_Sec_709: Leg_state_109 + 999999 H_139 <= 1.00000135e+06
 H_Physic_Sec_710: - Leg_state_109 + 999999 H_139 <= 999997.15
 H_Physic_Sec_711: Leg_state_110 + 999999 H_139 <= 999999.4
 H_Physic_Sec_712: - Leg_state_110 + 999999 H_139 <= 999999.4
 H_Physic_Sec_713: Leg_state_111 + 999999 H_139 <= 999999.1
 H_Physic_Sec_714: - Leg_state_111 + 999999 H_139 <= 999998.9
 H_Physic_Sec_715: Leg_state_109 + 999999 H_140 <= 1.0000026e+06
 H_Physic_Sec_716: - Leg_state_109 + 999999 H_140 <= 999996.6
 H_Physic_Sec_717: Leg_state_110 + 999999 H_140 <= 999999.4
 H_Physic_Sec_718: - Leg_state_110 + 999999 H_140 <= 999999.4
 H_Physic_Sec_719: Leg_state_111 + 999999 H_140 <= 999999
 H_Physic_Sec_720: - Leg_state_111 + 999999 H_140 <= 999999
 H_Physic_Sec_721: Leg_state_113 + 999999 H_141 <= 999999.475
 H_Physic_Sec_722: - Leg_state_113 + 999999 H_141 <= 999999.475
 H_Physic_Sec_723: Leg_state_114 + 999999 H_141 <= 999999.4
 H_Physic_Sec_724: - Leg_state_114 + 999999 H_141 <= 999999.4
 H_Physic_Sec_725: Leg_state_115 + 999999 H_141 <= 999999
 H_Physic_Sec_726: - Leg_state_115 + 999999 H_141 <= 999999
 H_Physic_Sec_727: Leg_state_113 + 999999 H_142 <= 1.0000003e+06
 H_Physic_Sec_728: - Leg_state_113 + 999999 H_142 <= 999998.5
 H_Physic_Sec_729: Leg_state_114 + 999999 H_142 <= 999999.4
 H_Physic_Sec_730: - Leg_state_114 + 999999 H_142 <= 999999.4
 H_Physic_Sec_731: Leg_state_115 + 999999 H_142 <= 999999.1
 H_Physic_Sec_732: - Leg_state_115 + 999999 H_142 <= 999998.9
 H_Physic_Sec_733: Leg_state_113 + 999999 H_143 <= 1.000000825e+06
 H_Physic_Sec_734: - Leg_state_113 + 999999 H_143 <= 999997.675
 H_Physic_Sec_735: Leg_state_114 + 999999 H_143 <= 999999.4
 H_Physic_Sec_736: - Leg_state_114 + 999999 H_143 <= 999999.4
 H_Physic_Sec_737: Leg_state_115 + 999999 H_143 <= 999999.2
 H_Physic_Sec_738: - Leg_state_115 + 999999 H_143 <= 999998.8
 H_Physic_Sec_739: Leg_state_113 + 999999 H_144 <= 1.00000135e+06
 H_Physic_Sec_740: - Leg_state_113 + 999999 H_144 <= 999997.15
 H_Physic_Sec_741: Leg_state_114 + 999999 H_144 <= 999999.4
 H_Physic_Sec_742: - Leg_state_114 + 999999 H_144 <= 999999.4
 H_Physic_Sec_743: Leg_state_115 + 999999 H_144 <= 999999.1
 H_Physic_Sec_744: - Leg_state_115 + 999999 H_144 <= 999998.9
 H_Physic_Sec_745: Leg_state_113 + 999999 H_145 <= 1.0000026e+06
 H_Physic_Sec_746: - Leg_state_113 + 999999 H_145 <= 999996.6
 H_Physic_Sec_747: Leg_state_114 + 999999 H_145 <= 999999.4
 H_Physic_Sec_748: - Leg_state_114 + 999999 H_145 <= 999999.4
 H_Physic_Sec_749: Leg_state_115 + 999999 H_145 <= 999999
 H_Physic_Sec_750: - Leg_state_115 + 999999 H_145 <= 999999
 H_Physic_Sec_751: Leg_state_117 + 999999 H_146 <= 999999.475
 H_Physic_Sec_752: - Leg_state_117 + 999999 H_146 <= 999999.475
 H_Physic_Sec_753: Leg_state_118 + 999999 H_146 <= 999999.4
 H_Physic_Sec_754: - Leg_state_118 + 999999 H_146 <= 999999.4
 H_Physic_Sec_755: Leg_state_119 + 999999 H_146 <= 999999
 H_Physic_Sec_756: - Leg_state_119 + 999999 H_146 <= 999999
 H_Physic_Sec_757: Leg_state_117 + 999999 H_147 <= 1.0000003e+06
 H_Physic_Sec_758: - Leg_state_117 + 999999 H_147 <= 999998.5
 H_Physic_Sec_759: Leg_state_118 + 999999 H_147 <= 999999.4
 H_Physic_Sec_760: - Leg_state_118 + 999999 H_147 <= 999999.4
 H_Physic_Sec_761: Leg_state_119 + 999999 H_147 <= 999999.1
 H_Physic_Sec_762: - Leg_state_119 + 999999 H_147 <= 999998.9
 H_Physic_Sec_763: Leg_state_117 + 999999 H_148 <= 1.000000825e+06
 H_Physic_Sec_764: - Leg_state_117 + 999999 H_148 <= 999997.675
 H_Physic_Sec_765: Leg_state_118 + 999999 H_148 <= 999999.4
 H_Physic_Sec_766: - Leg_state_118 + 999999 H_148 <= 999999.4
 H_Physic_Sec_767: Leg_state_119 + 999999 H_148 <= 999999.2
 H_Physic_Sec_768: - Leg_state_119 + 999999 H_148 <= 999998.8
 H_Physic_Sec_769: Leg_state_117 + 999999 H_149 <= 1.00000135e+06
 H_Physic_Sec_770: - Leg_state_117 + 999999 H_149 <= 999997.15
 H_Physic_Sec_771: Leg_state_118 + 999999 H_149 <= 999999.4
 H_Physic_Sec_772: - Leg_state_118 + 999999 H_149 <= 999999.4
 H_Physic_Sec_773: Leg_state_119 + 999999 H_149 <= 999999.1
 H_Physic_Sec_774: - Leg_state_119 + 999999 H_149 <= 999998.9
 H_Physic_Sec_775: Leg_state_117 + 999999 H_150 <= 1.0000026e+06
 H_Physic_Sec_776: - Leg_state_117 + 999999 H_150 <= 999996.6
 H_Physic_Sec_777: Leg_state_118 + 999999 H_150 <= 999999.4
 H_Physic_Sec_778: - Leg_state_118 + 999999 H_150 <= 999999.4
 H_Physic_Sec_779: Leg_state_119 + 999999 H_150 <= 999999
 H_Physic_Sec_780: - Leg_state_119 + 999999 H_150 <= 999999
 H_Physic_Sec_781: Leg_state_121 + 999999 H_151 <= 999999.475
 H_Physic_Sec_782: - Leg_state_121 + 999999 H_151 <= 999999.475
 H_Physic_Sec_783: Leg_state_122 + 999999 H_151 <= 999999.4
 H_Physic_Sec_784: - Leg_state_122 + 999999 H_151 <= 999999.4
 H_Physic_Sec_785: Leg_state_123 + 999999 H_151 <= 999999
 H_Physic_Sec_786: - Leg_state_123 + 999999 H_151 <= 999999
 H_Physic_Sec_787: Leg_state_121 + 999999 H_152 <= 1.0000003e+06
 H_Physic_Sec_788: - Leg_state_121 + 999999 H_152 <= 999998.5
 H_Physic_Sec_789: Leg_state_122 + 999999 H_152 <= 999999.4
 H_Physic_Sec_790: - Leg_state_122 + 999999 H_152 <= 999999.4
 H_Physic_Sec_791: Leg_state_123 + 999999 H_152 <= 999999.1
 H_Physic_Sec_792: - Leg_state_123 + 999999 H_152 <= 999998.9
 H_Physic_Sec_793: Leg_state_121 + 999999 H_153 <= 1.000000825e+06
 H_Physic_Sec_794: - Leg_state_121 + 999999 H_153 <= 999997.675
 H_Physic_Sec_795: Leg_state_122 + 999999 H_153 <= 999999.4
 H_Physic_Sec_796: - Leg_state_122 + 999999 H_153 <= 999999.4
 H_Physic_Sec_797: Leg_state_123 + 999999 H_153 <= 999999.2
 H_Physic_Sec_798: - Leg_state_123 + 999999 H_153 <= 999998.8
 H_Physic_Sec_799: Leg_state_121 + 999999 H_154 <= 1.00000135e+06
 H_Physic_Sec_800: - Leg_state_121 + 999999 H_154 <= 999997.15
 H_Physic_Sec_801: Leg_state_122 + 999999 H_154 <= 999999.4
 H_Physic_Sec_802: - Leg_state_122 + 999999 H_154 <= 999999.4
 H_Physic_Sec_803: Leg_state_123 + 999999 H_154 <= 999999.1
 H_Physic_Sec_804: - Leg_state_123 + 999999 H_154 <= 999998.9
 H_Physic_Sec_805: Leg_state_121 + 999999 H_155 <= 1.0000026e+06
 H_Physic_Sec_806: - Leg_state_121 + 999999 H_155 <= 999996.6
 H_Physic_Sec_807: Leg_state_122 + 999999 H_155 <= 999999.4
 H_Physic_Sec_808: - Leg_state_122 + 999999 H_155 <= 999999.4
 H_Physic_Sec_809: Leg_state_123 + 999999 H_155 <= 999999
 H_Physic_Sec_810: - Leg_state_123 + 999999 H_155 <= 999999
 H_Physic_Sec_811: Leg_state_125 + 999999 H_156 <= 999999.475
 H_Physic_Sec_812: - Leg_state_125 + 999999 H_156 <= 999999.475
 H_Physic_Sec_813: Leg_state_126 + 999999 H_156 <= 999999.4
 H_Physic_Sec_814: - Leg_state_126 + 999999 H_156 <= 999999.4
 H_Physic_Sec_815: Leg_state_127 + 999999 H_156 <= 999999
 H_Physic_Sec_816: - Leg_state_127 + 999999 H_156 <= 999999
 H_Physic_Sec_817: Leg_state_125 + 999999 H_157 <= 1.0000003e+06
 H_Physic_Sec_818: - Leg_state_125 + 999999 H_157 <= 999998.5
 H_Physic_Sec_819: Leg_state_126 + 999999 H_157 <= 999999.4
 H_Physic_Sec_820: - Leg_state_126 + 999999 H_157 <= 999999.4
 H_Physic_Sec_821: Leg_state_127 + 999999 H_157 <= 999999.1
 H_Physic_Sec_822: - Leg_state_127 + 999999 H_157 <= 999998.9
 H_Physic_Sec_823: Leg_state_125 + 999999 H_158 <= 1.000000825e+06
 H_Physic_Sec_824: - Leg_state_125 + 999999 H_158 <= 999997.675
 H_Physic_Sec_825: Leg_state_126 + 999999 H_158 <= 999999.4
 H_Physic_Sec_826: - Leg_state_126 + 999999 H_158 <= 999999.4
 H_Physic_Sec_827: Leg_state_127 + 999999 H_158 <= 999999.2
 H_Physic_Sec_828: - Leg_state_127 + 999999 H_158 <= 999998.8
 H_Physic_Sec_829: Leg_state_125 + 999999 H_159 <= 1.00000135e+06
 H_Physic_Sec_830: - Leg_state_125 + 999999 H_159 <= 999997.15
 H_Physic_Sec_831: Leg_state_126 + 999999 H_159 <= 999999.4
 H_Physic_Sec_832: - Leg_state_126 + 999999 H_159 <= 999999.4
 H_Physic_Sec_833: Leg_state_127 + 999999 H_159 <= 999999.1
 H_Physic_Sec_834: - Leg_state_127 + 999999 H_159 <= 999998.9
 H_Physic_Sec_835: Leg_state_125 + 999999 H_160 <= 1.0000026e+06
 H_Physic_Sec_836: - Leg_state_125 + 999999 H_160 <= 999996.6
 H_Physic_Sec_837: Leg_state_126 + 999999 H_160 <= 999999.4
 H_Physic_Sec_838: - Leg_state_126 + 999999 H_160 <= 999999.4
 H_Physic_Sec_839: Leg_state_127 + 999999 H_160 <= 999999
 H_Physic_Sec_840: - Leg_state_127 + 999999 H_160 <= 999999
 H_Physic_Sec_841: Leg_state_129 + 999999 H_161 <= 999999.475
 H_Physic_Sec_842: - Leg_state_129 + 999999 H_161 <= 999999.475
 H_Physic_Sec_843: Leg_state_130 + 999999 H_161 <= 999999.4
 H_Physic_Sec_844: - Leg_state_130 + 999999 H_161 <= 999999.4
 H_Physic_Sec_845: Leg_state_131 + 999999 H_161 <= 999999
 H_Physic_Sec_846: - Leg_state_131 + 999999 H_161 <= 999999
 H_Physic_Sec_847: Leg_state_129 + 999999 H_162 <= 1.0000003e+06
 H_Physic_Sec_848: - Leg_state_129 + 999999 H_162 <= 999998.5
 H_Physic_Sec_849: Leg_state_130 + 999999 H_162 <= 999999.4
 H_Physic_Sec_850: - Leg_state_130 + 999999 H_162 <= 999999.4
 H_Physic_Sec_851: Leg_state_131 + 999999 H_162 <= 999999.1
 H_Physic_Sec_852: - Leg_state_131 + 999999 H_162 <= 999998.9
 H_Physic_Sec_853: Leg_state_129 + 999999 H_163 <= 1.000000825e+06
 H_Physic_Sec_854: - Leg_state_129 + 999999 H_163 <= 999997.675
 H_Physic_Sec_855: Leg_state_130 + 999999 H_163 <= 999999.4
 H_Physic_Sec_856: - Leg_state_130 + 999999 H_163 <= 999999.4
 H_Physic_Sec_857: Leg_state_131 + 999999 H_163 <= 999999.2
 H_Physic_Sec_858: - Leg_state_131 + 999999 H_163 <= 999998.8
 H_Physic_Sec_859: Leg_state_129 + 999999 H_164 <= 1.00000135e+06
 H_Physic_Sec_860: - Leg_state_129 + 999999 H_164 <= 999997.15
 H_Physic_Sec_861: Leg_state_130 + 999999 H_164 <= 999999.4
 H_Physic_Sec_862: - Leg_state_130 + 999999 H_164 <= 999999.4
 H_Physic_Sec_863: Leg_state_131 + 999999 H_164 <= 999999.1
 H_Physic_Sec_864: - Leg_state_131 + 999999 H_164 <= 999998.9
 H_Physic_Sec_865: Leg_state_129 + 999999 H_165 <= 1.0000026e+06
 H_Physic_Sec_866: - Leg_state_129 + 999999 H_165 <= 999996.6
 H_Physic_Sec_867: Leg_state_130 + 999999 H_165 <= 999999.4
 H_Physic_Sec_868: - Leg_state_130 + 999999 H_165 <= 999999.4
 H_Physic_Sec_869: Leg_state_131 + 999999 H_165 <= 999999
 H_Physic_Sec_870: - Leg_state_131 + 999999 H_165 <= 999999
 H_Physic_Sec_871: Leg_state_133 + 999999 H_166 <= 999999.475
 H_Physic_Sec_872: - Leg_state_133 + 999999 H_166 <= 999999.475
 H_Physic_Sec_873: Leg_state_134 + 999999 H_166 <= 999999.4
 H_Physic_Sec_874: - Leg_state_134 + 999999 H_166 <= 999999.4
 H_Physic_Sec_875: Leg_state_135 + 999999 H_166 <= 999999
 H_Physic_Sec_876: - Leg_state_135 + 999999 H_166 <= 999999
 H_Physic_Sec_877: Leg_state_133 + 999999 H_167 <= 1.0000003e+06
 H_Physic_Sec_878: - Leg_state_133 + 999999 H_167 <= 999998.5
 H_Physic_Sec_879: Leg_state_134 + 999999 H_167 <= 999999.4
 H_Physic_Sec_880: - Leg_state_134 + 999999 H_167 <= 999999.4
 H_Physic_Sec_881: Leg_state_135 + 999999 H_167 <= 999999.1
 H_Physic_Sec_882: - Leg_state_135 + 999999 H_167 <= 999998.9
 H_Physic_Sec_883: Leg_state_133 + 999999 H_168 <= 1.000000825e+06
 H_Physic_Sec_884: - Leg_state_133 + 999999 H_168 <= 999997.675
 H_Physic_Sec_885: Leg_state_134 + 999999 H_168 <= 999999.4
 H_Physic_Sec_886: - Leg_state_134 + 999999 H_168 <= 999999.4
 H_Physic_Sec_887: Leg_state_135 + 999999 H_168 <= 999999.2
 H_Physic_Sec_888: - Leg_state_135 + 999999 H_168 <= 999998.8
 H_Physic_Sec_889: Leg_state_133 + 999999 H_169 <= 1.00000135e+06
 H_Physic_Sec_890: - Leg_state_133 + 999999 H_169 <= 999997.15
 H_Physic_Sec_891: Leg_state_134 + 999999 H_169 <= 999999.4
 H_Physic_Sec_892: - Leg_state_134 + 999999 H_169 <= 999999.4
 H_Physic_Sec_893: Leg_state_135 + 999999 H_169 <= 999999.1
 H_Physic_Sec_894: - Leg_state_135 + 999999 H_169 <= 999998.9
 H_Physic_Sec_895: Leg_state_133 + 999999 H_170 <= 1.0000026e+06
 H_Physic_Sec_896: - Leg_state_133 + 999999 H_170 <= 999996.6
 H_Physic_Sec_897: Leg_state_134 + 999999 H_170 <= 999999.4
 H_Physic_Sec_898: - Leg_state_134 + 999999 H_170 <= 999999.4
 H_Physic_Sec_899: Leg_state_135 + 999999 H_170 <= 999999
 H_Physic_Sec_900: - Leg_state_135 + 999999 H_170 <= 999999
 H_Physic_Sec_901: Leg_state_137 + 999999 H_171 <= 999999.475
 H_Physic_Sec_902: - Leg_state_137 + 999999 H_171 <= 999999.475
 H_Physic_Sec_903: Leg_state_138 + 999999 H_171 <= 999999.4
 H_Physic_Sec_904: - Leg_state_138 + 999999 H_171 <= 999999.4
 H_Physic_Sec_905: Leg_state_139 + 999999 H_171 <= 999999
 H_Physic_Sec_906: - Leg_state_139 + 999999 H_171 <= 999999
 H_Physic_Sec_907: Leg_state_137 + 999999 H_172 <= 1.0000003e+06
 H_Physic_Sec_908: - Leg_state_137 + 999999 H_172 <= 999998.5
 H_Physic_Sec_909: Leg_state_138 + 999999 H_172 <= 999999.4
 H_Physic_Sec_910: - Leg_state_138 + 999999 H_172 <= 999999.4
 H_Physic_Sec_911: Leg_state_139 + 999999 H_172 <= 999999.1
 H_Physic_Sec_912: - Leg_state_139 + 999999 H_172 <= 999998.9
 H_Physic_Sec_913: Leg_state_137 + 999999 H_173 <= 1.000000825e+06
 H_Physic_Sec_914: - Leg_state_137 + 999999 H_173 <= 999997.675
 H_Physic_Sec_915: Leg_state_138 + 999999 H_173 <= 999999.4
 H_Physic_Sec_916: - Leg_state_138 + 999999 H_173 <= 999999.4
 H_Physic_Sec_917: Leg_state_139 + 999999 H_173 <= 999999.2
 H_Physic_Sec_918: - Leg_state_139 + 999999 H_173 <= 999998.8
 H_Physic_Sec_919: Leg_state_137 + 999999 H_174 <= 1.00000135e+06
 H_Physic_Sec_920: - Leg_state_137 + 999999 H_174 <= 999997.15
 H_Physic_Sec_921: Leg_state_138 + 999999 H_174 <= 999999.4
 H_Physic_Sec_922: - Leg_state_138 + 999999 H_174 <= 999999.4
 H_Physic_Sec_923: Leg_state_139 + 999999 H_174 <= 999999.1
 H_Physic_Sec_924: - Leg_state_139 + 999999 H_174 <= 999998.9
 H_Physic_Sec_925: Leg_state_137 + 999999 H_175 <= 1.0000026e+06
 H_Physic_Sec_926: - Leg_state_137 + 999999 H_175 <= 999996.6
 H_Physic_Sec_927: Leg_state_138 + 999999 H_175 <= 999999.4
 H_Physic_Sec_928: - Leg_state_138 + 999999 H_175 <= 999999.4
 H_Physic_Sec_929: Leg_state_139 + 999999 H_175 <= 999999
 H_Physic_Sec_930: - Leg_state_139 + 999999 H_175 <= 999999
 H_Physic_Sec_931: Leg_state_141 + 999999 H_176 <= 999999.475
 H_Physic_Sec_932: - Leg_state_141 + 999999 H_176 <= 999999.475
 H_Physic_Sec_933: Leg_state_142 + 999999 H_176 <= 999999.4
 H_Physic_Sec_934: - Leg_state_142 + 999999 H_176 <= 999999.4
 H_Physic_Sec_935: Leg_state_143 + 999999 H_176 <= 999999
 H_Physic_Sec_936: - Leg_state_143 + 999999 H_176 <= 999999
 H_Physic_Sec_937: Leg_state_141 + 999999 H_177 <= 1.0000003e+06
 H_Physic_Sec_938: - Leg_state_141 + 999999 H_177 <= 999998.5
 H_Physic_Sec_939: Leg_state_142 + 999999 H_177 <= 999999.4
 H_Physic_Sec_940: - Leg_state_142 + 999999 H_177 <= 999999.4
 H_Physic_Sec_941: Leg_state_143 + 999999 H_177 <= 999999.1
 H_Physic_Sec_942: - Leg_state_143 + 999999 H_177 <= 999998.9
 H_Physic_Sec_943: Leg_state_141 + 999999 H_178 <= 1.000000825e+06
 H_Physic_Sec_944: - Leg_state_141 + 999999 H_178 <= 999997.675
 H_Physic_Sec_945: Leg_state_142 + 999999 H_178 <= 999999.4
 H_Physic_Sec_946: - Leg_state_142 + 999999 H_178 <= 999999.4
 H_Physic_Sec_947: Leg_state_143 + 999999 H_178 <= 999999.2
 H_Physic_Sec_948: - Leg_state_143 + 999999 H_178 <= 999998.8
 H_Physic_Sec_949: Leg_state_141 + 999999 H_179 <= 1.00000135e+06
 H_Physic_Sec_950: - Leg_state_141 + 999999 H_179 <= 999997.15
 H_Physic_Sec_951: Leg_state_142 + 999999 H_179 <= 999999.4
 H_Physic_Sec_952: - Leg_state_142 + 999999 H_179 <= 999999.4
 H_Physic_Sec_953: Leg_state_143 + 999999 H_179 <= 999999.1
 H_Physic_Sec_954: - Leg_state_143 + 999999 H_179 <= 999998.9
 H_Physic_Sec_955: Leg_state_141 + 999999 H_180 <= 1.0000026e+06
 H_Physic_Sec_956: - Leg_state_141 + 999999 H_180 <= 999996.6
 H_Physic_Sec_957: Leg_state_142 + 999999 H_180 <= 999999.4
 H_Physic_Sec_958: - Leg_state_142 + 999999 H_180 <= 999999.4
 H_Physic_Sec_959: Leg_state_143 + 999999 H_180 <= 999999
 H_Physic_Sec_960: - Leg_state_143 + 999999 H_180 <= 999999
 H_Physic_Sec_961: Leg_state_145 + 999999 H_181 <= 999999.475
 H_Physic_Sec_962: - Leg_state_145 + 999999 H_181 <= 999999.475
 H_Physic_Sec_963: Leg_state_146 + 999999 H_181 <= 999999.4
 H_Physic_Sec_964: - Leg_state_146 + 999999 H_181 <= 999999.4
 H_Physic_Sec_965: Leg_state_147 + 999999 H_181 <= 999999
 H_Physic_Sec_966: - Leg_state_147 + 999999 H_181 <= 999999
 H_Physic_Sec_967: Leg_state_145 + 999999 H_182 <= 1.0000003e+06
 H_Physic_Sec_968: - Leg_state_145 + 999999 H_182 <= 999998.5
 H_Physic_Sec_969: Leg_state_146 + 999999 H_182 <= 999999.4
 H_Physic_Sec_970: - Leg_state_146 + 999999 H_182 <= 999999.4
 H_Physic_Sec_971: Leg_state_147 + 999999 H_182 <= 999999.1
 H_Physic_Sec_972: - Leg_state_147 + 999999 H_182 <= 999998.9
 H_Physic_Sec_973: Leg_state_145 + 999999 H_183 <= 1.000000825e+06
 H_Physic_Sec_974: - Leg_state_145 + 999999 H_183 <= 999997.675
 H_Physic_Sec_975: Leg_state_146 + 999999 H_183 <= 999999.4
 H_Physic_Sec_976: - Leg_state_146 + 999999 H_183 <= 999999.4
 H_Physic_Sec_977: Leg_state_147 + 999999 H_183 <= 999999.2
 H_Physic_Sec_978: - Leg_state_147 + 999999 H_183 <= 999998.8
 H_Physic_Sec_979: Leg_state_145 + 999999 H_184 <= 1.00000135e+06
 H_Physic_Sec_980: - Leg_state_145 + 999999 H_184 <= 999997.15
 H_Physic_Sec_981: Leg_state_146 + 999999 H_184 <= 999999.4
 H_Physic_Sec_982: - Leg_state_146 + 999999 H_184 <= 999999.4
 H_Physic_Sec_983: Leg_state_147 + 999999 H_184 <= 999999.1
 H_Physic_Sec_984: - Leg_state_147 + 999999 H_184 <= 999998.9
 H_Physic_Sec_985: Leg_state_145 + 999999 H_185 <= 1.0000026e+06
 H_Physic_Sec_986: - Leg_state_145 + 999999 H_185 <= 999996.6
 H_Physic_Sec_987: Leg_state_146 + 999999 H_185 <= 999999.4
 H_Physic_Sec_988: - Leg_state_146 + 999999 H_185 <= 999999.4
 H_Physic_Sec_989: Leg_state_147 + 999999 H_185 <= 999999
 H_Physic_Sec_990: - Leg_state_147 + 999999 H_185 <= 999999
 H_Physic_Sec_991: Leg_state_149 + 999999 H_186 <= 999999.475
 H_Physic_Sec_992: - Leg_state_149 + 999999 H_186 <= 999999.475
 H_Physic_Sec_993: Leg_state_150 + 999999 H_186 <= 999999.4
 H_Physic_Sec_994: - Leg_state_150 + 999999 H_186 <= 999999.4
 H_Physic_Sec_995: Leg_state_151 + 999999 H_186 <= 999999
 H_Physic_Sec_996: - Leg_state_151 + 999999 H_186 <= 999999
 H_Physic_Sec_997: Leg_state_149 + 999999 H_187 <= 1.0000003e+06
 H_Physic_Sec_998: - Leg_state_149 + 999999 H_187 <= 999998.5
 H_Physic_Sec_999: Leg_state_150 + 999999 H_187 <= 999999.4
 H_Physic_Sec_1000: - Leg_state_150 + 999999 H_187 <= 999999.4
 H_Physic_Sec_1001: Leg_state_151 + 999999 H_187 <= 999999.1
 H_Physic_Sec_1002: - Leg_state_151 + 999999 H_187 <= 999998.9
 H_Physic_Sec_1003: Leg_state_149 + 999999 H_188 <= 1.000000825e+06
 H_Physic_Sec_1004: - Leg_state_149 + 999999 H_188 <= 999997.675
 H_Physic_Sec_1005: Leg_state_150 + 999999 H_188 <= 999999.4
 H_Physic_Sec_1006: - Leg_state_150 + 999999 H_188 <= 999999.4
 H_Physic_Sec_1007: Leg_state_151 + 999999 H_188 <= 999999.2
 H_Physic_Sec_1008: - Leg_state_151 + 999999 H_188 <= 999998.8
 H_Physic_Sec_1009: Leg_state_149 + 999999 H_189 <= 1.00000135e+06
 H_Physic_Sec_1010: - Leg_state_149 + 999999 H_189 <= 999997.15
 H_Physic_Sec_1011: Leg_state_150 + 999999 H_189 <= 999999.4
 H_Physic_Sec_1012: - Leg_state_150 + 999999 H_189 <= 999999.4
 H_Physic_Sec_1013: Leg_state_151 + 999999 H_189 <= 999999.1
 H_Physic_Sec_1014: - Leg_state_151 + 999999 H_189 <= 999998.9
 H_Physic_Sec_1015: Leg_state_149 + 999999 H_190 <= 1.0000026e+06
 H_Physic_Sec_1016: - Leg_state_149 + 999999 H_190 <= 999996.6
 H_Physic_Sec_1017: Leg_state_150 + 999999 H_190 <= 999999.4
 H_Physic_Sec_1018: - Leg_state_150 + 999999 H_190 <= 999999.4
 H_Physic_Sec_1019: Leg_state_151 + 999999 H_190 <= 999999
 H_Physic_Sec_1020: - Leg_state_151 + 999999 H_190 <= 999999
 H_Physic_Sec_1021: Leg_state_153 + 999999 H_191 <= 999999.475
 H_Physic_Sec_1022: - Leg_state_153 + 999999 H_191 <= 999999.475
 H_Physic_Sec_1023: Leg_state_154 + 999999 H_191 <= 999999.4
 H_Physic_Sec_1024: - Leg_state_154 + 999999 H_191 <= 999999.4
 H_Physic_Sec_1025: Leg_state_155 + 999999 H_191 <= 999999
 H_Physic_Sec_1026: - Leg_state_155 + 999999 H_191 <= 999999
 H_Physic_Sec_1027: Leg_state_153 + 999999 H_192 <= 1.0000003e+06
 H_Physic_Sec_1028: - Leg_state_153 + 999999 H_192 <= 999998.5
 H_Physic_Sec_1029: Leg_state_154 + 999999 H_192 <= 999999.4
 H_Physic_Sec_1030: - Leg_state_154 + 999999 H_192 <= 999999.4
 H_Physic_Sec_1031: Leg_state_155 + 999999 H_192 <= 999999.1
 H_Physic_Sec_1032: - Leg_state_155 + 999999 H_192 <= 999998.9
 H_Physic_Sec_1033: Leg_state_153 + 999999 H_193 <= 1.000000825e+06
 H_Physic_Sec_1034: - Leg_state_153 + 999999 H_193 <= 999997.675
 H_Physic_Sec_1035: Leg_state_154 + 999999 H_193 <= 999999.4
 H_Physic_Sec_1036: - Leg_state_154 + 999999 H_193 <= 999999.4
 H_Physic_Sec_1037: Leg_state_155 + 999999 H_193 <= 999999.2
 H_Physic_Sec_1038: - Leg_state_155 + 999999 H_193 <= 999998.8
 H_Physic_Sec_1039: Leg_state_153 + 999999 H_194 <= 1.00000135e+06
 H_Physic_Sec_1040: - Leg_state_153 + 999999 H_194 <= 999997.15
 H_Physic_Sec_1041: Leg_state_154 + 999999 H_194 <= 999999.4
 H_Physic_Sec_1042: - Leg_state_154 + 999999 H_194 <= 999999.4
 H_Physic_Sec_1043: Leg_state_155 + 999999 H_194 <= 999999.1
 H_Physic_Sec_1044: - Leg_state_155 + 999999 H_194 <= 999998.9
 H_Physic_Sec_1045: Leg_state_153 + 999999 H_195 <= 1.0000026e+06
 H_Physic_Sec_1046: - Leg_state_153 + 999999 H_195 <= 999996.6
 H_Physic_Sec_1047: Leg_state_154 + 999999 H_195 <= 999999.4
 H_Physic_Sec_1048: - Leg_state_154 + 999999 H_195 <= 999999.4
 H_Physic_Sec_1049: Leg_state_155 + 999999 H_195 <= 999999
 H_Physic_Sec_1050: - Leg_state_155 + 999999 H_195 <= 999999
 H_Physic_Sec_1051: Leg_state_157 + 999999 H_196 <= 999999.475
 H_Physic_Sec_1052: - Leg_state_157 + 999999 H_196 <= 999999.475
 H_Physic_Sec_1053: Leg_state_158 + 999999 H_196 <= 999999.4
 H_Physic_Sec_1054: - Leg_state_158 + 999999 H_196 <= 999999.4
 H_Physic_Sec_1055: Leg_state_159 + 999999 H_196 <= 999999
 H_Physic_Sec_1056: - Leg_state_159 + 999999 H_196 <= 999999
 H_Physic_Sec_1057: Leg_state_157 + 999999 H_197 <= 1.0000003e+06
 H_Physic_Sec_1058: - Leg_state_157 + 999999 H_197 <= 999998.5
 H_Physic_Sec_1059: Leg_state_158 + 999999 H_197 <= 999999.4
 H_Physic_Sec_1060: - Leg_state_158 + 999999 H_197 <= 999999.4
 H_Physic_Sec_1061: Leg_state_159 + 999999 H_197 <= 999999.1
 H_Physic_Sec_1062: - Leg_state_159 + 999999 H_197 <= 999998.9
 H_Physic_Sec_1063: Leg_state_157 + 999999 H_198 <= 1.000000825e+06
 H_Physic_Sec_1064: - Leg_state_157 + 999999 H_198 <= 999997.675
 H_Physic_Sec_1065: Leg_state_158 + 999999 H_198 <= 999999.4
 H_Physic_Sec_1066: - Leg_state_158 + 999999 H_198 <= 999999.4
 H_Physic_Sec_1067: Leg_state_159 + 999999 H_198 <= 999999.2
 H_Physic_Sec_1068: - Leg_state_159 + 999999 H_198 <= 999998.8
 H_Physic_Sec_1069: Leg_state_157 + 999999 H_199 <= 1.00000135e+06
 H_Physic_Sec_1070: - Leg_state_157 + 999999 H_199 <= 999997.15
 H_Physic_Sec_1071: Leg_state_158 + 999999 H_199 <= 999999.4
 H_Physic_Sec_1072: - Leg_state_158 + 999999 H_199 <= 999999.4
 H_Physic_Sec_1073: Leg_state_159 + 999999 H_199 <= 999999.1
 H_Physic_Sec_1074: - Leg_state_159 + 999999 H_199 <= 999998.9
 H_Physic_Sec_1075: Leg_state_157 + 999999 H_200 <= 1.0000026e+06
 H_Physic_Sec_1076: - Leg_state_157 + 999999 H_200 <= 999996.6
 H_Physic_Sec_1077: Leg_state_158 + 999999 H_200 <= 999999.4
 H_Physic_Sec_1078: - Leg_state_158 + 999999 H_200 <= 999999.4
 H_Physic_Sec_1079: Leg_state_159 + 999999 H_200 <= 999999
 H_Physic_Sec_1080: - Leg_state_159 + 999999 H_200 <= 999999
 H_Physic_Sec_1081: Leg_state_161 + 999999 H_201 <= 999999.475
 H_Physic_Sec_1082: - Leg_state_161 + 999999 H_201 <= 999999.475
 H_Physic_Sec_1083: Leg_state_162 + 999999 H_201 <= 999999.4
 H_Physic_Sec_1084: - Leg_state_162 + 999999 H_201 <= 999999.4
 H_Physic_Sec_1085: Leg_state_163 + 999999 H_201 <= 999999
 H_Physic_Sec_1086: - Leg_state_163 + 999999 H_201 <= 999999
 H_Physic_Sec_1087: Leg_state_161 + 999999 H_202 <= 1.0000003e+06
 H_Physic_Sec_1088: - Leg_state_161 + 999999 H_202 <= 999998.5
 H_Physic_Sec_1089: Leg_state_162 + 999999 H_202 <= 999999.4
 H_Physic_Sec_1090: - Leg_state_162 + 999999 H_202 <= 999999.4
 H_Physic_Sec_1091: Leg_state_163 + 999999 H_202 <= 999999.1
 H_Physic_Sec_1092: - Leg_state_163 + 999999 H_202 <= 999998.9
 H_Physic_Sec_1093: Leg_state_161 + 999999 H_203 <= 1.000000825e+06
 H_Physic_Sec_1094: - Leg_state_161 + 999999 H_203 <= 999997.675
 H_Physic_Sec_1095: Leg_state_162 + 999999 H_203 <= 999999.4
 H_Physic_Sec_1096: - Leg_state_162 + 999999 H_203 <= 999999.4
 H_Physic_Sec_1097: Leg_state_163 + 999999 H_203 <= 999999.2
 H_Physic_Sec_1098: - Leg_state_163 + 999999 H_203 <= 999998.8
 H_Physic_Sec_1099: Leg_state_161 + 999999 H_204 <= 1.00000135e+06
 H_Physic_Sec_1100: - Leg_state_161 + 999999 H_204 <= 999997.15
 H_Physic_Sec_1101: Leg_state_162 + 999999 H_204 <= 999999.4
 H_Physic_Sec_1102: - Leg_state_162 + 999999 H_204 <= 999999.4
 H_Physic_Sec_1103: Leg_state_163 + 999999 H_204 <= 999999.1
 H_Physic_Sec_1104: - Leg_state_163 + 999999 H_204 <= 999998.9
 H_Physic_Sec_1105: Leg_state_161 + 999999 H_205 <= 1.0000026e+06
 H_Physic_Sec_1106: - Leg_state_161 + 999999 H_205 <= 999996.6
 H_Physic_Sec_1107: Leg_state_162 + 999999 H_205 <= 999999.4
 H_Physic_Sec_1108: - Leg_state_162 + 999999 H_205 <= 999999.4
 H_Physic_Sec_1109: Leg_state_163 + 999999 H_205 <= 999999
 H_Physic_Sec_1110: - Leg_state_163 + 999999 H_205 <= 999999
 H_Physic_Sec_1111: Leg_state_165 + 999999 H_206 <= 999999.475
 H_Physic_Sec_1112: - Leg_state_165 + 999999 H_206 <= 999999.475
 H_Physic_Sec_1113: Leg_state_166 + 999999 H_206 <= 999999.4
 H_Physic_Sec_1114: - Leg_state_166 + 999999 H_206 <= 999999.4
 H_Physic_Sec_1115: Leg_state_167 + 999999 H_206 <= 999999
 H_Physic_Sec_1116: - Leg_state_167 + 999999 H_206 <= 999999
 H_Physic_Sec_1117: Leg_state_165 + 999999 H_207 <= 1.0000003e+06
 H_Physic_Sec_1118: - Leg_state_165 + 999999 H_207 <= 999998.5
 H_Physic_Sec_1119: Leg_state_166 + 999999 H_207 <= 999999.4
 H_Physic_Sec_1120: - Leg_state_166 + 999999 H_207 <= 999999.4
 H_Physic_Sec_1121: Leg_state_167 + 999999 H_207 <= 999999.1
 H_Physic_Sec_1122: - Leg_state_167 + 999999 H_207 <= 999998.9
 H_Physic_Sec_1123: Leg_state_165 + 999999 H_208 <= 1.000000825e+06
 H_Physic_Sec_1124: - Leg_state_165 + 999999 H_208 <= 999997.675
 H_Physic_Sec_1125: Leg_state_166 + 999999 H_208 <= 999999.4
 H_Physic_Sec_1126: - Leg_state_166 + 999999 H_208 <= 999999.4
 H_Physic_Sec_1127: Leg_state_167 + 999999 H_208 <= 999999.2
 H_Physic_Sec_1128: - Leg_state_167 + 999999 H_208 <= 999998.8
 H_Physic_Sec_1129: Leg_state_165 + 999999 H_209 <= 1.00000135e+06
 H_Physic_Sec_1130: - Leg_state_165 + 999999 H_209 <= 999997.15
 H_Physic_Sec_1131: Leg_state_166 + 999999 H_209 <= 999999.4
 H_Physic_Sec_1132: - Leg_state_166 + 999999 H_209 <= 999999.4
 H_Physic_Sec_1133: Leg_state_167 + 999999 H_209 <= 999999.1
 H_Physic_Sec_1134: - Leg_state_167 + 999999 H_209 <= 999998.9
 H_Physic_Sec_1135: Leg_state_165 + 999999 H_210 <= 1.0000026e+06
 H_Physic_Sec_1136: - Leg_state_165 + 999999 H_210 <= 999996.6
 H_Physic_Sec_1137: Leg_state_166 + 999999 H_210 <= 999999.4
 H_Physic_Sec_1138: - Leg_state_166 + 999999 H_210 <= 999999.4
 H_Physic_Sec_1139: Leg_state_167 + 999999 H_210 <= 999999
 H_Physic_Sec_1140: - Leg_state_167 + 999999 H_210 <= 999999
 H_Physic_Sec_1141: Leg_state_169 + 999999 H_211 <= 999999.475
 H_Physic_Sec_1142: - Leg_state_169 + 999999 H_211 <= 999999.475
 H_Physic_Sec_1143: Leg_state_170 + 999999 H_211 <= 999999.4
 H_Physic_Sec_1144: - Leg_state_170 + 999999 H_211 <= 999999.4
 H_Physic_Sec_1145: Leg_state_171 + 999999 H_211 <= 999999
 H_Physic_Sec_1146: - Leg_state_171 + 999999 H_211 <= 999999
 H_Physic_Sec_1147: Leg_state_169 + 999999 H_212 <= 1.0000003e+06
 H_Physic_Sec_1148: - Leg_state_169 + 999999 H_212 <= 999998.5
 H_Physic_Sec_1149: Leg_state_170 + 999999 H_212 <= 999999.4
 H_Physic_Sec_1150: - Leg_state_170 + 999999 H_212 <= 999999.4
 H_Physic_Sec_1151: Leg_state_171 + 999999 H_212 <= 999999.1
 H_Physic_Sec_1152: - Leg_state_171 + 999999 H_212 <= 999998.9
 H_Physic_Sec_1153: Leg_state_169 + 999999 H_213 <= 1.000000825e+06
 H_Physic_Sec_1154: - Leg_state_169 + 999999 H_213 <= 999997.675
 H_Physic_Sec_1155: Leg_state_170 + 999999 H_213 <= 999999.4
 H_Physic_Sec_1156: - Leg_state_170 + 999999 H_213 <= 999999.4
 H_Physic_Sec_1157: Leg_state_171 + 999999 H_213 <= 999999.2
 H_Physic_Sec_1158: - Leg_state_171 + 999999 H_213 <= 999998.8
 H_Physic_Sec_1159: Leg_state_169 + 999999 H_214 <= 1.00000135e+06
 H_Physic_Sec_1160: - Leg_state_169 + 999999 H_214 <= 999997.15
 H_Physic_Sec_1161: Leg_state_170 + 999999 H_214 <= 999999.4
 H_Physic_Sec_1162: - Leg_state_170 + 999999 H_214 <= 999999.4
 H_Physic_Sec_1163: Leg_state_171 + 999999 H_214 <= 999999.1
 H_Physic_Sec_1164: - Leg_state_171 + 999999 H_214 <= 999998.9
 H_Physic_Sec_1165: Leg_state_169 + 999999 H_215 <= 1.0000026e+06
 H_Physic_Sec_1166: - Leg_state_169 + 999999 H_215 <= 999996.6
 H_Physic_Sec_1167: Leg_state_170 + 999999 H_215 <= 999999.4
 H_Physic_Sec_1168: - Leg_state_170 + 999999 H_215 <= 999999.4
 H_Physic_Sec_1169: Leg_state_171 + 999999 H_215 <= 999999
 H_Physic_Sec_1170: - Leg_state_171 + 999999 H_215 <= 999999
 H_Physic_Sec_1171: Leg_state_173 + 999999 H_216 <= 999999.475
 H_Physic_Sec_1172: - Leg_state_173 + 999999 H_216 <= 999999.475
 H_Physic_Sec_1173: Leg_state_174 + 999999 H_216 <= 999999.4
 H_Physic_Sec_1174: - Leg_state_174 + 999999 H_216 <= 999999.4
 H_Physic_Sec_1175: Leg_state_175 + 999999 H_216 <= 999999
 H_Physic_Sec_1176: - Leg_state_175 + 999999 H_216 <= 999999
 H_Physic_Sec_1177: Leg_state_173 + 999999 H_217 <= 1.0000003e+06
 H_Physic_Sec_1178: - Leg_state_173 + 999999 H_217 <= 999998.5
 H_Physic_Sec_1179: Leg_state_174 + 999999 H_217 <= 999999.4
 H_Physic_Sec_1180: - Leg_state_174 + 999999 H_217 <= 999999.4
 H_Physic_Sec_1181: Leg_state_175 + 999999 H_217 <= 999999.1
 H_Physic_Sec_1182: - Leg_state_175 + 999999 H_217 <= 999998.9
 H_Physic_Sec_1183: Leg_state_173 + 999999 H_218 <= 1.000000825e+06
 H_Physic_Sec_1184: - Leg_state_173 + 999999 H_218 <= 999997.675
 H_Physic_Sec_1185: Leg_state_174 + 999999 H_218 <= 999999.4
 H_Physic_Sec_1186: - Leg_state_174 + 999999 H_218 <= 999999.4
 H_Physic_Sec_1187: Leg_state_175 + 999999 H_218 <= 999999.2
 H_Physic_Sec_1188: - Leg_state_175 + 999999 H_218 <= 999998.8
 H_Physic_Sec_1189: Leg_state_173 + 999999 H_219 <= 1.00000135e+06
 H_Physic_Sec_1190: - Leg_state_173 + 999999 H_219 <= 999997.15
 H_Physic_Sec_1191: Leg_state_174 + 999999 H_219 <= 999999.4
 H_Physic_Sec_1192: - Leg_state_174 + 999999 H_219 <= 999999.4
 H_Physic_Sec_1193: Leg_state_175 + 999999 H_219 <= 999999.1
 H_Physic_Sec_1194: - Leg_state_175 + 999999 H_219 <= 999998.9
 H_Physic_Sec_1195: Leg_state_173 + 999999 H_220 <= 1.0000026e+06
 H_Physic_Sec_1196: - Leg_state_173 + 999999 H_220 <= 999996.6
 H_Physic_Sec_1197: Leg_state_174 + 999999 H_220 <= 999999.4
 H_Physic_Sec_1198: - Leg_state_174 + 999999 H_220 <= 999999.4
 H_Physic_Sec_1199: Leg_state_175 + 999999 H_220 <= 999999
 H_Physic_Sec_1200: - Leg_state_175 + 999999 H_220 <= 999999
 H_Physic_Sec_1201: Leg_state_177 + 999999 H_221 <= 999999.475
 H_Physic_Sec_1202: - Leg_state_177 + 999999 H_221 <= 999999.475
 H_Physic_Sec_1203: Leg_state_178 + 999999 H_221 <= 999999.4
 H_Physic_Sec_1204: - Leg_state_178 + 999999 H_221 <= 999999.4
 H_Physic_Sec_1205: Leg_state_179 + 999999 H_221 <= 999999
 H_Physic_Sec_1206: - Leg_state_179 + 999999 H_221 <= 999999
 H_Physic_Sec_1207: Leg_state_177 + 999999 H_222 <= 1.0000003e+06
 H_Physic_Sec_1208: - Leg_state_177 + 999999 H_222 <= 999998.5
 H_Physic_Sec_1209: Leg_state_178 + 999999 H_222 <= 999999.4
 H_Physic_Sec_1210: - Leg_state_178 + 999999 H_222 <= 999999.4
 H_Physic_Sec_1211: Leg_state_179 + 999999 H_222 <= 999999.1
 H_Physic_Sec_1212: - Leg_state_179 + 999999 H_222 <= 999998.9
 H_Physic_Sec_1213: Leg_state_177 + 999999 H_223 <= 1.000000825e+06
 H_Physic_Sec_1214: - Leg_state_177 + 999999 H_223 <= 999997.675
 H_Physic_Sec_1215: Leg_state_178 + 999999 H_223 <= 999999.4
 H_Physic_Sec_1216: - Leg_state_178 + 999999 H_223 <= 999999.4
 H_Physic_Sec_1217: Leg_state_179 + 999999 H_223 <= 999999.2
 H_Physic_Sec_1218: - Leg_state_179 + 999999 H_223 <= 999998.8
 H_Physic_Sec_1219: Leg_state_177 + 999999 H_224 <= 1.00000135e+06
 H_Physic_Sec_1220: - Leg_state_177 + 999999 H_224 <= 999997.15
 H_Physic_Sec_1221: Leg_state_178 + 999999 H_224 <= 999999.4
 H_Physic_Sec_1222: - Leg_state_178 + 999999 H_224 <= 999999.4
 H_Physic_Sec_1223: Leg_state_179 + 999999 H_224 <= 999999.1
 H_Physic_Sec_1224: - Leg_state_179 + 999999 H_224 <= 999998.9
 H_Physic_Sec_1225: Leg_state_177 + 999999 H_225 <= 1.0000026e+06
 H_Physic_Sec_1226: - Leg_state_177 + 999999 H_225 <= 999996.6
 H_Physic_Sec_1227: Leg_state_178 + 999999 H_225 <= 999999.4
 H_Physic_Sec_1228: - Leg_state_178 + 999999 H_225 <= 999999.4
 H_Physic_Sec_1229: Leg_state_179 + 999999 H_225 <= 999999
 H_Physic_Sec_1230: - Leg_state_179 + 999999 H_225 <= 999999
 H_Physic_Sec_1231: Leg_state_181 + 999999 H_226 <= 999999.475
 H_Physic_Sec_1232: - Leg_state_181 + 999999 H_226 <= 999999.475
 H_Physic_Sec_1233: Leg_state_182 + 999999 H_226 <= 999999.4
 H_Physic_Sec_1234: - Leg_state_182 + 999999 H_226 <= 999999.4
 H_Physic_Sec_1235: Leg_state_183 + 999999 H_226 <= 999999
 H_Physic_Sec_1236: - Leg_state_183 + 999999 H_226 <= 999999
 H_Physic_Sec_1237: Leg_state_181 + 999999 H_227 <= 1.0000003e+06
 H_Physic_Sec_1238: - Leg_state_181 + 999999 H_227 <= 999998.5
 H_Physic_Sec_1239: Leg_state_182 + 999999 H_227 <= 999999.4
 H_Physic_Sec_1240: - Leg_state_182 + 999999 H_227 <= 999999.4
 H_Physic_Sec_1241: Leg_state_183 + 999999 H_227 <= 999999.1
 H_Physic_Sec_1242: - Leg_state_183 + 999999 H_227 <= 999998.9
 H_Physic_Sec_1243: Leg_state_181 + 999999 H_228 <= 1.000000825e+06
 H_Physic_Sec_1244: - Leg_state_181 + 999999 H_228 <= 999997.675
 H_Physic_Sec_1245: Leg_state_182 + 999999 H_228 <= 999999.4
 H_Physic_Sec_1246: - Leg_state_182 + 999999 H_228 <= 999999.4
 H_Physic_Sec_1247: Leg_state_183 + 999999 H_228 <= 999999.2
 H_Physic_Sec_1248: - Leg_state_183 + 999999 H_228 <= 999998.8
 H_Physic_Sec_1249: Leg_state_181 + 999999 H_229 <= 1.00000135e+06
 H_Physic_Sec_1250: - Leg_state_181 + 999999 H_229 <= 999997.15
 H_Physic_Sec_1251: Leg_state_182 + 999999 H_229 <= 999999.4
 H_Physic_Sec_1252: - Leg_state_182 + 999999 H_229 <= 999999.4
 H_Physic_Sec_1253: Leg_state_183 + 999999 H_229 <= 999999.1
 H_Physic_Sec_1254: - Leg_state_183 + 999999 H_229 <= 999998.9
 H_Physic_Sec_1255: Leg_state_181 + 999999 H_230 <= 1.0000026e+06
 H_Physic_Sec_1256: - Leg_state_181 + 999999 H_230 <= 999996.6
 H_Physic_Sec_1257: Leg_state_182 + 999999 H_230 <= 999999.4
 H_Physic_Sec_1258: - Leg_state_182 + 999999 H_230 <= 999999.4
 H_Physic_Sec_1259: Leg_state_183 + 999999 H_230 <= 999999
 H_Physic_Sec_1260: - Leg_state_183 + 999999 H_230 <= 999999
 H_Physic_Sec_1261: Leg_state_185 + 999999 H_231 <= 999999.475
 H_Physic_Sec_1262: - Leg_state_185 + 999999 H_231 <= 999999.475
 H_Physic_Sec_1263: Leg_state_186 + 999999 H_231 <= 999999.4
 H_Physic_Sec_1264: - Leg_state_186 + 999999 H_231 <= 999999.4
 H_Physic_Sec_1265: Leg_state_187 + 999999 H_231 <= 999999
 H_Physic_Sec_1266: - Leg_state_187 + 999999 H_231 <= 999999
 H_Physic_Sec_1267: Leg_state_185 + 999999 H_232 <= 1.0000003e+06
 H_Physic_Sec_1268: - Leg_state_185 + 999999 H_232 <= 999998.5
 H_Physic_Sec_1269: Leg_state_186 + 999999 H_232 <= 999999.4
 H_Physic_Sec_1270: - Leg_state_186 + 999999 H_232 <= 999999.4
 H_Physic_Sec_1271: Leg_state_187 + 999999 H_232 <= 999999.1
 H_Physic_Sec_1272: - Leg_state_187 + 999999 H_232 <= 999998.9
 H_Physic_Sec_1273: Leg_state_185 + 999999 H_233 <= 1.000000825e+06
 H_Physic_Sec_1274: - Leg_state_185 + 999999 H_233 <= 999997.675
 H_Physic_Sec_1275: Leg_state_186 + 999999 H_233 <= 999999.4
 H_Physic_Sec_1276: - Leg_state_186 + 999999 H_233 <= 999999.4
 H_Physic_Sec_1277: Leg_state_187 + 999999 H_233 <= 999999.2
 H_Physic_Sec_1278: - Leg_state_187 + 999999 H_233 <= 999998.8
 H_Physic_Sec_1279: Leg_state_185 + 999999 H_234 <= 1.00000135e+06
 H_Physic_Sec_1280: - Leg_state_185 + 999999 H_234 <= 999997.15
 H_Physic_Sec_1281: Leg_state_186 + 999999 H_234 <= 999999.4
 H_Physic_Sec_1282: - Leg_state_186 + 999999 H_234 <= 999999.4
 H_Physic_Sec_1283: Leg_state_187 + 999999 H_234 <= 999999.1
 H_Physic_Sec_1284: - Leg_state_187 + 999999 H_234 <= 999998.9
 H_Physic_Sec_1285: Leg_state_185 + 999999 H_235 <= 1.0000026e+06
 H_Physic_Sec_1286: - Leg_state_185 + 999999 H_235 <= 999996.6
 H_Physic_Sec_1287: Leg_state_186 + 999999 H_235 <= 999999.4
 H_Physic_Sec_1288: - Leg_state_186 + 999999 H_235 <= 999999.4
 H_Physic_Sec_1289: Leg_state_187 + 999999 H_235 <= 999999
 H_Physic_Sec_1290: - Leg_state_187 + 999999 H_235 <= 999999
 H_Physic_Sec_1291: Leg_state_189 + 999999 H_236 <= 999999.475
 H_Physic_Sec_1292: - Leg_state_189 + 999999 H_236 <= 999999.475
 H_Physic_Sec_1293: Leg_state_190 + 999999 H_236 <= 999999.4
 H_Physic_Sec_1294: - Leg_state_190 + 999999 H_236 <= 999999.4
 H_Physic_Sec_1295: Leg_state_191 + 999999 H_236 <= 999999
 H_Physic_Sec_1296: - Leg_state_191 + 999999 H_236 <= 999999
 H_Physic_Sec_1297: Leg_state_189 + 999999 H_237 <= 1.0000003e+06
 H_Physic_Sec_1298: - Leg_state_189 + 999999 H_237 <= 999998.5
 H_Physic_Sec_1299: Leg_state_190 + 999999 H_237 <= 999999.4
 H_Physic_Sec_1300: - Leg_state_190 + 999999 H_237 <= 999999.4
 H_Physic_Sec_1301: Leg_state_191 + 999999 H_237 <= 999999.1
 H_Physic_Sec_1302: - Leg_state_191 + 999999 H_237 <= 999998.9
 H_Physic_Sec_1303: Leg_state_189 + 999999 H_238 <= 1.000000825e+06
 H_Physic_Sec_1304: - Leg_state_189 + 999999 H_238 <= 999997.675
 H_Physic_Sec_1305: Leg_state_190 + 999999 H_238 <= 999999.4
 H_Physic_Sec_1306: - Leg_state_190 + 999999 H_238 <= 999999.4
 H_Physic_Sec_1307: Leg_state_191 + 999999 H_238 <= 999999.2
 H_Physic_Sec_1308: - Leg_state_191 + 999999 H_238 <= 999998.8
 H_Physic_Sec_1309: Leg_state_189 + 999999 H_239 <= 1.00000135e+06
 H_Physic_Sec_1310: - Leg_state_189 + 999999 H_239 <= 999997.15
 H_Physic_Sec_1311: Leg_state_190 + 999999 H_239 <= 999999.4
 H_Physic_Sec_1312: - Leg_state_190 + 999999 H_239 <= 999999.4
 H_Physic_Sec_1313: Leg_state_191 + 999999 H_239 <= 999999.1
 H_Physic_Sec_1314: - Leg_state_191 + 999999 H_239 <= 999998.9
 H_Physic_Sec_1315: Leg_state_189 + 999999 H_240 <= 1.0000026e+06
 H_Physic_Sec_1316: - Leg_state_189 + 999999 H_240 <= 999996.6
 H_Physic_Sec_1317: Leg_state_190 + 999999 H_240 <= 999999.4
 H_Physic_Sec_1318: - Leg_state_190 + 999999 H_240 <= 999999.4
 H_Physic_Sec_1319: Leg_state_191 + 999999 H_240 <= 999999
 H_Physic_Sec_1320: - Leg_state_191 + 999999 H_240 <= 999999
 H_Physic_Sec_1321: Leg_state_193 + 999999 H_241 <= 999999.475
 H_Physic_Sec_1322: - Leg_state_193 + 999999 H_241 <= 999999.475
 H_Physic_Sec_1323: Leg_state_194 + 999999 H_241 <= 999999.4
 H_Physic_Sec_1324: - Leg_state_194 + 999999 H_241 <= 999999.4
 H_Physic_Sec_1325: Leg_state_195 + 999999 H_241 <= 999999
 H_Physic_Sec_1326: - Leg_state_195 + 999999 H_241 <= 999999
 H_Physic_Sec_1327: Leg_state_193 + 999999 H_242 <= 1.0000003e+06
 H_Physic_Sec_1328: - Leg_state_193 + 999999 H_242 <= 999998.5
 H_Physic_Sec_1329: Leg_state_194 + 999999 H_242 <= 999999.4
 H_Physic_Sec_1330: - Leg_state_194 + 999999 H_242 <= 999999.4
 H_Physic_Sec_1331: Leg_state_195 + 999999 H_242 <= 999999.1
 H_Physic_Sec_1332: - Leg_state_195 + 999999 H_242 <= 999998.9
 H_Physic_Sec_1333: Leg_state_193 + 999999 H_243 <= 1.000000825e+06
 H_Physic_Sec_1334: - Leg_state_193 + 999999 H_243 <= 999997.675
 H_Physic_Sec_1335: Leg_state_194 + 999999 H_243 <= 999999.4
 H_Physic_Sec_1336: - Leg_state_194 + 999999 H_243 <= 999999.4
 H_Physic_Sec_1337: Leg_state_195 + 999999 H_243 <= 999999.2
 H_Physic_Sec_1338: - Leg_state_195 + 999999 H_243 <= 999998.8
 H_Physic_Sec_1339: Leg_state_193 + 999999 H_244 <= 1.00000135e+06
 H_Physic_Sec_1340: - Leg_state_193 + 999999 H_244 <= 999997.15
 H_Physic_Sec_1341: Leg_state_194 + 999999 H_244 <= 999999.4
 H_Physic_Sec_1342: - Leg_state_194 + 999999 H_244 <= 999999.4
 H_Physic_Sec_1343: Leg_state_195 + 999999 H_244 <= 999999.1
 H_Physic_Sec_1344: - Leg_state_195 + 999999 H_244 <= 999998.9
 H_Physic_Sec_1345: Leg_state_193 + 999999 H_245 <= 1.0000026e+06
 H_Physic_Sec_1346: - Leg_state_193 + 999999 H_245 <= 999996.6
 H_Physic_Sec_1347: Leg_state_194 + 999999 H_245 <= 999999.4
 H_Physic_Sec_1348: - Leg_state_194 + 999999 H_245 <= 999999.4
 H_Physic_Sec_1349: Leg_state_195 + 999999 H_245 <= 999999
 H_Physic_Sec_1350: - Leg_state_195 + 999999 H_245 <= 999999
 H_Physic_Sec_1351: Leg_state_197 + 999999 H_246 <= 999999.475
 H_Physic_Sec_1352: - Leg_state_197 + 999999 H_246 <= 999999.475
 H_Physic_Sec_1353: Leg_state_198 + 999999 H_246 <= 999999.4
 H_Physic_Sec_1354: - Leg_state_198 + 999999 H_246 <= 999999.4
 H_Physic_Sec_1355: Leg_state_199 + 999999 H_246 <= 999999
 H_Physic_Sec_1356: - Leg_state_199 + 999999 H_246 <= 999999
 H_Physic_Sec_1357: Leg_state_197 + 999999 H_247 <= 1.0000003e+06
 H_Physic_Sec_1358: - Leg_state_197 + 999999 H_247 <= 999998.5
 H_Physic_Sec_1359: Leg_state_198 + 999999 H_247 <= 999999.4
 H_Physic_Sec_1360: - Leg_state_198 + 999999 H_247 <= 999999.4
 H_Physic_Sec_1361: Leg_state_199 + 999999 H_247 <= 999999.1
 H_Physic_Sec_1362: - Leg_state_199 + 999999 H_247 <= 999998.9
 H_Physic_Sec_1363: Leg_state_197 + 999999 H_248 <= 1.000000825e+06
 H_Physic_Sec_1364: - Leg_state_197 + 999999 H_248 <= 999997.675
 H_Physic_Sec_1365: Leg_state_198 + 999999 H_248 <= 999999.4
 H_Physic_Sec_1366: - Leg_state_198 + 999999 H_248 <= 999999.4
 H_Physic_Sec_1367: Leg_state_199 + 999999 H_248 <= 999999.2
 H_Physic_Sec_1368: - Leg_state_199 + 999999 H_248 <= 999998.8
 H_Physic_Sec_1369: Leg_state_197 + 999999 H_249 <= 1.00000135e+06
 H_Physic_Sec_1370: - Leg_state_197 + 999999 H_249 <= 999997.15
 H_Physic_Sec_1371: Leg_state_198 + 999999 H_249 <= 999999.4
 H_Physic_Sec_1372: - Leg_state_198 + 999999 H_249 <= 999999.4
 H_Physic_Sec_1373: Leg_state_199 + 999999 H_249 <= 999999.1
 H_Physic_Sec_1374: - Leg_state_199 + 999999 H_249 <= 999998.9
 H_Physic_Sec_1375: Leg_state_197 + 999999 H_250 <= 1.0000026e+06
 H_Physic_Sec_1376: - Leg_state_197 + 999999 H_250 <= 999996.6
 H_Physic_Sec_1377: Leg_state_198 + 999999 H_250 <= 999999.4
 H_Physic_Sec_1378: - Leg_state_198 + 999999 H_250 <= 999999.4
 H_Physic_Sec_1379: Leg_state_199 + 999999 H_250 <= 999999
 H_Physic_Sec_1380: - Leg_state_199 + 999999 H_250 <= 999999
 H_Physic_Sec_1381: Leg_state_201 + 999999 H_251 <= 999999.475
 H_Physic_Sec_1382: - Leg_state_201 + 999999 H_251 <= 999999.475
 H_Physic_Sec_1383: Leg_state_202 + 999999 H_251 <= 999999.4
 H_Physic_Sec_1384: - Leg_state_202 + 999999 H_251 <= 999999.4
 H_Physic_Sec_1385: Leg_state_203 + 999999 H_251 <= 999999
 H_Physic_Sec_1386: - Leg_state_203 + 999999 H_251 <= 999999
 H_Physic_Sec_1387: Leg_state_201 + 999999 H_252 <= 1.0000003e+06
 H_Physic_Sec_1388: - Leg_state_201 + 999999 H_252 <= 999998.5
 H_Physic_Sec_1389: Leg_state_202 + 999999 H_252 <= 999999.4
 H_Physic_Sec_1390: - Leg_state_202 + 999999 H_252 <= 999999.4
 H_Physic_Sec_1391: Leg_state_203 + 999999 H_252 <= 999999.1
 H_Physic_Sec_1392: - Leg_state_203 + 999999 H_252 <= 999998.9
 H_Physic_Sec_1393: Leg_state_201 + 999999 H_253 <= 1.000000825e+06
 H_Physic_Sec_1394: - Leg_state_201 + 999999 H_253 <= 999997.675
 H_Physic_Sec_1395: Leg_state_202 + 999999 H_253 <= 999999.4
 H_Physic_Sec_1396: - Leg_state_202 + 999999 H_253 <= 999999.4
 H_Physic_Sec_1397: Leg_state_203 + 999999 H_253 <= 999999.2
 H_Physic_Sec_1398: - Leg_state_203 + 999999 H_253 <= 999998.8
 H_Physic_Sec_1399: Leg_state_201 + 999999 H_254 <= 1.00000135e+06
 H_Physic_Sec_1400: - Leg_state_201 + 999999 H_254 <= 999997.15
 H_Physic_Sec_1401: Leg_state_202 + 999999 H_254 <= 999999.4
 H_Physic_Sec_1402: - Leg_state_202 + 999999 H_254 <= 999999.4
 H_Physic_Sec_1403: Leg_state_203 + 999999 H_254 <= 999999.1
 H_Physic_Sec_1404: - Leg_state_203 + 999999 H_254 <= 999998.9
 H_Physic_Sec_1405: Leg_state_201 + 999999 H_255 <= 1.0000026e+06
 H_Physic_Sec_1406: - Leg_state_201 + 999999 H_255 <= 999996.6
 H_Physic_Sec_1407: Leg_state_202 + 999999 H_255 <= 999999.4
 H_Physic_Sec_1408: - Leg_state_202 + 999999 H_255 <= 999999.4
 H_Physic_Sec_1409: Leg_state_203 + 999999 H_255 <= 999999
 H_Physic_Sec_1410: - Leg_state_203 + 999999 H_255 <= 999999
 H_Physic_Sec_1411: Leg_state_205 + 999999 H_256 <= 999999.475
 H_Physic_Sec_1412: - Leg_state_205 + 999999 H_256 <= 999999.475
 H_Physic_Sec_1413: Leg_state_206 + 999999 H_256 <= 999999.4
 H_Physic_Sec_1414: - Leg_state_206 + 999999 H_256 <= 999999.4
 H_Physic_Sec_1415: Leg_state_207 + 999999 H_256 <= 999999
 H_Physic_Sec_1416: - Leg_state_207 + 999999 H_256 <= 999999
 H_Physic_Sec_1417: Leg_state_205 + 999999 H_257 <= 1.0000003e+06
 H_Physic_Sec_1418: - Leg_state_205 + 999999 H_257 <= 999998.5
 H_Physic_Sec_1419: Leg_state_206 + 999999 H_257 <= 999999.4
 H_Physic_Sec_1420: - Leg_state_206 + 999999 H_257 <= 999999.4
 H_Physic_Sec_1421: Leg_state_207 + 999999 H_257 <= 999999.1
 H_Physic_Sec_1422: - Leg_state_207 + 999999 H_257 <= 999998.9
 H_Physic_Sec_1423: Leg_state_205 + 999999 H_258 <= 1.000000825e+06
 H_Physic_Sec_1424: - Leg_state_205 + 999999 H_258 <= 999997.675
 H_Physic_Sec_1425: Leg_state_206 + 999999 H_258 <= 999999.4
 H_Physic_Sec_1426: - Leg_state_206 + 999999 H_258 <= 999999.4
 H_Physic_Sec_1427: Leg_state_207 + 999999 H_258 <= 999999.2
 H_Physic_Sec_1428: - Leg_state_207 + 999999 H_258 <= 999998.8
 H_Physic_Sec_1429: Leg_state_205 + 999999 H_259 <= 1.00000135e+06
 H_Physic_Sec_1430: - Leg_state_205 + 999999 H_259 <= 999997.15
 H_Physic_Sec_1431: Leg_state_206 + 999999 H_259 <= 999999.4
 H_Physic_Sec_1432: - Leg_state_206 + 999999 H_259 <= 999999.4
 H_Physic_Sec_1433: Leg_state_207 + 999999 H_259 <= 999999.1
 H_Physic_Sec_1434: - Leg_state_207 + 999999 H_259 <= 999998.9
 H_Physic_Sec_1435: Leg_state_205 + 999999 H_260 <= 1.0000026e+06
 H_Physic_Sec_1436: - Leg_state_205 + 999999 H_260 <= 999996.6
 H_Physic_Sec_1437: Leg_state_206 + 999999 H_260 <= 999999.4
 H_Physic_Sec_1438: - Leg_state_206 + 999999 H_260 <= 999999.4
 H_Physic_Sec_1439: Leg_state_207 + 999999 H_260 <= 999999
 H_Physic_Sec_1440: - Leg_state_207 + 999999 H_260 <= 999999
 H_Physic_Sec_1441: Leg_state_209 + 999999 H_261 <= 999999.475
 H_Physic_Sec_1442: - Leg_state_209 + 999999 H_261 <= 999999.475
 H_Physic_Sec_1443: Leg_state_210 + 999999 H_261 <= 999999.4
 H_Physic_Sec_1444: - Leg_state_210 + 999999 H_261 <= 999999.4
 H_Physic_Sec_1445: Leg_state_211 + 999999 H_261 <= 999999
 H_Physic_Sec_1446: - Leg_state_211 + 999999 H_261 <= 999999
 H_Physic_Sec_1447: Leg_state_209 + 999999 H_262 <= 1.0000003e+06
 H_Physic_Sec_1448: - Leg_state_209 + 999999 H_262 <= 999998.5
 H_Physic_Sec_1449: Leg_state_210 + 999999 H_262 <= 999999.4
 H_Physic_Sec_1450: - Leg_state_210 + 999999 H_262 <= 999999.4
 H_Physic_Sec_1451: Leg_state_211 + 999999 H_262 <= 999999.1
 H_Physic_Sec_1452: - Leg_state_211 + 999999 H_262 <= 999998.9
 H_Physic_Sec_1453: Leg_state_209 + 999999 H_263 <= 1.000000825e+06
 H_Physic_Sec_1454: - Leg_state_209 + 999999 H_263 <= 999997.675
 H_Physic_Sec_1455: Leg_state_210 + 999999 H_263 <= 999999.4
 H_Physic_Sec_1456: - Leg_state_210 + 999999 H_263 <= 999999.4
 H_Physic_Sec_1457: Leg_state_211 + 999999 H_263 <= 999999.2
 H_Physic_Sec_1458: - Leg_state_211 + 999999 H_263 <= 999998.8
 H_Physic_Sec_1459: Leg_state_209 + 999999 H_264 <= 1.00000135e+06
 H_Physic_Sec_1460: - Leg_state_209 + 999999 H_264 <= 999997.15
 H_Physic_Sec_1461: Leg_state_210 + 999999 H_264 <= 999999.4
 H_Physic_Sec_1462: - Leg_state_210 + 999999 H_264 <= 999999.4
 H_Physic_Sec_1463: Leg_state_211 + 999999 H_264 <= 999999.1
 H_Physic_Sec_1464: - Leg_state_211 + 999999 H_264 <= 999998.9
 H_Physic_Sec_1465: Leg_state_209 + 999999 H_265 <= 1.0000026e+06
 H_Physic_Sec_1466: - Leg_state_209 + 999999 H_265 <= 999996.6
 H_Physic_Sec_1467: Leg_state_210 + 999999 H_265 <= 999999.4
 H_Physic_Sec_1468: - Leg_state_210 + 999999 H_265 <= 999999.4
 H_Physic_Sec_1469: Leg_state_211 + 999999 H_265 <= 999999
 H_Physic_Sec_1470: - Leg_state_211 + 999999 H_265 <= 999999
 H_Physic_Sec_1471: Leg_state_213 + 999999 H_266 <= 999999.475
 H_Physic_Sec_1472: - Leg_state_213 + 999999 H_266 <= 999999.475
 H_Physic_Sec_1473: Leg_state_214 + 999999 H_266 <= 999999.4
 H_Physic_Sec_1474: - Leg_state_214 + 999999 H_266 <= 999999.4
 H_Physic_Sec_1475: Leg_state_215 + 999999 H_266 <= 999999
 H_Physic_Sec_1476: - Leg_state_215 + 999999 H_266 <= 999999
 H_Physic_Sec_1477: Leg_state_213 + 999999 H_267 <= 1.0000003e+06
 H_Physic_Sec_1478: - Leg_state_213 + 999999 H_267 <= 999998.5
 H_Physic_Sec_1479: Leg_state_214 + 999999 H_267 <= 999999.4
 H_Physic_Sec_1480: - Leg_state_214 + 999999 H_267 <= 999999.4
 H_Physic_Sec_1481: Leg_state_215 + 999999 H_267 <= 999999.1
 H_Physic_Sec_1482: - Leg_state_215 + 999999 H_267 <= 999998.9
 H_Physic_Sec_1483: Leg_state_213 + 999999 H_268 <= 1.000000825e+06
 H_Physic_Sec_1484: - Leg_state_213 + 999999 H_268 <= 999997.675
 H_Physic_Sec_1485: Leg_state_214 + 999999 H_268 <= 999999.4
 H_Physic_Sec_1486: - Leg_state_214 + 999999 H_268 <= 999999.4
 H_Physic_Sec_1487: Leg_state_215 + 999999 H_268 <= 999999.2
 H_Physic_Sec_1488: - Leg_state_215 + 999999 H_268 <= 999998.8
 H_Physic_Sec_1489: Leg_state_213 + 999999 H_269 <= 1.00000135e+06
 H_Physic_Sec_1490: - Leg_state_213 + 999999 H_269 <= 999997.15
 H_Physic_Sec_1491: Leg_state_214 + 999999 H_269 <= 999999.4
 H_Physic_Sec_1492: - Leg_state_214 + 999999 H_269 <= 999999.4
 H_Physic_Sec_1493: Leg_state_215 + 999999 H_269 <= 999999.1
 H_Physic_Sec_1494: - Leg_state_215 + 999999 H_269 <= 999998.9
 H_Physic_Sec_1495: Leg_state_213 + 999999 H_270 <= 1.0000026e+06
 H_Physic_Sec_1496: - Leg_state_213 + 999999 H_270 <= 999996.6
 H_Physic_Sec_1497: Leg_state_214 + 999999 H_270 <= 999999.4
 H_Physic_Sec_1498: - Leg_state_214 + 999999 H_270 <= 999999.4
 H_Physic_Sec_1499: Leg_state_215 + 999999 H_270 <= 999999
 H_Physic_Sec_1500: - Leg_state_215 + 999999 H_270 <= 999999
 H_Physic_Sec_1501: Leg_state_217 + 999999 H_271 <= 999999.475
 H_Physic_Sec_1502: - Leg_state_217 + 999999 H_271 <= 999999.475
 H_Physic_Sec_1503: Leg_state_218 + 999999 H_271 <= 999999.4
 H_Physic_Sec_1504: - Leg_state_218 + 999999 H_271 <= 999999.4
 H_Physic_Sec_1505: Leg_state_219 + 999999 H_271 <= 999999
 H_Physic_Sec_1506: - Leg_state_219 + 999999 H_271 <= 999999
 H_Physic_Sec_1507: Leg_state_217 + 999999 H_272 <= 1.0000003e+06
 H_Physic_Sec_1508: - Leg_state_217 + 999999 H_272 <= 999998.5
 H_Physic_Sec_1509: Leg_state_218 + 999999 H_272 <= 999999.4
 H_Physic_Sec_1510: - Leg_state_218 + 999999 H_272 <= 999999.4
 H_Physic_Sec_1511: Leg_state_219 + 999999 H_272 <= 999999.1
 H_Physic_Sec_1512: - Leg_state_219 + 999999 H_272 <= 999998.9
 H_Physic_Sec_1513: Leg_state_217 + 999999 H_273 <= 1.000000825e+06
 H_Physic_Sec_1514: - Leg_state_217 + 999999 H_273 <= 999997.675
 H_Physic_Sec_1515: Leg_state_218 + 999999 H_273 <= 999999.4
 H_Physic_Sec_1516: - Leg_state_218 + 999999 H_273 <= 999999.4
 H_Physic_Sec_1517: Leg_state_219 + 999999 H_273 <= 999999.2
 H_Physic_Sec_1518: - Leg_state_219 + 999999 H_273 <= 999998.8
 H_Physic_Sec_1519: Leg_state_217 + 999999 H_274 <= 1.00000135e+06
 H_Physic_Sec_1520: - Leg_state_217 + 999999 H_274 <= 999997.15
 H_Physic_Sec_1521: Leg_state_218 + 999999 H_274 <= 999999.4
 H_Physic_Sec_1522: - Leg_state_218 + 999999 H_274 <= 999999.4
 H_Physic_Sec_1523: Leg_state_219 + 999999 H_274 <= 999999.1
 H_Physic_Sec_1524: - Leg_state_219 + 999999 H_274 <= 999998.9
 H_Physic_Sec_1525: Leg_state_217 + 999999 H_275 <= 1.0000026e+06
 H_Physic_Sec_1526: - Leg_state_217 + 999999 H_275 <= 999996.6
 H_Physic_Sec_1527: Leg_state_218 + 999999 H_275 <= 999999.4
 H_Physic_Sec_1528: - Leg_state_218 + 999999 H_275 <= 999999.4
 H_Physic_Sec_1529: Leg_state_219 + 999999 H_275 <= 999999
 H_Physic_Sec_1530: - Leg_state_219 + 999999 H_275 <= 999999
 H_Physic_Sec_1531: Leg_state_221 + 999999 H_276 <= 999999.475
 H_Physic_Sec_1532: - Leg_state_221 + 999999 H_276 <= 999999.475
 H_Physic_Sec_1533: Leg_state_222 + 999999 H_276 <= 999999.4
 H_Physic_Sec_1534: - Leg_state_222 + 999999 H_276 <= 999999.4
 H_Physic_Sec_1535: Leg_state_223 + 999999 H_276 <= 999999
 H_Physic_Sec_1536: - Leg_state_223 + 999999 H_276 <= 999999
 H_Physic_Sec_1537: Leg_state_221 + 999999 H_277 <= 1.0000003e+06
 H_Physic_Sec_1538: - Leg_state_221 + 999999 H_277 <= 999998.5
 H_Physic_Sec_1539: Leg_state_222 + 999999 H_277 <= 999999.4
 H_Physic_Sec_1540: - Leg_state_222 + 999999 H_277 <= 999999.4
 H_Physic_Sec_1541: Leg_state_223 + 999999 H_277 <= 999999.1
 H_Physic_Sec_1542: - Leg_state_223 + 999999 H_277 <= 999998.9
 H_Physic_Sec_1543: Leg_state_221 + 999999 H_278 <= 1.000000825e+06
 H_Physic_Sec_1544: - Leg_state_221 + 999999 H_278 <= 999997.675
 H_Physic_Sec_1545: Leg_state_222 + 999999 H_278 <= 999999.4
 H_Physic_Sec_1546: - Leg_state_222 + 999999 H_278 <= 999999.4
 H_Physic_Sec_1547: Leg_state_223 + 999999 H_278 <= 999999.2
 H_Physic_Sec_1548: - Leg_state_223 + 999999 H_278 <= 999998.8
 H_Physic_Sec_1549: Leg_state_221 + 999999 H_279 <= 1.00000135e+06
 H_Physic_Sec_1550: - Leg_state_221 + 999999 H_279 <= 999997.15
 H_Physic_Sec_1551: Leg_state_222 + 999999 H_279 <= 999999.4
 H_Physic_Sec_1552: - Leg_state_222 + 999999 H_279 <= 999999.4
 H_Physic_Sec_1553: Leg_state_223 + 999999 H_279 <= 999999.1
 H_Physic_Sec_1554: - Leg_state_223 + 999999 H_279 <= 999998.9
 H_Physic_Sec_1555: Leg_state_221 + 999999 H_280 <= 1.0000026e+06
 H_Physic_Sec_1556: - Leg_state_221 + 999999 H_280 <= 999996.6
 H_Physic_Sec_1557: Leg_state_222 + 999999 H_280 <= 999999.4
 H_Physic_Sec_1558: - Leg_state_222 + 999999 H_280 <= 999999.4
 H_Physic_Sec_1559: Leg_state_223 + 999999 H_280 <= 999999
 H_Physic_Sec_1560: - Leg_state_223 + 999999 H_280 <= 999999
 H_Physic_Sec_1561: Leg_state_225 + 999999 H_281 <= 999999.475
 H_Physic_Sec_1562: - Leg_state_225 + 999999 H_281 <= 999999.475
 H_Physic_Sec_1563: Leg_state_226 + 999999 H_281 <= 999999.4
 H_Physic_Sec_1564: - Leg_state_226 + 999999 H_281 <= 999999.4
 H_Physic_Sec_1565: Leg_state_227 + 999999 H_281 <= 999999
 H_Physic_Sec_1566: - Leg_state_227 + 999999 H_281 <= 999999
 H_Physic_Sec_1567: Leg_state_225 + 999999 H_282 <= 1.0000003e+06
 H_Physic_Sec_1568: - Leg_state_225 + 999999 H_282 <= 999998.5
 H_Physic_Sec_1569: Leg_state_226 + 999999 H_282 <= 999999.4
 H_Physic_Sec_1570: - Leg_state_226 + 999999 H_282 <= 999999.4
 H_Physic_Sec_1571: Leg_state_227 + 999999 H_282 <= 999999.1
 H_Physic_Sec_1572: - Leg_state_227 + 999999 H_282 <= 999998.9
 H_Physic_Sec_1573: Leg_state_225 + 999999 H_283 <= 1.000000825e+06
 H_Physic_Sec_1574: - Leg_state_225 + 999999 H_283 <= 999997.675
 H_Physic_Sec_1575: Leg_state_226 + 999999 H_283 <= 999999.4
 H_Physic_Sec_1576: - Leg_state_226 + 999999 H_283 <= 999999.4
 H_Physic_Sec_1577: Leg_state_227 + 999999 H_283 <= 999999.2
 H_Physic_Sec_1578: - Leg_state_227 + 999999 H_283 <= 999998.8
 H_Physic_Sec_1579: Leg_state_225 + 999999 H_284 <= 1.00000135e+06
 H_Physic_Sec_1580: - Leg_state_225 + 999999 H_284 <= 999997.15
 H_Physic_Sec_1581: Leg_state_226 + 999999 H_284 <= 999999.4
 H_Physic_Sec_1582: - Leg_state_226 + 999999 H_284 <= 999999.4
 H_Physic_Sec_1583: Leg_state_227 + 999999 H_284 <= 999999.1
 H_Physic_Sec_1584: - Leg_state_227 + 999999 H_284 <= 999998.9
 H_Physic_Sec_1585: Leg_state_225 + 999999 H_285 <= 1.0000026e+06
 H_Physic_Sec_1586: - Leg_state_225 + 999999 H_285 <= 999996.6
 H_Physic_Sec_1587: Leg_state_226 + 999999 H_285 <= 999999.4
 H_Physic_Sec_1588: - Leg_state_226 + 999999 H_285 <= 999999.4
 H_Physic_Sec_1589: Leg_state_227 + 999999 H_285 <= 999999
 H_Physic_Sec_1590: - Leg_state_227 + 999999 H_285 <= 999999
 H_Physic_Sec_1591: Leg_state_229 + 999999 H_286 <= 999999.475
 H_Physic_Sec_1592: - Leg_state_229 + 999999 H_286 <= 999999.475
 H_Physic_Sec_1593: Leg_state_230 + 999999 H_286 <= 999999.4
 H_Physic_Sec_1594: - Leg_state_230 + 999999 H_286 <= 999999.4
 H_Physic_Sec_1595: Leg_state_231 + 999999 H_286 <= 999999
 H_Physic_Sec_1596: - Leg_state_231 + 999999 H_286 <= 999999
 H_Physic_Sec_1597: Leg_state_229 + 999999 H_287 <= 1.0000003e+06
 H_Physic_Sec_1598: - Leg_state_229 + 999999 H_287 <= 999998.5
 H_Physic_Sec_1599: Leg_state_230 + 999999 H_287 <= 999999.4
 H_Physic_Sec_1600: - Leg_state_230 + 999999 H_287 <= 999999.4
 H_Physic_Sec_1601: Leg_state_231 + 999999 H_287 <= 999999.1
 H_Physic_Sec_1602: - Leg_state_231 + 999999 H_287 <= 999998.9
 H_Physic_Sec_1603: Leg_state_229 + 999999 H_288 <= 1.000000825e+06
 H_Physic_Sec_1604: - Leg_state_229 + 999999 H_288 <= 999997.675
 H_Physic_Sec_1605: Leg_state_230 + 999999 H_288 <= 999999.4
 H_Physic_Sec_1606: - Leg_state_230 + 999999 H_288 <= 999999.4
 H_Physic_Sec_1607: Leg_state_231 + 999999 H_288 <= 999999.2
 H_Physic_Sec_1608: - Leg_state_231 + 999999 H_288 <= 999998.8
 H_Physic_Sec_1609: Leg_state_229 + 999999 H_289 <= 1.00000135e+06
 H_Physic_Sec_1610: - Leg_state_229 + 999999 H_289 <= 999997.15
 H_Physic_Sec_1611: Leg_state_230 + 999999 H_289 <= 999999.4
 H_Physic_Sec_1612: - Leg_state_230 + 999999 H_289 <= 999999.4
 H_Physic_Sec_1613: Leg_state_231 + 999999 H_289 <= 999999.1
 H_Physic_Sec_1614: - Leg_state_231 + 999999 H_289 <= 999998.9
 H_Physic_Sec_1615: Leg_state_229 + 999999 H_290 <= 1.0000026e+06
 H_Physic_Sec_1616: - Leg_state_229 + 999999 H_290 <= 999996.6
 H_Physic_Sec_1617: Leg_state_230 + 999999 H_290 <= 999999.4
 H_Physic_Sec_1618: - Leg_state_230 + 999999 H_290 <= 999999.4
 H_Physic_Sec_1619: Leg_state_231 + 999999 H_290 <= 999999
 H_Physic_Sec_1620: - Leg_state_231 + 999999 H_290 <= 999999
 H_Physic_Sec_1621: Leg_state_233 + 999999 H_291 <= 999999.475
 H_Physic_Sec_1622: - Leg_state_233 + 999999 H_291 <= 999999.475
 H_Physic_Sec_1623: Leg_state_234 + 999999 H_291 <= 999999.4
 H_Physic_Sec_1624: - Leg_state_234 + 999999 H_291 <= 999999.4
 H_Physic_Sec_1625: Leg_state_235 + 999999 H_291 <= 999999
 H_Physic_Sec_1626: - Leg_state_235 + 999999 H_291 <= 999999
 H_Physic_Sec_1627: Leg_state_233 + 999999 H_292 <= 1.0000003e+06
 H_Physic_Sec_1628: - Leg_state_233 + 999999 H_292 <= 999998.5
 H_Physic_Sec_1629: Leg_state_234 + 999999 H_292 <= 999999.4
 H_Physic_Sec_1630: - Leg_state_234 + 999999 H_292 <= 999999.4
 H_Physic_Sec_1631: Leg_state_235 + 999999 H_292 <= 999999.1
 H_Physic_Sec_1632: - Leg_state_235 + 999999 H_292 <= 999998.9
 H_Physic_Sec_1633: Leg_state_233 + 999999 H_293 <= 1.000000825e+06
 H_Physic_Sec_1634: - Leg_state_233 + 999999 H_293 <= 999997.675
 H_Physic_Sec_1635: Leg_state_234 + 999999 H_293 <= 999999.4
 H_Physic_Sec_1636: - Leg_state_234 + 999999 H_293 <= 999999.4
 H_Physic_Sec_1637: Leg_state_235 + 999999 H_293 <= 999999.2
 H_Physic_Sec_1638: - Leg_state_235 + 999999 H_293 <= 999998.8
 H_Physic_Sec_1639: Leg_state_233 + 999999 H_294 <= 1.00000135e+06
 H_Physic_Sec_1640: - Leg_state_233 + 999999 H_294 <= 999997.15
 H_Physic_Sec_1641: Leg_state_234 + 999999 H_294 <= 999999.4
 H_Physic_Sec_1642: - Leg_state_234 + 999999 H_294 <= 999999.4
 H_Physic_Sec_1643: Leg_state_235 + 999999 H_294 <= 999999.1
 H_Physic_Sec_1644: - Leg_state_235 + 999999 H_294 <= 999998.9
 H_Physic_Sec_1645: Leg_state_233 + 999999 H_295 <= 1.0000026e+06
 H_Physic_Sec_1646: - Leg_state_233 + 999999 H_295 <= 999996.6
 H_Physic_Sec_1647: Leg_state_234 + 999999 H_295 <= 999999.4
 H_Physic_Sec_1648: - Leg_state_234 + 999999 H_295 <= 999999.4
 H_Physic_Sec_1649: Leg_state_235 + 999999 H_295 <= 999999
 H_Physic_Sec_1650: - Leg_state_235 + 999999 H_295 <= 999999
 H_Physic_Sec_1651: Leg_state_237 + 999999 H_296 <= 999999.475
 H_Physic_Sec_1652: - Leg_state_237 + 999999 H_296 <= 999999.475
 H_Physic_Sec_1653: Leg_state_238 + 999999 H_296 <= 999999.4
 H_Physic_Sec_1654: - Leg_state_238 + 999999 H_296 <= 999999.4
 H_Physic_Sec_1655: Leg_state_239 + 999999 H_296 <= 999999
 H_Physic_Sec_1656: - Leg_state_239 + 999999 H_296 <= 999999
 H_Physic_Sec_1657: Leg_state_237 + 999999 H_297 <= 1.0000003e+06
 H_Physic_Sec_1658: - Leg_state_237 + 999999 H_297 <= 999998.5
 H_Physic_Sec_1659: Leg_state_238 + 999999 H_297 <= 999999.4
 H_Physic_Sec_1660: - Leg_state_238 + 999999 H_297 <= 999999.4
 H_Physic_Sec_1661: Leg_state_239 + 999999 H_297 <= 999999.1
 H_Physic_Sec_1662: - Leg_state_239 + 999999 H_297 <= 999998.9
 H_Physic_Sec_1663: Leg_state_237 + 999999 H_298 <= 1.000000825e+06
 H_Physic_Sec_1664: - Leg_state_237 + 999999 H_298 <= 999997.675
 H_Physic_Sec_1665: Leg_state_238 + 999999 H_298 <= 999999.4
 H_Physic_Sec_1666: - Leg_state_238 + 999999 H_298 <= 999999.4
 H_Physic_Sec_1667: Leg_state_239 + 999999 H_298 <= 999999.2
 H_Physic_Sec_1668: - Leg_state_239 + 999999 H_298 <= 999998.8
 H_Physic_Sec_1669: Leg_state_237 + 999999 H_299 <= 1.00000135e+06
 H_Physic_Sec_1670: - Leg_state_237 + 999999 H_299 <= 999997.15
 H_Physic_Sec_1671: Leg_state_238 + 999999 H_299 <= 999999.4
 H_Physic_Sec_1672: - Leg_state_238 + 999999 H_299 <= 999999.4
 H_Physic_Sec_1673: Leg_state_239 + 999999 H_299 <= 999999.1
 H_Physic_Sec_1674: - Leg_state_239 + 999999 H_299 <= 999998.9
 H_Physic_Sec_1675: Leg_state_237 + 999999 H_300 <= 1.0000026e+06
 H_Physic_Sec_1676: - Leg_state_237 + 999999 H_300 <= 999996.6
 H_Physic_Sec_1677: Leg_state_238 + 999999 H_300 <= 999999.4
 H_Physic_Sec_1678: - Leg_state_238 + 999999 H_300 <= 999999.4
 H_Physic_Sec_1679: Leg_state_239 + 999999 H_300 <= 999999
 H_Physic_Sec_1680: - Leg_state_239 + 999999 H_300 <= 999999
 H_Physic_Sec_1681: Leg_state_241 + 999999 H_301 <= 999999.475
 H_Physic_Sec_1682: - Leg_state_241 + 999999 H_301 <= 999999.475
 H_Physic_Sec_1683: Leg_state_242 + 999999 H_301 <= 999999.4
 H_Physic_Sec_1684: - Leg_state_242 + 999999 H_301 <= 999999.4
 H_Physic_Sec_1685: Leg_state_243 + 999999 H_301 <= 999999
 H_Physic_Sec_1686: - Leg_state_243 + 999999 H_301 <= 999999
 H_Physic_Sec_1687: Leg_state_241 + 999999 H_302 <= 1.0000003e+06
 H_Physic_Sec_1688: - Leg_state_241 + 999999 H_302 <= 999998.5
 H_Physic_Sec_1689: Leg_state_242 + 999999 H_302 <= 999999.4
 H_Physic_Sec_1690: - Leg_state_242 + 999999 H_302 <= 999999.4
 H_Physic_Sec_1691: Leg_state_243 + 999999 H_302 <= 999999.1
 H_Physic_Sec_1692: - Leg_state_243 + 999999 H_302 <= 999998.9
 H_Physic_Sec_1693: Leg_state_241 + 999999 H_303 <= 1.000000825e+06
 H_Physic_Sec_1694: - Leg_state_241 + 999999 H_303 <= 999997.675
 H_Physic_Sec_1695: Leg_state_242 + 999999 H_303 <= 999999.4
 H_Physic_Sec_1696: - Leg_state_242 + 999999 H_303 <= 999999.4
 H_Physic_Sec_1697: Leg_state_243 + 999999 H_303 <= 999999.2
 H_Physic_Sec_1698: - Leg_state_243 + 999999 H_303 <= 999998.8
 H_Physic_Sec_1699: Leg_state_241 + 999999 H_304 <= 1.00000135e+06
 H_Physic_Sec_1700: - Leg_state_241 + 999999 H_304 <= 999997.15
 H_Physic_Sec_1701: Leg_state_242 + 999999 H_304 <= 999999.4
 H_Physic_Sec_1702: - Leg_state_242 + 999999 H_304 <= 999999.4
 H_Physic_Sec_1703: Leg_state_243 + 999999 H_304 <= 999999.1
 H_Physic_Sec_1704: - Leg_state_243 + 999999 H_304 <= 999998.9
 H_Physic_Sec_1705: Leg_state_241 + 999999 H_305 <= 1.0000026e+06
 H_Physic_Sec_1706: - Leg_state_241 + 999999 H_305 <= 999996.6
 H_Physic_Sec_1707: Leg_state_242 + 999999 H_305 <= 999999.4
 H_Physic_Sec_1708: - Leg_state_242 + 999999 H_305 <= 999999.4
 H_Physic_Sec_1709: Leg_state_243 + 999999 H_305 <= 999999
 H_Physic_Sec_1710: - Leg_state_243 + 999999 H_305 <= 999999
 H_Physic_Sec_1711: Leg_state_245 + 999999 H_306 <= 999999.475
 H_Physic_Sec_1712: - Leg_state_245 + 999999 H_306 <= 999999.475
 H_Physic_Sec_1713: Leg_state_246 + 999999 H_306 <= 999999.4
 H_Physic_Sec_1714: - Leg_state_246 + 999999 H_306 <= 999999.4
 H_Physic_Sec_1715: Leg_state_247 + 999999 H_306 <= 999999
 H_Physic_Sec_1716: - Leg_state_247 + 999999 H_306 <= 999999
 H_Physic_Sec_1717: Leg_state_245 + 999999 H_307 <= 1.0000003e+06
 H_Physic_Sec_1718: - Leg_state_245 + 999999 H_307 <= 999998.5
 H_Physic_Sec_1719: Leg_state_246 + 999999 H_307 <= 999999.4
 H_Physic_Sec_1720: - Leg_state_246 + 999999 H_307 <= 999999.4
 H_Physic_Sec_1721: Leg_state_247 + 999999 H_307 <= 999999.1
 H_Physic_Sec_1722: - Leg_state_247 + 999999 H_307 <= 999998.9
 H_Physic_Sec_1723: Leg_state_245 + 999999 H_308 <= 1.000000825e+06
 H_Physic_Sec_1724: - Leg_state_245 + 999999 H_308 <= 999997.675
 H_Physic_Sec_1725: Leg_state_246 + 999999 H_308 <= 999999.4
 H_Physic_Sec_1726: - Leg_state_246 + 999999 H_308 <= 999999.4
 H_Physic_Sec_1727: Leg_state_247 + 999999 H_308 <= 999999.2
 H_Physic_Sec_1728: - Leg_state_247 + 999999 H_308 <= 999998.8
 H_Physic_Sec_1729: Leg_state_245 + 999999 H_309 <= 1.00000135e+06
 H_Physic_Sec_1730: - Leg_state_245 + 999999 H_309 <= 999997.15
 H_Physic_Sec_1731: Leg_state_246 + 999999 H_309 <= 999999.4
 H_Physic_Sec_1732: - Leg_state_246 + 999999 H_309 <= 999999.4
 H_Physic_Sec_1733: Leg_state_247 + 999999 H_309 <= 999999.1
 H_Physic_Sec_1734: - Leg_state_247 + 999999 H_309 <= 999998.9
 H_Physic_Sec_1735: Leg_state_245 + 999999 H_310 <= 1.0000026e+06
 H_Physic_Sec_1736: - Leg_state_245 + 999999 H_310 <= 999996.6
 H_Physic_Sec_1737: Leg_state_246 + 999999 H_310 <= 999999.4
 H_Physic_Sec_1738: - Leg_state_246 + 999999 H_310 <= 999999.4
 H_Physic_Sec_1739: Leg_state_247 + 999999 H_310 <= 999999
 H_Physic_Sec_1740: - Leg_state_247 + 999999 H_310 <= 999999
 H_Physic_Sec_1741: Leg_state_249 + 999999 H_311 <= 999999.475
 H_Physic_Sec_1742: - Leg_state_249 + 999999 H_311 <= 999999.475
 H_Physic_Sec_1743: Leg_state_250 + 999999 H_311 <= 999999.4
 H_Physic_Sec_1744: - Leg_state_250 + 999999 H_311 <= 999999.4
 H_Physic_Sec_1745: Leg_state_251 + 999999 H_311 <= 999999
 H_Physic_Sec_1746: - Leg_state_251 + 999999 H_311 <= 999999
 H_Physic_Sec_1747: Leg_state_249 + 999999 H_312 <= 1.0000003e+06
 H_Physic_Sec_1748: - Leg_state_249 + 999999 H_312 <= 999998.5
 H_Physic_Sec_1749: Leg_state_250 + 999999 H_312 <= 999999.4
 H_Physic_Sec_1750: - Leg_state_250 + 999999 H_312 <= 999999.4
 H_Physic_Sec_1751: Leg_state_251 + 999999 H_312 <= 999999.1
 H_Physic_Sec_1752: - Leg_state_251 + 999999 H_312 <= 999998.9
 H_Physic_Sec_1753: Leg_state_249 + 999999 H_313 <= 1.000000825e+06
 H_Physic_Sec_1754: - Leg_state_249 + 999999 H_313 <= 999997.675
 H_Physic_Sec_1755: Leg_state_250 + 999999 H_313 <= 999999.4
 H_Physic_Sec_1756: - Leg_state_250 + 999999 H_313 <= 999999.4
 H_Physic_Sec_1757: Leg_state_251 + 999999 H_313 <= 999999.2
 H_Physic_Sec_1758: - Leg_state_251 + 999999 H_313 <= 999998.8
 H_Physic_Sec_1759: Leg_state_249 + 999999 H_314 <= 1.00000135e+06
 H_Physic_Sec_1760: - Leg_state_249 + 999999 H_314 <= 999997.15
 H_Physic_Sec_1761: Leg_state_250 + 999999 H_314 <= 999999.4
 H_Physic_Sec_1762: - Leg_state_250 + 999999 H_314 <= 999999.4
 H_Physic_Sec_1763: Leg_state_251 + 999999 H_314 <= 999999.1
 H_Physic_Sec_1764: - Leg_state_251 + 999999 H_314 <= 999998.9
 H_Physic_Sec_1765: Leg_state_249 + 999999 H_315 <= 1.0000026e+06
 H_Physic_Sec_1766: - Leg_state_249 + 999999 H_315 <= 999996.6
 H_Physic_Sec_1767: Leg_state_250 + 999999 H_315 <= 999999.4
 H_Physic_Sec_1768: - Leg_state_250 + 999999 H_315 <= 999999.4
 H_Physic_Sec_1769: Leg_state_251 + 999999 H_315 <= 999999
 H_Physic_Sec_1770: - Leg_state_251 + 999999 H_315 <= 999999
 H_Physic_Sec_1771: Leg_state_253 + 999999 H_316 <= 999999.475
 H_Physic_Sec_1772: - Leg_state_253 + 999999 H_316 <= 999999.475
 H_Physic_Sec_1773: Leg_state_254 + 999999 H_316 <= 999999.4
 H_Physic_Sec_1774: - Leg_state_254 + 999999 H_316 <= 999999.4
 H_Physic_Sec_1775: Leg_state_255 + 999999 H_316 <= 999999
 H_Physic_Sec_1776: - Leg_state_255 + 999999 H_316 <= 999999
 H_Physic_Sec_1777: Leg_state_253 + 999999 H_317 <= 1.0000003e+06
 H_Physic_Sec_1778: - Leg_state_253 + 999999 H_317 <= 999998.5
 H_Physic_Sec_1779: Leg_state_254 + 999999 H_317 <= 999999.4
 H_Physic_Sec_1780: - Leg_state_254 + 999999 H_317 <= 999999.4
 H_Physic_Sec_1781: Leg_state_255 + 999999 H_317 <= 999999.1
 H_Physic_Sec_1782: - Leg_state_255 + 999999 H_317 <= 999998.9
 H_Physic_Sec_1783: Leg_state_253 + 999999 H_318 <= 1.000000825e+06
 H_Physic_Sec_1784: - Leg_state_253 + 999999 H_318 <= 999997.675
 H_Physic_Sec_1785: Leg_state_254 + 999999 H_318 <= 999999.4
 H_Physic_Sec_1786: - Leg_state_254 + 999999 H_318 <= 999999.4
 H_Physic_Sec_1787: Leg_state_255 + 999999 H_318 <= 999999.2
 H_Physic_Sec_1788: - Leg_state_255 + 999999 H_318 <= 999998.8
 H_Physic_Sec_1789: Leg_state_253 + 999999 H_319 <= 1.00000135e+06
 H_Physic_Sec_1790: - Leg_state_253 + 999999 H_319 <= 999997.15
 H_Physic_Sec_1791: Leg_state_254 + 999999 H_319 <= 999999.4
 H_Physic_Sec_1792: - Leg_state_254 + 999999 H_319 <= 999999.4
 H_Physic_Sec_1793: Leg_state_255 + 999999 H_319 <= 999999.1
 H_Physic_Sec_1794: - Leg_state_255 + 999999 H_319 <= 999998.9
 H_Physic_Sec_1795: Leg_state_253 + 999999 H_320 <= 1.0000026e+06
 H_Physic_Sec_1796: - Leg_state_253 + 999999 H_320 <= 999996.6
 H_Physic_Sec_1797: Leg_state_254 + 999999 H_320 <= 999999.4
 H_Physic_Sec_1798: - Leg_state_254 + 999999 H_320 <= 999999.4
 H_Physic_Sec_1799: Leg_state_255 + 999999 H_320 <= 999999
 H_Physic_Sec_1800: - Leg_state_255 + 999999 H_320 <= 999999
 H_Physic_Sec_1801: Leg_state_257 + 999999 H_321 <= 999999.475
 H_Physic_Sec_1802: - Leg_state_257 + 999999 H_321 <= 999999.475
 H_Physic_Sec_1803: Leg_state_258 + 999999 H_321 <= 999999.4
 H_Physic_Sec_1804: - Leg_state_258 + 999999 H_321 <= 999999.4
 H_Physic_Sec_1805: Leg_state_259 + 999999 H_321 <= 999999
 H_Physic_Sec_1806: - Leg_state_259 + 999999 H_321 <= 999999
 H_Physic_Sec_1807: Leg_state_257 + 999999 H_322 <= 1.0000003e+06
 H_Physic_Sec_1808: - Leg_state_257 + 999999 H_322 <= 999998.5
 H_Physic_Sec_1809: Leg_state_258 + 999999 H_322 <= 999999.4
 H_Physic_Sec_1810: - Leg_state_258 + 999999 H_322 <= 999999.4
 H_Physic_Sec_1811: Leg_state_259 + 999999 H_322 <= 999999.1
 H_Physic_Sec_1812: - Leg_state_259 + 999999 H_322 <= 999998.9
 H_Physic_Sec_1813: Leg_state_257 + 999999 H_323 <= 1.000000825e+06
 H_Physic_Sec_1814: - Leg_state_257 + 999999 H_323 <= 999997.675
 H_Physic_Sec_1815: Leg_state_258 + 999999 H_323 <= 999999.4
 H_Physic_Sec_1816: - Leg_state_258 + 999999 H_323 <= 999999.4
 H_Physic_Sec_1817: Leg_state_259 + 999999 H_323 <= 999999.2
 H_Physic_Sec_1818: - Leg_state_259 + 999999 H_323 <= 999998.8
 H_Physic_Sec_1819: Leg_state_257 + 999999 H_324 <= 1.00000135e+06
 H_Physic_Sec_1820: - Leg_state_257 + 999999 H_324 <= 999997.15
 H_Physic_Sec_1821: Leg_state_258 + 999999 H_324 <= 999999.4
 H_Physic_Sec_1822: - Leg_state_258 + 999999 H_324 <= 999999.4
 H_Physic_Sec_1823: Leg_state_259 + 999999 H_324 <= 999999.1
 H_Physic_Sec_1824: - Leg_state_259 + 999999 H_324 <= 999998.9
 H_Physic_Sec_1825: Leg_state_257 + 999999 H_325 <= 1.0000026e+06
 H_Physic_Sec_1826: - Leg_state_257 + 999999 H_325 <= 999996.6
 H_Physic_Sec_1827: Leg_state_258 + 999999 H_325 <= 999999.4
 H_Physic_Sec_1828: - Leg_state_258 + 999999 H_325 <= 999999.4
 H_Physic_Sec_1829: Leg_state_259 + 999999 H_325 <= 999999
 H_Physic_Sec_1830: - Leg_state_259 + 999999 H_325 <= 999999
 H_Physic_Sec_1831: Leg_state_261 + 999999 H_326 <= 999999.475
 H_Physic_Sec_1832: - Leg_state_261 + 999999 H_326 <= 999999.475
 H_Physic_Sec_1833: Leg_state_262 + 999999 H_326 <= 999999.4
 H_Physic_Sec_1834: - Leg_state_262 + 999999 H_326 <= 999999.4
 H_Physic_Sec_1835: Leg_state_263 + 999999 H_326 <= 999999
 H_Physic_Sec_1836: - Leg_state_263 + 999999 H_326 <= 999999
 H_Physic_Sec_1837: Leg_state_261 + 999999 H_327 <= 1.0000003e+06
 H_Physic_Sec_1838: - Leg_state_261 + 999999 H_327 <= 999998.5
 H_Physic_Sec_1839: Leg_state_262 + 999999 H_327 <= 999999.4
 H_Physic_Sec_1840: - Leg_state_262 + 999999 H_327 <= 999999.4
 H_Physic_Sec_1841: Leg_state_263 + 999999 H_327 <= 999999.1
 H_Physic_Sec_1842: - Leg_state_263 + 999999 H_327 <= 999998.9
 H_Physic_Sec_1843: Leg_state_261 + 999999 H_328 <= 1.000000825e+06
 H_Physic_Sec_1844: - Leg_state_261 + 999999 H_328 <= 999997.675
 H_Physic_Sec_1845: Leg_state_262 + 999999 H_328 <= 999999.4
 H_Physic_Sec_1846: - Leg_state_262 + 999999 H_328 <= 999999.4
 H_Physic_Sec_1847: Leg_state_263 + 999999 H_328 <= 999999.2
 H_Physic_Sec_1848: - Leg_state_263 + 999999 H_328 <= 999998.8
 H_Physic_Sec_1849: Leg_state_261 + 999999 H_329 <= 1.00000135e+06
 H_Physic_Sec_1850: - Leg_state_261 + 999999 H_329 <= 999997.15
 H_Physic_Sec_1851: Leg_state_262 + 999999 H_329 <= 999999.4
 H_Physic_Sec_1852: - Leg_state_262 + 999999 H_329 <= 999999.4
 H_Physic_Sec_1853: Leg_state_263 + 999999 H_329 <= 999999.1
 H_Physic_Sec_1854: - Leg_state_263 + 999999 H_329 <= 999998.9
 H_Physic_Sec_1855: Leg_state_261 + 999999 H_330 <= 1.0000026e+06
 H_Physic_Sec_1856: - Leg_state_261 + 999999 H_330 <= 999996.6
 H_Physic_Sec_1857: Leg_state_262 + 999999 H_330 <= 999999.4
 H_Physic_Sec_1858: - Leg_state_262 + 999999 H_330 <= 999999.4
 H_Physic_Sec_1859: Leg_state_263 + 999999 H_330 <= 999999
 H_Physic_Sec_1860: - Leg_state_263 + 999999 H_330 <= 999999
 H_Physic_Sec_1861: Leg_state_265 + 999999 H_331 <= 999999.475
 H_Physic_Sec_1862: - Leg_state_265 + 999999 H_331 <= 999999.475
 H_Physic_Sec_1863: Leg_state_266 + 999999 H_331 <= 999999.4
 H_Physic_Sec_1864: - Leg_state_266 + 999999 H_331 <= 999999.4
 H_Physic_Sec_1865: Leg_state_267 + 999999 H_331 <= 999999
 H_Physic_Sec_1866: - Leg_state_267 + 999999 H_331 <= 999999
 H_Physic_Sec_1867: Leg_state_265 + 999999 H_332 <= 1.0000003e+06
 H_Physic_Sec_1868: - Leg_state_265 + 999999 H_332 <= 999998.5
 H_Physic_Sec_1869: Leg_state_266 + 999999 H_332 <= 999999.4
 H_Physic_Sec_1870: - Leg_state_266 + 999999 H_332 <= 999999.4
 H_Physic_Sec_1871: Leg_state_267 + 999999 H_332 <= 999999.1
 H_Physic_Sec_1872: - Leg_state_267 + 999999 H_332 <= 999998.9
 H_Physic_Sec_1873: Leg_state_265 + 999999 H_333 <= 1.000000825e+06
 H_Physic_Sec_1874: - Leg_state_265 + 999999 H_333 <= 999997.675
 H_Physic_Sec_1875: Leg_state_266 + 999999 H_333 <= 999999.4
 H_Physic_Sec_1876: - Leg_state_266 + 999999 H_333 <= 999999.4
 H_Physic_Sec_1877: Leg_state_267 + 999999 H_333 <= 999999.2
 H_Physic_Sec_1878: - Leg_state_267 + 999999 H_333 <= 999998.8
 H_Physic_Sec_1879: Leg_state_265 + 999999 H_334 <= 1.00000135e+06
 H_Physic_Sec_1880: - Leg_state_265 + 999999 H_334 <= 999997.15
 H_Physic_Sec_1881: Leg_state_266 + 999999 H_334 <= 999999.4
 H_Physic_Sec_1882: - Leg_state_266 + 999999 H_334 <= 999999.4
 H_Physic_Sec_1883: Leg_state_267 + 999999 H_334 <= 999999.1
 H_Physic_Sec_1884: - Leg_state_267 + 999999 H_334 <= 999998.9
 H_Physic_Sec_1885: Leg_state_265 + 999999 H_335 <= 1.0000026e+06
 H_Physic_Sec_1886: - Leg_state_265 + 999999 H_335 <= 999996.6
 H_Physic_Sec_1887: Leg_state_266 + 999999 H_335 <= 999999.4
 H_Physic_Sec_1888: - Leg_state_266 + 999999 H_335 <= 999999.4
 H_Physic_Sec_1889: Leg_state_267 + 999999 H_335 <= 999999
 H_Physic_Sec_1890: - Leg_state_267 + 999999 H_335 <= 999999
 H_Physic_Sec_1891: Leg_state_269 + 999999 H_336 <= 999999.475
 H_Physic_Sec_1892: - Leg_state_269 + 999999 H_336 <= 999999.475
 H_Physic_Sec_1893: Leg_state_270 + 999999 H_336 <= 999999.4
 H_Physic_Sec_1894: - Leg_state_270 + 999999 H_336 <= 999999.4
 H_Physic_Sec_1895: Leg_state_271 + 999999 H_336 <= 999999
 H_Physic_Sec_1896: - Leg_state_271 + 999999 H_336 <= 999999
 H_Physic_Sec_1897: Leg_state_269 + 999999 H_337 <= 1.0000003e+06
 H_Physic_Sec_1898: - Leg_state_269 + 999999 H_337 <= 999998.5
 H_Physic_Sec_1899: Leg_state_270 + 999999 H_337 <= 999999.4
 H_Physic_Sec_1900: - Leg_state_270 + 999999 H_337 <= 999999.4
 H_Physic_Sec_1901: Leg_state_271 + 999999 H_337 <= 999999.1
 H_Physic_Sec_1902: - Leg_state_271 + 999999 H_337 <= 999998.9
 H_Physic_Sec_1903: Leg_state_269 + 999999 H_338 <= 1.000000825e+06
 H_Physic_Sec_1904: - Leg_state_269 + 999999 H_338 <= 999997.675
 H_Physic_Sec_1905: Leg_state_270 + 999999 H_338 <= 999999.4
 H_Physic_Sec_1906: - Leg_state_270 + 999999 H_338 <= 999999.4
 H_Physic_Sec_1907: Leg_state_271 + 999999 H_338 <= 999999.2
 H_Physic_Sec_1908: - Leg_state_271 + 999999 H_338 <= 999998.8
 H_Physic_Sec_1909: Leg_state_269 + 999999 H_339 <= 1.00000135e+06
 H_Physic_Sec_1910: - Leg_state_269 + 999999 H_339 <= 999997.15
 H_Physic_Sec_1911: Leg_state_270 + 999999 H_339 <= 999999.4
 H_Physic_Sec_1912: - Leg_state_270 + 999999 H_339 <= 999999.4
 H_Physic_Sec_1913: Leg_state_271 + 999999 H_339 <= 999999.1
 H_Physic_Sec_1914: - Leg_state_271 + 999999 H_339 <= 999998.9
 H_Physic_Sec_1915: Leg_state_269 + 999999 H_340 <= 1.0000026e+06
 H_Physic_Sec_1916: - Leg_state_269 + 999999 H_340 <= 999996.6
 H_Physic_Sec_1917: Leg_state_270 + 999999 H_340 <= 999999.4
 H_Physic_Sec_1918: - Leg_state_270 + 999999 H_340 <= 999999.4
 H_Physic_Sec_1919: Leg_state_271 + 999999 H_340 <= 999999
 H_Physic_Sec_1920: - Leg_state_271 + 999999 H_340 <= 999999
 H_Physic_Sec_1921: Leg_state_273 + 999999 H_341 <= 999999.475
 H_Physic_Sec_1922: - Leg_state_273 + 999999 H_341 <= 999999.475
 H_Physic_Sec_1923: Leg_state_274 + 999999 H_341 <= 999999.4
 H_Physic_Sec_1924: - Leg_state_274 + 999999 H_341 <= 999999.4
 H_Physic_Sec_1925: Leg_state_275 + 999999 H_341 <= 999999
 H_Physic_Sec_1926: - Leg_state_275 + 999999 H_341 <= 999999
 H_Physic_Sec_1927: Leg_state_273 + 999999 H_342 <= 1.0000003e+06
 H_Physic_Sec_1928: - Leg_state_273 + 999999 H_342 <= 999998.5
 H_Physic_Sec_1929: Leg_state_274 + 999999 H_342 <= 999999.4
 H_Physic_Sec_1930: - Leg_state_274 + 999999 H_342 <= 999999.4
 H_Physic_Sec_1931: Leg_state_275 + 999999 H_342 <= 999999.1
 H_Physic_Sec_1932: - Leg_state_275 + 999999 H_342 <= 999998.9
 H_Physic_Sec_1933: Leg_state_273 + 999999 H_343 <= 1.000000825e+06
 H_Physic_Sec_1934: - Leg_state_273 + 999999 H_343 <= 999997.675
 H_Physic_Sec_1935: Leg_state_274 + 999999 H_343 <= 999999.4
 H_Physic_Sec_1936: - Leg_state_274 + 999999 H_343 <= 999999.4
 H_Physic_Sec_1937: Leg_state_275 + 999999 H_343 <= 999999.2
 H_Physic_Sec_1938: - Leg_state_275 + 999999 H_343 <= 999998.8
 H_Physic_Sec_1939: Leg_state_273 + 999999 H_344 <= 1.00000135e+06
 H_Physic_Sec_1940: - Leg_state_273 + 999999 H_344 <= 999997.15
 H_Physic_Sec_1941: Leg_state_274 + 999999 H_344 <= 999999.4
 H_Physic_Sec_1942: - Leg_state_274 + 999999 H_344 <= 999999.4
 H_Physic_Sec_1943: Leg_state_275 + 999999 H_344 <= 999999.1
 H_Physic_Sec_1944: - Leg_state_275 + 999999 H_344 <= 999998.9
 H_Physic_Sec_1945: Leg_state_273 + 999999 H_345 <= 1.0000026e+06
 H_Physic_Sec_1946: - Leg_state_273 + 999999 H_345 <= 999996.6
 H_Physic_Sec_1947: Leg_state_274 + 999999 H_345 <= 999999.4
 H_Physic_Sec_1948: - Leg_state_274 + 999999 H_345 <= 999999.4
 H_Physic_Sec_1949: Leg_state_275 + 999999 H_345 <= 999999
 H_Physic_Sec_1950: - Leg_state_275 + 999999 H_345 <= 999999
 H_Physic_Sec_1951: Leg_state_277 + 999999 H_346 <= 999999.475
 H_Physic_Sec_1952: - Leg_state_277 + 999999 H_346 <= 999999.475
 H_Physic_Sec_1953: Leg_state_278 + 999999 H_346 <= 999999.4
 H_Physic_Sec_1954: - Leg_state_278 + 999999 H_346 <= 999999.4
 H_Physic_Sec_1955: Leg_state_279 + 999999 H_346 <= 999999
 H_Physic_Sec_1956: - Leg_state_279 + 999999 H_346 <= 999999
 H_Physic_Sec_1957: Leg_state_277 + 999999 H_347 <= 1.0000003e+06
 H_Physic_Sec_1958: - Leg_state_277 + 999999 H_347 <= 999998.5
 H_Physic_Sec_1959: Leg_state_278 + 999999 H_347 <= 999999.4
 H_Physic_Sec_1960: - Leg_state_278 + 999999 H_347 <= 999999.4
 H_Physic_Sec_1961: Leg_state_279 + 999999 H_347 <= 999999.1
 H_Physic_Sec_1962: - Leg_state_279 + 999999 H_347 <= 999998.9
 H_Physic_Sec_1963: Leg_state_277 + 999999 H_348 <= 1.000000825e+06
 H_Physic_Sec_1964: - Leg_state_277 + 999999 H_348 <= 999997.675
 H_Physic_Sec_1965: Leg_state_278 + 999999 H_348 <= 999999.4
 H_Physic_Sec_1966: - Leg_state_278 + 999999 H_348 <= 999999.4
 H_Physic_Sec_1967: Leg_state_279 + 999999 H_348 <= 999999.2
 H_Physic_Sec_1968: - Leg_state_279 + 999999 H_348 <= 999998.8
 H_Physic_Sec_1969: Leg_state_277 + 999999 H_349 <= 1.00000135e+06
 H_Physic_Sec_1970: - Leg_state_277 + 999999 H_349 <= 999997.15
 H_Physic_Sec_1971: Leg_state_278 + 999999 H_349 <= 999999.4
 H_Physic_Sec_1972: - Leg_state_278 + 999999 H_349 <= 999999.4
 H_Physic_Sec_1973: Leg_state_279 + 999999 H_349 <= 999999.1
 H_Physic_Sec_1974: - Leg_state_279 + 999999 H_349 <= 999998.9
 H_Physic_Sec_1975: Leg_state_277 + 999999 H_350 <= 1.0000026e+06
 H_Physic_Sec_1976: - Leg_state_277 + 999999 H_350 <= 999996.6
 H_Physic_Sec_1977: Leg_state_278 + 999999 H_350 <= 999999.4
 H_Physic_Sec_1978: - Leg_state_278 + 999999 H_350 <= 999999.4
 H_Physic_Sec_1979: Leg_state_279 + 999999 H_350 <= 999999
 H_Physic_Sec_1980: - Leg_state_279 + 999999 H_350 <= 999999
 H_Physic_Sec_1981: Leg_state_281 + 999999 H_351 <= 999999.475
 H_Physic_Sec_1982: - Leg_state_281 + 999999 H_351 <= 999999.475
 H_Physic_Sec_1983: Leg_state_282 + 999999 H_351 <= 999999.4
 H_Physic_Sec_1984: - Leg_state_282 + 999999 H_351 <= 999999.4
 H_Physic_Sec_1985: Leg_state_283 + 999999 H_351 <= 999999
 H_Physic_Sec_1986: - Leg_state_283 + 999999 H_351 <= 999999
 H_Physic_Sec_1987: Leg_state_281 + 999999 H_352 <= 1.0000003e+06
 H_Physic_Sec_1988: - Leg_state_281 + 999999 H_352 <= 999998.5
 H_Physic_Sec_1989: Leg_state_282 + 999999 H_352 <= 999999.4
 H_Physic_Sec_1990: - Leg_state_282 + 999999 H_352 <= 999999.4
 H_Physic_Sec_1991: Leg_state_283 + 999999 H_352 <= 999999.1
 H_Physic_Sec_1992: - Leg_state_283 + 999999 H_352 <= 999998.9
 H_Physic_Sec_1993: Leg_state_281 + 999999 H_353 <= 1.000000825e+06
 H_Physic_Sec_1994: - Leg_state_281 + 999999 H_353 <= 999997.675
 H_Physic_Sec_1995: Leg_state_282 + 999999 H_353 <= 999999.4
 H_Physic_Sec_1996: - Leg_state_282 + 999999 H_353 <= 999999.4
 H_Physic_Sec_1997: Leg_state_283 + 999999 H_353 <= 999999.2
 H_Physic_Sec_1998: - Leg_state_283 + 999999 H_353 <= 999998.8
 H_Physic_Sec_1999: Leg_state_281 + 999999 H_354 <= 1.00000135e+06
 H_Physic_Sec_2000: - Leg_state_281 + 999999 H_354 <= 999997.15
 H_Physic_Sec_2001: Leg_state_282 + 999999 H_354 <= 999999.4
 H_Physic_Sec_2002: - Leg_state_282 + 999999 H_354 <= 999999.4
 H_Physic_Sec_2003: Leg_state_283 + 999999 H_354 <= 999999.1
 H_Physic_Sec_2004: - Leg_state_283 + 999999 H_354 <= 999998.9
 H_Physic_Sec_2005: Leg_state_281 + 999999 H_355 <= 1.0000026e+06
 H_Physic_Sec_2006: - Leg_state_281 + 999999 H_355 <= 999996.6
 H_Physic_Sec_2007: Leg_state_282 + 999999 H_355 <= 999999.4
 H_Physic_Sec_2008: - Leg_state_282 + 999999 H_355 <= 999999.4
 H_Physic_Sec_2009: Leg_state_283 + 999999 H_355 <= 999999
 H_Physic_Sec_2010: - Leg_state_283 + 999999 H_355 <= 999999
 H_Physic_Sec_2011: Leg_state_285 + 999999 H_356 <= 999999.475
 H_Physic_Sec_2012: - Leg_state_285 + 999999 H_356 <= 999999.475
 H_Physic_Sec_2013: Leg_state_286 + 999999 H_356 <= 999999.4
 H_Physic_Sec_2014: - Leg_state_286 + 999999 H_356 <= 999999.4
 H_Physic_Sec_2015: Leg_state_287 + 999999 H_356 <= 999999
 H_Physic_Sec_2016: - Leg_state_287 + 999999 H_356 <= 999999
 H_Physic_Sec_2017: Leg_state_285 + 999999 H_357 <= 1.0000003e+06
 H_Physic_Sec_2018: - Leg_state_285 + 999999 H_357 <= 999998.5
 H_Physic_Sec_2019: Leg_state_286 + 999999 H_357 <= 999999.4
 H_Physic_Sec_2020: - Leg_state_286 + 999999 H_357 <= 999999.4
 H_Physic_Sec_2021: Leg_state_287 + 999999 H_357 <= 999999.1
 H_Physic_Sec_2022: - Leg_state_287 + 999999 H_357 <= 999998.9
 H_Physic_Sec_2023: Leg_state_285 + 999999 H_358 <= 1.000000825e+06
 H_Physic_Sec_2024: - Leg_state_285 + 999999 H_358 <= 999997.675
 H_Physic_Sec_2025: Leg_state_286 + 999999 H_358 <= 999999.4
 H_Physic_Sec_2026: - Leg_state_286 + 999999 H_358 <= 999999.4
 H_Physic_Sec_2027: Leg_state_287 + 999999 H_358 <= 999999.2
 H_Physic_Sec_2028: - Leg_state_287 + 999999 H_358 <= 999998.8
 H_Physic_Sec_2029: Leg_state_285 + 999999 H_359 <= 1.00000135e+06
 H_Physic_Sec_2030: - Leg_state_285 + 999999 H_359 <= 999997.15
 H_Physic_Sec_2031: Leg_state_286 + 999999 H_359 <= 999999.4
 H_Physic_Sec_2032: - Leg_state_286 + 999999 H_359 <= 999999.4
 H_Physic_Sec_2033: Leg_state_287 + 999999 H_359 <= 999999.1
 H_Physic_Sec_2034: - Leg_state_287 + 999999 H_359 <= 999998.9
 H_Physic_Sec_2035: Leg_state_285 + 999999 H_360 <= 1.0000026e+06
 H_Physic_Sec_2036: - Leg_state_285 + 999999 H_360 <= 999996.6
 H_Physic_Sec_2037: Leg_state_286 + 999999 H_360 <= 999999.4
 H_Physic_Sec_2038: - Leg_state_286 + 999999 H_360 <= 999999.4
 H_Physic_Sec_2039: Leg_state_287 + 999999 H_360 <= 999999
 H_Physic_Sec_2040: - Leg_state_287 + 999999 H_360 <= 999999
 H_Physic_Sec_2041: Leg_state_289 + 999999 H_361 <= 999999.475
 H_Physic_Sec_2042: - Leg_state_289 + 999999 H_361 <= 999999.475
 H_Physic_Sec_2043: Leg_state_290 + 999999 H_361 <= 999999.4
 H_Physic_Sec_2044: - Leg_state_290 + 999999 H_361 <= 999999.4
 H_Physic_Sec_2045: Leg_state_291 + 999999 H_361 <= 999999
 H_Physic_Sec_2046: - Leg_state_291 + 999999 H_361 <= 999999
 H_Physic_Sec_2047: Leg_state_289 + 999999 H_362 <= 1.0000003e+06
 H_Physic_Sec_2048: - Leg_state_289 + 999999 H_362 <= 999998.5
 H_Physic_Sec_2049: Leg_state_290 + 999999 H_362 <= 999999.4
 H_Physic_Sec_2050: - Leg_state_290 + 999999 H_362 <= 999999.4
 H_Physic_Sec_2051: Leg_state_291 + 999999 H_362 <= 999999.1
 H_Physic_Sec_2052: - Leg_state_291 + 999999 H_362 <= 999998.9
 H_Physic_Sec_2053: Leg_state_289 + 999999 H_363 <= 1.000000825e+06
 H_Physic_Sec_2054: - Leg_state_289 + 999999 H_363 <= 999997.675
 H_Physic_Sec_2055: Leg_state_290 + 999999 H_363 <= 999999.4
 H_Physic_Sec_2056: - Leg_state_290 + 999999 H_363 <= 999999.4
 H_Physic_Sec_2057: Leg_state_291 + 999999 H_363 <= 999999.2
 H_Physic_Sec_2058: - Leg_state_291 + 999999 H_363 <= 999998.8
 H_Physic_Sec_2059: Leg_state_289 + 999999 H_364 <= 1.00000135e+06
 H_Physic_Sec_2060: - Leg_state_289 + 999999 H_364 <= 999997.15
 H_Physic_Sec_2061: Leg_state_290 + 999999 H_364 <= 999999.4
 H_Physic_Sec_2062: - Leg_state_290 + 999999 H_364 <= 999999.4
 H_Physic_Sec_2063: Leg_state_291 + 999999 H_364 <= 999999.1
 H_Physic_Sec_2064: - Leg_state_291 + 999999 H_364 <= 999998.9
 H_Physic_Sec_2065: Leg_state_289 + 999999 H_365 <= 1.0000026e+06
 H_Physic_Sec_2066: - Leg_state_289 + 999999 H_365 <= 999996.6
 H_Physic_Sec_2067: Leg_state_290 + 999999 H_365 <= 999999.4
 H_Physic_Sec_2068: - Leg_state_290 + 999999 H_365 <= 999999.4
 H_Physic_Sec_2069: Leg_state_291 + 999999 H_365 <= 999999
 H_Physic_Sec_2070: - Leg_state_291 + 999999 H_365 <= 999999
 H_Physic_Sec_2071: Leg_state_293 + 999999 H_366 <= 999999.475
 H_Physic_Sec_2072: - Leg_state_293 + 999999 H_366 <= 999999.475
 H_Physic_Sec_2073: Leg_state_294 + 999999 H_366 <= 999999.4
 H_Physic_Sec_2074: - Leg_state_294 + 999999 H_366 <= 999999.4
 H_Physic_Sec_2075: Leg_state_295 + 999999 H_366 <= 999999
 H_Physic_Sec_2076: - Leg_state_295 + 999999 H_366 <= 999999
 H_Physic_Sec_2077: Leg_state_293 + 999999 H_367 <= 1.0000003e+06
 H_Physic_Sec_2078: - Leg_state_293 + 999999 H_367 <= 999998.5
 H_Physic_Sec_2079: Leg_state_294 + 999999 H_367 <= 999999.4
 H_Physic_Sec_2080: - Leg_state_294 + 999999 H_367 <= 999999.4
 H_Physic_Sec_2081: Leg_state_295 + 999999 H_367 <= 999999.1
 H_Physic_Sec_2082: - Leg_state_295 + 999999 H_367 <= 999998.9
 H_Physic_Sec_2083: Leg_state_293 + 999999 H_368 <= 1.000000825e+06
 H_Physic_Sec_2084: - Leg_state_293 + 999999 H_368 <= 999997.675
 H_Physic_Sec_2085: Leg_state_294 + 999999 H_368 <= 999999.4
 H_Physic_Sec_2086: - Leg_state_294 + 999999 H_368 <= 999999.4
 H_Physic_Sec_2087: Leg_state_295 + 999999 H_368 <= 999999.2
 H_Physic_Sec_2088: - Leg_state_295 + 999999 H_368 <= 999998.8
 H_Physic_Sec_2089: Leg_state_293 + 999999 H_369 <= 1.00000135e+06
 H_Physic_Sec_2090: - Leg_state_293 + 999999 H_369 <= 999997.15
 H_Physic_Sec_2091: Leg_state_294 + 999999 H_369 <= 999999.4
 H_Physic_Sec_2092: - Leg_state_294 + 999999 H_369 <= 999999.4
 H_Physic_Sec_2093: Leg_state_295 + 999999 H_369 <= 999999.1
 H_Physic_Sec_2094: - Leg_state_295 + 999999 H_369 <= 999998.9
 H_Physic_Sec_2095: Leg_state_293 + 999999 H_370 <= 1.0000026e+06
 H_Physic_Sec_2096: - Leg_state_293 + 999999 H_370 <= 999996.6
 H_Physic_Sec_2097: Leg_state_294 + 999999 H_370 <= 999999.4
 H_Physic_Sec_2098: - Leg_state_294 + 999999 H_370 <= 999999.4
 H_Physic_Sec_2099: Leg_state_295 + 999999 H_370 <= 999999
 H_Physic_Sec_2100: - Leg_state_295 + 999999 H_370 <= 999999
 H_Physic_Sec_2101: Leg_state_297 + 999999 H_371 <= 999999.475
 H_Physic_Sec_2102: - Leg_state_297 + 999999 H_371 <= 999999.475
 H_Physic_Sec_2103: Leg_state_298 + 999999 H_371 <= 999999.4
 H_Physic_Sec_2104: - Leg_state_298 + 999999 H_371 <= 999999.4
 H_Physic_Sec_2105: Leg_state_299 + 999999 H_371 <= 999999
 H_Physic_Sec_2106: - Leg_state_299 + 999999 H_371 <= 999999
 H_Physic_Sec_2107: Leg_state_297 + 999999 H_372 <= 1.0000003e+06
 H_Physic_Sec_2108: - Leg_state_297 + 999999 H_372 <= 999998.5
 H_Physic_Sec_2109: Leg_state_298 + 999999 H_372 <= 999999.4
 H_Physic_Sec_2110: - Leg_state_298 + 999999 H_372 <= 999999.4
 H_Physic_Sec_2111: Leg_state_299 + 999999 H_372 <= 999999.1
 H_Physic_Sec_2112: - Leg_state_299 + 999999 H_372 <= 999998.9
 H_Physic_Sec_2113: Leg_state_297 + 999999 H_373 <= 1.000000825e+06
 H_Physic_Sec_2114: - Leg_state_297 + 999999 H_373 <= 999997.675
 H_Physic_Sec_2115: Leg_state_298 + 999999 H_373 <= 999999.4
 H_Physic_Sec_2116: - Leg_state_298 + 999999 H_373 <= 999999.4
 H_Physic_Sec_2117: Leg_state_299 + 999999 H_373 <= 999999.2
 H_Physic_Sec_2118: - Leg_state_299 + 999999 H_373 <= 999998.8
 H_Physic_Sec_2119: Leg_state_297 + 999999 H_374 <= 1.00000135e+06
 H_Physic_Sec_2120: - Leg_state_297 + 999999 H_374 <= 999997.15
 H_Physic_Sec_2121: Leg_state_298 + 999999 H_374 <= 999999.4
 H_Physic_Sec_2122: - Leg_state_298 + 999999 H_374 <= 999999.4
 H_Physic_Sec_2123: Leg_state_299 + 999999 H_374 <= 999999.1
 H_Physic_Sec_2124: - Leg_state_299 + 999999 H_374 <= 999998.9
 H_Physic_Sec_2125: Leg_state_297 + 999999 H_375 <= 1.0000026e+06
 H_Physic_Sec_2126: - Leg_state_297 + 999999 H_375 <= 999996.6
 H_Physic_Sec_2127: Leg_state_298 + 999999 H_375 <= 999999.4
 H_Physic_Sec_2128: - Leg_state_298 + 999999 H_375 <= 999999.4
 H_Physic_Sec_2129: Leg_state_299 + 999999 H_375 <= 999999
 H_Physic_Sec_2130: - Leg_state_299 + 999999 H_375 <= 999999
 H_Physic_Sec_2131: Leg_state_301 + 999999 H_376 <= 999999.475
 H_Physic_Sec_2132: - Leg_state_301 + 999999 H_376 <= 999999.475
 H_Physic_Sec_2133: Leg_state_302 + 999999 H_376 <= 999999.4
 H_Physic_Sec_2134: - Leg_state_302 + 999999 H_376 <= 999999.4
 H_Physic_Sec_2135: Leg_state_303 + 999999 H_376 <= 999999
 H_Physic_Sec_2136: - Leg_state_303 + 999999 H_376 <= 999999
 H_Physic_Sec_2137: Leg_state_301 + 999999 H_377 <= 1.0000003e+06
 H_Physic_Sec_2138: - Leg_state_301 + 999999 H_377 <= 999998.5
 H_Physic_Sec_2139: Leg_state_302 + 999999 H_377 <= 999999.4
 H_Physic_Sec_2140: - Leg_state_302 + 999999 H_377 <= 999999.4
 H_Physic_Sec_2141: Leg_state_303 + 999999 H_377 <= 999999.1
 H_Physic_Sec_2142: - Leg_state_303 + 999999 H_377 <= 999998.9
 H_Physic_Sec_2143: Leg_state_301 + 999999 H_378 <= 1.000000825e+06
 H_Physic_Sec_2144: - Leg_state_301 + 999999 H_378 <= 999997.675
 H_Physic_Sec_2145: Leg_state_302 + 999999 H_378 <= 999999.4
 H_Physic_Sec_2146: - Leg_state_302 + 999999 H_378 <= 999999.4
 H_Physic_Sec_2147: Leg_state_303 + 999999 H_378 <= 999999.2
 H_Physic_Sec_2148: - Leg_state_303 + 999999 H_378 <= 999998.8
 H_Physic_Sec_2149: Leg_state_301 + 999999 H_379 <= 1.00000135e+06
 H_Physic_Sec_2150: - Leg_state_301 + 999999 H_379 <= 999997.15
 H_Physic_Sec_2151: Leg_state_302 + 999999 H_379 <= 999999.4
 H_Physic_Sec_2152: - Leg_state_302 + 999999 H_379 <= 999999.4
 H_Physic_Sec_2153: Leg_state_303 + 999999 H_379 <= 999999.1
 H_Physic_Sec_2154: - Leg_state_303 + 999999 H_379 <= 999998.9
 H_Physic_Sec_2155: Leg_state_301 + 999999 H_380 <= 1.0000026e+06
 H_Physic_Sec_2156: - Leg_state_301 + 999999 H_380 <= 999996.6
 H_Physic_Sec_2157: Leg_state_302 + 999999 H_380 <= 999999.4
 H_Physic_Sec_2158: - Leg_state_302 + 999999 H_380 <= 999999.4
 H_Physic_Sec_2159: Leg_state_303 + 999999 H_380 <= 999999
 H_Physic_Sec_2160: - Leg_state_303 + 999999 H_380 <= 999999
 H_Physic_Sec_2161: Leg_state_305 + 999999 H_381 <= 999999.475
 H_Physic_Sec_2162: - Leg_state_305 + 999999 H_381 <= 999999.475
 H_Physic_Sec_2163: Leg_state_306 + 999999 H_381 <= 999999.4
 H_Physic_Sec_2164: - Leg_state_306 + 999999 H_381 <= 999999.4
 H_Physic_Sec_2165: Leg_state_307 + 999999 H_381 <= 999999
 H_Physic_Sec_2166: - Leg_state_307 + 999999 H_381 <= 999999
 H_Physic_Sec_2167: Leg_state_305 + 999999 H_382 <= 1.0000003e+06
 H_Physic_Sec_2168: - Leg_state_305 + 999999 H_382 <= 999998.5
 H_Physic_Sec_2169: Leg_state_306 + 999999 H_382 <= 999999.4
 H_Physic_Sec_2170: - Leg_state_306 + 999999 H_382 <= 999999.4
 H_Physic_Sec_2171: Leg_state_307 + 999999 H_382 <= 999999.1
 H_Physic_Sec_2172: - Leg_state_307 + 999999 H_382 <= 999998.9
 H_Physic_Sec_2173: Leg_state_305 + 999999 H_383 <= 1.000000825e+06
 H_Physic_Sec_2174: - Leg_state_305 + 999999 H_383 <= 999997.675
 H_Physic_Sec_2175: Leg_state_306 + 999999 H_383 <= 999999.4
 H_Physic_Sec_2176: - Leg_state_306 + 999999 H_383 <= 999999.4
 H_Physic_Sec_2177: Leg_state_307 + 999999 H_383 <= 999999.2
 H_Physic_Sec_2178: - Leg_state_307 + 999999 H_383 <= 999998.8
 H_Physic_Sec_2179: Leg_state_305 + 999999 H_384 <= 1.00000135e+06
 H_Physic_Sec_2180: - Leg_state_305 + 999999 H_384 <= 999997.15
 H_Physic_Sec_2181: Leg_state_306 + 999999 H_384 <= 999999.4
 H_Physic_Sec_2182: - Leg_state_306 + 999999 H_384 <= 999999.4
 H_Physic_Sec_2183: Leg_state_307 + 999999 H_384 <= 999999.1
 H_Physic_Sec_2184: - Leg_state_307 + 999999 H_384 <= 999998.9
 H_Physic_Sec_2185: Leg_state_305 + 999999 H_385 <= 1.0000026e+06
 H_Physic_Sec_2186: - Leg_state_305 + 999999 H_385 <= 999996.6
 H_Physic_Sec_2187: Leg_state_306 + 999999 H_385 <= 999999.4
 H_Physic_Sec_2188: - Leg_state_306 + 999999 H_385 <= 999999.4
 H_Physic_Sec_2189: Leg_state_307 + 999999 H_385 <= 999999
 H_Physic_Sec_2190: - Leg_state_307 + 999999 H_385 <= 999999
 H_Physic_Sec_2191: Leg_state_309 + 999999 H_386 <= 999999.475
 H_Physic_Sec_2192: - Leg_state_309 + 999999 H_386 <= 999999.475
 H_Physic_Sec_2193: Leg_state_310 + 999999 H_386 <= 999999.4
 H_Physic_Sec_2194: - Leg_state_310 + 999999 H_386 <= 999999.4
 H_Physic_Sec_2195: Leg_state_311 + 999999 H_386 <= 999999
 H_Physic_Sec_2196: - Leg_state_311 + 999999 H_386 <= 999999
 H_Physic_Sec_2197: Leg_state_309 + 999999 H_387 <= 1.0000003e+06
 H_Physic_Sec_2198: - Leg_state_309 + 999999 H_387 <= 999998.5
 H_Physic_Sec_2199: Leg_state_310 + 999999 H_387 <= 999999.4
 H_Physic_Sec_2200: - Leg_state_310 + 999999 H_387 <= 999999.4
 H_Physic_Sec_2201: Leg_state_311 + 999999 H_387 <= 999999.1
 H_Physic_Sec_2202: - Leg_state_311 + 999999 H_387 <= 999998.9
 H_Physic_Sec_2203: Leg_state_309 + 999999 H_388 <= 1.000000825e+06
 H_Physic_Sec_2204: - Leg_state_309 + 999999 H_388 <= 999997.675
 H_Physic_Sec_2205: Leg_state_310 + 999999 H_388 <= 999999.4
 H_Physic_Sec_2206: - Leg_state_310 + 999999 H_388 <= 999999.4
 H_Physic_Sec_2207: Leg_state_311 + 999999 H_388 <= 999999.2
 H_Physic_Sec_2208: - Leg_state_311 + 999999 H_388 <= 999998.8
 H_Physic_Sec_2209: Leg_state_309 + 999999 H_389 <= 1.00000135e+06
 H_Physic_Sec_2210: - Leg_state_309 + 999999 H_389 <= 999997.15
 H_Physic_Sec_2211: Leg_state_310 + 999999 H_389 <= 999999.4
 H_Physic_Sec_2212: - Leg_state_310 + 999999 H_389 <= 999999.4
 H_Physic_Sec_2213: Leg_state_311 + 999999 H_389 <= 999999.1
 H_Physic_Sec_2214: - Leg_state_311 + 999999 H_389 <= 999998.9
 H_Physic_Sec_2215: Leg_state_309 + 999999 H_390 <= 1.0000026e+06
 H_Physic_Sec_2216: - Leg_state_309 + 999999 H_390 <= 999996.6
 H_Physic_Sec_2217: Leg_state_310 + 999999 H_390 <= 999999.4
 H_Physic_Sec_2218: - Leg_state_310 + 999999 H_390 <= 999999.4
 H_Physic_Sec_2219: Leg_state_311 + 999999 H_390 <= 999999
 H_Physic_Sec_2220: - Leg_state_311 + 999999 H_390 <= 999999
 H_Physic_Sec_2221: Leg_state_313 + 999999 H_391 <= 999999.475
 H_Physic_Sec_2222: - Leg_state_313 + 999999 H_391 <= 999999.475
 H_Physic_Sec_2223: Leg_state_314 + 999999 H_391 <= 999999.4
 H_Physic_Sec_2224: - Leg_state_314 + 999999 H_391 <= 999999.4
 H_Physic_Sec_2225: Leg_state_315 + 999999 H_391 <= 999999
 H_Physic_Sec_2226: - Leg_state_315 + 999999 H_391 <= 999999
 H_Physic_Sec_2227: Leg_state_313 + 999999 H_392 <= 1.0000003e+06
 H_Physic_Sec_2228: - Leg_state_313 + 999999 H_392 <= 999998.5
 H_Physic_Sec_2229: Leg_state_314 + 999999 H_392 <= 999999.4
 H_Physic_Sec_2230: - Leg_state_314 + 999999 H_392 <= 999999.4
 H_Physic_Sec_2231: Leg_state_315 + 999999 H_392 <= 999999.1
 H_Physic_Sec_2232: - Leg_state_315 + 999999 H_392 <= 999998.9
 H_Physic_Sec_2233: Leg_state_313 + 999999 H_393 <= 1.000000825e+06
 H_Physic_Sec_2234: - Leg_state_313 + 999999 H_393 <= 999997.675
 H_Physic_Sec_2235: Leg_state_314 + 999999 H_393 <= 999999.4
 H_Physic_Sec_2236: - Leg_state_314 + 999999 H_393 <= 999999.4
 H_Physic_Sec_2237: Leg_state_315 + 999999 H_393 <= 999999.2
 H_Physic_Sec_2238: - Leg_state_315 + 999999 H_393 <= 999998.8
 H_Physic_Sec_2239: Leg_state_313 + 999999 H_394 <= 1.00000135e+06
 H_Physic_Sec_2240: - Leg_state_313 + 999999 H_394 <= 999997.15
 H_Physic_Sec_2241: Leg_state_314 + 999999 H_394 <= 999999.4
 H_Physic_Sec_2242: - Leg_state_314 + 999999 H_394 <= 999999.4
 H_Physic_Sec_2243: Leg_state_315 + 999999 H_394 <= 999999.1
 H_Physic_Sec_2244: - Leg_state_315 + 999999 H_394 <= 999998.9
 H_Physic_Sec_2245: Leg_state_313 + 999999 H_395 <= 1.0000026e+06
 H_Physic_Sec_2246: - Leg_state_313 + 999999 H_395 <= 999996.6
 H_Physic_Sec_2247: Leg_state_314 + 999999 H_395 <= 999999.4
 H_Physic_Sec_2248: - Leg_state_314 + 999999 H_395 <= 999999.4
 H_Physic_Sec_2249: Leg_state_315 + 999999 H_395 <= 999999
 H_Physic_Sec_2250: - Leg_state_315 + 999999 H_395 <= 999999
 H_Physic_Sec_2251: Leg_state_317 + 999999 H_396 <= 999999.475
 H_Physic_Sec_2252: - Leg_state_317 + 999999 H_396 <= 999999.475
 H_Physic_Sec_2253: Leg_state_318 + 999999 H_396 <= 999999.4
 H_Physic_Sec_2254: - Leg_state_318 + 999999 H_396 <= 999999.4
 H_Physic_Sec_2255: Leg_state_319 + 999999 H_396 <= 999999
 H_Physic_Sec_2256: - Leg_state_319 + 999999 H_396 <= 999999
 H_Physic_Sec_2257: Leg_state_317 + 999999 H_397 <= 1.0000003e+06
 H_Physic_Sec_2258: - Leg_state_317 + 999999 H_397 <= 999998.5
 H_Physic_Sec_2259: Leg_state_318 + 999999 H_397 <= 999999.4
 H_Physic_Sec_2260: - Leg_state_318 + 999999 H_397 <= 999999.4
 H_Physic_Sec_2261: Leg_state_319 + 999999 H_397 <= 999999.1
 H_Physic_Sec_2262: - Leg_state_319 + 999999 H_397 <= 999998.9
 H_Physic_Sec_2263: Leg_state_317 + 999999 H_398 <= 1.000000825e+06
 H_Physic_Sec_2264: - Leg_state_317 + 999999 H_398 <= 999997.675
 H_Physic_Sec_2265: Leg_state_318 + 999999 H_398 <= 999999.4
 H_Physic_Sec_2266: - Leg_state_318 + 999999 H_398 <= 999999.4
 H_Physic_Sec_2267: Leg_state_319 + 999999 H_398 <= 999999.2
 H_Physic_Sec_2268: - Leg_state_319 + 999999 H_398 <= 999998.8
 H_Physic_Sec_2269: Leg_state_317 + 999999 H_399 <= 1.00000135e+06
 H_Physic_Sec_2270: - Leg_state_317 + 999999 H_399 <= 999997.15
 H_Physic_Sec_2271: Leg_state_318 + 999999 H_399 <= 999999.4
 H_Physic_Sec_2272: - Leg_state_318 + 999999 H_399 <= 999999.4
 H_Physic_Sec_2273: Leg_state_319 + 999999 H_399 <= 999999.1
 H_Physic_Sec_2274: - Leg_state_319 + 999999 H_399 <= 999998.9
 H_Physic_Sec_2275: Leg_state_317 + 999999 H_400 <= 1.0000026e+06
 H_Physic_Sec_2276: - Leg_state_317 + 999999 H_400 <= 999996.6
 H_Physic_Sec_2277: Leg_state_318 + 999999 H_400 <= 999999.4
 H_Physic_Sec_2278: - Leg_state_318 + 999999 H_400 <= 999999.4
 H_Physic_Sec_2279: Leg_state_319 + 999999 H_400 <= 999999
 H_Physic_Sec_2280: - Leg_state_319 + 999999 H_400 <= 999999
 H_Physic_Sec_2281: Leg_state_321 + 999999 H_401 <= 999999.475
 H_Physic_Sec_2282: - Leg_state_321 + 999999 H_401 <= 999999.475
 H_Physic_Sec_2283: Leg_state_322 + 999999 H_401 <= 999999.4
 H_Physic_Sec_2284: - Leg_state_322 + 999999 H_401 <= 999999.4
 H_Physic_Sec_2285: Leg_state_323 + 999999 H_401 <= 999999
 H_Physic_Sec_2286: - Leg_state_323 + 999999 H_401 <= 999999
 H_Physic_Sec_2287: Leg_state_321 + 999999 H_402 <= 1.0000003e+06
 H_Physic_Sec_2288: - Leg_state_321 + 999999 H_402 <= 999998.5
 H_Physic_Sec_2289: Leg_state_322 + 999999 H_402 <= 999999.4
 H_Physic_Sec_2290: - Leg_state_322 + 999999 H_402 <= 999999.4
 H_Physic_Sec_2291: Leg_state_323 + 999999 H_402 <= 999999.1
 H_Physic_Sec_2292: - Leg_state_323 + 999999 H_402 <= 999998.9
 H_Physic_Sec_2293: Leg_state_321 + 999999 H_403 <= 1.000000825e+06
 H_Physic_Sec_2294: - Leg_state_321 + 999999 H_403 <= 999997.675
 H_Physic_Sec_2295: Leg_state_322 + 999999 H_403 <= 999999.4
 H_Physic_Sec_2296: - Leg_state_322 + 999999 H_403 <= 999999.4
 H_Physic_Sec_2297: Leg_state_323 + 999999 H_403 <= 999999.2
 H_Physic_Sec_2298: - Leg_state_323 + 999999 H_403 <= 999998.8
 H_Physic_Sec_2299: Leg_state_321 + 999999 H_404 <= 1.00000135e+06
 H_Physic_Sec_2300: - Leg_state_321 + 999999 H_404 <= 999997.15
 H_Physic_Sec_2301: Leg_state_322 + 999999 H_404 <= 999999.4
 H_Physic_Sec_2302: - Leg_state_322 + 999999 H_404 <= 999999.4
 H_Physic_Sec_2303: Leg_state_323 + 999999 H_404 <= 999999.1
 H_Physic_Sec_2304: - Leg_state_323 + 999999 H_404 <= 999998.9
 H_Physic_Sec_2305: Leg_state_321 + 999999 H_405 <= 1.0000026e+06
 H_Physic_Sec_2306: - Leg_state_321 + 999999 H_405 <= 999996.6
 H_Physic_Sec_2307: Leg_state_322 + 999999 H_405 <= 999999.4
 H_Physic_Sec_2308: - Leg_state_322 + 999999 H_405 <= 999999.4
 H_Physic_Sec_2309: Leg_state_323 + 999999 H_405 <= 999999
 H_Physic_Sec_2310: - Leg_state_323 + 999999 H_405 <= 999999
 H_Physic_Sec_2311: Leg_state_325 + 999999 H_406 <= 999999.475
 H_Physic_Sec_2312: - Leg_state_325 + 999999 H_406 <= 999999.475
 H_Physic_Sec_2313: Leg_state_326 + 999999 H_406 <= 999999.4
 H_Physic_Sec_2314: - Leg_state_326 + 999999 H_406 <= 999999.4
 H_Physic_Sec_2315: Leg_state_327 + 999999 H_406 <= 999999
 H_Physic_Sec_2316: - Leg_state_327 + 999999 H_406 <= 999999
 H_Physic_Sec_2317: Leg_state_325 + 999999 H_407 <= 1.0000003e+06
 H_Physic_Sec_2318: - Leg_state_325 + 999999 H_407 <= 999998.5
 H_Physic_Sec_2319: Leg_state_326 + 999999 H_407 <= 999999.4
 H_Physic_Sec_2320: - Leg_state_326 + 999999 H_407 <= 999999.4
 H_Physic_Sec_2321: Leg_state_327 + 999999 H_407 <= 999999.1
 H_Physic_Sec_2322: - Leg_state_327 + 999999 H_407 <= 999998.9
 H_Physic_Sec_2323: Leg_state_325 + 999999 H_408 <= 1.000000825e+06
 H_Physic_Sec_2324: - Leg_state_325 + 999999 H_408 <= 999997.675
 H_Physic_Sec_2325: Leg_state_326 + 999999 H_408 <= 999999.4
 H_Physic_Sec_2326: - Leg_state_326 + 999999 H_408 <= 999999.4
 H_Physic_Sec_2327: Leg_state_327 + 999999 H_408 <= 999999.2
 H_Physic_Sec_2328: - Leg_state_327 + 999999 H_408 <= 999998.8
 H_Physic_Sec_2329: Leg_state_325 + 999999 H_409 <= 1.00000135e+06
 H_Physic_Sec_2330: - Leg_state_325 + 999999 H_409 <= 999997.15
 H_Physic_Sec_2331: Leg_state_326 + 999999 H_409 <= 999999.4
 H_Physic_Sec_2332: - Leg_state_326 + 999999 H_409 <= 999999.4
 H_Physic_Sec_2333: Leg_state_327 + 999999 H_409 <= 999999.1
 H_Physic_Sec_2334: - Leg_state_327 + 999999 H_409 <= 999998.9
 H_Physic_Sec_2335: Leg_state_325 + 999999 H_410 <= 1.0000026e+06
 H_Physic_Sec_2336: - Leg_state_325 + 999999 H_410 <= 999996.6
 H_Physic_Sec_2337: Leg_state_326 + 999999 H_410 <= 999999.4
 H_Physic_Sec_2338: - Leg_state_326 + 999999 H_410 <= 999999.4
 H_Physic_Sec_2339: Leg_state_327 + 999999 H_410 <= 999999
 H_Physic_Sec_2340: - Leg_state_327 + 999999 H_410 <= 999999
 H_Physic_Sec_2341: Leg_state_329 + 999999 H_411 <= 999999.475
 H_Physic_Sec_2342: - Leg_state_329 + 999999 H_411 <= 999999.475
 H_Physic_Sec_2343: Leg_state_330 + 999999 H_411 <= 999999.4
 H_Physic_Sec_2344: - Leg_state_330 + 999999 H_411 <= 999999.4
 H_Physic_Sec_2345: Leg_state_331 + 999999 H_411 <= 999999
 H_Physic_Sec_2346: - Leg_state_331 + 999999 H_411 <= 999999
 H_Physic_Sec_2347: Leg_state_329 + 999999 H_412 <= 1.0000003e+06
 H_Physic_Sec_2348: - Leg_state_329 + 999999 H_412 <= 999998.5
 H_Physic_Sec_2349: Leg_state_330 + 999999 H_412 <= 999999.4
 H_Physic_Sec_2350: - Leg_state_330 + 999999 H_412 <= 999999.4
 H_Physic_Sec_2351: Leg_state_331 + 999999 H_412 <= 999999.1
 H_Physic_Sec_2352: - Leg_state_331 + 999999 H_412 <= 999998.9
 H_Physic_Sec_2353: Leg_state_329 + 999999 H_413 <= 1.000000825e+06
 H_Physic_Sec_2354: - Leg_state_329 + 999999 H_413 <= 999997.675
 H_Physic_Sec_2355: Leg_state_330 + 999999 H_413 <= 999999.4
 H_Physic_Sec_2356: - Leg_state_330 + 999999 H_413 <= 999999.4
 H_Physic_Sec_2357: Leg_state_331 + 999999 H_413 <= 999999.2
 H_Physic_Sec_2358: - Leg_state_331 + 999999 H_413 <= 999998.8
 H_Physic_Sec_2359: Leg_state_329 + 999999 H_414 <= 1.00000135e+06
 H_Physic_Sec_2360: - Leg_state_329 + 999999 H_414 <= 999997.15
 H_Physic_Sec_2361: Leg_state_330 + 999999 H_414 <= 999999.4
 H_Physic_Sec_2362: - Leg_state_330 + 999999 H_414 <= 999999.4
 H_Physic_Sec_2363: Leg_state_331 + 999999 H_414 <= 999999.1
 H_Physic_Sec_2364: - Leg_state_331 + 999999 H_414 <= 999998.9
 H_Physic_Sec_2365: Leg_state_329 + 999999 H_415 <= 1.0000026e+06
 H_Physic_Sec_2366: - Leg_state_329 + 999999 H_415 <= 999996.6
 H_Physic_Sec_2367: Leg_state_330 + 999999 H_415 <= 999999.4
 H_Physic_Sec_2368: - Leg_state_330 + 999999 H_415 <= 999999.4
 H_Physic_Sec_2369: Leg_state_331 + 999999 H_415 <= 999999
 H_Physic_Sec_2370: - Leg_state_331 + 999999 H_415 <= 999999
 H_Physic_Sec_2371: Leg_state_333 + 999999 H_416 <= 999999.475
 H_Physic_Sec_2372: - Leg_state_333 + 999999 H_416 <= 999999.475
 H_Physic_Sec_2373: Leg_state_334 + 999999 H_416 <= 999999.4
 H_Physic_Sec_2374: - Leg_state_334 + 999999 H_416 <= 999999.4
 H_Physic_Sec_2375: Leg_state_335 + 999999 H_416 <= 999999
 H_Physic_Sec_2376: - Leg_state_335 + 999999 H_416 <= 999999
 H_Physic_Sec_2377: Leg_state_333 + 999999 H_417 <= 1.0000003e+06
 H_Physic_Sec_2378: - Leg_state_333 + 999999 H_417 <= 999998.5
 H_Physic_Sec_2379: Leg_state_334 + 999999 H_417 <= 999999.4
 H_Physic_Sec_2380: - Leg_state_334 + 999999 H_417 <= 999999.4
 H_Physic_Sec_2381: Leg_state_335 + 999999 H_417 <= 999999.1
 H_Physic_Sec_2382: - Leg_state_335 + 999999 H_417 <= 999998.9
 H_Physic_Sec_2383: Leg_state_333 + 999999 H_418 <= 1.000000825e+06
 H_Physic_Sec_2384: - Leg_state_333 + 999999 H_418 <= 999997.675
 H_Physic_Sec_2385: Leg_state_334 + 999999 H_418 <= 999999.4
 H_Physic_Sec_2386: - Leg_state_334 + 999999 H_418 <= 999999.4
 H_Physic_Sec_2387: Leg_state_335 + 999999 H_418 <= 999999.2
 H_Physic_Sec_2388: - Leg_state_335 + 999999 H_418 <= 999998.8
 H_Physic_Sec_2389: Leg_state_333 + 999999 H_419 <= 1.00000135e+06
 H_Physic_Sec_2390: - Leg_state_333 + 999999 H_419 <= 999997.15
 H_Physic_Sec_2391: Leg_state_334 + 999999 H_419 <= 999999.4
 H_Physic_Sec_2392: - Leg_state_334 + 999999 H_419 <= 999999.4
 H_Physic_Sec_2393: Leg_state_335 + 999999 H_419 <= 999999.1
 H_Physic_Sec_2394: - Leg_state_335 + 999999 H_419 <= 999998.9
 H_Physic_Sec_2395: Leg_state_333 + 999999 H_420 <= 1.0000026e+06
 H_Physic_Sec_2396: - Leg_state_333 + 999999 H_420 <= 999996.6
 H_Physic_Sec_2397: Leg_state_334 + 999999 H_420 <= 999999.4
 H_Physic_Sec_2398: - Leg_state_334 + 999999 H_420 <= 999999.4
 H_Physic_Sec_2399: Leg_state_335 + 999999 H_420 <= 999999
 H_Physic_Sec_2400: - Leg_state_335 + 999999 H_420 <= 999999
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
 COM_state_AND_Leg_state_49: - 4 COM_state_53 + Leg_state_209
   + Leg_state_213 + Leg_state_217 + Leg_state_221 = 0
 COM_state_AND_Leg_state_50: - 4 COM_state_54 + Leg_state_210
   + Leg_state_214 + Leg_state_218 + Leg_state_222 = 0
 COM_state_AND_Leg_state_51: - 4 COM_state_55 + Leg_state_211
   + Leg_state_215 + Leg_state_219 + Leg_state_223 = 0
 COM_state_AND_Leg_state_52: - 4 COM_state_56 + Leg_state_212
   + Leg_state_216 + Leg_state_220 + Leg_state_224 = 0
 COM_state_AND_Leg_state_53: - 4 COM_state_57 + Leg_state_225
   + Leg_state_229 + Leg_state_233 + Leg_state_237 = 0
 COM_state_AND_Leg_state_54: - 4 COM_state_58 + Leg_state_226
   + Leg_state_230 + Leg_state_234 + Leg_state_238 = 0
 COM_state_AND_Leg_state_55: - 4 COM_state_59 + Leg_state_227
   + Leg_state_231 + Leg_state_235 + Leg_state_239 = 0
 COM_state_AND_Leg_state_56: - 4 COM_state_60 + Leg_state_228
   + Leg_state_232 + Leg_state_236 + Leg_state_240 = 0
 COM_state_AND_Leg_state_57: - 4 COM_state_61 + Leg_state_241
   + Leg_state_245 + Leg_state_249 + Leg_state_253 = 0
 COM_state_AND_Leg_state_58: - 4 COM_state_62 + Leg_state_242
   + Leg_state_246 + Leg_state_250 + Leg_state_254 = 0
 COM_state_AND_Leg_state_59: - 4 COM_state_63 + Leg_state_243
   + Leg_state_247 + Leg_state_251 + Leg_state_255 = 0
 COM_state_AND_Leg_state_60: - 4 COM_state_64 + Leg_state_244
   + Leg_state_248 + Leg_state_252 + Leg_state_256 = 0
 COM_state_AND_Leg_state_61: - 4 COM_state_65 + Leg_state_257
   + Leg_state_261 + Leg_state_265 + Leg_state_269 = 0
 COM_state_AND_Leg_state_62: - 4 COM_state_66 + Leg_state_258
   + Leg_state_262 + Leg_state_266 + Leg_state_270 = 0
 COM_state_AND_Leg_state_63: - 4 COM_state_67 + Leg_state_259
   + Leg_state_263 + Leg_state_267 + Leg_state_271 = 0
 COM_state_AND_Leg_state_64: - 4 COM_state_68 + Leg_state_260
   + Leg_state_264 + Leg_state_268 + Leg_state_272 = 0
 COM_state_AND_Leg_state_65: - 4 COM_state_69 + Leg_state_273
   + Leg_state_277 + Leg_state_281 + Leg_state_285 = 0
 COM_state_AND_Leg_state_66: - 4 COM_state_70 + Leg_state_274
   + Leg_state_278 + Leg_state_282 + Leg_state_286 = 0
 COM_state_AND_Leg_state_67: - 4 COM_state_71 + Leg_state_275
   + Leg_state_279 + Leg_state_283 + Leg_state_287 = 0
 COM_state_AND_Leg_state_68: - 4 COM_state_72 + Leg_state_276
   + Leg_state_280 + Leg_state_284 + Leg_state_288 = 0
 COM_state_AND_Leg_state_69: - 4 COM_state_73 + Leg_state_289
   + Leg_state_293 + Leg_state_297 + Leg_state_301 = 0
 COM_state_AND_Leg_state_70: - 4 COM_state_74 + Leg_state_290
   + Leg_state_294 + Leg_state_298 + Leg_state_302 = 0
 COM_state_AND_Leg_state_71: - 4 COM_state_75 + Leg_state_291
   + Leg_state_295 + Leg_state_299 + Leg_state_303 = 0
 COM_state_AND_Leg_state_72: - 4 COM_state_76 + Leg_state_292
   + Leg_state_296 + Leg_state_300 + Leg_state_304 = 0
 COM_state_AND_Leg_state_73: - 4 COM_state_77 + Leg_state_305
   + Leg_state_309 + Leg_state_313 + Leg_state_317 = 0
 COM_state_AND_Leg_state_74: - 4 COM_state_78 + Leg_state_306
   + Leg_state_310 + Leg_state_314 + Leg_state_318 = 0
 COM_state_AND_Leg_state_75: - 4 COM_state_79 + Leg_state_307
   + Leg_state_311 + Leg_state_315 + Leg_state_319 = 0
 COM_state_AND_Leg_state_76: - 4 COM_state_80 + Leg_state_308
   + Leg_state_312 + Leg_state_316 + Leg_state_320 = 0
 COM_state_AND_Leg_state_77: - 4 COM_state_81 + Leg_state_321
   + Leg_state_325 + Leg_state_329 + Leg_state_333 = 0
 COM_state_AND_Leg_state_78: - 4 COM_state_82 + Leg_state_322
   + Leg_state_326 + Leg_state_330 + Leg_state_334 = 0
 COM_state_AND_Leg_state_79: - 4 COM_state_83 + Leg_state_323
   + Leg_state_327 + Leg_state_331 + Leg_state_335 = 0
 COM_state_AND_Leg_state_80: - 4 COM_state_84 + Leg_state_324
   + Leg_state_328 + Leg_state_332 + Leg_state_336 = 0
 compute_delta_z_1: - COM_state_3 + COM_state_7 - delta_z_2 = 0
 compute_delta_z_2: - COM_state_7 + COM_state_11 - delta_z_3 = 0
 compute_delta_z_3: - COM_state_11 + COM_state_15 - delta_z_4 = 0
 compute_delta_z_4: - COM_state_15 + COM_state_19 - delta_z_5 = 0
 compute_delta_z_5: - COM_state_19 + COM_state_23 - delta_z_6 = 0
 compute_delta_z_6: - COM_state_23 + COM_state_27 - delta_z_7 = 0
 compute_delta_z_7: - COM_state_27 + COM_state_31 - delta_z_8 = 0
 compute_delta_z_8: - COM_state_31 + COM_state_35 - delta_z_9 = 0
 compute_delta_z_9: - COM_state_35 + COM_state_39 - delta_z_10 = 0
 compute_delta_z_10: - COM_state_39 + COM_state_43 - delta_z_11 = 0
 compute_delta_z_11: - COM_state_43 + COM_state_47 - delta_z_12 = 0
 compute_delta_z_12: - COM_state_47 + COM_state_51 - delta_z_13 = 0
 compute_delta_z_13: - COM_state_51 + COM_state_55 - delta_z_14 = 0
 compute_delta_z_14: - COM_state_55 + COM_state_59 - delta_z_15 = 0
 compute_delta_z_15: - COM_state_59 + COM_state_63 - delta_z_16 = 0
 compute_delta_z_16: - COM_state_63 + COM_state_67 - delta_z_17 = 0
 compute_delta_z_17: - COM_state_67 + COM_state_71 - delta_z_18 = 0
 compute_delta_z_18: - COM_state_71 + COM_state_75 - delta_z_19 = 0
 compute_delta_z_19: - COM_state_75 + COM_state_79 - delta_z_20 = 0
 compute_delta_z_20: - COM_state_79 + COM_state_83 - delta_z_21 = 0
 compute_sum_Flag_1: delta_z_flag_1 + delta_z_flag_2
   - Sum_Column_delta_z_Flag_1 = 0
 compute_sum_Flag_2: delta_z_flag_3 + delta_z_flag_4
   - Sum_Column_delta_z_Flag_2 = 0
 compute_sum_Flag_3: delta_z_flag_5 + delta_z_flag_6
   - Sum_Column_delta_z_Flag_3 = 0
 compute_sum_Flag_4: delta_z_flag_7 + delta_z_flag_8
   - Sum_Column_delta_z_Flag_4 = 0
 compute_sum_Flag_5: delta_z_flag_9 + delta_z_flag_10
   - Sum_Column_delta_z_Flag_5 = 0
 compute_sum_Flag_6: delta_z_flag_11 + delta_z_flag_12
   - Sum_Column_delta_z_Flag_6 = 0
 compute_sum_Flag_7: delta_z_flag_13 + delta_z_flag_14
   - Sum_Column_delta_z_Flag_7 = 0
 compute_sum_Flag_8: delta_z_flag_15 + delta_z_flag_16
   - Sum_Column_delta_z_Flag_8 = 0
 compute_sum_Flag_9: delta_z_flag_17 + delta_z_flag_18
   - Sum_Column_delta_z_Flag_9 = 0
 compute_sum_Flag_10: delta_z_flag_19 + delta_z_flag_20
   - Sum_Column_delta_z_Flag_10 = 0
 compute_sum_Flag_11: delta_z_flag_21 + delta_z_flag_22
   - Sum_Column_delta_z_Flag_11 = 0
 compute_sum_Flag_12: delta_z_flag_23 + delta_z_flag_24
   - Sum_Column_delta_z_Flag_12 = 0
 compute_sum_Flag_13: delta_z_flag_25 + delta_z_flag_26
   - Sum_Column_delta_z_Flag_13 = 0
 compute_sum_Flag_14: delta_z_flag_27 + delta_z_flag_28
   - Sum_Column_delta_z_Flag_14 = 0
 compute_sum_Flag_15: delta_z_flag_29 + delta_z_flag_30
   - Sum_Column_delta_z_Flag_15 = 0
 compute_sum_Flag_16: delta_z_flag_31 + delta_z_flag_32
   - Sum_Column_delta_z_Flag_16 = 0
 compute_sum_Flag_17: delta_z_flag_33 + delta_z_flag_34
   - Sum_Column_delta_z_Flag_17 = 0
 compute_sum_Flag_18: delta_z_flag_35 + delta_z_flag_36
   - Sum_Column_delta_z_Flag_18 = 0
 compute_sum_Flag_19: delta_z_flag_37 + delta_z_flag_38
   - Sum_Column_delta_z_Flag_19 = 0
 compute_sum_Flag_20: delta_z_flag_39 + delta_z_flag_40
   - Sum_Column_delta_z_Flag_20 = 0
 compute_sum_Flag_21: delta_z_flag_41 + delta_z_flag_42
   - Sum_Column_delta_z_Flag_21 = 0
 H_Physic_Fir_1: H_21 + H_22 + H_23 + H_24 + H_25 = 1
 H_Physic_Fir_2: H_26 + H_27 + H_28 + H_29 + H_30 = 1
 H_Physic_Fir_3: H_31 + H_32 + H_33 + H_34 + H_35 = 1
 H_Physic_Fir_4: H_36 + H_37 + H_38 + H_39 + H_40 = 1
 H_Physic_Fir_5: H_41 + H_42 + H_43 + H_44 + H_45 = 1
 H_Physic_Fir_6: H_46 + H_47 + H_48 + H_49 + H_50 = 1
 H_Physic_Fir_7: H_51 + H_52 + H_53 + H_54 + H_55 = 1
 H_Physic_Fir_8: H_56 + H_57 + H_58 + H_59 + H_60 = 1
 H_Physic_Fir_9: H_61 + H_62 + H_63 + H_64 + H_65 = 1
 H_Physic_Fir_10: H_66 + H_67 + H_68 + H_69 + H_70 = 1
 H_Physic_Fir_11: H_71 + H_72 + H_73 + H_74 + H_75 = 1
 H_Physic_Fir_12: H_76 + H_77 + H_78 + H_79 + H_80 = 1
 H_Physic_Fir_13: H_81 + H_82 + H_83 + H_84 + H_85 = 1
 H_Physic_Fir_14: H_86 + H_87 + H_88 + H_89 + H_90 = 1
 H_Physic_Fir_15: H_91 + H_92 + H_93 + H_94 + H_95 = 1
 H_Physic_Fir_16: H_96 + H_97 + H_98 + H_99 + H_100 = 1
 H_Physic_Fir_17: H_101 + H_102 + H_103 + H_104 + H_105 = 1
 H_Physic_Fir_18: H_106 + H_107 + H_108 + H_109 + H_110 = 1
 H_Physic_Fir_19: H_111 + H_112 + H_113 + H_114 + H_115 = 1
 H_Physic_Fir_20: H_116 + H_117 + H_118 + H_119 + H_120 = 1
 H_Physic_Fir_21: H_121 + H_122 + H_123 + H_124 + H_125 = 1
 H_Physic_Fir_22: H_126 + H_127 + H_128 + H_129 + H_130 = 1
 H_Physic_Fir_23: H_131 + H_132 + H_133 + H_134 + H_135 = 1
 H_Physic_Fir_24: H_136 + H_137 + H_138 + H_139 + H_140 = 1
 H_Physic_Fir_25: H_141 + H_142 + H_143 + H_144 + H_145 = 1
 H_Physic_Fir_26: H_146 + H_147 + H_148 + H_149 + H_150 = 1
 H_Physic_Fir_27: H_151 + H_152 + H_153 + H_154 + H_155 = 1
 H_Physic_Fir_28: H_156 + H_157 + H_158 + H_159 + H_160 = 1
 H_Physic_Fir_29: H_161 + H_162 + H_163 + H_164 + H_165 = 1
 H_Physic_Fir_30: H_166 + H_167 + H_168 + H_169 + H_170 = 1
 H_Physic_Fir_31: H_171 + H_172 + H_173 + H_174 + H_175 = 1
 H_Physic_Fir_32: H_176 + H_177 + H_178 + H_179 + H_180 = 1
 H_Physic_Fir_33: H_181 + H_182 + H_183 + H_184 + H_185 = 1
 H_Physic_Fir_34: H_186 + H_187 + H_188 + H_189 + H_190 = 1
 H_Physic_Fir_35: H_191 + H_192 + H_193 + H_194 + H_195 = 1
 H_Physic_Fir_36: H_196 + H_197 + H_198 + H_199 + H_200 = 1
 H_Physic_Fir_37: H_201 + H_202 + H_203 + H_204 + H_205 = 1
 H_Physic_Fir_38: H_206 + H_207 + H_208 + H_209 + H_210 = 1
 H_Physic_Fir_39: H_211 + H_212 + H_213 + H_214 + H_215 = 1
 H_Physic_Fir_40: H_216 + H_217 + H_218 + H_219 + H_220 = 1
 H_Physic_Fir_41: H_221 + H_222 + H_223 + H_224 + H_225 = 1
 H_Physic_Fir_42: H_226 + H_227 + H_228 + H_229 + H_230 = 1
 H_Physic_Fir_43: H_231 + H_232 + H_233 + H_234 + H_235 = 1
 H_Physic_Fir_44: H_236 + H_237 + H_238 + H_239 + H_240 = 1
 H_Physic_Fir_45: H_241 + H_242 + H_243 + H_244 + H_245 = 1
 H_Physic_Fir_46: H_246 + H_247 + H_248 + H_249 + H_250 = 1
 H_Physic_Fir_47: H_251 + H_252 + H_253 + H_254 + H_255 = 1
 H_Physic_Fir_48: H_256 + H_257 + H_258 + H_259 + H_260 = 1
 H_Physic_Fir_49: H_261 + H_262 + H_263 + H_264 + H_265 = 1
 H_Physic_Fir_50: H_266 + H_267 + H_268 + H_269 + H_270 = 1
 H_Physic_Fir_51: H_271 + H_272 + H_273 + H_274 + H_275 = 1
 H_Physic_Fir_52: H_276 + H_277 + H_278 + H_279 + H_280 = 1
 H_Physic_Fir_53: H_281 + H_282 + H_283 + H_284 + H_285 = 1
 H_Physic_Fir_54: H_286 + H_287 + H_288 + H_289 + H_290 = 1
 H_Physic_Fir_55: H_291 + H_292 + H_293 + H_294 + H_295 = 1
 H_Physic_Fir_56: H_296 + H_297 + H_298 + H_299 + H_300 = 1
 H_Physic_Fir_57: H_301 + H_302 + H_303 + H_304 + H_305 = 1
 H_Physic_Fir_58: H_306 + H_307 + H_308 + H_309 + H_310 = 1
 H_Physic_Fir_59: H_311 + H_312 + H_313 + H_314 + H_315 = 1
 H_Physic_Fir_60: H_316 + H_317 + H_318 + H_319 + H_320 = 1
 H_Physic_Fir_61: H_321 + H_322 + H_323 + H_324 + H_325 = 1
 H_Physic_Fir_62: H_326 + H_327 + H_328 + H_329 + H_330 = 1
 H_Physic_Fir_63: H_331 + H_332 + H_333 + H_334 + H_335 = 1
 H_Physic_Fir_64: H_336 + H_337 + H_338 + H_339 + H_340 = 1
 H_Physic_Fir_65: H_341 + H_342 + H_343 + H_344 + H_345 = 1
 H_Physic_Fir_66: H_346 + H_347 + H_348 + H_349 + H_350 = 1
 H_Physic_Fir_67: H_351 + H_352 + H_353 + H_354 + H_355 = 1
 H_Physic_Fir_68: H_356 + H_357 + H_358 + H_359 + H_360 = 1
 H_Physic_Fir_69: H_361 + H_362 + H_363 + H_364 + H_365 = 1
 H_Physic_Fir_70: H_366 + H_367 + H_368 + H_369 + H_370 = 1
 H_Physic_Fir_71: H_371 + H_372 + H_373 + H_374 + H_375 = 1
 H_Physic_Fir_72: H_376 + H_377 + H_378 + H_379 + H_380 = 1
 H_Physic_Fir_73: H_381 + H_382 + H_383 + H_384 + H_385 = 1
 H_Physic_Fir_74: H_386 + H_387 + H_388 + H_389 + H_390 = 1
 H_Physic_Fir_75: H_391 + H_392 + H_393 + H_394 + H_395 = 1
 H_Physic_Fir_76: H_396 + H_397 + H_398 + H_399 + H_400 = 1
 H_Physic_Fir_77: H_401 + H_402 + H_403 + H_404 + H_405 = 1
 H_Physic_Fir_78: H_406 + H_407 + H_408 + H_409 + H_410 = 1
 H_Physic_Fir_79: H_411 + H_412 + H_413 + H_414 + H_415 = 1
 H_Physic_Fir_80: H_416 + H_417 + H_418 + H_419 + H_420 = 1
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
 -10 <= COM_state_53 <= 10
 -10 <= COM_state_54 <= 10
 -10 <= COM_state_55 <= 10
 -10 <= COM_state_56 <= 10
 -10 <= COM_state_57 <= 10
 -10 <= COM_state_58 <= 10
 -10 <= COM_state_59 <= 10
 -10 <= COM_state_60 <= 10
 -10 <= COM_state_61 <= 10
 -10 <= COM_state_62 <= 10
 -10 <= COM_state_63 <= 10
 -10 <= COM_state_64 <= 10
 -10 <= COM_state_65 <= 10
 -10 <= COM_state_66 <= 10
 -10 <= COM_state_67 <= 10
 -10 <= COM_state_68 <= 10
 -10 <= COM_state_69 <= 10
 -10 <= COM_state_70 <= 10
 -10 <= COM_state_71 <= 10
 -10 <= COM_state_72 <= 10
 -10 <= COM_state_73 <= 10
 -10 <= COM_state_74 <= 10
 -10 <= COM_state_75 <= 10
 -10 <= COM_state_76 <= 10
 -10 <= COM_state_77 <= 10
 -10 <= COM_state_78 <= 10
 -10 <= COM_state_79 <= 10
 -10 <= COM_state_80 <= 10
 -10 <= COM_state_81 <= 10
 -10 <= COM_state_82 <= 10
 -10 <= COM_state_83 <= 10
 -10 <= COM_state_84 <= 10
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
 -20 <= Leg_state_209 <= 20
 -20 <= Leg_state_210 <= 20
 -20 <= Leg_state_211 <= 20
 Leg_state_212 = 0
 -20 <= Leg_state_213 <= 20
 -20 <= Leg_state_214 <= 20
 -20 <= Leg_state_215 <= 20
 Leg_state_216 = 0
 -20 <= Leg_state_217 <= 20
 -20 <= Leg_state_218 <= 20
 -20 <= Leg_state_219 <= 20
 Leg_state_220 = 0
 -20 <= Leg_state_221 <= 20
 -20 <= Leg_state_222 <= 20
 -20 <= Leg_state_223 <= 20
 Leg_state_224 = 0
 -20 <= Leg_state_225 <= 20
 -20 <= Leg_state_226 <= 20
 -20 <= Leg_state_227 <= 20
 Leg_state_228 = 0
 -20 <= Leg_state_229 <= 20
 -20 <= Leg_state_230 <= 20
 -20 <= Leg_state_231 <= 20
 Leg_state_232 = 0
 -20 <= Leg_state_233 <= 20
 -20 <= Leg_state_234 <= 20
 -20 <= Leg_state_235 <= 20
 Leg_state_236 = 0
 -20 <= Leg_state_237 <= 20
 -20 <= Leg_state_238 <= 20
 -20 <= Leg_state_239 <= 20
 Leg_state_240 = 0
 -20 <= Leg_state_241 <= 20
 -20 <= Leg_state_242 <= 20
 -20 <= Leg_state_243 <= 20
 Leg_state_244 = 0
 -20 <= Leg_state_245 <= 20
 -20 <= Leg_state_246 <= 20
 -20 <= Leg_state_247 <= 20
 Leg_state_248 = 0
 -20 <= Leg_state_249 <= 20
 -20 <= Leg_state_250 <= 20
 -20 <= Leg_state_251 <= 20
 Leg_state_252 = 0
 -20 <= Leg_state_253 <= 20
 -20 <= Leg_state_254 <= 20
 -20 <= Leg_state_255 <= 20
 Leg_state_256 = 0
 -20 <= Leg_state_257 <= 20
 -20 <= Leg_state_258 <= 20
 -20 <= Leg_state_259 <= 20
 Leg_state_260 = 0
 -20 <= Leg_state_261 <= 20
 -20 <= Leg_state_262 <= 20
 -20 <= Leg_state_263 <= 20
 Leg_state_264 = 0
 -20 <= Leg_state_265 <= 20
 -20 <= Leg_state_266 <= 20
 -20 <= Leg_state_267 <= 20
 Leg_state_268 = 0
 -20 <= Leg_state_269 <= 20
 -20 <= Leg_state_270 <= 20
 -20 <= Leg_state_271 <= 20
 Leg_state_272 = 0
 -20 <= Leg_state_273 <= 20
 -20 <= Leg_state_274 <= 20
 -20 <= Leg_state_275 <= 20
 Leg_state_276 = 0
 -20 <= Leg_state_277 <= 20
 -20 <= Leg_state_278 <= 20
 -20 <= Leg_state_279 <= 20
 Leg_state_280 = 0
 -20 <= Leg_state_281 <= 20
 -20 <= Leg_state_282 <= 20
 -20 <= Leg_state_283 <= 20
 Leg_state_284 = 0
 -20 <= Leg_state_285 <= 20
 -20 <= Leg_state_286 <= 20
 -20 <= Leg_state_287 <= 20
 Leg_state_288 = 0
 -20 <= Leg_state_289 <= 20
 -20 <= Leg_state_290 <= 20
 -20 <= Leg_state_291 <= 20
 Leg_state_292 = 0
 -20 <= Leg_state_293 <= 20
 -20 <= Leg_state_294 <= 20
 -20 <= Leg_state_295 <= 20
 Leg_state_296 = 0
 -20 <= Leg_state_297 <= 20
 -20 <= Leg_state_298 <= 20
 -20 <= Leg_state_299 <= 20
 Leg_state_300 = 0
 -20 <= Leg_state_301 <= 20
 -20 <= Leg_state_302 <= 20
 -20 <= Leg_state_303 <= 20
 Leg_state_304 = 0
 -20 <= Leg_state_305 <= 20
 -20 <= Leg_state_306 <= 20
 -20 <= Leg_state_307 <= 20
 Leg_state_308 = 0
 -20 <= Leg_state_309 <= 20
 -20 <= Leg_state_310 <= 20
 -20 <= Leg_state_311 <= 20
 Leg_state_312 = 0
 -20 <= Leg_state_313 <= 20
 -20 <= Leg_state_314 <= 20
 -20 <= Leg_state_315 <= 20
 Leg_state_316 = 0
 -20 <= Leg_state_317 <= 20
 -20 <= Leg_state_318 <= 20
 -20 <= Leg_state_319 <= 20
 Leg_state_320 = 0
 -20 <= Leg_state_321 <= 20
 -20 <= Leg_state_322 <= 20
 -20 <= Leg_state_323 <= 20
 Leg_state_324 = 0
 -20 <= Leg_state_325 <= 20
 -20 <= Leg_state_326 <= 20
 -20 <= Leg_state_327 <= 20
 Leg_state_328 = 0
 -20 <= Leg_state_329 <= 20
 -20 <= Leg_state_330 <= 20
 -20 <= Leg_state_331 <= 20
 Leg_state_332 = 0
 -20 <= Leg_state_333 <= 20
 -20 <= Leg_state_334 <= 20
 -20 <= Leg_state_335 <= 20
 Leg_state_336 = 0
 delta_z_1 = 0
 -10 <= delta_z_2 <= 10
 -10 <= delta_z_3 <= 10
 -10 <= delta_z_4 <= 10
 -10 <= delta_z_5 <= 10
 -10 <= delta_z_6 <= 10
 -10 <= delta_z_7 <= 10
 -10 <= delta_z_8 <= 10
 -10 <= delta_z_9 <= 10
 -10 <= delta_z_10 <= 10
 -10 <= delta_z_11 <= 10
 -10 <= delta_z_12 <= 10
 -10 <= delta_z_13 <= 10
 -10 <= delta_z_14 <= 10
 -10 <= delta_z_15 <= 10
 -10 <= delta_z_16 <= 10
 -10 <= delta_z_17 <= 10
 -10 <= delta_z_18 <= 10
 -10 <= delta_z_19 <= 10
 -10 <= delta_z_20 <= 10
 -10 <= delta_z_21 <= 10
 delta_z_flag_1 = 0
 delta_z_flag_2 = 0
 Trim_1 = 0
 H_1 = 0
 H_2 = 0
 H_3 = 0
 H_4 = 0
 H_5 = 0
 H_6 = 0
 H_7 = 0
 H_8 = 0
 H_9 = 0
 H_10 = 0
 H_11 = 0
 H_12 = 0
 H_13 = 0
 H_14 = 0
 H_15 = 0
 H_16 = 0
 H_17 = 0
 H_18 = 0
 H_19 = 0
 H_20 = 0
 Constant = 1
Binaries
 T_1 T_2 T_3 T_4 T_5 T_6 T_7 T_8 T_9 T_10 T_11 T_12 T_13 T_14 T_15 T_16
 T_17 T_18 T_19 T_20 T_21 T_22 T_23 T_24 T_25 T_26 T_27 T_28 T_29 T_30 T_31
 T_32 T_33 T_34 T_35 T_36 T_37 T_38 T_39 T_40 T_41 T_42 T_43 T_44 T_45 T_46
 T_47 T_48 T_49 T_50 T_51 T_52 T_53 T_54 T_55 T_56 T_57 T_58 T_59 T_60 T_61
 T_62 T_63 T_64 T_65 T_66 T_67 T_68 T_69 T_70 T_71 T_72 T_73 T_74 T_75 T_76
 T_77 T_78 T_79 T_80 T_81 T_82 T_83 T_84 delta_z_flag_1 delta_z_flag_2
 delta_z_flag_3 delta_z_flag_4 delta_z_flag_5 delta_z_flag_6 delta_z_flag_7
 delta_z_flag_8 delta_z_flag_9 delta_z_flag_10 delta_z_flag_11
 delta_z_flag_12 delta_z_flag_13 delta_z_flag_14 delta_z_flag_15
 delta_z_flag_16 delta_z_flag_17 delta_z_flag_18 delta_z_flag_19
 delta_z_flag_20 delta_z_flag_21 delta_z_flag_22 delta_z_flag_23
 delta_z_flag_24 delta_z_flag_25 delta_z_flag_26 delta_z_flag_27
 delta_z_flag_28 delta_z_flag_29 delta_z_flag_30 delta_z_flag_31
 delta_z_flag_32 delta_z_flag_33 delta_z_flag_34 delta_z_flag_35
 delta_z_flag_36 delta_z_flag_37 delta_z_flag_38 delta_z_flag_39
 delta_z_flag_40 delta_z_flag_41 delta_z_flag_42 Sum_Column_delta_z_Flag_1
 Sum_Column_delta_z_Flag_2 Sum_Column_delta_z_Flag_3
 Sum_Column_delta_z_Flag_4 Sum_Column_delta_z_Flag_5
 Sum_Column_delta_z_Flag_6 Sum_Column_delta_z_Flag_7
 Sum_Column_delta_z_Flag_8 Sum_Column_delta_z_Flag_9
 Sum_Column_delta_z_Flag_10 Sum_Column_delta_z_Flag_11
 Sum_Column_delta_z_Flag_12 Sum_Column_delta_z_Flag_13
 Sum_Column_delta_z_Flag_14 Sum_Column_delta_z_Flag_15
 Sum_Column_delta_z_Flag_16 Sum_Column_delta_z_Flag_17
 Sum_Column_delta_z_Flag_18 Sum_Column_delta_z_Flag_19
 Sum_Column_delta_z_Flag_20 Sum_Column_delta_z_Flag_21 Trim_1 Trim_2 Trim_3
 Trim_4 Trim_5 Trim_6 Trim_7 Trim_8 Trim_9 Trim_10 Trim_11 Trim_12 Trim_13
 Trim_14 Trim_15 Trim_16 Trim_17 Trim_18 Trim_19 Trim_20 Trim_21 H_1 H_2
 H_3 H_4 H_5 H_6 H_7 H_8 H_9 H_10 H_11 H_12 H_13 H_14 H_15 H_16 H_17 H_18
 H_19 H_20 H_21 H_22 H_23 H_24 H_25 H_26 H_27 H_28 H_29 H_30 H_31 H_32 H_33
 H_34 H_35 H_36 H_37 H_38 H_39 H_40 H_41 H_42 H_43 H_44 H_45 H_46 H_47 H_48
 H_49 H_50 H_51 H_52 H_53 H_54 H_55 H_56 H_57 H_58 H_59 H_60 H_61 H_62 H_63
 H_64 H_65 H_66 H_67 H_68 H_69 H_70 H_71 H_72 H_73 H_74 H_75 H_76 H_77 H_78
 H_79 H_80 H_81 H_82 H_83 H_84 H_85 H_86 H_87 H_88 H_89 H_90 H_91 H_92 H_93
 H_94 H_95 H_96 H_97 H_98 H_99 H_100 H_101 H_102 H_103 H_104 H_105 H_106
 H_107 H_108 H_109 H_110 H_111 H_112 H_113 H_114 H_115 H_116 H_117 H_118
 H_119 H_120 H_121 H_122 H_123 H_124 H_125 H_126 H_127 H_128 H_129 H_130
 H_131 H_132 H_133 H_134 H_135 H_136 H_137 H_138 H_139 H_140 H_141 H_142
 H_143 H_144 H_145 H_146 H_147 H_148 H_149 H_150 H_151 H_152 H_153 H_154
 H_155 H_156 H_157 H_158 H_159 H_160 H_161 H_162 H_163 H_164 H_165 H_166
 H_167 H_168 H_169 H_170 H_171 H_172 H_173 H_174 H_175 H_176 H_177 H_178
 H_179 H_180 H_181 H_182 H_183 H_184 H_185 H_186 H_187 H_188 H_189 H_190
 H_191 H_192 H_193 H_194 H_195 H_196 H_197 H_198 H_199 H_200 H_201 H_202
 H_203 H_204 H_205 H_206 H_207 H_208 H_209 H_210 H_211 H_212 H_213 H_214
 H_215 H_216 H_217 H_218 H_219 H_220 H_221 H_222 H_223 H_224 H_225 H_226
 H_227 H_228 H_229 H_230 H_231 H_232 H_233 H_234 H_235 H_236 H_237 H_238
 H_239 H_240 H_241 H_242 H_243 H_244 H_245 H_246 H_247 H_248 H_249 H_250
 H_251 H_252 H_253 H_254 H_255 H_256 H_257 H_258 H_259 H_260 H_261 H_262
 H_263 H_264 H_265 H_266 H_267 H_268 H_269 H_270 H_271 H_272 H_273 H_274
 H_275 H_276 H_277 H_278 H_279 H_280 H_281 H_282 H_283 H_284 H_285 H_286
 H_287 H_288 H_289 H_290 H_291 H_292 H_293 H_294 H_295 H_296 H_297 H_298
 H_299 H_300 H_301 H_302 H_303 H_304 H_305 H_306 H_307 H_308 H_309 H_310
 H_311 H_312 H_313 H_314 H_315 H_316 H_317 H_318 H_319 H_320 H_321 H_322
 H_323 H_324 H_325 H_326 H_327 H_328 H_329 H_330 H_331 H_332 H_333 H_334
 H_335 H_336 H_337 H_338 H_339 H_340 H_341 H_342 H_343 H_344 H_345 H_346
 H_347 H_348 H_349 H_350 H_351 H_352 H_353 H_354 H_355 H_356 H_357 H_358
 H_359 H_360 H_361 H_362 H_363 H_364 H_365 H_366 H_367 H_368 H_369 H_370
 H_371 H_372 H_373 H_374 H_375 H_376 H_377 H_378 H_379 H_380 H_381 H_382
 H_383 H_384 H_385 H_386 H_387 H_388 H_389 H_390 H_391 H_392 H_393 H_394
 H_395 H_396 H_397 H_398 H_399 H_400 H_401 H_402 H_403 H_404 H_405 H_406
 H_407 H_408 H_409 H_410 H_411 H_412 H_413 H_414 H_415 H_416 H_417 H_418
 H_419 H_420
End
