function cl = clfunc(in1,in2,in3,T)
%CLFUNC
%    CL = CLFUNC(IN1,IN2,IN3,T)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    26-Nov-2024 09:09:09

Q10_1 = in1(10);
Q10_2 = in1(21);
Q10_3 = in1(32);
Q10_4 = in1(43);
Q10_5 = in1(54);
Q10_6 = in1(65);
Q10_7 = in1(76);
Q10_8 = in1(87);
Q10_9 = in1(98);
Q11_1 = in1(11);
Q11_2 = in1(22);
Q11_3 = in1(33);
Q11_4 = in1(44);
Q11_5 = in1(55);
Q11_6 = in1(66);
Q11_7 = in1(77);
Q11_8 = in1(88);
Q11_9 = in1(99);
Q10_10 = in1(109);
Q10_11 = in1(120);
Q11_10 = in1(110);
Q11_11 = in1(121);
Q1_1 = in1(1);
Q1_2 = in1(12);
Q1_3 = in1(23);
Q1_4 = in1(34);
Q1_5 = in1(45);
Q1_6 = in1(56);
Q1_7 = in1(67);
Q1_8 = in1(78);
Q1_9 = in1(89);
Q2_1 = in1(2);
Q2_2 = in1(13);
Q2_3 = in1(24);
Q2_4 = in1(35);
Q2_5 = in1(46);
Q2_6 = in1(57);
Q2_7 = in1(68);
Q2_8 = in1(79);
Q2_9 = in1(90);
Q3_1 = in1(3);
Q3_2 = in1(14);
Q3_3 = in1(25);
Q3_4 = in1(36);
Q3_5 = in1(47);
Q3_6 = in1(58);
Q3_7 = in1(69);
Q3_8 = in1(80);
Q3_9 = in1(91);
Q4_1 = in1(4);
Q4_2 = in1(15);
Q4_3 = in1(26);
Q4_4 = in1(37);
Q4_5 = in1(48);
Q4_6 = in1(59);
Q4_7 = in1(70);
Q4_8 = in1(81);
Q4_9 = in1(92);
Q5_1 = in1(5);
Q5_2 = in1(16);
Q5_3 = in1(27);
Q5_4 = in1(38);
Q5_5 = in1(49);
Q5_6 = in1(60);
Q5_7 = in1(71);
Q5_8 = in1(82);
Q5_9 = in1(93);
Q6_1 = in1(6);
Q6_2 = in1(17);
Q6_3 = in1(28);
Q6_4 = in1(39);
Q6_5 = in1(50);
Q6_6 = in1(61);
Q6_7 = in1(72);
Q6_8 = in1(83);
Q6_9 = in1(94);
Q7_1 = in1(7);
Q7_2 = in1(18);
Q7_3 = in1(29);
Q7_4 = in1(40);
Q7_5 = in1(51);
Q7_6 = in1(62);
Q7_7 = in1(73);
Q7_8 = in1(84);
Q7_9 = in1(95);
Q8_1 = in1(8);
Q8_2 = in1(19);
Q8_3 = in1(30);
Q8_4 = in1(41);
Q8_5 = in1(52);
Q8_6 = in1(63);
Q8_7 = in1(74);
Q8_8 = in1(85);
Q8_9 = in1(96);
Q9_1 = in1(9);
Q9_2 = in1(20);
Q9_3 = in1(31);
Q9_4 = in1(42);
Q9_5 = in1(53);
Q9_6 = in1(64);
Q9_7 = in1(75);
Q9_8 = in1(86);
Q9_9 = in1(97);
Q1_10 = in1(100);
Q1_11 = in1(111);
Q2_10 = in1(101);
Q2_11 = in1(112);
Q3_10 = in1(102);
Q3_11 = in1(113);
Q4_10 = in1(103);
Q4_11 = in1(114);
Q5_10 = in1(104);
Q5_11 = in1(115);
Q6_10 = in1(105);
Q6_11 = in1(116);
Q7_10 = in1(106);
Q7_11 = in1(117);
Q8_10 = in1(107);
Q8_11 = in1(118);
Q9_10 = in1(108);
Q9_11 = in1(119);
qdes1 = in3(1,:);
qdes2 = in3(2,:);
qdes3 = in3(3,:);
qdes4 = in3(4,:);
qdes5 = in3(5,:);
qdes6 = in3(6,:);
qdes7 = in3(7,:);
qdes8 = in3(8,:);
qdes9 = in3(9,:);
qdes10 = in3(10,:);
qdes11 = in3(11,:);
t2 = Q10_1.*qdes1;
t3 = Q10_2.*qdes2;
t4 = Q10_3.*qdes3;
t5 = Q10_4.*qdes4;
t6 = Q10_5.*qdes5;
t7 = Q10_1.*qdes10;
t8 = Q10_2.*qdes10;
t9 = Q10_6.*qdes6;
t10 = Q11_1.*qdes1;
t11 = Q10_3.*qdes10;
t12 = Q10_4.*qdes10;
t13 = Q10_7.*qdes7;
t14 = Q11_2.*qdes2;
t15 = Q10_5.*qdes10;
t16 = Q10_6.*qdes10;
t17 = Q10_8.*qdes8;
t18 = Q11_3.*qdes3;
t19 = Q10_7.*qdes10;
t20 = Q10_8.*qdes10;
t21 = Q10_9.*qdes9;
t22 = Q11_4.*qdes4;
t23 = Q10_9.*qdes10;
t24 = Q11_5.*qdes5;
t25 = Q11_1.*qdes11;
t26 = Q11_6.*qdes6;
t27 = Q11_2.*qdes11;
t28 = Q11_3.*qdes11;
t29 = Q11_7.*qdes7;
t30 = Q11_4.*qdes11;
t31 = Q11_5.*qdes11;
t32 = Q11_8.*qdes8;
t33 = Q11_6.*qdes11;
t34 = Q11_7.*qdes11;
t35 = Q11_9.*qdes9;
t36 = Q11_8.*qdes11;
t37 = Q11_9.*qdes11;
t38 = Q10_10.*qdes10;
t39 = Q10_11.*qdes10;
t40 = Q10_11.*qdes11;
t41 = Q11_10.*qdes10;
t42 = Q11_10.*qdes11;
t43 = Q11_11.*qdes11;
t44 = Q1_1.*qdes1;
t45 = Q1_2.*qdes1;
t46 = Q1_2.*qdes2;
t47 = Q1_3.*qdes1;
t48 = Q1_4.*qdes1;
t49 = Q1_3.*qdes3;
t50 = Q1_5.*qdes1;
t51 = Q1_6.*qdes1;
t52 = Q1_4.*qdes4;
t53 = Q1_7.*qdes1;
t54 = Q1_8.*qdes1;
t55 = Q1_5.*qdes5;
t56 = Q1_9.*qdes1;
t57 = Q1_6.*qdes6;
t58 = Q2_1.*qdes1;
t59 = Q2_1.*qdes2;
t60 = Q1_7.*qdes7;
t61 = Q2_2.*qdes2;
t62 = Q2_3.*qdes2;
t63 = Q1_8.*qdes8;
t64 = Q2_3.*qdes3;
t65 = Q2_4.*qdes2;
t66 = Q2_5.*qdes2;
t67 = Q1_9.*qdes9;
t68 = Q2_4.*qdes4;
t69 = Q2_6.*qdes2;
t70 = Q2_7.*qdes2;
t71 = Q2_5.*qdes5;
t72 = Q2_8.*qdes2;
t73 = Q2_9.*qdes2;
t74 = Q2_6.*qdes6;
t75 = Q3_1.*qdes1;
t76 = Q2_7.*qdes7;
t77 = Q3_1.*qdes3;
t78 = Q3_2.*qdes2;
t79 = Q3_2.*qdes3;
t80 = Q2_8.*qdes8;
t81 = Q3_3.*qdes3;
t82 = Q3_4.*qdes3;
t83 = Q2_9.*qdes9;
t84 = Q3_4.*qdes4;
t85 = Q3_5.*qdes3;
t86 = Q3_6.*qdes3;
t87 = Q3_5.*qdes5;
t88 = Q3_7.*qdes3;
t89 = Q3_8.*qdes3;
t90 = Q3_6.*qdes6;
t91 = Q3_9.*qdes3;
t92 = Q4_1.*qdes1;
t93 = Q3_7.*qdes7;
t94 = Q4_2.*qdes2;
t95 = Q4_1.*qdes4;
t96 = Q3_8.*qdes8;
t97 = Q4_2.*qdes4;
t98 = Q4_3.*qdes3;
t99 = Q4_3.*qdes4;
t100 = Q3_9.*qdes9;
t101 = Q4_4.*qdes4;
t102 = Q4_5.*qdes4;
t103 = Q4_5.*qdes5;
t104 = Q4_6.*qdes4;
t105 = Q4_7.*qdes4;
t106 = Q4_6.*qdes6;
t107 = Q4_8.*qdes4;
t108 = Q5_1.*qdes1;
t109 = Q4_9.*qdes4;
t110 = Q4_7.*qdes7;
t111 = Q5_2.*qdes2;
t112 = Q4_8.*qdes8;
t113 = Q5_1.*qdes5;
t114 = Q5_3.*qdes3;
t115 = Q5_2.*qdes5;
t116 = Q4_9.*qdes9;
t117 = Q5_3.*qdes5;
t118 = Q5_4.*qdes4;
t119 = Q5_4.*qdes5;
t120 = Q5_5.*qdes5;
t121 = Q5_6.*qdes5;
t122 = Q5_6.*qdes6;
t123 = Q5_7.*qdes5;
t124 = Q6_1.*qdes1;
t125 = Q5_8.*qdes5;
t126 = Q5_7.*qdes7;
t127 = Q5_9.*qdes5;
t128 = Q6_2.*qdes2;
t129 = Q5_8.*qdes8;
t130 = Q6_3.*qdes3;
t131 = Q6_1.*qdes6;
t132 = Q5_9.*qdes9;
t133 = Q6_2.*qdes6;
t134 = Q6_4.*qdes4;
t135 = Q6_3.*qdes6;
t136 = Q6_4.*qdes6;
t137 = Q6_5.*qdes5;
t138 = Q6_5.*qdes6;
t139 = Q6_6.*qdes6;
t140 = Q7_1.*qdes1;
t141 = Q6_7.*qdes6;
t142 = Q6_7.*qdes7;
t143 = Q6_8.*qdes6;
t144 = Q7_2.*qdes2;
t145 = Q6_9.*qdes6;
t146 = Q6_8.*qdes8;
t147 = Q7_3.*qdes3;
t148 = Q6_9.*qdes9;
t149 = Q7_1.*qdes7;
t150 = Q7_4.*qdes4;
t151 = Q7_2.*qdes7;
t152 = Q7_3.*qdes7;
t153 = Q7_5.*qdes5;
t154 = Q7_4.*qdes7;
t155 = Q7_5.*qdes7;
t156 = Q7_6.*qdes6;
t157 = Q8_1.*qdes1;
t158 = Q7_6.*qdes7;
t159 = Q7_7.*qdes7;
t160 = Q8_2.*qdes2;
t161 = Q7_8.*qdes7;
t162 = Q7_8.*qdes8;
t163 = Q7_9.*qdes7;
t164 = Q8_3.*qdes3;
t165 = Q7_9.*qdes9;
t166 = Q8_4.*qdes4;
t167 = Q8_1.*qdes8;
t168 = Q8_2.*qdes8;
t169 = Q8_5.*qdes5;
t170 = Q8_3.*qdes8;
t171 = Q8_4.*qdes8;
t172 = Q8_6.*qdes6;
t173 = Q9_1.*qdes1;
t174 = Q8_5.*qdes8;
t175 = Q8_6.*qdes8;
t176 = Q8_7.*qdes7;
t177 = Q9_2.*qdes2;
t178 = Q8_7.*qdes8;
t179 = Q8_8.*qdes8;
t180 = Q9_3.*qdes3;
t181 = Q8_9.*qdes8;
t182 = Q8_9.*qdes9;
t183 = Q9_4.*qdes4;
t184 = Q9_1.*qdes9;
t185 = Q9_5.*qdes5;
t186 = Q9_2.*qdes9;
t187 = Q9_3.*qdes9;
t188 = Q9_6.*qdes6;
t189 = Q9_4.*qdes9;
t190 = Q9_5.*qdes9;
t191 = Q9_7.*qdes7;
t192 = Q9_6.*qdes9;
t193 = Q9_7.*qdes9;
t194 = Q9_8.*qdes8;
t195 = Q9_8.*qdes9;
t196 = Q9_9.*qdes9;
t197 = Q1_10.*qdes1;
t198 = Q1_11.*qdes1;
t199 = Q1_10.*qdes10;
t200 = Q1_11.*qdes11;
t201 = Q2_10.*qdes2;
t202 = Q2_11.*qdes2;
t203 = Q2_10.*qdes10;
t204 = Q2_11.*qdes11;
t205 = Q3_10.*qdes3;
t206 = Q3_11.*qdes3;
t207 = Q3_10.*qdes10;
t208 = Q3_11.*qdes11;
t209 = Q4_10.*qdes4;
t210 = Q4_11.*qdes4;
t211 = Q4_10.*qdes10;
t212 = Q4_11.*qdes11;
t213 = Q5_10.*qdes5;
t214 = Q5_11.*qdes5;
t215 = Q5_10.*qdes10;
t216 = Q5_11.*qdes11;
t217 = Q6_10.*qdes6;
t218 = Q6_11.*qdes6;
t219 = Q6_10.*qdes10;
t220 = Q6_11.*qdes11;
t221 = Q7_10.*qdes7;
t222 = Q7_11.*qdes7;
t223 = Q7_10.*qdes10;
t224 = Q7_11.*qdes11;
t225 = Q8_10.*qdes8;
t226 = Q8_11.*qdes8;
t227 = Q8_10.*qdes10;
t228 = Q8_11.*qdes11;
t229 = Q9_10.*qdes9;
t230 = Q9_10.*qdes10;
t231 = Q9_11.*qdes9;
t232 = Q9_11.*qdes11;
t233 = t38.*2.0;
t234 = t43.*2.0;
t235 = t44.*2.0;
t236 = t61.*2.0;
t237 = t81.*2.0;
t238 = t101.*2.0;
t239 = t120.*2.0;
t240 = t139.*2.0;
t241 = t159.*2.0;
t242 = t179.*2.0;
t243 = t196.*2.0;
t244 = t2+t3+t4+t5+t6+t9+t13+t17+t21+t38+t40;
t245 = t10+t14+t18+t22+t24+t26+t29+t32+t35+t41+t43;
t246 = t7+t25+t44+t59+t77+t95+t113+t131+t149+t167+t184;
t247 = t8+t27+t45+t61+t79+t97+t115+t133+t151+t168+t186;
t248 = t11+t28+t47+t62+t81+t99+t117+t135+t152+t170+t187;
t249 = t12+t30+t48+t65+t82+t101+t119+t136+t154+t171+t189;
t250 = t15+t31+t50+t66+t85+t102+t120+t138+t155+t174+t190;
t251 = t16+t33+t51+t69+t86+t104+t121+t139+t158+t175+t192;
t252 = t19+t34+t53+t70+t88+t105+t123+t141+t159+t178+t193;
t253 = t20+t36+t54+t72+t89+t107+t125+t143+t161+t179+t195;
t254 = t23+t37+t56+t73+t91+t109+t127+t145+t163+t181+t196;
t255 = t38+t42+t197+t201+t205+t209+t213+t217+t221+t225+t229;
t256 = t39+t43+t198+t202+t206+t210+t214+t218+t222+t226+t231;
t257 = t44+t46+t49+t52+t55+t57+t60+t63+t67+t199+t200;
t258 = t58+t61+t64+t68+t71+t74+t76+t80+t83+t203+t204;
t259 = t75+t78+t81+t84+t87+t90+t93+t96+t100+t207+t208;
t260 = t92+t94+t98+t101+t103+t106+t110+t112+t116+t211+t212;
t261 = t108+t111+t114+t118+t120+t122+t126+t129+t132+t215+t216;
t262 = t124+t128+t130+t134+t137+t139+t142+t146+t148+t219+t220;
t263 = t140+t144+t147+t150+t153+t156+t159+t162+t165+t223+t224;
t264 = t157+t160+t164+t166+t169+t172+t176+t179+t182+t227+t228;
t265 = t173+t177+t180+t183+t185+t188+t191+t194+t196+t230+t232;
t266 = T.*t244.*(1.9e+1./2.0e+1);
t267 = T.*t245.*(1.9e+1./2.0e+1);
t268 = T.*t246.*(1.9e+1./2.0e+1);
t269 = T.*t247.*(1.9e+1./2.0e+1);
t270 = T.*t248.*(1.9e+1./2.0e+1);
t271 = T.*t249.*(1.9e+1./2.0e+1);
t272 = T.*t250.*(1.9e+1./2.0e+1);
t273 = T.*t251.*(1.9e+1./2.0e+1);
t274 = T.*t252.*(1.9e+1./2.0e+1);
t275 = T.*t253.*(1.9e+1./2.0e+1);
t276 = T.*t254.*(1.9e+1./2.0e+1);
t277 = T.*t255.*(1.9e+1./2.0e+1);
t278 = T.*t256.*(1.9e+1./2.0e+1);
t279 = T.*t257.*(1.9e+1./2.0e+1);
t280 = T.*t258.*(1.9e+1./2.0e+1);
t281 = T.*t259.*(1.9e+1./2.0e+1);
t282 = T.*t260.*(1.9e+1./2.0e+1);
t283 = T.*t261.*(1.9e+1./2.0e+1);
t284 = T.*t262.*(1.9e+1./2.0e+1);
t285 = T.*t263.*(1.9e+1./2.0e+1);
t286 = T.*t264.*(1.9e+1./2.0e+1);
t287 = T.*t265.*(1.9e+1./2.0e+1);
t310 = t2+t3+t4+t5+t6+t9+t13+t17+t21+t40+t42+t197+t201+t205+t209+t213+t217+t221+t225+t229+t233;
t311 = t10+t14+t18+t22+t24+t26+t29+t32+t35+t39+t41+t198+t202+t206+t210+t214+t218+t222+t226+t231+t234;
t312 = t7+t25+t46+t49+t52+t55+t57+t59+t60+t63+t67+t77+t95+t113+t131+t149+t167+t184+t199+t200+t235;
t313 = t8+t27+t45+t58+t64+t68+t71+t74+t76+t79+t80+t83+t97+t115+t133+t151+t168+t186+t203+t204+t236;
t314 = t11+t28+t47+t62+t75+t78+t84+t87+t90+t93+t96+t99+t100+t117+t135+t152+t170+t187+t207+t208+t237;
t315 = t12+t30+t48+t65+t82+t92+t94+t98+t103+t106+t110+t112+t116+t119+t136+t154+t171+t189+t211+t212+t238;
t316 = t15+t31+t50+t66+t85+t102+t108+t111+t114+t118+t122+t126+t129+t132+t138+t155+t174+t190+t215+t216+t239;
t317 = t16+t33+t51+t69+t86+t104+t121+t124+t128+t130+t134+t137+t142+t146+t148+t158+t175+t192+t219+t220+t240;
t318 = t19+t34+t53+t70+t88+t105+t123+t140+t141+t144+t147+t150+t153+t156+t162+t165+t178+t193+t223+t224+t241;
t319 = t20+t36+t54+t72+t89+t107+t125+t143+t157+t160+t161+t164+t166+t169+t172+t176+t182+t195+t227+t228+t242;
t320 = t23+t37+t56+t73+t91+t109+t127+t145+t163+t173+t177+t180+t181+t183+t185+t188+t191+t194+t230+t232+t243;
t288 = -t266;
t289 = -t267;
t290 = -t268;
t291 = -t269;
t292 = -t270;
t293 = -t271;
t294 = -t272;
t295 = -t273;
t296 = -t274;
t297 = -t275;
t298 = -t276;
t299 = -t277;
t300 = -t278;
t301 = -t279;
t302 = -t280;
t303 = -t281;
t304 = -t282;
t305 = -t283;
t306 = -t284;
t307 = -t285;
t308 = -t286;
t309 = -t287;
t321 = (T.*t310)./2.0e+1;
t322 = (T.*t311)./2.0e+1;
t323 = (T.*t312)./2.0e+1;
t324 = (T.*t313)./2.0e+1;
t325 = (T.*t314)./2.0e+1;
t326 = (T.*t315)./2.0e+1;
t327 = (T.*t316)./2.0e+1;
t328 = (T.*t317)./2.0e+1;
t329 = (T.*t318)./2.0e+1;
t330 = (T.*t319)./2.0e+1;
t331 = (T.*t320)./2.0e+1;
t332 = -t321;
t333 = -t322;
t334 = -t323;
t335 = -t324;
t336 = -t325;
t337 = -t326;
t338 = -t327;
t339 = -t328;
t340 = -t329;
t341 = -t330;
t342 = -t331;
t343 = t288+t299+t332;
t344 = t289+t300+t333;
t345 = t290+t301+t334;
t346 = t291+t302+t335;
t347 = t292+t303+t336;
t348 = t293+t304+t337;
t349 = t294+t305+t338;
t350 = t295+t306+t339;
t351 = t296+t307+t340;
t352 = t297+t308+t341;
t353 = t298+t309+t342;
mt1 = [0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348];
mt2 = [t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;t345;t346;t347;t348;t349;t350;t351;t352;t353;t343;t344;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0];
cl = [mt1;mt2];