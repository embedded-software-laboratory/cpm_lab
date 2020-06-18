% MIT License
% 
% Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of cpm_lab.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

function [trajectory_row] = get_trajectory(pos)
    trajectory = [2.260001 3.745000 1.216739 0.000010;...
    2.560043 3.745395 1.162774 0.003355;...
    2.840081 3.745780 1.128092 -0.003919;...
    3.120050 3.740395 1.109884 -0.047540;...
    3.398905 3.716023 1.096797 -0.159898;...
    3.670829 3.651048 1.053819 -0.376931;...
    3.912498 3.513856 0.803640 -0.812125;...
    4.022087 3.251596 0.132963 -1.167448;...
    4.054445 2.953448 0.203288 -1.199055;...
    4.065642 2.645513 -0.245664 -1.236723;...
    3.902990 2.377713 -1.018228 -0.813792;...
    3.601739 2.251320 -1.319360 -0.243277;...
    3.263066 2.225721 -1.373132 -0.017274;...
    2.923042 2.223267 -1.398086 -0.054277;...
    2.570693 2.157594 -1.325575 -0.504153;...
    2.278523 1.969615 -1.005177 -1.008489;...
    2.087999 1.667466 -0.483987 -1.328965;...
    2.026633 1.324327 -0.051979 -1.390183;...
    2.020310 0.984392 -0.091283 -1.357727;...
    1.936917 0.657581 -0.642081 -1.160127;...
    1.693122 0.445722 -1.203914 -0.451775;...
    1.387364 0.409455 -1.240591 0.049041;...
    1.079737 0.386963 -1.164269 -0.276695;...
    0.793834 0.367515 -1.113211 0.297223;...
    0.553809 0.507400 -0.823768 0.742763;...
    0.389585 0.732047 -0.464746 0.962933;...
    0.306952 0.977989 -0.222543 1.007389;...
    0.270627 1.225193 -0.087276 0.993957;...
    0.257801 1.474857 -0.023326 0.968826;...
    0.255146 1.714871 -0.002628 0.948418;...
    0.254999 1.944903 0.000040 0.940834;...
    0.255006 2.184937 0.000415 0.950716;...
    0.256061 2.424966 0.011401 0.975451;...
    0.263545 2.674869 0.056143 1.009258;...
    0.290061 2.933428 0.169017 1.038408;...
    0.354335 3.184972 0.382667 1.028406;...
    0.491320 3.427670 0.735402 0.879267;...
    0.728052 3.608270 1.084945 0.505057;...
    1.021174 3.626920 1.217132 -0.275099;...
    1.337236 3.588159 1.296257 0.064244;...
    1.664564 3.564033 1.285829 -0.397765;...
    1.926804 3.359882 0.720137 -1.188742;...
    2.019593 3.025763 0.105524 -1.423687;...
    2.027483 2.655867 0.070985 -1.457915;...
    2.098673 2.304685 0.554631 -1.376928;...
    2.314775 1.996210 1.119556 -0.991517;...
    2.636859 1.820132 1.429071 -0.421411;...
    3.002854 1.775099 1.470261 -0.009158;...
    3.362872 1.771967 1.439461 -0.052622;...
    3.717878 1.719510 1.332002 -0.438293;...
    3.997639 1.520084 0.755740 -1.124436;...
    4.074981 1.205327 -0.068637 -1.297790;...
    4.033738 0.888109 -0.106255 -1.238230;...
    3.984608 0.593625 -0.471105 -1.091594;...
    3.781313 0.396087 -1.016941 -0.513171;...
    3.517158 0.305351 -1.068765 -0.234650;...
    3.250002 0.267456 -1.052704 -0.084846;...
    2.990271 0.255741 -1.024388 -0.019168;...
    2.740246 0.254137 -1.000829 0.001211;...
    2.490212 0.254786 -0.985833 0.002217;...
    2.240177 0.255000 -0.980787 -0.000008;...
    2.000144 0.254764 -0.986636 -0.002325;...
    1.750110 0.254127 -1.002119 -0.000907;...
    1.500086 0.255928 -1.026469 0.020644;...
    1.240386 0.268247 -1.054307 0.088599;...
    0.973428 0.307450 -1.069863 0.242284;...
    0.710101 0.400479 -1.014315 0.526020;...
    0.511682 0.602527 -0.445354 1.106755;...
    0.464497 0.907676 -0.118101 1.241424;...
    0.424232 1.224956 -0.034736 1.302143;...
    0.513680 1.536139 0.805879 1.090734;...
    0.791302 1.722465 1.337388 0.421314;...
    1.146766 1.772304 1.438462 0.048176;...
    1.506790 1.775171 1.468341 0.012975;...
    1.872378 1.822907 1.422786 0.435581;...
    2.199772 2.009415 1.093222 1.017094;...
    2.404881 2.313651 0.538554 1.380480;...
    2.473360 2.675495 0.054571 1.455755;...
    2.481152 3.035379 0.114985 1.419724;...
    2.578271 3.368083 0.740740 1.171536;...
    2.844675 3.566799 1.289311 0.373342;...
    3.172397 3.587694 1.292939 -0.059481;...
    3.488240 3.629016 1.215516 0.265698;...
    3.780651 3.604114 1.071570 -0.523718;...
    4.008451 3.427944 0.733136 -0.875612;...
    4.145541 3.185306 0.381987 -1.025505;...
    4.211462 2.923905 0.163162 -1.037339;...
    4.236978 2.665239 0.053477 -1.008212;...
    4.244048 2.415322 0.010530 -0.975658;...
    4.244997 2.175292 0.000315 -0.952755;...
    4.245001 1.945260 0.000040 -0.945275;...
    4.244826 1.705227 -0.002998 -0.954330;...
    4.241959 1.465215 -0.024973 -0.975544;...
    4.228512 1.215587 -0.091473 -1.000233;...
    4.190936 0.968578 -0.230511 -1.011337;...
    4.101680 0.714450 -0.488457 -0.956914;...
    3.938980 0.500995 -0.838230 -0.733679;...
    3.687055 0.362970 -1.131124 -0.236116;...
    3.401134 0.391441 -1.168092 0.267515;...
    3.102998 0.409094 -1.241586 -0.043499;...
    2.788626 0.452988 -1.184155 0.499241;...
    2.553891 0.674944 -0.598101 1.181606;...
    2.479073 0.994017 -0.082804 1.356393;...
    2.472981 1.333964 -0.059294 1.387289;...
    2.408652 1.676511 -0.497004 1.320135;...
    2.214627 1.976405 -1.013844 0.992961;...
    1.920274 2.160974 -1.326425 0.488094;...
    1.577315 2.223253 -1.394795 0.054423;...
    1.227290 2.225850 -1.369051 0.019456;...
    0.888784 2.253112 -1.312498 0.254478;...
    0.589535 2.383808 -0.998095 0.830483;...
    0.432570 2.654991 -0.221081 1.235741;...
    0.447161 2.962958 0.200352 1.195761;...
    0.479048 3.261173 0.144203 1.165919;...
    0.594380 3.520617 0.826646 0.790914;...
    0.838267 3.654256 1.062335 0.369414;...
    1.110642 3.717391 1.106339 0.155738;...
    1.389586 3.740797 1.122975 0.045613;...
    1.669564 3.745810 1.145336 0.003276;...
    1.959603 3.745340 1.181361 -0.003245];

    %Get size
    [m,n] = size(trajectory);

    %modulo nutzen
    if pos >= 1
        pos = mod(pos-1, m)+1;
        trajectory_row = trajectory(pos,:);
    else
        error('Wrong index for pos!');
    end
end