    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
            paramMap.sections(nTotSects) = dumSection; %prealloc
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (rtP)
        ;%
            section.nData     = 55;
            section.data(55)  = dumData; %prealloc

                    ;% rtP.PIDController1_D
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.PIDController_D
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% rtP.PIDController_D_pgipdvl2t0
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% rtP.PIDController_D_cmpawloux5
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% rtP.PIDController_D_bbj5cvi5gf
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 4;

                    ;% rtP.PIDController_D_mgncsilnse
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 5;

                    ;% rtP.PIDController_I
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 6;

                    ;% rtP.PIDController1_I
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 7;

                    ;% rtP.PIDController_I_f1l2iwf344
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 8;

                    ;% rtP.PIDController_I_hz4g5v32qr
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 9;

                    ;% rtP.PIDController_I_ijsojcoxkl
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 10;

                    ;% rtP.PIDController_I_ay5xqbhbz1
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 11;

                    ;% rtP.PIDController1_InitialConditionForFilter
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 12;

                    ;% rtP.PIDController_InitialConditionForFilter
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 13;

                    ;% rtP.PIDController_InitialConditionForFilter_bjzek0osia
                    section.data(15).logicalSrcIdx = 14;
                    section.data(15).dtTransOffset = 14;

                    ;% rtP.PIDController_InitialConditionForFilter_aks1ksah31
                    section.data(16).logicalSrcIdx = 15;
                    section.data(16).dtTransOffset = 15;

                    ;% rtP.PIDController_InitialConditionForFilter_bojlusop1m
                    section.data(17).logicalSrcIdx = 16;
                    section.data(17).dtTransOffset = 16;

                    ;% rtP.PIDController_InitialConditionForFilter_nwa4cn4h0f
                    section.data(18).logicalSrcIdx = 17;
                    section.data(18).dtTransOffset = 17;

                    ;% rtP.PIDController1_InitialConditionForIntegrator
                    section.data(19).logicalSrcIdx = 18;
                    section.data(19).dtTransOffset = 18;

                    ;% rtP.PIDController_InitialConditionForIntegrator
                    section.data(20).logicalSrcIdx = 19;
                    section.data(20).dtTransOffset = 19;

                    ;% rtP.PIDController_InitialConditionForIntegrator_iwhwqfir5i
                    section.data(21).logicalSrcIdx = 20;
                    section.data(21).dtTransOffset = 20;

                    ;% rtP.PIDController_InitialConditionForIntegrator_cmdkqikqzm
                    section.data(22).logicalSrcIdx = 21;
                    section.data(22).dtTransOffset = 21;

                    ;% rtP.PIDController_InitialConditionForIntegrator_n5qjw4tqvb
                    section.data(23).logicalSrcIdx = 22;
                    section.data(23).dtTransOffset = 22;

                    ;% rtP.PIDController_InitialConditionForIntegrator_e3x5mj5zgc
                    section.data(24).logicalSrcIdx = 23;
                    section.data(24).dtTransOffset = 23;

                    ;% rtP.PIDController1_N
                    section.data(25).logicalSrcIdx = 24;
                    section.data(25).dtTransOffset = 24;

                    ;% rtP.PIDController_N
                    section.data(26).logicalSrcIdx = 25;
                    section.data(26).dtTransOffset = 25;

                    ;% rtP.PIDController_N_crc4kkzmcw
                    section.data(27).logicalSrcIdx = 26;
                    section.data(27).dtTransOffset = 26;

                    ;% rtP.PIDController_N_du3ycvz5e5
                    section.data(28).logicalSrcIdx = 27;
                    section.data(28).dtTransOffset = 27;

                    ;% rtP.PIDController_N_fdn2lygism
                    section.data(29).logicalSrcIdx = 28;
                    section.data(29).dtTransOffset = 28;

                    ;% rtP.PIDController_N_o5l1fyg2f1
                    section.data(30).logicalSrcIdx = 29;
                    section.data(30).dtTransOffset = 29;

                    ;% rtP.PIDController1_P
                    section.data(31).logicalSrcIdx = 30;
                    section.data(31).dtTransOffset = 30;

                    ;% rtP.PIDController_P
                    section.data(32).logicalSrcIdx = 31;
                    section.data(32).dtTransOffset = 31;

                    ;% rtP.PIDController_P_crkbcfm51g
                    section.data(33).logicalSrcIdx = 32;
                    section.data(33).dtTransOffset = 32;

                    ;% rtP.PIDController_P_eek150r3nk
                    section.data(34).logicalSrcIdx = 33;
                    section.data(34).dtTransOffset = 33;

                    ;% rtP.PIDController_P_cqjd4trxgu
                    section.data(35).logicalSrcIdx = 34;
                    section.data(35).dtTransOffset = 34;

                    ;% rtP.PIDController_P_mcr05tm55t
                    section.data(36).logicalSrcIdx = 35;
                    section.data(36).dtTransOffset = 35;

                    ;% rtP.DesiredAngleSetpoint_Time
                    section.data(37).logicalSrcIdx = 36;
                    section.data(37).dtTransOffset = 36;

                    ;% rtP.DesiredAngleSetpoint_Y0
                    section.data(38).logicalSrcIdx = 37;
                    section.data(38).dtTransOffset = 37;

                    ;% rtP.DesiredAngleSetpoint_YFinal
                    section.data(39).logicalSrcIdx = 38;
                    section.data(39).dtTransOffset = 38;

                    ;% rtP.Integrator_IC
                    section.data(40).logicalSrcIdx = 39;
                    section.data(40).dtTransOffset = 39;

                    ;% rtP.Gain_Gain
                    section.data(41).logicalSrcIdx = 40;
                    section.data(41).dtTransOffset = 40;

                    ;% rtP.Step_Time
                    section.data(42).logicalSrcIdx = 41;
                    section.data(42).dtTransOffset = 41;

                    ;% rtP.Step_Y0
                    section.data(43).logicalSrcIdx = 42;
                    section.data(43).dtTransOffset = 42;

                    ;% rtP.Step_YFinal
                    section.data(44).logicalSrcIdx = 43;
                    section.data(44).dtTransOffset = 43;

                    ;% rtP.Step_Time_ccmcys4jti
                    section.data(45).logicalSrcIdx = 44;
                    section.data(45).dtTransOffset = 44;

                    ;% rtP.Step_Y0_kuxnkwajmq
                    section.data(46).logicalSrcIdx = 45;
                    section.data(46).dtTransOffset = 45;

                    ;% rtP.Step_YFinal_mqgxnbnv5v
                    section.data(47).logicalSrcIdx = 46;
                    section.data(47).dtTransOffset = 46;

                    ;% rtP.Step_Time_leph3y11xg
                    section.data(48).logicalSrcIdx = 47;
                    section.data(48).dtTransOffset = 47;

                    ;% rtP.Step_Y0_lect43vy12
                    section.data(49).logicalSrcIdx = 48;
                    section.data(49).dtTransOffset = 48;

                    ;% rtP.Step_YFinal_comjertxag
                    section.data(50).logicalSrcIdx = 49;
                    section.data(50).dtTransOffset = 49;

                    ;% rtP.Step_Time_mcpuvzrbmb
                    section.data(51).logicalSrcIdx = 50;
                    section.data(51).dtTransOffset = 50;

                    ;% rtP.Step_Y0_bp5qi5uzqe
                    section.data(52).logicalSrcIdx = 51;
                    section.data(52).dtTransOffset = 51;

                    ;% rtP.Step_YFinal_gjnpk0skzo
                    section.data(53).logicalSrcIdx = 52;
                    section.data(53).dtTransOffset = 52;

                    ;% rtP.DEG_Value
                    section.data(54).logicalSrcIdx = 53;
                    section.data(54).dtTransOffset = 53;

                    ;% rtP.Gain1_Gain
                    section.data(55).logicalSrcIdx = 54;
                    section.data(55).dtTransOffset = 54;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (rtB)
        ;%
            section.nData     = 70;
            section.data(70)  = dumData; %prealloc

                    ;% rtB.mby2gg4jgt
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtB.difagtosdt
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% rtB.os0ebvnexy
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% rtB.gzkmrjqeel
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% rtB.djrrpvjwx5
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 4;

                    ;% rtB.dwo1yb3eu3
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 5;

                    ;% rtB.cerdp32tn4
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 6;

                    ;% rtB.hhzs1ndn4k
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 7;

                    ;% rtB.foqywexyz5
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 8;

                    ;% rtB.medr5m5ecu
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 9;

                    ;% rtB.lpppkq0mo3
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 10;

                    ;% rtB.ds3xmzccwx
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 11;

                    ;% rtB.dp4ex0qtp1
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 12;

                    ;% rtB.c2ihsbt0dj
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 13;

                    ;% rtB.lxgozgn0yl
                    section.data(15).logicalSrcIdx = 14;
                    section.data(15).dtTransOffset = 14;

                    ;% rtB.okw4xf2pdu
                    section.data(16).logicalSrcIdx = 15;
                    section.data(16).dtTransOffset = 15;

                    ;% rtB.eiselq1fmm
                    section.data(17).logicalSrcIdx = 16;
                    section.data(17).dtTransOffset = 16;

                    ;% rtB.go3oa1beoa
                    section.data(18).logicalSrcIdx = 17;
                    section.data(18).dtTransOffset = 17;

                    ;% rtB.p4hgdattf2
                    section.data(19).logicalSrcIdx = 18;
                    section.data(19).dtTransOffset = 18;

                    ;% rtB.gmhbg2apiv
                    section.data(20).logicalSrcIdx = 19;
                    section.data(20).dtTransOffset = 19;

                    ;% rtB.chisoov5k4
                    section.data(21).logicalSrcIdx = 20;
                    section.data(21).dtTransOffset = 20;

                    ;% rtB.gs0sxl0fxr
                    section.data(22).logicalSrcIdx = 21;
                    section.data(22).dtTransOffset = 21;

                    ;% rtB.gkjugtc5ui
                    section.data(23).logicalSrcIdx = 22;
                    section.data(23).dtTransOffset = 240;

                    ;% rtB.armhtwdr0g
                    section.data(24).logicalSrcIdx = 23;
                    section.data(24).dtTransOffset = 245;

                    ;% rtB.emfbzb4tgv
                    section.data(25).logicalSrcIdx = 24;
                    section.data(25).dtTransOffset = 246;

                    ;% rtB.h4sb2hdvqs
                    section.data(26).logicalSrcIdx = 25;
                    section.data(26).dtTransOffset = 247;

                    ;% rtB.ffqrfknnoz
                    section.data(27).logicalSrcIdx = 26;
                    section.data(27).dtTransOffset = 248;

                    ;% rtB.fyqu4fz511
                    section.data(28).logicalSrcIdx = 27;
                    section.data(28).dtTransOffset = 249;

                    ;% rtB.mrf3exm2xd
                    section.data(29).logicalSrcIdx = 28;
                    section.data(29).dtTransOffset = 250;

                    ;% rtB.eefa3vsr3u
                    section.data(30).logicalSrcIdx = 29;
                    section.data(30).dtTransOffset = 251;

                    ;% rtB.hi3rqi5vyt
                    section.data(31).logicalSrcIdx = 30;
                    section.data(31).dtTransOffset = 252;

                    ;% rtB.oieixnysf1
                    section.data(32).logicalSrcIdx = 31;
                    section.data(32).dtTransOffset = 253;

                    ;% rtB.k2g2xqija3
                    section.data(33).logicalSrcIdx = 32;
                    section.data(33).dtTransOffset = 254;

                    ;% rtB.misakfid0j
                    section.data(34).logicalSrcIdx = 33;
                    section.data(34).dtTransOffset = 255;

                    ;% rtB.gb4x0jfanf
                    section.data(35).logicalSrcIdx = 34;
                    section.data(35).dtTransOffset = 256;

                    ;% rtB.plhwhurr2l
                    section.data(36).logicalSrcIdx = 35;
                    section.data(36).dtTransOffset = 257;

                    ;% rtB.fajvs1ugem
                    section.data(37).logicalSrcIdx = 36;
                    section.data(37).dtTransOffset = 258;

                    ;% rtB.gzsfvwfyan
                    section.data(38).logicalSrcIdx = 37;
                    section.data(38).dtTransOffset = 259;

                    ;% rtB.hir5pemzzc
                    section.data(39).logicalSrcIdx = 38;
                    section.data(39).dtTransOffset = 260;

                    ;% rtB.azhyg2i4qg
                    section.data(40).logicalSrcIdx = 39;
                    section.data(40).dtTransOffset = 261;

                    ;% rtB.n3iwvgj15u
                    section.data(41).logicalSrcIdx = 40;
                    section.data(41).dtTransOffset = 262;

                    ;% rtB.gq4e4lo25z
                    section.data(42).logicalSrcIdx = 41;
                    section.data(42).dtTransOffset = 263;

                    ;% rtB.lj35kknfkm
                    section.data(43).logicalSrcIdx = 42;
                    section.data(43).dtTransOffset = 264;

                    ;% rtB.je41jdmpg2
                    section.data(44).logicalSrcIdx = 43;
                    section.data(44).dtTransOffset = 265;

                    ;% rtB.lrgijcn3ft
                    section.data(45).logicalSrcIdx = 44;
                    section.data(45).dtTransOffset = 266;

                    ;% rtB.cqo0fmq5gh
                    section.data(46).logicalSrcIdx = 45;
                    section.data(46).dtTransOffset = 267;

                    ;% rtB.htbabqfbrr
                    section.data(47).logicalSrcIdx = 46;
                    section.data(47).dtTransOffset = 268;

                    ;% rtB.a1lmlx15bv
                    section.data(48).logicalSrcIdx = 47;
                    section.data(48).dtTransOffset = 269;

                    ;% rtB.hzfqkzjqra
                    section.data(49).logicalSrcIdx = 48;
                    section.data(49).dtTransOffset = 270;

                    ;% rtB.kceny33cji
                    section.data(50).logicalSrcIdx = 49;
                    section.data(50).dtTransOffset = 271;

                    ;% rtB.ez0zrbtqsy
                    section.data(51).logicalSrcIdx = 50;
                    section.data(51).dtTransOffset = 272;

                    ;% rtB.l125fp4ibr
                    section.data(52).logicalSrcIdx = 51;
                    section.data(52).dtTransOffset = 273;

                    ;% rtB.erhydliqya
                    section.data(53).logicalSrcIdx = 52;
                    section.data(53).dtTransOffset = 274;

                    ;% rtB.jalptojx2m
                    section.data(54).logicalSrcIdx = 53;
                    section.data(54).dtTransOffset = 275;

                    ;% rtB.bdilohfvdz
                    section.data(55).logicalSrcIdx = 54;
                    section.data(55).dtTransOffset = 276;

                    ;% rtB.k42gmmqwyh
                    section.data(56).logicalSrcIdx = 55;
                    section.data(56).dtTransOffset = 277;

                    ;% rtB.jxfwvbegog
                    section.data(57).logicalSrcIdx = 56;
                    section.data(57).dtTransOffset = 278;

                    ;% rtB.owj31uuwct
                    section.data(58).logicalSrcIdx = 57;
                    section.data(58).dtTransOffset = 279;

                    ;% rtB.cb4gonmskq
                    section.data(59).logicalSrcIdx = 58;
                    section.data(59).dtTransOffset = 280;

                    ;% rtB.nvwjv0tfyk
                    section.data(60).logicalSrcIdx = 59;
                    section.data(60).dtTransOffset = 281;

                    ;% rtB.p1if2uccjq
                    section.data(61).logicalSrcIdx = 60;
                    section.data(61).dtTransOffset = 282;

                    ;% rtB.ntjvp1exme
                    section.data(62).logicalSrcIdx = 61;
                    section.data(62).dtTransOffset = 283;

                    ;% rtB.fnp1kweqe2
                    section.data(63).logicalSrcIdx = 62;
                    section.data(63).dtTransOffset = 284;

                    ;% rtB.k5kn2vruzi
                    section.data(64).logicalSrcIdx = 63;
                    section.data(64).dtTransOffset = 285;

                    ;% rtB.a0k5xyl25h
                    section.data(65).logicalSrcIdx = 64;
                    section.data(65).dtTransOffset = 289;

                    ;% rtB.ppu3j4e43j
                    section.data(66).logicalSrcIdx = 65;
                    section.data(66).dtTransOffset = 293;

                    ;% rtB.mngdq0v4mv
                    section.data(67).logicalSrcIdx = 66;
                    section.data(67).dtTransOffset = 297;

                    ;% rtB.h0nwpxh25t
                    section.data(68).logicalSrcIdx = 67;
                    section.data(68).dtTransOffset = 301;

                    ;% rtB.h3tfpg3121
                    section.data(69).logicalSrcIdx = 68;
                    section.data(69).dtTransOffset = 305;

                    ;% rtB.kobb0qacwc
                    section.data(70).logicalSrcIdx = 69;
                    section.data(70).dtTransOffset = 309;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 5;
        sectIdxOffset = 1;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (rtDW)
        ;%
            section.nData     = 12;
            section.data(12)  = dumData; %prealloc

                    ;% rtDW.ef0tjfqxq2
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.gsh0bhwzck
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 2;

                    ;% rtDW.f1tzv3yf52
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 3;

                    ;% rtDW.o4fsuetuds
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 4;

                    ;% rtDW.ahtltjzp4f
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 6;

                    ;% rtDW.l4plqwgfvd
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 8;

                    ;% rtDW.ldzxxir0ou
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 10;

                    ;% rtDW.ngjaietdzr
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 11;

                    ;% rtDW.abeyvkp1wu
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 12;

                    ;% rtDW.pn4f1iubcb
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 13;

                    ;% rtDW.nydhqhpgj2
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 14;

                    ;% rtDW.i4igjngorc
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 15;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 22;
            section.data(22)  = dumData; %prealloc

                    ;% rtDW.lsl5oegdau.LoggedData
                    section.data(1).logicalSrcIdx = 12;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.kke31lspbs
                    section.data(2).logicalSrcIdx = 13;
                    section.data(2).dtTransOffset = 2;

                    ;% rtDW.adc1dhx1xv
                    section.data(3).logicalSrcIdx = 14;
                    section.data(3).dtTransOffset = 3;

                    ;% rtDW.hwyzdtvule
                    section.data(4).logicalSrcIdx = 15;
                    section.data(4).dtTransOffset = 4;

                    ;% rtDW.fylriinzod
                    section.data(5).logicalSrcIdx = 16;
                    section.data(5).dtTransOffset = 5;

                    ;% rtDW.bdfbjodek0
                    section.data(6).logicalSrcIdx = 17;
                    section.data(6).dtTransOffset = 6;

                    ;% rtDW.ipybrgxmsj
                    section.data(7).logicalSrcIdx = 18;
                    section.data(7).dtTransOffset = 7;

                    ;% rtDW.gf3dofpjfq
                    section.data(8).logicalSrcIdx = 19;
                    section.data(8).dtTransOffset = 8;

                    ;% rtDW.fijtoipbrp
                    section.data(9).logicalSrcIdx = 20;
                    section.data(9).dtTransOffset = 9;

                    ;% rtDW.lazkvdq4xy
                    section.data(10).logicalSrcIdx = 21;
                    section.data(10).dtTransOffset = 10;

                    ;% rtDW.gajsjtep3g
                    section.data(11).logicalSrcIdx = 22;
                    section.data(11).dtTransOffset = 11;

                    ;% rtDW.fodkyk4o3c.LoggedData
                    section.data(12).logicalSrcIdx = 23;
                    section.data(12).dtTransOffset = 12;

                    ;% rtDW.b2qcbxfqhv.LoggedData
                    section.data(13).logicalSrcIdx = 24;
                    section.data(13).dtTransOffset = 14;

                    ;% rtDW.j40xjawgtd.LoggedData
                    section.data(14).logicalSrcIdx = 25;
                    section.data(14).dtTransOffset = 15;

                    ;% rtDW.bsizlxhy4g.LoggedData
                    section.data(15).logicalSrcIdx = 26;
                    section.data(15).dtTransOffset = 16;

                    ;% rtDW.if00eqe43y.LoggedData
                    section.data(16).logicalSrcIdx = 27;
                    section.data(16).dtTransOffset = 18;

                    ;% rtDW.m5agigng2p.LoggedData
                    section.data(17).logicalSrcIdx = 28;
                    section.data(17).dtTransOffset = 19;

                    ;% rtDW.nvkibpxgyb
                    section.data(18).logicalSrcIdx = 29;
                    section.data(18).dtTransOffset = 20;

                    ;% rtDW.f02lveu5sb
                    section.data(19).logicalSrcIdx = 30;
                    section.data(19).dtTransOffset = 21;

                    ;% rtDW.g5ujc20rct
                    section.data(20).logicalSrcIdx = 31;
                    section.data(20).dtTransOffset = 22;

                    ;% rtDW.pvocqr40ve
                    section.data(21).logicalSrcIdx = 32;
                    section.data(21).dtTransOffset = 23;

                    ;% rtDW.okvpaob3h5
                    section.data(22).logicalSrcIdx = 33;
                    section.data(22).dtTransOffset = 24;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 7;
            section.data(7)  = dumData; %prealloc

                    ;% rtDW.epi4mp50fu
                    section.data(1).logicalSrcIdx = 34;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.ojhughkrf1
                    section.data(2).logicalSrcIdx = 35;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.etq5dvu3fk
                    section.data(3).logicalSrcIdx = 36;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.d5wsirya45
                    section.data(4).logicalSrcIdx = 37;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.akbpgtx52o
                    section.data(5).logicalSrcIdx = 38;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.gh4k1smub5
                    section.data(6).logicalSrcIdx = 39;
                    section.data(6).dtTransOffset = 5;

                    ;% rtDW.k1k2vpdtfo
                    section.data(7).logicalSrcIdx = 40;
                    section.data(7).dtTransOffset = 6;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 4;
            section.data(4)  = dumData; %prealloc

                    ;% rtDW.jbiypwbkax
                    section.data(1).logicalSrcIdx = 41;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.ef4vp4u1ze
                    section.data(2).logicalSrcIdx = 42;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.gagbx1bw4u
                    section.data(3).logicalSrcIdx = 43;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.cdpmqpsy10
                    section.data(4).logicalSrcIdx = 44;
                    section.data(4).dtTransOffset = 3;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.ptfxlysanb
                    section.data(1).logicalSrcIdx = 45;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.cp44xoo0qb
                    section.data(2).logicalSrcIdx = 46;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(5) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 1176751829;
    targMap.checksum1 = 2934734618;
    targMap.checksum2 = 4185153874;
    targMap.checksum3 = 3565652223;

