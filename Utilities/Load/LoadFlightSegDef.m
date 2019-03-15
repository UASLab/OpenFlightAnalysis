function [seg] = LoadFlightSegDef(param, time_s)

switch lower(param.vehName)
    case {'mjolnir'}
        switch lower(param.fltNum)
            case {1, '1', 'flt01'}
                
                seg.climb.time = [NaN, NaN]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.approach.time = [NaN, NaN]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time); % Manual
                seg.flare.time = [NaN, NaN]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time); % Manual
                
            case {2, '2', 'flt02'}
                
                seg.climb.time = [446, 165]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.climb.cntrlEng = [478.8, NaN]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                
                iExcite = 1; seg.excite{iExcite}.time = [640.2 671.413594]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 2; seg.excite{iExcite}.time = [687.791677 721.905935]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 3; seg.excite{iExcite}.time = [740.161405 774.45522]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 4; seg.excite{iExcite}.time = [792.770526 823.069633]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 5; seg.excite{iExcite}.time = [843.861524 868.188604]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                
                iTurn = 1; seg.turn{iTurn}.time = [seg.excite{1}.time(2), seg.excite{2}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 2; seg.turn{iTurn}.time = [seg.excite{2}.time(2), seg.excite{3}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 3; seg.turn{iTurn}.time = [seg.excite{3}.time(2), seg.excite{4}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 4; seg.turn{iTurn}.time = [seg.excite{4}.time(2), seg.excite{5}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                
            case {3, '3', 'flt03'}
                
                seg.wind{1}.time = [1073, 1100]; [seg.wind{1}.indx] = SegmentTime2Index(time_s, seg.wind{1}.time);
                seg.wind{2}.time = [1114, 1136]; [seg.wind{2}.indx] = SegmentTime2Index(time_s, seg.wind{2}.time);
                seg.popu.time = [1150, 1171]; [seg.popu.indx] = SegmentTime2Index(time_s, seg.popu.time);
                
                seg.aircal.time = [seg.wind{1}.time(1), seg.popu.time(2)]; [seg.aircal.indx] = SegmentTime2Index(time_s, seg.aircal.time);
                
                iExcite = 1; seg.excite{iExcite}.time = [1205.550312 1228.480277]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 2; seg.excite{iExcite}.time = [1297.82791 1320.137839]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 3; seg.excite{iExcite}.time = [1267.288812 1287.76144]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 4; seg.excite{iExcite}.time = [1324.511921 1349.597892]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 5; seg.excite{iExcite}.time = [1355.729551 1376.680982]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 6; seg.excite{iExcite}.time = [1383.431739 1406.180554]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 7; seg.excite{iExcite}.time = [1414.069711 1436.638639]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 8; seg.excite{iExcite}.time = [1442.430646 1465.43884]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 9; seg.excite{iExcite}.time = [1471.869933 1493.14045]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 10; seg.excite{iExcite}.time = [1501.848365 1523.138767]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 11; seg.excite{iExcite}.time = [1536.060784 1550.860159]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 12; seg.excite{iExcite}.time = [1564.461164 1569.254457]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 13; seg.excite{iExcite}.time = [1572.769538 1577.822476]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 14; seg.excite{iExcite}.time = [1582.515921 1588.887019]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 15; seg.excite{iExcite}.time = [1598.693282 1603.266874]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 16; seg.excite{iExcite}.time = [1606.023016 1610.856234]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 17; seg.excite{iExcite}.time = [1626.953656 1633.72416]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 18; seg.excite{iExcite}.time = [1636.220662 1643.21086]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 19; seg.excite{iExcite}.time = [1658.149897 1676.504111]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                
                iTurn = 1; seg.turn{iTurn}.time = [seg.excite{1}.time(2), seg.excite{2}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 2; seg.turn{iTurn}.time = [seg.excite{2}.time(2), seg.excite{3}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 3; seg.turn{iTurn}.time = [seg.excite{3}.time(2), seg.excite{4}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 4; seg.turn{iTurn}.time = [seg.excite{4}.time(2), seg.excite{5}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 5; seg.turn{iTurn}.time = [seg.excite{5}.time(2), seg.excite{6}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 6; seg.turn{iTurn}.time = [seg.excite{6}.time(2), seg.excite{7}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 7; seg.turn{iTurn}.time = [seg.excite{7}.time(2), seg.excite{8}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 8; seg.turn{iTurn}.time = [seg.excite{8}.time(2), seg.excite{9}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 9; seg.turn{iTurn}.time = [seg.excite{9}.time(2), seg.excite{10}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 10; seg.turn{iTurn}.time = [seg.excite{10}.time(2), seg.excite{11}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 11; seg.turn{iTurn}.time = [seg.excite{11}.time(2), seg.excite{12}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                
            case {4, '4', 'flt04'}
                
                seg.wind{1}.time = [500, 528]; [seg.wind{1}.indx] = SegmentTime2Index(time_s, seg.wind{1}.time);
                seg.wind{2}.time = [537, 558]; [seg.wind{2}.indx] = SegmentTime2Index(time_s, seg.wind{2}.time);
                seg.popu.time = [580, 600]; [seg.popu.indx] = SegmentTime2Index(time_s, seg.popu.time);
                
                seg.aircal.time = [seg.wind{1}.time(1), seg.popu.time(2)]; [seg.aircal.indx] = SegmentTime2Index(time_s, seg.aircal.time);
                
                iExcite = 1; seg.excite{iExcite}.time = [631.548599 658.811462]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 2; seg.excite{iExcite}.time = [669.397053 690.887781]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 3; seg.excite{iExcite}.time = [709.422501 733.829275]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 4; seg.excite{iExcite}.time = [746.631829 780.625456]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 5; seg.excite{iExcite}.time = [788.994032 814.998588]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 6; seg.excite{iExcite}.time = [823.646793 847.254568]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 7; seg.excite{iExcite}.time = [863.13288 887.919082]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 8; seg.excite{iExcite}.time = [901.939963 924.349401]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 9; seg.excite{iExcite}.time = [937.411569 963.136485]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 10; seg.excite{iExcite}.time = [979.054764 989.740185]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 11; seg.excite{iExcite}.time = [1008.234918 1015.065588]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 12; seg.excite{iExcite}.time = [1017.602123 1024.552632]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 13; seg.excite{iExcite}.time = [1027.248953 1051.096381]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 14; seg.excite{iExcite}.time = [1053.133594 1058.186682]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 15; seg.excite{iExcite}.time = [1071.388601 1075.902409]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 16; seg.excite{iExcite}.time = [1082.852883 1120.401513]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 17; seg.excite{iExcite}.time = [1122.538589 1135.740534]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 18; seg.excite{iExcite}.time = [1140.45408 1151.498935]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 19; seg.excite{iExcite}.time = [1153.536144 1163.342716]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                
                iTurn = 1; seg.turn{iTurn}.time = [seg.excite{1}.time(2), seg.excite{2}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 2; seg.turn{iTurn}.time = [seg.excite{2}.time(2), seg.excite{3}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 3; seg.turn{iTurn}.time = [seg.excite{3}.time(2), seg.excite{4}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 4; seg.turn{iTurn}.time = [seg.excite{4}.time(2), seg.excite{5}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 5; seg.turn{iTurn}.time = [seg.excite{5}.time(2), seg.excite{6}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 6; seg.turn{iTurn}.time = [seg.excite{6}.time(2), seg.excite{7}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 7; seg.turn{iTurn}.time = [seg.excite{7}.time(2), seg.excite{8}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 8; seg.turn{iTurn}.time = [seg.excite{8}.time(2), seg.excite{9}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 9; seg.turn{iTurn}.time = [seg.excite{9}.time(2), seg.excite{10}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 10; seg.turn{iTurn}.time = [seg.excite{10}.time(2), seg.excite{11}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 11; seg.turn{iTurn}.time = [seg.excite{11}.time(2), seg.excite{12}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                
            case {5, '5', 'flt05'}
                
                seg.wind{1}.time = [1276, 1300]; [seg.wind{1}.indx] = SegmentTime2Index(time_s, seg.wind{1}.time);
                seg.wind{2}.time = [1314, 1334]; [seg.wind{2}.indx] = SegmentTime2Index(time_s, seg.wind{2}.time);
                seg.popu.time = [1351, 1372]; [seg.popu.indx] = SegmentTime2Index(time_s, seg.popu.time);
                
                seg.aircal.time = [seg.wind{1}.time(1), seg.popu.time(2)]; [seg.aircal.indx] = SegmentTime2Index(time_s, seg.aircal.time);
                
                iExcite = 1; seg.excite{iExcite}.time = [1377.14518 1402.611827]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 2; seg.excite{iExcite}.time = [1408.524049 1434.909178]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 3; seg.excite{iExcite}.time = [1443.477814 1469.243522]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 4; seg.excite{iExcite}.time = [1473.977213 1503.357871]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 5; seg.excite{iExcite}.time = [1512.066168 1536.932642]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 6; seg.excite{iExcite}.time = [1545.14154 1568.969236]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 7; seg.excite{iExcite}.time = [1580.453622 1607.556688]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 8; seg.excite{iExcite}.time = [1614.946601 1674.984431]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 9; seg.excite{iExcite}.time = [1683.832296 1708.837931]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 10; seg.excite{iExcite}.time = [1720.581796 1734.502634]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 11; seg.excite{iExcite}.time = [1747.804327 1752.05847]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 12; seg.excite{iExcite}.time = [1754.415226 1758.449673]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 13; seg.excite{iExcite}.time = [1762.284393 1778.941474]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 14; seg.excite{iExcite}.time = [1781.358148 1786.990385]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 15; seg.excite{iExcite}.time = [1796.856791 1802.24936]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 16; seg.excite{iExcite}.time = [1806.0641 1812.674979]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 17; seg.excite{iExcite}.time = [1828.133637 1833.805807]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 18; seg.excite{iExcite}.time = [1835.603324 1842.114325]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 19; seg.excite{iExcite}.time = [1855.555772 1912.197705]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                
                iTurn = 1; seg.turn{iTurn}.time = [seg.excite{1}.time(2), seg.excite{2}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 2; seg.turn{iTurn}.time = [seg.excite{2}.time(2), seg.excite{3}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 3; seg.turn{iTurn}.time = [seg.excite{3}.time(2), seg.excite{4}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 4; seg.turn{iTurn}.time = [seg.excite{4}.time(2), seg.excite{5}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 5; seg.turn{iTurn}.time = [seg.excite{5}.time(2), seg.excite{6}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 6; seg.turn{iTurn}.time = [seg.excite{6}.time(2), seg.excite{7}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 7; seg.turn{iTurn}.time = [seg.excite{7}.time(2), seg.excite{8}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 8; seg.turn{iTurn}.time = [seg.excite{8}.time(2), seg.excite{9}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 9; seg.turn{iTurn}.time = [seg.excite{9}.time(2), seg.excite{10}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 10; seg.turn{iTurn}.time = [seg.excite{10}.time(2), seg.excite{11}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 11; seg.turn{iTurn}.time = [seg.excite{11}.time(2), seg.excite{12}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                
            case {6, '6', 'flt06'}
                
                seg.wind{1}.time = [384, 408.2]; [seg.wind{1}.indx] = SegmentTime2Index(time_s, seg.wind{1}.time);
                seg.wind{2}.time = [411, 438.3]; [seg.wind{2}.indx] = SegmentTime2Index(time_s, seg.wind{2}.time);
                seg.popu.time = [510.9, 530]; [seg.popu.indx] = SegmentTime2Index(time_s, seg.popu.time);
                
                seg.aircal.time = [seg.wind{1}.time(1), seg.popu.time(2)]; [seg.aircal.indx] = SegmentTime2Index(time_s, seg.aircal.time);
                
                iExcite = 1; seg.excite{iExcite}.time = [573.753227 602.213793]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 2; seg.excite{iExcite}.time = [630.534652 663.269581]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 3; seg.excite{iExcite}.time = [689.933008 723.167561]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 4; seg.excite{iExcite}.time = [750.709972 782.227061]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 5; seg.excite{iExcite}.time = [807.233079 838.790306]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 6; seg.excite{iExcite}.time = [865.933549 895.533545]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 7; seg.excite{iExcite}.time = [921.019148 951.498084]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 8; seg.excite{iExcite}.time = [980.079635 1013.594655]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 9; seg.excite{iExcite}.time = [1039.499899 1072.176071]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 10; seg.excite{iExcite}.time = [1076.670054 1081.184014]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 11; seg.excite{iExcite}.time = [1106.070669 1110.065318]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 12; seg.excite{iExcite}.time = [1112.402193 1117.195777]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 13; seg.excite{iExcite}.time = [1119.772328 1140.963963]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 14; seg.excite{iExcite}.time = [1143.240917 1165.271394]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 15; seg.excite{iExcite}.time = [1178.134156 1203.560098]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 16; seg.excite{iExcite}.time = [1226.629206 1231.003349]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 17; seg.excite{iExcite}.time = [1233.979362 1243.44669]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 18; seg.excite{iExcite}.time = [1255.770188 1264.138981]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 19; seg.excite{iExcite}.time = [1287.387836 1300.210687]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                
                % Tested upwind only for Multisines and Chirps, some segments consist of 2 turns and a downwind segment
                iTurn = 1; seg.turn{iTurn}.time = [seg.excite{1}.time(2), seg.excite{2}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 2; seg.turn{iTurn}.time = [seg.excite{2}.time(2), seg.excite{3}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 3; seg.turn{iTurn}.time = [seg.excite{3}.time(2), seg.excite{4}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 4; seg.turn{iTurn}.time = [seg.excite{4}.time(2), seg.excite{5}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 5; seg.turn{iTurn}.time = [seg.excite{5}.time(2), seg.excite{6}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 6; seg.turn{iTurn}.time = [seg.excite{6}.time(2), seg.excite{7}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 7; seg.turn{iTurn}.time = [seg.excite{7}.time(2), seg.excite{8}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 8; seg.turn{iTurn}.time = [seg.excite{8}.time(2), seg.excite{9}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 9; seg.turn{iTurn}.time = [seg.excite{9}.time(2), seg.excite{10}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 10; seg.turn{iTurn}.time = [seg.excite{10}.time(2), seg.excite{11}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 11; seg.turn{iTurn}.time = [seg.excite{11}.time(2), seg.excite{12}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                
            case {7, '7', 'flt07'}
                
                seg.wind{1}.time = [542, 577.1]; [seg.wind{1}.indx] = SegmentTime2Index(time_s, seg.wind{1}.time);
                seg.wind{2}.time = [581.6, 610.9]; [seg.wind{2}.indx] = SegmentTime2Index(time_s, seg.wind{2}.time);
                seg.popu.time = [620.6, 645]; [seg.popu.indx] = SegmentTime2Index(time_s, seg.popu.time);
                
                seg.aircal.time = [seg.wind{1}.time(1), seg.popu.time(2)]; [seg.aircal.indx] = SegmentTime2Index(time_s, seg.aircal.time);
                
                iExcite = 1; seg.excite{iExcite}.time = [671.45488 698.036988]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 2; seg.excite{iExcite}.time = [724.8986 751.06116]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 3; seg.excite{iExcite}.time = [782.076659 810.276122]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 4; seg.excite{iExcite}.time = [836.797968 864.697786]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 5; seg.excite{iExcite}.time = [893.436351 923.892423]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 6; seg.excite{iExcite}.time = [948.037583 972.941616]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 7; seg.excite{iExcite}.time = [1006.932455 1033.813569]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 8; seg.excite{iExcite}.time = [1062.372209 1093.267486]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 9; seg.excite{iExcite}.time = [1120.567986 1151.263565]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 10; seg.excite{iExcite}.time = [1154.578772 1158.832621]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 11; seg.excite{iExcite}.time = [1160.769816 1164.903831]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 12; seg.excite{iExcite}.time = [1180.321535 1184.93487]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 13; seg.excite{iExcite}.time = [1188.130255 1222.280835]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 14; seg.excite{iExcite}.time = [1238.35758 1242.132124]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 15; seg.excite{iExcite}.time = [1245.207677 1250.040688]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 16; seg.excite{iExcite}.time = [1264.599636 1271.190115]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 17; seg.excite{iExcite}.time = [1285.789041 1292.579252]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 18; seg.excite{iExcite}.time = [1294.396636 1303.144028]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 19; seg.excite{iExcite}.time = [1317.962698 1359.682671]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                                
                % Tested upwind only for Multisines and Chirps, some segments consist of 2 turns and a downwind segment
                iTurn = 1; seg.turn{iTurn}.time = [seg.excite{1}.time(2), seg.excite{2}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 2; seg.turn{iTurn}.time = [seg.excite{2}.time(2), seg.excite{3}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 3; seg.turn{iTurn}.time = [seg.excite{3}.time(2), seg.excite{4}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 4; seg.turn{iTurn}.time = [seg.excite{4}.time(2), seg.excite{5}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 5; seg.turn{iTurn}.time = [seg.excite{5}.time(2), seg.excite{6}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 6; seg.turn{iTurn}.time = [seg.excite{6}.time(2), seg.excite{7}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 7; seg.turn{iTurn}.time = [seg.excite{7}.time(2), seg.excite{8}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 8; seg.turn{iTurn}.time = [seg.excite{8}.time(2), seg.excite{9}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 9; seg.turn{iTurn}.time = [seg.excite{9}.time(2), seg.excite{10}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 10; seg.turn{iTurn}.time = [seg.excite{10}.time(2), seg.excite{11}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                iTurn = 11; seg.turn{iTurn}.time = [seg.excite{11}.time(2), seg.excite{12}.time(1)]; seg.turn{iTurn}.indx = SegmentTime2Index(time_s, seg.turn{iTurn}.time);
                
            otherwise
                disp('Configuration for aircraft unknown')
                
        end
        
    case {'skoll'}
        switch lower(param.fltNum)
            case {3, '3', 'flt03'}
                
                seg.climb.time = [NaN, NaN]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.turn{1}.time = [NaN, NaN]; [seg.turn{1}.indx] = SegmentTime2Index(time_s, seg.turn{1}.time);
                seg.turn{2}.time = [NaN, NaN]; [seg.turn{2}.indx] = SegmentTime2Index(time_s, seg.turn{2}.time);
                seg.turn{3}.time = [NaN, NaN]; [seg.turn{3}.indx] = SegmentTime2Index(time_s, seg.turn{3}.time);
                seg.turn{4}.time = [NaN, NaN]; [seg.turn{4}.indx] = SegmentTime2Index(time_s, seg.turn{4}.time);
                seg.approach.time = [NaN, NaN]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time);
                seg.flare.time = [NaN, NaN]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time);
                
            case {4, '4', 'flt04'}
                
                seg.climb.time = [NaN, NaN]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.turn{1}.time = [NaN, NaN]; [seg.turn{1}.indx] = SegmentTime2Index(time_s, seg.turn{1}.time);
                seg.accel{1}.time = [650, 656]; [seg.accel{1}.indx] = SegmentTime2Index(time_s, seg.accel{1}.time);
                seg.turn{2}.time = [683, 715]; [seg.turn{2}.indx] = SegmentTime2Index(time_s, seg.turn{2}.time);
                seg.accel{2}.time = [720, 727]; [seg.accel{2}.indx] = SegmentTime2Index(time_s, seg.accel{2}.time);
                seg.turnaccel{1}.time = [680, 727]; [seg.turnaccel{1}.indx] = SegmentTime2Index(time_s, seg.turnaccel{1}.time);
                seg.turn{3}.time = [765, 792]; [seg.turn{3}.indx] = SegmentTime2Index(time_s, seg.turn{3}.time);
                seg.approach.time = [792, 810]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time);
                seg.flare.time = [810, 819]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time);
                
            case {9, '9', 'flt09'}
                
                seg.climb.time = [NaN, NaN]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.turn{1}.time = [178, 208]; [seg.turn{1}.indx] = SegmentTime2Index(time_s, seg.turn{1}.time);
                seg.turn{2}.time = [238, 262]; [seg.turn{2}.indx] = SegmentTime2Index(time_s, seg.turn{2}.time);
                seg.turn{3}.time = [294, 317]; [seg.turn{3}.indx] = SegmentTime2Index(time_s, seg.turn{3}.time);
                seg.turn{4}.time = [347, 372]; [seg.turn{4}.indx] = SegmentTime2Index(time_s, seg.turn{4}.time);
                seg.approach.time = [372, 388]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time);
                seg.flare.time = [388, 399]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time);
                
            case {12, '12', 'flt12'}
                
                seg.climb.time = [NaN, NaN]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.turn{1}.time = [257, 289]; [seg.turn{1}.indx] = SegmentTime2Index(time_s, seg.turn{1}.time);
                seg.accel{1}.time = [289, 296]; [seg.accel{1}.indx] = SegmentTime2Index(time_s, seg.accel{1}.time);
                seg.turnaccel{1}.time = [257, 296]; [seg.turnaccel{1}.indx] = SegmentTime2Index(time_s, seg.turnaccel{1}.time);
                
                seg.turn{2}.time = [318, 339]; [seg.turn{2}.indx] = SegmentTime2Index(time_s, seg.turn{2}.time);
                seg.accel{2}.time = [340, 346]; [seg.accel{2}.indx] = SegmentTime2Index(time_s, seg.accel{2}.time);
                seg.turnaccel{2}.time = [318, 346]; [seg.turnaccel{2}.indx] = SegmentTime2Index(time_s, seg.turnaccel{2}.time);
                
                seg.turn{3}.time = [368, 408]; [seg.turn{3}.indx] = SegmentTime2Index(time_s, seg.turn{3}.time);
                seg.approach.time = [408, 421]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time);
                seg.flare.time = [421, 432]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time);
                
            otherwise
                disp('Flight Designation unknown');
        end
        
    case {'geri'}
        switch lower(param.fltNum)
            case {1, '1', 'flt01'}
                
                seg.climb.time = [NaN, NaN]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.approach.time = [NaN, NaN]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time); % Manual
                seg.flare.time = [NaN, NaN]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time); % Manual
                
            case {2, '2', 'flt02'}
                
                seg.climb.time = [505, 539.4]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.turn{1}.time = [523.2, 537.1]; [seg.turn{1}.indx] = SegmentTime2Index(time_s, seg.turn{1}.time);
                
                seg.excite{1}.time = [539.4, 563]; [seg.excite{1}.indx] = SegmentTime2Index(time_s, seg.excite{1}.time);
                
                seg.turn{2}.time = [574.4, 588.8]; [seg.turn{2}.indx] = SegmentTime2Index(time_s, seg.turn{2}.time);
                
                seg.excite{2}.time = [590.1, 614]; [seg.excite{2}.indx] = SegmentTime2Index(time_s, seg.excite{2}.time);
                
                seg.turn{3}.time = [630.4, 648.3]; [seg.turn{3}.indx] = SegmentTime2Index(time_s, seg.turn{3}.time);
                
                seg.excite{3}.time = [650.7, 674]; [seg.excite{3}.indx] = SegmentTime2Index(time_s, seg.excite{3}.time);
                
                seg.turn{4}.time = [684.7, 707.8]; [seg.turn{4}.indx] = SegmentTime2Index(time_s, seg.turn{4}.time);
                
                seg.approach.time = [711.2, 717]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time);
                seg.flare.time = [717, 719.3]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time);
                
                seg.flt.time = [seg.climb.time(1), seg.approach.time(2)]; [seg.flt.indx] = SegmentTime2Index(time_s, seg.flt.time);
                
            case {3, '3', 'flt03'}
                
                seg.climb.time = [NaN, NaN]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.turn.time = [NaN, NaN]; [seg.turn{1}.indx] = SegmentTime2Index(time_s, seg.turn{1}.time);
                seg.approach.time = [NaN, NaN]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time); % Manual
                seg.flare.time = [NaN, NaN]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time); % Manual
                
                seg.flt.time = [seg.climb.time(1), seg.approach.time(2)]; [seg.flt.indx] = SegmentTime2Index(time_s, seg.flt.time);
                
            case {4, '4', 'flt04'}
                
                seg.climb.time = [547, 580]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.turn{1}.time = [564, 585]; [seg.turn{1}.indx] = SegmentTime2Index(time_s, seg.turn{1}.time);
                
                seg.excite{1}.time = [585, 601]; [seg.excite{1}.indx] = SegmentTime2Index(time_s, seg.excite{1}.time);
                seg.excite{2}.time = [601, 616]; [seg.excite{2}.indx] = SegmentTime2Index(time_s, seg.excite{2}.time);
                
                seg.turn{2}.time = [614, 634]; [seg.turn{2}.indx] = SegmentTime2Index(time_s, seg.turn{2}.time);
                
                seg.excite{3}.time = [619, 634]; [seg.excite{3}.indx] = SegmentTime2Index(time_s, seg.excite{3}.time); % Executed during the turn and switched mid-excitation
                seg.excite{4}.time = [634, 649]; [seg.excite{4}.indx] = SegmentTime2Index(time_s, seg.excite{4}.time);
                
                seg.turn{3}.time = [670, 690]; [seg.turn{3}.indx] = SegmentTime2Index(time_s, seg.turn{3}.time);
                
                seg.excite{5}.time = [691, 706]; [seg.excite{5}.indx] = SegmentTime2Index(time_s, seg.excite{5}.time);
                seg.excite{6}.time = [706, 720]; [seg.excite{6}.indx] = SegmentTime2Index(time_s, seg.excite{6}.time);
                
                seg.turn{4}.time = [718, 747]; [seg.turn{4}.indx] = SegmentTime2Index(time_s, seg.turn{4}.time);
                
                seg.approach.time = [746.7, 761.7]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time);
                seg.flare.time = [761.7, 791.2]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time);
                
                seg.flt.time = [seg.climb.time(1), seg.approach.time(2)]; [seg.flt.indx] = SegmentTime2Index(time_s, seg.flt.time);
                
            case {5, '5', 'flt05'}
                
                seg.climb.time = [480, 498]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.turn{1}.time = [498, 515]; [seg.turn{1}.indx] = SegmentTime2Index(time_s, seg.turn{1}.time);
                
                seg.excite{1}.time = [515, 538]; [seg.excite{1}.indx] = SegmentTime2Index(time_s, seg.excite{1}.time);
                
                seg.turn{2}.time = [545, 562]; [seg.turn{2}.indx] = SegmentTime2Index(time_s, seg.turn{2}.time);
                
                seg.excite{2}.time = [562, 586]; [seg.excite{2}.indx] = SegmentTime2Index(time_s, seg.excite{2}.time);
                
                seg.turn{3}.time = [605, 619]; [seg.turn{3}.indx] = SegmentTime2Index(time_s, seg.turn{3}.time);
                
                seg.excite{3}.time = [618.1, 632.6]; [seg.excite{3}.indx] = SegmentTime2Index(time_s, seg.excite{3}.time);
                seg.excite{4}.time = [632.6, 646.2]; [seg.excite{4}.indx] = SegmentTime2Index(time_s, seg.excite{4}.time);
                
                seg.turn{4}.time = [647.3, 675]; [seg.turn{4}.indx] = SegmentTime2Index(time_s, seg.turn{4}.time);
                
                seg.approach.time = [677.3, 687]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time);
                seg.flare.time = [688, 690.7]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time);
                
                seg.flt.time = [seg.climb.time(1), seg.approach.time(2)]; [seg.flt.indx] = SegmentTime2Index(time_s, seg.flt.time);
                
            case {8, '8', 'flt08'}
                
                seg.climb.time = [276, 295]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.turn{1}.time = [296, 312]; [seg.turn{1}.indx] = SegmentTime2Index(time_s, seg.turn{1}.time);
                
                seg.excite{1}.time = [312.7, 333]; [seg.excite{1}.indx] = SegmentTime2Index(time_s, seg.excite{1}.time);
                
                seg.turn{2}.time = [336, 354]; [seg.turn{2}.indx] = SegmentTime2Index(time_s, seg.turn{2}.time);
                
                seg.excite{2}.time = [350.6, 360.6]; [seg.excite{2}.indx] = SegmentTime2Index(time_s, seg.excite{2}.time);
                seg.excite{3}.time = [360.6, 377]; [seg.excite{3}.indx] = SegmentTime2Index(time_s, seg.excite{3}.time);
                
                seg.turn{3}.time = [377, 391]; [seg.turn{3}.indx] = SegmentTime2Index(time_s, seg.turn{3}.time);
                
                seg.excite{4}.time = [392.2, 402.2]; [seg.excite{4}.indx] = SegmentTime2Index(time_s, seg.excite{4}.time);
                seg.excite{5}.time = [402.2, 418]; [seg.excite{5}.indx] = SegmentTime2Index(time_s, seg.excite{5}.time);
                
                seg.turn{4}.time = [420, 439]; [seg.turn{4}.indx] = SegmentTime2Index(time_s, seg.turn{4}.time);
                
                seg.approach.time = [445.6, 461.2]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time);
                seg.flare.time = [461.2, 467]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time);
                
                seg.flt.time = [seg.climb.time(1), seg.approach.time(2)]; [seg.flt.indx] = SegmentTime2Index(time_s, seg.flt.time);
                
            case {13, '13', 'flt13'}
                
                seg.climb.time = [514, 523]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.turn{1}.time = [523, 541]; [seg.turn{1}.indx] = SegmentTime2Index(time_s, seg.turn{1}.time);
                
                seg.excite{1}.time = [544.5, 556.7]; [seg.excite{1}.indx] = SegmentTime2Index(time_s, seg.excite{1}.time);
                
                seg.turn{2}.time = [571, 583]; [seg.turn{2}.indx] = SegmentTime2Index(time_s, seg.turn{2}.time);
                
                seg.excite{2}.time = [585.1, 591.9]; [seg.excite{2}.indx] = SegmentTime2Index(time_s, seg.excite{2}.time);
                seg.excite{3}.time = [591.9, 604.1]; [seg.excite{3}.indx] = SegmentTime2Index(time_s, seg.excite{3}.time);
                
                seg.turn{3}.time = [612, 631]; [seg.turn{3}.indx] = SegmentTime2Index(time_s, seg.turn{3}.time);
                
                seg.excite{4}.time = [630.3, 636.5]; [seg.excite{4}.indx] = SegmentTime2Index(time_s, seg.excite{4}.time);
                seg.excite{5}.time = [636.5, 648.7]; [seg.excite{5}.indx] = SegmentTime2Index(time_s, seg.excite{5}.time);
                
                seg.turn{4}.time = [658, 672]; [seg.turn{4}.indx] = SegmentTime2Index(time_s, seg.turn{4}.time);
                
                seg.approach.time = [665, 698.8]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time);
                seg.flare.time = [698.8, 708]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time);
                
                seg.flt.time = [seg.climb.time(1), seg.approach.time(2)]; [seg.flt.indx] = SegmentTime2Index(time_s, seg.flt.time);
                
            case {14, '14', 'flt14'}
                
                seg.climb.time = [640, 661.6]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.turn{1}.time = [656.2, 674.5]; [seg.turn{1}.indx] = SegmentTime2Index(time_s, seg.turn{1}.time);
                seg.turnaccel{1}.time = [seg.turn{1}.time(1), 678.3]; [seg.turnaccel{1}.indx] = SegmentTime2Index(time_s, seg.turnaccel{1}.time);
                
                seg.excite{1}.time = [674.5, 694.6]; [seg.excite{1}.indx] = SegmentTime2Index(time_s, seg.excite{1}.time);
                
                seg.turn{2}.time = [694.6, 713.1]; [seg.turn{2}.indx] = SegmentTime2Index(time_s, seg.turn{2}.time);
                seg.turnaccel{2}.time = [690.7, 718.9]; [seg.turnaccel{2}.indx] = SegmentTime2Index(time_s, seg.turnaccel{2}.time);
                
                seg.excite{2}.time = [713.1, 733]; [seg.excite{2}.indx] = SegmentTime2Index(time_s, seg.excite{2}.time);
                
                seg.turn{3}.time = [733, 756.4]; [seg.turn{3}.indx] = SegmentTime2Index(time_s, seg.turn{3}.time);
                seg.turnaccel{3}.time = [731.2, 756.3]; [seg.turnaccel{3}.indx] = SegmentTime2Index(time_s, seg.turnaccel{3}.time);
                
                seg.excite{3}.time = [750.8, 770.5]; [seg.excite{3}.indx] = SegmentTime2Index(time_s, seg.excite{3}.time);
                
                seg.turn{4}.time = [772, 800]; [seg.turn{4}.indx] = SegmentTime2Index(time_s, seg.turn{4}.time);
                
                seg.approach.time = [794.4, 807]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time);
                seg.flare.time = [807, 812]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time);
                
                seg.flt.time = [seg.climb.time(1), seg.approach.time(2)]; [seg.flt.indx] = SegmentTime2Index(time_s, seg.flt.time);
                
            case {15, '15', 'flt15'}
                
                seg.climb.time = [405, 432]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.turn{1}.time = [420, 435.3]; [seg.turn{1}.indx] = SegmentTime2Index(time_s, seg.turn{1}.time);
                seg.turnaccel{1}.time = [seg.turn{1}.time(1), 440.7]; [seg.turnaccel{1}.indx] = SegmentTime2Index(time_s, seg.turnaccel{1}.time);
                
                seg.excite{1}.time = [435.3, 455.2]; [seg.excite{1}.indx] = SegmentTime2Index(time_s, seg.excite{1}.time);
                
                seg.turn{2}.time = [455.2, 474.3]; [seg.turn{2}.indx] = SegmentTime2Index(time_s, seg.turn{2}.time);
                seg.turnaccel{2}.time = [451.5, 479.4]; [seg.turnaccel{2}.indx] = SegmentTime2Index(time_s, seg.turnaccel{2}.time);
                
                seg.excite{2}.time = [474.3, 495.5]; [seg.excite{2}.indx] = SegmentTime2Index(time_s, seg.excite{2}.time);
                
                seg.turn{3}.time = [513.2, 526.5]; [seg.turn{3}.indx] = SegmentTime2Index(time_s, seg.turn{3}.time);
                seg.turnaccel{3}.time = [490.5, 531.5]; [seg.turnaccel{3}.indx] = SegmentTime2Index(time_s, seg.turnaccel{3}.time);
                
                seg.excite{3}.time = [526.5, 546.6]; [seg.excite{3}.indx] = SegmentTime2Index(time_s, seg.excite{3}.time);
                
                seg.turn{4}.time = [546.6, 576.3]; [seg.turn{4}.indx] = SegmentTime2Index(time_s, seg.turn{4}.time);
                
                seg.approach.time = [569.8, 599]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time);
                seg.flare.time = [599, 609]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time);
                
                seg.flt.time = [seg.climb.time(1), seg.approach.time(2)]; [seg.flt.indx] = SegmentTime2Index(time_s, seg.flt.time);
                
            case {16, '16', 'flt16'}
                
                seg.climb.time = [516.1, 545]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.turn{1}.time = [530, 547.5]; [seg.turn{1}.indx] = SegmentTime2Index(time_s, seg.turn{1}.time);
                seg.turnaccel{1}.time = [seg.turn{1}.time(1), 555.5]; [seg.turnaccel{1}.indx] = SegmentTime2Index(time_s, seg.turnaccel{1}.time);
                
                seg.excite{1}.time = [547.5, 565.3]; [seg.excite{1}.indx] = SegmentTime2Index(time_s, seg.excite{1}.time);
                
                seg.turn{2}.time = [578.7, 592]; [seg.turn{2}.indx] = SegmentTime2Index(time_s, seg.turn{2}.time);
                seg.turnaccel{2}.time = [564, 600]; [seg.turnaccel{2}.indx] = SegmentTime2Index(time_s, seg.turnaccel{2}.time);
                
                seg.excite{2}.time = [592, 610.3]; [seg.excite{2}.indx] = SegmentTime2Index(time_s, seg.excite{2}.time);
                
                seg.turn{3}.time = [615.3, 630]; [seg.turn{3}.indx] = SegmentTime2Index(time_s, seg.turn{3}.time);
                seg.turnaccel{3}.time = [608.5, 632]; [seg.turnaccel{3}.indx] = SegmentTime2Index(time_s, seg.turnaccel{3}.time);
                
                seg.excite{3}.time = [624.2, 660.3]; [seg.excite{3}.indx] = SegmentTime2Index(time_s, seg.excite{3}.time);
                
                seg.turn{4}.time = [658, 680]; [seg.turn{4}.indx] = SegmentTime2Index(time_s, seg.turn{4}.time);
                
                seg.approach.time = [670.9, 692.1]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time);
                seg.flare.time = [692.1, 699.5]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time);
                
                seg.flt.time = [seg.climb.time(1), seg.approach.time(2)]; [seg.flt.indx] = SegmentTime2Index(time_s, seg.flt.time);
                
            case {17, '17', 'flt17'}
                
                seg.climb.time = [390, 415]; [seg.climb.indx] = SegmentTime2Index(time_s, seg.climb.time);
                seg.turn{1}.time = [405.7, 418]; [seg.turn{1}.indx] = SegmentTime2Index(time_s, seg.turn{1}.time);
                seg.turnaccel{1}.time = [seg.turn{1}.time(1), 440]; [seg.turnaccel{1}.indx] = SegmentTime2Index(time_s, seg.turnaccel{1}.time);
                
                seg.excite{1}.time = [419.1, 443.2]; [seg.excite{1}.indx] = SegmentTime2Index(time_s, seg.excite{1}.time);
                
                seg.turn{2}.time = [451, 465.4]; [seg.turn{2}.indx] = SegmentTime2Index(time_s, seg.turn{2}.time);
                seg.turnaccel{2}.time = [440, 480]; [seg.turnaccel{2}.indx] = SegmentTime2Index(time_s, seg.turnaccel{2}.time);
                
                seg.excite{2}.time = [465.4, 488.3]; [seg.excite{2}.indx] = SegmentTime2Index(time_s, seg.excite{2}.time);
                
                seg.turn{3}.time = [490, 510]; [seg.turn{3}.indx] = SegmentTime2Index(time_s, seg.turn{3}.time);
                seg.turnaccel{3}.time = [480, 515]; [seg.turnaccel{3}.indx] = SegmentTime2Index(time_s, seg.turnaccel{3}.time);
                
                seg.excite{3}.time = [502.7, 545]; [seg.excite{3}.indx] = SegmentTime2Index(time_s, seg.excite{3}.time);
                
                seg.turn{4}.time = [545, 560]; [seg.turn{4}.indx] = SegmentTime2Index(time_s, seg.turn{4}.time);
                
                seg.approach.time = [559.7, 568.9]; [seg.approach.indx] = SegmentTime2Index(time_s, seg.approach.time);
                seg.flare.time = [568.9, 600]; [seg.flare.indx] = SegmentTime2Index(time_s, seg.flare.time);
                
                seg.flt.time = [seg.climb.time(1), seg.approach.time(2)]; [seg.flt.indx] = SegmentTime2Index(time_s, seg.flt.time);
                
            otherwise
                disp('Configuration for aircraft unknown')
                
        end
                
    case {'thor'}
        switch lower(param.fltNum)
            case {120, '120', 'flt120'}
                iExcite = 1; seg.excite{iExcite}.time = [2476.358985 2495.698919]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 2; seg.excite{iExcite}.time = [2501.838861 2517.018664]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 3; seg.excite{iExcite}.time = [2528.018492 2544.938198]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 4; seg.excite{iExcite}.time = [2551.518079 2564.937833]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 5; seg.excite{iExcite}.time = [2577.837604 2602.977167]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                
            case {121, '121', 'flt121'}
                
                iExcite = 1; seg.excite{iExcite}.time = [647.645798 672.126781]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);
                iExcite = 2; seg.excite{iExcite}.time = [673.786847 692.787591]; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);

            otherwise
                disp('Configuration for aircraft unknown')
                
        end
        
    otherwise
        disp('Aircraft Name unknown');
        
end
end

function [indx] = SegmentTime2Index(time, timeSeg)
indx = find(time >= min(timeSeg), 1 ) : 1 : find(time <= max(timeSeg), 1, 'last' );
end
