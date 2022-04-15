/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef TEMP_MEAS_H_INCLUDED
#define TEMP_MEAS_H_INCLUDED

#include "my_fp.h"

class TempMeas
{
public:
    enum Sensors
    {
        TEMP_JCURVE = 0,
        TEMP_SEMIKRON = 1,
        TEMP_MBB600 = 2,
        TEMP_KTY81HS = 3,
        TEMP_PT1000 = 4,
        TEMP_NTCK45 = 5, /*hier muss ein NTC K45 2k2 verwendet werden, der mit einem Parallelwiderstand mit 2k verschaltet wird. Achtung: Der Parallelwiderstand ist im Schaltplan und Layout nicht vorhanden! */
        TEMP_LEAFHS = 6,
        NUM_HS_SENSORS = 7,
        TEMP_KTY83 = 12,
        TEMP_KTY84 = 13,
        TEMP_LEAF = 14,
        TEMP_KTY81M = 15,
        TEMP_TOYOTA = 16,
        TEMP_TESLA_100K = 17,
        TEMP_TESLA_52K = 18,
        TEMP_TESLA_LDU_FLUID = 19,
        TEMP_TESLA_10K = 20,
        TEMP_LAST
    };

    static float Lookup(int digit, Sensors sensorId);
};


#ifdef __TEMP_LU_TABLES
#define JCURVE \
57	,\
76	,\
100	,\
132	,\
171	,\
220	,\
280	,\
353	,\
440	,\
544	,\
665	,\
805	,\
963	,\
1141	,\
1338	,\
1551	,\
1779	,\
2019	,\
2268	,\
2523	,\
2779	,\
3032	,\
3279	,\
3519	,\
3748	,\
3964	,\
4167

#define MBB600 \
3971	,\
3632	,\
3292	,\
2959	,\
2641	,\
2343	,\
2069	,\
1819	,\
1596	,\
1397	,\
1222	,\
1069	,\
935	,\
819	,\
718	,\
631	,\
555	,\
489	,\
433	,\
383	,\
340	,\
303

#define LEAF \
49	,\
68	,\
91	,\
119	,\
153	,\
193	,\
239	,\
290	,\
348	,\
411	,\
479	,\
551	,\
628	,\
707	,\
788	,\
871	,\
955	,\
1039

#define LEAFHS \
1061	, \
1175	, \
1388	, \
1550	, \
1707	, \
1876	, \
1993	, \
2156	, \
2320	, \
2483	, \
2647	, \
2810

#define SEMIKRON \
3767	,\
3685	,\
3605	,\
3528	,\
3456	,\
3385	,\
3317	,\
3254	,\
3191	,\
3130	,\
3074	,\
3018	,\
2963	,\
2913	,\
2862	,\
2813	,\
2767	,\
2722	,\
2678	,\
2636	,\
2595


#define KTY83 \
2035	,\
1968	,\
1901	,\
1835	,\
1769	,\
1705	,\
1643	,\
1582	,\
1522	,\
1465	,\
1409	,\
1356	,\
1304	,\
1255	,\
1208	,\
1162	,\
1119	,\
1077	,\
1037	,\
999	,\
962	,\
927	,\
894	,\

#define KTY84 \
2283	,\
2231	,\
2179	,\
2125	,\
2071	,\
2018	,\
1963	,\
1936	,\
1908	,\
1856	,\
1802	,\
1750	,\
1699	,\
1649	,\
1600	,\
1552	,\
1505	,\
1459	,\
1414	,\
1372	,\
1329	,\
1289	,\
1250	,\
1212	,\
1175	,\
1140	,\
1105	,\
1073	,\
1041	,\
1010	,\
980	,\
952	,\
924	,\
899	,\
876	,\
856

#define TESLA_100K \
2969	,\
2692	,\
2423	,\
2172	,\
1935	,\
1719	,\
1524	,\
1345	,\
1186	,\
1043	,\
917	,\
805	,\
707	,\
619	,\
544	,\
479	,\
420	,\
370	,\
325	,\
287	,\
253	,\
223	,\
197	,\
175	,\
155	,\
137	,\
122	,\
109	,\
97	,\
86	,\
77	,\
69	,\
62	,\
55	,\
50	,\
45	,\
40	,\
36	,\
32	,\
29	,\
27	,\
24	,\
22	,\
20	,\
18

#define TESLA_52K \
3416, \
2789, \
1882, \
1109, \
902, \
670, \
504, \
468, \
432, \
395, \
358

#define TESLA_LDU_FLUID \
3855	,\
3435	,\
3105	,\
2723	,\
2414	,\
2178	,\
1836	,\
1609	,\
1408	,\
1180	,\
994	,\
868	,\
749	,\
638	,\
574	,\
498	,\
431	,\
373	,\
314	,\
277

#define TESLA_10K \
2610	,\
2161	,\
1766	,\
1431	,\
1152	,\
925	,\
742	,\
523	,\
480	,\
388	,\
315	,\
257	,\
210	,\
173	,\
143	,\
119	,\
99	,\
83	,\
70	,\
60	,\
51	,\
43	,\
37	,\
32	,\
28	,\
24	,\
21	,\
18	,\
16	,\
14	,\
13	,\
11	,\
10	,\
9	,\
8	,\
7	,\
6	,\
6	,\
5	,\
5	,\
4	,\
4	,\
3	,\
3	,\
3

#define KTY81_HS \
1863	,\
1991	,\
2123	,\
2253	,\
2381	,\
2510	,\
2635	,\
2760	,\
2881	,\
2998	,\
3114	,\
3226	,\
3332	,\
3437	,\
3537	,\
3634	,\
3727	,\
3815	,\
3895	,\
3965	,\
4022


#define PT1000 \
2488, \
2560, \
2629, \
2696, \
2760, \
2821, \
2880, \
2937, \
2991, \
3044, \
3095, \
3144, \
3192, \
3238, \
3282, \
3325, \
3367, \
3407, \
3446, \
3484, \
3521

#define NTCK45 \
3855, \
3846, \
3833, \
3817, \
3796, \
3769, \
3735, \
3692, \
3639, \
3575, \
3498, \
3406, \
3300, \
3179, \
3043, \
2893, \
2731, \
2560, \
2382, \
2201, \
2021, \
1843, \
1672, \
1509, \
1356, \
1214, \
1084, \
966, \
859, \
763, \
677, \
601, \
534, \
474, \
422, \
375, \
336, \
301, \
270, \
243, \
219

#define KTY81_M \
2062	,\
1994	,\
1925	,\
1857	,\
1791	,\
1725	,\
1661	,\
1598	,\
1536	,\
1478	,\
1420	,\
1365	,\
1312	,\
1261	,\
1212	,\
1165	,\
1120	,\
1078	,\
1040	,\
1007	,\
980	,\

#define TOYOTA_M \
4009	,\
4007	,\
4004	,\
4000	,\
3996	,\
3990	,\
3983	,\
3974	,\
3963	,\
3950	,\
3935	,\
3917	,\
3895	,\
3870	,\
3841	,\
3808	,\
3770	,\
3726	,\
3678	,\
3624	,\
3564	,\
3499	,\
3427	,\
3350	,\
3267	,\
3180	,\
3087	,\
2990	,\
2890	,\
2786	,\
2680	,\
2572	,\
2464	,\
2355	,\
2247	,\
2140	,\
2034	,\
1931	,\
1831	,\
1733	,\
1640	,\
1549	,\
1463	,\
1380	,\
1302


#endif

#endif // TEMP_MEAS_H_INCLUDED
