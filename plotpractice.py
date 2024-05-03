import matplotlib.pyplot as plt
import numpy
import math
import inverted_pendulum
import PID_controller_object

  
# x axis values 
# plotting the points
x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 379, 380, 381, 382, 383, 384, 385, 386, 387, 388, 389, 390, 391, 392, 393, 394, 395, 396, 397, 398, 399, 400, 401, 402, 403, 404, 405, 406, 407, 408, 409, 410, 411, 412, 413, 414, 415, 416, 417, 418, 419, 420, 421, 422, 423, 424, 425, 426, 427, 428, 429, 430, 431, 432, 433, 434, 435, 436, 437, 438, 439, 440, 441, 442, 443, 444, 445, 446, 447, 448, 449, 450, 451, 452, 453, 454, 455, 456, 457, 458, 459, 460, 461, 462, 463, 464, 465, 466, 467, 468, 469, 470, 471, 472, 473, 474, 475, 476, 477, 478, 479, 480, 481, 482, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 493, 494, 495, 496, 497, 498, 499, 500, 501, 502, 503, 504, 505, 506, 507, 508, 509, 510, 511, 512, 513, 514, 515, 516, 517, 518, 519, 520, 521, 522, 523, 524, 525, 526, 527, 528, 529, 530, 531, 532, 533, 534, 535, 536, 537, 538, 539, 540, 541, 542, 543, 544, 545, 546, 547, 548, 549, 550, 551, 552, 553, 554, 555, 556, 557, 558, 559, 560, 561, 562, 563, 564, 565, 566, 567, 568, 569, 570, 571, 572, 573, 574, 575, 576, 577, 578, 579, 580, 581, 582, 583, 584, 585, 586, 587, 588, 589, 590, 591, 592, 593, 594, 595, 596, 597, 598, 599, 600, 601, 602, 603, 604, 605, 606, 607, 608, 609, 610, 611, 612, 613, 614, 615, 616, 617, 618, 619, 620, 621, 622, 623, 624, 625, 626, 627, 628, 629, 630, 631, 632, 633, 634, 635, 636, 637, 638, 639, 640, 641, 642, 643, 644, 645, 646, 647, 648, 649, 650, 651, 652, 653, 654, 655, 656, 657, 658, 659, 660, 661, 662, 663, 664, 665, 666, 667, 668, 669, 670, 671, 672, 673, 674, 675, 676, 677, 678, 679, 680, 681, 682, 683, 684, 685, 686, 687, 688, 689, 690, 691, 692, 693, 694, 695, 696, 697, 698, 699, 700, 701, 702, 703, 704, 705, 706, 707, 708, 709, 710, 711, 712, 713, 714, 715, 716, 717, 718, 719, 720, 721, 722, 723, 724, 725, 726, 727, 728, 729, 730, 731, 732, 733, 734, 735, 736, 737, 738, 739, 740, 741, 742, 743, 744, 745, 746, 747, 748, 749, 750, 751, 752, 753, 754, 755, 756, 757, 758, 759, 760, 761, 762, 763, 764, 765, 766, 767, 768, 769, 770, 771, 772, 773, 774, 775, 776, 777, 778, 779, 780, 781, 782, 783, 784, 785, 786, 787, 788, 789, 790, 791, 792, 793, 794, 795, 796, 797, 798, 799, 800, 801, 802, 803, 804, 805, 806, 807, 808, 809, 810, 811, 812, 813, 814, 815, 816, 817, 818, 819, 820, 821, 822, 823, 824, 825, 826, 827, 828, 829, 830, 831, 832, 833, 834, 835, 836, 837, 838, 839, 840, 841, 842, 843, 844, 845, 846, 847, 848, 849, 850, 851, 852, 853, 854, 855, 856, 857, 858, 859, 860, 861, 862, 863, 864, 865, 866, 867, 868, 869, 870, 871, 872, 873, 874, 875, 876, 877, 878, 879, 880, 881, 882, 883, 884, 885, 886, 887, 888, 889, 890, 891, 892, 893, 894, 895, 896, 897, 898, 899, 900, 901, 902, 903, 904, 905, 906, 907, 908, 909, 910, 911, 912, 913, 914, 915, 916, 917, 918, 919, 920, 921, 922, 923, 924, 925, 926, 927, 928, 929, 930, 931, 932, 933, 934, 935, 936, 937, 938, 939, 940, 941, 942, 943, 944, 945, 946, 947, 948, 949, 950, 951, 952, 953, 954, 955, 956, 957, 958, 959, 960, 961, 962, 963, 964, 965, 966, 967, 968, 969, 970, 971, 972, 973, 974, 975, 976, 977, 978, 979, 980, 981, 982, 983, 984, 985, 986, 987, 988, 989, 990, 991, 992, 993, 994, 995, 996, 997, 998, 999, 1000]
y = [0.009011084014354973, 0.009029441219150697, 0.009042940555040407, 0.009051571748272282, 0.009055327319789303, 0.009054202590979464, 0.009048195687820195, 0.009037307543413794, 0.009021541898911623, 0.009000905302825659, 0.008975407108726983, 0.008945059471331653, 0.008909877340975328, 0.008869878456478967, 0.008825083336408789, 0.008775515268734636, 0.008721200298891769, 0.008662167216252055, 0.008598447539011398, 0.00853007549750118, 0.008457088015932384, 0.008379524692581946, 0.008297427778431832, 0.00821084215427214, 0.008119815306280486, 0.008024397300090788, 0.007924640753365401, 0.007820600806885477, 0.007712335094175225, 0.007599903709676637, 0.00748336917549207, 0.007362796406712902, 0.007238252675353311, 0.007109807572909042, 0.006977532971561818, 0.006841502984050854, 0.006701793922233718, 0.006558484254359534, 0.006411654561078301, 0.006261387490210838, 0.006107767710304591, 0.005950881863001274, 0.005790818514242995, 0.005627668104344246, 0.0054615228969577614, 0.005292476926962964, 0.005120625947306317, 0.0049460673748235595, 0.004768900235074395, 0.004589225106220817, 0.004407144061980807, 0.0042227606136897335, 0.004036179651502289, 0.003847507384768349, 0.003656851281616632, 0.003464320007780522, 0.003270023364700889, 0.0030740722269411787, 0.002876578478950476, 0.002677654951210644, 0.0024774153558040305, 0.0022759742214385945, 0.0020734468279676407, 0.0018699491404416788, 0.0016655977427302152, 0.00146050977075156, 0.0012548028453489882, 0.0010485950048518228, 0.0008420046373602113, 0.0006351504127925605, 0.00042815121473474354, 0.00022112607213034319, 1.4194090851301722e-05, 0.00019252561481155605, 0.00039891399069859443, 0.0006048521113324555, 0.0008102212484305673, 0.0010149029393394752, 0.0012187790553514289, 0.0014217318698637057, 0.0016236441263412253, 0.0018243991060431066, 0.002023880695473942, 0.002221973453520703, 0.002418562678236364, 0.0026135344732315137, 0.0028067758136354473, 0.002998174611588462, 0.0031876197812273346, 0.0033750013031262527, 0.003560210288155762, 0.0037431390407226283, 0.003923681121353851, 0.004101731408588445, 0.004277186160140996, 0.0044499430733013845, 0.004619901344535557, 0.004786961728252612, 0.004951026594703996, 0.005111999986981065, 0.005269787677077801, 0.005424297220985976, 0.005575438012790649, 0.005723121337734409, 0.005867260424219393, 0.006007770494716694, 0.006144568815553411, 0.006277574745548229, 0.00640670978346706, 0.006531897614270963, 0.006653064154129247, 0.0067701375941713535, 0.006883048442951845, 0.006991729567603544, 0.007096116233654634, 0.007196146143486264, 0.007291759473408004, 0.00738289890932925, 0.00746950968100551, 0.0075515395948392745, 0.007628939065216026, 0.007701661144356751, 0.007769661550669178, 0.0078328986955808, 0.0078913337088376, 0.007944930462253297, 0.00799365559189475, 0.008037478518690118, 0.008076371467447182, 0.008110309484270228, 0.008139270452364704, 0.008163235106219855, 0.008182187044160405, 0.008196112739259303, 0.008205001548604478, 0.008208845720913467, 0.008207640402490737, 0.00820138364152343, 0.008190076390712227, 0.008173722508234988, 0.008152328757041695, 0.00812590480248028, 0.00809446320825377, 0.0080580194307102, 0.008016591811467638, 0.007970201568377668, 0.007918872784831568, 0.00786263239741441, 0.007801510181913226, 0.007735538737686334, 0.007664753470401847, 0.007589192573154331, 0.007508897005969509, 0.007423910473707809, 0.007334279402378509, 0.007240052913877123, 0.007141282799159581, 0.00703802348986767, 0.006930332028421083, 0.006818268036592327, 0.006701893682581595, 0.006581273646609613, 0.0064564750850473, 0.006327567593101942, 0.00619462316608045, 0.0060577161592510545, 0.005916923246325647, 0.005772323376585779, 0.0056239977306761215, 0.005472029675089968, 0.005316504715372158, 0.005157510448065526, 0.004995136511427749, 0.004829474534946182, 0.004660618087679003, 0.004488662625451656, 0.004313705436938304, 0.004135845588658652, 0.0039551838689211374, 0.0037718227307441538, 0.003585866233787556, 0.0033974199853273198, 0.0032065910803067855, 0.003013488040498501, 0.0028182207528112, 0.0026209004067769812, 0.0024216394312542707, 0.0022205514303826022, 0.0020177511188257417, 0.001813354256340102, 0.0016074775817058237, 0.001400238746058289, 0.0011917562456582173, 0.0009821493541388366, 0.0007715380542689617, 0.0005600429692711153, 0.00034778529373411177, 0.00013488672415978842, 7.853061081619758e-05, 0.00029234422048936997, 0.000506431322396907, 0.0007206689128330951, 0.0009349338375291284, 0.0011491028624565007, 0.0013630527447131427, 0.0015766603034513868, 0.0017898024908067996, 0.0020023564627868968, 0.002214199650078763, 0.002425209828734624, 0.0026352651906944766, 0.00284424441410495, 0.0030520267333936863, 0.0032584920090586384, 0.003463520797131851, 0.003666994418277448, 0.003868795026483761, 0.0040688056773097465, 0.004266910395646094, 0.004462994242951683, 0.004656943383926362, 0.004848645152581311, 0.005037988117668601, 0.005224862147431945, 0.005729870071108487, 0.006232192833356287, 0.006731540091468688, 0.007227623057713273, 0.007720154676730293, 0.008208849791865817, 0.008693425310472569, 0.00917360036808119, 0.009649096491345361, 0.010119637759665085, 0.01058495096539323, 0.011044765772531441, 0.011498814873822445, 0.011946834146146872, 0.012388562804133812, 0.012823743551895467, 0.013252122732797512, 0.013673450477178021, 0.014087480847929143, 0.014493971983857077, 0.014892686240737339, 0.01528339032998374, 0.015665855454851028, 0.016039857444092677, 0.016405176882996936, 0.016761599241725815, 0.017108915000883424, 0.01744691977424173, 0.017775414428553557, 0.018094205200384395, 0.018403103809896442, 0.018701927571520054, 0.018990499501449664, 0.01926864842190314, 0.019536209062085384, 0.019793022155798982, 0.02003893453564657, 0.02027379922377162, 0.020497475519086327, 0.020709829080937183, 0.020910732009161007, 0.021100062920486065, 0.021277707021235066, 0.021443556176288826, 0.021597508974271495, 0.021739470788920277, 0.021869353836604703, 0.021987077229962595, 0.02209256702762197, 0.02218575627998026, 0.02226658507101429, 0.022335000556096704, 0.022390956995796497, 0.02243441578564362, 0.022465345481839618, 0.022483721822898567, 0.02248952774720455, 0.02248275340647426, 0.022463396175115315, 0.02243146065547314, 0.022386958678961427, 0.022329909303073287, 0.02226033880427247, 0.02217828066676616, 0.022083775567163042, 0.02197687135502249, 0.021857623029302976, 0.021726092710719916, 0.021582349610025385, 0.02142646999222431, 0.021258537136743964, 0.0210786412935757, 0.021222854846007326, 0.021355306660921447, 0.021475914546457035, 0.021584603064454498, 0.02168130358762412, 0.02176595434223658, 0.02183850044692117, 0.021898893947548802, 0.021947093848179198, 0.021983066138053592, 0.02200678381461648, 0.022018226902552014, 0.02235405263181066, 0.022677584709413796, 0.022988630888224396, 0.023287005992015584, 0.02357253203543209, 0.02384503832923747, 0.024104361581420816, 0.024350345994107007, 0.024582843356216837, 0.02480171313182576, 0.02500682254417242, 0.025198046655270437, 0.0253752684410794, 0.02553837886219342, 0.02568727693000792, 0.02582186976832788, 0.025942072670383107, 0.026047809151218473, 0.02613901099542965, 0.02621561830021709, 0.026277579513733638, 0.026324851468703413, 0.026357399411292195, 0.026375197025211877, 0.02637822645104401, 0.026366478300769956, 0.026339951667497585, 0.026298654130376846, 0.02624260175469909, 0.026171819087177407, 0.026086339146407744, 0.025986203408512962, 0.02621193581084574, 0.026423120560446222, 0.026619628856768197, 0.026801340253720593, 0.026968142743195773, 0.027119932823072766, 0.027256615560408988, 0.027378104649785462, 0.027484322466773122, 0.027575200116490284, 0.027650677477223908, 0.027710703239089807, 0.02775523493770948, 0.027784238982883794, 0.027797690682246256, 0.027795574259881176, 0.02777788286989452, 0.027744618604927846, 0.027695792499608178, 0.02763142452892932, 0.027551543601562537, 0.027456187548097217, 0.027345403104214523, 0.02721924588879977, 0.02707778037700169, 0.026921079868249386, 0.02674922644924034, 0.026562310951915382, 0.02636043290643918, 0.026143700489207333, 0.025912230465903765, 0.02566614812963477, 0.025405587234168574, 0.025130689922311975, 0.02484160664945819, 0.024538496102342704, 0.024221525113046465, 0.02389086856828851, 0.023546709314052637, 0.02318923805559545, 0.022818653252885644, 0.022435161011527177, 0.02203897496922142, 0.02163031617782619, 0.02120941298107205, 0.02077650088799894, 0.020331822442178838, 0.019875627086792625, 0.019408171025632102, 0.018929717080100446, 0.018440534542287147, 0.017940899024195846, 0.017431092303206094, 0.016911402163852472, 0.01638212223600699, 0.0158435518295531, 0.015295995765642034, 0.014739764204624525, 0.014175172470753295, 0.013602540873753927, 0.013022194527363955, 0.012434463164942235, 0.011839680952252653, 0.01123818629752841, 0.010630321658925038, 0.010016433349472253, 0.009396871339636658, 0.008771989057609036, 0.00814214318743178, 0.007507693465083599, 0.006869002472640268, 0.006226435430631611, 0.005580359988716418, 0.00493114601479824, 0.004279165382706285, 0.003624791758566781, 0.0029684003859912402, 0.0023103678702089954, 0.001651071961272278, 0.0009908913364628622, 0.0003302053820299725, 0.0003306060256102703, 0.0009911627390831227, 0.0016510845595400971, 0.0023099914562890672, 0.0029675037864380534, 0.003623242514421086, 0.004276829431274583, 0.004927887373532821, 0.005576040441611333, 0.006220914217547422, 0.006862135981967399, 0.007499334930150762, 0.008132142387062122, 0.008760192021222486, 0.009383120057292298, 0.010000565487239646, 0.010612170279968004, 0.011217579589279054, 0.011816441960047334, 0.012408409532484727, 0.012993138244374253, 0.013570288031154008, 0.014139523023733714, 0.014700511743927915, 0.015252927297391559, 0.015796447563945486, 0.01633075538518114, 0.01685553874923574, 0.017370490972631103, 0.017875310879071314, 0.018369702975096498, 0.01885337762249209, 0.019326051207355168, 0.019787446305721598, 0.02023729184566004, 0.020675323265741136, 0.02110128266979252, 0.021514918977852692, 0.021915988073239197, 0.02230425294564891, 0.022679483830210793, 0.023041458342413845, 0.023389961608835556, 0.023724786393598648, 0.024045733220486407, 0.024352610490649497, 0.024645234595839647, 0.024923430027108225, 0.025187029478910255, 0.02543587394855703, 0.025669812830963045, 0.025888704008635615, 0.02609241393685807, 0.026280817724020085, 0.026453799207051275, 0.026611251021916783, 0.026753074669136206, 0.026879180574289813, 0.026989488143478586, 0.027083925813707267, 0.02716243109816214, 0.027224950626357915, 0.027271440179130694, 0.02730186471845657, 0.027316198412078035, 0.02731442465292296, 0.027296536073303554, 0.02726253455388526, 0.027212431227418187, 0.027146246477226323, 0.02706400993045227, 0.026965760446058033, 0.02685154609758484, 0.026721424150677762, 0.02657546103538338, 0.026413732313231495, 0.026236322639114507, 0.026043325717980646, 0.02583484425636005, 0.025610989908745205, 0.025371883218850026, 0.0251176535557745, 0.024848439045104503, 0.02456438649497911, 0.024265651317160374, 0.02395239744314334, 0.023624797235346672, 0.023283031393427085, 0.022633935324506884, 0.021971058941979454, 0.02129477588868936, 0.020605467744804352, 0.019903523804433287, 0.01918934085681677, 0.018463322962944662, 0.01772588122772175, 0.0169774335678064, 0.01621840447525029, 0.015449224777070754, 0.014670331390890412, 0.013882167076782045, 0.01308518018545975, 0.012279824402960465, 0.011466558491962901, 0.010645846029893795, 0.009818155143974163, 0.008983958243360872, 0.008143731748541433, 0.007297955818142339, 0.006447114073313564, 0.0055916933198540825, 0.004732183268245258, 0.0038690762517609215, 0.0030028669428247153, 0.0021340520677869015, 0.001263130120294345, 0.000390601073428684, 0.0004830339092111092, 0.0013572727633045668, 0.0022316128143058944, 0.003105551068461717, 0.0039785845041711175, 0.0048502103635083334, 0.005719926443728273, 0.006587231388575004, 0.007451624979213495, 0.008312608424605216, 0.009169684651148665, 0.010022358591406535, 0.010870137471742043, 0.011712531098687909, 0.01254905214387256, 0.013379216427329473, 0.01420254319901694, 0.015018555418377138, 0.015826780031765117, 0.01662674824758015, 0.01741799580893392, 0.018200063263692074, 0.01897249623172805, 0.019734845669230238, 0.020486668129906297, 0.021227526022930734, 0.021956987867484807, 0.0226746285437404, 0.023380029540142507, 0.02407277919684788, 0.024752472945180407, 0.025418713542966924, 0.02607111130562033, 0.02670928433284005, 0.02733285873080324, 0.02794146882972336, 0.028534757396656202, 0.029112375843436754, 0.029673984429633756, 0.030219252460412255, 0.030747858479197875, 0.03125949045504003, 0.03175384596457476, 0.03223063236849141, 0.0326895669824107, 0.033130377242085396, 0.033552800862838156, 0.033956585993154514, 0.03434149136235252, 0.0347072864222539, 0.03505375148278504, 0.03538067784143934, 0.03568786790653603, 0.035975135314213716, 0.03624230503910023, 0.03648921349860372, 0.036715708650773005, 0.03692165008567863, 0.03745729909866983, 0.03797214786415216, 0.0384658905060999, 0.03893823310943897, 0.03938889390237128, 0.03981760342075445, 0.04022410466551799, 0.04060815325304155, 0.04096951755842443, 0.04130797885157948, 0.04162333142608801, 0.04191538272075613, 0.04218395343381655, 0.0424288776297232, 0.042650002838489715, 0.04284719014752606, 0.04302031428593112, 0.04316926370120223, 0.043293940628326, 0.043394261151217965, 0.04347015525648179, 0.04352156687946193, 0.04354845394256674, 0.04355078838584216, 0.04352855618977916, 0.04348175739034126, 0.04341040608620131, 0.04331453043818001, 0.04319417266088155, 0.043049389006524795, 0.042880249740971654, 0.042686839111957135, 0.04282534209799182, 0.042939784116654986, 0.04303008722688599, 0.04309618725836728, 0.04313803386552884, 0.04315559056265095, 0.043148834751282804, 0.04311775773996152, 0.04306236475621889, 0.042982674950866426, 0.042878721394552274, 0.0427505510665866, 0.04259822483603511, 0.042421817435083456, 0.042221417424678415, 0.041997127152454766, 0.04174906270296001, 0.04147735384019217, 0.04118214394246923, 0.04086358992965203, 0.04052186218274557, 0.040157144455907325, 0.03976963378089432, 0.03935954036398433, 0.038927087475410085, 0.03847251133134894, 0.03799606096851409, 0.03749799811139718, 0.03697859703221603, 0.03643814440362481, 0.035876939144248214, 0.03529529225710497, 0.03469352666099018, 0.03407197701489006, 0.0334309895355068, 0.03277092180797566, 0.03209214258986043, 0.03139503160851802, 0.03067997935192716, 0.029947386853080647, 0.029197665468045066, 0.02843123664779643, 0.027648531703944636, 0.02684999156846422, 0.026036066547553447, 0.025207216069748243, 0.02436390842842211, 0.023506620518807617, 0.02263583756967958, 0.021752052869844517, 0.02085576748958535, 0.01994748999721474, 0.019027736170894715, 0.018097028705884474, 0.01715589691738243, 0.01620487643913262, 0.015244508917969496, 0.014275341704479073, 0.013297927539958013, 0.012312824239855922, 0.011320594373889497, 0.010321804943020565, 0.009317027053493096, 0.008306835588127347, 0.0072918088750720325, 0.006272528354218075, 0.005249578241479913, 0.0042235451911525786, 0.0031950179565548046, 0.002164587049170226, 0.001132844396500376, 0.00010038299884456412, 0.0009322034147771076, 0.0019643207313403766, 0.0029953747989551706, 0.0040247717722072494, 0.0050519184574844625, 0.0060762226580689, 0.0070970935187764286, 0.008113941869925537, 0.009126180570418066, 0.010133224849715298, 0.011134492648493989, 0.01212940495776825, 0.013117386156264758, 0.014097864345840492, 0.01507027168473422, 0.016034044718445068, 0.016988624708033896, 0.01793345795564575, 0.018867996127054386, 0.019791696571032746, 0.020704022635356353, 0.02160444397924978, 0.022492436882089732, 0.023072326571566888, 0.023638762427658857, 0.024191411994879838, 0.024729950598760184, 0.025254061525926495, 0.025763436208933597, 0.026257774406425915, 0.026736784378529818, 0.027200183057381696, 0.02764769621269958, 0.02807905861230926, 0.028494014177538957, 0.02889231613339979, 0.029273727153472314, 0.02963801949942266, 0.029984975155074825, 0.030314385954968832, 0.030626053707337626, 0.030919790311438586, 0.031195417869178715, 0.031452768790975597, 0.0316916858957993, 0.031912022505343446, 0.03211364253227671, 0.03229642056252904, 0.0324602419315699, 0.03260500279463883, 0.032730610190891546, 0.03283698210142792, 0.03292404750117093, 0.03299174640456877, 0.033040029905095254, 0.03306886020852637, 0.03307821065997403, 0.033068065764660814, 0.03303842120242241, 0.032989283835927447, 0.03292067171260724, 0.03283261406029098, 0.03272515127654461, 0.03259833491171489, 0.03245222764568267, 0.032286903258332646, 0.032102446593749716, 0.031898953518154936, 0.031676530871597185, 0.03143529641341956, 0.03117537876152253, 0.03089691732544898, 0.030600062233319212, 0.03028497425264717, 0.029951824705072155, 0.029600795375043454, 0.02923207841249843, 0.02884587622957783, 0.02844240139142518, 0.02802187650112041, 0.02758453407880106, 0.027130616435027663, 0.026660375538453206, 0.026174072877859823, 0.025671979318629218, 0.02515437495371662, 0.024621548949201426, 0.024073799384491015, 0.023511433087257613, 0.022934765463191384, 0.022344120320656358, 0.02173982969033911, 0.02112223363998348, 0.020491680084307972, 0.019848524590205772, 0.01919313017733069, 0.018525867114175518, 0.017847112709752675, 0.01715725110099013, 0.01645667303595884, 0.015745775653051054, 0.015024962256231924, 0.014294642086489924, 0.013555230089614536, 0.01280714668043256, 0.012050817503637281, 0.011286673191347471, 0.010515149117535857, 0.009736685149469342, 0.0089517253963057, 0.007854161792904261, 0.006751001806963627, 0.0056428771401295045, 0.0045304227198917175, 0.0034142763224463044, 0.002295078203234923, 0.0011734707257123264, 5.0097988578072516e-05, 0.0010743945482896095, 0.002199360438958391, 0.0033241526273842283, 0.004448123823929982, 0.0055706268822180265, 0.006691015176074179, 0.007808642976320201, 0.00892286582717221, 0.01003304092200283, 0.011138527478225576, 0.012238687111060936, 0.013332884205944891, 0.014420486289342067, 0.015500864397727541, 0.016573393444503273, 0.017637452584617457, 0.018692425576657564, 0.019737701142190596, 0.020772673322126983, 0.02179674182988784, 0.022809312401158542, 0.023809797140015236, 0.024797614861214633, 0.02577219142844136, 0.02673296008831118, 0.02767936179993275, 0.028610845559834758, 0.02952686872206997, 0.0304268973133122, 0.03131040634276692, 0.032176880106721054, 0.033025812487562374, 0.0338567072471037, 0.034669078314052276, 0.03546245006546952, 0.03623635760207146, 0.03699034701722518, 0.037723975659501655, 0.03843681238865037, 0.03912843782486612, 0.039798444591223434, 0.04044643754915888, 0.041072034026886604, 0.04167486404063707, 0.04225457050861396, 0.042810809457568794, 0.0433432502218974, 0.04385157563516722, 0.044335482213988475, 0.044794680334147054, 0.04522889439892096, 0.04563786299950648, 0.04602133906748447, 0.04637909001926091, 0.04671089789242007, 0.04701655947393234, 0.04729588642016247, 0.04754870536862779, 0.04777485804145936, 0.047974201340522604, 0.0481466074341574, 0.048291963835500815, 0.048410173472359104, 0.04850115474859863, 0.04856484159702868, 0.04860118352375208, 0.048610145643962724, 0.04859170870917204, 0.04854586912584949, 0.04847263896546514, 0.0483720459659253, 0.04824413352439522, 0.048088960681505796, 0.047906602096944174, 0.04769714801643126, 0.04746070423009204, 0.04719739202222782, 0.04690734811250249, 0.04659072458855824, 0.04624768883007923, 0.045878423424325215, 0.04548312607316025, 0.04506200949160534, 0.044615301297947295, 0.044143243895439596, 0.04364609434563511, 0.043124124233393964, 0.04257761952361418, 0.04200688040973632, 0.04141222115407803, 0.040793969920058085, 0.04015246859637439, 0.039488072613204514, 0.03880115075050212, 0.038092084938467254, 0.03736127005027327, 0.03660911368713797, 0.03583603595583172, 0.03504246923872012, 0.034228857956444106, 0.03339565832334562, 0.03254333809575211, 0.03167237631323863, 0.03078326303299167, 0.02987649905740428, 0.02895259565503751, 0.028012074275088745, 0.02705546625551294, 0.026083312524948398, 0.025096163298604028, 0.024094577768270703, 0.02307912378662458, 0.022050377545995745, 0.021008923251780737, 0.019955352790682862, 0.01889026539396918, 0.017814267295938227, 0.016727971387797302, 0.015631996867153025, 0.01452696888332339, 0.013413518178684069, 0.012292280726265984, 0.011163897363825222, 0.010029013424610315, 0.008888278365055565, 0.007742345389632547, 0.006591871073095194, 0.005437514980356797, 0.004279939284240047, 0.003119808381343684, 0.0019577885062715366, 0.0007945473444716852, 0.00036924635606489105, 0.001532923173995434, 0.0026958134044544778, 0.003857247449268264, 0.005016556207301413, 0.006173071464682563, 0.007326126284655792, 0.008475055396805058, 0.009619195585399563, 0.010757886076608948, 0.01189046892433852, 0.013016289394436251, 0.014134696347025128, 0.01524504261671659, 0.016346685390463127, 0.017438986582810758, 0.018521313208315033, 0.019593037750887294, 0.02065353852984129, 0.02170220006241387, 0.022738413422537178, 0.023761576595643857, 0.024771094829290848, 0.02576638097939176, 0.02674685585185227, 0.02771194853940763, 0.028661096753466172, 0.029593747150767562, 0.03050935565466953, 0.03140738777088193, 0.0322873188974721, 0.033148634628970726, 0.03399083105441273, 0.03481341504915287, 0.03561590456030122, 0.036397828885628994, 0.0371587289458004, 0.03789815754979171, 0.038615679653363856, 0.0393108726104603, 0.03998332641740689, 0.040632643949795905, 0.0412584411919412, 0.04186034745879682, 0.04243800561023594, 0.0429910722575922, 0.04351921796237015, 0.04402212742703608, 0.04449949967780534, 0.04495104823934641]

plt.plot(x, y) 
  
# naming the x axis 
plt.xlabel('Timestep') 
# naming the y axis 
plt.ylabel('Theta (radians)') 
  
# giving a title to my graph 
plt.title('PID Controller with disturbances -- Angle Theta of the Pole vs Timestep') 
  
# function to show the plot 
plt.show() 