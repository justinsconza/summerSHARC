#define N 1010

#define FILTER_SIZE 64

double coeffs[64] = {

	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625,
	0.015625

};

double x[1010] = {

	0.000000,
	0.099178,
	0.198089,
	0.296469,
	0.394053,
	0.490580,
	0.585791,
	0.679433,
	0.771254,
	0.861011,
	0.948465,
	1.033385,
	1.115545,
	1.194730,
	1.270733,
	1.343356,
	1.412409,
	1.477717,
	1.539111,
	1.596436,
	1.649549,
	1.698320,
	1.742628,
	1.782369,
	1.817451,
	1.847793,
	1.873332,
	1.894016,
	1.909806,
	1.920681,
	1.926630,
	1.927659,
	1.923785,
	1.915042,
	1.901476,
	1.883146,
	1.860126,
	1.832502,
	1.800374,
	1.763851,
	1.723059,
	1.678131,
	1.629214,
	1.576466,
	1.520052,
	1.460150,
	1.396945,
	1.330631,
	1.261411,
	1.189493,
	1.115092,
	1.038431,
	0.959736,
	0.879238,
	0.797170,
	0.713771,
	0.629281,
	0.543941,
	0.457992,
	0.371676,
	0.285236,
	0.198909,
	0.112935,
	0.027547,
	-0.057024,
	-0.140550,
	-0.222811,
	-0.303590,
	-0.382677,
	-0.459870,
	-0.534972,
	-0.607797,
	-0.678165,
	-0.745906,
	-0.810860,
	-0.872876,
	-0.931813,
	-0.987541,
	-1.039943,
	-1.088909,
	-1.134345,
	-1.176166,
	-1.214298,
	-1.248682,
	-1.279270,
	-1.306025,
	-1.328922,
	-1.347950,
	-1.363110,
	-1.374412,
	-1.381881,
	-1.385552,
	-1.385472,
	-1.381698,
	-1.374301,
	-1.363358,
	-1.348960,
	-1.331205,
	-1.310202,
	-1.286068,
	-1.258930,
	-1.228921,
	-1.196182,
	-1.160861,
	-1.123113,
	-1.083096,
	-1.040978,
	-0.996926,
	-0.951114,
	-0.903720,
	-0.854922,
	-0.804902,
	-0.753842,
	-0.701925,
	-0.649335,
	-0.596255,
	-0.542866,
	-0.489348,
	-0.435877,
	-0.382630,
	-0.329775,
	-0.277479,
	-0.225905,
	-0.175209,
	-0.125541,
	-0.077046,
	-0.029862,
	0.015880,
	0.060057,
	0.102553,
	0.143262,
	0.182083,
	0.218928,
	0.253715,
	0.286373,
	0.316837,
	0.345057,
	0.370987,
	0.394595,
	0.415856,
	0.434755,
	0.451288,
	0.465460,
	0.477284,
	0.486783,
	0.493990,
	0.498946,
	0.501701,
	0.502314,
	0.500849,
	0.497381,
	0.491991,
	0.484767,
	0.475804,
	0.465202,
	0.453067,
	0.439511,
	0.424649,
	0.408602,
	0.391493,
	0.373448,
	0.354598,
	0.335073,
	0.315006,
	0.294530,
	0.273779,
	0.252886,
	0.231984,
	0.211204,
	0.190675,
	0.170525,
	0.150876,
	0.131849,
	0.113560,
	0.096122,
	0.079640,
	0.064216,
	0.049946,
	0.036919,
	0.025217,
	0.014917,
	0.006088,
	-0.001211,
	-0.006925,
	-0.011009,
	-0.013427,
	-0.014149,
	-0.013156,
	-0.010437,
	-0.005988,
	0.000185,
	0.008067,
	0.017634,
	0.028855,
	0.041689,
	0.056088,
	0.071995,
	0.089345,
	0.108065,
	0.128075,
	0.149288,
	0.171610,
	0.194941,
	0.219174,
	0.244196,
	0.269890,
	0.296135,
	0.322802,
	0.349762,
	0.376880,
	0.404021,
	0.431045,
	0.457812,
	0.484180,
	0.510007,
	0.535150,
	0.559468,
	0.582820,
	0.605067,
	0.626072,
	0.645702,
	0.663827,
	0.680320,
	0.695059,
	0.707927,
	0.718812,
	0.727611,
	0.734223,
	0.738556,
	0.740527,
	0.740058,
	0.737082,
	0.731537,
	0.723372,
	0.712546,
	0.699025,
	0.682787,
	0.663818,
	0.642114,
	0.617683,
	0.590540,
	0.560714,
	0.528242,
	0.493170,
	0.455558,
	0.415472,
	0.372990,
	0.328200,
	0.281200,
	0.232094,
	0.181000,
	0.128041,
	0.073349,
	0.017067,
	-0.040658,
	-0.099671,
	-0.159808,
	-0.220902,
	-0.282777,
	-0.345255,
	-0.408150,
	-0.471274,
	-0.534437,
	-0.597442,
	-0.660092,
	-0.722190,
	-0.783536,
	-0.843929,
	-0.903170,
	-0.961062,
	-1.017406,
	-1.072009,
	-1.124680,
	-1.175230,
	-1.223477,
	-1.269241,
	-1.312352,
	-1.352641,
	-1.389949,
	-1.424125,
	-1.455023,
	-1.482509,
	-1.506455,
	-1.526745,
	-1.543270,
	-1.555935,
	-1.564653,
	-1.569349,
	-1.569960,
	-1.566434,
	-1.558732,
	-1.546826,
	-1.530701,
	-1.510355,
	-1.485799,
	-1.457055,
	-1.424160,
	-1.387162,
	-1.346122,
	-1.301114,
	-1.252226,
	-1.199554,
	-1.143211,
	-1.083319,
	-1.020011,
	-0.953433,
	-0.883741,
	-0.811101,
	-0.735689,
	-0.657691,
	-0.577301,
	-0.494723,
	-0.410166,
	-0.323850,
	-0.235999,
	-0.146842,
	-0.056617,
	0.034437,
	0.126076,
	0.218052,
	0.310115,
	0.402014,
	0.493497,
	0.584310,
	0.674201,
	0.762921,
	0.850220,
	0.935853,
	1.019578,
	1.101156,
	1.180356,
	1.256950,
	1.330717,
	1.401446,
	1.468929,
	1.532971,
	1.593383,
	1.649988,
	1.702616,
	1.751113,
	1.795330,
	1.835135,
	1.870405,
	1.901030,
	1.926915,
	1.947975,
	1.964141,
	1.975357,
	1.981581,
	1.982784,
	1.978952,
	1.970086,
	1.956200,
	1.937323,
	1.913498,
	1.884782,
	1.851246,
	1.812974,
	1.770064,
	1.722628,
	1.670790,
	1.614686,
	1.554466,
	1.490289,
	1.422328,
	1.350765,
	1.275792,
	1.197611,
	1.116434,
	1.032480,
	0.945977,
	0.857157,
	0.766264,
	0.673541,
	0.579242,
	0.483620,
	0.386936,
	0.289449,
	0.191424,
	0.093125,
	-0.005184,
	-0.103238,
	-0.200775,
	-0.297533,
	-0.393255,
	-0.487686,
	-0.580575,
	-0.671679,
	-0.760757,
	-0.847576,
	-0.931911,
	-1.013544,
	-1.092264,
	-1.167871,
	-1.240175,
	-1.308994,
	-1.374157,
	-1.435506,
	-1.492892,
	-1.546180,
	-1.595246,
	-1.639980,
	-1.680282,
	-1.716069,
	-1.747268,
	-1.773821,
	-1.795684,
	-1.812826,
	-1.825230,
	-1.832892,
	-1.835823,
	-1.834046,
	-1.827599,
	-1.816531,
	-1.800908,
	-1.780803,
	-1.756307,
	-1.727520,
	-1.694555,
	-1.657536,
	-1.616597,
	-1.571883,
	-1.523551,
	-1.471764,
	-1.416696,
	-1.358528,
	-1.297451,
	-1.233660,
	-1.167358,
	-1.098753,
	-1.028060,
	-0.955495,
	-0.881281,
	-0.805641,
	-0.728803,
	-0.650994,
	-0.572443,
	-0.493379,
	-0.414030,
	-0.334624,
	-0.255384,
	-0.176534,
	-0.098291,
	-0.020871,
	0.055518,
	0.130669,
	0.204386,
	0.276476,
	0.346754,
	0.415042,
	0.481172,
	0.544984,
	0.606324,
	0.665052,
	0.721035,
	0.774151,
	0.824288,
	0.871344,
	0.915230,
	0.955867,
	0.993187,
	1.027133,
	1.057661,
	1.084737,
	1.108339,
	1.128456,
	1.145090,
	1.158252,
	1.167966,
	1.174265,
	1.177194,
	1.176809,
	1.173174,
	1.166365,
	1.156466,
	1.143570,
	1.127779,
	1.109204,
	1.087962,
	1.064179,
	1.037986,
	1.009521,
	0.978928,
	0.946355,
	0.911956,
	0.875887,
	0.838309,
	0.799385,
	0.759279,
	0.718157,
	0.676188,
	0.633538,
	0.590375,
	0.546864,
	0.503169,
	0.459454,
	0.415878,
	0.372596,
	0.329761,
	0.287521,
	0.246019,
	0.205393,
	0.165773,
	0.127287,
	0.090052,
	0.054180,
	0.019777,
	-0.013063,
	-0.044250,
	-0.073703,
	-0.101352,
	-0.127134,
	-0.150994,
	-0.172886,
	-0.192777,
	-0.210637,
	-0.226451,
	-0.240209,
	-0.251913,
	-0.261572,
	-0.269204,
	-0.274838,
	-0.278510,
	-0.280264,
	-0.280152,
	-0.278235,
	-0.274582,
	-0.269267,
	-0.262373,
	-0.253988,
	-0.244207,
	-0.233130,
	-0.220863,
	-0.207516,
	-0.193204,
	-0.178044,
	-0.162158,
	-0.145670,
	-0.128706,
	-0.111393,
	-0.093862,
	-0.076239,
	-0.058656,
	-0.041240,
	-0.024119,
	-0.007419,
	0.008738,
	0.024231,
	0.038942,
	0.052758,
	0.065571,
	0.077275,
	0.087772,
	0.096968,
	0.104776,
	0.111116,
	0.115913,
	0.119100,
	0.120619,
	0.120417,
	0.118453,
	0.114689,
	0.109100,
	0.101668,
	0.092382,
	0.081243,
	0.068259,
	0.053447,
	0.036833,
	0.018452,
	-0.001653,
	-0.023429,
	-0.046815,
	-0.071742,
	-0.098134,
	-0.125904,
	-0.154962,
	-0.185206,
	-0.216530,
	-0.248821,
	-0.281960,
	-0.315821,
	-0.350275,
	-0.385186,
	-0.420414,
	-0.455817,
	-0.491247,
	-0.526557,
	-0.561593,
	-0.596204,
	-0.630235,
	-0.663531,
	-0.695938,
	-0.727303,
	-0.757473,
	-0.786297,
	-0.813629,
	-0.839322,
	-0.863235,
	-0.885233,
	-0.905182,
	-0.922956,
	-0.938434,
	-0.951502,
	-0.962051,
	-0.969982,
	-0.975202,
	-0.977627,
	-0.977182,
	-0.973799,
	-0.967421,
	-0.958001,
	-0.945500,
	-0.929892,
	-0.911159,
	-0.889294,
	-0.864302,
	-0.836197,
	-0.805005,
	-0.770763,
	-0.733517,
	-0.693327,
	-0.650261,
	-0.604399,
	-0.555831,
	-0.504658,
	-0.450990,
	-0.394947,
	-0.336660,
	-0.276266,
	-0.213915,
	-0.149761,
	-0.083968,
	-0.016708,
	0.051840,
	0.121493,
	0.192058,
	0.263341,
	0.335141,
	0.407252,
	0.479467,
	0.551574,
	0.623360,
	0.694610,
	0.765108,
	0.834639,
	0.902988,
	0.969939,
	1.035281,
	1.098805,
	1.160305,
	1.219578,
	1.276427,
	1.330659,
	1.382089,
	1.430538,
	1.475832,
	1.517809,
	1.556311,
	1.591191,
	1.622313,
	1.649549,
	1.672781,
	1.691904,
	1.706823,
	1.717455,
	1.723728,
	1.725585,
	1.722978,
	1.715875,
	1.704255,
	1.688110,
	1.667448,
	1.642286,
	1.612658,
	1.578610,
	1.540201,
	1.497502,
	1.450600,
	1.399592,
	1.344589,
	1.285714,
	1.223102,
	1.156898,
	1.087262,
	1.014361,
	0.938374,
	0.859490,
	0.777908,
	0.693834,
	0.607483,
	0.519078,
	0.428849,
	0.337032,
	0.243868,
	0.149604,
	0.054490,
	-0.041219,
	-0.137267,
	-0.233394,
	-0.329341,
	-0.424847,
	-0.519652,
	-0.613498,
	-0.706128,
	-0.797286,
	-0.886722,
	-0.974190,
	-1.059447,
	-1.142258,
	-1.222393,
	-1.299629,
	-1.373751,
	-1.444553,
	-1.511837,
	-1.575416,
	-1.635111,
	-1.690755,
	-1.742194,
	-1.789282,
	-1.831887,
	-1.869891,
	-1.903187,
	-1.931682,
	-1.955295,
	-1.973962,
	-1.987629,
	-1.996259,
	-1.999828,
	-1.998327,
	-1.991760,
	-1.980147,
	-1.963522,
	-1.941932,
	-1.915438,
	-1.884116,
	-1.848054,
	-1.807356,
	-1.762137,
	-1.712523,
	-1.658657,
	-1.600689,
	-1.538782,
	-1.473112,
	-1.403862,
	-1.331226,
	-1.255409,
	-1.176623,
	-1.095086,
	-1.011026,
	-0.924678,
	-0.836280,
	-0.746077,
	-0.654319,
	-0.561257,
	-0.467148,
	-0.372250,
	-0.276821,
	-0.181121,
	-0.085409,
	0.010056,
	0.105017,
	0.199221,
	0.292416,
	0.384354,
	0.474795,
	0.563501,
	0.650241,
	0.734791,
	0.816935,
	0.896464,
	0.973178,
	1.046886,
	1.117407,
	1.184570,
	1.248215,
	1.308192,
	1.364365,
	1.416606,
	1.464803,
	1.508854,
	1.548671,
	1.584178,
	1.615312,
	1.642024,
	1.664278,
	1.682050,
	1.695331,
	1.704125,
	1.708447,
	1.708328,
	1.703809,
	1.694946,
	1.681806,
	1.664468,
	1.643022,
	1.617573,
	1.588232,
	1.555123,
	1.518380,
	1.478148,
	1.434577,
	1.387830,
	1.338074,
	1.285487,
	1.230252,
	1.172556,
	1.112596,
	1.050571,
	0.986683,
	0.921141,
	0.854153,
	0.785932,
	0.716691,
	0.646644,
	0.576004,
	0.504986,
	0.433801,
	0.362658,
	0.291766,
	0.221328,
	0.151543,
	0.082608,
	0.014712,
	-0.051960,
	-0.117230,
	-0.180927,
	-0.242887,
	-0.302954,
	-0.360978,
	-0.416822,
	-0.470355,
	-0.521456,
	-0.570012,
	-0.615924,
	-0.659099,
	-0.699457,
	-0.736928,
	-0.771450,
	-0.802977,
	-0.831469,
	-0.856899,
	-0.879250,
	-0.898517,
	-0.914705,
	-0.927828,
	-0.937913,
	-0.944995,
	-0.949120,
	-0.950342,
	-0.948728,
	-0.944349,
	-0.937289,
	-0.927637,
	-0.915493,
	-0.900960,
	-0.884153,
	-0.865189,
	-0.844194,
	-0.821297,
	-0.796633,
	-0.770342,
	-0.742567,
	-0.713454,
	-0.683151,
	-0.651809,
	-0.619579,
	-0.586616,
	-0.553071,
	-0.519098,
	-0.484848,
	-0.450470,
	-0.416114,
	-0.381923,
	-0.348041,
	-0.314606,
	-0.281751,
	-0.249606,
	-0.218295,
	-0.187937,
	-0.158644,
	-0.130522,
	-0.103670,
	-0.078180,
	-0.054136,
	-0.031615,
	-0.010686,
	0.008591,
	0.026164,
	0.041990,
	0.056034,
	0.068270,
	0.078681,
	0.087260,
	0.094006,
	0.098930,
	0.102050,
	0.103392,
	0.102992,
	0.100893,
	0.097147,
	0.091813,
	0.084958,
	0.076655,
	0.066987,
	0.056039,
	0.043906,
	0.030686,
	0.016484,
	0.001410,
	-0.014424,
	-0.030900,
	-0.047897,
	-0.065292,
	-0.082958,
	-0.100769,
	-0.118596,
	-0.136309,
	-0.153778,
	-0.170875,
	-0.187471,
	-0.203441,
	-0.218659,
	-0.233004,
	-0.246358,
	-0.258605,
	-0.269635,
	-0.279342,
	-0.287626,
	-0.294390,
	-0.299547,
	-0.303013,
	-0.304712,
	-0.304577,
	-0.302547,
	-0.298568,
	-0.292595,
	-0.284592,
	-0.274531,
	-0.262393,
	-0.248168,
	-0.231856,
	-0.213463,
	-0.193008,
	-0.170517,
	-0.146027,
	-0.119582,
	-0.091236,
	-0.061053,
	-0.029104,
	0.004531,
	0.039761,
	0.076491,
	0.114615,
	0.154021,
	0.194590,
	0.236195,
	0.278704,
	0.321978,
	0.365874,
	0.410241,
	0.454928,
	0.499775,
	0.544624,
	0.589308,
	0.633664,
	0.677523,
	0.720717,
	0.763075,
	0.804430,
	0.844613,
	0.883456,
	0.920795,
	0.956468,
	0.990315,
	1.022181,
	1.051915,
	1.079371,
	1.104409,
	1.126893,
	1.146698,
	1.163701,
	1.177791,
	1.188862,
	1.196819,
	1.201575,
	1.203052,
	1.201182,
	1.195907,
	1.187181,
	1.174966,
	1.159237,
	1.139980,
	1.117191,
	1.090879,
	1.061062,
	1.027773,
	0.991054,
	0.950959,
	0.907555,
	0.860919,
	0.811138,
	0.758313,
	0.702553,
	0.643980,
	0.582725,
	0.518928,
	0.452741,
	0.384322,
	0.313842

};